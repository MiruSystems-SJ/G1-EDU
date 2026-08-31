#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mission7_judge.py — 과제 1 '판정 실행' 도구 (교재4 7.2 · 시뮬레이션).

무엇을 하나 : 아래 SETS 의 각 세트를 '같은 조건'(같은 시작 자세·같은 실행 시간)으로
              N회 반복 실행하고, 완주/넘어짐을 자동 판정해 비교표(md)까지 만들어 준다.
              — 교재 7.2 절차 ③(판정 실행)·④(비교표 작성)의 자동화판.
어디서       : g1-edu 저장소 루트(~/robot_ws/src/g1-edu)에 두고 그 위치에서 실행.
              (param_sweep.py 와 같은 위치·같은 방식 — 스윕은 '경계 찾기'용,
               judge 는 '확정 세트 판정'용이다.)
무엇이 아닌가: 근거는 자동화되지 않는다 — 비교표의 '근거(관찰표 행)' 열과
              '한 줄 해석'은 수요일 관찰표를 보고 여러분이 채운다(교재 7.2).

사용 예:
  python3 mission7_judge.py --dry-run          # 계획 검토(시뮬 불필요)
  python3 mission7_judge.py                    # 두 세트 × 3회 헤드리스 판정
  python3 mission7_judge.py --only fast --view # 속달 세트만 뷰어로 1회씩 확인
  python3 mission7_judge.py --repeats 5 --dur 40
"""
import argparse
import csv
import math
import os
import sys
import time
import datetime as _dt

# ══════════════════════════════════════════════════════════════════════
# [학생 편집 구역] — 수요일 관찰표를 근거로 두 세트를 확정하는 곳 (교재 7.2 ②)
#   vx/vy/vyaw : Move() 명령 값.  gait : config/gait_params.yaml 의 파라미터 이름 그대로
#   (기본값 참고: vx=0.10, step_period=0.45, step_height=0.04, com_shift=0.045 /
#    낙상 경계 예: vx≈0.5, vyaw≈0.5 — 여러분 관찰표의 경계를 근거로 그 '안쪽'을 고른다)
SETS = {
    "stable": dict(
        label="① 안정 세트",
        vx=0.10, vy=0.0, vyaw=0.0,
        gait=dict(),                     # 기본값 유지 — 바꿨다면 근거 행 번호를 메모
        근거="관찰표   행 (기입)",
    ),
    "fast": dict(
        label="② 속달 세트",
        vx=0.35, vy=0.0, vyaw=0.0,
        gait=dict(step_period=0.40),     # 예시 — 반드시 자기 관찰표 값으로 교체
        근거="관찰표   행 (기입)",
    ),
}
REPEATS = 3          # 통과 기준(7.1): 예 — 3회 중 3회 완주
DURATION = 30.0      # 시도당 시뮬레이션 시간[s] — 조에서 합의한 숫자로
# ══════════════════════════════════════════════════════════════════════


def quat_to_rp(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return roll, math.asin(s)


def get_base_state(sim):
    """베이스 위치(x,y)·기울기(roll,pitch) — MuJoCo qpos 접근(버전 방어)."""
    data = getattr(getattr(sim, "m", None), "data", None) or getattr(sim, "data", None)
    if data is None:
        return None
    try:
        q = data.qpos
        roll, pitch = quat_to_rp(float(q[3]), float(q[4]), float(q[5]), float(q[6]))
        return float(q[0]), float(q[1]), roll, pitch
    except Exception:
        return None


def make_sim(G1Sim, GaitParams, gait_overrides):
    """gait 파라미터 적용 — param_sweep.py 와 같은 경로를 순서대로 시도한다."""
    if gait_overrides and GaitParams is not None:
        try:
            return G1Sim(start_standing=True, gait=GaitParams(**gait_overrides))
        except TypeError:
            pass
    sim = G1Sim(start_standing=True)
    if gait_overrides:
        target = getattr(sim, "gait", None) or getattr(sim, "params", None)
        applied = False
        if target is not None:
            for k, v in gait_overrides.items():
                if hasattr(target, k):
                    setattr(target, k, v)
                    applied = True
        if not applied:
            sim_stop(sim)
            raise SystemExit(
                "gait 파라미터 적용 경로를 찾지 못했습니다 — 이 저장소 버전의 적용 방식은 "
                "param_sweep.py 에 있으니 make_sim() 을 그 방식에 맞춰 한 곳만 수정하세요.")
    return sim


def sim_start(sim, view):
    try:
        sim.start(viewer=view, realtime=view)
    except TypeError:
        sim.start(realtime=view)


def sim_stop(sim):
    try:
        sim.stop()
    except Exception:
        pass


def run_trial(G1Sim, LocoClient, GaitParams, spec, dur, view):
    """한 번의 판정 실행 — 같은 시작 자세(start_standing) → Move → 완주/낙상 판정."""
    sim = make_sim(G1Sim, GaitParams, spec.get("gait") or {})
    sim_start(sim, view)
    client = LocoClient(sim)
    out = dict(outcome="완주", fall_t=None, mean_speed=None, max_tilt_deg=0.0, err="")
    try:
        client.Move(spec.get("vx", 0.0), spec.get("vy", 0.0), spec.get("vyaw", 0.0))
        s0 = get_base_state(sim)
        t_cmd0 = sim.sim_time()
        while sim.sim_time() - t_cmd0 < dur:
            if getattr(sim, "fallen", False):
                out["outcome"] = "넘어짐"
                out["fall_t"] = round(sim.sim_time() - t_cmd0, 2)
                try:
                    out["err"] = client.GetLastError() or ""
                except Exception:
                    pass
                break
            s = get_base_state(sim)
            if s is not None:
                tilt = math.degrees(math.hypot(s[2], s[3]))
                out["max_tilt_deg"] = max(out["max_tilt_deg"], round(tilt, 1))
            time.sleep(0.005)
        s1 = get_base_state(sim)
        elapsed = sim.sim_time() - t_cmd0
        if s0 and s1 and elapsed > 1.0:
            dist = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
            out["mean_speed"] = round(dist / elapsed, 3)   # 실측 — 명령값과 1:1 이 아니다
        try:
            client.StopMove()
        except Exception:
            pass
    finally:
        try:
            client.Damp()
        except Exception:
            pass
        sim_stop(sim)
    return out


def fmt(v):
    return "-" if v is None else v


def main():
    ap = argparse.ArgumentParser(description="과제 1 판정 실행 (교재 7.2)")
    ap.add_argument("--only", choices=list(SETS), help="한 세트만 실행")
    ap.add_argument("--repeats", type=int, default=REPEATS)
    ap.add_argument("--dur", type=float, default=DURATION)
    ap.add_argument("--view", action="store_true", help="뷰어 표시(느림 — 판정은 헤드리스 권장)")
    ap.add_argument("--dry-run", action="store_true", help="계획만 출력(시뮬 불필요)")
    args = ap.parse_args()

    names = [args.only] if args.only else list(SETS)
    print("\n판정 계획 — 통과 기준을 '숫자'로 먼저 합의했는가? (교재 7.1)")
    for nm in names:
        s = SETS[nm]
        g = ", ".join(f"{k}={v}" for k, v in (s.get("gait") or {}).items()) or "기본값"
        print(f"  {s['label']:<10s} vx={s['vx']} vy={s['vy']} vyaw={s['vyaw']} | "
              f"gait: {g} | {args.repeats}회 × {args.dur:.0f}s | 근거: {s.get('근거','')}")
    if args.dry_run:
        print("\n--dry-run: 여기까지. 같은 조건 반복(변인 통제)이 판정의 전부입니다 — 7.2 ③.")
        return

    # g1edu 임포트 — 저장소 루트 실행 전제 (param_sweep.py 와 동일)
    sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
    sys.path.insert(0, os.getcwd())
    try:
        from g1edu import G1Sim, LocoClient
    except ImportError:
        sys.exit("g1edu 를 찾을 수 없습니다 — 이 파일을 저장소 루트(~/robot_ws/src/g1-edu)에 "
                 "두고 그 위치에서 실행하세요.")
    try:
        from g1edu.gait import GaitParams
    except ImportError:
        GaitParams = None

    ts = _dt.datetime.now().strftime("%m%d_%H%M%S")
    rows = []
    summary = {}
    for nm in names:
        spec = SETS[nm]
        print(f"\n── {spec['label']} ──")
        oks = 0
        speeds, tilts, fall_ts = [], [], []
        for r in range(1, args.repeats + 1):
            res = run_trial(G1Sim, LocoClient, GaitParams, spec, args.dur, args.view)
            ok = res["outcome"] == "완주"
            oks += ok
            if res["mean_speed"] is not None:
                speeds.append(res["mean_speed"])
            tilts.append(res["max_tilt_deg"])
            if res["fall_t"] is not None:
                fall_ts.append(res["fall_t"])
            print(f"  {r}회차: {res['outcome']}"
                  + (f" (t={res['fall_t']}s, {res['err']})" if not ok else "")
                  + f" | 실측 평균속도 {fmt(res['mean_speed'])} m/s"
                  + f" | 최대 기울기 {res['max_tilt_deg']}°")
            rows.append([spec["label"], r, res["outcome"], fmt(res["fall_t"]),
                         fmt(res["mean_speed"]), res["max_tilt_deg"], res["err"]])
        summary[nm] = dict(
            ok=f"{oks}/{args.repeats}",
            speed=(round(sum(speeds) / len(speeds), 3) if speeds else None),
            tilt=(max(tilts) if tilts else None),
            fall=(f"평균 {sum(fall_ts)/len(fall_ts):.1f}s" if fall_ts else "-"))
        verdict = "통과" if oks == args.repeats else "미통과(기준: 전 회 완주 시)"
        print(f"  ⇒ {spec['label']}: {oks}/{args.repeats} 완주 — {verdict}")

    # 결과 CSV + 비교표(md) — 근거·해석 칸은 비워 둔다(사람 몫)
    csv_path = f"judge_results_{ts}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["세트", "회차", "결과", "낙상 t[s]", "실측 평균속도[m/s]",
                    "최대 기울기[deg]", "에러"])
        w.writerows(rows)

    md_path = f"과제1_비교표_{ts}.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(f"# 과제 1 파라미터 비교표 (판정 자동 기록 {ts})\n\n")
        f.write("| 항목 | " + " | ".join(SETS[n]["label"] for n in names)
                + " | 근거 (관찰표 행) |\n")
        f.write("|---" * (len(names) + 2) + "|\n")
        keys = ["vx", "vy", "vyaw"]
        gait_keys = sorted({k for n in names for k in (SETS[n].get("gait") or {})})
        for k in keys + gait_keys:
            vals = [str(SETS[n].get(k, (SETS[n].get("gait") or {}).get(k, "-")))
                    for n in names]
            f.write(f"| {k} | " + " | ".join(vals) + " |  |\n")
        f.write("| 반복 실행 결과 | "
                + " | ".join(f"{summary[n]['ok']} 완주" for n in names)
                + " | 영상 타임스탬프: |\n")
        f.write("| 실측 평균 속도 | "
                + " | ".join(f"{fmt(summary[n]['speed'])} m/s" for n in names)
                + " | (자동 측정 — 명령값과 다름을 관찰) |\n")
        f.write("| 최대 기울기 | "
                + " | ".join(f"{fmt(summary[n]['tilt'])}°" for n in names)
                + " |  |\n")
        f.write("| 낙상 시각 | "
                + " | ".join(summary[n]["fall"] for n in names) + " |  |\n")
        f.write("| 한 줄 해석 | (속도를 얻는 대가로 무엇을 지불했는가 — "
                "지지 영역·스텝 타이밍의 언어로, 교재 2.2) |  |  |\n")
    print(f"\n결과 저장: {csv_path} / {md_path}"
          "\n비교표의 '근거' 열과 '한 줄 해석'을 채워 조 repo 에 커밋하면 제출물 ① 완성입니다.")


if __name__ == "__main__":
    main()
