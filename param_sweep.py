#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""교재 ④ 4장 — 보행 파라미터 경계 스윕 (헤드리스, 멘토·검증용).

교재 4.3 체크포인트 ③("무엇이 언제 넘어지게 만드는가")의 경계를 뷰어 없이
빠르게 훑습니다. 학생 실습의 본론은 뷰어 관찰이므로, 이 스크립트는
관찰표(4.4)의 '넘어짐 여부' 경계를 미리 확인해 두는 보조 도구입니다.

사용법 — 반드시 저장소 루트에서:
    cd ~/robot_ws/src/g1-edu
    python3 param_sweep.py                      # 아래 TRIALS 기본 스윕
    python3 param_sweep.py --dur 40             # 시도당 40초로
    python3 param_sweep.py --set step_period=0.9            # 단일 시도
    python3 param_sweep.py --set step_height=0.06 --vx 0.25 # 조합도 가능
                                                 # (교재 원칙은 '한 번에 하나')

원칙(교재 4.3): 한 번에 하나만 바꾸고, 반드시 기본값 대비로 기록합니다.
파라미터 이름·기본값·주석은 config/gait_params.yaml 참조.

역할 구분: 이 스크립트는 '경계 찾기'용이고, 금요일 과제 1의 '확정 세트
반복 판정'은 mission7_judge.py 가 맡습니다(같은 위치·같은 적용 방식).
"""
import argparse
import math
import os
import sys
import time

# 저장소 루트 실행 전제 — g1edu 패키지 탐색 경로 추가
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
sys.path.insert(0, os.getcwd())

try:
    from g1edu import G1Sim, LocoClient
except ImportError:
    sys.exit("g1edu 를 찾을 수 없습니다 — 저장소 루트(~/robot_ws/src/g1-edu)에 "
             "이 파일을 두고 그 위치에서 실행하세요.")
try:
    from g1edu.gait import GaitParams
except ImportError:
    GaitParams = None

# 기본 스윕 목록 — 기본값(vx=0.10, step_period=0.45, step_height=0.04,
# com_shift=0.045)에서 '한 번에 하나'씩 벗어나는 시도들.
# 검증된 참고 경계: vx≈0.5 낙상 / vyaw 0.25 완주·0.5 낙상 (경계는 우리
# 저장소 버전 기준이므로, 값이 다르게 나오면 그 관측이 곧 새 데이터입니다).
TRIALS = [
    dict(label="기본값 확인",              vx=0.10),
    dict(label="고속 경계 안쪽",            vx=0.45),
    dict(label="고속 경계(낙상 예상)",       vx=0.50),
    dict(label="짧은 주기",                vx=0.10, set={"step_period": 0.30}),
    dict(label="긴 주기",                  vx=0.10, set={"step_period": 0.90}),
    dict(label="낮은 스윙 높이",            vx=0.10, set={"step_height": 0.02}),
    dict(label="높은 스윙 높이",            vx=0.10, set={"step_height": 0.07}),
    dict(label="무게중심 이동 작게",         vx=0.10, set={"com_shift": 0.02}),
    dict(label="무게중심 이동 크게",         vx=0.10, set={"com_shift": 0.08}),
    dict(label="횡이동",                   vy=0.08),
    dict(label="제자리 회전 안정",           vyaw=0.25),
    dict(label="제자리 회전 경계(낙상 예상)", vyaw=0.50),
]


# ── gait 적용·시뮬 기동 헬퍼 — mission7_judge.py 와 동일한 경로 ─────────
def make_sim(gait_overrides):
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
                "gait 파라미터 적용 경로를 찾지 못했습니다 — 이 저장소 버전의 "
                "생성자/속성 이름에 맞춰 make_sim() 한 곳만 수정하세요 "
                "(mission7_judge.py 의 make_sim 과 함께).")
    return sim


def sim_start(sim, view=False):
    try:
        sim.start(viewer=view, realtime=view)
    except TypeError:
        sim.start(realtime=view)


def sim_stop(sim):
    try:
        sim.stop()
    except Exception:
        pass


def get_base_xy(sim):
    data = getattr(getattr(sim, "m", None), "data", None) or getattr(sim, "data", None)
    if data is None:
        return None
    try:
        return float(data.qpos[0]), float(data.qpos[1])
    except Exception:
        return None


# ── 한 번의 시도 ──────────────────────────────────────────────────────
def run_trial(spec, dur, view):
    sim = make_sim(spec.get("set") or {})
    sim_start(sim, view)
    client = LocoClient(sim)
    outcome, fall_t, err, speed = "완주", None, "", None
    try:
        client.Move(spec.get("vx", 0.0), spec.get("vy", 0.0), spec.get("vyaw", 0.0))
        p0 = get_base_xy(sim)
        t0 = sim.sim_time()
        while sim.sim_time() - t0 < dur:
            if getattr(sim, "fallen", False):
                outcome = "낙상"
                fall_t = round(sim.sim_time() - t0, 2)
                try:
                    err = client.GetLastError() or ""
                except Exception:
                    pass
                break
            time.sleep(0.005)
        if outcome == "완주" and getattr(sim, "fallen", False):
            outcome = "낙상"
            fall_t = round(sim.sim_time() - t0, 2)
            try:
                err = client.GetLastError() or ""
            except Exception:
                pass
        p1 = get_base_xy(sim)
        el = sim.sim_time() - t0
        if p0 and p1 and el > 1.0:
            speed = round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / el, 3)
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
    return outcome, fall_t, err, speed


def parse_set(items):
    out = {}
    for it in items or []:
        if "=" not in it:
            sys.exit(f"--set 형식 오류: '{it}' (예: --set step_period=0.9)")
        k, v = it.split("=", 1)
        out[k.strip()] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description="보행 파라미터 경계 스윕 (교재 4.3)")
    ap.add_argument("--dur", type=float, default=30.0, help="시도당 시뮬 시간[s]")
    ap.add_argument("--set", action="append", metavar="key=val",
                    help="gait 파라미터 단일 시도 (반복 지정 가능하나 원칙은 한 번에 하나)")
    ap.add_argument("--vx", type=float, default=None)
    ap.add_argument("--vy", type=float, default=None)
    ap.add_argument("--vyaw", type=float, default=None)
    ap.add_argument("--view", action="store_true", help="뷰어 표시(느림)")
    args = ap.parse_args()

    if args.set or args.vx is not None or args.vy is not None or args.vyaw is not None:
        trials = [dict(label="단일 시도",
                       vx=args.vx if args.vx is not None else 0.10,
                       vy=args.vy or 0.0, vyaw=args.vyaw or 0.0,
                       set=parse_set(args.set))]
    else:
        trials = TRIALS

    print(f"\n경계 스윕 — {len(trials)}개 시도 × {args.dur:.0f}s "
          "(기본값 대비 '한 번에 하나', 교재 4.3)")
    rows = []
    for i, spec in enumerate(trials, 1):
        g = ", ".join(f"{k}={v}" for k, v in (spec.get("set") or {}).items())
        cmd = f"vx={spec.get('vx', 0.0)} vy={spec.get('vy', 0.0)} vyaw={spec.get('vyaw', 0.0)}"
        print(f"\n[{i}/{len(trials)}] {spec['label']}  ({cmd}"
              + (f" | {g}" if g else "") + ")")
        outcome, fall_t, err, speed = run_trial(spec, args.dur, args.view)
        line = (f"    → {outcome}"
                + (f" (t={fall_t}s, {err})" if outcome == "낙상" else "")
                + f" | 실측 평균속도 {speed if speed is not None else '-'} m/s")
        print(line)
        rows.append((spec["label"], cmd, g or "기본값", outcome, fall_t, speed))

    print("\n요약 — 관찰표(4.4) '넘어짐 여부' 열의 사전 확인 자료:")
    print("  ┌────────────────────────────┬──────────┬──────────┬──────────┐")
    print("  │ 시도                       │ 결과     │ 낙상 t   │ 실측속도 │")
    print("  ├────────────────────────────┼──────────┼──────────┼──────────┤")
    for label, cmd, g, outcome, fall_t, speed in rows:
        print(f"  │ {label:<26s} │ {outcome:<8s} │ "
              f"{(str(fall_t) + 's') if fall_t is not None else '-':>8s} │ "
              f"{(str(speed)) if speed is not None else '-':>8s} │")
    print("  └────────────────────────────┴──────────┴──────────┴──────────┘")
    print("\n실측 속도가 명령값보다 작게 나오는 것은 정상입니다(README 기재) — "
          "관찰 미션 소재로 쓰세요.")


if __name__ == "__main__":
    main()
