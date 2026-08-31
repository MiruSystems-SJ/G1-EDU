#!/usr/bin/env python3
"""예제 3 — 동작 시퀀스 템플릿 (교재 7장 과제 2 '동작 시퀀스 설계').

7.3 설계 시트의 표를 그대로 코드로 옮기는 틀이다.
SEQUENCE 리스트를 채워 자신만의 시퀀스를 만들어 보자.

행 형식: (동작, 인자, 다음 단계 전 대기 시간[s])
  ("standup",  None,          4.0)   # 기립
  ("action",   "wave",        1.0)   # 상체 동작: wave / hands_up / bow
  ("move",     (0.1, 0, 0),   5.0)   # 보행 시작 (vx, vy, vyaw) — 5초 동안 걷기
  ("stop",     None,          2.0)   # 정지(자동으로 balance_stand 복귀)
  ("damp",     None,          1.0)   # 마무리

개선 힌트(도전): 고정 대기 대신 client.WaitMode("balance_stand") 같은
'상태 기반 대기'로 바꾸면 어떤 점이 좋아질까? (교재 5.4 도전 미션)
"""
import argparse
import time

import _common  # noqa: F401
from g1edu import G1Sim, LocoClient

# ── 여기를 채우세요 ──────────────────────────────────────────────
# 설계 시트(교재 7.3)에 적은 표를 한 줄씩 옮깁니다.
SEQUENCE = [
    ("standup", None,        4.0),
    ("action",  "wave",      1.0),
    # TODO: 아래에 자신의 시퀀스를 추가
    # ("move",  (0.1, 0, 0), 5.0),
    # ("stop",  None,        2.0),
    ("damp",    None,        1.0),
]
# ────────────────────────────────────────────────────────────────


def run_step(client: LocoClient, kind: str, arg):
    if kind == "standup":
        client.StandUp()
    elif kind == "action":
        client.PlayAction(arg)
        while client.ActionActive():
            time.sleep(0.05)
    elif kind == "move":
        client.Move(*arg)
    elif kind == "stop":
        client.StopMove()
    elif kind == "damp":
        client.Damp()
    else:
        raise ValueError(f"알 수 없는 동작: {kind}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-hanger", action="store_true")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args()

    hanger = not args.no_hanger
    sim = G1Sim(hanger=hanger, start_standing=not hanger)
    sim.start(viewer=not args.no_viewer, realtime=not args.fast)
    client = LocoClient(sim)
    z = 0.06 if args.fast else 1.0

    try:
        client.Damp()
        time.sleep(1.0 * z)
        for i, (kind, arg, wait) in enumerate(SEQUENCE, 1):
            print(f"[{i}/{len(SEQUENCE)}] {kind} {arg if arg is not None else ''}"
                  f"  (모드: {client.GetMode()})")
            run_step(client, kind, arg)
            time.sleep(wait * z)
            if sim.fallen:
                print("낙상 발생 →", client.GetLastError())
                break
    finally:
        client.Damp()
        time.sleep(0.5 * z)
        print("종료 모드:", client.GetMode())
        sim.stop()


if __name__ == "__main__":
    main()
