#!/usr/bin/env python3
"""예제 4 — 걸으면서 손 흔들기 (교재 7장 도전 과제 · 시뮬레이션 한정).

⚠ 이 도전은 시뮬레이션에서만 시도한다. 실기체에서 하지 않는다(교재 7장).

보행 중 상체 모션은 기본적으로 '거부'된다(안전 게이트).
allow_walk_arm=True 로 게이트를 열면 실행되며, 보행 중에는 동작 진폭이
자동으로 줄어든다(균형 여유 확보). 그래도 속도·파라미터에 따라 넘어질 수 있다 —
어떤 조건에서 성공하는지 찾아보는 것이 도전의 내용이다.
"""
import argparse
import time

import _common  # noqa: F401
from g1edu import G1Sim, LocoClient


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args()

    sim = G1Sim(start_standing=True, allow_walk_arm=True)   # ← 게이트 개방
    sim.start(viewer=not args.no_viewer, realtime=not args.fast)
    client = LocoClient(sim)
    z = 0.06 if args.fast else 1.0

    try:
        time.sleep(1.0 * z)
        client.Move(args.vx, 0, 0)
        print(f"보행 시작 (vx={args.vx})")
        time.sleep(3.0 * z)

        print("걸으면서 손 흔들기…")
        client.WaveHand()
        while client.ActionActive() and not sim.fallen:
            time.sleep(0.05)

        time.sleep(3.0 * z)
        if sim.fallen:
            print("낙상 —", client.GetLastError())
            print("→ vx를 낮추거나 파라미터를 조정해 다시 시도해 보세요.")
        else:
            print("성공: 보행을 유지한 채 동작을 마쳤습니다.")
            client.StopMove()
            time.sleep(1.5 * z)
    finally:
        client.Damp()
        time.sleep(0.5 * z)
        sim.stop()


if __name__ == "__main__":
    main()
