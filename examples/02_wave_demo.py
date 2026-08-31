#!/usr/bin/env python3
"""예제 2 — 기립 → 손 흔들기 → 복귀 (교재 5장 미션 B의 기준 예제).

구조 포인트(교재 5.3):
  * 전이 사이에는 반드시 대기가 필요하다(로봇이 '자세를 잡을' 시간).
  * 어떤 예외가 나도 finally 에서 Damp — 안전한 종료가 항상 마지막이다.

실행:
    python3 examples/02_wave_demo.py               # 행어(거치대) + 뷰어
    python3 examples/02_wave_demo.py --no-hanger   # 바닥에서 시작
    python3 examples/02_wave_demo.py --action bow  # 다른 동작(hands_up / bow)
"""
import argparse
import time

import _common  # noqa: F401
from g1edu import G1Sim, LocoClient


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-hanger", action="store_true",
                    help="행어(거치대) 없이 바닥 기립 자세로 시작")
    ap.add_argument("--action", default="wave", choices=["wave", "hands_up", "bow"])
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args()

    hanger = not args.no_hanger
    sim = G1Sim(hanger=hanger, start_standing=not hanger)
    sim.start(viewer=not args.no_viewer, realtime=not args.fast)
    client = LocoClient(sim)
    z = lambda: 0.06 if args.fast else 1.0   # noqa: E731 — 대기 배율

    try:
        print("모드:", client.GetMode())
        if hanger:
            client.Damp()                     # 절차는 언제나 damp에서 시작
            time.sleep(1.0 * z())
            print("기립 시작…")
            client.StandUp()
            time.sleep(4.0 * z())             # 기립 3초 + 여유  ← 고정 대기
            print("모드:", client.GetMode())  # balance_stand 확인 (교재 3.2 확인 루틴)
        else:
            print("바닥 기립 상태에서 시작 — 바로 동작으로 넘어갑니다.")
            time.sleep(1.0 * z())

        print(f"동작 재생: {args.action}")
        client.PlayAction(args.action)
        while client.ActionActive():          # 동작이 끝날 때까지
            time.sleep(0.1 * z())
        time.sleep(1.0 * z())

        print("복귀(Damp)…")
    except Exception as e:                    # 무슨 일이 있어도 →
        print("예외 발생:", e)
    finally:
        client.Damp()                         # ← 안전한 마무리 (교재 5.3)
        time.sleep(1.0 * z())
        print("종료 모드:", client.GetMode(), "| 에러:", client.GetLastError() or "없음")
        sim.stop()


if __name__ == "__main__":
    main()
