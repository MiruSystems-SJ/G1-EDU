#!/usr/bin/env python3
"""예제 1 — 보행 데모 (교재 4장).

config/gait_params.yaml 의 파라미터로 G1을 걷게 한다.
파라미터를 '한 번에 하나씩' 바꿔 가며 걸음이 어떻게 달라지는지 관찰한다(교재 4.3).

실행:
    python3 examples/01_walk_demo.py                      # 뷰어 + 실시간
    python3 examples/01_walk_demo.py --duration 60        # 60초 후 자동 종료
    python3 examples/01_walk_demo.py --no-viewer --fast   # 헤드리스 최고속(검증용)
    python3 examples/01_walk_demo.py --ros                # ROS 2 토픽 발행 켜기

뷰어 조작: 마우스 드래그=회전, 스크롤=줌,
          몸통 더블클릭 후 Ctrl+오른쪽 드래그 = 외란(밀기) 실험 (교재 4.4)
"""
import argparse
import time

import _common  # noqa: F401
from g1edu import G1Sim, LocoClient
from g1edu.gait import GaitParams


def load_params(path: str) -> GaitParams:
    import yaml
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    return GaitParams.from_dict(d)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=_common.ROOT + "/config/gait_params.yaml")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="시뮬레이션 시간 [s], 0이면 계속(창 닫기/Ctrl+C로 종료)")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--fast", action="store_true", help="실시간 페이스 없이 최고 속도로")
    ap.add_argument("--ros", action="store_true", help="ROS 2 토픽 발행(빌드 필요)")
    args = ap.parse_args()

    p = load_params(args.params)
    print("─" * 56)
    print("보행 파라미터 (관찰표에 기록해 두세요 — 교재 4.4)")
    for k in ("vx", "vy", "vyaw", "step_period", "step_height",
              "com_shift", "pelvis_height", "arm_swing"):
        print(f"  {k:14s} = {getattr(p, k)}")
    print("─" * 56)

    sim = G1Sim(params=p, start_standing=True)
    if args.ros:
        from g1edu.ros_bridge import RosBridge
        RosBridge().attach(sim)

    sim.start(viewer=not args.no_viewer, realtime=not args.fast)
    client = LocoClient(sim)

    time.sleep(0.5 if args.fast else 2.0)      # 기립 자세 안정 대기
    x0 = sim.m.pelvis_pos()[0]
    t0 = sim.sim_time()
    client.Move(p.vx, p.vy, p.vyaw)
    print("보행 시작 — 모드:", client.GetMode())

    try:
        while True:
            time.sleep(0.1)
            if sim.fallen:
                print("\n[낙상] GetLastError():", client.GetLastError())
                print("→ 방금 바꾼 파라미터와 값, 넘어진 방향을 관찰표에 기록하세요"
                      " (교재 4.3 체크포인트 ③).")
                break
            if args.duration and sim.sim_time() - t0 >= args.duration:
                break
            if not getattr(sim, "_thread").is_alive():   # 뷰어 창을 닫은 경우
                break
    except KeyboardInterrupt:
        pass
    finally:
        dt = sim.sim_time() - t0
        dx = sim.m.pelvis_pos()[0] - x0
        if dt > 1.0:
            print(f"\n보행 {dt:.1f} s, 전진 {dx:+.2f} m → 실측 평균속도 {dx/dt:+.3f} m/s")
            print(f"명령 속도 vx={p.vx} 와 비교해 보세요 — 왜 다를까요? (README '관찰 노트')")
        client.Damp()
        sim.stop()


if __name__ == "__main__":
    main()
