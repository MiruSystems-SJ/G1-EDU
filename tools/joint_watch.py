#!/usr/bin/env python3
"""관심 관절 모니터 (교재 6장 도전 미션의 기준 구현).

/g1/lowstate 를 구독해 선택한 관절의 q / dq / tau 만 골라 출력한다.
교재 ② 5장의 구독 노드 뼈대를 G1 low state에 적용한 형태다.

사용:
    # 터미널 1: 시뮬레이터 (ROS 발행 켜서)
    python3 examples/01_walk_demo.py --ros
    # 터미널 2:
    python3 tools/joint_watch.py --joints right_shoulder_roll_joint,right_elbow_joint
    python3 tools/joint_watch.py --joints 15,18,22        # 인덱스로도 가능
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from g1edu.joints import JOINT_INDEX, JOINT_NAMES  # noqa: E402


def parse_joints(spec: str) -> list[int]:
    out = []
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if tok.isdigit():
            out.append(int(tok))
        elif tok in JOINT_INDEX:
            out.append(JOINT_INDEX[tok])
        else:
            sys.exit(f"알 수 없는 관절: {tok}\n"
                     f"이름 목록은 docs/joint_map.md 를 참고하세요.")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--joints", default="15,22",
                    help="쉼표 구분 관절 이름 또는 인덱스 (docs/joint_map.md)")
    ap.add_argument("--rate", type=float, default=5.0, help="출력 주기 [Hz]")
    args = ap.parse_args()
    idxs = parse_joints(args.joints)

    try:
        import rclpy
        from rclpy.node import Node
        from g1_edu_interfaces.msg import LowState
    except ImportError as e:
        sys.exit(f"ROS 2 환경이 필요합니다({e}).\n"
                 "source /opt/ros/humble/setup.bash 와 워크스페이스 "
                 "install/setup.bash 를 확인하세요 (교재 3.4).")

    class Watch(Node):
        def __init__(self):
            super().__init__("joint_watch")
            self._last = self.get_clock().now()
            self._period = 1.0 / args.rate
            self.create_subscription(LowState, "/g1/lowstate", self.cb, 10)
            hdr = " | ".join(f"{JOINT_NAMES[i]}" for i in idxs)
            print(f"tick | {hdr}   (q[rad] / dq[rad/s] / tau[Nm])")

        def cb(self, msg: LowState):
            now = self.get_clock().now()
            if (now - self._last).nanoseconds < self._period * 1e9:
                return
            self._last = now
            cols = []
            for i in idxs:
                m = msg.motor_state[i]
                cols.append(f"{m.q:+7.3f} / {m.dq:+7.3f} / {m.tau_est:+7.2f}")
            print(f"{msg.tick:8d} | " + " | ".join(cols))

    rclpy.init()
    try:
        rclpy.spin(Watch())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
