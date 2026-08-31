#!/usr/bin/env python3
"""docs/joint_map.md 와 g1edu/joints.py 를 MJCF 모델에서 자동 생성.

관절 순서(액추에이터 순서)는 실기체 DDS 29자유도 인덱스와 동일하다.
근거: docs/g1_joint_index_dds.md (Unitree 공식 문서 사본)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mujoco  # noqa: E402

from g1edu.model import G1Model  # noqa: E402


def part_of(idx: int) -> str:
    if idx <= 5:
        return "왼다리"
    if idx <= 11:
        return "오른다리"
    if idx <= 14:
        return "허리"
    if idx <= 21:
        return "왼팔"
    return "오른팔"


def main():
    m = G1Model()
    rows = []
    for i in range(m.model.nu):
        jid = m.model.actuator_trnid[i][0]
        name = mujoco.mj_id2name(m.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        rows.append((i, name, part_of(i)))

    root = os.path.join(os.path.dirname(__file__), "..")
    with open(os.path.join(root, "docs", "joint_map.md"), "w") as f:
        f.write("# G1 관절 인덱스 맵 (29 DOF)\n\n")
        f.write("- 이 표의 **인덱스 = 시뮬레이터 low state의 `motor_state` 배열 순서 = "
                "실기체 DDS 29자유도 순서**입니다.\n")
        f.write("- 공식 근거 문서: `docs/g1_joint_index_dds.md` (Unitree 문서 사본)\n")
        f.write("- 교재 3.4 미션 A-3(관절 배열 순서 확인), 6장 모니터링 미션에서 사용합니다.\n\n")
        f.write("| idx | 관절 이름 (MJCF/DDS 동일) | 부위 |\n|---:|---|---|\n")
        for i, name, part in rows:
            f.write(f"| {i} | `{name}` | {part} |\n")

    with open(os.path.join(root, "g1edu", "joints.py"), "w") as f:
        f.write('"""자동 생성 파일 — tools/gen_joint_map.py 로 재생성. 직접 수정 금지.\n\n')
        f.write('인덱스 = low state motor_state 순서 = 실기체 DDS 29자유도 순서.\n"""\n\n')
        f.write("JOINT_NAMES = [\n")
        for i, name, part in rows:
            f.write(f'    "{name}",  # {i:2d} {part}\n')
        f.write("]\n\nJOINT_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}\n")
    print(f"생성 완료: docs/joint_map.md, g1edu/joints.py ({len(rows)}개 관절)")


if __name__ == "__main__":
    main()
