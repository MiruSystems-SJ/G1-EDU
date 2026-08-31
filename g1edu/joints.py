"""자동 생성 파일 — tools/gen_joint_map.py 로 재생성. 직접 수정 금지.

인덱스 = low state motor_state 순서 = 실기체 DDS 29자유도 순서.
"""

JOINT_NAMES = [
    "left_hip_pitch_joint",  #  0 왼다리
    "left_hip_roll_joint",  #  1 왼다리
    "left_hip_yaw_joint",  #  2 왼다리
    "left_knee_joint",  #  3 왼다리
    "left_ankle_pitch_joint",  #  4 왼다리
    "left_ankle_roll_joint",  #  5 왼다리
    "right_hip_pitch_joint",  #  6 오른다리
    "right_hip_roll_joint",  #  7 오른다리
    "right_hip_yaw_joint",  #  8 오른다리
    "right_knee_joint",  #  9 오른다리
    "right_ankle_pitch_joint",  # 10 오른다리
    "right_ankle_roll_joint",  # 11 오른다리
    "waist_yaw_joint",  # 12 허리
    "waist_roll_joint",  # 13 허리
    "waist_pitch_joint",  # 14 허리
    "left_shoulder_pitch_joint",  # 15 왼팔
    "left_shoulder_roll_joint",  # 16 왼팔
    "left_shoulder_yaw_joint",  # 17 왼팔
    "left_elbow_joint",  # 18 왼팔
    "left_wrist_roll_joint",  # 19 왼팔
    "left_wrist_pitch_joint",  # 20 왼팔
    "left_wrist_yaw_joint",  # 21 왼팔
    "right_shoulder_pitch_joint",  # 22 오른팔
    "right_shoulder_roll_joint",  # 23 오른팔
    "right_shoulder_yaw_joint",  # 24 오른팔
    "right_elbow_joint",  # 25 오른팔
    "right_wrist_roll_joint",  # 26 오른팔
    "right_wrist_pitch_joint",  # 27 오른팔
    "right_wrist_yaw_joint",  # 28 오른팔
]

JOINT_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}
