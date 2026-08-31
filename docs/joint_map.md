# G1 관절 인덱스 맵 (29 DOF)

- 이 표의 **인덱스 = 시뮬레이터 low state의 `motor_state` 배열 순서 = 실기체 DDS 29자유도 순서**입니다.
- 공식 근거 문서: `docs/g1_joint_index_dds.md` (Unitree 문서 사본)
- 교재 3.4 미션 A-3(관절 배열 순서 확인), 6장 모니터링 미션에서 사용합니다.

| idx | 관절 이름 (MJCF/DDS 동일) | 부위 |
|---:|---|---|
| 0 | `left_hip_pitch_joint` | 왼다리 |
| 1 | `left_hip_roll_joint` | 왼다리 |
| 2 | `left_hip_yaw_joint` | 왼다리 |
| 3 | `left_knee_joint` | 왼다리 |
| 4 | `left_ankle_pitch_joint` | 왼다리 |
| 5 | `left_ankle_roll_joint` | 왼다리 |
| 6 | `right_hip_pitch_joint` | 오른다리 |
| 7 | `right_hip_roll_joint` | 오른다리 |
| 8 | `right_hip_yaw_joint` | 오른다리 |
| 9 | `right_knee_joint` | 오른다리 |
| 10 | `right_ankle_pitch_joint` | 오른다리 |
| 11 | `right_ankle_roll_joint` | 오른다리 |
| 12 | `waist_yaw_joint` | 허리 |
| 13 | `waist_roll_joint` | 허리 |
| 14 | `waist_pitch_joint` | 허리 |
| 15 | `left_shoulder_pitch_joint` | 왼팔 |
| 16 | `left_shoulder_roll_joint` | 왼팔 |
| 17 | `left_shoulder_yaw_joint` | 왼팔 |
| 18 | `left_elbow_joint` | 왼팔 |
| 19 | `left_wrist_roll_joint` | 왼팔 |
| 20 | `left_wrist_pitch_joint` | 왼팔 |
| 21 | `left_wrist_yaw_joint` | 왼팔 |
| 22 | `right_shoulder_pitch_joint` | 오른팔 |
| 23 | `right_shoulder_roll_joint` | 오른팔 |
| 24 | `right_shoulder_yaw_joint` | 오른팔 |
| 25 | `right_elbow_joint` | 오른팔 |
| 26 | `right_wrist_roll_joint` | 오른팔 |
| 27 | `right_wrist_pitch_joint` | 오른팔 |
| 28 | `right_wrist_yaw_joint` | 오른팔 |
