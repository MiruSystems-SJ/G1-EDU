# 제3자 라이선스 고지

## Unitree G1 MuJoCo 모델 (`assets/g1/`)

- 출처: [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
  및 unitree_ros 의 G1 description (29 DOF)
- 라이선스: **BSD-3-Clause** — 원문은 `assets/g1/LICENSE.unitree_mujoco` 에 포함
- 본 저장소는 모델 파일(MJCF, 메시)을 교육 목적으로 재배포하며,
  원저작권 고지와 라이선스 조항을 유지합니다.

## 관절 인덱스 문서 (`docs/g1_joint_index_dds.md`)

- Unitree 공식 문서의 G1 29자유도 관절 인덱스 표 사본(출처 표기 포함).
- 교재 3.4 미션(관절 배열 순서 확인)의 근거 자료로 포함합니다.

## API 명칭에 관하여

`LocoClient` 의 함수 이름(Damp, StandUp, Move 등)은 실기체 SDK
(unitree_sdk2 / unitree_sdk2_python)의 high-level 클라이언트와 같은
'모양'이 되도록 지은 것이며, 해당 SDK의 코드는 포함하지 않습니다.
