# G1 교육용 저장소 (g1-edu)

세종대 로봇 플랫폼 교육 프로그램 — **G1 휴머노이드 로테이션 주간(교재 ④)** 실습
저장소입니다. MuJoCo 기반 G1(29자유도) 시뮬레이터, 실기체 SDK와 같은 모양의
`LocoClient`, ROS 2 상태 토픽, 그리고 교재의 미션에 대응하는 예제들이 들어
있습니다.

> ⚠ **이 저장소는 시뮬레이션 전용입니다.** 실기체 제어 코드는 포함되어 있지
> 않으며, 실기체 연결·조작은 반드시 멘토의 검증된 스택으로만 합니다(교재 2.3).
> 미션 ①이 이 README 통독입니다 — 끝까지 읽고 시작하세요.

---

## 1. 교재 ↔ 저장소 대응표

| 교재 위치 | 할 일 | 저장소에서 쓰는 것 |
|---|---|---|
| 3.3 | SDK 실행 인자(네트워크 인터페이스) 이해 | §7 네트워크 안내 |
| 3.4 미션 A | 클론 → `colcon build` → 토픽 조사 | `g1_edu_interfaces/`, §3·§6 |
| 4장 | 보행 관찰·파라미터 실험·외란 | `examples/01_walk_demo.py`, `config/gait_params.yaml` |
| 5장 | 기립→손 흔들기→복귀, 전이 거부 확인 | `examples/02_wave_demo.py`, §5 API |
| 6장 | 모니터링·관심 관절 출력·인수인계 | `tools/joint_watch.py`, `HANDOVER_TEMPLATE.md` |
| 7장 과제 | 파라미터 세트 비교 / 동작 시퀀스 | `config/gait_params.yaml`, `examples/03_sequence_template.py` |
| 7장 도전 | 보행+상체 동시(시뮬 한정) | `examples/04_walk_and_wave.py` |

## 2. 요구 환경

- Ubuntu 22.04 + **ROS 2 Humble** (교재 3.1에서 준비한 환경)
- Python 3.10+
- ROS 2 없이도(§3의 1단계만으로) 시뮬레이터·예제는 모두 동작합니다.
  ROS 토픽 실습(교재 3.4)에만 2단계가 필요합니다.

## 3. 설치 — 2단계

**1단계: 파이썬 의존성** (시뮬레이터 실행에 필요한 전부)

```bash
cd ~/robot_ws/src/g1-edu     # 클론 위치 예시
pip install -r requirements.txt
```

**2단계: 메시지 패키지 빌드** (ROS 2 토픽 실습용 — 교재 3.4 미션 A)

```bash
cd ~/robot_ws
colcon build --packages-select g1_edu_interfaces
source install/setup.bash    # 새 터미널마다 필요 (또는 ~/.bashrc에 추가)
```

설치 확인:

```bash
python3 -c "import mujoco, numpy, yaml; print('1단계 OK')"
ros2 interface show g1_edu_interfaces/msg/LowState   # 2단계 OK 확인
```

## 4. 빠른 시작

```bash
python3 examples/01_walk_demo.py            # 뷰어가 뜨고 G1이 걷기 시작
```

- **뷰어 조작**: 마우스 드래그 = 회전, 스크롤 = 줌.
- **외란(밀기) 실험(교재 4.4)**: 몸통을 **더블클릭**해 선택한 뒤
  **Ctrl + 오른쪽 드래그**로 미는 힘을 가할 수 있습니다. 얼마나 밀면
  넘어지는지, 회복할 때 발목·골반이 어떻게 움직이는지 관찰하세요.
- 파라미터 실험(교재 4.3): `config/gait_params.yaml` 을 열어 **한 번에 하나만**
  바꾸고 다시 실행 → 관찰표 기록 → 기본값 복원. 넘어지면 화면에
  `FALL_DETECTED` 에러와 함께 기록 안내가 출력됩니다(그게 정상 흐름입니다).

예제 목록:

| 파일 | 내용 | 교재 |
|---|---|---|
| `examples/01_walk_demo.py` | 파라미터 파일로 보행, 실측 속도 리포트 | 4장 |
| `examples/02_wave_demo.py` | 행어 → 기립 → 동작 → 복귀 (try/finally 안전 패턴) | 5장 |
| `examples/03_sequence_template.py` | 7.3 설계 시트를 코드로 옮기는 템플릿 | 7장 과제 2 |
| `examples/04_walk_and_wave.py` | 보행+상체 동시 (도전 · 시뮬 한정) | 7장 도전 |

공통 옵션: `--no-viewer`(헤드리스), `--fast`(실시간 페이스 해제), `01`은
`--ros`(토픽 발행), `02/03`은 `--no-hanger`.

## 5. LocoClient API (교재 5장의 기준)

```python
from g1edu import G1Sim, LocoClient
sim = G1Sim(hanger=True)      # 행어(거치대)에 매달린 상태로 시작
sim.start(viewer=True)        # 백그라운드에서 로봇이 '항상' 돌아감
client = LocoClient(sim)
```

| 함수 | 설명 | 허용 상태 |
|---|---|---|
| `Damp()` | 전신 감쇠(안전 상태). **모든 절차의 시작과 끝** | 언제나 |
| `StandUp()` | 3초에 걸쳐 기립 → 자동으로 `balance_stand` | `damp` |
| `BalanceStand()` | 균형 유지 서기 | `standing_up` 완료 후 자동 |
| `Move(vx, vy, vyaw)` | 보행 시작/속도 변경 [m/s, m/s, rad/s] | `balance_stand`, `walk` |
| `StopMove()` | 보행 정지 → `balance_stand` 복귀 | `walk` |
| `WaveHand()` / `PlayAction(name)` | 상체 동작: `wave`·`hands_up`·`bow` | `balance_stand` |
| `GetMode()` / `GetLastError()` | 현재 모드 / 마지막 에러 문자열 | 언제나 |
| `ActionActive()` | 상체 동작 재생 중인지 | 언제나 |
| `WaitMode(mode, timeout)` | 모드가 될 때까지 대기(상태 기반 대기) | 언제나 |

**모드(FSM)**: `damp → standing_up → balance_stand ↔ walk` (교재 5.1 그림과
동일). 허용되지 않는 전이는 **거부**됩니다 — 기본 설정에서는 거부 로그를
출력하고 `False` 를 반환합니다(실기체 SDK가 에러 코드를 돌려주는 방식과 같은
감각). `LocoClient(sim, strict=True)` 로 만들면 거부 시 `CommandRejected`
예외를 던집니다. *거부당하는 코드를 쓰지 않는 것이 이번 주의 목표입니다.*

전이 사이에는 로봇이 자세를 잡을 시간이 필요합니다 — `02_wave_demo.py` 의
`time.sleep` 위치를 보세요. 넘어지면 어떤 모드에서든 자동으로 `damp` 로
전환되고 `GetLastError()` 에 `FALL_DETECTED` 가 남습니다.

## 6. ROS 2 관찰 창구 (교재 3.4 미션 A)

시뮬레이터에는 창구가 둘 있습니다. **명령은 SDK(LocoClient)로만** 들어가고,
**상태는 ROS 2 토픽으로** 흘러나옵니다. `ros2 topic list` 에 명령 토픽이 안
보이는 이유를 미션 A-4에서 설명하게 될 것입니다.

```bash
# 터미널 1
python3 examples/01_walk_demo.py --ros
# 터미널 2
ros2 topic list
ros2 topic echo /g1/lowstate --once
ros2 topic hz /g1/lowstate
ros2 interface show g1_edu_interfaces/msg/LowState
```

| 토픽 | 타입 | 주기 | 내용 |
|---|---|---|---|
| `/g1/lowstate` | `g1_edu_interfaces/LowState` | 50 Hz | 29관절 q·dq·tau·온도 + IMU |
| `/g1/mode` | `g1_edu_interfaces/ModeState` | 10 Hz | FSM 모드·에러 |

`motor_state[]` 배열 순서 = **실기체 DDS 29자유도 순서**입니다.
전체 표는 `docs/joint_map.md`, 공식 근거는 `docs/g1_joint_index_dds.md`.
관심 관절만 골라 보려면(6장 미션):

```bash
python3 tools/joint_watch.py --joints right_shoulder_roll_joint,right_elbow_joint
```

## 7. 네트워크 인터페이스와 DOMAIN ID (교재 3.3)

- 실기체 SDK 예제가 실행 인자로 받는 "네트워크 인터페이스 이름"은 **로봇과
  연결된 유선 랜 이름**(예: `eth0`)입니다. **시뮬레이터는 로컬에서 돌므로 이
  인자가 필요 없습니다** — 실기체 세션에서 멘토가 값을 알려줍니다.
- 같은 랜의 다른 조와 토픽이 섞이면 `export ROS_DOMAIN_ID=<조별 번호>` 로
  분리하세요(모든 터미널에서 같은 값이어야 함).

## 8. 관찰 노트 — 명령 속도 vs 실측 속도

`01_walk_demo.py` 는 종료 시 **실측 평균속도**를 출력합니다. 기본값에서
`vx=0.10` 을 명령해도 실측은 그보다 낮게 나옵니다. 시뮬 보행기가 관찰·실험용
단순 제어기라 미끄러짐·자세 보상으로 손실이 생기기 때문입니다. *실측을 직접
재고, 명령과 왜 다른지 조에서 토론해 보세요* — 실기체에서도 명령과 실측은
같지 않습니다(sim-to-real의 출발점).

회전(`vyaw`)은 제자리에서 안정적으로 동작하며, 전진과 동시에 걸면 약해지거나
드리프트가 생길 수 있습니다(단순 보행기의 한계 — 이것도 관찰 소재입니다).

## 9. 트러블슈팅

| 증상 | 확인/해결 |
|---|---|
| `colcon build` 실패 | ROS 2 Humble source 여부(`ros2 --help`), `rosidl` 관련 메시지면 `sudo apt install ros-humble-rosidl-default-generators` |
| `ros2 topic echo` 타입 에러 | 빌드 후 **모든 터미널**에서 `source ~/robot_ws/install/setup.bash` |
| 토픽이 아무것도 안 보임 | `01_walk_demo.py` 를 `--ros` 로 실행했는지, `ROS_DOMAIN_ID` 가 터미널마다 같은지 |
| 뷰어가 안 뜸 / GL 에러 | 원격·WSL2 등 그래픽 제약 환경 → `--no-viewer` 로 실행. 데이터 관찰은 토픽으로 가능 |
| `ModuleNotFoundError: mujoco` | venv 활성화 여부, `pip install -r requirements.txt` 를 그 venv에서 했는지 |
| 로봇이 자꾸 넘어짐 | 파라미터를 기본값으로 복원(교재 4.3 원칙). 어떤 값에서 넘어졌는지가 곧 실험 결과 — 관찰표에 기록 |
| `FALL_DETECTED` 후 명령 거부 | 정상 동작(자동 damp). `StandUp()` 부터 다시 |

교재 4.3 "자주 겪는 상황 미리보기"의 각 항목이 위 표의 어느 줄에 해당하는지
찾아보는 것도 좋은 점검이 됩니다.

## 10. 산출물과 인수인계

- 관찰표·미션 답은 교재 양식대로 조 폴더에 정리
- 주간 인수인계는 `HANDOVER_TEMPLATE.md` 를 복사해 작성 (교재 6장 산출물 ③)

## 11. 저장소 구조

```
g1-edu/
├── g1edu/                  # 시뮬레이터 파이썬 패키지
│   ├── model.py            #   모델 로딩·IK·관절 인덱스
│   ├── gait.py             #   보행 생성기(파라미터는 config/에 노출)
│   ├── arm.py              #   상체 동작 시퀀서
│   ├── sim.py              #   FSM·PD 제어·행어·낙상 감지
│   ├── client.py           #   LocoClient (SDK 창구)
│   ├── ros_bridge.py       #   ROS 2 발행자 (관찰 창구)
│   └── joints.py           #   관절 이름/인덱스 (자동 생성)
├── g1_edu_interfaces/      # ROS 2 메시지 패키지 (colcon 대상)
├── examples/               # 01~04 예제
├── tools/                  # joint_watch, gen_joint_map
├── config/gait_params.yaml # 학생용 보행 파라미터
├── docs/                   # joint_map, DDS 근거 문서, real_robot(멘토용)
└── assets/g1/              # Unitree G1 모델 (BSD-3, 고지 포함)
```

## 12. 라이선스

- 본 저장소 코드: MIT (`LICENSE`)
- G1 모델(`assets/g1/`): Unitree BSD-3-Clause — `THIRD_PARTY_NOTICES.md` 참고
