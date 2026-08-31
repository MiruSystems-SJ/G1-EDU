"""g1edu.model — G1 29DoF 모델 로딩과 관절 인덱스, 기본 자세, 다리 IK.

관절/액추에이터 순서는 Unitree 공식 MJCF(g1_29dof.xml)를 그대로 따르며,
이는 실기체 DDS LowState/LowCmd의 모터 순서와 동일합니다.
(docs/joint_map.md, docs/g1_joint_index_dds.md 참조)
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_XML = REPO_ROOT / "assets" / "g1" / "scene_29dof.xml"

NUM_MOTORS = 29

# --- 부위별 액추에이터 인덱스 (MJCF/DDS 공통 순서) ---------------------------
LEG_L = dict(hip_pitch=0, hip_roll=1, hip_yaw=2, knee=3, ankle_pitch=4, ankle_roll=5)
LEG_R = dict(hip_pitch=6, hip_roll=7, hip_yaw=8, knee=9, ankle_pitch=10, ankle_roll=11)
WAIST = dict(yaw=12, roll=13, pitch=14)
ARM_L = dict(shoulder_pitch=15, shoulder_roll=16, shoulder_yaw=17, elbow=18,
             wrist_roll=19, wrist_pitch=20, wrist_yaw=21)
ARM_R = dict(shoulder_pitch=22, shoulder_roll=23, shoulder_yaw=24, elbow=25,
             wrist_roll=26, wrist_pitch=27, wrist_yaw=28)

# --- 링크 길이 (모델 기하에서 측정) ------------------------------------------
THIGH_LEN = 0.3366   # hip_pitch → knee
SHANK_LEN = 0.3004   # knee → ankle_pitch
HIP_OFFSET_Y = 0.0645          # pelvis 중심 → 고관절 (좌 +y)
FOOT_Y = 0.1185
FOOT_FWD = 0.008               # 발목이 전신 COM 아래에 오도록 하는 전방 오프셋                # 기본 발 좌우 간격의 절반
PELVIS_STAND_H = 0.755         # 기립 시 pelvis 목표 높이(지면 기준)

# --- 기본(명목) 관절 목표 -----------------------------------------------------
def nominal_pose() -> np.ndarray:
    """기립 자세의 관절 목표(29,). 다리는 IK로, 상체는 자연스러운 값으로."""
    q = np.zeros(NUM_MOTORS)
    hp, kn, ap = leg_ik_sagittal(FOOT_FWD, PELVIS_STAND_H - 0.1027 - 0.036)
    for leg in (LEG_L, LEG_R):
        q[leg["hip_pitch"]] = hp
        q[leg["knee"]] = kn
        q[leg["ankle_pitch"]] = ap
    # 팔: 자연스럽게 몸 옆에 살짝 벌리고 팔꿈치 약간 굽힘
    for arm, sgn in ((ARM_L, +1.0), (ARM_R, -1.0)):
        q[arm["shoulder_roll"]] = 0.16 * sgn
        q[arm["shoulder_pitch"]] = 0.30
        q[arm["elbow"]] = 0.60
    q[WAIST["pitch"]] = 0.02
    return q


def leg_ik_sagittal(x_fwd: float, z_down: float) -> tuple[float, float, float]:
    """시상면 2링크 IK.

    x_fwd  : 고관절 기준 발목의 전방 거리 [m] (+앞)
    z_down : 고관절 기준 발목까지의 수직 거리 [m] (+아래)
    반환   : (hip_pitch, knee, ankle_pitch) — 발바닥 수평 기준.
    관례   : G1 관절은 +가 '뒤로 기울임'(y축 우수계). 무릎 +가 굽힘.
    """
    l1, l2 = THIGH_LEN, SHANK_LEN
    d = math.hypot(x_fwd, z_down)
    d = min(max(d, 1e-6), (l1 + l2) * 0.999)
    cos_k = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    knee = math.acos(max(-1.0, min(1.0, cos_k)))          # 0=폄, +=굽힘
    alpha = math.atan2(x_fwd, z_down)                      # HA선의 전방 기울기
    beta = math.atan2(l2 * math.sin(knee), l1 + l2 * math.cos(knee))
    hip_pitch = -(alpha + beta)
    ankle_pitch = -(hip_pitch + knee)                      # 발바닥 수평
    return hip_pitch, knee, ankle_pitch


class G1Model:
    """MjModel/MjData + 자주 쓰는 인덱스 캐시."""

    def __init__(self, xml_path: str | Path = SCENE_XML):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.dt = m.opt.timestep

        # 액추에이터 i ↔ 관절 qpos/qvel 주소
        self.act_joint = [m.actuator_trnid[i][0] for i in range(m.nu)]
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.act_joint])
        self.vadr = np.array([m.jnt_dofadr[j] for j in self.act_joint])
        self.torque_lim = m.actuator_ctrlrange[:, 1].copy()
        self.joint_names = [
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.act_joint
        ]
        self.pelvis_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

        def sadr(name):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            return m.sensor_adr[sid], m.sensor_dim[sid]

        self._s_quat = sadr("imu_quat")
        self._s_gyro = sadr("imu_gyro")
        self._s_acc = sadr("imu_acc")

    # ------------------------------------------------------------------ 상태
    def q(self) -> np.ndarray:
        return self.data.qpos[self.qadr]

    def dq(self) -> np.ndarray:
        return self.data.qvel[self.vadr]

    def imu_quat(self) -> np.ndarray:
        a, n = self._s_quat
        return self.data.sensordata[a:a + n]

    def imu_gyro(self) -> np.ndarray:
        a, n = self._s_gyro
        return self.data.sensordata[a:a + n]

    def imu_acc(self) -> np.ndarray:
        a, n = self._s_acc
        return self.data.sensordata[a:a + n]

    def imu_rpy(self) -> tuple[float, float, float]:
        w, x, y, z = self.imu_quat()
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def pelvis_pos(self) -> np.ndarray:
        return self.data.xpos[self.pelvis_bid]

    def reset_standing(self):
        """기립 자세로 초기화(발이 지면에 닿도록)."""
        mujoco.mj_resetData(self.model, self.data)
        q = nominal_pose()
        self.data.qpos[self.qadr] = q
        self.data.qpos[0:3] = [0.0, 0.0, PELVIS_STAND_H + 0.005]
        # 미세한 초기 후방 피치: 접촉 평형이 직립 분지에 정착하도록(실측 튜닝)
        ip = -0.06
        self.data.qpos[3:7] = [math.cos(ip / 2), 0, math.sin(ip / 2), 0]
        mujoco.mj_forward(self.model, self.data)

    def reset_hanging(self, anchor_h: float):
        """행어에 매달린(무릎 약간 굽힌) 초기 상태."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = 0.0
        self.data.qpos[0:3] = [0.0, 0.0, anchor_h]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
