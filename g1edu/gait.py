"""g1edu.gait — 파라미터 기반 보행 생성기(CPG + 다리 IK + 자세 피드백).

학생용 파라미터는 config/gait_params.yaml 로 노출됩니다.
정밀한 보행 제어 이론이 아니라 '관찰과 실험'을 위한 교육용 보행기입니다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import (ARM_L, ARM_R, FOOT_FWD, FOOT_Y, LEG_L, LEG_R,
                    PELVIS_STAND_H, WAIST, leg_ik_sagittal, nominal_pose)


@dataclass
class GaitParams:
    # ---- 학생 노출 파라미터 (config/gait_params.yaml) ----
    vx: float = 0.10          # 전진 속도 [m/s]
    vy: float = 0.0           # 좌우 속도 [m/s] (+왼쪽)
    vyaw: float = 0.0         # 회전 속도 [rad/s] (+반시계)
    step_period: float = 0.45 # 걸음 주기(양발 1사이클) [s]
    step_height: float = 0.04 # 유각(스윙) 발 들어올림 높이 [m]
    com_shift: float = 0.045  # 좌우 무게중심 이동 폭 [m]
    pelvis_height: float = PELVIS_STAND_H  # 보행 중 골반 높이 [m]
    arm_swing: float = 0.12   # 보행 중 팔 스윙 폭 [rad] (0이면 고정)

    # ---- 내부 파라미터 (균형 피드백·보정 — 기본값 권장) ----
    fb_ankle_pitch: float = 0.55
    fb_ankle_roll: float = 0.45
    fb_hip_roll: float = 0.30
    fb_waist_pitch: float = 0.40
    fb_gyro_roll: float = 0.06
    fb_gyro_pitch: float = 0.15
    swing_frac: float = 0.55   # 스윙 구간 비율(이후는 이중지지 후퇴)
    fb_shift_roll: float = 0.0
    raibert_x: float = 0.22
    raibert_y: float = 0.35
    speed_kp: float = 0.0
    speed_ki: float = 0.0
    pitch_setpoint: float = 0.06  # 정지 기립의 직립 보정(보행 중엔 미적용)
    max_stride: float = 0.42
    stride_gain: float = 1.0   # 보폭 보정 계수(속도↔보폭 환산)

    @classmethod
    def from_dict(cls, d: dict) -> "GaitParams":
        gp = cls()
        for k, v in (d or {}).items():
            if hasattr(gp, k):
                setattr(gp, k, float(v))
        return gp


@dataclass
class GaitState:
    phase: float = 0.0        # [0,1) — 전반: 왼발 스윙, 후반: 오른발 스윙
    vx: float = 0.0           # 슬루 적용된 실제 명령
    vy: float = 0.0
    vyaw: float = 0.0
    walking: bool = False


def _smoothstep(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class GaitGenerator:
    """phase를 진행시키며 29개 관절 목표를 계산."""

    SLEW_V = 0.4      # [m/s²] 속도 명령 슬루
    SLEW_W = 0.8      # [rad/s²]
    LEAD = 0.55       # 체중이동 위상 선행 [rad]

    def __init__(self, params: GaitParams):
        self.p = params
        self.s = GaitState()
        self.nominal = nominal_pose()
        self._cmd = (0.0, 0.0, 0.0)
        self._ramp = 0.0
        self._vint = 0.0
        self._vf = 0.0
        self._anchor: list = [None, None]     # 지지발 월드 (x,y) 고정점
        self._was_swing = [False, False]
        self._lift_x = [None, None]           # 스윙 시작 시 발의 상대 x (연속성)
        self._vfx = 0.0                       # 착지점 계산용 빠른 속도 필터

    # ------------------------------------------------------------------
    def set_command(self, vx: float, vy: float, vyaw: float):
        self._cmd = (vx, vy, vyaw)

    def start(self):
        self.s.walking = True
        self.s.phase = 0.0
        self._ramp = 0.0

    def stop(self):
        self._cmd = (0.0, 0.0, 0.0)

    def idle(self):
        self.s.walking = False
        self.s.vx = self.s.vy = self.s.vyaw = 0.0
        self._cmd = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    def targets(self, dt: float, rpy, base_vel=(0.0, 0.0), gyro=(0.0, 0.0, 0.0),
                pelvis_xy=(0.0, 0.0), feet_xy=((0.0, 0.0), (0.0, 0.0))) -> np.ndarray:
        p, s = self.p, self.s
        # 명령 슬루(급격한 변속 방지 — 실기체 감각과 동일)
        for i, (name, lim) in enumerate((("vx", self.SLEW_V * dt),
                                         ("vy", self.SLEW_V * dt),
                                         ("vyaw", self.SLEW_W * dt))):
            cur = getattr(s, name)
            setattr(s, name, cur + min(max(self._cmd[i] - cur, -lim), lim))

        roll, pitch, yaw = rpy
        cy, sy = math.cos(yaw), math.sin(yaw)
        vbx = cy * base_vel[0] + sy * base_vel[1]     # 몸통 기준 속도
        vby = -sy * base_vel[0] + cy * base_vel[1]

        stopped = (not s.walking) or (
            abs(s.vx) + abs(s.vy) + abs(s.vyaw) < 1e-3
            and abs(self._cmd[0]) + abs(self._cmd[1]) + abs(self._cmd[2]) < 1e-3
        )

        if stopped:
            s.phase = 0.0
            self._ramp = 0.0
            self._vint = 0.0
            self._vf = 0.0
            self._anchor = [None, None]
            self._was_swing = [False, False]
            self._lift_x = [None, None]
            self._vfx = 0.0
            shift, half, spd_off = 0.0, 0.0, 0.0
        else:
            s.phase = (s.phase + dt / p.step_period) % 1.0
            self._ramp = min(self._ramp + dt / 1.2, 1.0)
            shift = self._ramp * p.com_shift * math.sin(
                2 * math.pi * s.phase + self.LEAD)
            shift += p.fb_shift_roll * (-roll)
            half = self._ramp * 0.25 * min(abs(s.vx) * p.step_period * p.stride_gain,
                                           p.max_stride) * (1 if s.vx >= 0 else -1)
            af = dt / (0.10 + dt)
            self._vfx += af * (vbx - self._vfx)
            # 속도 서보(발목 전략): 느리면 뒤꿈치(-), 빠르면 발끝(+) 압박
            a = dt / (0.30 + dt)
            self._vf += a * (vbx - self._vf)
            verr = s.vx - self._vf
            if self._ramp >= 1.0:
                self._vint = min(max(self._vint + verr * dt, -0.08), 0.08)
            spd_off = -(p.speed_kp * verr + p.speed_ki * self._vint)
            spd_off = min(max(spd_off, -0.025), 0.025)

        q = self.nominal.copy()
        z_leg = p.pelvis_height - 0.1027 - 0.036   # 고관절→발목 수직거리

        for li, (leg, is_left) in enumerate(((LEG_L, True), (LEG_R, False))):
            ph = s.phase if is_left else (s.phase + 0.5) % 1.0
            swing = ph < 0.5 and not stopped
            u = ph / 0.5 if swing else (ph - 0.5) / 0.5
            y_side = FOOT_Y if is_left else -FOOT_Y
            dstride = 0.0  # (차동 보폭은 불안정 유발 → 회전은 hip_yaw로만)
            h2 = half - dstride
            if swing:
                # 스윙(u<0.75): 1.5·h2 전방까지 이동 후 접지 —
                # 이후(u>0.75)는 이중지지: 발을 세계좌표 고정처럼 명령 속도로 후퇴
                cap_x = p.raibert_x * (self._vfx - s.vx)
                cap_y = p.raibert_y * vby
                sf = p.swing_frac
                x_apex = h2 * (1.0 + (1.0 - sf))   # 착지 시 +h2가 되도록 선행량 보정
                if u <= sf:
                    su = _smoothstep(u / sf)
                    x = (FOOT_FWD - h2) + ((h2 + x_apex) + cap_x) * su
                    z = z_leg - self._ramp * p.step_height * math.sin(
                        math.pi * min(u / sf, 1.0))
                else:
                    su = 1.0
                    x = (FOOT_FWD + x_apex + cap_x) \
                        - s.vx * (u - sf) * (p.step_period * 0.5)
                    z = z_leg
                y_v = -0.5 * s.vy * p.step_period * (2 * su - 1) + cap_y * su
            else:
                # 지지: 몸이 지나가는 만큼 발을 뒤로 스윕(개루프 강성 유지)
                x = FOOT_FWD + h2 - 2 * h2 * u
                z = z_leg
                y_v = -0.5 * s.vy * p.step_period * (1 - 2 * u)
            self._was_swing[li] = swing

            hp, kn, ap = leg_ik_sagittal(x, z)
            q[leg["hip_pitch"]] = hp
            q[leg["knee"]] = kn
            q[leg["ankle_pitch"]] = ap
            y_err = shift + y_v
            hr = math.asin(min(max(y_err / z_leg, -0.4), 0.4))
            q[leg["hip_roll"]] = hr
            q[leg["ankle_roll"]] = -hr
            # 회전: 지지발 hip_yaw 반작용으로 골반을 돌림(+vyaw = 반시계, 실측 확정)
            q[leg["hip_yaw"]] = 0.12 * s.vyaw * (1 if swing else -1)

        # ---- 자세 피드백(IMU) — '자세 보상'의 실체 -----------------------
        # 부호 관례(기구학 검증): 앞으로 기울면 pitch>0... 는 아님 —
        #   본 모델 실측: +pitch = 앞기울임(+y축 회전), +roll = 오른쪽 기울임
        pe = pitch - (p.pitch_setpoint if stopped else 0.0)
        ap_fb = p.fb_ankle_pitch * pe + p.fb_gyro_pitch * gyro[1] + spd_off
        q[LEG_L["ankle_pitch"]] += ap_fb
        q[LEG_R["ankle_pitch"]] += ap_fb
        q[LEG_L["ankle_roll"]] += p.fb_ankle_roll * roll + p.fb_gyro_roll * gyro[0]
        q[LEG_R["ankle_roll"]] += p.fb_ankle_roll * roll + p.fb_gyro_roll * gyro[0]
        q[LEG_L["hip_roll"]] -= p.fb_hip_roll * roll
        q[LEG_R["hip_roll"]] -= p.fb_hip_roll * roll
        q[WAIST["pitch"]] = 0.02 - p.fb_waist_pitch * pe

        # ---- 보행 팔 스윙(반대 위상) ------------------------------------
        if not stopped and p.arm_swing > 0:
            sw = self._ramp * p.arm_swing * math.sin(2 * math.pi * s.phase)
            q[ARM_L["shoulder_pitch"]] += sw
            q[ARM_R["shoulder_pitch"]] -= sw
        return q
