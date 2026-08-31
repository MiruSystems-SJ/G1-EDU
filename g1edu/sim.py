"""g1edu.sim — G1 교육용 시뮬레이터 본체.

모드(상태 기계)  : damp → standing_up → balance_stand ↔ walk
                   (상체 모션은 balance_stand 에서, 이상 시 언제든 damp)
제어             : 관절 PD (토크 한계는 실기체 값으로 클립)
행어(hanger)     : 골반에 가상 스프링-댐퍼 — 실기체의 거치대 역할
낙상 감지        : 골반 높이/기울기 초과 시 자동 damp + error 기록
"""
from __future__ import annotations

import time

import mujoco
import numpy as np
import threading
import time as _time

from .arm import ArmSequencer
from .gait import GaitGenerator, GaitParams
from .model import (ARM_L, ARM_R, G1Model, NUM_MOTORS, PELVIS_STAND_H,
                    nominal_pose)

# 모드 이름 — 교재 2.4절 상태 기계와 대응
DAMP = "damp"
STANDING_UP = "standing_up"
BALANCE_STAND = "balance_stand"
WALK = "walk"

STANDUP_TIME = 3.0
HANGER_ANCHOR_H = 0.80


def _default_gains():
    kp = np.zeros(NUM_MOTORS)
    kd = np.zeros(NUM_MOTORS)
    kp[0:12] = [120, 120, 120, 180, 250, 150] * 2          # 다리
    kd[0:12] = [3.0, 3.0, 3.0, 5.0, 6.0, 4.0] * 2
    kp[12:15] = [150, 150, 150]                          # 허리
    kd[12:15] = [3.0, 3.0, 3.0]
    for a in (ARM_L, ARM_R):                             # 팔
        for name, i in a.items():
            if name.startswith("wrist"):
                kp[i], kd[i] = 20, 0.6
            else:
                kp[i], kd[i] = 60, 1.5
    return kp, kd


class G1Sim:
    """예제(=SDK 창구)가 조작하는 시뮬레이터. ROS 브리지는 관찰 창구."""

    def __init__(self, params: GaitParams | None = None,
                 hanger: bool = False, start_standing: bool = True,
                 allow_walk_arm: bool = False):
        self.m = G1Model()
        self.params = params or GaitParams()
        self.gait = GaitGenerator(self.params)
        self.arm = ArmSequencer(nominal_pose())
        self.kp, self.kd = _default_gains()
        self.kp_walk = self.kp.copy()          # 보행 중 발목 피치 강성 완화(전진 전달)
        self.kp_walk[[4, 10]] = 120.0
        self.kd_walk = self.kd.copy()
        self.kd_walk[[4, 10]] = 3.0
        self.hanger = hanger
        self.allow_walk_arm = allow_walk_arm   # 도전 과제(보행+상체 동시)용
        self.on_tick = None                    # 매 스텝 콜백(ROS 브리지 등)
        self.mode = DAMP
        self.error = ""
        self.tick = 0
        self._standup_t = 0.0
        self._standup_from = np.zeros(NUM_MOTORS)
        self._gain_scale = 0.0
        self._fallen = False

        if start_standing and not hanger:
            self.m.reset_standing()
            self.mode = BALANCE_STAND
            self.gait.idle()
            self._gain_scale = 1.0
        else:
            self.m.reset_hanging(HANGER_ANCHOR_H)
            self.mode = DAMP
        self._q_target = self.m.q().copy()

    # ================================================== 명령(=SDK 창구) ===
    def _reject(self, msg: str) -> tuple[bool, str]:
        self.error = msg
        return False, msg

    def damp(self) -> tuple[bool, str]:
        """전 관절 힘 빼기 — 어느 모드에서든 허용되는 복귀 지점."""
        self.mode = DAMP
        self.gait.idle()
        self.arm.cancel()
        self._gain_scale = 0.0
        return True, "ok"

    def stand_up(self) -> tuple[bool, str]:
        if self.mode != DAMP:
            return self._reject(f"stand_up rejected: mode={self.mode} (damp에서만 가능)")
        if not self.hanger and self.m.pelvis_pos()[2] < 0.55:
            return self._reject("stand_up rejected: 행어 미거치 상태에서 바닥 기립 불가")
        self.mode = STANDING_UP
        self._standup_t = 0.0
        self._standup_from = self.m.q().copy()
        self.error = ""
        return True, "ok"

    def balance_stand(self) -> tuple[bool, str]:
        if self.mode not in (WALK, BALANCE_STAND):
            return self._reject(f"balance_stand rejected: mode={self.mode}")
        self.mode = BALANCE_STAND
        self.gait.idle()
        return True, "ok"

    def move(self, vx: float, vy: float, vyaw: float) -> tuple[bool, str]:
        if self.mode not in (BALANCE_STAND, WALK):
            return self._reject(f"move rejected: mode={self.mode} (기립 후에만 가능)")
        if self.arm.active and not self.allow_walk_arm:
            return self._reject("move rejected: 상체 모션 재생 중")
        if self.mode == BALANCE_STAND:
            self.gait.start()
        self.mode = WALK
        self.gait.set_command(vx, vy, vyaw)
        return True, "ok"

    def stop_move(self) -> tuple[bool, str]:
        if self.mode != WALK:
            return self._reject(f"stop_move rejected: mode={self.mode}")
        self.gait.stop()
        return True, "ok"

    def play_action(self, name: str) -> tuple[bool, str]:
        if self.mode == WALK and not self.allow_walk_arm:
            return self._reject("action rejected: 보행 중 상체 모션은 도전 과제 플래그 필요")
        if self.mode not in (BALANCE_STAND, WALK):
            return self._reject(f"action rejected: mode={self.mode} (기립 상태에서만)")
        if not self.arm.play(name):
            return self._reject(f"action rejected: 미정의 액션 '{name}'")
        self.error = ""
        return True, "ok"

    # ====================================================== 물리 1스텝 ===
    def step(self):
        m, d = self.m.model, self.m.data
        dt = self.m.dt
        rpy = self.m.imu_rpy()

        if self.mode == DAMP:
            self._gain_scale = 0.0
            self._q_target = self.m.q().copy()
        elif self.mode == STANDING_UP:
            self._standup_t += dt
            u = min(self._standup_t / STANDUP_TIME, 1.0)
            u = u * u * (3 - 2 * u)
            self._gain_scale = u
            self._q_target = (1 - u) * self._standup_from + u * nominal_pose()
            if self._standup_t >= STANDUP_TIME:
                self.mode = BALANCE_STAND
                self.gait.idle()
        else:  # BALANCE_STAND / WALK
            self._gain_scale = 1.0
            bv = self.m.data.qvel[0:2]
            import mujoco as _mj
            if not hasattr(self, '_foot_bids'):
                self._foot_bids = [
                    _mj.mj_name2id(self.m.model, _mj.mjtObj.mjOBJ_BODY, n)
                    for n in ("left_ankle_roll_link", "right_ankle_roll_link")]
            fl = self.m.data.xpos[self._foot_bids[0]]
            fr = self.m.data.xpos[self._foot_bids[1]]
            pp = self.m.pelvis_pos()
            self._q_target = self.gait.targets(
                dt, rpy, (float(bv[0]), float(bv[1])), tuple(self.m.imu_gyro()),
                pelvis_xy=(float(pp[0]), float(pp[1])),
                feet_xy=((float(fl[0]), float(fl[1])), (float(fr[0]), float(fr[1]))))
            if self.mode == WALK and not self.gait.s.walking:
                pass
            if self.mode == WALK and self._walk_stopped():
                self.mode = BALANCE_STAND
                self.gait.idle()

        arm_scale = 0.55 if (self.mode == WALK) else 1.0   # 보행 중엔 진폭 축소
        self.arm.apply(dt, self._q_target, scale=arm_scale)

        # ---- PD 토크 ----
        q, dq = self.m.q(), self.m.dq()
        kp = self.kp_walk if self.mode == WALK else self.kp
        kd = self.kd_walk if self.mode == WALK else self.kd
        tau = self._gain_scale * (kp * (self._q_target - q)) - \
            np.maximum(self._gain_scale, 0.15) * kd * dq
        np.clip(tau, -self.m.torque_lim, self.m.torque_lim, out=tau)
        d.ctrl[:] = tau

        # ---- 행어(가상 거치대) ----
        d.xfrc_applied[self.m.pelvis_bid][:] = 0.0
        if self.hanger:
            pos = self.m.pelvis_pos()
            vel = d.cvel[self.m.pelvis_bid][3:6]
            anchor = np.array([0.0, 0.0, HANGER_ANCHOR_H])
            f = np.array([300.0, 300.0, 1800.0]) * (anchor - pos) - \
                np.array([60.0, 60.0, 220.0]) * vel
            f[2] = max(f[2], 0.0)          # 끈: 위로만 당김
            d.xfrc_applied[self.m.pelvis_bid][0:3] = f
            w = d.cvel[self.m.pelvis_bid][0:3]
            r, p, _ = rpy
            d.xfrc_applied[self.m.pelvis_bid][3:6] = \
                np.array([-120.0 * r, -120.0 * p, 0.0]) - 8.0 * w

        mujoco.mj_step(m, d)
        self.tick += 1
        if self.on_tick is not None:
            try:
                self.on_tick(self)
            except Exception as e:
                print(f"[g1edu] on_tick 콜백 오류(계속 진행): {e}")
                self.on_tick = None

        # ---- 낙상 감지 ----
        if not self.hanger and self.mode in (BALANCE_STAND, WALK):
            r, p, _ = self.m.imu_rpy()
            if self.m.pelvis_pos()[2] < 0.45 or abs(r) > 0.7 or abs(p) > 0.7:
                self.damp()
                self.error = "FALL_DETECTED — damp로 전환됨 (교재 4.3 빈출 상황 참고)"
                self._fallen = True

    # ---- 백그라운드 실행(실기체처럼 '항상 돌아가는' 로봇) ------------------
    def start(self, viewer: bool = False, realtime: bool = True):
        """시뮬레이션을 백그라운드 스레드로 시작.

        이후 LocoClient 명령 + time.sleep 의 순차 스크립트 패턴을 그대로 쓸 수
        있습니다(교재 5장). viewer=True 면 MuJoCo 패시브 뷰어를 함께 띄웁니다.
        """
        if getattr(self, "_thread", None) and self._thread.is_alive():
            return
        self._stop_flag = False
        self._viewer_req = viewer
        self._realtime = realtime
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag = True
        if getattr(self, "_thread", None):
            self._thread.join(timeout=2.0)

    def _loop(self):
        v = None
        if self._viewer_req:
            try:
                import mujoco.viewer
                v = mujoco.viewer.launch_passive(self.m.model, self.m.data)
            except Exception as e:  # 헤드리스 환경 등
                print(f"[g1edu] 뷰어를 열 수 없어 헤드리스로 진행합니다: {e}")
        t_wall = _time.time()
        try:
            while not self._stop_flag:
                if v is not None and not v.is_running():
                    break
                self.step()
                if v is not None:
                    v.sync()
                if self._realtime:
                    t_wall += self.m.model.opt.timestep
                    d = t_wall - _time.time()
                    if d > 0:
                        _time.sleep(d)
                    else:
                        t_wall = _time.time()
        finally:
            if v is not None:
                v.close()

    def _walk_stopped(self) -> bool:
        s = self.gait.s
        return (abs(s.vx) + abs(s.vy) + abs(s.vyaw) < 5e-3 and
                self.gait._cmd == (0.0, 0.0, 0.0))

    # ======================================================== 상태 조회 ===
    @property
    def fallen(self) -> bool:
        return self._fallen

    def sim_time(self) -> float:
        return self.m.data.time

    def lowstate(self) -> dict:
        """ROS 브리지·도구가 쓰는 상태 스냅샷 (실기체 LowState 구성을 모사)."""
        q, dq = self.m.q(), self.m.dq()
        tau = self.m.data.actuator_force
        temp = 25.0 + 0.6 * np.abs(tau)
        return dict(tick=self.tick, q=q.copy(), dq=dq.copy(),
                    tau=np.asarray(tau).copy(), temp=temp,
                    quat=self.m.imu_quat().copy(), gyro=self.m.imu_gyro().copy(),
                    acc=self.m.imu_acc().copy(), rpy=self.m.imu_rpy(),
                    mode=self.mode, error=self.error)


# ---------------------------------------------------------------- 실행 루프
def run(sim: G1Sim, duration: float | None = None, viewer: bool = True,
        realtime: bool = True, on_tick=None):
    """뷰어(있으면)와 함께 시뮬레이션을 진행. on_tick(sim)이 False를 주면 종료."""
    def _loop(sync=None):
        t0 = time.perf_counter()
        while True:
            sim.step()
            if on_tick is not None and on_tick(sim) is False:
                return
            if duration is not None and sim.sim_time() >= duration:
                return
            if sync is not None:
                sync()
            elif realtime:
                lag = sim.sim_time() - (time.perf_counter() - t0)
                if lag > 0:
                    time.sleep(min(lag, 0.05))

    if viewer:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(sim.m.model, sim.m.data) as v:
            def sync():
                if not v.is_running():
                    raise KeyboardInterrupt
                v.sync()
            _loop(sync)
    else:
        _loop()
