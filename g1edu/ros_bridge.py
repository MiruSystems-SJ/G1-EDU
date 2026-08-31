"""g1edu.ros_bridge — 시뮬레이터 상태를 ROS 2 토픽으로 발행(관찰 전용).

두 개의 창구(교재 3.4·5장):
  - 제어 창구  : LocoClient (SDK 모양) — 명령은 이쪽으로만 들어간다
  - 관찰 창구  : ROS 2 토픽            — 상태는 이쪽으로 흘러나온다
  → 그래서 `ros2 topic list` 에는 상태 토픽만 보이고, 명령 토픽은 없다.
    (교재 3.4 미션 A-4 "제어 명령은 어느 경로로 들어가는가"의 답과 연결)

발행 토픽:
  /g1/lowstate  g1_edu_interfaces/msg/LowState   (기본 50 Hz)
  /g1/mode      g1_edu_interfaces/msg/ModeState  (기본 10 Hz)

rclpy 또는 g1_edu_interfaces 가 없으면 안내를 출력하고 조용히 비활성화된다.
(pip 만 설치한 환경에서도 예제가 그대로 돌아가게 하기 위함)
"""
from __future__ import annotations


class RosBridge:
    def __init__(self, lowstate_hz: float = 50.0, mode_hz: float = 10.0):
        self.ok = False
        self._why = ""
        self._ls_every = 1
        self._mode_every = 1
        try:
            import rclpy  # noqa: F401
        except ImportError:
            self._why = ("rclpy 를 찾을 수 없습니다 — ROS 2 Humble 환경을 source "
                         "했는지 확인하세요 (교재 3.1). ROS 없이도 시뮬레이터는 "
                         "정상 동작합니다.")
            return
        try:
            from g1_edu_interfaces.msg import (IMUState, LowState, ModeState,
                                               MotorState)  # noqa: F401
        except ImportError:
            self._why = ("g1_edu_interfaces 메시지를 찾을 수 없습니다 — "
                         "~/robot_ws 에서 colcon build 후 "
                         "`source install/setup.bash` 를 했는지 확인하세요 "
                         "(교재 3.4 미션 A-2).")
            return

        import rclpy
        from g1_edu_interfaces.msg import LowState, ModeState

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = rclpy.create_node("g1_sim")
        self.pub_low = self.node.create_publisher(LowState, "/g1/lowstate", 10)
        self.pub_mode = self.node.create_publisher(ModeState, "/g1/mode", 10)
        self.lowstate_hz = lowstate_hz
        self.mode_hz = mode_hz
        self.ok = True

    # ------------------------------------------------------------------
    def attach(self, sim) -> bool:
        """sim.on_tick 에 연결. 성공 여부를 돌려준다."""
        if not self.ok:
            print(f"[RosBridge] 비활성: {self._why}")
            return False
        dt = sim.m.model.opt.timestep
        self._ls_every = max(1, round(1.0 / (self.lowstate_hz * dt)))
        self._mode_every = max(1, round(1.0 / (self.mode_hz * dt)))
        sim.on_tick = self._on_tick
        print(f"[RosBridge] 발행 시작: /g1/lowstate {self.lowstate_hz:.0f} Hz, "
              f"/g1/mode {self.mode_hz:.0f} Hz")
        return True

    def _on_tick(self, sim):
        if sim.tick % self._ls_every == 0:
            self.pub_low.publish(self._make_lowstate(sim))
        if sim.tick % self._mode_every == 0:
            self.pub_mode.publish(self._make_mode(sim))

    # ------------------------------------------------------------------
    def _make_lowstate(self, sim):
        from g1_edu_interfaces.msg import IMUState, LowState, MotorState
        ls = sim.lowstate()
        msg = LowState()
        msg.tick = int(ls["tick"])
        for i in range(len(ls["q"])):
            m = MotorState()
            m.q = float(ls["q"][i])
            m.dq = float(ls["dq"][i])
            m.tau_est = float(ls["tau"][i])
            m.temperature = float(ls["temp"][i])
            msg.motor_state.append(m)
        imu = IMUState()
        imu.quaternion = [float(v) for v in ls["quat"]]
        imu.gyroscope = [float(v) for v in ls["gyro"]]
        imu.accelerometer = [float(v) for v in ls["acc"]]
        imu.rpy = [float(v) for v in ls["rpy"]]
        msg.imu_state = imu
        return msg

    def _make_mode(self, sim):
        from g1_edu_interfaces.msg import ModeState
        ls = sim.lowstate()
        msg = ModeState()
        msg.tick = int(ls["tick"])
        msg.mode = str(ls["mode"])
        msg.error = str(ls["error"])
        return msg

    def shutdown(self):
        if self.ok:
            self.node.destroy_node()
