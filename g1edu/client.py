"""g1edu.client — SDK 창구: LocoClient.

실기체의 unitree_sdk2 high-level 클라이언트와 같은 '모양'의 API입니다.
(함수명·시그니처가 이번 주 교재/예제의 기준 — 교재 5.1절)

    client = LocoClient(sim)
    client.Damp(); client.StandUp(); client.Move(0.1, 0, 0)
    client.StopMove(); client.WaveHand(); client.GetMode()

시뮬레이터 전용입니다. 실기체 연결은 멘토의 검증된 스택으로만 합니다
(docs/real_robot.md — 이 저장소 코드를 실기체에 직접 연결하지 않습니다).
"""
from __future__ import annotations

import time

from .sim import G1Sim


class CommandRejected(RuntimeError):
    """허용되지 않는 상태 전이 — 교재 5.1 '거부당하는 코드를 쓰지 않는 것이 목표'."""


class LocoClient:
    def __init__(self, sim: G1Sim, strict: bool = False):
        """strict=True면 거부된 명령이 예외를 던집니다(기본: False, 로그만)."""
        self.sim = sim
        self.strict = strict

    # ---- 상태 전이 명령 --------------------------------------------------
    def Damp(self):            return self._call(self.sim.damp)
    def StandUp(self):         return self._call(self.sim.stand_up)
    def BalanceStand(self):    return self._call(self.sim.balance_stand)
    def Move(self, vx: float, vy: float, vyaw: float):
        return self._call(lambda: self.sim.move(vx, vy, vyaw))
    def StopMove(self):        return self._call(self.sim.stop_move)

    # ---- 상체 모션 -------------------------------------------------------
    def WaveHand(self):        return self.PlayAction("wave")
    def PlayAction(self, name: str):
        return self._call(lambda: self.sim.play_action(name))

    # ---- 상태 조회 (교재 3.2 '조작 전 확인 루틴') ------------------------
    def GetMode(self) -> str:  return self.sim.mode
    def GetLastError(self) -> str: return self.sim.error
    def ActionActive(self) -> bool: return self.sim.arm.active

    def WaitMode(self, mode: str, timeout: float = 8.0, poll: float = 0.05) -> bool:
        """상태 기반 대기 — 고정 sleep의 대안 (교재 5장 도전 미션)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.sim.mode == mode:
                return True
            time.sleep(poll)
        return False

    # ----------------------------------------------------------------------
    def _call(self, fn):
        ok, msg = fn()
        if not ok:
            print(f"[LocoClient] 명령 거부: {msg}")
            if self.strict:
                raise CommandRejected(msg)
        return ok
