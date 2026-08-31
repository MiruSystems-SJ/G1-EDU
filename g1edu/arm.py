"""g1edu.arm — 상체(팔·손목) 모션 시퀀스 정의와 재생기.

각 액션은 (관절목표 dict, 이동시간, 유지시간)의 키프레임 목록입니다.
관절 이름은 model.ARM_L/ARM_R/WAIST 의 키를 '부위.관절' 형태로 씁니다.
예: "R.shoulder_roll", "L.elbow", "W.pitch"
"""
from __future__ import annotations

import numpy as np

from .model import ARM_L, ARM_R, WAIST

_IDX = {}
for k, v in ARM_L.items():
    _IDX[f"L.{k}"] = v
for k, v in ARM_R.items():
    _IDX[f"R.{k}"] = v
for k, v in WAIST.items():
    _IDX[f"W.{k}"] = v

# ---------------------------------------------------------------- 액션 정의
# 좌표 관례: shoulder_roll  L:+바깥 / R:-바깥,  shoulder_pitch +뒤(-앞·위),
#            elbow +굽힘,  W.pitch +앞으로 숙임
ARM_ACTIONS: dict[str, list[tuple[dict, float, float]]] = {
    # 오른손 흔들기 — 5장 미션 B의 그 예제
    "wave": [
        ({"R.shoulder_roll": -1.45, "R.shoulder_pitch": -0.35, "R.elbow": 1.30}, 1.0, 0.2),
        ({"R.shoulder_yaw": +0.45}, 0.35, 0.0),
        ({"R.shoulder_yaw": -0.45}, 0.35, 0.0),
        ({"R.shoulder_yaw": +0.45}, 0.35, 0.0),
        ({"R.shoulder_yaw": -0.45}, 0.35, 0.0),
        ({"R.shoulder_yaw": 0.0}, 0.3, 0.1),
    ],
    # 두 팔 앞으로 들어올리기
    "hands_up": [
        ({"L.shoulder_pitch": -1.30, "R.shoulder_pitch": -1.30,
          "L.elbow": 0.25, "R.elbow": 0.25}, 1.2, 0.8),
    ],
    # 가벼운 목례(허리) — 기립 상태 전용
    "bow": [
        ({"W.pitch": 0.32}, 0.9, 0.7),
    ],
}


class ArmSequencer:
    """액션 키프레임을 시간 보간으로 재생. 종료 시 자동으로 명목자세 복귀."""

    RETURN_TIME = 1.0

    def __init__(self, nominal: np.ndarray):
        self.nominal = nominal
        self._frames: list[tuple[dict, float, float]] = []
        self._fi = 0
        self._t = 0.0
        self._from: dict[int, float] = {}
        self._cur: dict[int, float] = {}
        self.active = False
        self.name = ""

    def play(self, name: str) -> bool:
        if name not in ARM_ACTIONS:
            return False
        frames = list(ARM_ACTIONS[name])
        # 마지막에 복귀 프레임 자동 추가
        frames.append(({k: float(self.nominal[i]) for k, i in _IDX.items()
                        if i in self._collect_idx(frames)}, self.RETURN_TIME, 0.0))
        self._frames = frames
        self._fi = -1
        self._t = 0.0
        self._cur = {}
        self.active = True
        self.name = name
        self._advance()
        return True

    @staticmethod
    def _collect_idx(frames) -> set[int]:
        s = set()
        for tgt, _, _ in frames:
            for k in tgt:
                s.add(_IDX[k])
        return s

    def _advance(self):
        self._fi += 1
        if self._fi >= len(self._frames):
            self.active = False
            self.name = ""
            return
        tgt, _, _ = self._frames[self._fi]
        self._from = dict(self._cur)
        for k, v in tgt.items():
            i = _IDX[k]
            self._from.setdefault(i, self._cur.get(i, float(self.nominal[i])))
            self._cur[i] = float(v)
        self._t = 0.0

    def cancel(self):
        self.active = False
        self._frames = []
        self._cur = {}
        self.name = ""

    def apply(self, dt: float, q_target: np.ndarray, scale: float = 1.0):
        """현재 프레임을 보간해 q_target 위에 덮어쓴다.

        scale<1 이면 명목자세 기준 진폭을 줄여 재생(보행 중 도전 과제용).
        """
        if not self.active:
            return
        tgt, move, hold = self._frames[self._fi]
        self._t += dt
        u = 1.0 if move <= 0 else min(self._t / move, 1.0)
        u = u * u * (3 - 2 * u)
        for i, v_to in self._cur.items():
            v_from = self._from.get(i, float(self.nominal[i]))
            v = v_from + (v_to - v_from) * u
            n = float(self.nominal[i])
            q_target[i] = n + scale * (v - n)
        if self._t >= move + hold:
            self._advance()
