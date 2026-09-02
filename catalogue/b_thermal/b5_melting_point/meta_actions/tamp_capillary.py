# -*- coding: utf-8 -*-
"""TampCapillary：夹开口端拎起后（封口端朝下），竖直方向上下快速来回 N 次，把粉从开口端
震到封口端。

用户 2026-08-31 逐字：「第二次夹毛细管的 -X 端（实际开口端 +X），还是拎起来，这样子自动
竖直就可以上下快速移动把粉末从一端搞到另一端」——夹开口端拎起后毛细管竖直、封口端朝下、
粉在开口端（上），夹爪沿世界 Z 快速正弦往复 10 个来回，粉靠重力+惯性从开口端落到封口端
压实（经典熔点管装样「震实」的模拟简化：不做长玻璃管下落，只竖直快速来回）。

机制（仿 ShakeAction axis=Z，首帧采样拎起后实际朝向保持「封口端朝下」）：
  首帧采样拎起后**实际**朝向（fk_pose → _R_to_quat_wxyz，避免 mv orient=None 回退引擎默认
  手指朝下把毛细管又转回水平）→ 委托 ShakeAction axis=(0,0,1) 正弦振荡 cycles 次
  （相位 0→2π·cycles，首尾回中心，纯竖直无横移，全程保持朝向）。
"""
import numpy as np

from controllers.atomic_actions.flametest.shake_action import ShakeAction

from ._base import BaseMetaAction
from .move_preserve import _R_to_quat_wxyz
from .constants import GRASP_OPEN, TAMP_CENTER_Z, TAMP_AMP, TAMP_CYCLES, TAMP_PERIOD


class TampVerticalAction:
    """竖直快速来回 N 次：首帧采样当前朝向 → 转 ShakeAction axis=Z（保持朝向）。"""

    def __init__(self, engine, center, amplitude, cycles, period):
        self.engine = engine
        self.center = np.asarray(center, dtype=float)
        self.amplitude = float(amplitude)
        self.cycles = int(cycles)
        self.period = int(period)
        self.reset()

    def reset(self):
        self._shake = None   # 首帧采样朝向后才构造（拎起后朝向未知）
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        if self._shake is None:
            _, R = self.engine.fk_pose(cur)
            orient_q = _R_to_quat_wxyz(R)
            print(f"[b5tamp] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
                  f"{orient_q[2]:.4f},{orient_q[3]:.4f}] center z={self.center[2]:.3f} "
                  f"amp={self.amplitude:.3f} cycles={self.cycles}")
            self._shake = ShakeAction(self.engine, self.center, axis=(0, 0, 1),
                                      amplitude=self.amplitude, cycles=self.cycles,
                                      period=self.period, orient=orient_q)
        cmd = self._shake.forward(joints, gripper_pos, grip_target)
        self._frame += 1
        if self._shake.is_done():
            self._done = True
        return cmd

    def is_done(self):
        return self._done


class TampCapillary(BaseMetaAction):
    """夹开口端拎起后：竖直方向上下快速来回 10 次震实（保持「封口端朝下」朝向）。"""

    def _build_actions(self):
        e = self.engine
        gx, gy, _ = GRASP_OPEN
        return [
            TampVerticalAction(e, (gx, gy, TAMP_CENTER_Z),
                               amplitude=TAMP_AMP, cycles=TAMP_CYCLES, period=TAMP_PERIOD),
        ]
