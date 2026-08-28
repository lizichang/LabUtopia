# -*- coding: utf-8 -*-
"""HeatSweepAction：预热——试管在火焰上方沿 y 向正弦往复（连续移动，无冻结判定）。

用户 2026-08-28 逐字：「现在需要加的动作是来回预热，现在加动作，在y的方向上来回移动2cm，
来回移动5次速度不要太快」。试管已由 PickTubePass ⑨ 移到 TUBE_AT_FLAME_2=(0.50,0.131,0.8982)
（管底进外焰），本动作在 y 向 ±2cm 往复 5 次（慢速）——管内样品受热均匀（预热）。

机制 = 延迟采样朝向 + ShakeAction（flametest shake_action.py）：ShakeAction 在 center 附近沿
axis 正弦振荡，目标是**连续移动** → 不能用 MoveAction 的「TCP 距目标 <1cm 冻结」判定（会误冻）；
振荡总帧数到即 done。与 MovePreserveTubeAction 同理：首帧 fk_pose 采样当前**实际**工具朝向
（试管处于法兰转 -95° 的倾斜姿态，喂给 Lula 的 target_orientation 读 (w,x,y,z) scalar-first），
振荡全程保持该朝向（R_off 自动抵消，不会被甩回竖直）。

换热：ShakeAction(center=TUBE_AT_FLAME_2, axis=(0,1,0), amplitude=0.02, cycles=5, period=150)。
y 0.131±0.02=[0.111,0.151] 距灯中心 (0.50,0.0029) y≥10.8cm 无碰撞；相位 0→10π 首尾回中心，
起止无横移。夹爪通道每帧发 grip_target（保持试管夹住）。
"""
import numpy as np

from controllers.atomic_actions.flametest.shake_action import ShakeAction
from .move_x_preserve import _R_to_quat_wxyz


class HeatSweepAction:
    """试管在火焰上方 y 向正弦往复 cycles 次（慢速），全程保持当前倾斜朝向。"""

    def __init__(self, engine, center, axis=(0, 1, 0), amplitude=0.02,
                 cycles=5, period=150, label="heat_sweep"):
        self.engine = engine
        self.center = np.asarray(center, dtype=float)
        self.axis = np.asarray(axis, dtype=float)
        self.amplitude = float(amplitude)
        self.cycles = int(cycles)
        self.period = int(period)
        self.label = label
        self._shake = None
        self._done = False

    def reset(self):
        self._shake = None
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        if self._shake is None:
            cur = np.asarray(joints[:7], dtype=float)
            _, R = self.engine.fk_pose(cur)
            orient_q = _R_to_quat_wxyz(R)   # 采样当前实际朝向（法兰 -95° 倾斜姿态）
            print(f"[heat_sweep] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
                  f"{orient_q[2]:.4f},{orient_q[3]:.4f}] cycles={self.cycles} "
                  f"amp={self.amplitude:.3f} period={self.period}")
            self._shake = ShakeAction(self.engine, self.center, axis=self.axis,
                                      amplitude=self.amplitude, cycles=self.cycles,
                                      period=self.period, orient=orient_q, label=self.label)
        cmd = self._shake.forward(joints, gripper_pos, grip_target)
        self._done = self._shake.is_done()
        return cmd

    def is_done(self):
        return self._done
