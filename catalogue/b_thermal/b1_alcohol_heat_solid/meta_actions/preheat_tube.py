# -*- coding: utf-8 -*-
"""元动作：预热——试管在酒精灯火焰上方 y 向 ±2cm 往复 5 次（慢速）后停稳。

用户 2026-08-28 逐字：「现在需要加的动作是来回预热，现在加动作，在y的方向上来回移动2cm，
来回移动5次速度不要太快，最后持续加热持续8s，最后放回试管，先写完这些动作。」

PickTubePass ⑨ 已把试管移到 TUBE_AT_FLAME_2=(0.50,0.131,0.8982)（管底进外焰），本元动作：
  ① HeatSweepAction  y 向 ±2cm 正弦往复 5 次（周期 150 帧 = 2.5s/来回，慢速），全程保持
     法兰转 -95° 的倾斜姿态（首帧采样实际朝向，R_off 自动抵消）
  ② hold(SETTLE)     预热结束停稳（正弦相位回中心 y=0.131，管底仍正对外焰）
task 侧：试管持续 attached（矩阵持握 _T_HELD_TUBE·tool_world 随夹爪 6-DOF 转，含白粉柱），无新判定。
"""
from ._base import BaseMetaAction, hold
from .constants import (SETTLE, TUBE_AT_FLAME_2,
                        PREHEAT_AMPLITUDE, PREHEAT_CYCLES, PREHEAT_PERIOD)
from .heat_sweep import HeatSweepAction


class PreheatTubePass(BaseMetaAction):
    """试管在火焰上方 y 向 ±2cm 往复 5 次预热（慢速），预热后停稳准备持续加热。"""

    def _build_actions(self):
        e = self.engine
        return [
            HeatSweepAction(e, TUBE_AT_FLAME_2, axis=(0, 1, 0),
                            amplitude=PREHEAT_AMPLITUDE, cycles=PREHEAT_CYCLES,
                            period=PREHEAT_PERIOD),   # ① y 向 ±2cm 往复 5 次（慢速）
            hold(e, SETTLE),                          # ② 预热结束停稳
        ]
