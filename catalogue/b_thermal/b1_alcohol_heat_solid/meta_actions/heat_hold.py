# -*- coding: utf-8 -*-
"""元动作：持续加热——试管停在火焰上方 8s（外焰集中加热）。

用户 2026-08-28 逐字：「最后持续加热持续8s」。预热（PreheatTubePass）结束后试管回到振荡
中心 TUBE_AT_FLAME_2=(0.50,0.131,0.8982)，管底正对外焰；本元动作纯 hold 停留
HEAT_HOLD_FRAMES=480 帧（60fps × 8s）。B1 无温度模型 → 无加热现象（粉末不变色/无气体），
仅停留时长体现「持续加热」；task 侧试管保持 attached 矩阵跟随（_T_HELD_TUBE·tool_world）。
"""
from ._base import BaseMetaAction, hold
from .constants import HEAT_HOLD_FRAMES


class HeatHoldPass(BaseMetaAction):
    """试管停在火焰上方 hold HEAT_HOLD_FRAMES 帧（= 8s 持续加热）。"""

    def _build_actions(self):
        return [hold(self.engine, HEAT_HOLD_FRAMES)]
