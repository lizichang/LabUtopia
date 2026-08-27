# -*- coding: utf-8 -*-
"""A2 ② 挤洗瓶出水：挤胶头（<0.025 出水动画）→ 松开保持吸附。

d2s SqueezeWater 同款。
"""
from ._base import BaseMetaAction, grip
from .constants import GRIP_WASHBOT, WASH_SQUEEZE, WASH_SQUEEZE_DWELL


class SqueezeWater(BaseMetaAction):
    def _build_actions(self):
        return [
            grip(self.engine, WASH_SQUEEZE, WASH_SQUEEZE_DWELL),  # ① 挤胶头出水
            grip(self.engine, GRIP_WASHBOT, 20),                  # ② 松开仍吸附
        ]
