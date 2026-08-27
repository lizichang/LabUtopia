# -*- coding: utf-8 -*-
"""A2 ① 拿洗瓶：高位接近 → 直落瓶肚 → 横夹 → 提出 → 送红嘴到试管口。

d2s PickWashBottle 同款（横夹肚子 ORIENT_FWD），洗瓶在 A2 左前角 (-0.10,0.60)
rotZ-180 红嘴朝 +X；直落瓶中心（手指沿 ±Y 张开跨瓶身两侧，无碰撞）省掉偏移接近。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (
    H, ORIENT_FWD, GRIP_WASHBOT, WASH_GRASP, WASH_XY, WASH_LIFT,
    WASH_TO_TUBE_X, WASH_TO_TUBE_Y,
)


class PickWashBottle(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (WASH_XY[0], WASH_XY[1], H), orient=ORIENT_FWD),   # ① 高位接近瓶上方
            mv(self.engine, WASH_GRASP, orient=ORIENT_FWD),                    # ② 直落夹瓶肚
            grip(self.engine, GRIP_WASHBOT, 60),                               # ③ 横夹肚子
            mv(self.engine, (WASH_XY[0], WASH_XY[1], WASH_LIFT), orient=ORIENT_FWD),  # ④ 提出 15cm
            mv(self.engine, (WASH_TO_TUBE_X, WASH_XY[1], WASH_LIFT), orient=ORIENT_FWD),  # ⑤ +X 送红嘴到管口 x
            mv(self.engine, (WASH_TO_TUBE_X, WASH_TO_TUBE_Y, WASH_LIFT), orient=ORIENT_FWD),  # ⑥ -Y 对管口
        ]
