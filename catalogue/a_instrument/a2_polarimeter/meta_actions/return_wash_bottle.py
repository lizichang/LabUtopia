# -*- coding: utf-8 -*-
"""A2 ③ 放回洗瓶：+Y 退回 → -X 回原位 → 降到抓高 → 松爪 → 抬离。

d2s ReturnWashBottle 同款逆序。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, ORIENT_FWD, WASH_XY, WASH_GRASP_Z, WASH_LIFT, WASH_TO_TUBE_X, WASH_TO_TUBE_Y


class ReturnWashBottle(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (WASH_TO_TUBE_X, WASH_XY[1], WASH_LIFT), orient=ORIENT_FWD),   # ① +Y 退回
            mv(self.engine, (WASH_XY[0], WASH_XY[1], WASH_LIFT), orient=ORIENT_FWD),       # ② -X 回原位
            mv(self.engine, (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z), orient=ORIENT_FWD),    # ③ 降到抓高
            grip(self.engine, GRIP_OPEN, 25),                                             # ④ 松爪放回
            mv(self.engine, (WASH_XY[0], WASH_XY[1], H), orient=ORIENT_FWD),               # ⑤ 抬离
        ]
