# -*- coding: utf-8 -*-
"""A2 ⑦ 放回试管（空）：倒液点转正+抬到 H → 经走廊横穿 → 放回架孔 → 松爪。

转正须在倒液点 z≥1.0（回旋最低点 TCP-0.1393 ≥0.86 > 加液口 0.830）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (
    H, ORIENT_FWD, GRIP_OPEN, TUBE_XY, TUBE_GRASP_TCP, CORRIDOR_Y, FILL_XY,
)


class ReturnTestTube(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (FILL_XY[0], FILL_XY[1], H), orient=ORIENT_FWD),   # ① 倒液点转正+抬 H
            mv(self.engine, (FILL_XY[0], CORRIDOR_Y, H), orient=ORIENT_FWD),   # ② 进净空走廊
            mv(self.engine, (TUBE_XY[0], CORRIDOR_Y, H), orient=ORIENT_FWD),   # ③ 横穿走廊
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),   # ④ 对准架孔
            mv(self.engine, TUBE_GRASP_TCP, orient=ORIENT_FWD),                # ⑤ 放回管口下
            grip(self.engine, GRIP_OPEN, 25),                                  # ⑥ 松爪
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),   # ⑦ 抬离
        ]
