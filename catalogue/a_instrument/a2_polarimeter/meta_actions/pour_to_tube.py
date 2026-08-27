# -*- coding: utf-8 -*-
"""A2 ⑥ 倒液进旋光管：横移净空走廊 → 旋转倒置 → 对准加液口 → 下探倒液 → 提起。

试管 ORIENT_POUR 倒置（管口朝下），经 CORRIDOR_Y=0.40 净空走廊（旋光管已搬仪右前方，
走廊只剩 0.1m）；倒液点 = 旋光管加液口 (0.70,0.3325,0.830)，管口沉到 0.846（口上 16mm）
静置 POUR_HOLD。
"""
from ._base import BaseMetaAction, mv
from .constants import (
    H, ORIENT_FWD, ORIENT_POUR, TUBE_XY, CORRIDOR_Y, FILL_XY,
    POUR_APPROACH, POUR_TCP, POUR_HOLD, POUR_LIFT,
)


class PourToTube(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (TUBE_XY[0], CORRIDOR_Y, H), orient=ORIENT_FWD),       # ① 进净空走廊
            mv(self.engine, (FILL_XY[0], CORRIDOR_Y, H), orient=ORIENT_POUR),      # ② 横穿走廊，旋倒置
            mv(self.engine, POUR_APPROACH, orient=ORIENT_POUR),                    # ③ 对准加液口上方
            mv(self.engine, POUR_TCP, POUR_HOLD, orient=ORIENT_POUR),              # ④ 下探倒液
            mv(self.engine, POUR_LIFT, orient=ORIENT_POUR),                        # ⑤ 提起（仍倒置，供转正）
        ]
