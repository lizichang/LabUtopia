# -*- coding: utf-8 -*-
"""A2 ⑤ 拿试管（第二次吸附，为倒液）：高位接近 → 下探抓管口下 → 夹管 → 提出。

抓管口下 14mm（同 TubeShakePass），后接 PourToTube 旋转倒置。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, ORIENT_FWD, GRIP_TUBE, TUBE_XY, TUBE_GRASP_TCP


class PickTestTube(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),   # ① 高位接近
            mv(self.engine, TUBE_GRASP_TCP, orient=ORIENT_FWD),                # ② 下探抓管口下
            grip(self.engine, GRIP_TUBE, 60),                                  # ③ 夹试管
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),   # ④ 提出
        ]
