# -*- coding: utf-8 -*-
"""DipCapillaryIntoPowder：拎起竖直后（开口端朝下），水平移到粉丘上方再竖直下探蘸粉。

用户 2026-08-30「然后竖着插进粉丘里面」+ 2026-08-31 新方法（夹封口端拎起自动竖直后）：
夹封口端拎起后毛细管竖直、开口端朝下，水平移到粉丘正上方再竖直下探，让开口端沉入粉丘
5mm 蘸粉（同 d2s POWDER_Z 概念：勺尖沉入粉丘 5mm）。

轨迹（MovePreserveAction 全程保持「手指朝下、夹爪不旋转」朝向，管身竖直由 task 侧 pivot 持握）：
  ① 水平移到粉丘上方 mv((POWDER_XY, LIFT_HIGH))    # 拎起后夹点已在高位 1.05，仅 x/y 变 → 横移
  ② 竖直下探入粉   mv((POWDER_XY, DIP_SEALED_Z), dwell)  # 开口端 = 夹点 − 0.098 = 0.809

几何：夹封口端拎起后开口端在夹点正下方 END_OFFSET=0.098，夹点 DIP_SEALED_Z=0.907 → 开口端
0.809 = 粉丘顶 0.814 − 5mm（沉入）。横移全程夹点 z=1.05、开口端 0.952，高于提勒管口 1.078
与试管架，无碰撞。
"""
from ._base import BaseMetaAction
from .move_preserve import MovePreserveAction
from .constants import POWDER_XY, LIFT_HIGH, DIP_SEALED_Z


class DipCapillaryIntoPowder(BaseMetaAction):
    """拎起竖直后：水平移到粉丘上方 → 竖直下探开口端入粉丘蘸粉。"""

    def _build_actions(self):
        e = self.engine
        px, py = POWDER_XY
        return [
            MovePreserveAction(e, (px, py, LIFT_HIGH)),             # ① 水平移到粉丘上方（保持竖直朝向）
            MovePreserveAction(e, (px, py, DIP_SEALED_Z), dwell=20),  # ② 竖直下探开口端入粉
        ]
