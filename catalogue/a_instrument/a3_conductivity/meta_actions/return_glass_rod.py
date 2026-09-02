# -*- coding: utf-8 -*-
"""A3 ⑩ 搅拌完把玻璃棒放回试管架（2026-08-30 用户「加动作搅拌完放回去玻璃棒」）。

⑨ StirInBeaker 之后棒仍 attached 悬于烧杯口上方（TCP (0.412,0.0807,H=1.15)、棒底 0.95）。
本动作把棒放回试管架原位（PickGlassRod 的逆过程 + 开爪松放）：
  ① 水平移回试管架上方（TCP → (0.5434,0.1995,H)，z 锁 H 不变、xy 回移；夹爪保持 GRIP_ROD
     0.003 持握不松，棒随爪平移）
  ② 竖直下探到抓点高度（TCP → ROD_GRASP (0.5434,0.1995,1.00)，纯 z 降）——棒底 0.95→0.80
     = 台面顶 = rest，棒落回架孔
  ③ 开爪松放（grip GRIP_OPEN=0.04）：task 检测 opening > ROD_GRIP_OPEN(0.038) → released，
     棒回架位（棒底 0.80 = 表位，零跳变）
  ④ 抬回安全高位撤离（TCP → (0.5434,0.1995,H)），棒留在架内。

夹爪全程保持 GRIP_ROD 闭合直到 ③（grip_target 由 controller 从 StirInBeaker 传播，①②
无 grip 原子动作、首帧不开爪——工具已吸附类，ReturnWashBottle 同款）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, ORIENT_FWD, ROD_XY, ROD_GRASP


class ReturnGlassRod(BaseMetaAction):
    """把玻璃棒放回试管架：水平移回架上方 → 竖直下探 → 开爪松放 → 抬回撤离。"""

    def _build_actions(self):
        e = self.engine
        rx, ry = ROD_XY
        above = (rx, ry, H)
        return [
            mv(e, above, orient=ORIENT_FWD),    # ① 水平移回试管架上方
            mv(e, ROD_GRASP, orient=ORIENT_FWD),  # ② 竖直下探到抓点（棒底落回台面 0.80）
            grip(e, GRIP_OPEN, 40),             # ③ 开爪松放（task → released，棒回架位）
            mv(e, above, orient=ORIENT_FWD),    # ④ 抬回安全高位撤离（棒留架内）
        ]
