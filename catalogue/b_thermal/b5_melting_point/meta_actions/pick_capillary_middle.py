# -*- coding: utf-8 -*-
"""PickCapillaryMiddle：震实放回桌面后，从毛细管**中部**水平夹起（矩阵持握，保持水平）。

用户 2026-09-01 逐字：「还是用毛细管沾油吧，就是放下毛细管后再从中间拿起毛细管这次就
不竖直了保持水平」——与旧 pick_capillary（夹端部拎起 pivot 摆转竖直）相反，这里夹中部、
全程保持**水平**（task 侧 _CapillaryHoldLifecycle 矩阵持握 _CAP_HELD 刚性跟随，非 pivot），
供后续移到油皿蘸油、贴温度计泡。

夹法同旧（手指朝下竖直夹 Ø1.5mm 杆身），夹点 = CAP_MID（封口端 0.40 + 半长 0.05）。
机械臂只纯竖直下探/提起，不旋转夹爪（水平由矩阵持握保证）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import SETTLE, GRIP_CAPILLARY, CAP_MID, CAP_HIGH


class PickCapillaryMiddle(BaseMetaAction):
    """从毛细管中部水平夹起：高位接近 → 下探夹中部 → 停顿 → 合爪 → 纯竖直提起（保持水平）。"""

    def _build_actions(self):
        e = self.engine
        gx, gy, _ = CAP_MID
        return [
            mv(e, (gx, gy, CAP_HIGH)),       # ① 高位接近（手指朝下）
            mv(e, CAP_MID),                  # ② 竖直下探夹中部（两指竖直夹 Ø1.5mm）
            hold(e, SETTLE),                 # ③ 停顿稳定
            grip(e, GRIP_CAPILLARY, 60),     # ④ 合爪夹住中部（task 检测 held）
            mv(e, (gx, gy, CAP_HIGH), 5),    # ⑤ 纯竖直提起（保持水平，供蘸油横移）
        ]
