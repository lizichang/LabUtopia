# -*- coding: utf-8 -*-
"""A3 ② 夹着玻璃皿水平移动到烧杯口正上方（2026-08-28 用户，A3 第二个动作）。

PickSurfaceDish 之后皿已 attached 悬空（TCP 停在 DISH_LIFT (0.3442,0.5550,1.15)），本动作
把皿水平横移到样品烧杯口正上方（TCP → DISH_ABOVE_BEAKER (0.392,0.0807,1.15)）：纯 y 横移
0.47m、z 不变（=H），皿保持平放、随 TCP 纯平移（task attached 状态跟随）。夹爪全程保持
GRIP_DISH 闭合——grip_target 由 controller 从 PickSurfaceDish 传播，本动作无 grip 原子动作、
首帧不开爪（工具已吸附类，dip 铂丝同款）。

烧杯口中心几何（见 constants.py）：2026-08-29 用户改 beaker.usd 直立烧杯（内建
rotateXYZ(-135,0,0)），口朝上正对上方，口顶视图中心 = (0.3920,0.0807)（此前的侧躺对齐点
全部作废）。DISH_ABOVE_BEAKER = (BEAKER_MOUTH_XY, H)，后续倒粉动作再下降 + 倾斜。
"""
from ._base import BaseMetaAction, mv
from .constants import DISH_ABOVE_BEAKER


class MoveDishAboveBeaker(BaseMetaAction):
    """夹着玻璃皿水平移动到烧杯口正上方。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, DISH_ABOVE_BEAKER),   # ① 水平横移到烧杯口上方（z 不变 H，皿随夹爪）
        ]
