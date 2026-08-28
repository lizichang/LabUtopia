# -*- coding: utf-8 -*-
"""A2 ⑧ 拿旋光管：高位接近（引擎默认朝向手指朝下、指隙沿 X）→ 下探管身 → 横夹 → 提出。

管身 Ø13 横放在桌面 (0.20,0.3607,0.811)（试管架 y 对齐偏 -x），泡在 +y、加液口朝上；
抓管身中段 y=0.3607（避开泡/加液口）。**不用显式 orient**：旋光管是纯平移持握（set_object_position，
管朝向不随夹爪转），显式 ORIENT_TUBE_GRAB 的 FK 朝向检查在低 z 处解不出 IK（报
"IK FAIL … force-done"→ 永不 attach）；改用 orient=None = 引擎默认（位置 IK + 正确
朝向），无朝向检查，低 z 也解得开。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_PTUBE, PTUBE_APPROACH, PTUBE_GRASP, PTUBE_LIFT


class PickPolarimeterTube(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, PTUBE_APPROACH),   # ① 高位接近管上方
            mv(self.engine, PTUBE_GRASP, 5),    # ② 下探夹管身
            grip(self.engine, GRIP_PTUBE, 60),  # ③ 横夹管身
            mv(self.engine, PTUBE_LIFT),        # ④ 提出 32cm 净空横移
        ]
