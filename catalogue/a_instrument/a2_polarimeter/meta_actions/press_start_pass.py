# -*- coding: utf-8 -*-
"""A2 ⑩ 按启动键：高位接近 → 下探预按位 → 合爪夹键侧 → 按下（task 触发测量）→ 开爪 → 抬回。

a1 PressStartPass 同款 6 步，按钮 Ø64 顶世界 1.056。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import BTN_APPROACH, BTN_PREPRESS, BTN_PRESS, GRIP_BUTTON, GRIP_OPEN


class PressStartPass(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, BTN_APPROACH),     # ① 高位接近按钮上方
            mv(self.engine, BTN_PREPRESS),     # ② 下探预按位
            grip(self.engine, GRIP_BUTTON, 30),  # ③ 合爪夹按钮两侧
            mv(self.engine, BTN_PRESS, 30),    # ④ 按下按钮顶（task 触发测量）
            grip(self.engine, GRIP_OPEN, 20),  # ⑤ 开爪松手
            mv(self.engine, BTN_APPROACH),     # ⑥ 抬起回高位
        ]
