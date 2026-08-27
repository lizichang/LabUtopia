# -*- coding: utf-8 -*-
"""A2 ④ 试管震荡溶解：抓试管 → 提出架顶 → 水平正弦震荡 → 稳定 → 放回 → 松爪。

d2s TubeShakePass 同款（9 步），cycles 由 config.shake_cycles 控制。
"""
from ._base import BaseMetaAction, mv, grip, shake
from .constants import (
    H, ORIENT_FWD, GRIP_TUBE, TUBE_XY, TUBE_GRASP_TCP, SHAKE_CENTER_TCP,
    SHAKE_AMPLITUDE, SHAKE_PERIOD, SHAKE_HOLD_FRAMES, GRIP_OPEN,
)


class TubeShakePass(BaseMetaAction):
    def __init__(self, engine, cycles=3, shake_center=None):
        # 必须在 super().__init__() 之前赋值：基类 __init__ 会立刻调 _build_actions()，
        # 而 _build_actions() 里引用了 self._center / self._cycles
        self._cycles = max(1, int(cycles))
        self._center = tuple(shake_center) if shake_center else tuple(SHAKE_CENTER_TCP)
        super().__init__(engine)

    def _build_actions(self):
        return [
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),          # ① 高位接近
            mv(self.engine, TUBE_GRASP_TCP, orient=ORIENT_FWD),                       # ② 下探抓管口下
            grip(self.engine, GRIP_TUBE, 60),                                         # ③ 夹试管
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], self._center[2]), 5, orient=ORIENT_FWD),  # ④ 提出架顶
            shake(self.engine, self._center, axis=(1, 0, 0), amplitude=SHAKE_AMPLITUDE,
                  cycles=self._cycles, period=SHAKE_PERIOD, orient=ORIENT_FWD),        # ⑤ 水平震荡
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], self._center[2]), SHAKE_HOLD_FRAMES, orient=ORIENT_FWD),  # ⑥ 停稳
            mv(self.engine, TUBE_GRASP_TCP, orient=ORIENT_FWD),                       # ⑦ 放回管口下
            grip(self.engine, GRIP_OPEN, 25),                                         # ⑧ 松爪
            mv(self.engine, (TUBE_XY[0], TUBE_XY[1], H), orient=ORIENT_FWD),          # ⑨ 抬离
        ]
