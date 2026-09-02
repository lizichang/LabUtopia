"""元动作 ⑥：反复蘸酸灼烧（蘸酸 → 灼烧）循环 3 次。

在 _build_actions 里展开成固定原子动作列表——序列是确定的，不是运行时循环，
因此 reset() 语义与其它元动作一致（从头重跑整串）。
"""
from ._base import BaseMetaAction, mv
from .constants import (H, ACID_DIP, FLAME_APPROACH, FLAME_HOLD, COOL_POS)


class RepeatDipBurn(BaseMetaAction):
    LOOPS = 3

    def _build_actions(self):
        e = self.engine
        ax, ay, _ = ACID_DIP
        actions = []
        for _ in range(self.LOOPS):
            actions += [
                mv(e, (ax, ay, H)),              # 移向酸面皿
                mv(e, ACID_DIP, 20),             # 蘸酸
                mv(e, (ax, ay, H)),              # 提出
                mv(e, FLAME_APPROACH, 20),       # 接近火焰
                mv(e, FLAME_HOLD, 60),           # 灼烧
                mv(e, COOL_POS),                 # 冷却位
            ]
        return actions
