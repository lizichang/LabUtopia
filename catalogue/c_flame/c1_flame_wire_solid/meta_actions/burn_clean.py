"""元动作 ⑤：灼烧清洗铂丝（接近火焰 → 火焰中灼烧 → 冷却位）。

MoveAction 带 dwell：移到 FLAME_APPROACH/FLAME_HOLD 后冻结停留，贴合物理
（火焰灼烧不是掠过，需持续停留）。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv
from .constants import FLAME_APPROACH, FLAME_HOLD, COOL_POS


class BurnClean(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            mv(e, FLAME_APPROACH, 20),          # 接近火焰（dwell 20）
            mv(e, FLAME_HOLD, 60),              # 火焰中灼烧（dwell 60）
            mv(e, COOL_POS),                    # 冷却位
        ]
