"""元动作 ⑦：冷却铂丝 5 秒（停冷却位 300 帧）。

铂丝在上一元动作末尾已回到 COOL_POS，这里保持冻结 300 帧（冷却 = 停留，
不是继续移动）。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import COOL_POS


class Cool(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            mv(e, COOL_POS),                     # 确保在冷却位
            hold(e, 300),                        # 冷却 5 秒（300 帧 @60Hz）
        ]
