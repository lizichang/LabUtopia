"""元动作 ⑨：灼烧显色（火焰中烧 400 帧出黄色锥）。

铂丝蘸样品粉末后伸入火焰持续灼烧——task 端 _stain_fired 在铂丝环在火焰区
且带样品粉时触发染色锥。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv
from .constants import FLAME_APPROACH, FLAME_HOLD, COOL_POS


class BurnStain(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            mv(e, FLAME_APPROACH, 20),          # 接近火焰
            mv(e, FLAME_HOLD, 400),             # 火焰中灼烧显色
            mv(e, COOL_POS),                    # 冷却位
        ]
