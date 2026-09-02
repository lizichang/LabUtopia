"""元动作 ③：点燃酒精灯（取火柴 → 触灯芯 → 放回火柴）。

修 bug1：取火柴用原地 GripAction，避免低 z 抓点驱动 IK 追位置。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_MATCH,
                        MATCH_GRASP, MATCH_HIGH, IGNITE)


class IgniteLamp(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        return [
            mv(e, (mx, my, MATCH_HIGH)),
            mv(e, MATCH_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_MATCH, 60),            # 修 bug1
            mv(e, (mx, my, 0.90), 5),           # 提出 + 停顿
            mv(e, IGNITE, 20),                  # 触灯芯点燃
            mv(e, (mx, my, 0.90)),
            grip(e, GRIP_OPEN, 25),             # 安全高度释放
            mv(e, (mx, my, MATCH_HIGH)),
        ]
