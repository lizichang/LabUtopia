"""元动作 ①：开稀盐酸瓶塞（瓶口合爪 → 垂直提出 → 倒放桌面）。

修 bug1：合爪用原地 GripAction（不驱动 IK），瓶塞真被抓起。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_STOPPER,
                        STO_GRASP, STO_SIDE)


class OpenHclStopper(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy, _ = STO_GRASP
        return [
            mv(e, (sx, sy, H)),                      # 高位接近瓶口
            mv(e, STO_GRASP),                        # 下探瓶口抓点
            hold(e, SETTLE),
            grip(e, GRIP_STOPPER, 60),               # 原地合爪（修 bug1）
            mv(e, (sx, sy, 0.93), 5),                # 垂直提出 + 停顿
            mv(e, (STO_SIDE[0], STO_SIDE[1], 0.93)), # 平移旁侧
            mv(e, STO_SIDE),                         # 落座（task 放桌面）
            grip(e, GRIP_OPEN, 25),                  # 开爪释放
            mv(e, (STO_SIDE[0], STO_SIDE[1], H)),    # 归位
        ]
