"""元动作 ②：滴盐酸 3 滴（取滴管 → 吸液 → 滴皿 → 归架 → 盖回瓶塞）。

修 bug2：滴管取走后先垂直提出架再平移。
修 bug5：E 段从桌面真实塞位(STO_DESK 0.810)再抓起，不再空爪回瓶口瞬移。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_STOPPER, GRIP_DROPPER,
                        STO_GRASP, STO_DESK, DROP_GRASP, DROP_XY, DROP_LIFT,
                        HCL_DIP, DISH_DRIP)


class DripHclAcid(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        dx, dy, _ = DROP_GRASP
        hx, hy, _ = STO_GRASP
        ddx, ddy, _ = DISH_DRIP
        return [
            # A 取滴管（修 bug2：垂直提出架再平移）
            mv(e, (dx, dy, H)),
            mv(e, DROP_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_DROPPER, 60),          # 修 bug1
            mv(e, (dx, dy, DROP_LIFT), 15),     # 垂直提出架 + 停顿
            # B 吸液
            mv(e, (hx, hy, H)),
            mv(e, (hx, hy, 0.93)),
            mv(e, HCL_DIP, 25),                 # 下探吸液
            mv(e, (hx, hy, H), 5),              # 修 bug6：垂直提出瓶口再平移（嘴 1.031 > 瓶口 0.877）
            # C 滴液（task 每 30 帧滴 1 滴 ×3）
            mv(e, (ddx, ddy, H)),
            mv(e, DISH_DRIP, 200),
            # D 归架
            mv(e, (dx, dy, DROP_LIFT)),
            mv(e, DROP_GRASP),
            grip(e, GRIP_OPEN, 25),
            mv(e, (dx, dy, H)),
            # E 盖回瓶塞（从桌面真实取起，修 bug5）
            mv(e, (STO_DESK[0], STO_DESK[1], H)),
            mv(e, STO_DESK),
            hold(e, SETTLE),
            grip(e, GRIP_STOPPER, 60),          # 真正从桌面抓起
            mv(e, (STO_DESK[0], STO_DESK[1], 0.93), 5),
            mv(e, (hx, hy, 0.93)),
            mv(e, STO_GRASP),                   # 盖回瓶口
            grip(e, GRIP_OPEN, 25),
            mv(e, (hx, hy, H)),
        ]
