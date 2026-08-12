"""元动作 ④：铂丝蘸酸（取铂丝 → 蘸盐酸 → 提出）。

修 bug1：取铂丝用原地 GripAction。
修 bug2：取走后先垂直提出试管架（WIRE_LIFT=1.12 > 架顶 0.917）再平移。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_WIRE,
                        WIRE_GRASP, WIRE_XY, WIRE_LIFT, ACID_DIP)


class DipWireAcid(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy = WIRE_XY
        ax, ay, _ = ACID_DIP
        return [
            mv(e, (wx, wy, H)),
            mv(e, WIRE_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_WIRE, 60),             # 修 bug1
            mv(e, (wx, wy, WIRE_LIFT), 15),     # 垂直提出架（修 bug2）
            mv(e, (ax, ay, H)),
            mv(e, ACID_DIP, 25),                # 蘸酸
            mv(e, (ax, ay, H)),                 # 提出
        ]
