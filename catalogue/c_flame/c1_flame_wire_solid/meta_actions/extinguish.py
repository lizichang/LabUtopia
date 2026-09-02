"""元动作 ⑩：灯帽盖灭（归架铂丝 → 取灯帽 → 盖灯口）。

修 bug3：灯帽从桌面 rest（CAP_GRASP 0.824）取，不是在空中比划；
盖灭在 CAP_BURNER 扣灯口后开爪释放，task 灭焰 + cap 锁灯口。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_CAP,
                        WIRE_GRASP, WIRE_XY, WIRE_LIFT,
                        CAP_GRASP, CAP_HIGH, CAP_BURNER)


class Extinguish(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy = WIRE_XY
        cx, cy, _ = CAP_GRASP
        bx, by, _ = CAP_BURNER
        return [
            # A 归架铂丝
            mv(e, (wx, wy, WIRE_LIFT)),
            mv(e, WIRE_GRASP),
            grip(e, GRIP_OPEN, 25),              # 释放铂丝
            mv(e, (wx, wy, H)),
            # B 取灯帽（从桌面，修 bug3）
            mv(e, (cx, cy, CAP_HIGH)),
            mv(e, CAP_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_CAP, 60),               # 修 bug1
            mv(e, (cx, cy, CAP_HIGH), 5),        # 垂直提起 + 停顿
            # C 盖灭
            mv(e, (bx, by, 1.00)),               # 移到灯上方
            mv(e, CAP_BURNER, 25),               # 扣灯口盖灭
            grip(e, GRIP_OPEN, 25),              # 释放灯帽
            mv(e, (bx, by, H)),                  # 归位
        ]
