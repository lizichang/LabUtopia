"""元动作 ⑧：蘸待测粉末（归架铂丝 → 开样品瓶塞 → 再取铂丝 → 蘸粉）。

开样品瓶塞与 ① 同形态（瓶口合爪 → 提出 → 倒放桌面，修 bug1/bug2）。
蘸粉点 POWDER_DIP 在开着的样品瓶口上方，垂直下探后停留。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_STOPPER, GRIP_WIRE,
                        SSTO_GRASP, SSTO_SIDE, WIRE_GRASP, WIRE_XY, WIRE_LIFT,
                        POWDER_DIP)


class DipPowder(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy = WIRE_XY
        sx, sy, _ = SSTO_GRASP
        return [
            # A 归架铂丝
            mv(e, (wx, wy, WIRE_LIFT)),
            mv(e, WIRE_GRASP),
            grip(e, GRIP_OPEN, 25),              # 释放铂丝
            mv(e, (wx, wy, H)),
            # B 开样品瓶塞
            mv(e, (sx, sy, H)),
            mv(e, SSTO_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_STOPPER, 60),           # 修 bug1
            mv(e, (sx, sy, 0.93), 5),            # 垂直提出 + 停顿
            mv(e, (SSTO_SIDE[0], SSTO_SIDE[1], 0.93)),
            mv(e, SSTO_SIDE),                    # 落座桌面
            grip(e, GRIP_OPEN, 25),
            mv(e, (SSTO_SIDE[0], SSTO_SIDE[1], H)),
            # C 再取铂丝
            mv(e, (wx, wy, H)),
            mv(e, WIRE_GRASP),
            hold(e, SETTLE),
            grip(e, GRIP_WIRE, 60),              # 修 bug1
            mv(e, (wx, wy, WIRE_LIFT), 15),      # 垂直提出架（修 bug2）
            # D 蘸粉末
            mv(e, (sx, sy, H)),
            mv(e, (sx, sy, 1.03)),               # 下探到瓶口上方
            mv(e, POWDER_DIP, 20),               # 蘸粉
            mv(e, (sx, sy, 1.03)),               # 提出
            mv(e, (sx, sy, H)),
        ]
