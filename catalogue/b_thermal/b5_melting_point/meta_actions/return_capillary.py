# -*- coding: utf-8 -*-
"""ReturnCapillaryToTable：蘸粉后放回桌面，毛细管倒成水平（松爪）。

用户 2026-08-31 逐字：「然后再毛细管放回桌面（有倒在桌面上变成水平的）」+ 2026-09-01 验收
「机械臂就只是直上直下不要其他变化」——弃 RotateHeldAction 边降边转，改 MovePreserveAction
纯竖直降回，管身绕夹点摆转竖直→水平交给 task 侧 pivot 持握（TCP 降到 SWING_THRESHOLD_Z 以下
自动摆回水平）。放回后毛细管回原位（封口端 0.1710、开口端 0.2710 水平躺桌面），粉留在开口端内壁，
等第二次夹开口端拎起震实。

轨迹（全程 MovePreserveAction 保持夹爪朝向不变，管身摆转由 task 驱动）：
  ① 竖直提起   MovePreserveAction((px,py,LIFT_HIGH))   # 从蘸粉位 DIP_SEALED_Z=0.907 提起
  ② 横移回原位 MovePreserveAction((gx,gy,LIFT_HIGH))   # 保持竖直朝向横移回封口端上方
  ③ 竖直降回   MovePreserveAction((gx,gy,GRASP_SEALED.z))  # TCP 降到 0.813 → task 摆回水平
  ④ 松爪       grip(GRIP_OPEN)                          # task 检测 release 写回原位（无跳变）
"""
from ._base import BaseMetaAction, grip, mv
from .move_preserve import MovePreserveAction
from .constants import (GRIP_OPEN, GRASP_SEALED, GRASP_OPEN, POWDER_XY, LIFT_HIGH,
                        CAP_MID, CAP_HIGH)


class ReturnCapillaryToTable(BaseMetaAction):
    """蘸粉后放回桌面：提起 → 横移回原位 → 竖直降回（task 摆回水平）→ 松爪。"""

    def _build_actions(self):
        e = self.engine
        px, py = POWDER_XY
        gx, gy, gz = GRASP_SEALED
        return [
            MovePreserveAction(e, (px, py, LIFT_HIGH)),        # ① 竖直提起
            MovePreserveAction(e, (gx, gy, LIFT_HIGH)),        # ② 横移回原位上方
            MovePreserveAction(e, (gx, gy, gz)),               # ③ 竖直降回桌面（task 摆回水平）
            grip(e, GRIP_OPEN, 60),                            # ④ 松爪放回
        ]


class ReturnCapillaryAfterTamp(BaseMetaAction):
    """⑥ 震实后放回桌面：从开口端夹点（GRASP_OPEN，震实已在高位 1.05）竖直降回 → 松爪。

    震实后毛细管已竖直（封口端朝下）在高位 1.05、夹点 xy=GRASP_OPEN，无需横移；纯竖直降回
    桌面（TCP 降到 SWING_THRESHOLD_Z 以下 task 摆回水平）后松爪，毛细管回原位水平躺桌面。
    """

    def _build_actions(self):
        e = self.engine
        gx, gy, gz = GRASP_OPEN
        return [
            MovePreserveAction(e, (gx, gy, LIFT_HIGH)),   # ① 保持高位（幂等，已在 1.05）
            MovePreserveAction(e, (gx, gy, gz)),          # ② 竖直降回桌面（task 摆回水平）
            grip(e, GRIP_OPEN, 60),                       # ③ 松爪放回
        ]


class ReturnCapillaryAfterOil(BaseMetaAction):
    """⑧' 蘸油后放回桌面：从油皿（中部水平矩阵持握）提回毛细管原位 → 松爪。

    蘸油后毛细管中部矩阵持握（task 侧 _CapillaryHoldLifecycle），夹点在油皿上方 (0.27,0.15,
    0.804)；纯平移提回毛细管静止位上方（CAP_MID，保持水平），竖直降回桌面，松爪放回
    （task 检测 release 写回原位，零跳变）。随后 ⑨' 再夹封口端拎起竖直贴泡。
    """

    def _build_actions(self):
        e = self.engine
        gx, gy, gz = CAP_MID
        return [
            mv(e, (gx, gy, CAP_HIGH)),   # ① 提回原位上方（水平矩阵持握，保持水平）
            mv(e, (gx, gy, gz)),         # ② 竖直降回桌面（夹点回 CAP_MID）
            grip(e, GRIP_OPEN, 60),      # ③ 松爪放回（task 检测 release 写回原位）
        ]
