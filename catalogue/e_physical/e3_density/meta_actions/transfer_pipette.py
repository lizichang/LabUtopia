"""元动作 ③：把移液管内 5mL 移液放液到量筒（模拟挤压洗耳球排出）。

尖端目标 z=0.95（伸进筒口 0.995 下 4.5cm、5mL 液面 0.921 上方）。放液到位 hold
DISPENSE_HOLD 模拟排出，task 在尖端入筒口时显示量筒液柱、隐藏移液管吸液柱，并把
天平屏读数从 m1 切到 m2+ρ。

轨迹（先抬升到高位、再平行移动到量筒正上方、再竖直下降，避免从瓶口斜着甩向筒口
穿模）：
  ① 先竖直抬升到高位（瓶口上方，冗余保险，确保起手已在 H）→ ② 平行移动到量筒
  正上方（仅 y 变，直线）→ ③ 竖直下降滴加 → ④ hold 放液 → ⑤ 竖直提起回 H。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import (H, DISPENSE_HOLD, ORIENT_FWD, BOTTLE_XY, CYL_XY,
                        CYL_DISPENSE_TIP_Z, PIPE_TIP_TO_GRIP)


class TransferPipette(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        bx, by = BOTTLE_XY          # 吸液后停在瓶口上方 (0.40, 0.00)
        cx, cy = CYL_XY             # 量筒在天平称盘上 (0.40, 0.17)
        disp_grasp_z = CYL_DISPENSE_TIP_Z + PIPE_TIP_TO_GRIP
        return [
            mv(e, (bx, by, H), orient=ORIENT_FWD),              # ① 先竖直抬升到高位
            mv(e, (cx, cy, H), orient=ORIENT_FWD),              # ② 平行移动到量筒正上方
            mv(e, (cx, cy, disp_grasp_z), orient=ORIENT_FWD),   # ③ 竖直下降：尖端入筒口
            hold(e, DISPENSE_HOLD),                             # ④ 放液（模拟挤压洗耳球）
            mv(e, (cx, cy, H), orient=ORIENT_FWD),              # ⑤ 竖直提起
        ]
