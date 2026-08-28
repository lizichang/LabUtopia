"""元动作 ②：移液管尖端伸入样品瓶吸液（模拟挤压洗耳球吸满 5mL）。

尖端目标 z=0.815（液面 0.831 下 16mm、不触底 0.803）。抓点 z=0.905，夹爪始终在
瓶口(0.87)之上不撞瓶口。吸液到位 hold DRAW_HOLD 模拟吸满，task 在尖端入瓶口时显示
移液管内吸液柱。

轨迹（高位平移拆成两个单轴 mv，避免双轴对角单解 IK 把 TCP 拉成弧线穿模）：
  ① 高位 y 向（架列 → 瓶行，沿 x=架列避让中央天平）→ ② 高位 x 向（→瓶口正上方）
  → ③ 下探吸液 → ④ hold 吸液 → ⑤ 竖直提起回 H。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import (H, DRAW_HOLD, ORIENT_FWD, PIPE_XY, BOTTLE_XY,
                        BOTTLE_DRAW_TIP_Z, PIPE_TIP_TO_GRIP)


class DrawPipette(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        px, _ = PIPE_XY            # 移液管架 (0.22, 0.17)
        bx, by = BOTTLE_XY         # 样品瓶 (0.40, 0.00)
        draw_grasp_z = BOTTLE_DRAW_TIP_Z + PIPE_TIP_TO_GRIP
        return [
            mv(e, (px, by, H), orient=ORIENT_FWD),              # ① 高位 y 向：架→瓶行（避让天平）
            mv(e, (bx, by, H), orient=ORIENT_FWD),              # ② 高位 x 向：→瓶口正上方
            mv(e, (bx, by, draw_grasp_z), orient=ORIENT_FWD),   # ③ 下探：尖端入液
            hold(e, DRAW_HOLD),                                 # ④ 吸液（模拟挤压洗耳球）
            mv(e, (bx, by, H), orient=ORIENT_FWD),              # ⑤ 竖直提起
        ]
