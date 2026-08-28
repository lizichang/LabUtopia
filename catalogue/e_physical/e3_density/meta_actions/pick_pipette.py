"""元动作 ①：夹取移液管（竖直插架，抓点 = 尖端上方 0.09m）。

移液管 Ø7×185mm（v2 压缩）竖直插架孔（尖端在架底座顶 0.82），抓点 z=0.91（管身
中段、洗耳球底 0.96 下 5cm 可握管身段）。抓点取在管身上方是为了：吸液/放液时夹爪
始终在瓶口(0.87)/筒口(0.995)之上，尖端才能伸进而不让夹爪撞瓶口/筒口。

轨迹：① 高位 (0.22,0.17,H) → ② 下探抓点 (0.22,0.17,0.91) → ③ 合爪 GRIP_PIPETTE
→ ④ 竖直提起回 H（尖端离架孔）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_PIPETTE, ORIENT_FWD, PIPE_GRASP


class PickPipette(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        px, py, _ = PIPE_GRASP
        return [
            mv(e, (px, py, H), orient=ORIENT_FWD),   # ① 高位：管正上方
            mv(e, PIPE_GRASP, orient=ORIENT_FWD),     # ② 下探到抓点
            grip(e, GRIP_PIPETTE, 60),                # ③ 合爪夹管身
            mv(e, (px, py, H), orient=ORIENT_FWD),    # ④ 竖直提起
        ]
