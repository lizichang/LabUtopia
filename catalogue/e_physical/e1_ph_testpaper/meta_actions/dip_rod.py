"""元动作 ②：玻璃棒尖端伸入试管蘸取待测溶液。

棒尖（底）目标 z=0.808（液面 0.811 下 3mm、不触底 0.805）。抓点 z=1.008 = 棒尖 + 0.20，
夹爪始终在管口(0.9593)之上不撞管口。蘸取到位 settle 后提起。

轨迹：① 高位 (0.2787,0.1193,H) → ② 下探蘸取抓点 (0.2787,0.1193,1.008) → ③ settle
→ ④ 竖直提起回 H。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import H, SETTLE, ORIENT_FWD, TUBE_XY, DIP_GRASP_Z


class DipRod(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, (tx, ty, H), orient=ORIENT_FWD),               # ① 高位：试管口正上方
            mv(e, (tx, ty, DIP_GRASP_Z), orient=ORIENT_FWD),     # ② 下探：棒尖入液
            hold(e, SETTLE),                                     # ③ 蘸取 settle
            mv(e, (tx, ty, H), orient=ORIENT_FWD),               # ④ 竖直提起
        ]
