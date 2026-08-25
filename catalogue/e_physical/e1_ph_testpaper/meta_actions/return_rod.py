"""元动作 ④：把玻璃棒放回试管架前排右孔。

放回时抓点仍是棒底上方 0.20m（z=1.006）：夹爪降到该处时，棒底恰回到插孔底 z=0.806，
开爪后棒落回原位。

轨迹：① 高位 (0.319,0.117,H) → ② 下探抓点 (0.319,0.117,1.006) → ③ 开爪 GRIP_OPEN
→ ④ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, ORIENT_FWD, ROD_GRASP


class ReturnRod(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        rx, ry, _ = ROD_GRASP
        return [
            mv(e, (rx, ry, H), orient=ORIENT_FWD),   # ① 高位：棒孔正上方
            mv(e, ROD_GRASP, orient=ORIENT_FWD),     # ② 下探到抓点（棒底回插孔底）
            grip(e, GRIP_OPEN, 60),                  # ③ 开爪放回
            mv(e, (rx, ry, H), orient=ORIENT_FWD),   # ④ 撤离
        ]
