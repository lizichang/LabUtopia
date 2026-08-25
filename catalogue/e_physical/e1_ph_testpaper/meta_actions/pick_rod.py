"""元动作 ①：夹取玻璃棒（竖直插架前排右孔）。

玻璃棒 Ø6×261mm 竖直插架，抓点 = 棒底（尖端）上方 0.20m（z=1.006，架顶 0.917 上 8.9cm
可握段）。抓点取在尖端上方 0.20m 是为了：蘸取时夹爪保持管口(0.9593)之上，棒尖才能伸进
试管底部而不让夹爪撞管口。

轨迹：① 高位 (0.319,0.117,H) → ② 下探抓点 (0.319,0.117,1.006) → ③ 合爪 GRIP_ROD
→ ④ 竖直提起回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_ROD, ORIENT_FWD, ROD_GRASP


class PickRod(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        rx, ry, _ = ROD_GRASP
        return [
            mv(e, (rx, ry, H), orient=ORIENT_FWD),   # ① 高位：棒正上方
            mv(e, ROD_GRASP, orient=ORIENT_FWD),     # ② 下探到抓点
            grip(e, GRIP_ROD, 60),                   # ③ 合爪夹住杆壁
            mv(e, (rx, ry, H), orient=ORIENT_FWD),   # ④ 竖直提起
        ]
