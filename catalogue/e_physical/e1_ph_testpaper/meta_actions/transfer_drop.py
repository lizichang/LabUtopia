"""元动作 ③：把玻璃棒尖端液滴点触到试纸中央 → 原地等待显色。

棒尖（底）目标 z=0.8070（试纸顶面）。点触到位 settle 后原地 hold WAIT_FRAMES（2.5s）
等待显色（task 在棒尖贴近纸中央时按 cfg.ph_result 显示对应色斑变体）。

轨迹：① 高位 (0.46,0.32,H) → ② 下探点触抓点 (0.46,0.32,1.0070) → ③ settle →
④ hold WAIT_FRAMES 等待显色 → ⑤ 竖直提起回 H。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import H, SETTLE, WAIT_FRAMES, ORIENT_FWD, PLATE_XY, TRANSFER_GRASP_Z


class TransferDrop(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        px, py = PLATE_XY
        return [
            mv(e, (px, py, H), orient=ORIENT_FWD),                 # ① 高位：试纸中央上方
            mv(e, (px, py, TRANSFER_GRASP_Z), orient=ORIENT_FWD),  # ② 下探：棒尖触纸
            hold(e, SETTLE),                                       # ③ 点触 settle
            hold(e, WAIT_FRAMES),                                  # ④ 等待显色 2.5s
            mv(e, (px, py, H), orient=ORIENT_FWD),                 # ⑤ 竖直提起
        ]
