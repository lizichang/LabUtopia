"""元动作 ①：从试管架夹取试管，垂直提出后放到操作位。

v11 D2-S 步骤 1：
「夹具垂直下放至试管架中目标试管正上方对齐并夹持，竖直向上抬升至垂直抬升
 安全中转点（试管底部完全高出试管架其余管口与台面障碍物），保持抬升高度水平
 平移至操作位正上方完成水平对齐中转点，再竖直缓慢下放至操作位后松开夹具」

垂直段（下探/提出）由 MoveAction 的垂直约束保证（起点 xy≈目标 xy 时 xy 锁死、
z 每帧 VZ_STEP=0.002 推进，TCP 走直线），防斜着拿试管穿架——v46 教训。
试管 attach 由 task 侧判定（近窗门禁 + grip 阈值），本元动作只发运动 + 夹爪。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_TUBE,
                        TUBE_XY, TUBE_GRASP_Z, OP_XY, OP_DROP_Z)


class TakeTestTube(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        ox, oy = OP_XY
        return [
            mv(e, (tx, ty, H)),                  # ① 高位接近试管正上方
            mv(e, (tx, ty, TUBE_GRASP_Z)),       # ② 垂直下探到管口抓点
            hold(e, SETTLE),                     # ③ 停稳（attach 近窗）
            grip(e, GRIP_TUBE, 60),              # ④ 原地合爪 attach（不驱动 IK）
            mv(e, (tx, ty, H), 5),               # ⑤ 垂直提出（管底高出架顶 0.914）
            mv(e, (ox, oy, H)),                  # ⑥ 高位平移至操作位上方
            mv(e, (ox, oy, OP_DROP_Z)),          # ⑦ 垂直下放操作位
            grip(e, GRIP_OPEN, 25),              # ⑧ 松开释放（task 落座）
            mv(e, (ox, oy, H)),                  # ⑨ 归位
        ]
