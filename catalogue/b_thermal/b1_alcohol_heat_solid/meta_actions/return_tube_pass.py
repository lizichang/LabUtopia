# -*- coding: utf-8 -*-
"""元动作：放回试管——火焰上方退开 → 回架上方 → 法兰转回竖直 → 下降放回架孔 → 松爪 → 抬走。

用户 2026-08-28 逐字：「最后放回试管」。PickTubePass 的逆过程（⑨⑧⑦⑥⑤ 反向）：
  ⑨' 水平 +Y 退开火焰：TUBE_AT_FLAME_2 (0.50,0.131,0.8982) → (0.50,0.241,0.8982)，只 y 变
  ⑧' 竖直上升 z→TUBE_HIGH 1.10，只 z 变（管底 0.961 清架顶 0.917）
  ⑦' 水平 +X 回架上方：(0.50,0.241,1.10) → (0.659,0.241,1.10)，只 x 变
  ⑥' 法兰转回竖直：FlangeRollTubeAction(angle=+95°)，joint7 ≈-95° → 0，试管近水平→竖直
  ⑤' 竖直下降放回抓点：MovePreserveTubeAction(TUBE_GRASP_TCP)，只 z 变（管底 0.806 落架底）
  松爪 grip(GRIP_OPEN)：task 近抓点+开爪双条件 → tube released → 试管写回静置矩阵（架内竖插）
  抬走 mv((TUBE_XY, H))：空爪离开（松爪后试管已回 rest，白粉柱随管复位）

⑦⑧⑨ 已证伪 mv(orient=FLANGE_HOLD_ORIENT)（把试管甩回竖直）——全程序用
MovePreserveTubeAction 采样当前实际朝向 + MoveAction 单轴 linewalk，保持姿态不变。
矩阵持握 _T_HELD_TUBE 随夹爪 6-DOF，白粉柱随管刚性跟随。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD,
                        TUBE_XY, TUBE_GRASP_TCP, TUBE_HIGH, TUBE_AT_FLAME_2)
from .flange_roll_tube import FlangeRollTubeAction
from .move_x_preserve import MovePreserveTubeAction


class ReturnTubePass(BaseMetaAction):
    """火焰上方试管放回架内：+Y 退开 → 升 z → +X 回架 → 法兰转回竖直 → 下降 → 松爪 → 抬走。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            MovePreserveTubeAction(e, (TUBE_AT_FLAME_2[0], TUBE_XY[1], TUBE_AT_FLAME_2[2])),  # ⑨' +Y 0.131→0.241 退开火焰
            MovePreserveTubeAction(e, (TUBE_AT_FLAME_2[0], TUBE_XY[1], TUBE_HIGH)),           # ⑧' z 0.8982→1.10
            MovePreserveTubeAction(e, (TUBE_XY[0], TUBE_XY[1], TUBE_HIGH)),                   # ⑦' x 0.50→0.659 回架上方
            FlangeRollTubeAction(angle=95.0, dwell=15),                                       # ⑥' 法兰转回竖直（joint7 -95°→0）
            MovePreserveTubeAction(e, TUBE_GRASP_TCP, dwell=15),                              # ⑤' 下降放回抓点（管底0.806落架底）
            grip(e, GRIP_OPEN, 30),                                                           # 松爪：task 近抓点+开爪 → released
            mv(e, (tx, ty, H), orient=ORIENT_FWD),                                            # 抬走（空爪离开）
        ]
