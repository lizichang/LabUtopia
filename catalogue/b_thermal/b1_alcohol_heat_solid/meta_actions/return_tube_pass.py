# -*- coding: utf-8 -*-
"""元动作：放回试管——火焰上方退开 → 回架上方 → 法兰转回竖直 → 下降放回架孔 → 松爪 → 抬走。

用户 2026-08-28 逐字：「最后放回试管」。PickTubePass 的逆过程（⑨⑧⑦⑥⑤ 反向）：
  ⑨' 水平 +Y 退开火焰：TUBE_AT_FLAME_2 (0.50,0.131,0.8982) → (0.50,0.241,0.8982)，只 y 变
  ⑧' 竖直上升 z→TUBE_HIGH 1.10，只 z 变（管底 0.961 清架顶 0.917）
  ⑦' 水平 +X 回架上方：(0.50,0.241,1.10) → (0.659,0.241,1.10)，只 x 变
  ⑥' 法兰转回竖直：FlangeRollTubeAction(angle=+95°)，joint7 ≈-95° → 0，试管近水平→竖直
  ⑤a' 强制竖直 + 精确对准孔心（用户 2026-08-28 逐字「现在放回有问题，没有对准试管架的孔，
        应该对准再竖直插下来」「你就不能先抬高对准再下降放吗」）：mv((TUBE_XY,TUBE_HIGH),
        orient=ORIENT_FWD, orient_eps=0.01)——**不再采样当前朝向**（会继承 ⑥' 法兰回滚的残余
        倾斜，试管斜插穿模），直接喂验证过的竖直朝向 ORIENT_FWD（拾管 ② 入孔同款），在高 z
        安全位（管底 0.961 清架顶 0.917）先把工具转正并对准孔心。**orient_eps=0.01（≈0.57°）：
        默认 ORIENT_EPS=0.15（8.6°）下 ⑤a' 一进 8.6° 范围即冻，残余倾斜根本没被转正、带进
        下降（用户「还是没有完全竖直，插入的时候有点偏向-y方向」）；收紧后 ⑤a' 真正转到
        <0.6° 竖直才冻（Ø19.2 管入 Ø22 孔需 <0.72°）**
  ⑤b' 竖直下降放回抓点：mv(TUBE_GRASP_TCP, orient=ORIENT_FWD, orient_eps=0.01)——仿 d3l
        放回 `mv(TUBE_GRASP_TCP)` plain MoveAction + ORIENT_FWD，linewalk 把 x-y 锁到**目标值**
        （孔心），逐帧重解 IK 也按紧阈值收敛朝向，试管竖直插下（管底 0.806 落架底），与拾管
        ② 同几何
  松爪 grip(GRIP_OPEN)：task 近抓点+开爪双条件 → tube released → 试管写回静置矩阵（架内竖插）
  抬走 mv((TUBE_XY, H))：空爪离开（松爪后试管已回 rest，白粉柱随管复位）

⑦⑧⑨ 已证伪 mv(orient=FLANGE_HOLD_ORIENT)（把试管甩回竖直）——全程序用
MovePreserveTubeAction 采样当前实际朝向 + MoveAction 单轴 linewalk，保持姿态不变。
⑤' 下降前插 ⑤a' 对准、⑤b' 下降都用 plain `mv(..., orient=ORIENT_FWD)`（仿 d3l 放回 =
`mv(TUBE_GRASP_TCP)`，B1 拾管 ② 同款朝向）：MovePreserveTubeAction 会把 x-y 掩码锁成
当前冻结值（1.5cm 内不修正）；**采样当前朝向会继承 ⑥' 法兰回滚的残余倾斜**——ORIENT_EPS
0.15rad=8.6° 容忍 + 近奇异区 IK 漂移，管长 13.9cm 底端摆幅可达 2.1cm > 孔半径 1.1cm →
斜插穿模（用户「法兰没有旋转平，试管没竖直就插」）。改用**验证过的竖直朝向 ORIENT_FWD**
（拾管 ② 入孔同款，`_T_HELD_TUBE·tool_world(ORIENT_FWD)`=试管竖直已 pxr 验证）+ MoveAction
**不掩码**——linewalk 时非变化轴锁到**目标值**（孔心）而非当前值，下降全程 x-y 严格 = 孔心、
试管竖直插入。矩阵持握 _T_HELD_TUBE 随夹爪 6-DOF，白粉柱随管刚性跟随。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD,
                        TUBE_XY, TUBE_GRASP_TCP, TUBE_HIGH, TUBE_AT_FLAME_2)
from .flange_roll_tube import FlangeRollTubeAction
from .move_x_preserve import MovePreserveTubeAction


class ReturnTubePass(BaseMetaAction):
    """火焰上方试管放回架内：+Y 退开 → 升 z → +X 回架 → 法兰转回竖直 → 对准孔心 → 竖直下降
    → 松爪 → 抬走。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            MovePreserveTubeAction(e, (TUBE_AT_FLAME_2[0], TUBE_XY[1], TUBE_AT_FLAME_2[2])),  # ⑨' +Y 0.131→0.241 退开火焰
            MovePreserveTubeAction(e, (TUBE_AT_FLAME_2[0], TUBE_XY[1], TUBE_HIGH)),           # ⑧' z 0.8982→1.10
            MovePreserveTubeAction(e, (TUBE_XY[0], TUBE_XY[1], TUBE_HIGH)),                   # ⑦' x 0.50→0.659 回架上方
            FlangeRollTubeAction(angle=95.0, dwell=15),                                       # ⑥' 法兰转回竖直（joint7 → 抓取值）
            mv(e, (tx, ty, TUBE_HIGH), orient=ORIENT_FWD, dwell=15, orient_eps=0.01),          # ⑤a' 强制竖直 + 精确对准孔心（高 z 安全位；紧朝向阈值 0.57° 才冻，防残余倾斜带进下降）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD, orient_eps=0.01),                         # ⑤b' 竖直下降入孔（linewalk 锁 x-y 孔心；紧朝向阈值，逐帧保持 <0.6° 竖直）
            grip(e, GRIP_OPEN, 30),                                                           # 松爪：task 近抓点+开爪 → released
            mv(e, (tx, ty, H), orient=ORIENT_FWD),                                            # 抬走（空爪离开）
        ]
