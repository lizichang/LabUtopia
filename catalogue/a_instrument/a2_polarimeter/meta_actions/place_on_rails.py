# -*- coding: utf-8 -*-
"""A2 ⑧ 放旋光管上导轨：+X 到腔室上方 → -Y 对齐槽心 → 松爪（管落下导轨）→ 退向近侧。

导轨 tube_rails 顶 1.001、槽心 y=-0.03；管落座后中心 z=1.0075。管中心 = TCP-0.019（纯平移
持握），松爪时管在槽正上方，task 检测开爪后把管从当前高度动画「落下」到导轨（不再让机械臂
深下探进腔室）。

**2026-08-28 松爪落下改造（用户「绕过翻盖之后对齐了之后直接松开爪子，让旋光管落下去」）**：
旧 ③ 下探到 PTUBE_PLACE_TCP(1.0265) + ⑤ 抬回 (0.51,-0.24,1.20)——深 -y（导轨 y=-0.24）
近奇异区，② -y 横移到 z1.20 linewalk 分支翻转 force-done、⑤ 抬回单次 IK 报 IK FAIL（臂
卡住不动，用户「还是 ik 解不出来」）。改=横移/对齐高度降到 1.18（越过掀开翻盖自由边
1.1475 且 Lula 可达），到槽心上方即松爪，不再下探；管下落由 task 动画（`dropping` 态，
PTUBE_DROP_FRAMES 帧掉到 PTUBE_RAILS）。④ 退向近侧浅 y=-0.11（同 LID_APPROACH_HIGH 可达），
不再在深 y 抬升。

**不用显式 orient**（同 P1：纯平移持握 + 深 -y 处显式朝向 FK 检查解不出 IK，改用引擎默认
位置 IK 即可达）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_OPEN, PTUBE_PLACE_ABOVE


class PlaceOnRails(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (PTUBE_PLACE_ABOVE[0], 0.0, PTUBE_PLACE_ABOVE[2])),    # ① +X 到腔室上方
            mv(self.engine, PTUBE_PLACE_ABOVE),                                   # ② -Y 对齐槽心（高位 1.18）
            grip(self.engine, GRIP_OPEN, 25),                                     # ③ 松爪（task 让管落下导轨）
            mv(self.engine, (PTUBE_PLACE_ABOVE[0], -0.11, PTUBE_PLACE_ABOVE[2])), # ④ 退向近侧（浅 y 可达）
        ]
