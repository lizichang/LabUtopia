# -*- coding: utf-8 -*-
"""A2 ⑨ 放旋光管上导轨：+X 到腔室开口上方 → -Y 对槽心 → 垂直下探搭槽 → 松爪 → 抬离。

导轨 tube_rails 顶 1.001、槽心 y=-0.03；管搭槽上中心 z=1.0075（机械臂几乎不下探）。
管中心 = TCP-0.019（纯平移持握），TCP 到 1.0265 即落座；手指在槽内净空区（两轨
x±0.0205 之间），不碰轨。**不用显式 orient**（同 P1：纯平移持握 + 深 -y 处显式朝向
FK 检查解不出 IK，4 步全报 "IK FAIL" → 改用引擎默认位置 IK 即可达）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_OPEN, PTUBE_PLACE_ABOVE, PTUBE_PLACE_TCP


class PlaceOnRails(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (PTUBE_PLACE_ABOVE[0], 0.0, PTUBE_PLACE_ABOVE[2])),    # ① +X 到腔室上方
            mv(self.engine, (PTUBE_PLACE_ABOVE[0], PTUBE_PLACE_ABOVE[1], PTUBE_PLACE_ABOVE[2])),  # ② -Y 对槽心
            mv(self.engine, PTUBE_PLACE_TCP, 5),                                   # ③ 下探落座
            grip(self.engine, GRIP_OPEN, 25),                                      # ④ 松爪
            mv(self.engine, (PTUBE_PLACE_ABOVE[0], PTUBE_PLACE_ABOVE[1], PTUBE_PLACE_ABOVE[2])),  # ⑤ 抬离
        ]
