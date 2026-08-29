# -*- coding: utf-8 -*-
"""A2 ⑨ 拨回翻盖：高位接近（越过掀开翻盖自由边）→ 下探推盖高度 → 合爪 → 水平向 -y
折叠拨回（task 联动 lid rotateY 120°→0° 闭合）→ 开爪 → 抬离。按启动键（⑩）前合上遮光盖。

几何（pxr 实测，Polarimeter (0.48,-0.24) rotZ+90）：lid 铰链沿世界+X 在机身近侧
  y−0.184 / z1.0505（过 lid 局部原点，轴沿世界±X）；掀 120° 板斜向上，自由边 y−0.128 /
  z1.1475，板面在 TCP z1.09 处 y−0.161；闭 0° 盖 x 0.383..0.643 / z 1.0505..1.0595。
  旧 ① 直接 (0.51,−0.15,1.10) 接近会撞掀开板（板面 z1.1094 > TCP1.10），且接近即锁存
  → 改 ① 高位 1.20 越过自由边（手指 1.173 清 1.1475）→ ② 下探 1.18（手指 1.153 贴自由边
  上方）→ ③ 向 −y 折叠（y 过 −0.15 触发 task 联动 rotateXYZ.Y 120°→0°，到 −0.24 必闭）。
  task 按夹爪 y 进度 −0.15→−0.22 联动。x 固定 0.51；y −0.24 深侧用引擎默认 orient。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (
    LID_APPROACH_HIGH, LID_PUSH_START, LID_PUSH_END, LID_LIFT, GRIP_LID, GRIP_OPEN,
)


class CloseLidPass(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, LID_APPROACH_HIGH),  # ① 高位接近（越过掀开翻盖自由边）
            mv(self.engine, LID_PUSH_START),     # ② 下探到推盖高度（手指贴自由边上方）
            grip(self.engine, GRIP_LID, 20),     # ③ 合爪（推盖姿态，指端贴板面）
            mv(self.engine, LID_PUSH_END, 50),   # ④ 水平向 −y 折叠拨回（task 联动 lid 闭合）
            grip(self.engine, GRIP_OPEN, 20),    # ⑤ 开爪
            mv(self.engine, LID_LIFT),           # ⑥ 抬离
        ]
