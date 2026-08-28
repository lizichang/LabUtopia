# -*- coding: utf-8 -*-
"""A2 ⑩ 拨回翻盖：接近掀开翻盖近侧（+y 触板近面）→ 合爪 → 水平向 -y 折叠拨回
（task 联动 lid rotateY 120°→0° 闭合）→ 开爪 → 抬回。按启动键（⑪）前合上遮光盖。

几何（pxr 实测，Polarimeter (0.48,-0.24) rotZ+90）：lid 铰链沿世界+X 在机身近侧
  y−0.184 / z1.0505（过 lid 局部原点，轴沿世界±X）；掀 120° 板斜向上，自由边 y−0.128 /
  z1.1475，板面在 TCP z1.09 处 y−0.161；闭 0° 盖 x 0.383..0.643 / z 1.0505..1.0595。
  推点 (0.51,−0.15,1.10)（触板近面，3D 0.79m）→ (0.51,−0.24,1.10)（向 −y 折叠，3D 0.82m）
  全部 ≤0.83m 可达（旧 x 向拨盖深侧 z 上限 0.83 够不到是根因）。task 按夹爪 y 进度
  −0.15→−0.22 联动 rotateXYZ.Y 120°→0°。x 固定 0.51；y −0.24 深侧用引擎默认 orient。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import LID_APPROACH, LID_PUSH_END, GRIP_LID, GRIP_OPEN


class CloseLidPass(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, LID_APPROACH),     # ① 接近推盖起点（近侧 +y 触板近面）
            grip(self.engine, GRIP_LID, 20),   # ② 合爪（推盖姿态，指端贴板面）
            mv(self.engine, LID_PUSH_END, 50), # ③ 水平向 −y 折叠拨回（task 联动 lid 闭合）
            grip(self.engine, GRIP_OPEN, 20),  # ④ 开爪
            mv(self.engine, LID_APPROACH),     # ⑤ 抬回
        ]
