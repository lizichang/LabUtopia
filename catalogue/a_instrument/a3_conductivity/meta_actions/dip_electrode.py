# -*- coding: utf-8 -*-
"""A3 ⑫ 把电极移到烧杯上方 + 下降深入（2026-08-30 用户「加动作把仪器移动到烧杯上方然后
下降深入」）。

⑪ LiftElectrode 之后电极已竖直提起悬于导电率仪上方（TCP (0.3949,-0.018,ELECTRODE_LIFT_Z=1.10)、
blades 底 0.9375、仍 attached 跟随）。本动作把电极水平移到烧杯口正上方，再竖直下降使电极
底片（blades）浸入烧杯内的液体：
  ① 水平横移到烧杯口正上方（TCP → ELECTRODE_ABOVE (0.412,0.0807,1.10)；锁 z、xy 平移，
     blades 0.9375 全程清烧杯口 0.8904，电缆跟随不穿模）
  ② 竖直下降深入（TCP → ELECTRODE_DIP (0.412,0.0807,0.98)；blades 底 0.80→0.8175 浸入液面
     0.84 下 2.25cm、离杯底 0.80 约 1.75cm；电极 Ø20 在烧杯内径 Ø60 内居中，两侧净空 2cm）

朝向 = 引擎默认（手指朝下）全程，与 LiftElectrode 连续（不传 orient）。
task 侧：电极 attached 期间 3-DOF 平移跟随（electrode prim 写 (wy,−wx,wz) 局部平移 + 电缆
DynamicCable 逐帧 update cap 顶），本动作只发 mv 即可、无需 task 改动。
"""
from ._base import BaseMetaAction, mv
from .constants import ELECTRODE_ABOVE, ELECTRODE_DIP


class DipElectrodeIntoBeaker(BaseMetaAction):
    """把电极移到烧杯口上方 + 竖直下降深入：水平横移 → 竖直下降。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, ELECTRODE_ABOVE),   # ① 水平移到烧杯口正上方（blades 清烧杯口）
            mv(e, ELECTRODE_DIP),     # ② 竖直下降，blades 浸入液面下
        ]
