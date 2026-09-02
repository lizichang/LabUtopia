# -*- coding: utf-8 -*-
"""A3 ⑪ 竖直提起导电率仪电极（2026-08-30 用户「竖直提起导电率仪的棒子」→「竖直提不是水平
横夹，要像 d3l 夹滴管一样」）。

⑩ ReturnGlassRod 之后玻璃棒已回架、夹爪空。本动作到导电率仪（/World/Meter）前**竖直**抓住
电极探头（/World/Meter/electrode，cap 顶部手柄 + rod 细杆 + blades 底片，Ø20×166 竖直立台面），
竖直提起——全程手指朝下（引擎默认朝向 = euler(0,π,0)，同 d3l 夹滴管/夹皿，**不传 orient**）：
  ① 高位 x 偏开 -X（ELECTRODE_APPROACH_X=0.285，cap -X 壁 0.3849 前 10cm；电缆从 cap 顶向
     +X 前下垂 → -X 侧干净、meter 壳在 y≤-0.068 后 → y=-0.018 处 -X 无碰撞）
  ② 竖直下探到 cap 高（x 偏开 -X，纯 z 降到 ELECTRODE_CAP_Z=0.9625；指端 0.9355 = cap 下段）
  ③ 水平移入抓点（锁 y/z，仅 +X 平移入 cap 上方；手指朝下、两指沿 ±Y 竖直夹 Ø20 cap）
  ④ 合爪夹 cap（GRIP_ELECTRODE=0.010，半开 10mm 贴 Ø20 cap）
  ⑤ 竖直提起（锁 x/y，仅 z 抬升 0.9625→ELECTRODE_LIFT_Z=1.10；blades 底 0.80→0.9375 清烧杯口）

朝向 = 引擎默认（手指朝下）全程，同 d3l 夹滴管（传 ORIENT_FWD 会变成水平横夹，正是用户要改掉的）。
task 侧：电极 rest → 近抓点+合爪 → attached（纯 z 平移持握，电极 prim 写 (0,0,lift_z)，电缆
DynamicCable 逐帧 update 跟随 cap 顶）→ 开爪 → released（回 rest lift=0，电缆回 rest）。
电极 prim 无 xform op（mesh 烘焙在 meter 局部系），meter 仅 rotZ90 → 局部 +z = 世界 +z，
竖直提 = 对电极 prim 写 (0,0,lift_z) 纯 z 平移。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (GRIP_ELECTRODE, ELECTRODE_XY,
                        ELECTRODE_CAP_Z, ELECTRODE_APPROACH_X, ELECTRODE_GRASP,
                        ELECTRODE_LIFT_Z)


class LiftElectrode(BaseMetaAction):
    """竖直提起导电率仪电极：-X 高位接近 → 下探 → +X 移入 cap 上方 → 竖直夹 cap → 提起。"""

    def _build_actions(self):
        e = self.engine
        ex, ey = ELECTRODE_XY
        return [
            mv(e, (ELECTRODE_APPROACH_X, ey, ELECTRODE_LIFT_Z)),        # ① 高位（x 偏开 -X 避 +X 电缆）
            mv(e, (ELECTRODE_APPROACH_X, ey, ELECTRODE_CAP_Z)),         # ② 竖直下探到 cap 高
            mv(e, ELECTRODE_GRASP),                                     # ③ 水平移入 cap 上方（仅 +X）
            grip(e, GRIP_ELECTRODE, 60),                                # ④ 合爪竖直夹 cap（手指朝下）
            mv(e, (ex, ey, ELECTRODE_LIFT_Z)),                          # ⑤ 竖直提起（中高位，清烧杯口）
        ]
