# -*- coding: utf-8 -*-
"""A3 ⑨ 下降玻璃棒插入烧杯并在杯内搅拌（2026-08-30 用户「现在下降玻璃棒并在烧杯里面搅拌」）。

⑧ PickGlassRod 之后棒悬于烧杯口上方（TCP (0.412,0.0807,1.15)、棒底 0.95 高于口顶
0.8904）。本动作 3 段：
  ① 竖直下探插入：mv(TCP → STIR_CENTER (0.412,0.0807,1.02))——z 1.15→1.02，棒底
     0.95→0.82 穿过烧杯口（口顶 0.8904）沉入混合液（液面顶 0.84 下 2cm、杯内底
     0.80 上 2cm）；棒在口中心竖直下探，Ø6 对 Ø60 内口无碰壁
  ② 圆周搅拌：stir(STIR_CENTER, radius=STIR_RADIUS, cycles=STIR_CYCLES,
     period=STIR_PERIOD)——TCP 画半径 15mm 水平圆（z 锁 1.02），棒随爪纯平移
     （task _update_rod 已覆盖 attached 全程），棒底在液内画同半径圆搅拌；圆
     半径+棒半宽 18mm < 杯内径 r 30mm 不碰壁
  ③ 竖直提出回 H：mv(TCP → (0.412,0.0807,H))——棒底回 0.95 出杯，姿态恢复 ⑧ 末端

夹爪全程保持 GRIP_ROD(0.003) 持握（grip_target 由 controller 从 PickGlassRod
传播，①②③ 无 grip 原子动作不松爪——工具已吸附类，ReturnWashBottle 同款）。
"""
from ._base import BaseMetaAction, mv, stir
from .constants import H, ORIENT_FWD, STIR_CENTER, STIR_RADIUS, STIR_CYCLES, STIR_PERIOD


class StirInBeaker(BaseMetaAction):
    """下降玻璃棒插入烧杯并搅拌：竖直下探 → 圆周搅拌 N 圈 → 竖直提出回高。"""

    def _build_actions(self):
        e = self.engine
        bx, by, _ = STIR_CENTER
        return [
            mv(e, STIR_CENTER, orient=ORIENT_FWD),                     # ① 竖直下探插入烧杯
            stir(e, STIR_CENTER, radius=STIR_RADIUS,
                 cycles=STIR_CYCLES, period=STIR_PERIOD, orient=ORIENT_FWD),  # ② 圆周搅拌
            mv(e, (bx, by, H), orient=ORIENT_FWD),                     # ③ 竖直提出回高
        ]
