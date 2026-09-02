# -*- coding: utf-8 -*-
"""A3 ⑧ 到试管架上水平横夹取出玻璃棒并移动到烧杯上方（2026-08-30 用户「到试管架上水平
横夹取出玻璃棒并移动到烧杯上方」）。

玻璃棒 Ø6×261mm 竖直插试管架（/World/GlassRod (0.5434,0.1995)，底 0.80=台面 顶 1.061、
架顶 0.917）。抓点 = 棒底上方 0.20m（z=1.00，架顶 0.917 上 8.3cm 可握段，同 E1 PickRod）。
水平横夹 = 手指 ORIENT_FWD 朝 +X、两指沿 ±Y 夹 Ø6 棒身（同洗瓶/药匙），棒顶 1.061 伸出
指端之上无碰撞（E1 同款已验证）。纯平移持握：棒底 = TCP − 0.20。

轨迹：① 高位 (0.5434,0.1995,H) → ② 下探抓点 (0.5434,0.1995,1.00) → ③ 合爪 GRIP_ROD
→ ④ 竖直提出架孔（棒底 0.80→0.95）→ ⑤ 水平移到烧杯口正上方 (0.412,0.0807,H)（棒底悬
0.95，烧杯口顶 0.8904 上 6cm；后续动作再下探搅拌/蘸取）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_ROD, ORIENT_FWD, ROD_XY, ROD_GRASP, BEAKER_MOUTH_XY


class PickGlassRod(BaseMetaAction):
    """横夹取出玻璃棒并移到烧杯上方：高位接近 → 下探横夹 → 合爪 → 提出 → 移烧杯上方。"""

    def _build_actions(self):
        e = self.engine
        rx, ry = ROD_XY
        bx, by = BEAKER_MOUTH_XY
        return [
            mv(e, (rx, ry, H), orient=ORIENT_FWD),      # ① 高位：棒正上方
            mv(e, ROD_GRASP, orient=ORIENT_FWD),        # ② 下探到抓点（z=1.00，架顶上方可握段）
            grip(e, GRIP_ROD, 60),                      # ③ 合爪横夹棒身（Ø6 → 开度 0.003）
            mv(e, (rx, ry, H), orient=ORIENT_FWD),      # ④ 竖直提出架孔（棒底 0.80→0.95）
            mv(e, (bx, by, H), orient=ORIENT_FWD),      # ⑤ 水平移到烧杯口正上方（棒底悬 0.95）
        ]
