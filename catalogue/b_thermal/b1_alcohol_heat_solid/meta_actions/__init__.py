"""B1 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

B1 本批次三个过程（用户 2026-08-27：「先写咬粉末咬进试管里面，然后拿起酒精灯盖儿放到
一边儿，再拿起火柴点燃酒精灯，这几个过程先只写这些我来验收」）：
  ① PickSpatula —— 复用 d2s（横向夹药匙 → 竖直提 → 法兰转 → 对齐粉堆 → 下探挖粉
     → 抬升 → 平移 → 回卷倒粉入管），坐标不动（home=None → d2s SPAT_XY）
  ② ReturnSpatula —— 复用 d2s（药匙放回试管架），坐标不动
  ③ OpenCapPass —— 拿起酒精灯盖放到一边（本包新写：取帽 → 提起 → 横移 → 落台面 →
     松爪归位；纯平移持握，帽中心 = 夹爪）
  ④ LightFlamePass —— 取火柴点燃酒精灯（B2 同款逐字：抓杆 → 触灯芯 → 点燃 → 放回火柴；
     B1 无温度模型，flame_lit 置位即 reveal 火焰）
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .open_cap import OpenCapPass
from .light_flame import LightFlamePass
# 挖粉/放回药匙 直接复用 d2s 元动作包（药匙/皿/粉/试管/试管架坐标逐字一致）
from catalogue.d_wetchem.d2s_water_solubility.meta_actions import (PickSpatula, ReturnSpatula)

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake",
           "PickSpatula", "ReturnSpatula", "OpenCapPass", "LightFlamePass"]
