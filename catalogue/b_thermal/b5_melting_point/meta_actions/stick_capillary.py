# -*- coding: utf-8 -*-
"""StickCapillaryToBulb：夹封口端拎起竖直后，移到倒插温度计泡旁，竖直下探封口端贴泡（task 侧吸附）。

用户 2026-09-02 逐字：「然后再拎起毛细管的一端像前两次把他变成竖直的，然后再贴到温度计上，
你现在这个是横着的根本就不符合生活实际不符合物理世界啊」——弃旧「中部水平持握横着贴泡」，
改「夹封口端拎起（pivot 竖直，封口端朝上、开口端垂下，同 ① 蘸粉姿势）→ 移到倒插温度计泡旁
竖直贴泡」。夹封口端时 TCP = 封口端（封口端就在夹爪处，非旧中部持握的「夹点 − 0.05·X」），
故贴泡 TCP 目标 = STICK_SEALED（倒插泡中心 +Y 侧）直接下探。

轨迹（手指朝下朝向全程不变，竖直由 task 侧 pivot 持握保证）：
  ① 横移到泡上方   mv((STICK_SEALED.xy, STICK_APPROACH_Z))
  ② 竖直下探贴泡   mv(STICK_SEALED, dwell)   # 封口端(=TCP)贴泡，task 检测吸附置 stuck
  ③ 松爪           grip(GRIP_OPEN)          # 毛细管已粘泡（stuck），随温度计走

几何：封口端=TCP 下探到 STICK_SEALED=(0.3471,0.2706,1.076) = 泡中心 (0.3471,0.2696,1.076) +Y 侧
（2026-09-02 用户报「贴不够近偏+y 0.5cm」→ 0.2756→0.2706 -Y 0.5cm 贴紧泡）；开口端沿竖直垂到
0.976（泡下方），毛细管竖直平行贴温度计杆身，符合物理。随温度计法兰翻转/插管全程整组移动
（_update_stuck_capillary）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_OPEN, STICK_SEALED, STICK_APPROACH_Z


class StickCapillaryToBulb(BaseMetaAction):
    """夹封口端拎起竖直后：横移到温度计泡上方 → 竖直下探封口端贴泡（task 侧吸附）→ 松爪。"""

    def _build_actions(self):
        e = self.engine
        sx, sy, sz = STICK_SEALED
        return [
            mv(e, (sx, sy, STICK_APPROACH_Z)),   # ① 横移到泡上方（泡顶 1.084 上方）
            mv(e, (sx, sy, sz), dwell=40),       # ② 竖直下探封口端贴泡（task 检测吸附）
            grip(e, GRIP_OPEN, 60),              # ③ 松爪（毛细管已粘泡，随温度计走）
        ]
