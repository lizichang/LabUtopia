"""元动作 ②：拿火柴点燃酒精灯（D9 氧气检验，2026-09-01 用户「拿火柴点燃酒精灯」）。

照 B2 LightFlamePass（catalogue/b_thermal/b2_alcohol_heat_liquid/meta_actions/light_flame.py）
逐字复制，仅改类名 IgniteLamp（坐标常量 D9 复用 B2 同款：火柴 (0.40,-0.06)、灯芯 (0.5286,
0.0029,0.9005)、火焰 base/apex 同 B2）。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 取火柴：高位接近 → 竖直下探到杆身中部抓点，合爪夹住火柴杆（GRIP_MATCH 贴合 Ø3mm）。
  ② 低位运移：竖直提起火柴到 MATCH_LIFT_Z=0.90，水平运到灯芯偏 -X 侧（IGNITE）。
  ③ 触灯芯：火柴头（夹爪 +X 0.0494）落在灯芯顶 WICK，task 检测头近灯芯 → 点火（flame reveal）。
  ④ 放回：原路退回火柴上方 → 松爪 → 高位归位（task 火柴生命周期写回 rest）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP,
                        MATCH_LIFT_Z, IGNITE, MATCH_HIGH)


class IgniteLamp(BaseMetaAction):
    """取火柴 → 低位运移 → 触灯芯点燃 → 放回火柴。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        return [
            mv(e, (mx, my, MATCH_HIGH)),            # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),                     # ② 竖直下探到杆身中心（两指竖直夹杆）
            hold(e, SETTLE),                        # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_LIFT_Z), 5),       # ⑤ 竖直提出（低位 0.90）
            mv(e, IGNITE, 20),                      # ⑥ 触灯芯点燃（头落灯芯，dwell 20）
            mv(e, (mx, my, MATCH_LIFT_Z)),          # ⑦ 原路退回火柴上方
            grip(e, GRIP_OPEN, 25),                 # ⑧ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),            # ⑨ 高位归位
        ]
