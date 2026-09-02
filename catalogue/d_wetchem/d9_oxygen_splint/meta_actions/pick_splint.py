"""元动作 ③：夹木条（D9 氧气检验，2026-09-01 用户「拿木条点燃」的第一段=取木条）。

照 B2 火柴 LightFlamePass 的取火柴段（纯平移持握）：木条 Ø6mm×150mm 水平横躺（轴 +X），
手指朝下竖直夹杆身（local x=0.04 处，点燃端在 +X 0.15）。持握 = 纯平移 offset
（SPLINT_HELD_OFFSET）：木条原点 = 夹爪 + (-0.04,0,0)，点燃端 = 夹爪 + (0.11,0,0)。

流程（一次持握，取完不松爪，木条留夹爪手上给 ④ 点燃）：
  ① 高位接近：mv((sx,sy,SPLINT_HIGH))
  ② 竖直下探：mv(SPLINT_GRASP) 到杆身抓点（杆中心 z=0.813）
  ③ 停顿稳定：hold(SETTLE)
  ④ 合爪夹木条：grip(GRIP_SPLINT, 60) → task 检测 attached（纯平移持握）
  ⑤ 竖直提起：mv((sx,sy,SPLINT_HIGH), 5)
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import SETTLE, GRIP_SPLINT, SPLINT_GRASP, SPLINT_HIGH


class PickSplint(BaseMetaAction):
    """高位接近 → 竖直下探夹木条 → 提起（木条留夹爪手上）。"""

    def _build_actions(self):
        e = self.engine
        sx, sy, _ = SPLINT_GRASP
        return [
            mv(e, (sx, sy, SPLINT_HIGH)),       # ① 高位接近（手指朝下）
            mv(e, SPLINT_GRASP),                # ② 竖直下探到杆身抓点（两指竖直夹杆）
            hold(e, SETTLE),                    # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_SPLINT, 60),           # ④ 合爪夹住木条（task 检测 attached）
            mv(e, (sx, sy, SPLINT_HIGH), 5),    # ⑤ 竖直提起（木条留手上）
        ]
