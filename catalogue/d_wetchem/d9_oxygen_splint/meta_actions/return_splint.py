"""元动作 ⑦：木条取出归位（D9 氧气检验，2026-09-01 用户动作链末段「取出归位」）。

复燃观察完成后，**先把木条甩到摆动中心左右摇晃熄灭复燃明火**，再放回台面静止位。照 B2
火柴放回段（纯平移持握）：甩灭 → 高位横移回木条静止位上方 → 竖直下探到抓点 → 松爪 → 高位
归位（task 木条生命周期写回 rest）。

流程（一次持握，松爪后木条回静止位）：
  ① 高位横移：mv((bx,by,SPLINT_HIGH))——先到摆动中心上方
  ② 下探摆动：mv(SHAKE_GRIP)——降到摆动高度
  ③ 左右摇晃：shake(SHAKE_GRIP, axis=(1,0,0), ...)——甩灭复燃明火（task 二次熄火判定）
  ④ 高位横移：mv((sx,sy,SPLINT_HIGH))——回木条静止位上方
  ⑤ 竖直下探：mv(SPLINT_GRASP)——降到杆身抓点
  ⑥ 松爪释放：grip(GRIP_OPEN, 25)——task: splint released → 木条写回 rest
  ⑦ 高位归位：mv((sx,sy,SPLINT_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, shake
from .constants import (GRIP_OPEN, SPLINT_GRASP, SPLINT_HIGH,
                        SHAKE_GRIP, SHAKE_AMPLITUDE, SHAKE_CYCLES, SHAKE_PERIOD)


class ReturnSplint(BaseMetaAction):
    """甩灭复燃明火 → 高位横移回静止位上方 → 竖直下探 → 松爪放回 → 高位归位。"""

    def _build_actions(self):
        e = self.engine
        sx, sy, _ = SPLINT_GRASP
        bx, by, _ = SHAKE_GRIP
        return [
            mv(e, (bx, by, SPLINT_HIGH)),       # ① 高位横移到摆动中心上方
            mv(e, SHAKE_GRIP),                  # ② 下探到摆动高度
            shake(e, SHAKE_GRIP, axis=(1, 0, 0), amplitude=SHAKE_AMPLITUDE,
                  cycles=SHAKE_CYCLES, period=SHAKE_PERIOD),   # ③ 左右甩灭复燃明火
            mv(e, (sx, sy, SPLINT_HIGH)),       # ④ 高位横移回木条静止位上方
            mv(e, SPLINT_GRASP),                # ⑤ 竖直下探到杆身抓点
            grip(e, GRIP_OPEN, 25),             # ⑥ 松爪：task 木条写回 rest
            mv(e, (sx, sy, SPLINT_HIGH)),       # ⑦ 高位归位
        ]
