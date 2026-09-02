"""元动作 ⑥：火星端悬停氧气试管口上方（D9 氧气检验，2026-09-01 用户「放在氧气室管口的
上方不要伸进去然后复燃」）。

木条余烬（火星）端悬停竖立氧气试管口正上方 15mm，**不伸进管内**。氧气预收集在管内，扩散
上溢接触余烬 → task 检测木条端悬停管口连续帧 → oxygen_result=reignite 时余烬复燃
（SplintEmber 隐、SplintFlame 显）。

避穿模：HOVER_GRIP=(0.409,0.389,0.974) 夹爪在试管架 -X 侧（架 min x=0.457 之外），木条端
(+X 0.11) 横越架顶（顶板 0.917 < 木条 0.974）到管口上方，无立柱穿模。木条端世界坐标 =
(0.519,0.389,0.974)，恰在管口 (0.519,0.389,0.959) 上方 15mm。

流程（一次持握，不松爪）：
  ① 高位横移：mv((hx,hy,SPLINT_HIGH))——先到管口上方高位
  ② 下探悬停：mv(HOVER_GRIP, 30)——木条端悬停管口上方 15mm
  ③ 悬停观察：hold(HOVER_DWELL)——task 判定复燃/不复燃
  ④ 退回高位：mv((hx,hy,SPLINT_HIGH))
"""
from ._base import BaseMetaAction, mv, hold
from .constants import HOVER_GRIP, HOVER_DWELL, SPLINT_HIGH


class HoverSplint(BaseMetaAction):
    """高位横移 → 下探悬停管口上方（不伸入）→ 悬停观察 → 退回高位。"""

    def _build_actions(self):
        e = self.engine
        hx, hy, _ = HOVER_GRIP
        return [
            mv(e, (hx, hy, SPLINT_HIGH)),       # ① 高位横移到管口上方
            mv(e, HOVER_GRIP, 30),              # ② 下探：木条端悬停管口上方 15mm（不伸入）
            hold(e, HOVER_DWELL),               # ③ 悬停观察（task 判定复燃/不复燃）
            mv(e, (hx, hy, SPLINT_HIGH)),       # ④ 退回高位
        ]
