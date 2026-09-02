"""元动作 ④：木条端伸入灯焰点燃（D9 氧气检验，2026-09-01 用户「拿木条点燃」第二段）。

木条已夹在夹爪手上（PickSplint 之后），此步把木条点燃端伸进酒精灯火焰点燃。夹爪横移到
LIGHT_GRIP（灯体 -X 侧，木条端 +X 0.11 处恰落灯焰中心 FLAME_CENTER），dwell 让 task 检测
端近灯焰 → splint_lit（复燃焰显，钉木条端）。

避穿模：LIGHT_GRIP.x=0.4186 在灯体 min x=0.485 之外，仅木条端伸进灯焰；夹爪全程在灯体顶
0.9072 之上（LIGHT_GRIP.z=0.920 略高于灯体顶，木条水平横越）。

流程（一次持握，不松爪）：
  ① 高位横移：mv((lx,ly,SPLINT_HIGH))——先到灯焰上方高位
  ② 下探点燃：mv(LIGHT_GRIP, 20)——木条端伸入灯焰（dwell 点燃）
  ③ 退回高位：mv((lx,ly,SPLINT_HIGH))——木条已燃，退回高位
"""
from ._base import BaseMetaAction, mv
from .constants import LIGHT_GRIP, SPLINT_HIGH


class LightSplint(BaseMetaAction):
    """高位横移 → 下探使木条端入灯焰点燃 → 退回高位。"""

    def _build_actions(self):
        e = self.engine
        lx, ly, _ = LIGHT_GRIP
        return [
            mv(e, (lx, ly, SPLINT_HIGH)),       # ① 高位横移到灯焰上方
            mv(e, LIGHT_GRIP, 20),              # ② 下探：木条端伸入灯焰（dwell 点燃）
            mv(e, (lx, ly, SPLINT_HIGH)),       # ③ 退回高位（木条已燃）
        ]
