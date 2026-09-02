"""元动作 ⑤：快速摆动熄火（D9 氧气检验，2026-09-01 用户「快速摆动机械臂让它熄灭」）。

木条点燃后（明火 SplintFlame 钉木条端），快速水平摆动机械臂把明火甩灭，留下带火星的余烬
（task 检测夹爪近摆动中心 SHAKE_GRIP 连续帧 → splint_lit 隐、splint_ember 显）。

shake 基元（照 ShakeAction）：在 SHAKE_GRIP 附近沿 x 轴正弦振荡 SHAKE_CYCLES 个来回，
振幅 SHAKE_AMPLITUDE=0.03、周期 SHAKE_PERIOD=45（快速甩灭）。摆动期间木条端明火逐帧跟随
（task _update_effects 钉端），甩灭瞬间换成余烬。

流程（一次持握，不松爪）：
  ① 高位横移：mv((bx,by,SPLINT_HIGH))——先到摆动中心上方
  ② 下探到位：mv(SHAKE_GRIP)——降到摆动高度
  ③ 快速摆动：shake(SHAKE_GRIP, axis=(1,0,0), ...)——水平快速振荡甩灭明火
  ④ 退回高位：mv((bx,by,SPLINT_HIGH))
"""
from ._base import BaseMetaAction, mv, shake
from .constants import (SHAKE_GRIP, SHAKE_AMPLITUDE, SHAKE_CYCLES, SHAKE_PERIOD,
                        SPLINT_HIGH)


class BlowOutSplint(BaseMetaAction):
    """高位横移 → 下探 → 快速摆动甩灭明火 → 退回高位。"""

    def _build_actions(self):
        e = self.engine
        bx, by, _ = SHAKE_GRIP
        return [
            mv(e, (bx, by, SPLINT_HIGH)),       # ① 高位横移到摆动中心上方
            mv(e, SHAKE_GRIP),                  # ② 下探到摆动高度
            shake(e, SHAKE_GRIP, axis=(1, 0, 0), amplitude=SHAKE_AMPLITUDE,
                  cycles=SHAKE_CYCLES, period=SHAKE_PERIOD),   # ③ 快速摆动甩灭
            mv(e, (bx, by, SPLINT_HIGH)),       # ④ 退回高位（余烬火星留木条端）
        ]
