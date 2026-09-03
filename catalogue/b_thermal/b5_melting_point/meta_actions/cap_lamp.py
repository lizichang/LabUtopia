"""元动作 ⑯：盖灯帽灭火（B5 熔点测定，移灯 -X 5cm 后）。

用户 2026-09-03 逐字：「最后盖酒精灯帽前，应该先把酒精灯往-x移动5cm移出再盖酒精灯。夹酒精灯
参考B2、B3。」——LampHeatMovePass 移灯 -X 5cm 后，机械臂从桌面取灯帽、移到移灯后灯口
(0.305,0.0029) 上方扣下盖灭火焰。参考 B2 CapLampPass（焰色反应 Extinguish 取帽→盖灭）。

几何（gen CAP_DETACH 帽摘放灯 -X 侧 12cm 台面）：帽 = /World/AlcoholLamp/cap，Ø37mm×3.1cm
竖直开口朝下倒扣桌面，静止位 LAMPCAP_REST=(0.235,0.0029,0.8155)。盖灭时帽中心 0.8917
（帽底 0.8762 盖严实，套住灯口，同 B2 资产原始帽位），夹爪 LAMPCAP_BURNER=(0.305,0.0029,0.900)
= 帽中心 + LAMPCAP_HELD_OFFSET(0,0,-0.0083)。移灯后火焰尖在 (0.305,0.0029,0.9795)，帽竖直
下扣罩过火焰尖才灭（task LAMPCAP_EXTINGUISH_Z=1.00 门控）。

夹帽全程默认朝向（手指朝下），同 B2 三改：低 z 夹帽点 (0.235,0.0029,0.824) ORIENT_FWD 手指朝前
Lula 解不出 → 照抄火柴 LightFlamePass 结构（纯平移持握，帽竖直开口朝下跟夹爪）。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 高位接近   mv((cx,cy,LAMPCAP_HIGH))（帽上方，高于火焰尖 0.9795 清障）
  ② 竖直下探   mv(LAMPCAP_GRASP)（帽顶下 7mm，手指朝下竖直夹帽体上部）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹帽   grip(GRIP_CAP, 60) → task attached（帽纯平移持握，帽中心=夹爪+offset）
  ⑤ 垂直提起   mv((cx,cy,LAMPCAP_HIGH), 5)
  ⑥ 灯口上方   mv((bx,by,LAMPCAP_HIGH))（移灯后灯口 0.305）
  ⑦ 下扣盖灭   mv(LAMPCAP_BURNER, 25)（帽开口端套灯口盖住灯芯）
  ⑧ 松爪释放   grip(GRIP_OPEN, 25) → task cap settled → 火焰熄灭、帽锁灯口 → phase→done
  ⑨ 退回       mv((bx,by,LAMPCAP_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_CAP,
                        LAMPCAP_GRASP, LAMPCAP_HIGH, LAMPCAP_BURNER)


class CapLampPass(BaseMetaAction):
    """取灯帽（桌面）→ 高位 → 移到灯口上方 → 下扣盖灭 → 松爪（task 熄火 + 帽锁灯口）。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = LAMPCAP_GRASP
        bx, by, _ = LAMPCAP_BURNER
        return [
            mv(e, (cx, cy, LAMPCAP_HIGH)),          # ① 高位接近（帽上方，清障）
            mv(e, LAMPCAP_GRASP),                   # ② 竖直下探到夹帽点（手指朝下）
            hold(e, SETTLE),                        # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAP, 60),                  # ④ 合爪夹帽（task 检测 attached）
            mv(e, (cx, cy, LAMPCAP_HIGH), 5),       # ⑤ 垂直提起 + 停顿
            mv(e, (bx, by, LAMPCAP_HIGH)),          # ⑥ 运到灯口上方
            mv(e, LAMPCAP_BURNER, 25),              # ⑦ 下扣盖灭（帽开口端套灯口）
            grip(e, GRIP_OPEN, 25),                 # ⑧ 松爪：task 火焰熄灭 + 帽锁灯口
            mv(e, (bx, by, LAMPCAP_HIGH)),          # ⑨ 退回（帽已锁灯口）
        ]
