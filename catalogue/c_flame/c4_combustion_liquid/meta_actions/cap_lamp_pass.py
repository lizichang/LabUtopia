"""元动作 ⑤：取灯帽盖灭火焰（2026-09-02 用户「燃烧 4s → +y 5cm 观察 10s → 放回，然后盖上酒精灯帽熄灭
火焰」）。

阶段 ③ SpoonToFlamePass 燃烧 4s 观察 10s 放回燃烧匙后，机械臂从桌面取灯帽（灯旁 -X 12cm）、移到
灯口上方扣下盖灭火焰（照 B2 CapLampPass 逐字移植）。

C4 布局：帽 /World/AlcoholLamp/cap（灯子 prim）Ø37mm×3.1cm 开口朝下倒扣桌面，帽中心
静止位 (0.5894,-0.10,0.8155)（灯旁 -X 12cm，gen CAP_DETACH translate(0.12,0,-0.0762) 随
灯 R180 → 帽世界 x=灯x−0.12）。盖严实 = 帽中心 0.8915（帽底 0.8760 < 灯体顶 0.8897，
帽 local translate → (0,0,0) = 资产原始帽位），夹爪 CAP_BURNER 0.900。

2026-09-01 B2 三改移植：夹帽/全程默认朝向手指朝下（低 z 桌面夹帽 ORIENT_FWD 手指朝前
Lula 无解）；纯平移持握（帽中心 = 夹爪 + CAP_HELD_OFFSET），帽竖直开口朝下不旋转。
火焰熄灭时机照 B2 十一改：帽下降罩过火焰顶（CAP_EXTINGUISH_Z 门控）才熄，不移动时早灭。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 高位接近：mv((cx,cy,CAP_HIGH))——先到帽上方（高于火焰顶 0.936 清障）。
  ② 竖直下探：mv(CAP_GRASP) 到帽顶下 7mm（手指朝下竖直夹帽体上部）。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task 检测 attached（帽纯平移持握）。
  ⑤ 垂直提起：mv((cx,cy,CAP_HIGH), 5)（提离桌面，帽开口仍朝下）。
  ⑥ 运到灯口上方：mv((bx,by,CAP_HIGH))（灯口在 (0.7094,-0.10)）。
  ⑦ 下扣盖灭：mv(CAP_BURNER, 25)（帽开口端套灯口盖住灯芯；task 下降即熄火）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: cap settled → 火焰熄灭、帽锁灯口。
  ⑨ 退回：mv((bx,by,H))。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_CAP,
                        CAP_GRASP, CAP_HIGH, CAP_BURNER)


class CapLampPass(BaseMetaAction):
    """取灯帽（桌面）→ 高位 → 移到灯口上方 → 下扣盖灭 → 松爪（task 熄火 + 帽锁灯口）。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = CAP_GRASP
        bx, by, _ = CAP_BURNER
        return [
            mv(e, (cx, cy, CAP_HIGH)),          # ① 高位接近（帽上方，清障）
            mv(e, CAP_GRASP),                   # ② 竖直下探到夹帽点（手指朝下）
            hold(e, SETTLE),                    # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAP, 60),              # ④ 合爪夹帽（task 检测 attached）
            mv(e, (cx, cy, CAP_HIGH), 5),       # ⑤ 垂直提起 + 停顿
            mv(e, (bx, by, CAP_HIGH)),          # ⑥ 运到灯口上方
            mv(e, CAP_BURNER, 25),              # ⑦ 下扣盖灭（帽开口端套灯口）
            grip(e, GRIP_OPEN, 25),             # ⑧ 松爪：task 火焰熄灭 + 帽锁灯口
            mv(e, (bx, by, H)),                 # ⑨ 退回（帽已锁灯口）
        ]
