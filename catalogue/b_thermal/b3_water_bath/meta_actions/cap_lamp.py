"""元动作 ③：盖灯帽灭火（B3 水浴加热 阶段C，复刻 B2 CapLampPass）。

用户逐字（B2 2026-08-28）：「可以现在新增动作移动完酒精灯，把灯冒盖上参考焰色反应，
然后火焰熄灭」——机械臂从桌面取灯帽、移到灯口上方扣下盖灭火焰。参考焰色反应
Extinguish 的 B/C 两段（取帽→盖灭），无 A 段归架铂丝。

B3 与 B2 唯一差别：**无移灯动作**（B3 简化流程，加热后直接原位盖帽），故 CAP_BURNER
直指灯原位 (0.5286,0.0029,0.900)，而非 B2 灯移 20cm 后的 (0.5286,-0.1971,0.900)。
其余（夹帽默认朝向手指朝下、纯平移持握、盖到位熄火、帽锁灯口）全部照搬 B2。

几何（2026-08-28 pxr 实测，B2 复用）：帽 = /World/AlcoholLamp/cap（灯子 prim），
Ø37mm×3.1cm 竖直开口朝下倒扣桌面。帽静止位 CAP_REST=(0.42,-0.01,0.8155)（灯前 -X
侧桌面）；盖灭时帽中心 0.8917（盖严实，同资产原始帽位：帽底 0.8762 = 灯z0.8002+0.0760，
套住 holder_top 0.8912 的灯口），夹爪 CAP_BURNER 0.900 = 帽中心 + 持握偏移 0.0083。

流程（一次持握，无循环，全程默认朝向手指朝下，同火柴 LightFlamePass）：
  ① 高位接近：mv((cx,cy,CAP_HIGH))——先到帽上方（高于火焰顶 0.938 清障）。
  ② 竖直下探：mv(CAP_GRASP) 到帽顶下 7mm（手指朝下竖直夹帽体上部）。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task 检测 attached（帽纯平移持握，帽中心=夹爪+CAP_HELD_OFFSET）。
  ⑤ 垂直提起：mv((cx,cy,CAP_HIGH), 5)（提离桌面，帽开口仍朝下）。
  ⑥ 运到灯口上方：mv((bx,by,CAP_HIGH))（灯原位 (0.5286,0.0029)）。
  ⑦ 下扣盖灭：mv(CAP_BURNER, 25)（帽开口端套灯口，盖住灯芯）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: cap settled → 火焰熄灭、帽锁灯口 → phase→done。
  ⑨ 退回：mv((bx,by,H))。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((cx,cy,CAP_HIGH))
  ② 竖直下探   mv(CAP_GRASP)
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹帽   grip(GRIP_CAP, 60)
  ⑤ 垂直提起   mv((cx,cy,CAP_HIGH), 5)
  ⑥ 灯口上方   mv((bx,by,CAP_HIGH))
  ⑦ 下扣盖灭   mv(CAP_BURNER, 25)
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)
  ⑨ 退回       mv((bx,by,H))
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
