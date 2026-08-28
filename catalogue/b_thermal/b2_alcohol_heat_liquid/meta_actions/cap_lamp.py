"""元动作 ⑥：盖灯帽灭火（B2 沸点测定 V9，2026-08-28 用户新增）。

用户逐字：「可以现在新增动作移动完酒精灯，把灯冒盖上参考焰色反应，然后火焰熄灭」——
移灯动作（LampMovePass 把酒精灯 -y 移 10cm 到位）之后，机械臂从桌面取灯帽、移到灯口
上方扣下盖灭火焰。参考焰色反应 Extinguish 的 B/C 两段（取帽→盖灭），无 A 段归架铂丝。

几何（2026-08-28 pxr 实测）：帽 = /World/AlcoholLamp/cap（灯子 prim），Ø37mm×3.1cm
竖直开口朝下倒扣桌面。灯原位帽世界中心 (0.5286,-0.10,0.8157)；移灯后灯在
y=-0.0971，帽跟随灯到 (0.5286,-0.20,0.8157)（2026-08-28 帽 local ty 0.467→0.103：
旧位 -0.5645 在机械臂底座 y=-0.08 后方 48cm，Lula IK 无解卡死；新位 0.55m 可达）。
盖灭时帽中心 0.9067（开口端套灯口 holder 顶 0.8912），夹爪 CAP_BURNER 0.915 =
帽中心 + 持握偏移 0.0083。

流程（一次持握，无循环）：
  ① 高位接近：mv((cx,cy,CAP_HIGH))——先到帽上方（高于火焰顶 0.938 清障）。
  ② 竖直下探：mv(CAP_GRASP) 到帽顶下 7mm。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task 检测 attached（帽纯平移持握跟随夹爪）。
  ⑤ 垂直提起：mv((cx,cy,CAP_HIGH), 5)（提离桌面，帽开口仍朝下）。
  ⑥ 运到灯口上方：mv((bx,by,CAP_HIGH))（灯 -y 移走后灯口在 (0.5286,-0.0971)）。
  ⑦ 下扣盖灭：mv(CAP_BURNER, 25)（帽开口端套灯口，盖住灯芯）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: cap settled → 火焰熄灭、帽锁灯口 → phase→done。
  ⑨ 退回：mv((bx,by,H))。

轨迹（TCP 世界坐标，全程 ORIENT_FWD 手指朝前）：
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
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_CAP, ORIENT_FWD,
                        CAP_GRASP, CAP_HIGH, CAP_BURNER)


class CapLampPass(BaseMetaAction):
    """取灯帽（桌面）→ 高位 → 移到灯口上方 → 下扣盖灭 → 松爪（task 熄火 + 帽锁灯口）。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = CAP_GRASP
        bx, by, _ = CAP_BURNER
        return [
            mv(e, (cx, cy, CAP_HIGH), orient=ORIENT_FWD),   # ① 高位接近（帽上方，清障）
            mv(e, CAP_GRASP, orient=ORIENT_FWD),            # ② 竖直下探到夹帽点
            hold(e, SETTLE),                                 # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAP, 60),                           # ④ 合爪夹帽（task 检测 attached）
            mv(e, (cx, cy, CAP_HIGH), 5, orient=ORIENT_FWD), # ⑤ 垂直提起 + 停顿
            mv(e, (bx, by, CAP_HIGH), orient=ORIENT_FWD),    # ⑥ 运到灯口上方
            mv(e, CAP_BURNER, 25, orient=ORIENT_FWD),        # ⑦ 下扣盖灭（帽开口端套灯口）
            grip(e, GRIP_OPEN, 25),                          # ⑧ 松爪：task 火焰熄灭 + 帽锁灯口
            mv(e, (bx, by, H), orient=ORIENT_FWD),           # ⑨ 退回（帽已锁灯口）
        ]
