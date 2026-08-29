"""元动作 ⑥：盖灯帽灭火（B2 沸点测定 V9，2026-08-28 用户新增）。

用户逐字：「可以现在新增动作移动完酒精灯，把灯冒盖上参考焰色反应，然后火焰熄灭」——
移灯动作（LampMovePass 把酒精灯 -y 移 10cm 到位）之后，机械臂从桌面取灯帽、移到灯口
上方扣下盖灭火焰。参考焰色反应 Extinguish 的 B/C 两段（取帽→盖灭），无 A 段归架铂丝。

几何（2026-08-28 pxr 实测）：帽 = /World/AlcoholLamp/cap（灯子 prim），Ø37mm×3.1cm
竖直开口朝下倒扣桌面。帽静止位 CAP_REST=(0.42,-0.01,0.8157)（灯前 -X 侧桌面）；
移灯期间 task 逐帧 _set_cap_world(CAP_REST) 把帽钉在静止位，不随灯滑到 -0.20
（2026-08-28 二改：帽随灯滑到 (0.5286,-0.20) 在机械臂底座 y=-0.08 后方 12cm 低 z，
Lula IK 无解卡死 → 改在静止位夹帽，可达性同火柴夹点 MATCH_GRASP，3D 距底座 0.44m；
2026-08-28 九改往 -X 移 3cm 到 x=0.42 避石棉网左边缘 0.4726，见 constants）。
盖灭时帽中心 0.8917（盖严实，同资产原始帽位：帽底 0.8762 = 灯z0.8002+0.0760，套住
holder_top 0.8912 的灯口），夹爪 CAP_BURNER 0.900 = 帽中心 + 持握偏移 0.0083
（2026-08-28 六改：旧 0.915 帽中心 0.9067 只搭灯口沿，悬空 15mm 没盖下去）。

2026-08-28 三改（用户运行 debug_cap_lamp 报「下去夹的时候又卡住了、跑半天没结束」）：
  * 夹帽/全程改**默认朝向（手指朝下）**——旧 ORIENT_FWD 手指朝前在低 z 夹帽点
    (0.45,-0.01,0.824) Lula 解不出（linewalk 每帧 IK None → 1500 帧 force-done，
    帽 never attached → cap_lamp 相永不 done → CapLampPass 死循环重跑）。B2 已验证的
    低 z 桌面抓全是默认朝下（火柴 MATCH_GRASP(0.44,-0.06,0.8145)/沸石 0.53m），
    逐个可达 → 帽照抄火柴 LightFlamePass 结构（纯平移持握，帽竖直开口朝下跟夹爪）。
  * 运帽 ⑥ 同改默认朝向（火柴 ⑤→⑥ 低空横移同款，已验证）。

流程（一次持握，无循环，全程默认朝向手指朝下，同火柴 LightFlamePass）：
  ① 高位接近：mv((cx,cy,CAP_HIGH))——先到帽上方（高于火焰顶 0.938 清障）。
  ② 竖直下探：mv(CAP_GRASP) 到帽顶下 7mm（手指朝下竖直夹帽体上部）。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task 检测 attached（帽纯平移持握，帽中心=夹爪+CAP_HELD_OFFSET）。
  ⑤ 垂直提起：mv((cx,cy,CAP_HIGH), 5)（提离桌面，帽开口仍朝下）。
  ⑥ 运到灯口上方：mv((bx,by,CAP_HIGH))（灯 -y 移走后灯口在 (0.5286,-0.1971)）。
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
