"""元动作 ①：摘酒精灯帽（D9 氧气检验，2026-09-01 用户「摘开酒精灯帽」）。

B2 CapLampPass（盖帽灭火）的**逆操作**：帽起始在灯上，机械臂从灯口夹帽 → 提起 → 移到
桌面静止位 → 松爪放好。持握 = 纯平移（帽中心 = 夹爪 + CAP_HELD_OFFSET），全程默认朝向
手指朝下（B2 盖帽同款，已验证）。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 高位接近：mv((cx,cy,CAP_HIGH))——先到灯上方（高于帽顶 0.9072）。
  ② 竖直下探：mv(CAP_ON_GRASP) 到帽顶下 7mm（手指朝下竖直夹帽体上部）。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task 检测 attached（帽纯平移持握）。
  ⑤ 垂直提起：mv((cx,cy,CAP_HIGH), 5)（提离灯口，帽开口仍朝下）。
  ⑥ 运到帽静止位上方：mv((rx,ry,CAP_HIGH))。
  ⑦ 下探放帽：mv(CAP_REST_GRASP)（帽底贴桌面静止位）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: released → 帽锁 CAP_REST。
  ⑨ 退回：mv((rx,ry,CAP_HIGH))。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((cx,cy,CAP_HIGH))
  ② 竖直下探   mv(CAP_ON_GRASP)
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹帽   grip(GRIP_CAP, 60)
  ⑤ 垂直提起   mv((cx,cy,CAP_HIGH), 5)
  ⑥ 静止位上方 mv((rx,ry,CAP_HIGH))
  ⑦ 下探放帽   mv(CAP_REST_GRASP)
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)
  ⑨ 退回       mv((rx,ry,CAP_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_CAP,
                        CAP_ON_GRASP, CAP_REST_GRASP, CAP_HIGH)


class CapOffPass(BaseMetaAction):
    """夹帽（灯口）→ 提起 → 移到桌面静止位 → 松爪放好。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = CAP_ON_GRASP
        rx, ry, _ = CAP_REST_GRASP
        return [
            mv(e, (cx, cy, CAP_HIGH)),          # ① 高位接近（灯上方，清障）
            mv(e, CAP_ON_GRASP),                # ② 竖直下探到夹帽点（手指朝下）
            hold(e, SETTLE),                    # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAP, 60),              # ④ 合爪夹帽（task 检测 attached）
            mv(e, (cx, cy, CAP_HIGH), 5),       # ⑤ 垂直提起 + 停顿
            mv(e, (rx, ry, CAP_HIGH)),          # ⑥ 运到帽静止位上方
            mv(e, CAP_REST_GRASP),              # ⑦ 下探放帽（帽底贴桌面）
            grip(e, GRIP_OPEN, 25),             # ⑧ 松爪：task cap released 锁 CAP_REST
            mv(e, (rx, ry, CAP_HIGH)),          # ⑨ 退回（帽已放桌面）
        ]
