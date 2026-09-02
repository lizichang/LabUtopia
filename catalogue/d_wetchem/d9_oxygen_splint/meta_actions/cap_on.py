"""元动作 ⑧：盖酒精灯帽（D9 氧气检验，2026-09-01 用户「最后还要加一个盖灯冒的动作」）。

CapOffPass（① 摘帽）的**逆操作**：帽已在桌面静止位 CAP_REST（① 摘帽时放的），此步机械臂
从桌面夹帽 → 提起 → 运到灯口上方 → 下探盖帽 → 松爪（task cap_on released → 帽回灯上 +
酒精灯火焰熄灭）。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 高位接近：mv((rx,ry,CAP_HIGH))——先到帽静止位上方。
  ② 竖直下探：mv(CAP_REST_GRASP) 到桌面帽夹点（帽顶下 7mm，低 z 解冻）。
  ③ 停顿稳定：hold(SETTLE)。
  ④ 合爪夹帽：grip(GRIP_CAP, 60) → task cap_on 检测 attached（纯平移持握）。
  ⑤ 垂直提起：mv((rx,ry,CAP_HIGH), 5)（提离桌面）。
  ⑥ 运到灯口上方：mv((cx,cy,CAP_HIGH))。
  ⑦ 下探盖帽：mv(CAP_ON_GRASP)（帽落灯口，开口朝下盖住灯芯）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task cap_on released → 帽锁灯上 + 火焰熄。
  ⑨ 退回：mv((cx,cy,CAP_HIGH))。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((rx,ry,CAP_HIGH))
  ② 竖直下探   mv(CAP_REST_GRASP)
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹帽   grip(GRIP_CAP, 60)
  ⑤ 垂直提起   mv((rx,ry,CAP_HIGH), 5)
  ⑥ 灯口上方   mv((cx,cy,CAP_HIGH))
  ⑦ 下探盖帽   mv(CAP_ON_GRASP)
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)
  ⑨ 退回       mv((cx,cy,CAP_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_CAP,
                        CAP_ON_GRASP, CAP_REST_GRASP, CAP_HIGH)


class CapOnPass(BaseMetaAction):
    """夹帽（桌面）→ 提起 → 运到灯口 → 下探盖帽 → 松爪（帽回灯上 + 火焰熄）。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = CAP_ON_GRASP
        rx, ry, _ = CAP_REST_GRASP
        return [
            mv(e, (rx, ry, CAP_HIGH)),               # ① 高位接近（帽静止位上方，清障）
            mv(e, CAP_REST_GRASP),               # ② 竖直下探到桌面夹帽点
            hold(e, SETTLE),                         # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAP, 60),                   # ④ 合爪夹帽（task cap_on 检测 attached）
            mv(e, (rx, ry, CAP_HIGH), 5),            # ⑤ 垂直提起 + 停顿
            mv(e, (cx, cy, CAP_HIGH)),               # ⑥ 运到灯口上方
            mv(e, CAP_ON_GRASP),                     # ⑦ 下探盖帽（帽落灯口盖住灯芯）
            grip(e, GRIP_OPEN, 25),                  # ⑧ 松爪：task cap_on released 帽回灯上+熄焰
            mv(e, (cx, cy, CAP_HIGH)),               # ⑨ 退回（帽已盖回灯上）
        ]
