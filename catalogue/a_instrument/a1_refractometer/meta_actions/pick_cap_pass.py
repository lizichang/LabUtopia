"""元动作 ①：取瓶塞（直拔——只拔塞，不取瓶、不旋转开盖）。

参考 flametest 的 open_hcl_stopper（瓶口合爪 → 垂直提出 → 倒放桌面），A1 简化：
样品瓶白塞是**直插**的（无螺纹），故「取瓶盖」= 瓶口抓塞 → 原地合爪 → 垂直直拔
提出 → 平移旁侧 → 落座 → 开爪释放。**无取瓶、无旋转开盖**（用户 2026-08-25）。

轨迹（TCP 世界坐标，手指默认朝下）：
  ① 高位接近瓶口   mv((cx,cy,H))
  ② 下探抓塞       mv(CAP_GRASP)              # 塞顶 0.879 下 2mm（近顶抓）
  ③ 到位 settle    hold(SETTLE)
  ④ 原地合爪       grip(GRIP_STOPPER, 60)     # task 检测 attached
  ⑤ 垂直直拔       mv((cx,cy,0.93), 5)        # 塞离瓶口（瓶口 rim 0.870 上方 6cm）
  ⑥ 平移旁侧       mv((dx,dy,0.93))
  ⑦ 落座旁侧       mv((dx,dy,0.877))          # 抓点高度（task 释放时写塞到桌面 0.806）
  ⑧ 开爪释放       grip(GRIP_OPEN, 25)        # task: released → 塞倒放桌面 CAP_DESK
  ⑨ 归位           mv((dx,dy,H))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import H, SETTLE, GRIP_OPEN, GRIP_STOPPER, CAP_GRASP, CAP_DESK


class PickCapPass(BaseMetaAction):
    """取瓶塞：直拔（不取瓶、不旋转），倒放桌面旁侧。"""

    def _build_actions(self):
        e = self.engine
        cx, cy, _ = CAP_GRASP
        dx, dy, _ = CAP_DESK
        return [
            mv(e, (cx, cy, H)),                      # ① 高位接近瓶口
            mv(e, CAP_GRASP),                        # ② 下探瓶口抓塞
            hold(e, SETTLE),                         # ③ 到点 settle
            grip(e, GRIP_STOPPER, 60),               # ④ 原地合爪夹紧（task: attached）
            mv(e, (cx, cy, 0.93), 5),                # ⑤ 垂直直拔提出 + 停顿
            mv(e, (dx, dy, 0.93)),                   # ⑥ 平移旁侧
            mv(e, (dx, dy, 0.877)),                  # ⑦ 落座旁侧（task 释放时写桌面）
            grip(e, GRIP_OPEN, 25),                  # ⑧ 开爪释放（task: released → 倒放桌面）
            mv(e, (dx, dy, H)),                      # ⑨ 归位
        ]
