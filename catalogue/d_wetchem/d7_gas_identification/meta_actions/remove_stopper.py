"""元动作 ⑤：拔橡皮塞归位（D7 收尾，逆向 PickStopper）。

橡皮塞已塞紧产气试管口（塞底 0.950）。侧面横夹（手指朝前 ORIENT_FWD）：夹爪先到塞体 -X 侧
再水平移 +X 夹住塞体 ±Y 面（塞顶中心有导气管竖段 0.974..1.005，避正上方下探）。

拔塞 = 塞体整体平移 (0.260,0.041,0.950) → (0.40,0.06,0.80)，导气管末端随之
(0.300,0.200,0.955) → (0.44,0.219,0.805) 回到桌面起始位（不旋转）。

轨迹（11 段，全程 orient=ORIENT_FWD）：
  ① 塞体 -X 侧高位 → ② 降抓点高度 -X 侧 → ③ 水平移 +X 夹塞体 → ④ 合爪横夹
  → ⑤ 提预抓点(1.05，末端 1.039 离管口) → ⑥ 提安全高位 H → ⑦ 移桌面起始位上方 H
  → ⑧ 降到桌面抓点(0.816，塞底 0.80) → ⑨ 松爪放回 → ⑩ -X 退开 → ⑪ 提回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_STOPPER, ORIENT_FWD, STOPPER_INITIAL_XY,
                        STOPPER_APPROACH_DX, STOPPER_INITIAL_GRASP, STOPPER_PLUG_GRASP,
                        STOPPER_PRE_Z, GAS_TUBE_XY)


class RemoveStopper(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = STOPPER_INITIAL_XY
        gx, gy = GAS_TUBE_XY
        ax = gx - STOPPER_APPROACH_DX           # 塞体 -X 侧接近
        s_high = (sx, sy, H)
        g_high = (gx, gy, H)
        return [
            mv(e, (ax, gy, H), orient=ORIENT_FWD),                    # ① 塞体 -X 侧高位
            mv(e, (ax, gy, STOPPER_PLUG_GRASP[2]), orient=ORIENT_FWD),  # ② 降抓点高度 -X 侧
            mv(e, STOPPER_PLUG_GRASP, orient=ORIENT_FWD),             # ③ 水平移 +X 夹塞体
            grip(e, GRIP_STOPPER, 60),                                # ④ 合爪横夹
            mv(e, (gx, gy, STOPPER_PRE_Z), orient=ORIENT_FWD),        # ⑤ 提预抓点（末端 1.039 离管口）
            mv(e, g_high, orient=ORIENT_FWD),                         # ⑥ 提安全高位 H
            mv(e, s_high, orient=ORIENT_FWD),                         # ⑦ 移桌面起始位上方 H
            mv(e, STOPPER_INITIAL_GRASP, orient=ORIENT_FWD),          # ⑧ 降到桌面抓点（塞底 0.80）
            grip(e, GRIP_OPEN, 25),                                   # ⑨ 松爪放回
            mv(e, (sx - STOPPER_APPROACH_DX, sy, STOPPER_INITIAL_GRASP[2]), orient=ORIENT_FWD),  # ⑩ -X 退开
            mv(e, (sx - STOPPER_APPROACH_DX, sy, H), orient=ORIENT_FWD),  # ⑪ 提回 H
        ]
