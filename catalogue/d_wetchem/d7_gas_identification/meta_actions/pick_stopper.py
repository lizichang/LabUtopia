"""元动作 ①：夹橡皮塞（带导气管）→ 塞紧产气试管口（D7 气体鉴定）。

橡皮塞 rubber_stopper_delivery.usd 横放桌面 (0.40,0.06)，塞底 0.80、塞顶 0.824，塞顶中心
伸出玻璃导气管（短竖段 0.824..0.855 + 水平桥 +X+Y + 长竖段到末端 (0.44,0.219,0.805)）。
故橡皮塞**不能从正上方夹**（塞顶中心有导气管竖段），改侧面横夹（手指朝前 ORIENT_FWD，d2s
夹药匙同款）：夹爪先到塞体 -X 侧（避 +X 方向的导气管桥），再水平移 +X 夹住塞体 ±Y 面。

持握 = 纯平移跟随（橡皮塞 translate=塞底中心 = tool_center + (0,0,-0.016)）。塞紧 = 塞体整体
平移 (0.40,0.06,0.80) → (0.260,0.041,0.950)，导气管末端随之 (0.44,0.219,0.805) →
(0.300,0.200,0.955) 正好悬在检验试管空孔正前方（ΔX=40mm ΔY=159mm 资产内置，不旋转）。

轨迹（11 段，全程 orient=ORIENT_FWD）：
  ① 塞体 -X 侧高位 → ② 降到抓点高度 -X 侧 → ③ 水平移 +X 夹塞体 → ④ 合爪横夹
  → ⑤ 提预抓点(1.05) → ⑥ 提安全高位 H（末端 1.184 清架/管）→ ⑦ 移产气试管上方 H
  → ⑧ 降到塞紧抓点(0.966，塞底 0.950 沉入管口下 9mm) → ⑨ 松爪留塞 → ⑩ -X 退开
  → ⑪ 提回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_STOPPER, ORIENT_FWD, STOPPER_INITIAL_XY,
                        STOPPER_APPROACH_DX, STOPPER_INITIAL_GRASP, STOPPER_PLUG_GRASP,
                        STOPPER_PRE_Z, GAS_TUBE_XY)


class PickStopper(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = STOPPER_INITIAL_XY
        gx, gy = GAS_TUBE_XY
        ax = sx - STOPPER_APPROACH_DX           # 塞体 -X 侧接近（避 +X 导气管桥）
        s_high = (sx, sy, H)
        s_pre = (sx, sy, STOPPER_PRE_Z)
        g_high = (gx, gy, H)
        return [
            mv(e, (ax, sy, H), orient=ORIENT_FWD),                    # ① 塞体 -X 侧高位
            mv(e, (ax, sy, STOPPER_INITIAL_GRASP[2]), orient=ORIENT_FWD),  # ② 降抓点高度 -X 侧
            mv(e, STOPPER_INITIAL_GRASP, orient=ORIENT_FWD),          # ③ 水平移 +X 夹塞体
            grip(e, GRIP_STOPPER, 60),                                # ④ 合爪横夹
            mv(e, s_pre, orient=ORIENT_FWD),                          # ⑤ 提预抓点（末端 1.039 离桌）
            mv(e, s_high, orient=ORIENT_FWD),                         # ⑥ 提安全高位 H
            mv(e, g_high, orient=ORIENT_FWD),                         # ⑦ 移产气试管上方 H
            mv(e, STOPPER_PLUG_GRASP, orient=ORIENT_FWD),             # ⑧ 降到塞紧抓点（塞底 0.950）
            grip(e, GRIP_OPEN, 25),                                   # ⑨ 松爪留塞
            mv(e, (gx - STOPPER_APPROACH_DX, gy, STOPPER_PLUG_GRASP[2]), orient=ORIENT_FWD),  # ⑩ -X 退开
            mv(e, (gx - STOPPER_APPROACH_DX, gy, H), orient=ORIENT_FWD),  # ⑪ 提回 H
        ]
