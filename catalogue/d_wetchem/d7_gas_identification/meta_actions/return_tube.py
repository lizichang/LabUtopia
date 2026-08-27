"""元动作 ④：检验试管归位（D7 收尾，逆向 DipGasTube）。

逆着 ② 的「从下方接近」路线原路返回（2026-08-27 用户：从导气管上方直下/直上会穿模）：
下浸抓点先垂直降到下方抓点（管口向下脱出导气管末端）→ 平移出偏移位（管口仍低于末端，不穿桥）
→ 偏移位垂直升到高位 → 水平移回试管架上方 → 竖直降回抓点（试管垂直落回架孔零跳变）
→ 松爪放回 → 竖直提起 → **+X 侧退开撤离**（与 ② 的 +X 接近同侧，退向架外清空处）。

全程手指朝前 ORIENT_FWD；持握期间试管 translate=底中心 = tool_center + (0,0,-0.139)，降回
抓点 0.945 时试管已回 rest 0.806（零跳变），松爪即释放不动。

轨迹（10 段，竖直回架）：
  ① 垂直降到下方抓点(1.000，脱出末端) → ② 平移出偏移位(0.44,0.159，低于末端) → ③ 偏移位升 H
  → ④ 水平移回试管架上方 H → ⑤ 降预放点(1.02) → ⑥ 竖直降回抓点（试管垂直入架孔）
  → ⑦ 松爪放回 → ⑧ 竖直提起 → ⑨ +X 退开 → ⑩ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD, TEST_TUBE_XY, TUBE_GRASP,
                        TUBE_APPROACH_DX, TUBE_PRE_Z, DIP_XY, DIP_BELOW_Z,
                        DIP_APPROACH_XY)


class ReturnTube(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TEST_TUBE_XY
        dx, dy = DIP_XY
        ax = tx + TUBE_APPROACH_DX          # 管口 +X 侧退开（右列右侧架外清空）
        ox, oy = DIP_APPROACH_XY            # 下方退开偏移位（导气管桥 +Y 侧）
        t_high = (tx, ty, H)
        t_pre = (tx, ty, TUBE_PRE_Z)
        o_high = (ox, oy, H)
        o_below = (ox, oy, DIP_BELOW_Z)
        d_below = (dx, dy, DIP_BELOW_Z)
        return [
            mv(e, d_below, orient=ORIENT_FWD),      # ① 垂直降到下方抓点（脱出末端）
            mv(e, o_below, orient=ORIENT_FWD),      # ② 平移出偏移位（低于末端，不穿桥）
            mv(e, o_high, orient=ORIENT_FWD),       # ③ 偏移位垂直升 H（清导气管桥）
            mv(e, t_high, orient=ORIENT_FWD),       # ④ 水平移回试管架上方 H
            mv(e, t_pre, orient=ORIENT_FWD),        # ⑤ 降预放点
            mv(e, TUBE_GRASP, orient=ORIENT_FWD),   # ⑥ 竖直降回抓点（试管垂直入架孔）
            grip(e, GRIP_OPEN, 25),                 # ⑦ 松爪放回
            mv(e, t_pre, orient=ORIENT_FWD),        # ⑧ 竖直提起（清管口/架顶）
            mv(e, (ax, ty, TUBE_PRE_Z), orient=ORIENT_FWD),  # ⑨ +X 退开
            mv(e, (ax, ty, H), orient=ORIENT_FWD),  # ⑩ 撤离回 H
        ]
