"""元动作 ②：撤回磁铁归位（E2 收尾）。

逆着 PickMagnet 的路线：检测完提起 → 移回磁铁上方 → 降回抓点 → 松爪放回 → 撤离。
手指朝下（默认朝向）全程无手腕翻转；磁铁持握期间 translate=底中心 = tool_center +
(0,0,-0.03)，降回抓点 0.83 时磁铁已回 rest 0.80（零跳变），松爪即释放不动。

轨迹（7 段，含中间节点）：
  ① 提起（皿上方 H）→ ② 移磁铁上方 H → ③ 降到预放点 0.90 → ④ 垂直降回抓点 0.83
  → ⑤ 松爪放回 → ⑥ 提回预放点 → ⑦ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, MAGNET_XY, MAGNET_GRASP, MAGNET_PRE_Z, DISH_XY


class ReturnMagnet(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        mx, my = MAGNET_XY
        dx, dy = DISH_XY
        dt_high = (dx, dy, H)
        mt_high = (mx, my, H)
        mt_pre = (mx, my, MAGNET_PRE_Z)
        return [
            mv(e, dt_high),                  # ① 提起（皿上方 H）
            mv(e, mt_high),                  # ② 移磁铁上方 H
            mv(e, mt_pre),                   # ③ 降到预放点（中间节点）
            mv(e, MAGNET_GRASP),             # ④ 垂直降回抓点（磁铁已回 rest 位）
            grip(e, GRIP_OPEN, 25),          # ⑤ 松爪放回
            mv(e, mt_pre),                   # ⑥ 提回预放点
            mv(e, mt_high),                  # ⑦ 撤离回 H
        ]
