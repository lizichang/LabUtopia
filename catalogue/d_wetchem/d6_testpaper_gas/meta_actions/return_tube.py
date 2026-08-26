"""元动作 ④：试管归位（D6 收尾）。

逆着 MoveTubeUnderPaper 的侧面横夹路线：检测完提起 → 移回试管架上方 → 降到抓点 x 偏 -X 侧
→ 水平移 +X 入管身中心（试管落回架孔）→ 松爪放回 → 水平移 -X 退出 → 撤离高位。
全程手指朝前 ORIENT_FWD（与 ② 抓取一致，试管保持竖直静止朝向）；持握期间试管 translate=底中心
= tool_center + (0,0,-0.139)，降回抓点 0.945 时试管已回 rest 0.806（零跳变），松爪即释放不动。

轨迹（9 段，含中间节点）：
  ① 提起（检测位上方 H）→ ② 移试管架上方 H → ③ 降到预放点(1.02) → ④ 降到抓点高度 x 偏 -X 侧
  → ⑤ 水平移 +X 入管身中心（试管回架孔）→ ⑥ 松爪放回 → ⑦ 水平移 -X 退出（避竖直直拉穿模）
  → ⑧ 提回预放点 → ⑨ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD, TUBE_XY, TUBE_GRASP,
                        TUBE_APPROACH_DX, TUBE_PRE_Z, PAPER_WET_XY)


class ReturnTube(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        wx, wy = PAPER_WET_XY
        ax = tx - TUBE_APPROACH_DX          # 侧面接近 x（管身 -X 壁外侧）
        w_high = (wx, wy, H)
        t_high = (tx, ty, H)
        t_pre = (tx, ty, TUBE_PRE_Z)
        return [
            mv(e, w_high, orient=ORIENT_FWD),                   # ① 提起（检测位上方 H）
            mv(e, t_high, orient=ORIENT_FWD),                   # ② 移试管架上方 H
            mv(e, t_pre, orient=ORIENT_FWD),                    # ③ 降到预放点（中间节点）
            mv(e, (ax, ty, TUBE_GRASP[2]), orient=ORIENT_FWD),  # ④ 降到抓点高度 x 偏 -X 侧
            mv(e, TUBE_GRASP, orient=ORIENT_FWD),               # ⑤ 水平移 +X 入管身中心（试管回架孔）
            grip(e, GRIP_OPEN, 25),                             # ⑥ 松爪放回
            mv(e, (ax, ty, TUBE_GRASP[2]), orient=ORIENT_FWD),  # ⑦ 水平移 -X 退出（避竖直直拉穿模）
            mv(e, t_pre, orient=ORIENT_FWD),                    # ⑧ 提回预放点
            mv(e, t_high, orient=ORIENT_FWD),                   # ⑨ 撤离回 H
        ]
