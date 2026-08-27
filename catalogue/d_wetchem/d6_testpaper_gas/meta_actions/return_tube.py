"""元动作 ④：试管归位（D6 收尾）。

逆着 MoveTubeUnderPaper 的侧面横夹 + 检测位侧入路线（2026-08-27 用户：试管不能从试纸正上方
直降放回，须沿「下潜点」同路径原路返回）：检测位水平 -X 滑出到下潜点（仍在试纸平面之下）→
垂直升到高位（清试纸/试纸夹）→ 移回试管架上方 → 竖直降回抓点（试管垂直落回架孔）→ 松爪
放回 → 竖直提起撤离。

2026-08-27 用户「放回时突然向后拉了一下导致穿模」：原 ⑤ 降抓点高度 x 偏 -X 侧、⑥ 水平移
+X 入孔——但试管已被夹持（translate 逐帧跟随夹爪），这两步会拖着试管先 -X 再 +X 横拉，
管身顶段（0.917..0.959）横穿试管架顶板 → 穿模。改为纯竖直下探（d2s return_spatula 同款）：
抓点 0.9453 高于架顶板 0.917，ORIENT_FWD 手指水平不碰顶板，试管垂直入孔零跳变。

全程手指朝前 ORIENT_FWD（与 ② 抓取一致，试管保持竖直静止朝向）；持握期间试管 translate=底中心
= tool_center + (0,0,-0.139)，降回抓点 0.945 时试管已回 rest 0.806（零跳变），松爪即释放不动。

轨迹（8 段，竖直回架）：
  ① 水平 -X 滑出（检测位 → 下潜点，试纸平面下）→ ② 垂直升高位 → ③ 移试管架上方 H
  → ④ 降预放点(1.02) → ⑤ 竖直降回抓点（试管垂直入架孔）→ ⑥ 松爪放回
  → ⑦ 竖直提起（清管口/架顶）→ ⑧ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD, TUBE_XY, TUBE_GRASP,
                        TUBE_PRE_Z, TUBE_DESCEND_XY, TUBE_UNDER_PAPER_TCP)


class ReturnTube(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        t_high = (tx, ty, H)
        t_pre = (tx, ty, TUBE_PRE_Z)
        dx, dy = TUBE_DESCEND_XY
        d_high = (dx, dy, H)
        d_low = (dx, dy, TUBE_UNDER_PAPER_TCP[2])
        return [
            mv(e, d_low, orient=ORIENT_FWD),      # ① 水平 -X 滑出（检测位 → 下潜点）
            mv(e, d_high, orient=ORIENT_FWD),     # ② 垂直升高位（清试纸/试纸夹）
            mv(e, t_high, orient=ORIENT_FWD),     # ③ 移试管架上方 H
            mv(e, t_pre, orient=ORIENT_FWD),      # ④ 降到预放点
            mv(e, TUBE_GRASP, orient=ORIENT_FWD), # ⑤ 竖直降回抓点（试管垂直入架孔，零跳变）
            grip(e, GRIP_OPEN, 25),               # ⑥ 松爪放回
            mv(e, t_pre, orient=ORIENT_FWD),      # ⑦ 竖直提起（清管口/架顶）
            mv(e, t_high, orient=ORIENT_FWD),     # ⑧ 撤离回 H
        ]
