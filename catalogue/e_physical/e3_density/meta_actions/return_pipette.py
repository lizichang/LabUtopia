"""元动作 ④：把移液管放回移液管架。

放回时抓点仍是尖端上方 0.09m（z=0.91）：夹爪降到该处时，尖端恰回到架孔底 0.82，
开爪后移液管落回架孔。

轨迹（与放液同款「先抬升 → 平行 → 下降」，避免从筒口斜着甩回架孔穿模）：
  ① 先竖直抬升到高位（筒口上方，冗余保险）→ ② 平行移动到架正上方（仅 x 变，直线）
  → ③ 竖直下降到抓点 → ④ 开爪 GRIP_OPEN → ⑤ 竖直撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, ORIENT_FWD, CYL_XY, PIPE_GRASP


class ReturnPipette(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        cx, cy = CYL_XY           # 放液后停在筒口上方 (0.40, 0.17)
        px, py, _ = PIPE_GRASP    # 架孔抓点 (0.22, 0.17, 0.91)
        return [
            mv(e, (cx, cy, H), orient=ORIENT_FWD),   # ① 先竖直抬升到高位
            mv(e, (px, py, H), orient=ORIENT_FWD),   # ② 平行移动到架正上方
            mv(e, PIPE_GRASP, orient=ORIENT_FWD),    # ③ 竖直下降到抓点
            grip(e, GRIP_OPEN, 60),                  # ④ 开爪放回
            mv(e, (px, py, H), orient=ORIENT_FWD),   # ⑤ 竖直撤离
        ]
