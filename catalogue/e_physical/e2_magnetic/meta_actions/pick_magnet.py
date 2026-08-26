"""元动作 ①：夹取条形磁铁 → 缓慢靠近表面皿样品上方做磁性检测（E2 全部动作）。

磁铁 100×15×15mm 平放（长轴 X），手指朝下（引擎默认朝向，非 ORIENT_FWD）垂直下探夹
±Y 面；持握 = 纯平移跟随（磁铁 translate=底中心 = tool_center + (0,0,-0.03)）。检测 =
磁铁降到 MAGNET_DETECT_GRASP_Z（底 0.82，粉顶 0.8086 上 ~11mm）停留 MAGNET_DETECT_DWELL
（task 期间驱动磁性颗粒被吸起动画）。

朝向关键（2026-08-26 用户报「马上抓到磁铁就跳」）：磁铁是贴台面的扁平 15mm 物体，抓点
z 仅 0.83（近台面低 z）。手指朝下竖直（长 57mm）可从容罩住磁铁，是引擎自然朝向、低 z
不奇异；ORIENT_FWD 横夹（手指 +X）在低 z 近奇异 → Lula IK 落 home 分支跳变，故弃横夹改竖夹。

轨迹（9 段，用户要求「多安排中间节点保证不穿模」）：
  ① 高位磁铁上方 H（安全高度）
  ② 降到预抓点（磁铁正上方 0.90，中间节点，避免斜插）
  ③ 垂直下探到抓点 0.83（手指朝下指尖 0.805 罩住磁铁上段，不再悬空夹空气）
  ④ 合爪夹磁铁
  ⑤ 提回预抓点（先垂直提离，不横向扫）
  ⑥ 提到安全高度 H
  ⑦ 水平移到表面皿正上方 H（跨障碍高位平移）
  ⑧ 垂直下降到检测高度
  ⑨ 检测停留（磁性颗粒吸起动画）
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, GRIP_MAGNET, MAGNET_XY, MAGNET_GRASP,
                        MAGNET_PRE_Z, DISH_XY, MAGNET_DETECT_GRASP_Z, MAGNET_DETECT_DWELL)


class PickMagnet(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        mx, my = MAGNET_XY
        dx, dy = DISH_XY
        mt_high = (mx, my, H)              # 磁铁上方高位
        mt_pre = (mx, my, MAGNET_PRE_Z)    # 预抓点（磁铁正上方）
        dt_high = (dx, dy, H)              # 皿上方高位
        det = (dx, dy, MAGNET_DETECT_GRASP_Z)
        return [
            mv(e, mt_high),                  # ① 高位磁铁上方（手指朝下，默认朝向）
            mv(e, mt_pre),                   # ② 降到预抓点（中间节点）
            mv(e, MAGNET_GRASP),             # ③ 垂直下探抓点
            grip(e, GRIP_MAGNET, 60),        # ④ 合爪夹磁铁
            mv(e, mt_pre),                   # ⑤ 提回预抓点
            mv(e, mt_high),                  # ⑥ 提到安全高度
            mv(e, dt_high),                  # ⑦ 水平移皿上方
            mv(e, det),                      # ⑧ 垂直下降检测高度
            hold(e, MAGNET_DETECT_DWELL),    # ⑨ 检测停留
        ]
