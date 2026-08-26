"""元动作 ②：取滴管 → 吸样（瓶内吸液）→ 滴样（棱镜面）。

与 d4l SamplePass 同构（一次持握：抓滴管 → 瓶口挤空气 → 浸液吸液 → 滴样 → 放回，
中途不松开），A1 差异：滴样目标是**折光仪棱镜面**（平面，非试管口），液体是待测
有机样（瓶内液面 0.840）。夹爪开度：移动/持握全程 = GRIP_DROPPER = GRIP_ASPIRATE
= 0.0055（贴合胶头面），只在排空气/滴样瞬间挤到 GRIP_SQUEEZE=0.002。

轨迹（TCP 世界坐标，手指默认朝下）：
  ① 高位接近滴管架   mv((sx,sy,H))
  ② 垂直下探抓点     mv(DROP_GRASP)             # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)     # task 检测 attached
  ④ 垂直提出         mv((sx,sy,H), 5)
  ⑤ 高位平移瓶上     mv((bx,by,H))
  ⑥ 下探到瓶口       mv(BOTTLE_SQUEEZE_TCP)     # 尖嘴贴瓶口 rim（z 0.875）
  ⑦ 挤胶头排空气     grip(GRIP_SQUEEZE, 30)     # task: squeezed
  ⑧ 下探浸液         mv(SAMPLE_DIP_TCP)         # 尖嘴 0.830 入液面 0.840 下
  ⑨ 松胶头吸液       grip(GRIP_ASPIRATE, 40)    # task: filled → DropperFill 显
  ⑩ 垂直提出液面     mv((bx,by,H))
  ⑪ 高位平移棱镜上   mv((px,py,H))
  ⑫ 下探到棱镜上方   mv(PRISM_DROP_TCP)         # 尖嘴 0.9425 在棱镜顶 0.9175 上方 25mm
  ⑬ 挤胶头滴样       grip(GRIP_SQUEEZE, 40)     # task: dropped → 液滴坠落到棱镜 + PrismDrop 显
  ⑭ 松回持握宽       grip(GRIP_DROPPER, 20)
  ⑮ 垂直提出         mv((px,py,H))
  ⑯ 高位回架         mv((sx,sy,H))
  ⑰ 下探放回         mv(DROP_GRASP)
  ⑱ 松开释放         grip(GRIP_OPEN, 25)        # task: released → rest
  ⑲ 垂直归位         mv((sx,sy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_XY, DROP_GRASP,
                        BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SAMPLE_DIP_TCP,
                        PRISM_XY, PRISM_DROP_TCP)


class SamplePass(BaseMetaAction):
    """取滴管 → 吸样（瓶内）→ 滴样（棱镜面）→ 放回（一次持握，中途不松开）。"""

    def _build_actions(self):
        e = self.engine
        sx, sy = DROP_XY
        bx, by = BOTTLE_XY
        px, py = PRISM_XY
        return [
            # —— 取滴管（架左孔，尖嘴已在孔内）——
            mv(e, (sx, sy, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_GRASP),                   # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (sx, sy, H), 5),               # ④ 垂直提出试管架
            # —— 吸样：瓶口挤空气 → 浸液吸液 ——
            mv(e, (bx, by, H)),                  # ⑤ 高位平移到样品瓶上方
            mv(e, BOTTLE_SQUEEZE_TCP),           # ⑥ 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # ⑦ 挤胶头排空气
            mv(e, SAMPLE_DIP_TCP),               # ⑧ 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # ⑨ 松胶头吸液（回胶头直径=持握宽）
            mv(e, (bx, by, H)),                  # ⑩ 垂直提出液面
            # —— 滴样：棱镜上方挤胶头滴样 ——
            mv(e, (px, py, H)),                  # ⑪ 高位平移到棱镜上方
            mv(e, PRISM_DROP_TCP),               # ⑫ 下探到棱镜顶上方 25mm（尖嘴 0.9425）
            grip(e, GRIP_SQUEEZE, 40),           # ⑬ 挤胶头滴样（task: dropped → 液滴坠落到棱镜）
            grip(e, GRIP_DROPPER, 20),           # ⑭ 松回持握宽（提棱镜/移动全程=胶头直径）
            mv(e, (px, py, H)),                  # ⑮ 垂直提出
            # —— 放回滴管架 ——
            mv(e, (sx, sy, H)),                  # ⑯ 高位回架上方
            mv(e, DROP_GRASP),                   # ⑰ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑱ 松开释放（task: released → rest）
            mv(e, (sx, sy, H)),                  # ⑲ 垂直归位
        ]
