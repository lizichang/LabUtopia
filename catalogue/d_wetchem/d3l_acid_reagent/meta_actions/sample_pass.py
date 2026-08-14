"""元动作 ①：取样滴管 吸样品液 → 滴入试管（SAMPLE_PASS）。

V7 步 1-4：抓取样滴管 → 上提到样品瓶口 → 瓶口挤胶头排空气 → 浸入液面吸液
→ 上提到试管口 → 挤胶头滴液 → 放回滴管架。加酸滴管（ACID_PASS）下轮补。

几何（2026-08-14 pxr 实测 d3l_acid_reagent.usd）：滴管立插架 2 排左孔，尖嘴底
z=0.806，抓点在胶头顶 z=0.936（架顶 0.917 之上）。持握 _T_HELD_DROPPER 使尖嘴
0.13m 吊在夹爪下方（见 task.py）——TCP z = 尖嘴 z + 0.13。整段手指朝下、滴管保
持竖立，只做垂直下探/上提 + 高位水平平移。

轨迹（TCP 世界坐标，手指默认朝下）：
  ① 高位接近滴管架   mv((sx,sy,H))
  ② 垂直下探抓点     mv(DROP_SAMPLE_GRASP)      # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)     # task 检测 attached
  ④ 垂直提出         mv((sx,sy,H), 5)
  ⑤ 高位平移瓶上     mv((bx,by,H))
  ⑥ 下探到瓶口       mv(BOTTLE_SQUEEZE_TCP)     # 尖嘴贴瓶口 rim（z 0.875）
  ⑦ 挤胶头排空气     grip(GRIP_SQUEEZE, 30)     # task: squeezed
  ⑧ 下探浸液         mv(SAMPLE_DIP_TCP)         # 尖嘴 0.830 入液面 0.840 下
  ⑨ 松胶头吸液       grip(GRIP_ASPIRATE, 40)    # task: filled → DropperFill 显
  ⑩ 垂直提出液面     mv((bx,by,H))
  ⑪ 高位平移试管上   mv((tx,ty,H))
  ⑫ 下探入管口       mv(TUBE_DROP_TCP)          # 尖嘴 0.949 入管口 0.9593 下
  ⑬ 挤胶头滴液       grip(GRIP_SQUEEZE, 40)     # task: dropped → TubeDrops 显
  ⑭ 垂直提出         mv((tx,ty,H))
  ⑮ 高位回架         mv((sx,sy,H))
  ⑯ 下探放回         mv(DROP_SAMPLE_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)        # task: released
  ⑱ 垂直归位         mv((sx,sy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_SAMPLE_XY, DROP_SAMPLE_GRASP,
                        SAMPLE_BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SAMPLE_DIP_TCP,
                        TUBE_XY, TUBE_DROP_TCP)


class SamplePass(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = DROP_SAMPLE_XY
        bx, by = SAMPLE_BOTTLE_XY
        tx, ty = TUBE_XY
        return [
            # —— 抓取样滴管（架 2 排左孔，尖嘴已在孔内）——
            mv(e, (sx, sy, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_SAMPLE_GRASP),            # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧
            mv(e, (sx, sy, H), 5),               # ④ 垂直提出试管架
            # —— 上提到样品瓶口，排空气 ——
            mv(e, (bx, by, H)),                  # ⑤ 高位平移到样品瓶上方
            mv(e, BOTTLE_SQUEEZE_TCP),           # ⑥ 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # ⑦ 挤胶头排空气
            # —— 浸入液面吸液 ——
            mv(e, SAMPLE_DIP_TCP),               # ⑧ 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # ⑨ 松胶头吸液
            mv(e, (bx, by, H)),                  # ⑩ 垂直提出液面
            # —— 移到试管口滴液 ——
            mv(e, (tx, ty, H)),                  # ⑪ 高位平移到试管上方
            mv(e, TUBE_DROP_TCP),                # ⑫ 下探入管口（尖嘴 0.949）
            grip(e, GRIP_SQUEEZE, 40),           # ⑬ 挤胶头滴液
            mv(e, (tx, ty, H)),                  # ⑭ 垂直提出
            # —— 放回滴管架 ——
            mv(e, (sx, sy, H)),                  # ⑮ 高位回架上方
            mv(e, DROP_SAMPLE_GRASP),            # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑰ 松开释放
            mv(e, (sx, sy, H)),                  # ⑱ 垂直归位
        ]
