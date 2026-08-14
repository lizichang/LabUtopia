"""元动作 ①：取样滴管 吸样品液 → 滴入试管（SAMPLE_PASS）。

V7 步 1-4：抓取样滴管 → 上提到样品瓶口 → 瓶口挤胶头排空气 → 浸入液面吸液
→ 上提到试管口 → 挤胶头滴液 → 放回滴管架。加酸滴管（ACID_PASS）下轮补。

2026-08-14 重构（用户反馈）：不再「抓→滴→放回」重复 N 遍，而是**抓一次**→循环
「吸液-滴液」cfg.sample_cycles 遍→**放回一次**（中途不松开滴管）。同一次持握内
task 生命周期靠瓶口/试管口 TCP 位置区分各遍（见 task.py dropped 再挤 → squeezed）。

夹爪开度（用户反馈"移动时开合应正好是胶头直径"）：移动/持握全程
= GRIP_DROPPER = GRIP_ASPIRATE = 0.011（胶头 Ø11mm，正好贴上不陷不隔）；
只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.011 再移动。

几何（2026-08-14 pxr 实测 d3l_acid_reagent.usd）：滴管立插架 2 排左孔，尖嘴底
z=0.806，抓点在胶头顶 z=0.936（架顶 0.917 之上）。持握 = TCP + HELD_OFFSET
(0,0,-0.13) 使尖嘴 0.13m 吊在夹爪下方（task.py 纯平移保竖立，flametest 同款）
——TCP z = 尖嘴 z + 0.13。整段手指朝下、滴管保持竖立，只做垂直下探/上提 + 高位
水平平移。

轨迹（TCP 世界坐标，手指默认朝下；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((sx,sy,H))
  ② 垂直下探抓点     mv(DROP_SAMPLE_GRASP)      # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)     # task 检测 attached
  ④ 垂直提出         mv((sx,sy,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv(BOTTLE_SQUEEZE_TCP)     # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)     # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(SAMPLE_DIP_TCP)         # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)    # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移试管上 mv((tx,ty,H))
    H 下探入管口     mv(TUBE_DROP_TCP)          # 尖嘴 0.949 入管口 0.9593 下
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)     # task: dropped → TubeDrops 显
    J 松回持握宽     grip(GRIP_DROPPER, 20)     # 提管口前松回胶头直径（移动全程无缝隙）
    K 垂直提出       mv((tx,ty,H)) ]
  ⑮ 高位回架         mv((sx,sy,H))
  ⑯ 下探放回         mv(DROP_SAMPLE_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)        # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((sx,sy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_SAMPLE_XY, DROP_SAMPLE_GRASP,
                        SAMPLE_BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SAMPLE_DIP_TCP,
                        TUBE_XY, TUBE_DROP_TCP)


class SamplePass(BaseMetaAction):
    """抓取样滴管 → 循环「吸液-滴液」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        sx, sy = DROP_SAMPLE_XY
        bx, by = SAMPLE_BOTTLE_XY
        tx, ty = TUBE_XY
        actions = [
            # —— 抓取样滴管（架 2 排左孔，尖嘴已在孔内；只抓这一次）——
            mv(e, (sx, sy, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_SAMPLE_GRASP),            # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (sx, sy, H), 5),               # ④ 垂直提出试管架
        ]
        cycle = [
            # —— 上提到样品瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸液 ——
            mv(e, (bx, by, H)),                  # A 高位平移到样品瓶上方
            mv(e, BOTTLE_SQUEEZE_TCP),           # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, SAMPLE_DIP_TCP),               # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # E 松胶头吸液（回胶头直径=持握宽）
            mv(e, (bx, by, H)),                  # F 垂直提出液面
            # —— 移到试管口滴液 ——
            mv(e, (tx, ty, H)),                  # G 高位平移到试管上方
            mv(e, TUBE_DROP_TCP),                # H 下探入管口（尖嘴 0.949）
            grip(e, GRIP_SQUEEZE, 40),           # I 挤胶头滴液
            grip(e, GRIP_DROPPER, 20),           # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H)),                  # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (sx, sy, H)),                  # ⑮ 高位回架上方
            mv(e, DROP_SAMPLE_GRASP),            # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑰ 松开释放（task: released → rest）
            mv(e, (sx, sy, H)),                  # ⑱ 垂直归位
        ]
        return actions
