"""元动作 ①：抓滴管 吸药品瓶液 → 滴入燃烧匙碗（DRIP_SPOON_PASS）。

用户指令「模仿 d3l 往燃烧匙里加药品」= d3l SamplePass 骨架逐字换坐标：
抓滴管 → 循环「药品瓶吸液 → 燃烧匙碗口上方挤胶头滴液」cfg.drip_cycles 遍 →
放回滴管架（一次持握，中途不松开；task 生命周期靠瓶口/碗口 TCP 位置区分各遍）。

夹爪开度同 d3l：移动/持握全程 = GRIP_DROPPER = GRIP_ASPIRATE = 0.0055（贴合胶头
Ø11mm 面）；只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055 再移动。

几何（gen_c4_scene.py verify 实测）：滴管立插架右列最后一排（第7排）孔
(0.7385,0.5395)，尖嘴底
z=0.806，抓点 0.936（架顶 0.917 之上）。持握 = TCP + HELD_OFFSET(0,0,-0.13) 使尖嘴
0.13m 吊在夹爪下方（纯平移保竖立）。药品瓶 (0.50,0.55) 口 rim 0.870 液面 0.840
（瓶盖摘下倒放瓶旁）；
燃烧匙碗 (0.636,0.3093) 口 0.8068。整段手指朝下、滴管保持竖立。

轨迹（TCP 世界坐标，手指默认朝下；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((dx,dy,H))
  ② 垂直下探抓点     mv(DROP_GRASP)              # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)      # task 检测 attached
  ④ 垂直提出         mv((dx,dy,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv(BOTTLE_SQUEEZE_TCP)       # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)       # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(SAMPLE_DIP_TCP)           # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)      # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移碗上   mv((sx,sy,H))
    H 下探到碗口上方 mv(SPOON_DROP_TCP)           # 尖嘴 0.8318 在碗口 0.8068 上方 25mm
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)       # task: dropped → 液滴动画坠落 + SpoonLiquid 显
    J 松回持握宽     grip(GRIP_DROPPER, 20)       # 提碗口前松回胶头直径（移动全程无缝隙）
    K 垂直提出       mv((sx,sy,H)) ]
  ⑮ 高位回架         mv((dx,dy,H))
  ⑯ 下探放回         mv(DROP_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)         # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((dx,dy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_XY, DROP_GRASP,
                        SAMPLE_BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SAMPLE_DIP_TCP,
                        SPOON_XY, SPOON_DROP_TCP)


class DripSpoonPass(BaseMetaAction):
    """抓滴管 → 循环「药品瓶吸液-燃烧匙碗滴入」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        dx, dy = DROP_XY
        bx, by = SAMPLE_BOTTLE_XY
        sx, sy = SPOON_XY
        actions = [
            # —— 抓滴管（架右列最后一排孔，尖嘴已在孔内；只抓这一次）——
            mv(e, (dx, dy, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_GRASP),                   # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (dx, dy, H), 5),               # ④ 垂直提出试管架
        ]
        cycle = [
            # —— 上提到药品瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸液 ——
            mv(e, (bx, by, H)),                  # A 高位平移到药品瓶上方
            mv(e, BOTTLE_SQUEEZE_TCP),           # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, SAMPLE_DIP_TCP),               # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # E 松胶头吸液（回胶头直径=持握宽）
            mv(e, (bx, by, H)),                  # F 垂直提出液面
            # —— 移到燃烧匙碗口滴液 ——
            mv(e, (sx, sy, H)),                  # G 高位平移到燃烧匙上方
            mv(e, SPOON_DROP_TCP),               # H 下探到碗口上方 25mm（尖嘴 0.8318，液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),           # I 挤胶头滴液
            grip(e, GRIP_DROPPER, 20),           # J 松回持握宽（提碗口/移动全程=胶头直径）
            mv(e, (sx, sy, H)),                  # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (dx, dy, H)),                  # ⑮ 高位回架上方
            mv(e, DROP_GRASP),                   # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑰ 松开释放（task: released → rest）
            mv(e, (dx, dy, H)),                  # ⑱ 垂直归位
        ]
        return actions
