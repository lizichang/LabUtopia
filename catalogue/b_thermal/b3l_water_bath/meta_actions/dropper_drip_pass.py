"""元动作 ①：取样滴管 吸样品液 → 滴入试管（DROPPER_DRIP_PASS）。

B3L 液体实验（用户逐字「不是挖粉末而是往试管里面滴加溶液（动作参考d3l），其他动作
直接复制」）——复刻 d3l sample_pass.py 的「抓取样滴管 → 瓶口挤胶头排空气 → 浸液吸液
→ 管口挤胶头滴液 → 放回滴管架」单滴管流程，仅换 B3L 布局坐标。

2026-08-14 d3l 重构（用户反馈）：不再「抓→滴→放回」重复 N 遍，而是**抓一次**→循环
「吸液-滴液」cfg.sample_cycles 遍→**放回一次**（中途不松开滴管）。同一次持握内
task 生命周期靠瓶口/试管口 TCP 位置区分各遍（见 task.py dropped 再挤 → squeezed）。

夹爪开度（用户 2026-08-14 实测：0.011 太松、减半正好）：移动/持握全程
= GRIP_DROPPER = GRIP_ASPIRATE = 0.0055（≈ 指间距一半 ≈ 胶头 Ø11mm/2，正好贴胶头面）；
只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055 再移动。

几何（2026-08-31 修）：滴管立插架近侧列第3排 (0.659,0.3209)，尖嘴底 z=0.806，抓点在
胶头顶 z=0.936（架顶 0.917 之上）。持握 = 矩阵 _T_HELD_DROPPER（d3s/B2 同款水平横夹，
task.py）：手指朝前 ORIENT_FWD 水平横夹竖直管身，尖嘴 0.13m 吊在夹爪下方。旧 (0.6993,
0.3608) 中心孔距底座 0.904m 手指朝下 IK 不可达（运行 log force-done）。样品瓶
(0.5365,0.20) 瓶口 0.870、瓶内液面 0.840。

轨迹（TCP 世界坐标，全程 orient=ORIENT_FWD 手指朝前；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((sx,sy,H))
  ② 垂直下探抓点     mv(DROPPER_GRASP)              # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)         # task 检测 attached
  ④ 垂直提出         mv((sx,sy,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv(BOTTLE_SQUEEZE_TCP)         # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)         # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(SOLUTION_DIP_TCP)           # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)        # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移试管上 mv((tx,ty,H))
    H 下探到管口上方 mv(TUBE_DROP_TCP)              # 尖嘴 0.984 在管口 0.9593 上方 25mm
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)         # task: dropped → 液滴动画坠落 + TestTubeLiquid 显
    J 松回持握宽     grip(GRIP_DROPPER, 20)         # 提管口前松回胶头直径（移动全程无缝隙）
    K 垂直提出       mv((tx,ty,H)) ]
  ⑮ 高位回架         mv((sx,sy,H))
  ⑯ 下探放回         mv(DROPPER_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)            # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((sx,sy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        ORIENT_FWD,
                        DROPPER_XY, DROPPER_GRASP,
                        SOLUTION_BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SOLUTION_DIP_TCP,
                        TUBE_XY, TUBE_DROP_TCP)


class DropperDripPass(BaseMetaAction):
    """抓取样滴管 → 循环「吸液-滴液」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        sx, sy = DROPPER_XY
        bx, by = SOLUTION_BOTTLE_XY
        tx, ty = TUBE_XY
        actions = [
            # —— 抓取样滴管（架近侧列第3排，尖嘴已在孔内；只抓这一次；手指朝前水平横夹）——
            mv(e, (sx, sy, H), orient=ORIENT_FWD),          # ① 高位接近滴管架上方
            mv(e, DROPPER_GRASP, orient=ORIENT_FWD),        # ② 下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),                      # ③ 合爪横着夹紧竖直管身（开合=胶头直径）
            mv(e, (sx, sy, H), 5, orient=ORIENT_FWD),       # ④ 竖直提出试管架
        ]
        cycle = [
            # —— 上提到样品瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸液 ——
            mv(e, (bx, by, H), orient=ORIENT_FWD),          # A 高位平移到样品瓶上方
            mv(e, BOTTLE_SQUEEZE_TCP, orient=ORIENT_FWD),   # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),                      # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, SOLUTION_DIP_TCP, orient=ORIENT_FWD),     # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),                     # E 松胶头吸液（回胶头直径=持握宽）
            mv(e, (bx, by, H), orient=ORIENT_FWD),          # F 垂直提出液面
            # —— 移到试管口滴液 ——
            mv(e, (tx, ty, H), orient=ORIENT_FWD),          # G 高位平移到试管上方
            mv(e, TUBE_DROP_TCP, orient=ORIENT_FWD),        # H 下探到管口上方 25mm（尖嘴 0.984，液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),                      # I 挤胶头滴液
            grip(e, GRIP_DROPPER, 20),                      # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H), orient=ORIENT_FWD),          # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (sx, sy, H), orient=ORIENT_FWD),          # ⑮ 高位回架上方
            mv(e, DROPPER_GRASP, orient=ORIENT_FWD),        # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),                         # ⑰ 松开释放（task: released → rest）
            mv(e, (sx, sy, H), orient=ORIENT_FWD),          # ⑱ 垂直归位
        ]
        return actions
