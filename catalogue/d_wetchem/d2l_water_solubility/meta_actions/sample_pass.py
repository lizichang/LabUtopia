"""元动作 ①：取样滴管 吸样品液 → 滴入试管（SAMPLE_PASS）。

V7 步 1-4：抓取样滴管 → 上提到样品瓶口 → 瓶口挤胶头排空气 → 浸入液面吸液
→ 上提到试管口 → 挤胶头滴液 → 放回滴管架。

2026-08-14 重构（用户反馈）：不再「抓→滴→放回」重复 N 遍，而是**抓一次**→循环
「吸液-滴液」cfg.sample_cycles 遍→**放回一次**（中途不松开滴管）。同一次持握内
task 生命周期靠瓶口/试管口 TCP 位置区分各遍（见 task.py dropped 再挤 → squeezed）。

夹爪开度（用户 2026-08-14 实测：0.011 太松、减半正好）：移动/持握全程
= GRIP_DROPPER = GRIP_ASPIRATE = 0.0055（≈ 指间距一半 ≈ 胶头 Ø11mm/2，正好贴胶头面）；
只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055 再移动。

几何（2026-08-24 pxr 实测 d2l_water_solubility.usd）：滴管立插架后排左孔，尖嘴底
z=0.806，抓点在胶头顶 z=0.936（架顶 0.917 之上）。持握 = TCP + HELD_OFFSET
(0,0,-0.13) 使尖嘴 0.13m 吊在夹爪下方（task.py 纯平移保竖立，flametest 同款）
——TCP z = 尖嘴 z + 0.13。整段手指朝下、滴管保持竖立，只做垂直下探/上提 + 高位
水平平移。

轨迹（TCP 世界坐标，手指默认朝下；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 起点处竖直调高   mv((p0.xy, H))             # 远离架先定高度（纯直线，xy=起点）
  ② 水平平移到架上方 mv((sx,sy,H))              # z=H 不变，纯水平直线
  ③ 垂直下探抓点     mv(DROP_SAMPLE_GRASP)      # 尖嘴已在孔内，一次干净下探
  ④ 合爪夹紧         grip(GRIP_DROPPER, 60)     # task 检测 attached
  ⑤ 垂直提出         mv((sx,sy,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv(BOTTLE_SQUEEZE_TCP)     # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)     # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(SAMPLE_DIP_TCP)         # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)    # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移试管上 mv((tx,ty,H))
    H 下探到管口上方 mv(TUBE_DROP_TCP)          # 尖嘴 0.984 在管口 0.9593 上方 25mm
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)     # task: dropped → 液滴动画坠落 + TubeDrops 显
    J 松回持握宽     grip(GRIP_DROPPER, 20)     # 提管口前松回胶头直径（移动全程无缝隙）
    K 垂直提出       mv((tx,ty,H)) ]
  ⑤ 高位回架         mv((sx,sy,H))
  ⑥ 下探放回         mv(DROP_SAMPLE_GRASP)
  ⑦ 松开释放         grip(GRIP_OPEN, 25)        # task: released（末遍滴完才回架松）
  ⑧ 垂直归位         mv((sx,sy,H))
"""
import numpy as np
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
        actions = []
        # —— 抓取样滴管（架后排左孔，尖嘴已在孔内；只抓这一次）——
        # 接近拆成「起点处竖直到 H → 水平平移到架上方」两段纯直线，再做一次垂直下探。
        # 旧版一步 mv((sx,sy,H)) 从初始姿态到架上方是 x/y/z 三轴全变 → 单次 IK 斜线
        # （斜着下降接近），视频里像「先下去一次没夹、再下去才夹」。分段后接近全程
        # 无斜线：先定高度（远离架）、再水平到位（z=H）、最后一次干净下探合爪。
        p0 = None
        try:
            p0 = np.asarray(e.fk_pose(e.ik_home)[0], dtype=float)   # 初始 TCP（ik_home 姿态）
        except Exception:
            pass
        if p0 is not None:
            actions.append(mv(e, (p0[0], p0[1], H)))    # ① 起点处竖直升/降到 H（xy=起点，纯直线）
            actions.append(mv(e, (sx, sy, H)))          # ② 水平平移到架上方（z=H 不变，纯直线）
        else:
            actions.append(mv(e, (sx, sy, H)))
        actions += [
            mv(e, DROP_SAMPLE_GRASP),                   # ③ 一次垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),                  # ④ 合爪夹紧（开合=胶头直径）
            mv(e, (sx, sy, H), 5),                      # ⑤ 垂直提出试管架
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
            mv(e, TUBE_DROP_TCP),                # H 下探到管口上方 25mm（尖嘴 0.984，液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),           # I 挤胶头滴液
            grip(e, GRIP_DROPPER, 20),           # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H)),                  # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (sx, sy, H)),                  # ⑥ 高位回架上方
            mv(e, DROP_SAMPLE_GRASP),            # ⑦ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑧ 松开释放（task: released → rest）
            mv(e, (sx, sy, H)),                  # ⑨ 垂直归位
        ]
        return actions
