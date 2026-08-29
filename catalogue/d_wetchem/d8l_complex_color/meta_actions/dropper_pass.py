"""元动作：一支胶头滴管 吸液 → 滴入试管（DROPPER_PASS，三支滴管通用）。

复刻 d3l 的 SamplePass/AcidPass，但抽成**一支滴管的通用轨迹**：构造时传该滴管的
架孔 xy（dropper_xy）与所对瓶 xy（bottle_xy），抓点/瓶口/浸液/试管口的 z 全三支
相同（立放底面 0.806、瓶口 0.870、液面 0.840、试管口 0.9593）。三支滴管 =
DropperPass(DROP_SAMPLE_XY, SAMPLE_BOTTLE_XY) / (DROP_REAGENT1_XY, REAGENT1_BOTTLE_XY)
/ (DROP_REAGENT2_XY, REAGENT2_BOTTLE_XY)。task 靠 TCP 位置 + 关节开度区分哪支被持、
吸哪瓶、滴哪管（每支一个 _DropperLifecycle 句柄，见 task.py），元动作本身无需区分。

V7 步 1-4（每支滴管）：抓滴管 → 上提到瓶口 → 瓶口挤胶头排空气 → 浸入液面吸液
→ 上提到试管口 → 挤胶头滴液 → 放回滴管架。2026-08-14 重构（用户反馈）：不再
「抓→滴→放回」重复 N 遍，而是**抓一次**→循环「吸液-滴液」cycles 遍→**放回一次**
（中途不松开滴管）。同一次持握内 task 生命周期靠瓶口/试管口 TCP 位置区分各遍。

夹爪开度（d3l 用户实测 0.0055 正好贴胶头面）：移动/持握全程 = GRIP_DROPPER =
GRIP_ASPIRATE = 0.0055；只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055。

几何：滴管立插架孔，尖嘴底 z=0.806，抓点在胶头顶 z=0.936（架顶 0.917 之上）。持握
= TCP + HELD_OFFSET (0,0,-0.13) 使尖嘴 0.13m 吊在夹爪下方（task.py 纯平移保竖立）。
整段手指朝下、滴管保持竖立，只做垂直下探/上提 + 高位水平平移。

轨迹（TCP 世界坐标，手指默认朝下；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((dx,dy,H))
  ② 垂直下探抓点     mv((dx,dy,GRASP_Z))       # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)    # task 检测 attached
  ④ 垂直提出         mv((dx,dy,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv((bx,by,SQUEEZE_TCP_Z)) # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)    # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv((bx,by,DIP_TCP_Z))     # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)   # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移试管上 mv((tx,ty,H))
    H 下探到管口上方 mv(TUBE_DROP_TCP)         # 尖嘴 0.984 在管口 0.9593 上方 25mm
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)    # task: dropped → 液滴动画坠落 + TubeDrops 显
    J 松回持握宽     grip(GRIP_DROPPER, 20)    # 提管口前松回胶头直径
    K 垂直提出       mv((tx,ty,H)) ]
  ⑮ 高位回架         mv((dx,dy,H))
  ⑯ 下探放回         mv((dx,dy,GRASP_Z))
  ⑰ 松开释放         grip(GRIP_OPEN, 25)       # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((dx,dy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_GRASP_Z, BOTTLE_SQUEEZE_TCP_Z, DIP_TCP_Z,
                        TUBE_XY, TUBE_DROP_TCP)


class DropperPass(BaseMetaAction):
    """抓一支滴管 → 循环「吸液-滴液」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, dropper_xy, bottle_xy, cycles=1):
        self.dropper_xy = dropper_xy
        self.bottle_xy = bottle_xy
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        dx, dy = self.dropper_xy
        bx, by = self.bottle_xy
        tx, ty = TUBE_XY
        grasp = (dx, dy, DROP_GRASP_Z)              # 抓点（尖嘴已在孔内，抓胶头顶）
        squeeze_tcp = (bx, by, BOTTLE_SQUEEZE_TCP_Z)  # 瓶口挤空气 TCP（z 1.005）
        dip_tcp = (bx, by, DIP_TCP_Z)               # 浸液吸液 TCP（尖嘴 0.830 入液）
        actions = [
            # —— 抓滴管（架孔，尖嘴已在孔内；只抓这一次）——
            mv(e, (dx, dy, H)),                  # ① 高位接近滴管架上方
            mv(e, grasp),                        # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (dx, dy, H), 5),               # ④ 垂直提出试管架
        ]
        cycle = [
            # —— 上提到瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸液 ——
            mv(e, (bx, by, H)),                  # A 高位平移到瓶上方
            mv(e, squeeze_tcp),                  # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, dip_tcp),                      # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # E 松胶头吸液（回胶头直径=持握宽）
            mv(e, (bx, by, H)),                  # F 垂直提出液面
            # —— 移到试管口滴液 ——
            mv(e, (tx, ty, H)),                  # G 高位平移到试管上方
            mv(e, TUBE_DROP_TCP),                # H 下探到管口上方 25mm（液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),           # I 挤胶头滴液
            grip(e, GRIP_DROPPER, 20),           # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H)),                  # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (dx, dy, H)),                  # ⑮ 高位回架上方
            mv(e, grasp),                        # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑰ 松开释放（task: released → rest）
            mv(e, (dx, dy, H)),                  # ⑱ 垂直归位
        ]
        return actions
