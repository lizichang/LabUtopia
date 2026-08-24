"""元动作 ③：加碱滴管 吸碱性试剂 → 滴入试管（ALKALI_PASS）。

V7 步 5-8：抓加碱滴管 → 上提到碱瓶口 → 瓶口挤胶头排空气 → 浸入液面吸碱液
→ 上提到试管口 → 挤胶头滴碱 → 放回滴管架。与 SamplePass 同构，仅换
加碱滴管/碱瓶坐标——同一个试管口滴加，加碱（样品+碱混合）触发现象
（气泡/沉淀，task `_grow_tube_level` 在 name=="alkali" 时按 cfg 显示）。

D4-L 文档明确"加试剂滴管不可用取样滴管"，故用独立加碱滴管（DropperAlkali）。
吸碱液前 StopperOpenPass 已拔塞倒放（碱瓶口敞口可吸液）；本 pass 吸完碱液后
StopperClosePass 盖回塞子。

与 SamplePass 同一套：抓一次 → 循环「吸碱-滴碱」cfg.alkali_cycles 遍 → 放回一次
（中途不松开滴管）。task 生命周期靠瓶口/试管口 TCP 位置区分各遍。

夹爪开度同 SamplePass（用户 2026-08-14 实测 0.0055 正好贴胶头面）：移动/持握
= GRIP_DROPPER = GRIP_ASPIRATE = 0.0055；只在排空气/滴液瞬间挤到 GRIP_SQUEEZE
=0.002，随后松回 0.0055 再移动。

几何（2026-08-24 pxr 实测）：加碱滴管立插架后排右孔，尖嘴底 z=0.806，抓点在
胶头顶 z=0.936（架顶 0.917 之上）。持握 = TCP + HELD_OFFSET (0,0,-0.13) 使尖嘴
0.13m 吊在夹爪下方（纯平移保竖立）。TCP z = 尖嘴 z + 0.13。碱瓶 AlkaliBottle
(0.1696,0.361) 口 rim 0.870、液面 0.840（与 D3-L HClBottle 同坐标）。整段手指
朝下、滴管保持竖立。

轨迹（TCP 世界坐标，手指默认朝下；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((ax,ay,H))
  ② 垂直下探抓点     mv(DROP_ALKALI_GRASP)      # 尖嘴已在孔内，合爪抓胶头顶
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)     # task 检测 attached
  ④ 垂直提出         mv((ax,ay,H), 5)
  [ A 高位平移瓶上   mv((bx,by,H))
    B 下探到瓶口     mv(ALKALI_SQUEEZE_TCP)     # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)     # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(ALKALI_DIP_TCP)         # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸碱     grip(GRIP_ASPIRATE, 40)    # task: filled → DropperFill 显
    F 垂直提出液面   mv((bx,by,H))
    G 高位平移试管上 mv((tx,ty,H))
    H 下探到管口上方 mv(TUBE_DROP_TCP)          # 尖嘴 0.984 在管口 0.9593 上方 25mm
    I 挤胶头滴碱     grip(GRIP_SQUEEZE, 40)     # task: dropped → 液滴动画坠落 + TubeDrops 显
                                               #   + 加碱触发现象（气泡/沉淀）
    J 松回持握宽     grip(GRIP_DROPPER, 20)     # 提管口前松回胶头直径（移动全程无缝隙）
    K 垂直提出       mv((tx,ty,H)) ]
  ⑮ 高位回架         mv((ax,ay,H))
  ⑯ 下探放回         mv(DROP_ALKALI_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)        # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((ax,ay,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_ALKALI_XY, DROP_ALKALI_GRASP,
                        ALKALI_BOTTLE_XY, ALKALI_SQUEEZE_TCP, ALKALI_DIP_TCP,
                        TUBE_XY, TUBE_DROP_TCP)


class AlkaliPass(BaseMetaAction):
    """抓加碱滴管 → 循环「吸碱-滴碱」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        ax, ay = DROP_ALKALI_XY
        bx, by = ALKALI_BOTTLE_XY
        tx, ty = TUBE_XY
        actions = [
            # —— 抓加碱滴管（架后排右孔，尖嘴已在孔内；只抓这一次）——
            mv(e, (ax, ay, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_ALKALI_GRASP),            # ② 垂直下探到胶头顶（z 0.936）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (ax, ay, H), 5),               # ④ 垂直提出试管架
        ]
        cycle = [
            # —— 上提到碱瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸碱 ——
            mv(e, (bx, by, H)),                  # A 高位平移到碱瓶上方
            mv(e, ALKALI_SQUEEZE_TCP),           # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, ALKALI_DIP_TCP),               # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),          # E 松胶头吸碱（回胶头直径=持握宽）
            mv(e, (bx, by, H)),                  # F 垂直提出液面
            # —— 移到试管口滴碱 ——
            mv(e, (tx, ty, H)),                  # G 高位平移到试管上方
            mv(e, TUBE_DROP_TCP),                # H 下探到管口上方 25mm（尖嘴 0.984，液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),           # I 挤胶头滴碱（加碱触发现象）
            grip(e, GRIP_DROPPER, 20),           # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H)),                  # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (ax, ay, H)),                  # ⑮ 高位回架上方
            mv(e, DROP_ALKALI_GRASP),            # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑰ 松开释放（task: released → rest）
            mv(e, (ax, ay, H)),                  # ⑱ 垂直归位
        ]
        return actions
