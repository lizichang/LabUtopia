"""元动作 ③：碱滴管 吸碱性试剂 → 滴入试管（ALKALI_PASS）。

D4-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加碱性试剂」。挖粉动作（①PickSpatula →
②ReturnSpatula，d2s 元动作包复用，坐标逐字一致）完成后，本元动作接续：抓碱滴管 →
上提到碱瓶口 → 瓶口挤胶头排空气 → 浸入液面吸碱 → 上提到试管口 → 挤胶头滴碱 →
放回滴管架。加碱触发现象（气泡/沉淀/液体变色，task `_grow_tube_level` 在 name=="alkali"
时按 cfg 显示）。

与 d3l acid_pass 同构（单段滴加，无 B2 铁架台挂钩），仅换 D4-S 坐标 + 全程水平横夹
（ORIENT_FWD）。抓取方式照 B2（2026-08-25 用户逐字纠正）：「像夹药匙一样水平横着夹滴管」
——滴管像 d2s 药匙那样**手指朝前 (ORIENT_FWD) 水平横着夹住竖直管身**，不是手指朝下
竖直夹。task._T_HELD 沿 tool+X 伸 0.13、attach 后竖直挂夹爪下与架内零跳变。

夹爪开度同 d3l/B2：移动/持握 = GRIP_DROPPER = GRIP_ASPIRATE = 0.0055；只在排空气/
滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055 再移动。

轨迹（TCP 世界坐标，全程 orient=ORIENT_FWD 手指朝前；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((ax,ay,H), orient=ORIENT_FWD)
  ② 水平下探抓点     mv(DROP_ALKALI_GRASP, orient=ORIENT_FWD)   # 横着夹竖直管身（夹胶头 z 0.936）
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)                   # task 检测 attached（零跳变）
  ④ 竖直提出         mv((ax,ay,H), 5, orient=ORIENT_FWD)
  [ A 高位平移瓶上   mv((bx,by,H), orient=ORIENT_FWD)
    B 下探到瓶口     mv(ALKALI_SQUEEZE_TCP, orient=ORIENT_FWD)  # 尖嘴贴瓶口 rim（z 0.875）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)                   # task: squeezed（首遍=排空气，再遍=再吸）
    D 下探浸液       mv(ALKALI_DIP_TCP, orient=ORIENT_FWD)      # 尖嘴 0.830 入液面 0.840 下
    E 松胶头吸碱     grip(GRIP_ASPIRATE, 40)                  # task: filled
    F 垂直提出液面   mv((bx,by,H), orient=ORIENT_FWD)
    G 高位平移试管上 mv((tx,ty,H), orient=ORIENT_FWD)
    H 下探到管口上方 mv(TUBE_DROP_TCP, orient=ORIENT_FWD)     # 尖嘴 0.984 在管口 0.9593 上方 25mm
    I 挤胶头滴碱     grip(GRIP_SQUEEZE, 40)                   # task: dropped → 液滴坠落 + 触发现象
    J 松回持握宽     grip(GRIP_DROPPER, 20)                   # 提管口前松回胶头直径（移动无缝隙）
    K 垂直提出       mv((tx,ty,H), orient=ORIENT_FWD) ]
  ⑮ 高位回架         mv((ax,ay,H), orient=ORIENT_FWD)
  ⑯ 下探放回         mv(DROP_ALKALI_GRASP, orient=ORIENT_FWD)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)                      # task: released（末遍滴完才回架松）
  ⑱ 垂直归位         mv((ax,ay,H), orient=ORIENT_FWD)
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        ORIENT_FWD,
                        DROP_ALKALI_XY, DROP_ALKALI_GRASP,
                        ALKALI_BOTTLE_XY, ALKALI_SQUEEZE_TCP, ALKALI_DIP_TCP,
                        TUBE_XY, TUBE_DROP_TCP)


class AlkaliPass(BaseMetaAction):
    """抓碱滴管 → 循环「吸碱-滴碱」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        ax, ay = DROP_ALKALI_XY
        bx, by = ALKALI_BOTTLE_XY
        tx, ty = TUBE_XY
        actions = [
            # —— 抓碱滴管（滴管架后排右孔，尖嘴已在孔内；只抓这一次；手指朝前水平横夹）——
            mv(e, (ax, ay, H), orient=ORIENT_FWD),      # ① 高位接近滴管架上方
            mv(e, DROP_ALKALI_GRASP, orient=ORIENT_FWD),  # ② 水平下探到胶头（z 0.936）
            grip(e, GRIP_DROPPER, 60),                  # ③ 合爪横着夹住竖直管身（开合=胶头直径）
            mv(e, (ax, ay, H), 5, orient=ORIENT_FWD),   # ④ 竖直提出试管架
        ]
        cycle = [
            # —— 上提到碱瓶口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸碱 ——
            mv(e, (bx, by, H), orient=ORIENT_FWD),      # A 高位平移到碱瓶上方
            mv(e, ALKALI_SQUEEZE_TCP, orient=ORIENT_FWD),  # B 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),                  # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, ALKALI_DIP_TCP, orient=ORIENT_FWD),     # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),                 # E 松胶头吸碱（回胶头直径=持握宽）
            mv(e, (bx, by, H), orient=ORIENT_FWD),      # F 垂直提出液面
            # —— 移到试管口滴碱 ——
            mv(e, (tx, ty, H), orient=ORIENT_FWD),      # G 高位平移到试管上方
            mv(e, TUBE_DROP_TCP, orient=ORIENT_FWD),    # H 下探到管口上方 25mm（尖嘴 0.984，液滴下落可见）
            grip(e, GRIP_SQUEEZE, 40),                  # I 挤胶头滴碱（加碱触发现象）
            grip(e, GRIP_DROPPER, 20),                  # J 松回持握宽（提管口/移动全程=胶头直径）
            mv(e, (tx, ty, H), orient=ORIENT_FWD),      # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (ax, ay, H), orient=ORIENT_FWD),      # ⑮ 高位回架上方
            mv(e, DROP_ALKALI_GRASP, orient=ORIENT_FWD),  # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),                     # ⑰ 松开释放（task: released → rest）
            mv(e, (ax, ay, H), orient=ORIENT_FWD),      # ⑱ 垂直归位
        ]
        return actions
