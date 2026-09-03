"""元动作 ①：夹滴管吸酚酞 → 移到锥形瓶 W 口上滴 3 滴（INDICATOR_PASS，P1）。

V7 语义（用户 2026-09-02 指令）：夹滴管（胶头）→ 提出 → 伸进指示剂瓶液面下吸酚酞
→ 移到锥形瓶（清点 W）口上 → 滴 2-3 滴（坠滴动画）→ 样液无色→粉 → 滴管放回架孔。
形状照 d3l SamplePass（抓一次→瓶口挤空气→浸液吸液→移到目标口滴液→放回一次）。

夹爪开度（d3l 实测）：移动/持握全程 = GRIP_DROPPER = GRIP_ASPIRATE = 0.0055（贴合
胶头 Ø11mm/2）；只在瓶口排空气 / 滴液瞬间挤到 GRIP_SQUEEZE=0.002，随后松回 0.0055。

几何（pxr 实测 d1_tmp.usd）：滴管竖插架内（0.4997,0.4512），尖嘴底 z=0.80，抓点胶头
z=0.93（架顶 0.913 之上）。持握 = TCP + HELD_OFFSET(0,0,-0.13)（尖嘴 0.13m 吊夹爪下，
task.py 纯平移保竖立）——TCP z = 尖嘴 z + 0.13。整段手指朝下、滴管保持竖立。

轨迹（TCP 世界坐标，手指朝下）：
  ① 高位接近滴管架   mv((sx,sy,H))
  ② 垂直下探抓点     mv(DROP_GRASP)            # 捏胶头
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)    # task 检测 attached
  ④ 垂直提出         mv((sx,sy,H), 5)
  ⑤ 高位平移指示剂瓶上 mv((bx,by,H))
  ⑥ 下探到瓶口       mv(IND_SQUEEZE_TCP)       # 尖嘴贴瓶口 rim（z 0.875）
  ⑦ 挤胶头排空气     grip(GRIP_SQUEEZE, 30)    # task: squeezed
  ⑧ 下探浸液         mv(IND_DIP_TCP)           # 尖嘴 0.818 入液面 0.828 下 10mm
  ⑨ 松胶头吸液       grip(GRIP_ASPIRATE, 40)   # task: filled → DropperFill 显
  ⑩ 垂直提出液面     mv((bx,by,H))
  ⑪ 高位平移锥形瓶上 mv((fx,fy,H))             # 过瓶口 0.9645：尖嘴须 1.02
  ⑫ 下探到瓶口上方   mv(FLASK_DROP_TCP)        # 尖嘴 0.9895 在瓶口上方 25mm
  ⑬ 挤胶头滴液       grip(GRIP_SQUEEZE, 40)    # task: dropped → 3 粉球坠落入瓶
  ⑭ 松回持握宽       grip(GRIP_DROPPER, 20)
  ⑮ 垂直提出         mv((fx,fy,H))
  ⑯ 高位回架         mv((sx,sy,H))
  ⑰ 下探放回         mv(DROP_GRASP)
  ⑱ 松开释放         grip(GRIP_OPEN, 25)       # task: released → rest
  ⑲ 垂直归位         mv((sx,sy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        DROP_XY, DROP_GRASP,
                        IND_BOTTLE_XY, IND_SQUEEZE_TCP, IND_DIP_TCP,
                        FLASK_XY, FLASK_DROP_TCP)


class IndicatorPass(BaseMetaAction):
    """抓滴管 → 吸酚酞 → 滴 3 滴入锥形瓶 W → 放回（一次持握）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        sx, sy = DROP_XY
        bx, by = IND_BOTTLE_XY
        fx, fy = FLASK_XY
        actions = [
            # —— 抓滴管（架内，捏胶头）——
            mv(e, (sx, sy, H)),                  # ① 高位接近滴管架上方
            mv(e, DROP_GRASP),                   # ② 垂直下探到胶头（z 0.93）
            grip(e, GRIP_DROPPER, 60),           # ③ 合爪夹紧（开合=胶头直径）
            mv(e, (sx, sy, H), 5),               # ④ 垂直提出滴管架
        ]
        cycle = [
            # —— 移到指示剂瓶口：挤胶头排空气 → 浸液吸液 ——
            mv(e, (bx, by, H)),                  # ⑤ 高位平移到指示剂瓶上方
            mv(e, IND_SQUEEZE_TCP),              # ⑥ 下探到瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),           # ⑦ 挤胶头（排空气）
            mv(e, IND_DIP_TCP),                  # ⑧ 下探浸液（尖嘴 0.818 入液下）
            grip(e, GRIP_ASPIRATE, 40),          # ⑨ 松胶头吸液（回持握宽）
            mv(e, (bx, by, H)),                  # ⑩ 垂直提出液面
            # —— 移到锥形瓶 W 口滴液 ——
            mv(e, (fx, fy, H)),                  # ⑪ 高位平移到锥形瓶上方
            mv(e, FLASK_DROP_TCP),               # ⑫ 下探到瓶口上方 25mm（尖嘴 0.9895）
            grip(e, GRIP_SQUEEZE, 40),           # ⑬ 挤胶头滴液（task: dropped → 坠滴）
            grip(e, GRIP_DROPPER, 20),           # ⑭ 松回持握宽
            mv(e, (fx, fy, H)),                  # ⑮ 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 滴完放回滴管架 ——
        actions += [
            mv(e, (sx, sy, H)),                  # ⑯ 高位回架上方
            mv(e, DROP_GRASP),                   # ⑰ 下探放回
            grip(e, GRIP_OPEN, 25),              # ⑱ 松开释放（task: released → rest）
            mv(e, (sx, sy, H)),                  # ⑲ 垂直归位
        ]
        return actions
