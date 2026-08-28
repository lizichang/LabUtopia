"""元动作 ①：取样滴管 吸样品液 → 滴入试管（B2 沸点测定 V7 步骤 2-3 的滴加部分）。

V7 步 2-3：抓取样滴管 → 上提到样品瓶口 → 瓶口挤胶头排空气 → 浸入液面吸液
→ 上提到试管口 → 挤胶头滴液（出液柱）→ 放回滴管架（瓶塞盖回/沸石/挂温度计
等后续步骤由阶段 C-F 的元动作接续）。

2026-08-25 照 d3l sample_pass 重构结论：不「抓→滴→放回」重复 N 遍，而是**抓一次**
→循环「吸液-滴液」cfg.sample_cycles 遍→**放回一次**（中途不松开滴管）。同一次持握
内 task 生命周期靠瓶口/试管口尖嘴位置区分各遍（见 task.py dropped 再挤 → squeezed）。

抓取方式（2026-08-25 用户逐字纠正）：「抓试管应该水平横着夹在试管上而不是像现在
还是竖直的夹在试管上。你看d2s是怎么夹药匙的！」——滴管像 d2s 药匙那样**手指朝前
(ORIENT_FWD) 水平横着夹住竖直管身**（不是手指朝下竖直夹）。task._T_HELD 已改 d2s
药匙同款：滴管沿 tool+X 伸出 0.13、attach 后竖直挂夹爪下与架内零跳变。全程手指
朝前（横着操作），滴液也竖直在试管口滴加（2026-08-25 用户：没必要倾斜，同 d2l）。

夹爪开度：移动/持握全程 = GRIP_DROPPER = GRIP_ASPIRATE（横夹管身）；只在排空气/
滴液瞬间挤到 GRIP_SQUEEZE，随后松回再移动。

避挂钩支臂（z 1.246..1.257, x 0.5086..0.6446, 罩住试管正上方；2026-08-27 上移 2cm + 挂钩 +1cm）：
  - 横夹后滴管竖直挂夹爪下，杆体 z = 夹爪-0.13 ~ 夹爪。
  - 低空横越走 TRANSIT_Z=1.15：杆体 z∈[1.02,1.15] 全程在支臂底 1.246 之下。
  - 两段式滴液走 ORIENT_FWD（2026-08-25 用户：避免穿模）：夹爪 z1.2259+球 5.5mm=
    1.2314 在支臂底 1.246 之下；尖嘴在管口上方 2mm、杆体 z∈[1.096,1.226] 全在
    管口上方，最后只水平移 x 到管口正上方（不穿试管壁）。

轨迹（TCP 世界坐标；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近   mv((sx,sy,H), orient=ORIENT_FWD)      # 手指朝前水平接近（横夹姿态）
  ② 水平下探   mv(DROP_GRASP, orient=ORIENT_FWD)     # 横着对准竖直管身（d2s 夹药匙式）
  ③ 合爪夹紧   grip(GRIP_DROPPER, 60)                # task 检测 attached（零跳变）
  ④ 竖直提出   mv((sx,sy,H), 5, orient=ORIENT_FWD)   # 滴管竖直挂夹爪下提出
  [ A 瓶口上方 mv((bx,by,H), orient=ORIENT_FWD)      # 高位横移到瓶区（不经挂钩区）
    B 下探瓶口 mv(BOTTLE_SQUEEZE_TCP, orient=ORIENT_FWD)  # 尖嘴贴瓶口 rim（尖嘴=TCP-0.13）
    C 挤胶头   grip(GRIP_SQUEEZE, 30)                # task: squeezed（首遍排空气/再遍再吸）
    D 浸液     mv(SAMPLE_DIP_TCP, orient=ORIENT_FWD) # 尖嘴 0.830 入液面 0.840 下
    E 吸液     grip(GRIP_ASPIRATE, 40)               # task: filled
    F 提出低空 mv((bx,by,TRANSIT_Z), orient=ORIENT_FWD)  # 提出到低空横越高度
    G 横越西侧 mv((tx-DX,ty,TRANSIT_Z), orient=ORIENT_FWD)  # 低空横越到管口偏 -X 侧（避开试管）
    H 偏侧升滴 mv((tx-DX,ty,DRIP_Z), orient=ORIENT_FWD)  # 偏 -X 侧升到滴加高度（尖嘴在管口上）
    H2 水平入管 mv(DRIP_TCP, orient=ORIENT_FWD)      # 纯水平移 x 增大到管口正上方（yz 朝向不变）
    I 挤胶头滴 grip(GRIP_SQUEEZE, 40)                # task: dropped → 液滴坠落+液柱长高
    J 松回     grip(GRIP_DROPPER, 20)                # 提离前松回持握宽
    K1 原路退回 mv((tx-DX,ty,DRIP_Z), orient=ORIENT_FWD)  # 水平移 x 回偏 -X 侧（yz 朝向不变）
    K2 偏侧降空 mv((tx-DX,ty,TRANSIT_Z), orient=ORIENT_FWD) ]  # 偏 -X 侧降回低空（原路返回不穿模）
  ⑮ 低空回架  mv((sx,sy,TRANSIT_Z), orient=ORIENT_FWD)
  ⑯ 下探放回  mv(DROP_GRASP, orient=ORIENT_FWD)      # 尖嘴回孔底 z0.806（零跳变）
  ⑰ 松开释放  grip(GRIP_OPEN, 25)                    # task: released → rest
  ⑱ 垂直归位  mv((sx,sy,H), orient=ORIENT_FWD)
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE,
                        ORIENT_FWD, TRANSIT_Z,
                        DROP_XY, DROP_GRASP,
                        SAMPLE_BOTTLE_XY, BOTTLE_SQUEEZE_TCP, SAMPLE_DIP_TCP,
                        TUBE_XY, DRIP_TCP, DRIP_APPROACH_DX)


class DropperDripPass(BaseMetaAction):
    """抓取样滴管 → 循环「吸液-滴液」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=1):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        sx, sy = DROP_XY
        bx, by = SAMPLE_BOTTLE_XY
        tx, ty = TUBE_XY
        dx = DRIP_APPROACH_DX
        drip_z = DRIP_TCP[2]
        actions = [
            # —— 抓取样滴管（架左孔，尖嘴已在孔内）：手指朝前横着夹管身（d2s 夹药匙式）——
            mv(e, (sx, sy, H), orient=ORIENT_FWD),   # ① 高位接近 + 手指朝前（水平横夹姿态）
            mv(e, DROP_GRASP, orient=ORIENT_FWD),    # ② 水平下探到管身（抓点 z 0.936）
            grip(e, GRIP_DROPPER, 60),               # ③ 合爪横着夹住管身（task 检测 attached）
            mv(e, (sx, sy, H), 5, orient=ORIENT_FWD),  # ④ 竖直提出（滴管挂夹爪下）
        ]
        # —— 循环段（吸液-滴液一遍 = A-K，整体重复 cycles 遍；首遍 A 挤气+吸液，再滴液）——
        loop = [
            mv(e, (bx, by, H), orient=ORIENT_FWD),   # A 高位横移到瓶口上方（不经挂钩区）
            mv(e, BOTTLE_SQUEEZE_TCP, orient=ORIENT_FWD),  # B 下探瓶口（尖嘴贴 rim）
            grip(e, GRIP_SQUEEZE, 30),               # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, SAMPLE_DIP_TCP, orient=ORIENT_FWD),  # D 下探浸液（尖嘴 0.830 入液）
            grip(e, GRIP_ASPIRATE, 40),              # E 松胶头吸液（task: filled）
            mv(e, (bx, by, TRANSIT_Z), orient=ORIENT_FWD),  # F 提出到低空横越高度
            mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),  # G 横越到管口偏 -X 侧（避开试管）
            mv(e, (tx - dx, ty, drip_z), orient=ORIENT_FWD),  # H 偏 -X 侧升到滴加高度
            mv(e, DRIP_TCP, orient=ORIENT_FWD),      # H2 纯水平移 x 增大到管口正上方（yz 朝向不变）
            grip(e, GRIP_SQUEEZE, 40),               # I 挤胶头滴液（task: dropped）
            grip(e, GRIP_DROPPER, 20),               # J 松回持握宽
            mv(e, (tx - dx, ty, drip_z), orient=ORIENT_FWD),  # K1 原路水平移 x 回偏 -X 侧（yz 朝向不变）
            mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),  # K2 偏 -X 侧降到低空（原路返回不穿模）
        ]
        actions += loop * self.cycles
        # —— 末遍滴完才放回滴管架（低空横越回架区 → 下探放回）——
        actions += [
            mv(e, (sx, sy, TRANSIT_Z), orient=ORIENT_FWD),  # ⑮ 低空横越回架上方（压支臂之下）
            mv(e, DROP_GRASP, orient=ORIENT_FWD),    # ⑯ 下探放回（尖嘴回孔底，零跳变）
            grip(e, GRIP_OPEN, 25),                  # ⑰ 松开释放（task: released → rest）
            mv(e, (sx, sy, H), orient=ORIENT_FWD),   # ⑱ 垂直归位
        ]
        return actions
