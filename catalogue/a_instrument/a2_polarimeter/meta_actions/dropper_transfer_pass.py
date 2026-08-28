# -*- coding: utf-8 -*-
"""A2 ⑥ 胶头滴管转移：吸试管内液 → 挤进旋光管加液口（DROP_PASS）。

2026-08-27 滴管转移改造（用户：倒液时机械臂乱动转圈，改用胶头滴管移动液体）：一次持握，
从试管吸 DROP_CYCLES=3 次、每次水平 −x 移 10cm 挤进旋光管加液口，替代
PickTestTube→PourToTube→ReturnTestTube 倒液（已删除）。

持握 d3s 酸滴管同款：全程手指朝前水平横夹（ORIENT_FWD），_T_HELD 沿 tool+X 伸 0.13
（尖嘴吊夹爪下）；开度 移动/持握 = GRIP_DROPPER = 0.0055，只在排空气/滴液瞬间挤到
GRIP_SQUEEZE=0.002，随后松回 0.0055 再移动。抓点 = 架孔底 0.806 + TIP_OFFSET = 0.936
（横着夹竖直管身胶头）。

用户描述的每遍循环：
  从试管吸液体 → 抬起 → 水平往 −x 移动 10cm（y,z 不变，0.659→0.559）→
  竖直往下移动 → 挤压滴出液体。滴液点 = 加液口 (0.559,0.241,0.830) —— 即调整后
  旋光管位置 PTUBE_REST=(0.5265,0.241,0.811) 的加液口正上方。DROPPER_DROP_TCP 尖嘴
  0.855（口上 25mm）→ 液滴坠落可见。

轨迹（TCP 世界坐标，全程 orient=ORIENT_FWD；[ ] 为 cycle，整体重复 cycles 遍）：
  ① 高位接近滴管架   mv((dx,dy,H))
  ② 水平下探抓点     mv(DROP_GRASP)                    # 横着夹竖直管身（夹胶头 z 0.936）
  ③ 合爪夹紧         grip(GRIP_DROPPER, 60)            # task 检测 attached（零跳变）
  ④ 竖直提出         mv((dx,dy,H), 5)
  [ A 高位平移试管上  mv((tx,ty,H))
    B 下探到管口     mv(DROPPER_SQUEEZE_TCP)            # 尖嘴贴管口上 5mm（z 0.9643）
    C 挤胶头排空气   grip(GRIP_SQUEEZE, 30)             # task: squeezed（首遍排空气/再遍再吸）
    D 下探浸液       mv(DROPPER_DIP_TCP)                # 尖嘴 0.8525 沉液面 0.8725 下 20mm
    E 松胶头吸液     grip(GRIP_ASPIRATE, 40)            # task: filled（回胶头直径=持握宽）
    F 垂直提出液面   mv((tx,ty,H))
    G 高位平移到加液口 mv((fx,fy,H))                    # 水平 −x 10cm，y,z 不变
    H 下探到加液口上 mv(DROPPER_DROP_TCP)               # 尖嘴 0.855（口上 25mm）
    I 挤胶头滴液     grip(GRIP_SQUEEZE, 40)             # task: dropped → 液滴坠落 + TubeLiquid 长
    J 松回持握宽     grip(GRIP_DROPPER, 20)             # 提口/移动全程=胶头直径
    K 垂直提出       mv((fx,fy,H)) ]
  ⑮ 高位回架         mv((dx,dy,H))
  ⑯ 下探放回         mv(DROP_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN, 25)               # task: released → rest（末遍滴完才回架松）
  ⑱ 垂直归位         mv((dx,dy,H))
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (
    H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE, GRIP_ASPIRATE, ORIENT_FWD,
    TUBE_XY, DROPPER_SQUEEZE_TCP, DROPPER_DIP_TCP,
    FILL_XY, DROPPER_DROP_TCP,
    DROP_XY, DROP_GRASP,
)


class DropperTransferPass(BaseMetaAction):
    """抓滴管 → 循环「吸试管液-滴加液口」cycles 遍 → 放回（一次持握，中途不松开）。"""

    def __init__(self, engine, cycles=3):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        dx, dy = DROP_XY
        tx, ty = TUBE_XY
        fx, fy = FILL_XY
        actions = [
            # —— 抓滴管（主试管架第二列第5排，尖嘴已在孔内；只抓这一次；手指朝前水平横夹）——
            mv(e, (dx, dy, H), orient=ORIENT_FWD),      # ① 高位接近滴管架上方
            mv(e, DROP_GRASP, orient=ORIENT_FWD),       # ② 水平下探到胶头（z 0.936）
            grip(e, GRIP_DROPPER, 60),                  # ③ 合爪横着夹住竖直管身（开合=胶头直径）
            mv(e, (dx, dy, H), 5, orient=ORIENT_FWD),   # ④ 竖直提出试管架
        ]
        cycle = [
            # —— 上提到试管口：挤胶头（首遍排空气/再遍再吸）→ 浸液吸液 ——
            mv(e, (tx, ty, H), orient=ORIENT_FWD),      # A 高位平移到试管上方
            mv(e, DROPPER_SQUEEZE_TCP, orient=ORIENT_FWD),  # B 下探到管口（尖嘴贴口上 5mm）
            grip(e, GRIP_SQUEEZE, 30),                  # C 挤胶头（首遍排空气/再遍再吸）
            mv(e, DROPPER_DIP_TCP, orient=ORIENT_FWD),  # D 下探浸液（尖嘴 0.8525 入液面下）
            grip(e, GRIP_ASPIRATE, 40),                 # E 松胶头吸液（回胶头直径=持握宽）
            mv(e, (tx, ty, H), orient=ORIENT_FWD),      # F 垂直提出液面
            # —— 移到加液口滴液（水平 −x 10cm，y,z 不变）——
            mv(e, (fx, fy, H), orient=ORIENT_FWD),      # G 高位平移到加液口上方（0.659→0.559）
            mv(e, DROPPER_DROP_TCP, orient=ORIENT_FWD),  # H 下探到加液口上方 25mm（尖嘴 0.855）
            grip(e, GRIP_SQUEEZE, 40),                  # I 挤胶头滴液（task: dropped → 现象）
            grip(e, GRIP_DROPPER, 20),                  # J 松回持握宽（提口/移动全程=胶头直径）
            mv(e, (fx, fy, H), orient=ORIENT_FWD),      # K 垂直提出
        ]
        actions += cycle * self.cycles
        # —— 末遍滴完才放回滴管架 ——
        actions += [
            mv(e, (dx, dy, H), orient=ORIENT_FWD),      # ⑮ 高位回架上方
            mv(e, DROP_GRASP, orient=ORIENT_FWD),       # ⑯ 下探放回
            grip(e, GRIP_OPEN, 25),                     # ⑰ 松开释放（task: released → rest）
            mv(e, (dx, dy, H), orient=ORIENT_FWD),      # ⑱ 垂直归位
        ]
        return actions
