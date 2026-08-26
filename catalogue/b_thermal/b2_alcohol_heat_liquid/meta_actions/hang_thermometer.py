"""元动作 ②：挂温度计（B2 沸点测定 V7 步骤 5，2026-08-26 拆两段，先做段 1）。

用户逐字指令（2026-08-26）：「调整一下，首先夹温度计夹的位置太高了，然后这个放温度计我
感觉也分成两步吧，首先是像加液体的时候拿滴管的样子，先把温度计移动到离试管口x有一点偏移
的位置，然后这时候y值是对齐的（温度计的y坐标跟试管口的y坐标是一样的），因为只有斜着才能
插进去，所以温度计是有一点倾斜的，同时，温度计底部 温度炮是在试管口，然后整体倾斜有一点
x方向的偏移。这是第一步就像我之前说的倾斜加液体的那样子，你先完成这一步，其他的先不要，
你之前的先删除」

段 1（本元动作）= 抓温度计 + 倾斜 20° 带到试管口（泡尖落管口上方 5mm），不插入、不挂环：
  1. 水平横夹杆身（d2s 夹药匙式，手指朝前 ORIENT_FWD 横夹竖直杆身）。抓点比旧版低
     （THERMO_GRASP_OFFSET 0.254→0.20，用户「夹的位置太高了」——旧抓点贴挂环下方）。
  2. 高位竖直运到试管口 -X 侧（夹爪 x=0.4602 偏管口 0.0684、y=0.0029 与管口对齐）。
  3. 原地倾斜 20°（泡尖摆到试管口正上方，仍高于管口）→ 竖直下探：泡尖落到管口正上方
     5mm (0.5286,0.0029,1.0789)，整体倾斜保持 X 偏移（像加液体先偏 -X 再靠近的样子）。
     2026-08-26 用户：25°→10°→15°→20°（逐级）、玻璃球抬高 0.5cm、x 前移试过 0.5/0.1cm
     （泡尖会偏 +X）→ 回退泡尖锁管口 x、夹爪往 -X 偏 6.8cm。
段 2（后续再实现，已删除）：倾斜深入 → 竖直起来挂环套铁架台钩 → 松爪自然下垂。

避穿模关键：
  - 先在高位（APPROACH_HIGH z=1.35，泡尖 1.15 在口 1.0739 上）原地倾斜——泡尖摆到管口
    正上方 z=1.162 仍远高于管口，不横穿管壁；再竖直下探到 APPROACH（泡尖 1.0789 落管口
    上方 5mm）。
  - 持握 = 夹爪 + 0.20·tool+X（泡尖）；倾斜 20° 时 tool+X=(0.3420,0,-0.9397) → 泡尖相对
    夹爪 (0.0684,0,-0.1879)。挂环中心 = 夹爪 + 0.0695·tool-X（段 2 挂臂用）。

轨迹（TCP 世界坐标，全程 orient 显式传）：
  ① 高位接近  mv((THERMO_XY[0],THERMO_XY[1],H), orient=ORIENT_FWD)   # 手指朝前横着接近
  ② 水平下探  mv(THERMO_GRASP, orient=ORIENT_FWD)                    # 横着对准杆身（抓点 z 1.008）
  ③ 合爪夹杆  grip(GRIP_THERMO, 60)                                  # task: attached
  ④ 竖直提出  mv((THERMO_XY[0],THERMO_XY[1],H), 5, orient=ORIENT_FWD)
  ⑤ 高位运到试管口 -X 侧  mv(THERMO_APPROACH_HIGH, orient=ORIENT_FWD) # y 对齐、竖直
  ⑥ 原地倾斜 20°  mv(THERMO_APPROACH_HIGH, orient=ORIENT_TILT_20, linewalk=False)
                                                                      # 泡尖摆管口正上方（不穿管壁）
  ⑦ 竖直下探  mv(THERMO_APPROACH, orient=ORIENT_TILT_20)             # 泡尖 1.0789 正对管口上方 5mm（段 1 终点）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_THERMO, ORIENT_FWD,
                        ORIENT_TILT_20,
                        THERMO_XY, THERMO_GRASP,
                        THERMO_APPROACH_HIGH, THERMO_APPROACH)


class HangThermometer(BaseMetaAction):
    """抓温度计 → 倾斜 20° 带到试管口（泡尖落管口上方 5mm）。段 2（插入+挂环）未实现。"""

    def _build_actions(self):
        e = self.engine
        tx0, ty0 = THERMO_XY
        actions = [
            # —— 抓温度计（架右后排孔）：手指朝前横着夹竖直杆身（d2s 夹药匙式）——
            mv(e, (tx0, ty0, H), orient=ORIENT_FWD),   # ① 高位接近 + 手指朝前（水平横夹姿态）
            mv(e, THERMO_GRASP, orient=ORIENT_FWD),     # ② 水平下探到杆身（抓点 z 1.008，比旧版低）
            grip(e, GRIP_THERMO, 60),                   # ③ 合爪横着夹住杆身（task 检测 attached）
            mv(e, (tx0, ty0, H), 5, orient=ORIENT_FWD),  # ④ 竖直提出（温度计挂夹爪下）
            # —— 段 1：倾斜 20° 带到试管口（泡尖正对管口上方 5mm，不插入）——
            mv(e, THERMO_APPROACH_HIGH, orient=ORIENT_FWD),  # ⑤ 高位运到试管口 -X 侧（y 对齐，竖直）
            mv(e, THERMO_APPROACH_HIGH, orient=ORIENT_TILT_20, linewalk=False),  # ⑥ 原地倾斜 20°（泡尖摆管口正上方）
            mv(e, THERMO_APPROACH, orient=ORIENT_TILT_20),  # ⑦ 竖直下探：泡尖 1.0789 正对管口上方 5mm（段 1 终点）
        ]
        return actions
