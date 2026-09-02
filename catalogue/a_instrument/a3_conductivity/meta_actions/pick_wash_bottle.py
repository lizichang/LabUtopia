# -*- coding: utf-8 -*-
"""A3 ⑤ 水平横夹洗瓶肚子 + 抬起 + 把红嘴移到烧杯上方（仅移动过程，不含挤水）。

用户 2026-08-29（逐字）：「现在加动作，水平横向夹起washbottle然后将红嘴移动到烧杯上方往
里面挤水，多参考d2s」→ 澄清（逐字）：「那整个动作就是水平横向夹住洗瓶之后抬起往-x移动
（虽然现在几乎是同X但是红嘴的+x坐标更大），然后再向-y方向移动 这样子红嘴就移动到烧杯
上方了。你先只写这个移动过程」。

与 d2s PickWashBottle 同模式（ORIENT_FWD 手指朝前 + 横夹肚子 + 提起），但 ⑥⑦ 改为固定偏移：
2026-08-29 用户「不朝-x移动15cm，改为12cm，-y方向移动20cm」→ ⑥ -Y 20cm ⑦ -X 12cm；2026-08-30「-x有点多减2cm」→ -X 10cm（不再对齐烧杯口）。

洗瓶 rotZ180、红嘴尖朝 +X（/World/WashBottle/.../pCylinder3/Mesh，pxr 实测 2026-08-29 开口
x=0.4650）。手指 ORIENT_FWD tool+Z=+X、两指沿 ±Y 张开夹瓶身 ±Y 面，吸管在指间（y≈0.3062
中心线）、红嘴尖在指端外 → 正落不压管，只需 x 偏开 -X 下探避嘴尖，到位后水平移入瓶身中心
（2026-08-30 用户「夹太靠下 + 夹得偏+y」→ 夹点上移 2cm 至 z=0.90、偏 -y 0.5cm 至 y=0.3012）：
  ⓪ 归位高位   mv(WASH_HOME, orient=ORIENT_FWD)                  # 侧向原地转 ORIENT_FWD（手腕非奇异 j5≈-2.2），锁定①的 warm-start
  ① 高位       mv((WASH_APPROACH_X, wy, H), orient=ORIENT_FWD)   # 高位且 x 偏开 0.284（瓶身 -X 壁 0.3216 前 3.8cm）
  ② 竖直下探   mv((WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD)   # x 偏移竖直降，手指到瓶身 -X 外侧
  ③ 水平移入   mv(WASH_GRASP, orient=ORIENT_FWD)                  # 锁 z/y，仅 x 平移入瓶身中心（吸管在指间）
  ④ 横夹肚子   grip(GRIP_WASHBOT, 60)                             # 半开度 0.030→6cm 开口压 6.4cm 软瓶身每侧 2mm
  ⑤ 竖直提起   mv((wx, wy, WASH_LIFT), orient=ORIENT_FWD)         # 锁 x/y，仅 z 抬升 15cm（0.90→1.05）
  ⑥ 往 -Y      mv((wx, WASH_MOVE_Y, WASH_LIFT), orient=ORIENT_FWD)        # 锁 x/z 仅 y -0.20（0.3012→0.1012）
  ⑦ 往 -X      mv((WASH_MOVE_X, WASH_MOVE_Y, WASH_LIFT), orient=ORIENT_FWD)  # 锁 y/z 仅 x -0.10（0.3536→0.2536）

朝向 = ORIENT_FWD 全程（与 d2s 一致，瓶保持静止朝向，水平段边移边不转）。
task 侧：洗瓶 rest → 近抓点+合爪 → attached（纯平移持握，_T_HELD_WASHB = rest·tool^-1）→
开爪 → released（回 rest）。挤水/水流动画留待后续动作（用户「先只写这个移动过程」）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_WASHBOT, ORIENT_FWD,
                        WASH_APPROACH_X, WASH_GRASP, WASH_LIFT,
                        WASH_MOVE_X, WASH_MOVE_Y, WASH_HOME)


class PickWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy, wz = WASH_GRASP
        return [
            mv(e, WASH_HOME, orient=ORIENT_FWD),                   # ⓪ 侧向原地转 ORIENT_FWD（手腕非奇异），锁定 warm-start
            mv(e, (WASH_APPROACH_X, wy, H), orient=ORIENT_FWD),    # ① 高位（x 偏开 -X 避开 +X 嘴尖）
            mv(e, (WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD),   # ② 竖直下探到瓶身下段外侧
            mv(e, WASH_GRASP, orient=ORIENT_FWD),                  # ③ 水平移入瓶身中心（锁 y/z）
            grip(e, GRIP_WASHBOT, 60),                             # ④ 水平横夹肚子（开度 6cm）
            mv(e, (wx, wy, WASH_LIFT), orient=ORIENT_FWD),         # ⑤ 竖直提起 15cm（z 0.90→1.05）
            mv(e, (wx, WASH_MOVE_Y, WASH_LIFT), orient=ORIENT_FWD),        # ⑥ 先往 -Y 20cm（y 0.3012→0.1012）
            mv(e, (WASH_MOVE_X, WASH_MOVE_Y, WASH_LIFT), orient=ORIENT_FWD),  # ⑦ 再往 -X 10cm（x 0.3536→0.2536）
        ]
