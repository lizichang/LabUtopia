"""元动作 S1：手指朝前（朝向 camera1）水平横夹洗瓶肚子，竖直提起 15cm，向 +X 移 15cm。

用户动作要求（逐字，2026-08-29）：「现在写动作水平横着夹住washbottle然后抬起来
向+x移动15cm（参考d2s夹的方法）」。

与 D2-S PickWashBottle 同模式（ORIENT_FWD 手指朝前 + 夹持 + 提起），但 B4 洗瓶
在 (0.20,0.10) rot180 红嘴朝 +X 正对烧杯，终点只到「抬起 + 向 +X 移 15cm」（x
0.20→0.35），不送试管口：
  ① 高位       mv((WASH_APPROACH_X, wy, H), orient=ORIENT_FWD)   # 高位且 x 偏开 0.13（瓶身 -X 壁 x=0.168 前 1cm）
  ② 竖直下探   mv((WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD)   # x 偏移竖直降，手指到瓶身 -X 外侧
  ③ 水平移入   mv(WASH_GRASP, orient=ORIENT_FWD)                  # 锁 z/y，仅 x 平移入瓶身中心（吸管在指间）
  ④ 横夹肚子   grip(GRIP_WASHBOT, 60)                             # 半开度 0.030→6cm 开口压 6.4cm 软瓶身每侧 2mm
  ⑤ 竖直提起   mv((wx, wy, WASH_LIFT), orient=ORIENT_FWD)         # 锁 x/y，仅 z 抬升 15cm（0.88→1.03）
  ⑥ 往 +X      mv((WASH_TO_X, wy, WASH_LIFT), orient=ORIENT_FWD)   # 锁 y/z 仅 x +0.15（0.20→0.35）

抓取高度 z=0.88（瓶身中部，d2s 经验：低 z 远伸 IK 冻结，抬高中部留俯仰空间）。
夹持宽度 = 半开度 0.030（URDF 平移副开度=2×width；GRIP_OPEN=0.04 开 8cm > 6.4cm
瓶身可容纳，见 constants.py）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_WASHBOT, ORIENT_FWD,
                        WASH_APPROACH_X, WASH_GRASP, WASH_LIFT, WASH_TO_X)


class PickWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy, wz = WASH_GRASP
        return [
            mv(e, (WASH_APPROACH_X, wy, H), orient=ORIENT_FWD),    # ① 高位（x 偏开 -X 避开 +X 嘴尖）
            mv(e, (WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD),   # ② 竖直下探到瓶身下段外侧
            mv(e, WASH_GRASP, orient=ORIENT_FWD),                  # ③ 水平移入瓶身中心（锁 y/z，仅 x 平移：
                                                                   #   直线段逐帧重解走纯 x 直线，无 +y 弧线穿模）
            grip(e, GRIP_WASHBOT, 60),                             # ④ 水平横夹肚子（开度 6cm）
            mv(e, (wx, wy, WASH_LIFT), orient=ORIENT_FWD),         # ⑤ 竖直提起 15cm（z 0.88→1.03）
            mv(e, (WASH_TO_X, wy, WASH_LIFT), orient=ORIENT_FWD),  # ⑥ 往 +X 移 15cm（x 0.20→0.35）
        ]
