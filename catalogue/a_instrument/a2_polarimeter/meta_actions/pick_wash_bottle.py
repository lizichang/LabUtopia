# -*- coding: utf-8 -*-
"""A2 ① 拿洗瓶：高位偏开 → 偏开下探 → 水平移入 → 横夹 → 提出 → 送红嘴到试管口。

d2s PickWashBottle 同款（横夹肚子 ORIENT_FWD），洗瓶在 d2s 原位 (0.370,0.525)
rotZ-180 红嘴朝 +X；高位 x 偏开 0.30（避 +X 嘴尖）竖直下探到瓶身 -X 外侧，再水平移入
瓶身中心夹持 —— 与 d2s 完全一致的折叠接近构型。直落瓶中心会让臂落入 IK 另一解分支，
后续长 -Y 送红嘴时肘部翻转换分支 = 用户看到的绕大圈；偏开接近保证后续走直线。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (
    H, ORIENT_FWD, GRIP_WASHBOT, WASH_APPROACH_X, WASH_GRASP, WASH_LIFT,
    WASH_TO_TUBE_X, WASH_TO_TUBE_Y,
)


class PickWashBottle(BaseMetaAction):
    def _build_actions(self):
        wx, wy, wz = WASH_GRASP
        return [
            mv(self.engine, (WASH_APPROACH_X, wy, H), orient=ORIENT_FWD),    # ① 高位（x 偏开 -X 避开 +X 嘴尖）
            mv(self.engine, (WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD),   # ② 竖直下探到瓶身下段外侧
            mv(self.engine, WASH_GRASP, orient=ORIENT_FWD),                  # ③ 水平移入瓶身中心（锁 y/z）
            grip(self.engine, GRIP_WASHBOT, 60),                             # ④ 横夹肚子
            mv(self.engine, (wx, wy, WASH_LIFT), orient=ORIENT_FWD),         # ⑤ 提出 15cm
            mv(self.engine, (WASH_TO_TUBE_X, wy, WASH_LIFT), orient=ORIENT_FWD),  # ⑥ +X 送红嘴到管口 x
            mv(self.engine, (WASH_TO_TUBE_X, WASH_TO_TUBE_Y, WASH_LIFT), orient=ORIENT_FWD),  # ⑦ -Y 对管口
        ]
