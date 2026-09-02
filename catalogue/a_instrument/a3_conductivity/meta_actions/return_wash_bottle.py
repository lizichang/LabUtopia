# -*- coding: utf-8 -*-
"""A3 ⑦ 把挤完水的洗瓶放回原位（2026-08-30 用户「现在加动作把洗瓶放回原位」）。

SqueezeWashBottle 之后瓶仍 attached 悬在烧杯上方（TCP WASH_MOVE (0.2536,0.1012,1.05)、
夹爪 0.030 持握、水已挤入烧杯）。本动作把瓶放回表位：
  ① 水平移回瓶原位上方（TCP → (0.3536,0.3012,1.05)，z 锁 1.05 不变、xy 回移；夹爪保持
     0.030 持握不松，瓶随爪平移）
  ② 竖直下探到抓点高度（TCP → WASH_GRASP (0.3536,0.3012,0.90)，纯 z 降 15cm）——瓶底
     0.80 = 台面顶 = rest，正好落回原位
  ③ 开爪松放（grip GRIP_OPEN=0.04）：task 检测 opening > WASH_GRIP_OPEN(0.038) → released，
     瓶回 rest 位姿（瓶底 0.80 = 表位，零跳变）
  ④ 抬回安全高位撤离（TCP → (0.3536,0.3012,H)），瓶留在表位。

夹爪全程保持 GRIP_WASHBOT 闭合直到 ③（grip_target 由 controller 从 SqueezeWashBottle
传播，①② 无 grip 原子动作、首帧不开爪——工具已吸附类，ReturnSurfaceDish 同款）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import H, GRIP_OPEN, ORIENT_FWD, WASH_XY, WASH_GRASP_Y, WASH_GRASP, WASH_LIFT


class ReturnWashBottle(BaseMetaAction):
    """把洗瓶放回表位：水平移回原位上方 → 竖直下探 → 开爪松放 → 抬回撤离。"""

    def _build_actions(self):
        e = self.engine
        home_x, home_y = WASH_XY[0], WASH_GRASP_Y
        above = (home_x, home_y, WASH_LIFT)     # 原位上方（z 锁 1.05 不变，纯 xy 移回）
        return [
            mv(e, above, orient=ORIENT_FWD),    # ① 水平移回瓶原位上方
            mv(e, WASH_GRASP, orient=ORIENT_FWD),  # ② 竖直下探到抓点高度（瓶底落回台面）
            grip(e, GRIP_OPEN, 40),             # ③ 开爪松放（task → released，瓶回 rest）
            mv(e, (home_x, home_y, H), orient=ORIENT_FWD),   # ④ 抬回安全高位撤离（瓶留表位）
        ]
