"""元动作 ⑥：把冰浴后的试管放回试管架（2026-08-30 用户「最后放回试管」）。

ImmerseTube 之后试管仍 attached 竖直吊在烧杯上方（TCP (0.45,0.10,TUBE_HIGH)、夹爪
0.0096 持握）。本动作把试管放回架孔（ReturnWashBottle 同款）：
  ① 水平移回试管架上方 mv(TUBE_ABOVE_RACK)    # (0.279,0.241,1.10)，z 锁 1.10 不变、xy 回移
  ② 竖直下探到抓点高度 mv(TUBE_GRASP_TCP)     # (0.279,0.241,0.945)，纯 z 降 → 管底 0.806
                                                #   落回架孔底 = rest
  ③ 开爪松放 grip(GRIP_OPEN)                  # task: opening>0.03 + near 抓点 → released，
                                                #   试管回 rest 位姿（管底 0.806 = 架孔底，零跳变）
  ④ 抬回高位撤离 mv(TUBE_ABOVE_RACK)          # 纯 z 升回 1.10（管留架孔）

夹爪全程保持 GRIP_TUBE 闭合直到 ③（grip_target 由 controller 从 ImmerseTube 传播，
①② 无 grip 原子动作、首帧不开爪，与 ReturnWashBottle 同款）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_OPEN, TUBE_GRASP_TCP, TUBE_ABOVE_RACK


class ReturnTube(BaseMetaAction):
    """把试管放回架孔：水平移回架上方 → 竖直下探 → 开爪松放 → 抬回撤离。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, TUBE_ABOVE_RACK),      # ① 水平移回试管架上方（z 锁 1.10，xy 回移）
            mv(e, TUBE_GRASP_TCP),       # ② 竖直下探到抓点高度（管底落回架孔底 0.806）
            grip(e, GRIP_OPEN, 40),      # ③ 开爪松放（task → released，试管回 rest）
            mv(e, TUBE_ABOVE_RACK),      # ④ 抬回高位撤离（管留架孔）
        ]
