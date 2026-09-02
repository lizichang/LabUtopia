# -*- coding: utf-8 -*-
"""A3 ⑬ 松爪把电极放进烧杯（2026-08-30 用户「放进去就可以松手」）。

⑫ DipElectrodeIntoBeaker 之后电极仍 attached、blades 已浸入烧杯液面（TCP ELECTRODE_DIP=0.98）。
本动作开爪松放 → task 检测 opening > ELECTRODE_GRIP_OPEN(0.038) → released（电极 + 电缆冻结在
烧杯内、不回 meter），再抬空爪清电极 cap 顶撤离：
  ① 开爪松放（grip GRIP_OPEN=0.04，原地开爪；task → released，电极留在烧杯液体内）
  ② 抬空爪到 ELECTRODE_RELEASE_LIFT（TCP → (0.412,0.0807,1.10)，清电极 cap 顶 0.9825 撤离）

朝向 = 引擎默认（手指朝下）全程，与 ⑪ LiftElectrode / ⑫ DipElectrodeIntoBeaker 连续
（不传 orient）。电极 cap 顶 0.9825 < 1.10，抬升段指端不碰电极。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import GRIP_OPEN, ELECTRODE_RELEASE_LIFT


class ReleaseElectrode(BaseMetaAction):
    """松爪把电极留在烧杯内：开爪松放 → 抬空爪清电极撤离。"""

    def _build_actions(self):
        e = self.engine
        return [
            grip(e, GRIP_OPEN, 40),            # ① 开爪松放（task → released，电极留烧杯内）
            mv(e, ELECTRODE_RELEASE_LIFT),     # ② 抬空爪到 1.10（清电极 cap 顶撤离）
        ]
