# -*- coding: utf-8 -*-
"""DipCapillaryInOil：中部水平夹起后，横移到油皿上方再竖直下探，让封口端蘸石蜡油。

用户 2026-09-01 逐字：「然后移动到油里面沾一下」——毛细管水平持握（矩阵持握，封口端在
夹点 −0.05·世界+X 处），横移到油皿正上方再竖直下探，让封口端（闭口圆钝尖，贴泡那一端）
沉入石蜡油薄层蘸油（油膜提供后续贴泡吸附）。

轨迹（手指朝下朝向全程不变，水平由 task 侧矩阵持握保证）：
  ① 横移到油皿上方 mv((OIL_DIP_GRIP, CAP_HIGH))    # 封口端在油皿 (0.22,0.15)（皿中心 -X 3cm）上方
  ② 竖直下探蘸油   mv((OIL_DIP_GRIP, OIL_DIP_CAP_Z), dwell)

几何：封口端 = 夹点 − 0.05·世界+X。夹点 OIL_DIP_GRIP=(0.27,0.15) → 封口端 (0.22,0.15)=
油皿中心 (0.25,0.15) -X 3cm（用户 2026-09-02「再往 -X 移动 3cm」）；夹点 z=0.804 → 封口端
中心 0.804 沉入油层 0.802-0.806（管底 0.803 入油）。
"""
from ._base import BaseMetaAction, mv
from .constants import OIL_DIP_GRIP, OIL_DIP_CAP_Z, CAP_HIGH


class DipCapillaryInOil(BaseMetaAction):
    """水平持毛细管：横移到油皿上方 → 竖直下探封口端蘸石蜡油。"""

    def _build_actions(self):
        e = self.engine
        dx, dy = OIL_DIP_GRIP
        return [
            mv(e, (dx, dy, CAP_HIGH)),                 # ① 横移到油皿上方（封口端在皿中心 -X 3cm）
            mv(e, (dx, dy, OIL_DIP_CAP_Z), dwell=20),  # ② 竖直下探封口端蘸油
        ]
