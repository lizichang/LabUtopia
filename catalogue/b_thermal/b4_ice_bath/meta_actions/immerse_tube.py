"""元动作 ⑤：把竖直夹持的试管移到烧杯上方 → 竖直下探浸入冰水（2026-08-30 用户
「继续加动作把试管移动到烧杯上方并浸入冰水」）。

PickTube（④）结束时试管竖直吊在夹爪下方（管底 0.961，TCP 停在 (TUBE_XY, TUBE_HIGH)）。
本动作水平移到烧杯口上方 (0.45,0.10)，再竖直下探把管底浸入冰水（水面 0.842 下 1.4cm），
保持冷却 15s 后竖直提出，提出后看现象停留 8s（用户「冰浴时间再增加10s、现象应在冰浴期间出现」）。
纯平移持握（手指朝下）全程保竖立，管内药品液柱随管平移。

  ① 水平移到烧杯口上方 mv(TUBE_ABOVE_BEAKER)     # 管底 0.961 清杯口 0.8904（水平平移）
  ② 竖直下探浸入冰水   mv(TUBE_IMMERSE_TCP, 5)    # 管底 0.828 入水面 0.842 下 1.4cm
  ③ 浸冰保持           hold(IMMERSE_DWELL)        # 冷却 15s（浑浊渐显+晶体析出，慢）
  ④ 竖直提出           mv(TUBE_ABOVE_BEAKER)      # 回到杯口上方（管底清杯口）
  ⑤ 看现象停留         hold(OBSERVE_DWELL)        # 提出后观察 8s（浑浊渐褪回澄清、晶体留下、外壁起雾）
"""
from ._base import BaseMetaAction, mv, hold
from .constants import (TUBE_ABOVE_BEAKER, TUBE_IMMERSE_TCP, IMMERSE_DWELL, OBSERVE_DWELL)


class ImmerseTube(BaseMetaAction):
    """试管移到烧杯上方 → 竖直下探浸入冰水 → 保持冷却 → 提出看现象 → 停留 5s。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, TUBE_ABOVE_BEAKER),      # ① 水平移到烧杯口上方（管底清杯口）
            mv(e, TUBE_IMMERSE_TCP, 5),    # ② 竖直下探浸入冰水（管底入水面下 1.4cm）
            hold(e, IMMERSE_DWELL),        # ③ 浸冰保持（冷却 2s）
            mv(e, TUBE_ABOVE_BEAKER),      # ④ 竖直提出（回到杯口上方）
            hold(e, OBSERVE_DWELL),        # ⑤ 看现象停留 5s（观察冷却后现象）
        ]
