"""元动作 ③：保持 2.5s 通气观察（D7 气体鉴定，③④ 合并）。

检验试管已下浸到位（导气管末端 1.024 沉入检测液面 1.039 下 15mm），保持 HOLD_DETECT_DWELL 帧
让产气试管内气体经导气管通入检测液，task 期间驱动 GasBubbles 气泡从末端连续上升动画。
（用户：检测试剂统一表现为液体，仅初始颜色由输入决定，无浑浊/变色变体，故不换 visibility。）
"""
from ._base import BaseMetaAction, hold
from .constants import HOLD_DETECT_DWELL


class HoldDetect(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            hold(e, HOLD_DETECT_DWELL),     # ① 保持 2.5s（task 驱动气泡上升动画）
        ]
