"""元动作 ③：保持 2.5s 观察试纸变色（D6 检测）。

试管已在试纸湿润端正下方（管口 0.965），保持 HOLD_DETECT_DWELL 帧让气体上升接触湿润试纸，
task 期间按 cfg.gas_result 驱动试纸湿润端 visibility 变色（检出变蓝 / 未变不变）。
"""
from ._base import BaseMetaAction, hold
from .constants import HOLD_DETECT_DWELL


class HoldDetect(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            hold(e, HOLD_DETECT_DWELL),     # ① 保持 2.5s（task 驱动试纸变色动画）
        ]
