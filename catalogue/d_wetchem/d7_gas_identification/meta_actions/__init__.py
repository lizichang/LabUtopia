"""D7 气体鉴定元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest/d2s/d3l/e2/d6 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。全程手指朝前 ORIENT_FWD（侧面横夹），避导气管竖段
/试管架顶板穿模。

2026-08-27 用户方案（检测试剂统一为液体 + 单输入入口 + ③④ 合并）；
**二改：跳过 PickStopper/RemoveStopper（橡皮塞预装塞紧，机械臂不夹取/拔塞）→ 仅 3 元动作**：
  ① DipGasTube    取检验试管 → 移到下浸孔下放使末端浸入液面下 15mm
  ② HoldDetect    保持 2.5s 通气观察（task 驱动气泡上升）
  ③ ReturnTube    检验试管归位
  （pick_stopper.py / remove_stopper.py 文件保留但不再导出，便于日后恢复。）
"""
from ._base import BaseMetaAction, mv, grip, hold
from .dip_gas_tube import DipGasTube
from .hold_detect import HoldDetect
from .return_tube import ReturnTube

__all__ = ["BaseMetaAction", "mv", "grip", "hold",
           "DipGasTube", "HoldDetect", "ReturnTube"]
