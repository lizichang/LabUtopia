"""D6 试纸气体检测（通用）元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest/d2s/d3l/e2 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。2026-08-26 用户重新设计（专用试纸夹预夹好试纸，
机械臂不碰试纸）→ 4 个元动作，滴管竖直夹取（手指朝下）、试管侧面横夹（手指朝前 ORIENT_FWD）：
  ① WetPaper          取蒸馏水滴管 → 滴 1-2 滴润湿试纸湿润端 → 归位
  ② MoveTubeUnderPaper 取反应试管 → 移到试纸湿润端正下方（管口距试纸 2.5cm）
  ③ HoldDetect         保持 2.5s 观察试纸变色
  ④ ReturnTube         试管归位
"""
from ._base import BaseMetaAction, mv, grip, hold
from .wet_paper import WetPaper
from .move_tube_under_paper import MoveTubeUnderPaper
from .hold_detect import HoldDetect
from .return_tube import ReturnTube

__all__ = ["BaseMetaAction", "mv", "grip", "hold",
           "WetPaper", "MoveTubeUnderPaper", "HoldDetect", "ReturnTube"]
