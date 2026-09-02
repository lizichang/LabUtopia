"""D9 元动作基类：复用焰色反应已验证的 IK 运动原语。

BaseMetaAction / mv / grip / hold / shake 直接来自 controllers.flametest_meta_actions._base
（组合 atomic_actions/flametest 的 MoveAction/GripAction/HoldAction/ShakeAction +
IkMotionEngine，Lula IK 驱动，RMP 弃用）。D9 仅换一套坐标常量 + 多引 shake（甩灭），
运动机制完全复用（同 d2s/d3l/e2/d6/d7）。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold, shake

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake"]
