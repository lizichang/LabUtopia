"""C3 元动作基类：复用焰色反应已验证的 IK 运动原语。

BaseMetaAction / mv / grip / hold / shake 直接来自
controllers.flametest_meta_actions._base（组合 atomic_actions/flametest 的
MoveAction/GripAction/HoldAction + IkMotionEngine，Lula IK 驱动，RMP 弃用）。
C3 仅换一套坐标常量（横夹药匙 + 挖粉），运动机制完全复用，避免重造已踩坑验证的轮子。
"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold, shake

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake"]
