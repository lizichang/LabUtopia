"""E2「磁性检测」元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest/d2s/e1 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。2026-08-26 用户简化设计（待测固体预铺在
表面皿，删药匙/试管架/样品瓶/瓶盖）→ 只剩两个元动作，磁铁竖夹（手指朝下，无手腕翻转）：
  ① PickMagnet —— 夹磁铁 → 缓慢靠近样品上方检测（多中间节点不穿模）
  ② ReturnMagnet —— 磁铁撤回归位
"""
from ._base import BaseMetaAction, mv, grip, hold
from .pick_magnet import PickMagnet
from .return_magnet import ReturnMagnet

__all__ = ["BaseMetaAction", "mv", "grip", "hold",
           "PickMagnet", "ReturnMagnet"]
