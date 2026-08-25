"""E1「pH 试纸检测」元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest/d2s 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。E1 无手腕翻转（试纸已预铺白瓷板、棒竖直），
全部用 mv/grip/hold 平移原语。已实现：
  ① PickRod —— 夹取玻璃棒（抓点 = 棒底上方 0.20m）
  ② DipRod —— 玻璃棒尖端伸入试管蘸取待测溶液
  ③ TransferDrop —— 棒尖液滴点触试纸中央 → 原地等待显色 2.5s
  ④ ReturnRod —— 玻璃棒放回试管架前排右孔
"""
from ._base import BaseMetaAction, mv, grip, hold
from .pick_rod import PickRod
from .dip_rod import DipRod
from .transfer_drop import TransferDrop
from .return_rod import ReturnRod

__all__ = ["BaseMetaAction", "mv", "grip", "hold",
           "PickRod", "DipRod", "TransferDrop", "ReturnRod"]
