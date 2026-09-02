"""D5 蒸馏分离元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

D5 预组装蒸馏装置，机械臂仅执行点燃酒精灯（1 元动作），蒸馏现象（加热→沸腾→冷凝→
馏出液收集）由 task 现象状态机驱动。全程默认朝向（手指朝下）——火柴水平横躺轴 +X，
手指朝下竖直夹杆身（B2 火柴同款，已验证）。

  ① LightFlamePass  拿火柴点燃酒精灯（照 B2 LightFlamePass）
"""
from ._base import BaseMetaAction, mv, grip, hold
from .light_flame import LightFlamePass

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "LightFlamePass"]
