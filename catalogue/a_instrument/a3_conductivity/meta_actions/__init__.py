# -*- coding: utf-8 -*-
"""A3 电导率测量元动作包。

v5 = 5 元动作（controller 编排顺序）：
  ① PickSurfaceDish    竖直夹住玻璃皿提起来（接近 → 下探 → 夹紧 → 提出）
  ② MoveDishAboveBeaker 夹着皿水平移动到烧杯口正上方（后续倒粉动作再下降 + 倾斜）
  ③ PourDishIntoBeaker 倾斜玻璃皿把粉末倒入烧杯（下降 → 原地倾斜 → 保持）
  ④ ReturnSurfaceDish  把空皿放回天平秤盘（转竖直横移回 → 竖直下探 → 开爪松放 → 抬回）
  ⑤ PickWashBottle    水平横夹洗瓶肚子 + 抬起 + 把红嘴移到烧杯上方（仅移动，不含挤水）
后续步骤（挤水配液 / 电极浸入 / 读数）逐步追加。
"""
from .pick_surface_dish import PickSurfaceDish
from .move_dish_above_beaker import MoveDishAboveBeaker
from .pour_dish_into_beaker import PourDishIntoBeaker
from .return_surface_dish import ReturnSurfaceDish
from .pick_wash_bottle import PickWashBottle

__all__ = [
    "PickSurfaceDish",
    "MoveDishAboveBeaker",
    "PourDishIntoBeaker",
    "ReturnSurfaceDish",
    "PickWashBottle",
]
