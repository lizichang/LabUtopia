# -*- coding: utf-8 -*-
"""A3 电导率测量元动作包。

v10 = 12 元动作（controller 编排顺序）：
  ① PickSurfaceDish    竖直夹住玻璃皿提起来（接近 → 下探 → 夹紧 → 提出）
  ② MoveDishAboveBeaker 夹着皿水平移动到烧杯口正上方（后续倒粉动作再下降 + 倾斜）
  ③ PourDishIntoBeaker 倾斜玻璃皿把粉末倒入烧杯（下降 → 原地倾斜 → 保持）
  ④ ReturnSurfaceDish  把空皿放回天平秤盘（转竖直横移回 → 竖直下探 → 开爪松放 → 抬回）
  ⑤ PickWashBottle    水平横夹洗瓶肚子 + 抬起 + 把红嘴移到烧杯上方（仅移动，不含挤水）
  ⑥ SqueezeWashBottle 挤压洗瓶身出水（水从红嘴弧线落入烧杯，烧杯内液面上涨）
  ⑦ ReturnWashBottle  把挤完水的洗瓶放回原位（水平移回 → 下探 → 开爪松放 → 抬回）
  ⑧ PickGlassRod      到试管架水平横夹取出玻璃棒并移到烧杯口正上方（下探横夹 → 提出 → 移）
  ⑨ StirInBeaker      下降玻璃棒插入烧杯并圆周搅拌（下探 → 画圆搅拌 → 提出）
  ⑩ ReturnGlassRod    把搅拌完的玻璃棒放回试管架（水平移回 → 下探 → 开爪松放 → 抬回）
  ⑪ LiftElectrode     到导电率仪前竖直抓住电极探头并竖直提起（-X 接近 → 下探 → +X 移入 → 竖直夹 cap → 提起）
  ⑫ DipElectrodeIntoBeaker 把电极移到烧杯口上方 + 竖直下降深入（水平横移 → 下降浸入液面）
  ⑬ ReleaseElectrode  松爪把电极留在烧杯内（开爪松放 → 抬空爪清电极撤离）
  ⑭ PressConfirmButton 按下导电率仪机顶「开始」键（高位接近 → 下探 → 合爪夹按钮 → 下压按钮顶 → 松爪 → 抬回）
后续步骤（读数）逐步追加。
"""
from .pick_surface_dish import PickSurfaceDish
from .move_dish_above_beaker import MoveDishAboveBeaker
from .pour_dish_into_beaker import PourDishIntoBeaker
from .return_surface_dish import ReturnSurfaceDish
from .pick_wash_bottle import PickWashBottle
from .squeeze_wash_bottle import SqueezeWashBottle
from .return_wash_bottle import ReturnWashBottle
from .pick_glass_rod import PickGlassRod
from .stir_in_beaker import StirInBeaker
from .return_glass_rod import ReturnGlassRod
from .lift_electrode import LiftElectrode
from .dip_electrode import DipElectrodeIntoBeaker
from .release_electrode import ReleaseElectrode
from .press_confirm_button import PressConfirmButton

__all__ = [
    "PickSurfaceDish",
    "MoveDishAboveBeaker",
    "PourDishIntoBeaker",
    "ReturnSurfaceDish",
    "PickWashBottle",
    "SqueezeWashBottle",
    "ReturnWashBottle",
    "PickGlassRod",
    "StirInBeaker",
    "ReturnGlassRod",
    "LiftElectrode",
    "DipElectrodeIntoBeaker",
    "ReleaseElectrode",
    "PressConfirmButton",
]
