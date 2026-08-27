# -*- coding: utf-8 -*-
"""A2 旋光仪测量元动作包。

10 元动作（controller 编排顺序）：
  ① PickWashBottle 拿洗瓶 → ② SqueezeWater 挤水 → ③ ReturnWashBottle 放回
  → ④ TubeShakePass 试管震荡 → ⑤ PickTestTube 拿试管 → ⑥ PourToTube 倒液进旋光管
  → ⑦ ReturnTestTube 放回试管 → ⑧ PickPolarimeterTube 拿旋光管
  → ⑨ PlaceOnRails 放导轨 → ⑩ PressStartPass 按启动键
"""
from .pick_wash_bottle import PickWashBottle
from .squeeze_water import SqueezeWater
from .return_wash_bottle import ReturnWashBottle
from .tube_shake_pass import TubeShakePass
from .pick_test_tube import PickTestTube
from .pour_to_tube import PourToTube
from .return_test_tube import ReturnTestTube
from .pick_polarimeter_tube import PickPolarimeterTube
from .place_on_rails import PlaceOnRails
from .press_start_pass import PressStartPass

__all__ = [
    "PickWashBottle", "SqueezeWater", "ReturnWashBottle", "TubeShakePass",
    "PickTestTube", "PourToTube", "ReturnTestTube",
    "PickPolarimeterTube", "PlaceOnRails", "PressStartPass",
]
