# -*- coding: utf-8 -*-
"""A2 旋光仪测量元动作包。

10 元动作（controller 编排顺序）：
  ① PrePoseWash 预摆 d2s 洗瓶入口姿势 → ② PickWashBottle 拿洗瓶 → ③ SqueezeWater 挤水
  → ④ ReturnWashBottle 放回 → ⑤ TubeShakePass 试管震荡 → ⑥ DropperTransferPass 滴管转移
  （吸试管内液 3 次挤进旋光管加液口，2026-08-27 替代倒液）→ ⑦ PickPolarimeterTube 拿旋光管
  → ⑧ PlaceOnRails 放导轨 → ⑨ CloseLidPass 拨回翻盖（2026-08-27 新增）→ ⑩ PressStartPass 按启动键
"""
from .prepose_wash import PrePoseWash
from .pick_wash_bottle import PickWashBottle
from .squeeze_water import SqueezeWater
from .return_wash_bottle import ReturnWashBottle
from .tube_shake_pass import TubeShakePass
from .dropper_transfer_pass import DropperTransferPass
from .pick_polarimeter_tube import PickPolarimeterTube
from .place_on_rails import PlaceOnRails
from .close_lid_pass import CloseLidPass
from .press_start_pass import PressStartPass

__all__ = [
    "PrePoseWash", "PickWashBottle", "SqueezeWater", "ReturnWashBottle", "TubeShakePass",
    "DropperTransferPass",
    "PickPolarimeterTube", "PlaceOnRails", "CloseLidPass", "PressStartPass",
]
