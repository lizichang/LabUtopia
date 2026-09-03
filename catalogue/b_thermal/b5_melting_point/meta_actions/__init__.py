"""B5 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

B5 装样段（用户 2026-08-31「算了还是换一种方法」弃程序化旋转，改夹毛细管端部拎起自动竖直）
六个元动作 + 挂温度计段（用户 2026-09-01「还是用毛细管沾油吧」弃温度计泡蘸油，改毛细管中部
水平夹起蘸油；2026-09-02 改「蘸油后放回，再夹端部拎起竖直贴泡」弃横着贴泡）：
  ① PickCapillarySealedEnd  夹封口端(-X)拎起 → 开口端朝下（蘸粉准备，绕 Y +90°）
  ② DipCapillaryIntoPowder  水平移到粉丘上方 → 竖直下探开口端沉入粉丘 5mm 蘸粉
  ③ ReturnCapillaryToTable  蘸粉后放回桌面，毛细管倒成水平（松爪）
  ④ PickCapillaryOpenEnd    夹开口端(+X)拎起 → 封口端朝下（抖粉准备，绕 Y -90°）
  ⑤ TampCapillary           保持封口端朝下，竖直方向上下快速来回 10 次震实
  ⑥ ReturnCapillaryAfterTamp  震实后放回桌面，毛细管倒成水平（松爪）
  ⑦ PickCapillaryMiddle     从毛细管中部水平夹起（矩阵持握，保持水平）
  ⑧ DipCapillaryInOil       横移到油皿上方 → 下探封口端蘸石蜡油（用户 2026-09-02 再 -X 3cm）
  ⑧' ReturnCapillaryAfterOil  蘸油后放回桌面（矩阵持握提回原位松爪）
  ⑨' PickCapillarySealedEnd（第二次） 夹封口端拎起 → 封口端朝上、开口端垂下（贴泡准备）
  ⑩ StickCapillaryToBulb    横移到温度计泡旁 → 下探封口端竖贴泡（吸附）
  ⑪ PickThermometer         抓温度计杆身拎起 → 法兰滚 166° 泡翻朝下
  ⑫ InsertThermometerIntoTube  抬高安全高位 → 对齐管口 XY → 竖直下探插管（塞子封口）
  ⑬ LightFlamePass          拿起火柴 → 斜推进触灯芯点燃酒精灯 → 斜退放回（加热开始）
加热/观察熔点留待验收后接续。
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .pick_capillary import PickCapillarySealedEnd, PickCapillaryOpenEnd
from .move_preserve import MovePreserveAction, _R_to_quat_wxyz
from .dip_capillary import DipCapillaryIntoPowder
from .return_capillary import (ReturnCapillaryToTable, ReturnCapillaryAfterTamp,
                               ReturnCapillaryAfterOil)
from .tamp_capillary import TampCapillary, TampVerticalAction
from .pick_capillary_middle import PickCapillaryMiddle
from .dip_capillary_oil import DipCapillaryInOil
from .stick_capillary import StickCapillaryToBulb
from .pick_thermometer import PickThermometer
from .insert_thermometer import InsertThermometerIntoTube
from .light_flame import LightFlamePass
from .lamp_heat import LampHeatMovePass
from .cap_lamp import CapLampPass

__all__ = [
    "BaseMetaAction", "mv", "grip", "hold", "shake",
    "PickCapillarySealedEnd", "PickCapillaryOpenEnd",
    "MovePreserveAction", "DipCapillaryIntoPowder",
    "ReturnCapillaryToTable", "ReturnCapillaryAfterTamp", "ReturnCapillaryAfterOil",
    "TampCapillary", "TampVerticalAction",
    "PickCapillaryMiddle", "DipCapillaryInOil", "StickCapillaryToBulb",
    "PickThermometer", "InsertThermometerIntoTube", "LightFlamePass",
    "LampHeatMovePass", "CapLampPass",
]
