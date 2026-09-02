"""D9 氧气检验元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 d2s/d3l/d6/d7/flametest 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。全程默认朝向（手指朝下）——木条/火柴/灯帽均
水平横躺/竖直，手指朝下竖直夹杆身（B2 火柴/盖帽同款，已验证）。

用户逐字（2026-09-01）动作链对应的 8 元动作：
  ① CapOffPass     摘灯帽：帽从灯口 → 桌面 CAP_REST
  ② IgniteLamp     火柴点燃酒精灯（照 B2 LightFlamePass）
  ③ PickSplint     夹木条
  ④ LightSplint    木条端伸入灯焰点燃
  ⑤ BlowOutSplint  快速摆动熄火（甩灭明火留余烬）
  ⑥ HoverSplint    火星端悬停氧气试管口上方（不伸入）
  ⑦ ReturnSplint   木条取出归位
  ⑧ CapOnPass      盖灯帽：帽从桌面 → 灯口（盖帽熄焰）
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .cap_off import CapOffPass
from .ignite_lamp import IgniteLamp
from .pick_splint import PickSplint
from .light_splint import LightSplint
from .blow_out_splint import BlowOutSplint
from .hover_splint import HoverSplint
from .return_splint import ReturnSplint
from .cap_on import CapOnPass

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake",
           "CapOffPass", "IgniteLamp", "PickSplint", "LightSplint",
           "BlowOutSplint", "HoverSplint", "ReturnSplint", "CapOnPass"]
