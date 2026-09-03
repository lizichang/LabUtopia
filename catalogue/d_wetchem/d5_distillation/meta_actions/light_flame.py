# -*- coding: utf-8 -*-
"""元动作 ①：拿火柴点燃酒精灯（D5 蒸馏分离，机械臂唯一动作）。

D5 = 预组装蒸馏装置（酒精灯/烧瓶/冷凝管/接液瓶东侧集群），机械臂只做"取火柴 → 触灯芯 →
放回"。几何实测（2026-09-03 忠实用户 tmp）：火柴尾端原点 (0.5295,0.2918,0.8043) 头朝 +X，
灯芯顶 WICK=(0.6981,0.4610,0.9007)，灯玻璃体顶 0.8897、烧瓶底 0.9091。

夹爪默认手指朝下，纯平移持握火柴杆（杆横躺），火柴全程水平头朝 +X。低位运移 z 锁
LIFT_IGN_Z=0.899（灯体顶与烧瓶底之间仅 2cm 高的缝，太高撞烧瓶、太低刮灯体）。火柴从
灯 **西侧** 进灯芯（火柴前端 max x=0.6273 < 灯体 west 边 0.6545，西侧是空档）→ 路径两段直角：
  先 纯 +Y（x 锁火柴抓点列 x=0.5775）：火柴行 y0.2918 → 灯芯行 y0.4610（北移，全程在灯西侧外）；
  再 纯 +X（y 锁 0.4610）：火柴头从 x0.6273 推进到 WICK 处 x0.6483 → 落灯芯。
回程严格原路（先 -X 再 -Y 退），降回抓点高度后松爪（task 火柴生命周期写回 rest），高位归位。

流程（一次持握，无循环，全程默认朝向手指朝下）：
  ① 取火柴：高位接近 → 竖直下探到杆身中部抓点，合爪夹住（GRIP_MATCH 贴合 Ø3mm 杆）。
  ② 低位运移：竖直提起至 LIFT_IGN_Z=0.899 → 纯 +Y 北移到灯芯行（西侧空档）→ 纯 +X 推进。
  ③ 触灯芯：火柴头（夹爪 +X 0.0498）落灯芯顶 WICK，task 检测头近灯芯连续 15 帧 → 点火
     （火焰 reveal + B5 同款 flicker，见 task._step_match_ignite / _step_flame_anim）。
  ④ 放回：原路（-X → -Y）退回火柴上方 → 降回抓点 → 松爪 → 高位归位。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近     mv((mx,my,H))
  ② 竖直下探     mv(MATCH_GRASP)                 # 降杆身中心，两指竖直夹杆
  ③ 停顿稳定     hold(SETTLE)
  ④ 合爪夹紧     grip(GRIP_MATCH, 60)            # task 检测 attached
  ⑤ 竖直提出     mv((mx,my,LIFT_IGN_Z), 5)       # 低位 0.899
  ⑥ 北移(纯+Y)   mv((mx,iy,LIFT_IGN_Z), 8)       # x 锁抓点列，火柴头仍在灯西侧外
  ⑦ 东进触芯     mv(IGNITE, 20)                  # y 锁灯芯行，纯 +X 头落灯芯（dwell 20）
  ⑧ 原路 -X      mv((mx,iy,LIFT_IGN_Z))
  ⑨ 原路 -Y      mv((mx,my,LIFT_IGN_Z))
  ⑩ 降回抓点     mv(MATCH_GRASP)
  ⑪ 松爪释放     grip(GRIP_OPEN, 25)             # task: released → 火柴写回 rest
  ⑫ 高位归位     mv((mx,my,H))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP, LIFT_IGN_Z,
                        IGNITE, MATCH_HIGH)


class LightFlamePass(BaseMetaAction):
    """取火柴 → 低位两段直角（先 +Y 北移、再 +X 东进）→ 触灯芯点燃 → 原路放回。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        iy = IGNITE[1]                      # 灯芯行 y（北移终点行）
        return [
            mv(e, (mx, my, MATCH_HIGH)),            # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),                     # ② 竖直下探到杆身中心（两指竖直夹杆）
            hold(e, SETTLE),                        # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, LIFT_IGN_Z), 5),         # ⑤ 竖直提出（低位 0.899）
            mv(e, (mx, iy, LIFT_IGN_Z), 8),         # ⑥ 纯 +Y 北移：x 锁抓点列，灯西侧空档
            mv(e, IGNITE, 20),                      # ⑦ 纯 +X 东进：头落灯芯（dwell 20 点火）
            mv(e, (mx, iy, LIFT_IGN_Z)),            # ⑧ 原路 -X 退回（出灯芯行）
            mv(e, (mx, my, LIFT_IGN_Z)),            # ⑨ 原路 -Y 退回（出火柴列 → 火柴上方）
            mv(e, MATCH_GRASP),                     # ⑩ 降回抓点高度（= 火柴归位姿态）
            grip(e, GRIP_OPEN, 25),                 # ⑪ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),            # ⑫ 高位归位
        ]
