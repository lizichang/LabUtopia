"""元动作 ②a：手指朝前（朝向 camera1）水平横夹洗瓶肚子（可挤压的瓶身），竖直提起。

用户动作要求（2026-08-25 逐字，对齐 D2-S）：「现在加动作，机械臂像加药匙的方法一样水平横着
夹住wash bottle的肚子（就是能挤压的部分）」。

与 D2-S PickWashBottle 完全同构（ORIENT_FWD 手指朝前 + 夹持 + 提起）。洗瓶 2026-08-25 用户
「移动到 (0.370,0.525) 后 +Y→+X 转 90°」= rotZ -180°，红色嘴尖朝 +X：吸管转后沿 X 轴
（y≈0.525 中心线），背支瓶内 x≈0.368、前支出瓶前壁 x≈0.474、嘴尖 x≈0.476。手指 ORIENT_FWD
tool+Z=+X、两指沿 ±Y 张开夹瓶身 ±Y 面，吸管在指间、前支在指端外 → 正落不压管，
只需 x 偏开 -X 下探（避嘴尖），到位后水平移入瓶身中心夹持：
  ① 高位       mv((WASH_APPROACH_X, wy, H), orient=ORIENT_FWD)   # 高位且 x 偏开 0.30（瓶身 -X 壁前）
  ② 竖直下探   mv((WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD)   # x 偏移竖直降，手指到瓶身 -X 外侧
  ③ 水平移入   mv(WASH_GRASP, orient=ORIENT_FWD)                  # 锁 z/y，仅 x 平移入瓶身中心
  ④ 横夹肚子   grip(GRIP_WASHBOT)                                 # 半开度 0.030→6cm 开口压 6.4cm 软瓶身
  ⑤ 竖直提起   mv((wx, wy, WASH_LIFT), orient=ORIENT_FWD)         # 锁 x/y，仅 z 抬升 15cm
  ⑥ 往 +X      mv((WASH_TO_TUBE_X, wy, WASH_LIFT), orient=ORIENT_FWD)   # 红嘴 0.476→0.649
  ⑦ 往 -Y      mv((WASH_TO_TUBE_X, WASH_TO_TUBE_Y, WASH_LIFT), orient=ORIENT_FWD)  # 红嘴 y→0.231
                                                                 #   红嘴终位 (0.649,0.231,0.994)，管口 (0.659,0.241,0.959)

抓取高度 z=0.88（瓶身中部）：转后吸管沿 X 轴 y≈0.525 中心线，两指沿 ±Y 夹瓶身 ±Y 面，
吸管在指间、前支（x≈0.474）在指端（x≈0.398）外 7.6cm → 无碰撞。夹持宽度 = 半开度 0.030
（URDF 平移副开度=2×width；GRIP_OPEN=0.04 开 8cm > 6.4cm 瓶身可容纳）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_WASHBOT, ORIENT_FWD,
                        WASH_APPROACH_X, WASH_GRASP, WASH_LIFT,
                        WASH_TO_TUBE_X, WASH_TO_TUBE_Y)


class PickWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy, wz = WASH_GRASP
        return [
            mv(e, (WASH_APPROACH_X, wy, H), orient=ORIENT_FWD),    # ① 高位（x 偏开 -X 避开 +X 嘴尖）
            mv(e, (WASH_APPROACH_X, wy, wz), orient=ORIENT_FWD),   # ② 竖直下探到瓶身下段外侧
            mv(e, WASH_GRASP, orient=ORIENT_FWD),                  # ③ 水平移入瓶身中心（锁 y/z）
            grip(e, GRIP_WASHBOT, 60),                             # ④ 水平横夹肚子（开度 6cm）
            mv(e, (wx, wy, WASH_LIFT), orient=ORIENT_FWD),         # ⑤ 竖直提起 15cm（z 0.88→1.03）
            mv(e, (WASH_TO_TUBE_X, wy, WASH_LIFT), orient=ORIENT_FWD),   # ⑥ 往 +X 移 0.173
            mv(e, (WASH_TO_TUBE_X, WASH_TO_TUBE_Y, WASH_LIFT), orient=ORIENT_FWD),  # ⑦ 往 -Y 移 0.294
        ]
