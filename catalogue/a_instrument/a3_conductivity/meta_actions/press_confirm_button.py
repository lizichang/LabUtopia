# -*- coding: utf-8 -*-
"""A3 ⑭ 按下导电率仪机顶「开始」键（2026-08-30 用户「把开始按钮放到顶部」）。

⑬ ReleaseElectrode 之后电极留在烧杯内、夹爪空。本动作到导电率仪**机顶**红色「开始」键
（/World/Meter/start_button，中心 (0.3549,−0.133,0.908)、顶 0.911；front 面板确认键已删——
贴竖直面板太前+低 z，水平横向按 ORIENT_FWD 手腕近奇异 IK 卡住，故把键放机顶垂直按下）：
  ① 高位接近按钮上方  mv(BTN_APPROACH)     # (0.3549,−0.133,H)
  ② 下探预按位        mv(BTN_PREPRESS)     # 触顶上方 2cm (0.3549,−0.133,0.958)
  ③ 完全闭合          grip(GRIP_BUTTON)    # 0.001 完全闭合（2026-08-30 用户「爪子应完全闭合」）
  ④ 按下到按钮顶      mv(BTN_PRESS, dwell) # (0.3549,−0.133,0.938)，闭合指端压按钮顶
  ⑤ 开爪松手          grip(GRIP_OPEN)
  ⑥ 抬回高位          mv(BTN_APPROACH)

朝向 = 引擎默认（手指朝下）全程（同 ⑬ ReleaseElectrode 连续，不传 orient）：机顶按钮垂直
下探为正前可达（皿 0.852/电极 0.9625 手指朝下均已证）。按钮无碰撞，是「按」到触发点即下沉
（task._ButtonLifecycle 检测爪子近按钮顶 → 按钮下沉 5mm + 显示测量进度条 → 结果读数）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (BTN_APPROACH, BTN_PREPRESS, BTN_PRESS,
                        GRIP_BUTTON, GRIP_OPEN, BTN_DWELL)


class PressConfirmButton(BaseMetaAction):
    """按下机顶开始键：完全闭合爪子 → 垂直下探按钮顶，触发测量。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, BTN_APPROACH),                 # ① 高位接近按钮上方
            mv(e, BTN_PREPRESS),                 # ② 下探预按位（触顶上方 2cm，手指张开）
            grip(e, GRIP_BUTTON, 30),            # ③ 完全闭合爪子（0.001）
            mv(e, BTN_PRESS, dwell=BTN_DWELL),   # ④ 闭合指端压按钮顶（0.938）
            grip(e, GRIP_OPEN, 20),              # ⑤ 开爪松手
            mv(e, BTN_APPROACH),                 # ⑥ 抬起回高位
        ]
