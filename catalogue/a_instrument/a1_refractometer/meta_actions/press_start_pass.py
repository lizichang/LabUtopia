"""元动作 ④：按下测量键——滴样合盖后，按下机顶红色测量键触发测量读数。

按钮 = /World/Refractometer/start_button（机顶红色矮圆柱，中心 (0.30,0.05)、直径
Ø32mm、凸起 6mm：底 0.915..顶 0.921，pxr 实测，2026-08-26 直径加宽一倍）。按下 =
爪子先合爪夹住按钮两侧（GRIP_BUTTON=按钮半径），再垂直下探到按钮顶 0.921（手指夹着
按钮下压），task._ButtonLifecycle 检测爪子近按钮顶 → 触发测量：先 ScreenMeasuring
（红进度条测量中），MEASURE_FRAMES 帧后切 ScreenGlow（绿满条 + nD 1.4000 / 20.0°C）。
按钮无碰撞，是"按"到顶即触发，非物理下压。旧版爪开 GRIP_OPEN=0.04 时按钮 Ø16mm 悬空
在两根手指间没被按下，故合爪夹按钮两侧（用户 2026-08-26）。

轨迹（TCP 世界坐标，手指朝下）：
  ① 高位接近按钮上方   mv(BTN_APPROACH)     # (0.30,0.05,H)
  ② 下探预按位         mv(BTN_PREPRESS)     # 按钮顶上方 2cm (0.30,0.05,0.94)
  ③ 合爪夹按钮         grip(GRIP_BUTTON, 30) # 手指并拢夹住按钮两侧（Ø32mm）
  ④ 按下到底           mv(BTN_PRESS, 30)    # (0.30,0.05,0.916)，下探路过 0.921 检测点
                                               # 触发（按钮下沉）→ 手指压到下沉后按钮顶
  ⑤ 开爪松手           grip(GRIP_OPEN, 20)
  ⑥ 抬起回高位         mv(BTN_APPROACH)     # 抬离 > 0.930 触发 task 让按钮缓慢弹回
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (BTN_APPROACH, BTN_PREPRESS, BTN_PRESS,
                        GRIP_OPEN, GRIP_BUTTON)


class PressStartPass(BaseMetaAction):
    """按下测量键：合爪夹按钮两侧 → 垂直下探按钮顶，触发测量读数。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, BTN_APPROACH),       # ① 高位接近按钮上方
            mv(e, BTN_PREPRESS),       # ② 下探预按位（按钮顶上方 2cm，手指张开）
            grip(e, GRIP_BUTTON, 30),  # ③ 合爪夹按钮两侧（Ø32mm）
            mv(e, BTN_PRESS, 30),      # ④ 按下按钮顶（task 触发测量）
            grip(e, GRIP_OPEN, 20),    # ⑤ 开爪松手
            mv(e, BTN_APPROACH),       # ⑥ 抬起回高位
        ]
