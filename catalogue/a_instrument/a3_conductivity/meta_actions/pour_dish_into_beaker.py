# -*- coding: utf-8 -*-
"""A3 ③ 倾斜玻璃皿把粉末倒入烧杯（2026-08-28 用户逐字「机械臂只在 x 方向动，然后把
玻璃皿倾斜，把粉末都倒进烧杯」，A3 第三个动作）。

MoveDishAboveBeaker 之后皿已 attached 悬空在烧杯口正上方（TCP DISH_ABOVE_BEAKER
(0.392,0.0807,1.15)）。本动作：
  ① 竖直下降到倒粉高度（TCP → POUR_TCP (0.392,0.0807,0.95)，纯 z 降）
  ② 原地倾斜（TCP 不动，POUR_TILT_TCP=POUR_TCP (0.392,0.0807,0.95)，orient=TILT_ORIENT）：
     手腕绕世界 Y 轴 pitch 60°，皿 -X 侧下降、粉末沿 -X 滑出。皿绕 tool_center 倾斜只使皿原点
     +x 偏 ~0.004m、皿中心 ~0.001m（相对烧杯口 76mm 可忽略）→ 原地转、无需 x 补偿。
     MoveAction 判定该段位置无变化（无轴超 AXIS_EPS）→ 单次 IK 解倾斜位姿（warm start 当前
     关节），每帧关节钳制逼近、朝向收敛（<ORIENT_EPS 8.6°）连续 3 帧才冻结（倾斜渐进完成）。
  ③ 保持 POUR_HOLD=90 帧：task 检测到 tool+Z 倾斜（x 分量>0.5）→ 触发粉末下落动画
     （粉粒从皿 -x 低侧坠入烧杯口），~1.5s 落完显烧杯内粉末。

（2026-08-29 用户改 beaker.usd 直立烧杯：烧杯口朝上正对上方，口顶视图中心 = (0.392,0.0807)，
  皿对齐点从侧躺口 (0.4070,0.1274) 改为直立口中心，粉末竖直坠入直立口内。）


夹爪全程保持 GRIP_DISH 闭合（grip_target 由 controller 从 MoveDishAboveBeaker 传播，
本动作无 grip 原子动作、首帧不开爪——工具已吸附类，dip 铂丝同款）。倾斜用显式 orient
（与 PickSurfaceDish 的「不传 orient」不同：倒粉点 z=0.95 比抓点高 98mm、且向前伸展姿态
远离近奇异区，FK 可解；若 IK FAIL 用户实测再回退）。
"""
from ._base import BaseMetaAction, mv, hold
from .constants import POUR_TCP, POUR_TILT_TCP, TILT_ORIENT, POUR_HOLD


class PourDishIntoBeaker(BaseMetaAction):
    """倾斜玻璃皿把粉末倒入烧杯：下降 → 倾斜 + x 补偿 → 保持（粉末下落）。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, POUR_TCP),                            # ① 竖直下降到倒粉高度（纯 z，皿平放）
            mv(e, POUR_TILT_TCP, orient=TILT_ORIENT),   # ② 绕 Y 倾斜 60°（原地，无 x 补偿）
            hold(e, POUR_HOLD),                         # ③ 保持，task 触发粉末下落动画走完
        ]
