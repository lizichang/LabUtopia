# -*- coding: utf-8 -*-
"""A3 ④ 把倒完粉的空玻璃皿放回分析天平秤盘（2026-08-28 用户「现在加动作放回玻璃皿」，A3 第四个动作）。

PourDishIntoBeaker 之后皿已 attached 悬空在烧杯口上方（TCP POUR_TILT_TCP (0.352,0.1424,0.92)、
手腕倾斜 60°），粉末已全部倒进烧杯（粉堆 shrink 到 12% 并隐藏）。本动作把空皿放回秤盘：
  ① 转竖直 + 水平移回天平上方（TCP → (0.3442,0.5550,0.92)，z 锁 0.92 不变、纯 y 横移 0.41m；
     显式 orient=ORIENT_DOWN 手指朝下）——**必须显式传竖直朝向**：从倾斜 60° 转回竖直，
     orient=None 时 MoveAction 的 _rot_target=None、朝向收敛不检查，位置到位即冻 → 皿还斜着
     （d2s ReturnSpatula 同款教训，边移边调直）。
  ② 竖直下探到放回高度（TCP → DISH_GRASP (0.3442,0.5550,0.8520)，纯 z 降 68mm）——低 z 处
     不传显式 orient（A2 旋光管同款教训，低 z 显式朝向 FK 检查解不出 IK → force-done），此时
     朝向已竖直、引擎默认朝向即竖直，不会偏。
  ③ 开爪松放（grip GRIP_OPEN=0.04）：task 检测 opening > DISH_GRIP_OPEN(0.038) → released，
     皿+粉堆回 rest 位姿（皿原点 0.8474 = DISH_GRASP 0.8520 − 0.0046，零跳变）。
  ④ 抬回安全高位撤离（TCP → DISH_LIFT (0.3442,0.5550,1.15)），皿留在秤盘上。

夹爪全程保持 GRIP_DISH 闭合直到 ③（grip_target 由 controller 从 PourDishIntoBeaker 传播，
①② 无 grip 原子动作、首帧不开爪——工具已吸附类，dip 铂丝同款）。放回后皿空、粉已在烧杯，
后续步骤（配液 / 电极浸入 / 读数）逐步追加。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import DISH_XY, POUR_TCP_Z, ORIENT_DOWN, DISH_GRASP, GRIP_OPEN, DISH_LIFT


class ReturnSurfaceDish(BaseMetaAction):
    """把空玻璃皿放回天平秤盘：转竖直横移回 → 竖直下探 → 开爪松放 → 抬回撤离。"""

    def _build_actions(self):
        e = self.engine
        lx, ly = DISH_XY
        above = (lx, ly, POUR_TCP_Z)   # 天平上方高位（z 锁 0.92），边移边转竖直
        return [
            mv(e, above, orient=ORIENT_DOWN),   # ① 转竖直 + 水平移回天平上方（显式竖直调直）
            mv(e, DISH_GRASP),                  # ② 竖直下探到放回高度（低 z 不传 orient）
            grip(e, GRIP_OPEN, 40),             # ③ 开爪松放（task → released，皿回 rest）
            mv(e, DISH_LIFT),                   # ④ 抬回安全高位撤离（皿留秤盘）
        ]
