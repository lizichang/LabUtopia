# -*- coding: utf-8 -*-
"""A3 ① 竖直夹住玻璃皿提起来（2026-08-27 用户逐字，A3 第一个动作）。

拿表面皿：高位接近 → 竖直下探到皿中心 → 停顿 → 合爪夹皿壁 → 竖直提出。
**不用显式 orient**：低 z 处显式朝向的 FK 检查解不出 IK（A2 旋光管同款教训，
"IK FAIL … force-done"→ 永不 attach）；用引擎默认朝向（手指朝下、开合沿 Y），
两指在 y=±0.027 夹 Ø54 碗壁（皿是浅碗：Ø6 底陡峭外翻 Ø60 口沿；GRIP_DISH=0.027 指腹压住外翻
碗壁，口沿 clip 3mm）。抓取 TCP 高度 0.8520（tool_center 比指端高 0.027 → 指端 0.825 进天平机身顶
15mm——无碰撞仅接近时短暂穿入；皿底 0.8474 在指端上方 22.4mm、皿中心高出指端 25.75mm——五改：再往下伸 1cm）。

task 侧：皿 rest → 近抓点+合爪 → attached（6-DOF 跟随，皿原点=TCP−0.0046）→ 开爪 → released
（皿+粉回 rest）；粉堆（独立 prim）随皿同位移。释放阈值 DISH_GRIP_OPEN=0.038（GRIP 0.027
之上明显裕量，合爪后不会 attach 即 release）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import H, SETTLE, GRIP_DISH, DISH_XY, DISH_GRASP, DISH_LIFT


class PickSurfaceDish(BaseMetaAction):
    """竖直夹住玻璃皿提起来：接近 → 下探 → 夹紧 → 提出。"""

    def _build_actions(self):
        e = self.engine
        lx, ly = DISH_XY
        return [
            mv(e, (lx, ly, H)),            # ① 高位接近皿上方
            mv(e, DISH_GRASP),             # ② 竖直下探到皿中心（两指夹 Ø60 皿壁）
            hold(e, SETTLE),               # ③ 停顿稳定
            grip(e, GRIP_DISH, 60),        # ④ 合爪夹住皿壁（task 检测 attached）
            mv(e, DISH_LIFT, 5),           # ⑤ 竖直提出（皿随夹爪悬空）
        ]
