# -*- coding: utf-8 -*-
"""元动作 ①/④：夹毛细管**端部**纯竖直拎起（机械臂不旋转，管身由 task 侧 pivot 摆转竖直）。

用户 2026-08-31 逐字：「首先你夹这个毛细管还是以同样的方式夹但是夹的位置变了……夹起来的时候
因为重力水平细管就自动变成竖直了就像把它拎起来了」+ 2026-09-01 验收「机械臂就只是直上直下
的运动不要有其他的变化」「拎起来毛细管没有完全变竖直」——弃边提边转（RotateHeldAction），
拎起改 **MovePreserveAction 纯竖直**（只 z 推进、夹爪朝向不变），管身绕夹点摆转水平→竖直
交给 task 侧 pivot 持握（θ=swing_sign·90°·swing_frac，精确 90°）。
  ① PickCapillarySealedEnd  夹封口端(-X, 0.1730)拎起 → 开口端(+X)朝下（蘸粉准备，phase0 +90°）
  ④ PickCapillaryOpenEnd    夹开口端(+X, 0.2690)拎起 → 封口端(-X)朝下（抖粉准备，phase1 -90°）

夹法同旧版（照 b2 火柴：手指朝下竖直夹 Ø1.5mm 杆身），夹点离端部 2mm（2026-09-01「夹的
不够靠端」→ 从离端 2cm 挪到 2mm）。摆转方向由 task._phase 决定（不在此元动作内）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .move_preserve import MovePreserveAction
from .constants import (SETTLE, GRIP_CAPILLARY, GRASP_SEALED, GRASP_OPEN,
                        LIFT_HIGH, CAP_LIFT_Z)


class _PickCapillaryEnd(BaseMetaAction):
    """夹毛细管端部拎起（公共轨迹）：高位接近 → 下探夹端部 → 停顿 → 合爪 → 纯竖直拎起。"""

    grasp = GRASP_SEALED            # 子类覆盖：GRASP_SEALED（封口端）/ GRASP_OPEN（开口端）

    def __init__(self, engine, from_home=False):
        # from_home=True：开局第一个元动作（① 首抓封口端），先竖直上提再横移（见 _build_actions）。
        self.from_home = from_home
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        gx, gy, _ = self.grasp
        if self.from_home:
            # 开局先竖直上提（home 位 → 清高）再横移到毛细管正上方——用户 2026-09-02「第一步就是
            # 机械臂往上抬不要直接去拿毛细管」：旧版第一动作从 home 直接斜切扫过温度计/试管架穿模。
            # home 位由 fk_pose(ik_home) 反解（机器人默认即 ik_home），xy 与实际起始夹爪严格一致，
            # 上提段判为纯 z 直线（x,y 不变），再在高位横移、避开温度计泡顶 1.084 / 架顶 0.897。
            home_pos, _ = e.fk_pose(e.ik_home)
            hx, hy, _ = home_pos
            approach = [
                mv(e, (hx, hy, CAP_LIFT_Z)),      # ① 先竖直上提（home xy → 清高，纯 z）
                mv(e, (gx, gy, CAP_LIFT_Z)),      # ② 高位横移到毛细管正上方（清高，不碰温度计/架）
            ]
        else:
            # 非开局（④ 夹开口端 / ⑨ 再夹封口端）：臂已在毛细管附近，直接高位接近即可
            approach = [mv(e, (gx, gy, LIFT_HIGH))]   # 高位接近（手指朝下）
        return approach + [
            mv(e, self.grasp),               # 竖直下探到端部夹点（两指竖直夹杆身）
            hold(e, SETTLE),                 # 停顿稳定（合爪前多停稳）
            grip(e, GRIP_CAPILLARY, 60),     # 合爪夹住毛细管（task 检测 attached）
            MovePreserveAction(e, (gx, gy, LIFT_HIGH)),  # 纯竖直拎起（管身 task 侧摆转竖直）
        ]


class PickCapillarySealedEnd(_PickCapillaryEnd):
    """夹封口端(-X)拎起：task phase0 → 开口端(+X)朝下（蘸粉准备）。"""
    grasp = GRASP_SEALED


class PickCapillaryOpenEnd(_PickCapillaryEnd):
    """夹开口端(+X)拎起：task phase1 → 封口端(-X)朝下（抖粉准备）。"""
    grasp = GRASP_OPEN
