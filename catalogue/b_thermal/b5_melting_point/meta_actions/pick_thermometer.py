# -*- coding: utf-8 -*-
"""PickThermometer：抓倒插温度计竖直杆身（手指朝前 ORIENT_FWD 水平横夹），竖直提出后
只用法兰（panda_joint7）转 FLANGE_ANGLE=−166° 把泡翻朝下，再 IK 校直剩余 ~14°。

用户 2026-09-02 定案：「可以倒着插温度计，然后这样子就可以把毛细管贴上去了，水平横夹
出温度计后（像 d2s 夹药匙一样），只用法兰旋转 180 度就正过来了」——温度计倒插试管架
（泡朝上，见 gen_b5_scene.py），臂手指朝前 ORIENT_FWD 水平横夹竖直杆身（夹点 THERMO_GRASP
= (0.3471,0.2696,1.000)，塞子 0.957 上方、泡 1.068 下方），竖直提出清架顶后**只动法兰**
（joint7 限位 ±166° 转不到 180°，故转 166° 后 IK 校直补 ~14°，避免全臂 IK 在冻结 TCP 下
重解 180° 的退化/漂移）。温度计矩阵持握 6-DOF 刚性跟随夹爪（task 侧 _ThermometerLifecycle，
attach 时反解零跳变）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .flange_roll_tube import FlangeRollTubeAction
from .constants import (SETTLE, GRIP_THERMOMETER, THERMO_GRASP, THERMO_HIGH,
                        THERMO_APPROACH_X, ORIENT_FWD, ORIENT_VERT, FLANGE_ANGLE)


class PickThermometer(BaseMetaAction):
    """抓倒插温度计：手指朝前横夹竖直杆身 → 合爪 → 竖直提出 → 法兰滚 166° → IK 校直泡朝下。"""

    def _build_actions(self):
        e = self.engine
        gx, gy, gz = THERMO_GRASP
        return [
            # ① 从 -X 侧对齐杆身高度接近（用户 2026-09-02「拿温度计应该先对齐然后往 +X 移动，
            #    现在直接旋转到位直接跟温度计穿模」——旧高位竖直下探从泡正上方穿泡，改 -X 侧横移避开泡）
            mv(e, (gx - THERMO_APPROACH_X, gy, gz), orient=ORIENT_FWD),
            mv(e, THERMO_GRASP, orient=ORIENT_FWD),                        # ② +X 横移到杆身夹点
            hold(e, SETTLE),                                               # ③ 停顿稳定
            grip(e, GRIP_THERMOMETER, 60),                                 # ④ 合爪夹杆身
            mv(e, (gx, gy, THERMO_HIGH), 5, orient=ORIENT_FWD),            # ⑤ 竖直提出（挂环清架顶 0.914）
            # ⑥ 法兰滚 −166°（限位 ±166°，泡翻朝下 14° 短；纯关节不重解 IK，冻结 TCP 无漂移）
            FlangeRollTubeAction(angle=FLANGE_ANGLE),
            # ⑦ IK 校直剩余 ~14°：ORIENT_VERT=Rx(180°)·ORIENT_FWD（泡朝下精确朝向）
            mv(e, (gx, gy, THERMO_HIGH), orient=ORIENT_VERT, linewalk=False, orient_eps=0.03),
        ]
