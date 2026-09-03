# -*- coding: utf-8 -*-
"""InsertThermometerIntoTube：吸附后整组（温度计+毛细管）高位对齐管口 XY → **直接松爪**让
温度计靠重力落进提勒管、橡胶塞正好卡住管口（弃竖直下探按入）。

用户 2026-09-02 逐字：「整个过程都没有什么问题就是最把温度计要插进试管里面的时候有问题，
整个过程应该是位置比较高的地方完成对准然后直接松爪让温度计它落进去，塞子正好对准」——旧版
从高位竖直下探按进管口（mv 到 insert_z=1.023），用户要的是「高位对准 → 松爪 → 自由落体落进、
塞子对准卡口」，下探按入动作删除。

轨迹（全程 ORIENT_VERT=Rx(180°)·ORIENT_FWD 泡朝下）：
  ① 先竖直上提（x,y 锁温度计夹点）到清高 INSERT_CLEAR_Z=1.22（泡底 1.136 > 管口 1.078，先上移再横移）
  ② 只 -Y 对准（y 从夹点 0.2696 → 管口 0.0059，x 锁夹点 0.3471，纯 y 平移）
  ③ 只 +X 对准（x 从 0.3471 → 管口 0.397，y 锁 0.0059，纯 x 平移）
  ④ 直接松爪 grip(GRIP_OPEN)（不再下探）——温度计落体/塞子卡口交 task 侧 _ThermometerLifecycle
     dropping 状态（松爪触发落体动画，DROP_FRAMES 帧加速落到 INSERT_THERMO_ORIGIN_Z=0.941）。

对齐 (0.397,0.0059,1.22) 距底座 [-0.048,-0.311,0.71] 0.747m < 0.885m 安全（用户 2026-09-02 报
「松爪子高度太高」1.35→1.22 降 13cm）。松爪后 task 侧温度计 origin 从高位（夹点 1.22 下方
0.084=1.136）自由落体到 0.941，塞中心 0.137 精确封管口 1.078；毛细管贴泡随整组下落
（_update_stuck_capillary）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import TUBE_XY, INSERT_CLEAR_Z, ORIENT_VERT, GRIP_OPEN, THERMO_GRASP


class InsertThermometerIntoTube(BaseMetaAction):
    """吸附后：高位对齐管口 XY → 直接松爪（温度计落进管口、塞子卡口，task 侧落体动画）。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY                  # 提勒管口轴心 xy
        gx, gy, _ = THERMO_GRASP          # 温度计当前 xy（pick 结束位 = 夹点 xy，泡朝下竖直）
        return [
            # ① 先竖直上提（x,y 锁温度计位）到清高 INSERT_CLEAR_Z——用户 2026-09-02「机械臂开始应该
            #    直接往上移动」，先上移再横移，避免从温度计斜切穿提勒管/夹
            mv(e, (gx, gy, INSERT_CLEAR_Z), orient=ORIENT_VERT),
            # ② 只 -Y 对准（y 夹点→管口，x 锁夹点，纯 y 平移）——用户 2026-09-02「旋转完只往 -y 方向移动」
            mv(e, (gx, ty, INSERT_CLEAR_Z), orient=ORIENT_VERT),
            # ③ 只 +X 对准（x 夹点→管口，y 锁管口，纯 x 平移）——「-y 对准之后，只往 +x 方向移动」
            mv(e, (tx, ty, INSERT_CLEAR_Z), orient=ORIENT_VERT),
            # ④ 直接松爪（dwell 60 帧给 task 侧落体动画留时间），温度计落进管口、塞子卡口
            grip(e, GRIP_OPEN, 60),
        ]
