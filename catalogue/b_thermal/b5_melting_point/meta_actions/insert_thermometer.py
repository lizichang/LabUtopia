# -*- coding: utf-8 -*-
"""InsertThermometerIntoTube：吸附后整组（温度计+毛细管）高位对齐管口 XY → 竖直下探插管
（塞子封口）。

用户 2026-09-01 逐字：「然后将温度计竖直以后要抬高竖直插入试管（xy坐标先对齐，然后最后
只向下运动），现在你是水平穿模按进去的」——旧版从低位直接横移到管口再下探，会水平穿模穿
过提勒管壁。

2026-09-02 倒插改流程：PickThermometer 已竖直提出并法兰翻转泡朝下，且夹点停在
THERMO_HIGH=1.25。Insert 从 INSERT_CLEAR_Z=1.18（泡底 1.096 > 管口 1.078 清高；用户「对齐的
时候 z 坐标不用太高」降 1.25→1.18 保 IK 可达）对齐后下探，无需中转/抬升，两步即可：
  ① 高位对齐管口 XY（只动 x,y；泡底 1.096 高于管口 1.078，横移不碰壁；对齐 (0.40,0.0029,1.18)
     =0.844m 底座 < 0.885m 安全）
  ② 竖直下探插管（只动 z，泡底沉到 INSERT_BULB_Z=0.939、塞子封管口；紧朝向防杆碰口沿）

朝向：全程 ORIENT_VERT（=Rx(180°)·ORIENT_FWD 泡朝下），温度计矩阵持握随夹爪、毛细管相对
贴泡（task 侧 _update_stuck_capillary）整组竖直插下。

几何：法兰翻转后泡底在夹点正下方 THERMO_BULB_DZ=0.084；下探后夹点 0.939+0.084=1.023 →
泡底 0.939（塞子封管口 1.078）。毛细管封口端贴泡、开口端沿杆朝上（平行贴杆），随整组竖直
插下，均在 Ø25mm 主管内腔（泡半径 5mm + 毛细管偏移 5.75mm < 12.5mm），无穿模。
"""
from ._base import BaseMetaAction, mv
from .constants import (TUBE_XY, INSERT_BULB_Z, THERMO_BULB_DZ,
                        INSERT_CLEAR_Z, ORIENT_VERT)


class InsertThermometerIntoTube(BaseMetaAction):
    """吸附后：高位对齐管口 XY → 竖直下探插管（塞子封口）。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY                  # 提勒管口轴心 xy
        insert_z = INSERT_BULB_Z + THERMO_BULB_DZ   # 0.939 + 0.084 = 1.023
        return [
            # ① 高位对齐管口 XY（INSERT_CLEAR_Z=1.18，泡底 1.096 > 管口 1.078 清高横移）
            mv(e, (tx, ty, INSERT_CLEAR_Z), orient=ORIENT_VERT),
            # ② 竖直下探插管（只动 z，泡底沉到 0.939、塞子封管口；紧朝向防杆碰口沿）
            mv(e, (tx, ty, insert_z), orient=ORIENT_VERT, orient_eps=0.03, dwell=60),
        ]
