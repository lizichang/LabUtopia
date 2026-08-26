"""元动作 S6：拿起试管 → 高位震荡来回 N 下 → 放回试管架（观察粉末溶于水的现象）。

参考 d3l TubeShakePass（步9 抓起试管震荡混合）——同样的 ShakeAction 水平正弦振荡 +
同样的持握约定（抓管身中段管口下 14mm，管底 0.1393m 吊夹爪下方，纯平移保竖立）。

抓试管：试管立插架近侧左孔 (0.659,0.241)，管口 z=0.9593、管高 0.153m、架顶 0.917——管身
露在架上的可握段只有 42mm（0.917..0.9593），抓管口下 14mm（TCP z≈0.9453）。
持握 = TCP + (0,0,-0.1393)（管底吊在夹爪下方，task 纯平移保竖立），管内粉/水随管平移。

轨迹（TCP 世界坐标，手指默认朝下）：
  ① 高位接近试管上方   mv((tx,ty,H))
  ② 垂直下探抓管身中段 mv(TUBE_GRASP_TCP)          # 管口下 14mm（task: tube attached）
  ③ 合爪夹紧           grip(GRIP_TUBE, 60)         # 开度=管径/2
  ④ 垂直提出到震荡高度 mv(SHAKE_CENTER_TCP, 5)     # 管底 0.951 清架顶 0.917
  ⑤ 震荡来回 N 下       shake(center, cycles=N)     # 正弦往复，粉/水随管平移
  ⑥ 震荡后停留 5 秒     mv(SHAKE_CENTER_TCP, SHAKE_HOLD_FRAMES)  # 高位悬停观察溶解
  ⑦ 垂直下探回架       mv(TUBE_GRASP_TCP)          # 管底回插孔
  ⑧ 松开释放           grip(GRIP_OPEN, 25)          # task: tube released → rest
  ⑨ 垂直归位           mv((tx,ty,H))
"""
from ._base import BaseMetaAction, mv, grip, shake
from .constants import (H, GRIP_OPEN, GRIP_TUBE, ORIENT_FWD,
                        TUBE_XY, TUBE_GRASP_TCP,
                        SHAKE_CENTER_TCP, SHAKE_AMPLITUDE, SHAKE_PERIOD,
                        SHAKE_HOLD_FRAMES)


class TubeShakePass(BaseMetaAction):
    """拿起试管 → 高位震荡来回 cycles 下 → 放回试管架（液柱/粉随管平移）。

    shake_center 可覆盖震荡中心（d3s 把 z 抬高到 1.18 防穿模/碰滴管）；默认 None 用本包
    SHAKE_CENTER_TCP（d2s z=1.09）。
    """

    def __init__(self, engine, cycles=3, shake_center=None):
        self.cycles = max(1, int(cycles))
        self.shake_center = shake_center if shake_center is not None else SHAKE_CENTER_TCP
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        c = self.shake_center
        return [
            mv(e, (tx, ty, H), orient=ORIENT_FWD),             # ① 高位接近试管上方（手指朝前）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD),          # ② 垂直下探抓管身中段（管口下 14mm）
            grip(e, GRIP_TUBE, 60),                            # ③ 合爪夹紧试管（开度=管径/2）
            mv(e, c, 5, orient=ORIENT_FWD),                    # ④ 垂直提出到震荡高度（管底清架顶）
            shake(e, c,                                       # ⑤ 高位震荡来回 N 下（正弦往复）
                  amplitude=SHAKE_AMPLITUDE,
                  cycles=self.cycles, period=SHAKE_PERIOD, orient=ORIENT_FWD),
            mv(e, c, SHAKE_HOLD_FRAMES, orient=ORIENT_FWD),    # ⑥ 停留 5 秒（观察溶解）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD),          # ⑦ 垂直下探回架（管底回插孔）
            grip(e, GRIP_OPEN, 25),                            # ⑧ 松开释放（task: tube released → rest）
            mv(e, (tx, ty, H), orient=ORIENT_FWD),             # ⑨ 垂直归位
        ]
