"""元动作 ③：拿起试管 → 高位震荡来回 N 下 → 放回试管架（TUBE_SHAKE_PASS）。

V7 步9（边加边振摇）：两支滴管分别滴完样品/酸后，抓起试管震荡让液体混合
（本实现放在 AcidPass 之后作为收尾一步；每次滴加即振摇留后续轮次）。

抓试管：试管立插前排左孔，管口 z=0.9593、管高 0.153m、架顶 0.917——管身露在
架上的可握段只有 42mm（0.917..0.9593），抓**管口下 14mm**（TCP z≈0.9453）。
持握 = TCP + TUBE_HELD_OFFSET(0,0,-0.139)（管底 0.139m 吊在夹爪下方，task 纯平移
保竖立），管内液柱随管平移。夹爪开度 GRIP_TUBE≈Ø19.2mm/2=0.0096（同胶头/2 逻辑）。

轨迹（TCP 世界坐标，手指默认朝下）：
  ① 高位接近试管上方   mv((tx,ty,H))
  ② 垂直下探抓管身中段 mv(TUBE_GRASP_TCP)          # 管口下 14mm（task: tube attached）
  ③ 合爪夹紧           grip(GRIP_TUBE, 60)         # 开度=管径/2
  ④ 垂直提出到震荡高度 mv(SHAKE_CENTER_TCP, 5)     # 管底 0.951 清架顶 0.917
  ⑤ 震荡来回 N 下       shake(center, cycles=N)     # 正弦往复，液柱随管平移
  ⑥ 震荡后停留 5 秒     mv(SHAKE_CENTER_TCP, SHAKE_HOLD_FRAMES)  # 高位悬停（现象持续→
                                                      # 3s 后消退，见 task._step_phenomena）
  ⑦ 垂直下探回架       mv(TUBE_GRASP_TCP)          # 管底回插孔
  ⑧ 松开释放           grip(GRIP_OPEN, 25)          # task: tube released → rest
  ⑨ 垂直归位           mv((tx,ty,H))
"""
from ._base import BaseMetaAction, mv, grip, shake
from .constants import (H, GRIP_OPEN, GRIP_TUBE,
                        TUBE_XY, TUBE_GRASP_TCP,
                        SHAKE_CENTER_TCP, SHAKE_AMPLITUDE, SHAKE_PERIOD,
                        SHAKE_HOLD_FRAMES)


class TubeShakePass(BaseMetaAction):
    """拿起试管 → 高位震荡来回 cycles 下 → 放回试管架（液柱随管平移）。"""

    def __init__(self, engine, cycles=3):
        self.cycles = max(1, int(cycles))
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, (tx, ty, H)),                   # ① 高位接近试管上方
            mv(e, TUBE_GRASP_TCP),                # ② 垂直下探抓管身中段（管口下 14mm）
            grip(e, GRIP_TUBE, 60),               # ③ 合爪夹紧试管（开度=管径/2）
            mv(e, SHAKE_CENTER_TCP, 5),           # ④ 垂直提出到震荡高度（管底清架顶）
            shake(e, SHAKE_CENTER_TCP,            # ⑤ 高位震荡来回 N 下（正弦往复）
                  amplitude=SHAKE_AMPLITUDE,
                  cycles=self.cycles, period=SHAKE_PERIOD),
            mv(e, SHAKE_CENTER_TCP, SHAKE_HOLD_FRAMES),   # ⑥ 震荡后停留 5 秒（现象持续）
            mv(e, TUBE_GRASP_TCP),                # ⑦ 垂直下探回架（管底回插孔）
            grip(e, GRIP_OPEN, 25),               # ⑧ 松开释放（task: tube released → rest）
            mv(e, (tx, ty, H)),                   # ⑨ 垂直归位
        ]
