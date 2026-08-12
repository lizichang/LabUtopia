"""MoveAction：IK 移动到目标点，到达后冻结、按 dwell 停留。

approach / descend / translate / lift_vertical / dip_hold 都是"移到某点
（可带停留）"的参数化实例：
  - approach(pt)      = mv((pt.xy, H))           高位接近
  - descend(pt)       = mv(pt)                   垂直下探到抓/放点
  - lift_vertical     = mv((pt.xy, z), dwell)    同 xy 垂直提出 + 强制停顿
  - translate(pt)     = mv(pt)                   平移
  - dip_hold(pt, n)   = mv(pt, n)                下探到 pt 停留 n 帧

冻结判定（v43）：TCP 与目标 3D 距离 <1cm 连续 3 帧 → 冻结关节保持位置，等 dwell
帧。近奇异抓点处同一 TCP 可由多个 IK 分支到达，冻结即稳住抓点，让任务侧 attach
判定通过；不冻结会出现"dist 瞬时掠过即完成→臂还在摆→attach 拿不到连续近窗"。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


class MoveAction:
    """移到目标位置，到达冻结后停留 dwell 帧。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction。
    夹爪每帧显式发送 grip_target（v41：杜绝 NaN→applied 替换不可靠）。
    """

    def __init__(self, engine, pos, dwell=0, label=""):
        self.engine = engine
        self.pos = np.asarray(pos, dtype=float)
        self.dwell = int(dwell)
        self.label = label
        self.reset()

    def reset(self):
        self._frame = 0
        self._arrived = 0
        self._hold = 0
        self._ik_target = None
        self._frozen = None
        self._done = False
        self._solved = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        if not self._solved:
            self._solved = True
            self._ik_target = self.engine.solve_verified(self.pos, cur)
            if self._ik_target is None:
                print(f"[flametest] IK FAIL target={np.round(self.pos, 3)} — hold position, will force-done")

        if self._frozen is not None:
            cmd = self._frozen
        elif self._ik_target is not None:
            delta = np.clip(self._ik_target - cur,
                           -self.engine.MAX_JOINT_DELTA, self.engine.MAX_JOINT_DELTA)
            cmd = cur + delta
        else:
            cmd = cur

        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()

        self._frame += 1
        if self._frozen is not None:
            # 已冻结：只累计 hold 帧（不因微小漂移重置）
            self._hold += 1
            if self._hold >= self.dwell:
                self._done = True
        else:
            dist3d = float(np.linalg.norm(np.asarray(gripper_pos, dtype=float) - self.pos))
            if dist3d < 0.010:
                self._arrived += 1
            else:
                self._arrived = 0
            if self._arrived >= 3:
                self._frozen = np.asarray(joints[:7], dtype=float).copy()
                print(f"[flametest] freeze at tgt={np.round(self.pos, 3)} "
                      f"gripper={np.round(gripper_pos, 3)} dist3d={dist3d:.4f}")
                self._hold = 0
        if not self._done and self._frame >= self.dwell + 600:
            print(f"[flametest] move force-done t={self._frame} (dwell={self.dwell}) "
                  f"target={np.round(self.pos, 3)}")
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
