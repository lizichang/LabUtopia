"""MoveAction：IK 移动到目标点，到达后冻结、按 dwell 停留。

approach / descend / translate / lift_vertical / dip_hold 都是"移到某点
（可带停留）"的参数化实例：
  - approach(pt)      = mv((pt.xy, H))           高位接近
  - descend(pt)       = mv(pt)                   垂直下探到抓/放点
  - lift_vertical     = mv((pt.xy, z), dwell)    同 xy 垂直提出 + 强制停顿
  - translate(pt)     = mv(pt)                   平移
  - dip_hold(pt, n)   = mv(pt, n)                下探到 pt 停留 n 帧

垂直约束（v46）：原实现"解一次 IK + 每帧关节钳制"，joint 空间插值会把 TCP 拉成
弧线——滴管/瓶塞/灯帽下探和提出时就斜着走、带抖动（用户报"斜着拿/穿模"）。
若起点 xy 与目标 xy 几乎重合（<1.5cm），判定为垂直段：xy 锁死目标值、z 每帧
推进 VZ_STEP，逐帧沿垂直线上重新解 IK（cur7 warm start，FK 验证 <6mm，TCP 收敛
在这条线上），TCP 走严格直线，无斜拉、无弧线抖动。

冻结判定（v43）：TCP 与目标 3D 距离 <1cm 连续 3 帧 → 冻结关节保持位置，等 dwell
帧。近奇异抓点处同一 TCP 可由多个 IK 分支到达，冻结即稳住抓点，让任务侧 attach
判定通过；不冻结会出现"dist 瞬时掠过即完成→臂还在摆→attach 拿不到连续近窗"。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

from .ik_engine import ORIENT_EPS, quat_to_rot, rot_angle

# 垂直段判定：起点 xy 与目标 xy 偏差小于此值视为"原地升/降"
# （approach 冻结保证起止 xy 偏差 <1cm，这里留裕量）
VERTICAL_XY_EPS = 0.015
# 垂直段每帧 z 推进量（m/帧）。v46 初版 0.008 实测：z 步大 → 所需关节重配超
# MAX_JOINT_DELTA(0.015)，钳制后 TCP 横向拖滞 4~9cm（diag 轨迹验证），改小到
# 钳制内（2mm 步长 @60Hz ≈ 0.12 m/s，TCP 贴线无拖滞，下探/提出略慢但稳）
VZ_STEP = 0.002


class MoveAction:
    """移到目标位置，到达冻结后停留 dwell 帧。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction。
    夹爪每帧显式发送 grip_target（v41：杜绝 NaN→applied 替换不可靠）。
    """

    def __init__(self, engine, pos, dwell=0, label="", orient=None):
        self.engine = engine
        self.pos = np.asarray(pos, dtype=float)
        self.dwell = int(dwell)
        self.label = label
        # 可选朝向（w,x,y,z）：None 沿用引擎默认（手指朝下）；显式传时解 IK
        # 目标朝向 + 冻结需朝向收敛（原地转水平时位置已到、朝向仍在解）
        self.orient = orient
        self._rot_target = None if orient is None else quat_to_rot(orient)
        self.reset()

    def reset(self):
        self._frame = 0
        self._arrived = 0
        self._hold = 0
        self._ik_target = None
        self._frozen = None
        self._done = False
        self._solved = False
        self._vertical = False
        self._goal_z = None

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        if not self._solved:
            self._solved = True
            gp = np.asarray(gripper_pos, dtype=float)
            self._vertical = float(np.linalg.norm(gp[:2] - self.pos[:2])) < VERTICAL_XY_EPS
            if self._vertical:
                # 垂直段：xy 锁死目标值，z 从当前开始逐帧推进（v46）
                self._goal_z = gp[2]
                self._ik_target = None
            else:
                self._ik_target = self.engine.solve_verified(self.pos, cur, self.orient)
                if self._ik_target is None:
                    print(f"[flametest] IK FAIL target={np.round(self.pos, 3)} — hold position, will force-done")

        if self._frozen is not None:
            cmd = self._frozen
        elif self._vertical and self._goal_z is not None:
            # 垂直段：每帧沿 z 线推进并重新解 IK（cur7 warm start，TCP 走直线）
            dz = self.pos[2] - self._goal_z
            step = VZ_STEP if dz > 0 else -VZ_STEP
            if abs(dz) < VZ_STEP:
                step = dz
            self._goal_z += step
            tgt = np.array([self.pos[0], self.pos[1], self._goal_z])
            ik = self.engine.solve_verified(tgt, cur, self.orient)
            if ik is None:
                cmd = cur  # 解不出就保持，下一帧再试
            else:
                delta = np.clip(ik - cur,
                                -self.engine.MAX_JOINT_DELTA, self.engine.MAX_JOINT_DELTA)
                cmd = cur + delta
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
            # 朝向收敛（仅显式传 orient 的段）：位置到位 + 朝向对齐才冻结。
            # 原地转水平/倾倒时位置已到目标、朝向仍在 IK 收敛中，不能按位置即冻。
            orient_ok = True
            if self._rot_target is not None:
                _, fk_rot = self.engine.fk_pose(cur)
                orient_ok = rot_angle(fk_rot, self._rot_target) < ORIENT_EPS
            if dist3d < 0.010 and orient_ok:
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
