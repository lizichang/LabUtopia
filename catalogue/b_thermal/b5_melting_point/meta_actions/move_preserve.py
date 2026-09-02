# -*- coding: utf-8 -*-
"""MovePreserveAction：移动到目标、保持当前朝向（首帧采样 fk_pose 实际朝向）。

B1 move_x_preserve.MovePreserveTubeAction 的 B5 通用化逐字同构（只改类名/日志标签）：
首帧 fk_pose 采样当前**实际**工具朝向 → scipy from_matrix.as_quat() 出 [x,y,z,w] →
重排为引擎约定 [w,x,y,z]（Lula/quats_to_rot_matrices 同读法）→ 作 MoveAction
target_orientation；位置只改与目标相差的轴（其余锁首帧当前值）→ MoveAction 自动判
单轴 linewalk。

用于 B5 夹端部拎起后的运移（2026-09-01 用户「机械臂就只是直上直下不要其他变化」）：
夹爪全程手指朝下不旋转，拎起/横移/下探都必须**钉死**此朝向（mv orient=None 会解出
任意朝向 → 夹爪在移动中旋转，违背用户「直上直下」）。采样实际朝向 R_off 自动抵消
（B1 FLANGE_HOLD_ORIENT 手推朝向被证伪同因，见 controllers/atomic_actions/flametest/ik_engine.py）。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction, AXIS_EPS


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 引擎约定四元数 [w,x,y,z]（scalar-first，Lula/quats_to_rot_matrices
    同读法）。scipy from_matrix.as_quat() 出 [x,y,z,w]，重排为 [w,x,y,z] 喂引擎。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class MovePreserveAction:
    """移动到目标、保持当前朝向：首帧采样当前朝向，只移与目标相差的轴，其余锁当前。"""

    def __init__(self, engine, pos, dwell=0):
        self.engine = engine
        self.pos = np.asarray(pos, dtype=float)
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._move = None
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        if self._move is None:
            cur = np.asarray(joints[:7], dtype=float)
            _, R = self.engine.fk_pose(cur)
            orient_q = _R_to_quat_wxyz(R)
            gp = np.asarray(gripper_pos, dtype=float)
            # 只移与目标相差的轴（>AXIS_EPS），其余锁首帧当前值 → MoveAction 判单轴 linewalk
            pos = np.where(np.abs(self.pos - gp) > AXIS_EPS, self.pos, gp)
            print(f"[b5mvt] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
                  f"{orient_q[2]:.4f},{orient_q[3]:.4f}] target=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})")
            self._move = MoveAction(self.engine, pos, dwell=self.dwell, orient=orient_q)
        cmd = self._move.forward(joints, gripper_pos, grip_target)
        self._frame += 1
        if self._move.is_done():
            self._done = True
        return cmd

    def is_done(self):
        return self._done
