"""LiftToTube：⑨ 挖粉后，竖直抬升到试管管口上方 14cm（锁 x/y + 保持世界朝向，只变 z）。

用户 2026-08-24（逐字）：「所以现在增加动作，将爪子抬升到试管管口高2cm（只能z变，其他不要变）」；
同日「把第10步的2cm改成10cm」→ 改 10cm；再「第十步抬升再抬升4cm」→ 改 14cm。

- 朝向：首帧 fk_pose 采样当前**实际**工具朝向（= ⑨ 挖粉结束的法兰 -90° 朝向，药匙水平）
  → 转引擎 [w,x,y,z]（scalar-first）→ 整段保持，即"世界里绝对朝向不变"。与 ⑥⑦⑧ 同：
  不硬编码朝向常数（ORIENT_SCOOP 坑）。
- 位置：x/y 锁首帧当前值（x=0.537 已对齐粉堆、y=0.2008 ⑧ 平移后），仅 z 升到
  LIFT_TUBE_Z = 管口顶 0.9593 + 14cm = 1.0993（绝对目标，与运行时当前 z 无关）。
  MoveAction 自动判垂直段（x/y 锁目标值、z 逐帧推进 VZ_STEP、TCP 走严格竖线，v46 机制，
  与 flametest 提出/下降同）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结（位置 <1cm 连续 3 帧 + 朝向收敛）+ dwell +
  夹爪每帧发 grip_target。

完成后：TCP (0.537, 0.2008, 1.0993)——比⑨结束抬升 19.43cm，高于管口顶 0.9593 共 14cm；
法兰 -90° 水平时勺尖 z = TCP z（勺尖 = TCP + 0.134·(0,-1,0)），也高于管口，为下一步水平移到
管口上方倾倒留出净空。x/y 未动，药匙仍停在粉丘正上方。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import LIFT_TUBE_Z


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class LiftToTube:
    """保持当前世界朝向，竖直抬升到管口上方 14cm，x/y 锁当前值。"""

    def __init__(self, engine, z=LIFT_TUBE_Z, dwell=20):
        self.engine = engine
        self.z = float(z)
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
            pos = np.array([gp[0], gp[1], self.z])   # x/y 锁当前值，z 升到管口上方 14cm
            print(f"[lifttube] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
                  f"{orient_q[2]:.4f},{orient_q[3]:.4f}] "
                  f"target=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})")
            self._move = MoveAction(self.engine, pos, dwell=self.dwell, orient=orient_q)
        cmd = self._move.forward(joints, gripper_pos, grip_target)
        self._frame += 1
        if self._move.is_done():
            self._done = True
        return cmd

    def is_done(self):
        return self._done
