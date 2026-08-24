"""ShiftXPos：⑪ 往 +Y 18cm 后，再往 +X 平移 10cm（锁 y/z + 保持世界朝向，只变 x）。

用户 2026-08-24（逐字）：「然后再往+x移动5cm（只有x变）」→「最后一步还需要再往前伸到试管口」
（5cm 改 12cm）→「最后一步减少2厘米深得有点太靠前了」（12cm 改 10cm）。
（⑪ 已回调 17cm 让勺尖对准管口 y；⑫ 让勺尖伸到管口前——⑪⑫ 合起来把药匙从粉丘上方
水平挪到试管口近前。）

- 朝向：首帧 fk_pose 采样当前**实际**工具朝向（= ⑪ 结束的法兰 -90° 朝向，药匙水平）
  → 转引擎 [w,x,y,z]（scalar-first）→ 整段保持，即"世界里绝对朝向不变"。与 ⑥⑦⑧ 同。
- 位置：y/z 锁首帧当前值（y=0.3808 ⑪ 后、z=1.0593 ⑩ 后），仅 x 增 X_SHIFT_POS (0.10m)。
  MoveAction 自动判单轴水平段（仅 x 变 → x 逐帧推进、y/z 锁目标值、TCP 走严格直线，v47 机制）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结 + dwell + 夹爪每帧发 grip_target。

完成后：TCP (0.637, 0.3808, 1.0593)；勺尖 = TCP + 0.134·(0,-1,0) = (0.637, 0.2368, 1.0593)
（勺尖在管口中心 0.659 前 2.2cm，用户 2026-08-24 判定 12cm 时太靠前、减 2cm）。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import X_SHIFT_POS


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class ShiftXPos:
    """保持当前世界朝向，往 +X 平移 X_SHIFT_POS 米，y/z 锁当前值。"""

    def __init__(self, engine, shift=X_SHIFT_POS, dwell=20):
        self.engine = engine
        self.shift = float(shift)
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
            pos = np.array([gp[0] + self.shift, gp[1], gp[2]])   # y/z 锁当前值，x 增 10cm
            print(f"[shiftx+] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
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
