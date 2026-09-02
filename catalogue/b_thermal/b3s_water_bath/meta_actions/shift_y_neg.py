"""ShiftYNeg：第⑦步竖直降完后，夹爪往 -Y 平移 16cm（锁 x/z + 保持世界朝向，只变 y）。

用户 2026-08-22（逐字）：「那现在加动作机械臂夹爪往-y移动15cm，其他的xz还有朝向都要严格不变」；
2026-08-23 先后改为 18cm、23cm。

- 朝向：首帧 fk_pose 采样当前**实际**工具朝向（= 法兰 -45° 朝向，药匙 45° 倾斜夹着）→ 转引擎
  [w,x,y,z]（scalar-first）→ 整段保持，即"世界里绝对朝向不变"。与 AlignPowderX/LowerPowder 同：
  不硬编码朝向常数（ORIENT_SCOOP 坑）。
- 位置：x/z 锁首帧当前值（x=0.537 已对齐粉堆中心、z=0.905 第⑦步降完 1.15−0.245），仅 y 减
  Y_SHIFT_NEG (0.16m)。MoveAction 自动判水平段（仅 y 变 → y 逐帧推进、x/z 锁目标值、TCP 走
  严格直线，v47 机制）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结（位置 <1cm 连续 3 帧 + 朝向收敛）+ dwell + 夹爪
  每帧发 grip_target。

完成后：TCP (0.537, 0.2008, 0.905)，勺尖 (0.537, 0.1058, 0.810)——勺尖 z=0.810 低于粉顶 0.8141
（沉入粉丘 ~4mm，y=0.1058 在粉丘 bbox y[0.0814,0.1288] 内、z 在 z[0.8021,0.8141] 内 → **触发舀粉
效果 powder_on_spoon**）、高于皿沿 0.8066（差 3.4mm 不穿模）——⑧ 平移全程勺尖 z=0.810 高于皿沿，
不碰皿/不穿模。⑧ 终点 y=0.2008 相对底座 y=0.05 = 0.1508，已脱离 2026-08-23 实测失效边界 <0.10
（由 2026-08-24 皿+粉 +Y 6.5cm + 平移量改 16cm 达成，底座未动）。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import Y_SHIFT_NEG


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class ShiftYNeg:
    """保持当前世界朝向，往 -Y 平移 Y_SHIFT_NEG 米，x/z 锁当前值。"""

    def __init__(self, engine, shift=Y_SHIFT_NEG, dwell=20):
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
            pos = np.array([gp[0], gp[1] - self.shift, gp[2]])   # x/z 锁当前值，y 减 16cm
            print(f"[shifty] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
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
