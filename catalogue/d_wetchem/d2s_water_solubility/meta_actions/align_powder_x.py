"""AlignPowderX：法兰转完后保持当前世界朝向，水平移动到粉堆中心 x（0.537），y/z 锁当前值。

用户 2026-08-20（逐字）：「现在加动作，法兰旋转后机械臂移动到粉堆的x绝对位置（0.537），然后
机械臂的yz值都不要变，然后整个夹爪在视频里面（整个世界）里面的绝对朝向也不要变，就是药匙还是
要-60度这样子夹着」。

- 朝向：首帧 fk_pose 采样法兰转完后的**实际**工具朝向 → 转引擎约定的 [w,x,y,z]（scalar-first，
  Lula/quats_to_rot_matrices 同读法）→ 整段保持，这就是"整个世界里的绝对朝向不变"、药匙 -45°
  夹着。**不硬编码 ORIENT_SCOOP**——2026-08-17 该常数与实测不符、solve_verified 拒解、joint7
  翻 +83° 药匙面滚歪的坑（自采样与实际姿态必然一致）。
- 位置：x 锁 0.537（粉堆中心），y/z = 首帧当前值（法兰转完 = pick④ 提起到的高位 y=0.3608、
  z=1.15，全程不动）。仅 x 变化 → MoveAction 自动判单轴 linewalk（x 逐帧推进、y/z 锁目标值、
  TCP 走严格直线，v47 机制）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结（位置 <1cm 连续 3 帧 + 朝向收敛）+ dwell + 夹爪
  每帧发 grip_target。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import POWDER_X


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class AlignPowderX:
    """保持当前世界朝向，水平移到粉堆中心 x，y/z 锁当前值。

    anchor_y（可选）：非 None 时把 y 显式锚定到该值（挖粉轨迹 y 基准），用于跨列药匙
    家用——D3-S 药匙移到第一列第3排后，y 基准仍是 d2s 原家用 y（0.3608），否则 ⑧
    ShiftYNeg 从 0.3209 起勺尖 0.0659 会错过粉丘。默认 None = 锁当前 y（d2s 原行为）。
    """

    def __init__(self, engine, x=POWDER_X, anchor_y=None, dwell=20):
        self.engine = engine
        self.x = float(x)
        self.anchor_y = None if anchor_y is None else float(anchor_y)
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
            if self.anchor_y is not None:
                pos = np.array([self.x, self.anchor_y, gp[2]])   # y 锚定挖粉基准（跨列家用）
            else:
                pos = np.array([self.x, gp[1], gp[2]])   # y/z 锁当前值
            print(f"[alignx] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
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
