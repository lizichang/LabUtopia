"""LowerPowder：法兰转完 + 水平对齐粉堆后，竖直向下降 24.5cm（锁 x/y + 保持世界朝向，只变 z）。

用户 2026-08-22（逐字）：「现在增加动作，整个机械臂的夹爪部分竖直向下移动28cm，yx还有朝向都要
始终不变，所以只有z变」；随后「把28改成20」「改成22」定 0.22；2026-08-23 用户：「把倒数第二步的
下降改为下降20cm」→ DROP_DOWN = 0.20；2026-08-24 用户「倒数第二部的 z方向下降再下降3厘米」→ 0.23、
再「再下降2cm」但 25cm 勺尖 z=0.805 < 皿沿 0.8066 会平移穿模 → 用户改选 0.245。

- 朝向：首帧 fk_pose 采样当前**实际**工具朝向（= AlignPowderX 结束时的法兰 -45° 朝向，药匙仍
  45° 倾斜夹着）→ 转引擎 [w,x,y,z]（scalar-first）→ 整段保持，即"世界里绝对朝向不变"。
  与 AlignPowderX 同：不硬编码朝向常数（ORIENT_SCOOP 坑）。
- 位置：x/y 锁首帧当前值（x=0.537 已对齐粉堆中心、y=0.3608 法兰转完高位），仅 z 下降
  DROP_DOWN(0.245m)。MoveAction 自动判垂直段（x/y 锁目标值、z 逐帧推进 VZ_STEP、TCP 走严格
  竖线，v46 机制，与 flametest 下探/提出同）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结（位置 <1cm 连续 3 帧 + 朝向收敛）+ dwell + 夹爪
  每帧发 grip_target。

完成后：TCP (0.537, 0.3608, 0.905)，勺尖 (0.537, 0.266, 0.810)——勺尖 z=0.810 低于粉顶 0.8141
（沉入粉丘 ~4mm，触发舀粉效果 powder_on_spoon）、高于皿沿 0.8066（差 3.4mm 不碰皿沿），y=0.266
在表面皿 +Y 外沿外，⑦ 垂直下降不碰皿；⑧ 水平平移段勺尖 z=0.810 全程高于皿沿 0.8066，不穿模。
（勺尖位移按法兰 -45° 折算：toolX=(0,-0.707,-0.707)，0.134·0.707≈0.095。）
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import DROP_DOWN


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class LowerPowder:
    """保持当前世界朝向，竖直下降 DROP_DOWN 米，x/y 锁当前值。"""

    def __init__(self, engine, drop=DROP_DOWN, dwell=20):
        self.engine = engine
        self.drop = float(drop)
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
            pos = np.array([gp[0], gp[1], gp[2] - self.drop])   # x/y 锁当前值，z 降 20cm
            print(f"[lower] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
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
