"""ShiftYPos：⑩ 抬升后，往 +Y 平移 24cm（锁 x/z + 保持世界朝向，只变 y）。

用户 2026-08-24（逐字）：「现在新增动作，机械臂先往正y移动20cm（此时只有y变）」，
随后（同晚）：「那倒数第二移动y那一步再移动21cm」→ 改 21cm；「那再加20cm」→ 改 41cm；
「y改成移动32」→ 改 32cm。同日试管从 +Y 最远侧孔移到架最近侧孔 (0.659, 0.241) 后，
⑪ 回调到 17cm：勺尖对准管口 y；再「倒数第二步y再增加1cm」→ 改 18cm；
再「倒数第二步往正y方向再多移动6cm」→ 改 24cm；再「第11步移动y那一步，减少移动2cm」→ 改 22cm；
再「第十一步多移动2cm」→ 改 24cm（回到 24cm，⑬ 同步 -Y +2cm 净位移抵消；
2026-08-25 曾试 27cm 后用户回退，恢复 24cm）。
（紧接着 ⑫ 再往 +X 移 11cm；⑪⑫ 合起来把药匙从粉丘上方水平挪到试管口近前。）

- 朝向：首帧 fk_pose 采样当前**实际**工具朝向（= ⑩ 抬升后的法兰 -90° 朝向，药匙水平）
  → 转引擎 [w,x,y,z]（scalar-first）→ 整段保持，即"世界里绝对朝向不变"。与 ⑥⑦⑧ 同：
  不硬编码朝向常数（ORIENT_SCOOP 坑）。
- 位置：x/z 锁首帧当前值（x=0.537 已对齐粉堆、z=1.0993 ⑩ 抬升后 14cm），仅 y 增
  Y_SHIFT_POS (0.24m)。MoveAction 自动判单轴水平段（仅 y 变 → y 逐帧推进、x/z 锁目标值、
  TCP 走严格直线，v47 机制）。
- 复用 MoveAction 已验证逻辑：单轴直线 + 冻结（位置 <1cm 连续 3 帧 + 朝向收敛）+ dwell +
  夹爪每帧发 grip_target。

完成后：TCP (0.537, 0.4408, 1.0993)；法兰 -90° 水平时勺尖 = TCP + 0.134·(0,-1,0) =
(0.537, 0.3068, 1.0993)（管口中心 y=0.241 后 6.6cm；用户 2026-08-24「倒数第二步y再增加1cm」18cm，
再「倒数第二步再多移动6cm」→24cm，再「第11步移动y那一步，减少移动2cm」→22cm，再「多移动2cm」→24cm）。
x 仍 0.537（x 由⑫ +11cm 跟进到 0.647，管口 x=0.659，尚差 1.2cm，供后续倾倒段对齐）。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction

from .constants import Y_SHIFT_POS


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class ShiftYPos:
    """保持当前世界朝向，往 +Y 平移 Y_SHIFT_POS 米，x/z 锁当前值。"""

    def __init__(self, engine, shift=Y_SHIFT_POS, dwell=20):
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
            pos = np.array([gp[0], gp[1] + self.shift, gp[2]])   # x/z 锁当前值，y 增 24cm
            print(f"[shifty+] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
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
