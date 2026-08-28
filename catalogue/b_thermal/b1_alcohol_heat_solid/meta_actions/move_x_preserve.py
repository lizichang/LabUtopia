# -*- coding: utf-8 -*-
"""MovePreserveTubeAction：单轴移动试管（保持倾斜姿态），其余轴锁当前值。

仿 d2s「移动药匙」三件套（AlignPowderX / LiftToTube / LowerPowder）同构：首帧 fk_pose 采样当前
**实际**工具朝向 → scipy from_matrix.as_quat() 出 [x,y,z,w] → 重排为引擎约定 [w,x,y,z]
（Lula/quats_to_rot_matrices 同读法）→ 作 MoveAction target_orientation；位置只改与目标相差的轴
（其余锁首帧当前值）→ MoveAction 自动判单轴 linewalk（该轴逐帧推进、另两轴锁目标、TCP 走严格
直线、每帧重解 IK 保持朝向），joint7 全程 ≈-95° 不会转回竖直。

 ⑦ 水平 -X（用户 2026-08-27「现在加动作水平往负x方向移动（yz还有朝向都不变）让爪子x坐标对齐
    火焰的x坐标」）：target=TUBE_AT_FLAME，仅 x 变 → 其余 y/z 锁当前。
 ⑧ 竖直下降（用户 2026-08-28「再加动作，竖直z方向降低，让爪子在z坐标比火焰小2cm（过程中yx还有
    朝向不变，只有z变）」）：target=(x,y,FLAME_Z−0.02)，仅 z 变 → 其余 x/y 锁当前。

初版 mv(TUBE_AT_FLAME, orient=FLANGE_HOLD_ORIENT)（手工推导朝向常量）被用户证伪（「你为什么最后
又加了一个旋转法兰的动作？最后把试管又旋转到正的角度去了」）——FLANGE_HOLD_ORIENT 在 tool_center
系推导、未计与 Lula "right_gripper" 帧的固定偏移 R_off，喂给 Lula 时工具朝向被转偏 → 试管被甩回
竖直。改用采样实际朝向（d2s-wrist-flip 坑同款修复），R_off 自动抵消。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from controllers.atomic_actions.flametest.move_action import MoveAction, AXIS_EPS


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 引擎约定四元数 [w,x,y,z]（scalar-first，Lula/quats_to_rot_matrices 同读法）。

    与 d2s AlignPowderX._R_to_quat_wxyz 逐字同构：scipy from_matrix.as_quat() 出 [x,y,z,w]，
    重排为 [w,x,y,z] 喂引擎。
    """
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class MovePreserveTubeAction:
    """单轴移动试管到目标：首帧采样当前朝向，只移与目标相差的轴，其余轴锁当前，保持倾斜。"""

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
            print(f"[mvtube] sampled orient=[{orient_q[0]:.4f},{orient_q[1]:.4f},"
                  f"{orient_q[2]:.4f},{orient_q[3]:.4f}] target=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})")
            self._move = MoveAction(self.engine, pos, dwell=self.dwell, orient=orient_q)
        cmd = self._move.forward(joints, gripper_pos, grip_target)
        self._frame += 1
        if self._move.is_done():
            self._done = True
        return cmd

    def is_done(self):
        return self._done
