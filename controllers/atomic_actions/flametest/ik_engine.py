"""焰色反应共享 IK 运动引擎：Lula IK 求解 + FK 验证 + 关节钳制。

从原 flametest_controller._execute_seg 提取，供本子包的小动作类共用。
每个小动作持有一份引擎引用（同一求解器 / 同一 base pose）。

朝向控制（D2-S 转水平/倾倒用）：solve_verified 增加可选 orient 参数，
四元数约定 = (w,x,y,z) scalar-first（与 euler_angles_to_quat / Lula
quats_to_rot_matrices 一致）。orient 为 None 时保持原行为（固定 self.orient
= 手指朝下，仅位置 FK 验证）；显式传 orient 时额外做 FK 朝向验证
（旋转矩阵夹角 < ORIENT_EPS 才接受），避免 Lula 解出"位置到位但朝向错"
的坏分支。
"""
import numpy as np
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices

# 朝向验证阈值：FK 旋转矩阵与目标旋转矩阵的夹角（rad）上限。
# Lula CCD 默认 orientation_tolerance ~0.1 rad，这里留裕量取 0.15。
ORIENT_EPS = 0.15


def quat_to_rot(quat):
    """w,x,y,z 四元数 -> 3x3 旋转矩阵。"""
    return np.asarray(quats_to_rot_matrices(np.asarray(quat, dtype=float)), dtype=float)


def rot_angle(a, b):
    """两个 3x3 旋转矩阵之间的夹角（rad）。"""
    m = np.asarray(a, dtype=float).T @ np.asarray(b, dtype=float)
    c = (float(np.trace(m)) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


class IkMotionEngine:
    """Lula IK 运动引擎（焰色反应场景专用，D2-S 复用）。

    提供：
      - solve_verified(target, cur7, orient=None)：解 IK 并 Lula FK 验证
        （FK 距目标 <6mm、显式朝向时夹角 <ORIENT_EPS 才接受），
        cur7 → home 双 warm start（段间连续；近奇异区避免坏分支/分支翻转）。
      - fk_pose(joints)：FK 得到 TCP 位置 + 3x3 旋转矩阵（冻结/收敛判定用）。
      - MAX_JOINT_DELTA：每帧关节最大变化量（0.48 rad/s @60Hz，动作从容可辨）。
    """

    # 每帧关节最大变化量（rad）。原 0.015 ≈ 0.9 rad/s：2026-08-14 用户反馈移动时滴管/试管
    # 惯性倾斜、与夹爪穿模（焰色反应同样存在）——移动太快 → 末端惯量让手指相对竖立的被握
    # 物体微微旋转歪斜（物体本身纯平移保竖立），快速移动时显形。减半到 0.008 ≈ 0.48 rad/s，
    # 臂移动更慢更稳，惯性/穿模消失。垂直段 VZ_STEP=0.002 的每帧关节量远小于此，不受影响。
    MAX_JOINT_DELTA = 0.008

    def __init__(self, solver, orient, ik_home):
        self.solver = solver
        self.orient = np.asarray(orient, dtype=float)
        self.orient_rot = quat_to_rot(self.orient)
        self.ik_home = np.asarray(ik_home, dtype=float)

    def fk_pose(self, joints):
        """FK：返回 (TCP 位置, 3x3 旋转矩阵)。"""
        p, r = self.solver.compute_forward_kinematics("right_gripper", joints)
        return np.asarray(p, dtype=float), np.asarray(r, dtype=float)

    def solve_verified(self, target, cur7, orient=None):
        """解 IK 并 FK 验证：TCP 真的到达目标（位置+可选朝向）才接受解。

        近奇异抓点（match/cap/stopper 低 z）用固定 home 作回退 warm start 时，
        Lula 偶发选出"FK 位置摆到目标 17cm 外"的坏分支（v34b 注释），臂朝错误
        方向猛甩后 force-done。这里依次尝试当前关节（连续性→段间平滑、消除分支
        跳变）与固定 home，并拒绝 FK 距目标 >6mm 的解。显式传 orient 时再拒绝
        朝向夹角 >ORIENT_EPS 的解（D2-S 转水平/倾倒用）。
        """
        if orient is None:
            orient = self.orient
            check_orient = False
        else:
            orient = np.asarray(orient, dtype=float)
            check_orient = True
        rot_t = quat_to_rot(orient)
        for ws in (np.asarray(cur7, dtype=float), self.ik_home):
            try:
                ik, ok = self.solver.compute_inverse_kinematics(
                    frame_name="right_gripper",
                    target_position=target,
                    target_orientation=orient,
                    warm_start=ws,
                )
            except Exception:
                continue
            if not ok or ik is None:
                continue
            ik = np.asarray(ik, dtype=float)
            fk_pos, fk_rot = self.solver.compute_forward_kinematics("right_gripper", ik)
            fk_pos = np.asarray(fk_pos, dtype=float)
            err = float(np.linalg.norm(fk_pos - target))
            if check_orient and rot_angle(fk_rot, rot_t) > ORIENT_EPS:
                continue
            if err < 0.006:
                return ik
        return None
