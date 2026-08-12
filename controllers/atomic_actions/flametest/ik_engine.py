"""焰色反应共享 IK 运动引擎：Lula IK 求解 + FK 验证 + 关节钳制。

从原 flametest_controller._execute_seg 提取，供本子包的小动作类共用。
每个小动作持有一份引擎引用（同一求解器 / 同一 base pose）。
"""
import numpy as np


class IkMotionEngine:
    """Lula IK 运动引擎（焰色反应场景专用）。

    提供：
      - solve_verified(target, cur7)：解 IK 并 Lula FK 验证（FK 距目标 <6mm 才接受），
        cur7 → home 双 warm start（段间连续；近奇异区避免坏分支/分支翻转）。
      - MAX_JOINT_DELTA：每帧关节最大变化量（0.9 rad/s @60Hz，动作从容可辨）。
    """

    MAX_JOINT_DELTA = 0.015

    def __init__(self, solver, orient, ik_home):
        self.solver = solver
        self.orient = orient
        self.ik_home = np.asarray(ik_home, dtype=float)

    def solve_verified(self, target, cur7):
        """解 IK 并 FK 验证：TCP 真的到达目标才接受解。

        近奇异抓点（match/cap/stopper 低 z）用固定 home 作回退 warm start 时，
        Lula 偶发选出"FK 位置摆到目标 17cm 外"的坏分支（v34b 注释），臂朝错误
        方向猛甩后 force-done。这里依次尝试当前关节（连续性→段间平滑、消除分支
        跳变）与固定 home，并拒绝 FK 距目标 >6mm 的解。
        """
        for ws in (np.asarray(cur7, dtype=float), self.ik_home):
            try:
                ik, ok = self.solver.compute_inverse_kinematics(
                    frame_name="right_gripper",
                    target_position=target,
                    target_orientation=self.orient,
                    warm_start=ws,
                )
            except Exception:
                continue
            if not ok or ik is None:
                continue
            ik = np.asarray(ik, dtype=float)
            fk_pos, _ = self.solver.compute_forward_kinematics("right_gripper", ik)
            err = float(np.linalg.norm(np.asarray(fk_pos, dtype=float) - target))
            if err < 0.006:
                return ik
        return None
