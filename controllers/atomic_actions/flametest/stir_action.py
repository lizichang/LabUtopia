"""StirAction：绕 center 水平圆周搅拌（"搅拌 N 圈"，玻璃棒插烧杯后画圆）。

TCP 沿半径 radius 的水平圆周绕 center 转 cycles 圈，z 锁 center[2]。相位从 0
走 2π·cycles，每 period 帧一圈。被握棒纯平移跟随（棒底 = TCP − ROD_TIP_TO_GRIP，
由 task 逐帧更新），棒底在液体内画同一半径的圆 → 搅拌。逐帧 IK 重解（cur7
warm start，FK 验证）+ 关节钳制 MAX_JOINT_DELTA（ShakeAction 同款）。

不是 MoveAction 变体：目标是**连续移动**的，不能用"TCP 距目标 <1cm 冻结"判定
（会误冻）——总帧数到即 done（ShakeAction 同款）。圆周每帧弧长 = 2π·radius/
period：radius 15mm × period 45 → 2.1mm/帧，远小于 MAX_JOINT_DELTA 钳制量。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


class StirAction:
    """绕 center 水平圆周搅拌 cycles 圈后完成（z 锁 center[2]）。"""

    def __init__(self, engine, center, radius=0.015, cycles=3, period=45,
                 label="stir", orient=None):
        self.engine = engine
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)
        self.cycles = int(cycles)
        self.period = int(period)
        self.label = label
        # 可选朝向（w,x,y,z）：None 沿用引擎默认（手指朝下）；显式传时搅拌全程
        # 保持该朝向（棒横夹持握用 ORIENT_FWD，同 PickGlassRod）。
        self.orient = orient
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False
        self._total = int(self.cycles * self.period)

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        # 相位每 period 帧一圈（0 → 2π·cycles）；圆周起点角 0（+X），首帧目标
        # 距中心 radius，关节钳制平滑起步（ShakeAction 从中心起步同理靠钳制顺滑）。
        ang = 2.0 * np.pi * (self._frame / self.period)
        tgt = self.center + np.array([self.radius * np.cos(ang),
                                      self.radius * np.sin(ang), 0.0])
        ik = self.engine.solve_verified(tgt, cur, orient=self.orient)
        if ik is None:
            cmd = cur  # 解不出就保持，下一帧再试（圆周相位继续走）
        else:
            delta = np.clip(ik - cur, -self.engine.MAX_JOINT_DELTA,
                            self.engine.MAX_JOINT_DELTA)
            cmd = cur + delta
        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()
        self._frame += 1
        if self._frame >= self._total:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
