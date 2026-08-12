"""GripAction：原地合爪 / 开爪（不驱动臂）。

close(grip, n) / open(n) 的参数化实例。臂通道 NaN（pos=None 语义，关节不动，
杜绝旧 bug1：合爪段用"移动"语义驱动 IK 在近奇异低 z 点追位置），只把夹爪
通道发到 width。等 max(dwell, 25) 帧让夹爪实际走完（夹爪物理运动需要时间）。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


class GripAction:
    """原地把夹爪开到/闭到 width，dwell 帧后完成。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction。
    joints[7]/joints[8] 为夹爪关节（单位米，action 需除以 stage_units）。
    """

    MIN_FRAMES = 25   # 夹爪走完 + 触须判定留出的最少帧数

    def __init__(self, engine, width, dwell=25):
        self.engine = engine
        self.width = float(width)
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        target = np.full(joints.shape[0], np.nan)
        target[7] = self.width / get_stage_units()
        target[8] = self.width / get_stage_units()
        self._frame += 1
        if self._frame >= max(self.dwell, self.MIN_FRAMES):
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
