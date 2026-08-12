"""HoldAction：停在当前位置冻结 dwell 帧（settle / hold 原语）。

臂通道发当前关节值（保持不动，避免 NaN 依赖 applied 替换），夹爪通道跟随
grip_target（v41：每帧显式发送，杜绝替换不可靠）。dwell 帧后完成。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


class HoldAction:
    """原地保持当前关节，冻结 dwell 帧。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction。
    """

    def __init__(self, engine, dwell):
        self.engine = engine
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        target = np.full(joints.shape[0], np.nan)
        target[:7] = np.asarray(joints[:7], dtype=float)
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()
        self._frame += 1
        if self._frame >= self.dwell:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
