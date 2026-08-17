"""FlangeRollAction：只动最后一个关节（panda_joint7，索引 6 = 法兰自转），转到当前+90°。

用户 2026-08-14 晚：「现在再加上第二个微动作 机械臂的夹爪的那个最后一个关节 旋转90度，
只加这一个变化」；我最初错用 WristFlipAction（腕关节簇 3/4/5 贪心），用户纠正「写的不对，
只动最后一个关节！」——就是**最后一个关节单独转 90°**。

几何（pxr 行向量验证）：pick 以 ORIENT_FWD 夹起后药匙竖挂（长轴=世界 Z、勺头朝下）；
joint7 绕 tool+Z（=世界 +X，朝向 camera1）自转 -90° → 药匙长轴转到世界 -Y、由竖直放平
（水平）。方向按用户 2026-08-14 晚确认：+90° 转反了，改 -90°。纯关节命令、不重解 IK，
避免全臂 IK 在冻结 TCP 下重解的退化情形（wrist_flip 注释）。

夹爪通道每帧发 grip_target（保持药匙夹住）。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

# 最后一个关节 = panda_joint7（flange roll，索引 6）。panda URDF 限位 ±2.8973 rad（≈±166°）。
JOINT = 6
LIMIT = 2.8973
ANGLE = -np.pi / 2      # -90°（用户 2026-08-14 晚确认：+90° 方向反了，改 -90°）
RATE = 0.015            # 每帧推进（rad）≈0.9 rad/s，90°≈105帧≈1.75s，从容可辨
EPS = 0.005             # 到位判定
MAX_FRAMES = 240        # 兜底


class FlangeRollAction:
    """只动最后一个关节转 +90°，到位后 hold dwell 帧完成。"""

    def __init__(self, dwell=15):
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._target = None
        self._hold = 0
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints, dtype=float)
        if self._target is None:
            self._target = float(np.clip(cur[JOINT] + ANGLE, -LIMIT, LIMIT))

        cmd = cur[:7].copy()
        diff = self._target - cur[JOINT]
        if abs(diff) <= EPS:
            cmd[JOINT] = self._target
            self._hold += 1
        else:
            cmd[JOINT] = cur[JOINT] + float(np.clip(diff, -RATE, RATE))

        target = np.full(cur.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()

        self._frame += 1
        if self._hold >= self.dwell or self._frame >= MAX_FRAMES:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
