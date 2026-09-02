"""FlangeRollTubeAction：只动最后一个关节（panda_joint7，索引 6 = 法兰自转）转 -95°。

用户 2026-08-27 先逐字：「试管提出之后法兰旋转-100度」→ 改 -100°；随后再批：「法兰改为旋转
-95度吧」→ 定稿 -95°。PickTubePass ⑤ 竖直提出架顶后，法兰转 -95°：试管由竖直转成近水平
（过水平 5°，文档「拿试管→外焰预热（倾斜 10-15°）」的倾斜姿态）。本动作只转姿态不移动
位置；火焰上方横移由 PickTubePass ⑦ 完成（爪子 x 对齐火焰 x，yz/朝向不变，见 constants
FLANGE_HOLD_ORIENT）。

2026-08-28 参数化：构造可传 angle（度）覆盖默认 -95°（如放回试管 ReturnTubePass ⑥' 法兰
转回竖直 angle=+95°，从当前 ≈-95° 回到 0）。

机制 = D2S flange_roll.py 逐字（用户 2026-08-14 已定「只动最后一个关节！」，别用腕关节
簇贪心）：纯关节命令、不重解 IK，避免全臂 IK 在冻结 TCP 下重解的退化情形。

几何（pxr 行向量验证，同 D2S flange_roll）：pick 以 ORIENT_FWD 水平横夹后试管竖挂
（长轴=世界 Z、管口朝上，管底吊夹爪下 TUBE_HELD_X=0.1393，见 task._T_HELD_TUBE）；
joint7 绕 tool+Z（=世界 +X，朝向 camera1）自转 → 试管长轴从世界 Z 向水平转。
-90° = 完全放平（水平）；-95° = 过水平 5°（管口方向 (0,+0.996,-0.087) 朝 +Y 略向下、
管底 (0,-0.996,+0.087) 朝 -Y 略向上）。旋转全程试管最低点 z ≥ 0.9607（清架顶 0.917），
不碰试管架/表面皿。法兰转后 tool+Z 仍 = 世界 +X（手指朝前不变），仅 tool+X/tool+Y 绕
+Z 转了 -95° → 朝向四元数 (0.521334,-0.477714,0.521334,-0.477714)（FLANGE_HOLD_ORIENT，
⑦ 横移保持此朝向，见 constants.py 推导）。

夹爪通道每帧发 grip_target（保持试管夹住，GRIP_TUBE）。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

# 最后一个关节 = panda_joint7（flange roll，索引 6）。panda URDF 限位 ±2.8973 rad（≈±166°）。
JOINT = 6
LIMIT = 2.8973
ANGLE = -95.0 * np.pi / 180.0    # -95°（用户 2026-08-27 逐字：法兰改为旋转-95度）
RATE = 0.015                    # 每帧推进（rad）≈0.9 rad/s，95°≈110帧≈1.8s，从容可辨
EPS = 0.005                     # 到位判定
MAX_FRAMES = 400                # 兜底


class FlangeRollTubeAction:
    """只动最后一个关节转 ANGLE（默认 -95°，可传 angle 度覆盖），到位后 hold dwell 帧完成。"""

    def __init__(self, dwell=15, angle=None):
        self.dwell = int(dwell)
        self._angle = ANGLE if angle is None else float(angle) * np.pi / 180.0
        self.reset()

    def reset(self):
        self._target = None
        self._hold = 0
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints, dtype=float)
        if self._target is None:
            self._target = float(np.clip(cur[JOINT] + self._angle, -LIMIT, LIMIT))

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
