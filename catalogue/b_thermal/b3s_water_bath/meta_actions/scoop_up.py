"""ScoopUpAction：只动最后一个关节（panda_joint7，索引 6 = 法兰自转），再转 -45°（法兰 -45°→-90°）挖起粉末。

用户 2026-08-24（逐字）：「现在已经深入粉丘了，所以要挖起来，只增加动作法兰从-45旋转到-90」。

- 沿用 ⑤ FlangeRollAction 的纯关节命令机制：只命令最后一个关节（joint7 = 法兰自转），
  其他关节不动、不重解 IK（避免全臂 IK 在冻结 TCP 下重解的退化情形，wrist_flip 注释）。
- 与 ⑤ 一样是相对旋转：⑤ 把法兰从 pick 姿态转 -45°，⑨ 再相对当前转 -45° → 法兰总 -90°
  （用户以"法兰状态"描述：-45° → -90°）。目标 = 首帧 cur[JOINT] + ANGLE。
- 方向：与 ⑤ 同向为负（用户 2026-08-14 已确认 +90° 转反，方向沿用）。

几何（pxr 行向量验证，法兰绕 tool+Z=世界+X 自转，药匙长轴=勺头方向=tool+X）：
  -45°：勺尖 = TCP + 0.134·(0,-0.707,-0.707)（勺头低，⑧ 后勺尖 (0.537,0.106,0.810) 沉粉丘内）
  -90°：勺尖 = TCP + 0.134·(0,-1,0)（勺头水平、完全放平）
  ⑨ 过程 TCP 不动（纯关节命令），勺尖从粉丘内扫到水平——勺尖随旋转**上升 9.5cm**
  （z 0.810→0.905，高出粉顶 0.8141），把粉从粉丘里"挖起来"、凹槽朝上蓄粉；勺尖 z 全程
  ≥0.810 > 皿沿 0.8066，不穿皿。powder_on_spoon 已在⑧触发，效果每帧跟随勺尖。

夹爪通道每帧发 grip_target（保持药匙夹住）。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

# 最后一个关节 = panda_joint7（flange roll，索引 6）。panda URDF 限位 ±2.8973 rad（≈±166°）。
JOINT = 6
LIMIT = 2.8973
ANGLE = -np.pi / 4       # 再转 -45°（法兰 -45°→-90°；与 ⑤ 同量同向，用户 2026-08-24）
RATE = 0.015            # 每帧推进（rad）≈0.9 rad/s，45°≈52帧≈0.87s，从容可辨
EPS = 0.005             # 到位判定
MAX_FRAMES = 240        # 兜底


class ScoopUpAction:
    """只动最后一个关节再转 -45°（法兰 -45°→-90°），挖起粉丘粉末，到位后 hold dwell 帧完成。"""

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
            print(f"[scoop] joint7 {cur[JOINT]:.4f} -> {self._target:.4f} rad "
                  f"({np.degrees(cur[JOINT]):.1f}° -> {np.degrees(self._target):.1f}°), "
                  f"flange -45°→-90° 挖粉")

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
