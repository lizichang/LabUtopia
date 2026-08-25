"""⑬ FlangeRollShiftYNeg：边旋转法兰回卷（-90°→0°）边只往 -Y 平移 14cm，同始同终。

用户 2026-08-24（逐字）：「那现在在最后新增这个动作，机械臂边往-y移动（总共移动5cm），
边旋转法兰到-45度（当前是-90），这两个同步进行（同时开始同时结束）」；
同日「最后一步旋转45改成60」→ 回卷量 45°→60°；「改为法兰旋转到0，然后9cm变成13cm」
→ 回卷到 0°（+90°）、-Y 9→13cm；「最后一步向-y移动再增加2cm」→ 13→15cm；
再「第十三步也再多往回2cm正好抵消」→ 15→17cm（⑪ +2cm 起点随移、⑬ -2cm 抵消，净位移不变；
2026-08-25 曾试 ⑪ +3cm→27cm 后用户回退，恢复此终点；同日「现在只调整最后一步，17cm减少到14cm」→ 14cm，
⑬ 不再抵消）。

- 合成机制：一个进度 t = frame/N（N = 法兰转满 +90° 所需帧数，ROLL_RATE 步长）
  同时驱动 ① 法兰回卷（**直接命令 joint7** 每帧 +ROLL_RATE，⑤ FlangeRollAction / ⑨
  ScoopUpAction 同款纯关节命令——保证法兰一定转、不依赖 IK）② TCP 往 -Y 平移 shift·t
  （x/z 锁起点）。两者由同一个 t 推进 → 同时开始、同时结束。
- -Y 平移朝向：每帧从「把本帧法兰命令角写进关节集再 fk_pose」采样朝向（已含当前回卷角度，
  **非**滞后的实际角——实际角滞后会令其余关节逐帧漂移、末端不垂直）→ 转引擎 [w,x,y,z]
  → solve_verified 带 orient（ShiftYNeg 同款"保持当前世界朝向"语义，走严格直线，无斜拉）。
  warm start 用**上次命令关节**而非实际关节（MoveAction v47 教训：实际关节受 PD+钳制滞后，
  喂实际关节在近奇异区逐帧落入不同局部最小、永不到位）。
- 钳制：joint7（法兰）放宽到 ROLL_RATE=0.015 rad/帧（与 ⑤⑨ 同速）；其余关节按
  MAX_JOINT_DELTA=0.006（-Y 平移每帧 0.94mm，所需关节量远小于此，不拖滞）。
- 完成：-Y 名义到点（t=1）即冻结**实际**关节 hold dwell 帧 → 可见"卡住"。（2026-08-25 去掉
  强制竖直：不再等实际法兰收敛到 0°，残余 1-3° 倾斜就随它倾斜——用户「转到0度有一点倾斜就倾斜吧」）。
  不强钉角（ramp 自然钉到目标、命令连续无跳变=不闪现）、不冻结命令关节（避免 dwell 期追赶=闪现；
  2026-08-24「不要最后强制竖直，直接闪现了最后一步」根因即这两处）。

完成后：TCP (0.647, 0.3008, 1.0993)、法兰名义 0°（药匙竖直、勺头朝下）；勺尖 = TCP+0.134·(0,0,-1)
= (0.647, 0.3008, 0.9653)（勺尖 z 在管口顶 0.9593 上 6mm、y 在管口中心 0.241 后 6cm、x 在管口中心 0.659 前 1.2cm，为后续倾倒/倒粉做准备）。
（2026-08-25 去掉强制竖直后，实际法兰可能残留 1-3° 倾斜——勺尖略微偏向管口，用户接受；药粉
下落触发 `_vertical_over_mouth` 的法兰阈值 0.10 rad≈5.7° 仍可覆盖。）
夹爪通道每帧发 grip_target（保持药匙夹住）。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

from .constants import Y_SHIFT_NEG_LAST
from .flange_roll import RATE as FLANGE_RATE   # 法兰每帧转速 0.015 rad ≈0.9 rad/s（与 ⑤⑨ 同速）

# 最后一个关节 = panda_joint7（flange roll，索引 6），URDF 限位 ±2.8973 rad。
JOINT = 6
LIMIT = 2.8973
ROLL_BACK = +np.pi / 2       # 法兰 -90°→0°（+90° 回卷，与 ⑤⑨ 反向；2026-08-24「改为法兰旋转到0」）
ROLL_RATE = FLANGE_RATE      # 每帧推进 rad（与 ⑤⑨ 同速）
MAX_FRAMES = 240             # 兜底


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class FlangeRollShiftYNeg:
    """法兰 -90°→0°（回卷 90°）同时 TCP 只往 -Y 平移 14cm，同始同终，到位后冻结 hold dwell 帧完成。"""

    def __init__(self, engine, shift=Y_SHIFT_NEG_LAST, dwell=15):
        self.engine = engine
        self.shift = float(shift)
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._roll_target = None
        self._pos0 = None
        self._N = None
        self._last_cmd = None
        self._frozen = None
        self._hold = 0
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        if self._N is None:
            # 首帧：采样回卷目标（当前 joint7 + 90°）、起点 TCP、总帧数
            self._roll_target = float(np.clip(cur[JOINT] + ROLL_BACK, -LIMIT, LIMIT))
            self._pos0 = np.asarray(gripper_pos, dtype=float)
            self._N = max(1, int(np.ceil(abs(ROLL_BACK) / ROLL_RATE)))
            print(f"[rollshift] roll {np.degrees(ROLL_BACK):+.1f}° / shift-y {self.shift:.3f}m "
                  f"over {self._N} frames, pos0=({self._pos0[0]:.3f},{self._pos0[1]:.3f},"
                  f"{self._pos0[2]:.3f})")

        if self._frozen is not None:
            cmd = self._frozen
        else:
            t = min(1.0, (self._frame + 1) / self._N)
            # ① 法兰回卷：直接命令 joint7（⑤⑨ 同款纯关节命令，保证转）。
            #   不强制末帧钉角：ramp 在 |diff|<=ROLL_RATE 时自然把命令钉到目标（命令连续无跳变），
            #   实际角随后平滑爬满——强制钉角会跳到滞后实际角前头，dwell 期猛追→"闪现"
            #   （用户 2026-08-24「不要最后强制竖直，直接闪现了最后一步」）
            diff = self._roll_target - cur[JOINT]
            roll_cmd = cur[JOINT] + float(np.clip(diff, -ROLL_RATE, ROLL_RATE))
            # ② -Y 平移：朝向用「把本帧法兰命令角写进关节集再 FK」的结果（而非滞后实际角），
            #   保证 IK 解出的其余关节与本帧法兰角度自洽——旧写法逐帧喂滞后实际朝向，
            #   其余关节随滞后漂移，末端姿态渐失垂直
            fk_j = cur.copy()
            fk_j[JOINT] = roll_cmd
            _, R_now = self.engine.fk_pose(fk_j)
            orient_q = _R_to_quat_wxyz(R_now)
            pos = np.array([self._pos0[0], self._pos0[1] - self.shift * t, self._pos0[2]])
            # warm start 用命令关节（v47 教训），不用滞后实际关节
            ws = self._last_cmd if self._last_cmd is not None else cur
            ik = self.engine.solve_verified(pos, ws, orient=orient_q)
            if ik is None:
                cmd = cur.copy()          # -Y 解不出就保持 y，下一帧再试
                cmd[JOINT] = roll_cmd     # 法兰仍继续转（joint7 由直接命令驱动，不依赖 IK）
            else:
                lim = np.full(7, self.engine.MAX_JOINT_DELTA)
                lim[JOINT] = ROLL_RATE
                delta = np.clip(ik - cur, -lim, lim)
                cmd = cur + delta
                cmd[JOINT] = roll_cmd  # 法兰由直接命令驱动，覆盖 IK 的 joint7
                self._last_cmd = cmd
            # 完成判定 = -Y 名义到点（t=1）即冻结**实际**关节（2026-08-25 去掉强制竖直：不再等
            #   实际法兰收敛到 0°，残余 1-3° 倾斜就随它倾斜——用户「转到0度有一点倾斜就倾斜吧」）。
            #   冻结实际关节 → ① 命令连续无跳变（不闪现）② 停住即稳定（dwell 静止=卡住）③ 无追赶。
            if self._frame + 1 >= self._N:
                self._frozen = np.asarray(cur, dtype=float).copy()

        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()

        self._frame += 1
        if self._frozen is not None:
            self._hold += 1
            if self._hold >= self.dwell:
                self._done = True
        if self._frame >= MAX_FRAMES:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
