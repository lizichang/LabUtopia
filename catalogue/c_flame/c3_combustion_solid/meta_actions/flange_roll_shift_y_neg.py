"""⑭ FlangeRollShiftYNeg（C3 版·同步）：法兰 -90°→0° 转竖直 同时 TCP 往 -Y 平移 18cm，同始同终。

用户 2026-09-01 明确要求**同步**（「一个动作在药匙旋转竖直的同时往-y移动」）——我此前两阶段版
（先平移后旋转）被反馈「-y移动和法兰旋转不是同步的」。同步 = 同一进度 t 同时驱动
① joint7 法兰回卷（直接命令）② TCP -Y 平移（锁 x/z、y 推进），同始同终。

基于 d2s FlangeRollShiftYNeg 骨架 + 四点加固（2026-09-01 启动抖 → 改版）：
  ③ 命令基准：首帧用实际关节、之后用上一帧**命令**关节——朝向采样 / IK warm start /
     输出增量全基于它。勿用实际关节采样朝向（PD 滞后 → 朝向目标每帧漂移 → 一启动就抖）。
  ④ 收紧朝向验证 orient_eps=0.01（≈0.57°，B1 插管同值）：位置每帧只动 2.6mm、朝向每帧
     只动 0.86°，若用默认容差 8.6°，IK 在朝向球面内任意选解 → 前 6 关节空转横跳 → 抖。
  ⑤ 两级无解退路：收紧后低 z 双约束更易无解——先放宽 eps 到 0.10（≈5.7°，朝向目标不变），
     再退回只解位置（orient=None，不验朝向）——保证 TCP 仍往 -y 水平走不卡死。
  ⑥ 完成：TCP 名义到点（t>=1）即冻结实际关节 → dwell——**不等待实际法兰收敛**（用户
     2026-09-01「去掉最后的强制竖直，不然最后一步都不是同时开始同时结束的」）；命令每帧
     clip 推进连续 → 不抖，同一进度 t → 法兰与 -y 同时开始同时结束。

完成后：TCP (0.587, 0.3308, 0.98)、法兰随动（不强制转满，残余角度由 PD 滞后决定）；
勺尖 = TCP+0.134·(0,0,-1) = (0.587, 0.3308, 0.846)（台面上方 4.6cm）。旋转途中粉从勺口滑出
（同步倒粉，燃烧匙在下方路径上）。夹爪通道每帧发 grip_target。
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

from .constants import SPOON_Y_NEG_LAST
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.flange_roll import RATE as FLANGE_RATE

# 最后一个关节 = panda_joint7（flange roll，索引 6），URDF 限位 ±2.8973 rad。
JOINT = 6
LIMIT = 2.8973
ROLL_BACK = +np.pi / 2       # 法兰 -90°→0°（+90° 回卷，与 ⑤⑨ 反向）
ROLL_RATE = FLANGE_RATE      # 每帧推进 rad（0.015，≈0.9 rad/s）
MAX_FRAMES = 300             # 兜底


def _R_to_quat_wxyz(R):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（scalar-first，与 Lula/quats_to_rot_matrices 同读法）。"""
    q = SciRotation.from_matrix(np.asarray(R, dtype=float)).as_quat()  # scipy [x,y,z,w]
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


class FlangeRollShiftYNeg:
    """同步：法兰 -90°→0° + TCP 往 -Y 平移 shift，同一进度 t 同始同终；到位且实际法兰收敛才冻结。"""

    def __init__(self, engine, shift=SPOON_Y_NEG_LAST, dwell=15):
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
            print(f"[rollshift-c3] SYNC roll {np.degrees(ROLL_BACK):+.1f}° / shift-y "
                  f"{self.shift:.3f}m over {self._N} frames, pos0=({self._pos0[0]:.3f},"
                  f"{self._pos0[1]:.3f},{self._pos0[2]:.3f})")

        if self._frozen is not None:
            cmd = self._frozen
        else:
            t = min(1.0, (self._frame + 1) / self._N)
            # ① 法兰回卷：直接命令 joint7，每帧 clip 推进（连续无跳变 → 不闪现）
            diff = self._roll_target - cur[JOINT]
            roll_cmd = cur[JOINT] + float(np.clip(diff, -ROLL_RATE, ROLL_RATE))
            # ② TCP 位置：锁 x/z、y 推进 → 水平直线
            pos = np.array([self._pos0[0], self._pos0[1] - self.shift * t, self._pos0[2]])
            # ③ 命令基准：首帧用实际关节、之后用上一帧命令关节——勿用实际（PD 滞后 →
            #    朝向目标每帧漂移 → 一启动就抖）。朝向/IK 解/输出增量全基于它，命令系自洽。
            ws = self._last_cmd if self._last_cmd is not None else cur
            # ④ 朝向：本帧法兰命令角写进命令基准再 FK 采样（前 6 关节与法兰旋转同步）；
            #    收紧 orient_eps（0.01 rad≈0.57°）锁死 IK null space——位置每帧只动 2.6mm、
            #    朝向每帧只动 0.86°，若容差 8.6° 则 IK 在朝向球面内任意选解 → 前 6 关节
            #    空转横跳 → 启动抖
            fk_j = ws.copy()
            fk_j[JOINT] = roll_cmd
            _, R_now = self.engine.fk_pose(fk_j)
            orient_q = _R_to_quat_wxyz(R_now)
            ik = self.engine.solve_verified(pos, ws, orient=orient_q, orient_eps=0.01)
            if ik is None:
                # ⑤ 退路 1：收紧后低 z 双约束易无解——朝向目标不变，验证放宽到 5.7°
                ik = self.engine.solve_verified(pos, ws, orient=orient_q, orient_eps=0.10)
            if ik is None:
                # ⑤ 退路 2：只解位置（不验朝向），保证 TCP 仍往 -y 水平走不卡死
                ik = self.engine.solve_verified(pos, ws)
            if ik is None:
                cmd = cur.copy()          # 仍解不出就保持位置，下一帧再试
                cmd[JOINT] = roll_cmd     # 法兰继续转（joint7 直接命令，不依赖 IK）
            else:
                lim = np.full(7, self.engine.MAX_JOINT_DELTA)
                lim[JOINT] = ROLL_RATE
                delta = np.clip(ik - ws, -lim, lim)
                cmd = ws + delta          # 命令基准增量（非实际，无追赶振荡）
                cmd[JOINT] = roll_cmd     # 法兰由直接命令驱动，覆盖 IK 的 joint7
                self._last_cmd = cmd
            # ⑥ 完成：TCP 名义到点（t>=1）即冻结实际关节——不等待实际法兰收敛（用户
            #    2026-09-01「去掉最后的强制竖直，不然最后一步都不是同时开始同时结束的」；
            #    同一进度 t 驱动 → 命令层面法兰与 -y 同时到点，同始同终）。dwell 后结束。
            if t >= 1.0:
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
