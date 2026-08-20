"""MoveAction：IK 移动到目标点，到达后冻结、按 dwell 停留。

approach / descend / translate / lift_vertical / dip_hold 都是"移到某点
（可带停留）"的参数化实例：
  - approach(pt)      = mv((pt.xy, H))           高位接近
  - descend(pt)       = mv(pt)                   垂直下探到抓/放点
  - lift_vertical     = mv((pt.xy, z), dwell)    同 xy 垂直提出 + 强制停顿
  - translate(pt)     = mv(pt)                   平移
  - dip_hold(pt, n)   = mv(pt, n)                下探到 pt 停留 n 帧

直线约束（v46 垂直 / v47 任意单轴）：原实现"解一次 IK + 每帧关节钳制"，joint 空间
插值会把 TCP 拉成弧线——滴管/瓶塞/灯帽下探和提出时就斜着走、带抖动（用户报
"斜着拿/穿模"）；水平平移同样会把 TCP 甩高（D2-S ⑥ 法兰转后水平对齐粉末 z 先升高，
用户 2026-08-17 报）。若 x/y/z 中恰好只有一轴变化超过 AXIS_EPS（垂直 = 仅 z 变；
水平 = 仅 x 或仅 y 变），判定为直线段：不变两轴锁死目标值、变化轴每帧推进 VZ_STEP，
逐帧沿直线上重新解 IK（cur7 warm start，FK 验证 <6mm，TCP 收敛在这条线上），TCP 走
严格直线，无斜拉、无弧线抖动。

冻结判定（v43）：TCP 与目标 3D 距离 <1cm 连续 3 帧 → 冻结关节保持位置，等 dwell
帧。近奇异抓点处同一 TCP 可由多个 IK 分支到达，冻结即稳住抓点，让任务侧 attach
判定通过；不冻结会出现"dist 瞬时掠过即完成→臂还在摆→attach 拿不到连续近窗"。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

from .ik_engine import ORIENT_EPS, quat_to_rot, rot_angle

# 直线段判定阈值（v47）：x/y/z 任一轴起点与目标偏差小于此值视为"该轴不变"。
# 恰好只有一轴变化 → 沿该轴走直线（approach 冻结保证起止偏差 <1cm，这里留裕量）
AXIS_EPS = 0.015
# 垂直段每帧 z 推进量（m/帧）。v46 初版 0.008 实测：z 步大 → 所需关节重配超
# MAX_JOINT_DELTA(0.015)，钳制后 TCP 横向拖滞 4~9cm（diag 轨迹验证），改小到
# 钳制内（2mm 步长 @60Hz ≈ 0.12 m/s，TCP 贴线无拖滞，下探/提出略慢但稳）。
# 2026-08-14 晚：MAX_JOINT_DELTA 0.008→0.006，同步降到 0.0015（≈0.09 m/s）保持
# 4:1 比例，垂直段关节量仍在钳制内；药匙勺头宽面正对 camera1 后臂末端微振荡被
# 放大，降速减小每步起停冲击（见 ik_engine.MAX_JOINT_DELTA 注释）。
VZ_STEP = 0.0015


class MoveAction:
    """移到目标位置，到达冻结后停留 dwell 帧。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction。
    夹爪每帧显式发送 grip_target（v41：杜绝 NaN→applied 替换不可靠）。
    """

    def __init__(self, engine, pos, dwell=0, label="", orient=None, linewalk=True):
        self.engine = engine
        self.pos = np.asarray(pos, dtype=float)
        self.dwell = int(dwell)
        self.label = label
        # 可选朝向（w,x,y,z）：None 沿用引擎默认（手指朝下）；显式传时解 IK
        # 目标朝向 + 冻结需朝向收敛（原地转水平时位置已到、朝向仍在解）
        self.orient = orient
        self._rot_target = None if orient is None else quat_to_rot(orient)
        # linewalk=False：强制单次 IK（解目标一次 + 关节空间钳制逼近），不逐帧重解。
        # 近奇异区（d2s 入粉下降 y≈底座柱 z 低）逐帧重解会分支翻转/漂移、永不到位，
        # 单次 IK 在关节空间直达目标，鲁棒；代价是 TCP 非严格直线，仅用于短距下降。
        self._linewalk = linewalk
        self.reset()

    def reset(self):
        self._frame = 0
        self._arrived = 0
        self._hold = 0
        self._ik_target = None
        self._frozen = None
        self._done = False
        self._solved = False
        self._walk_axis = None    # 直线段：唯一变化轴的索引（0=x 1=y 2=z）；None=单次 IK
        self._goal = None         # 直线段：该轴当前推进到的坐标（首帧自 gripper_pos 起逐帧步进）
        self._last_cmd = None     # 上次命令的关节（直线段 warm start 用，见 forward 注释）

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        if not self._solved:
            self._solved = True
            gp = np.asarray(gripper_pos, dtype=float)
            # v47 直线段判定：x/y/z 中恰好只有一轴变化超过阈值 → 沿该轴走直线
            # （垂直 = 仅 z 变；水平 = 仅 x 或仅 y 变）。两轴以上变化 → 单次 IK。
            # linewalk=False 时强制走单次 IK（近奇异短距下降用）。
            changed = np.abs(self.pos - gp) > AXIS_EPS
            if self._linewalk and int(changed.sum()) == 1:
                self._walk_axis = int(np.argmax(changed))
                self._goal = float(gp[self._walk_axis])
                self._ik_target = None
            else:
                self._ik_target = self.engine.solve_verified(self.pos, cur, self.orient)
                if self._ik_target is None:
                    print(f"[flametest] IK FAIL target={np.round(self.pos, 3)} — hold position, will force-done")

        if self._frozen is not None:
            cmd = self._frozen
        elif self._walk_axis is not None and self._goal is not None:
            # 直线段：每帧沿变化轴推进并重新解 IK（不变轴锁目标值，TCP 走直线）
            a = self._walk_axis
            dv = self.pos[a] - self._goal
            step = VZ_STEP if dv > 0 else -VZ_STEP
            if abs(dv) < VZ_STEP:
                step = dv
            self._goal += step
            tgt = self.pos.copy()
            tgt[a] = self._goal
            # warm start 用上次"已命令"关节而非滞后的实际关节：实际关节受 PD+钳制
            # 滞后于命令，直接喂实际关节会让 Lula 在近奇异区（如 d2s 入粉 y≈底座柱 z 下降）
            # 逐帧落入不同局部最小，joint6 单调漂移（实测 0.15→1.94）、永不到位 force-done；
            # 命令跟踪的 warm start 与 probe 干净走线一致（joint6≈0.07）。钳制仍用实际 cur。
            ws = self._last_cmd if self._last_cmd is not None else cur
            ik = self.engine.solve_verified(tgt, ws, self.orient)
            if ik is None:
                cmd = cur  # 解不出就保持，下一帧再试
            else:
                delta = np.clip(ik - cur,
                                -self.engine.MAX_JOINT_DELTA, self.engine.MAX_JOINT_DELTA)
                cmd = cur + delta
                self._last_cmd = cmd
        elif self._ik_target is not None:
            delta = np.clip(self._ik_target - cur,
                           -self.engine.MAX_JOINT_DELTA, self.engine.MAX_JOINT_DELTA)
            cmd = cur + delta
        else:
            cmd = cur

        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()

        self._frame += 1
        if self._frozen is not None:
            # 已冻结：只累计 hold 帧（不因微小漂移重置）
            self._hold += 1
            if self._hold >= self.dwell:
                self._done = True
        else:
            dist3d = float(np.linalg.norm(np.asarray(gripper_pos, dtype=float) - self.pos))
            # 朝向收敛（仅显式传 orient 的段）：位置到位 + 朝向对齐才冻结。
            # 原地转水平/倾倒时位置已到目标、朝向仍在 IK 收敛中，不能按位置即冻。
            orient_ok = True
            if self._rot_target is not None:
                _, fk_rot = self.engine.fk_pose(cur)
                orient_ok = rot_angle(fk_rot, self._rot_target) < ORIENT_EPS
            if dist3d < 0.010 and orient_ok:
                self._arrived += 1
            else:
                self._arrived = 0
            if self._arrived >= 3:
                self._frozen = np.asarray(joints[:7], dtype=float).copy()
                print(f"[flametest] freeze at tgt={np.round(self.pos, 3)} "
                      f"gripper={np.round(gripper_pos, 3)} dist3d={dist3d:.4f}")
                self._hold = 0
        if not self._done and self._frame >= self.dwell + 600:
            print(f"[flametest] move force-done t={self._frame} (dwell={self.dwell}) "
                  f"target={np.round(self.pos, 3)}")
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
