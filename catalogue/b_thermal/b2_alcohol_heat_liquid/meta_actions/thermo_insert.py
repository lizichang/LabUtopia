"""ThermoInsertRotate：位置 + 朝向同步插值原子动作（B2 段 2 挂温度计用）。

2026-08-26 用户逐字：「现在加一个动作同时完成边竖直往下伸入边把温度计旋转垂直，
需要同时开始同时结束，而且最终温度计圆环是挂在钩子上然后伸进试管里面」。

段 1 终点（APPROACH，倾斜 20°、泡尖正对管口上方 5mm）→ 段 2 终点（竖直、挂环
套铁架台钩、泡尖伸进试管浸液）。本动作把**位置线性插值 + 朝向 Slerp 插值**用同一个
t = frame/total 同步推进（同时开始、同时结束），逐帧 solve_verified（cur warm start，
FK 验证位置 <6mm + 朝向 <ORIENT_EPS）+ 关节钳制 MAX_JOINT_DELTA。总帧数到即 done。

不是 MoveAction 变体：目标是**连续移动**（位置与朝向每帧都在变），不能用「TCP 距
目标 <1cm 冻结」判定（会误冻）；也不符合 line-walk（x/z 两轴同时变 + 朝向同时转）。
仿 ShakeAction 逐帧驱动（ShakeAction 仅单朝向振荡，本动作位置+朝向同 t 双插值）。

朝向约定 = 引擎/scipy [x,y,z,w] 元组（同 ORIENT_FWD/ORIENT_TILT_20，绕 Y 纯旋转；
Gf.Slerp 在 [x,y,z,w] 转 Gf.Quatf（real=w, imag=(x,y,z)）间插值，结果转回元组）。
"""
import numpy as np
from pxr import Gf
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


def _orient_to_gf(q):
    """引擎 [x,y,z,w] 元组 -> Gf.Quatf（Gf.Quat 布局 real=w、imag=(x,y,z)）。"""
    return Gf.Quatf(q[3], Gf.Vec3f(q[0], q[1], q[2]))


def _gf_to_orient(q):
    """Gf.Quatf -> 引擎 [x,y,z,w] 元组。"""
    im = q.GetImaginary()
    return (float(im[0]), float(im[1]), float(im[2]), float(q.GetReal()))


class ThermoInsertRotate:
    """位置线性 + 朝向 Slerp 同步插值（同时开始同时结束）。"""

    def __init__(self, engine, start_pos, end_pos, start_orient, end_orient,
                 total=180, label="insert-rotate"):
        self.engine = engine
        self.start = np.asarray(start_pos, dtype=float)
        self.end = np.asarray(end_pos, dtype=float)
        self.q0 = _orient_to_gf(start_orient)
        self.q1 = _orient_to_gf(end_orient)
        self.total = int(total)
        self.label = label
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        # 同 t 同步推进：t=0 → 起点（倾斜 20° 泡尖对管口上 5mm），t=1 → 终点（竖直挂钩）
        t = min(1.0, self._frame / self.total)
        pos = self.start + (self.end - self.start) * t
        orient = _gf_to_orient(Gf.Slerp(float(t), self.q0, self.q1))
        ik = self.engine.solve_verified(pos, cur, orient=orient)
        if ik is None:
            cmd = cur  # 解不出就保持，下一帧再试（插值相位继续走）
        else:
            delta = np.clip(ik - cur, -self.engine.MAX_JOINT_DELTA,
                            self.engine.MAX_JOINT_DELTA)
            cmd = cur + delta
        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()
        self._frame += 1
        if self._frame >= self.total:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
