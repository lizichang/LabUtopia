"""ShakeAction：高位水平来回振荡（"震荡来回 N 下"）。

抓试管后"震荡来回 N 下"用——在 center 附近沿 axis 做正弦往复（相位从 0 到
2π·cycles，首尾都回到中心，起止无横移），逐帧 IK 重解（cur7 warm start，FK
验证）+ 关节钳制 MAX_JOINT_DELTA。全程手指朝下、被握物纯平移保竖立。

不是 MoveAction 变体：目标是**连续移动**的，不能用"TCP 距目标 <1cm 冻结"判定
（会误冻）；振荡总帧数到即 done。一个来回 = period 帧（60Hz 下 period 60 ≈ 1s）。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction


class ShakeAction:
    """在 center 附近沿 axis 正弦振荡 cycles 个来回后完成。"""

    def __init__(self, engine, center, axis=(1, 0, 0), amplitude=0.02,
                 cycles=3, period=60, label="shake", orient=None):
        self.engine = engine
        self.center = np.asarray(center, dtype=float)
        self.axis = np.asarray(axis, dtype=float)
        self.axis = self.axis / float(np.linalg.norm(self.axis))
        self.amplitude = float(amplitude)
        self.cycles = int(cycles)
        self.period = int(period)
        self.label = label
        # 可选朝向（w,x,y,z）：None 沿用引擎默认（手指朝下）；显式传时震荡全程
        # 保持该朝向（D2-S 试管远在 +X，手指朝前 ORIENT_FWD 才够得着）。
        self.orient = orient
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False
        self._total = int(self.cycles * self.period)

    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        # 相位从 0 走 2π·cycles（每 period 帧一个来回），正弦 0→峰值→0 往复
        phase = 2.0 * np.pi * (self._frame / self.period)
        tgt = self.center + self.axis * (self.amplitude * np.sin(phase))
        ik = self.engine.solve_verified(tgt, cur, orient=self.orient)
        if ik is None:
            cmd = cur  # 解不出就保持，下一帧再试（振荡相位继续走）
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
