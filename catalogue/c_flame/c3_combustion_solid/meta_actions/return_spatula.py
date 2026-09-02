"""⑭ 后 ReturnSpatula：药匙放回试管架中心孔（⑭ 同步倒粉后，把药匙还回架里）。

用户 2026-09-01（逐字）：「放回药匙，放回前先对准，然后把药匙强制旋转竖直再向下放好。」

C3 关键几何（与 d2s 不同，**必须 lift_first**）：⑭ 同步倒粉结束 = TCP (0.587,0.3308,0.98)、
药匙竖直（法兰随动 ≈0）、勺尖 (0.587,0.3308,**0.846**)——勺尖低于架顶 0.917（净空为负）。
若按 d2s 普通版「当前 z 直接水平移回架孔上方」（勺尖 0.846），回程勺尖会拖穿架顶板。
故照 d2s **lift_first 模式**（D3-S 同款）：
  ① 原位竖直提到安全高位 H（勺尖 = H−0.134 = 1.016 > 架顶 0.917+0.05，长垂直段把药匙调直）
  ② 高位水平移回架孔正上方 (SPAT_XY, H)（勺尖离地高、全程不碰架/燃烧匙）
  ③ 竖直下探入孔到 SPAT_GRASP_Z=0.94（勺尖入孔至静止位 z≈0.806，x/y 锁）
  ④ 松爪放回（task 检测 opening>0.03 → spatula_state released、_rest_matrix() 归位、隐藏勺上粉）
  ⑤ 竖直抬离到安全高位 H（夹爪张开撤离，药匙留在架里）

朝向全段 = ORIENT_FWD（pick ①-④ 已验证：该朝向 ⇔ 药匙世界 = REST，竖直、头下柄上）。
**不能**首帧 FK 采样当前朝向保持——⑭ 不强制竖直（t>=1 即冻结），实际法兰有残余倾斜，
采样=保持倾斜带回架里（d2s 用户 2026-08-25「需要药匙调整竖直」），必须显式 ORIENT_FWD
让 IK 边移边把 joint7 转到竖直。

复用 MoveAction（到达冻结 + dwell）/ GripAction，夹爪每帧发 grip_target（controller 从
⑭ 传播 GRIP_SPATULA 进来；④ 张开后 grip_target=GRIP_OPEN，⑤ 撤离保持张开）。
"""
import numpy as np

from controllers.atomic_actions.flametest.move_action import MoveAction
from controllers.atomic_actions.flametest.grip_action import GripAction

from .constants import H, SPAT_XY, SPAT_GRASP_Z, GRIP_OPEN, GRIP_SPATULA, ORIENT_FWD


class ReturnSpatula:
    """⑭ 后把药匙原位提到安全高位 → 高位移回架孔 → 竖直下探入孔 → 松爪放回 → 撤离。"""

    def __init__(self, engine, dwell=20, home=None, lift_first=True):
        self.engine = engine
        self.dwell = int(dwell)
        self.spatula_home = SPAT_XY if home is None else tuple(float(v) for v in home)
        self.lift_first = bool(lift_first)   # C3 勺尖 0.846 < 架顶 0.917，必须 True
        self.grip_target = GRIP_SPATULA      # 进入时仍夹着药匙（controller 会从⑭传播）
        self.reset()

    def reset(self):
        self._steps = None
        self._idx = 0
        self._frame = 0
        self._done = False

    def _build(self, joints, gripper_pos):
        orient_q = np.asarray(ORIENT_FWD, dtype=float)
        gp = np.asarray(gripper_pos, dtype=float)
        sh = self.spatula_home
        home = (sh[0], sh[1], SPAT_GRASP_Z)   # 抓点高度（= pick ② SPAT_GRASP）
        top = (sh[0], sh[1], H)               # 撤离到安全高位
        if self.lift_first:
            # C3 ⑭ 结束勺尖 0.846 < 架顶 0.917：水平回程会拖穿架顶板 → 先原位竖直提到 H
            # （勺头 1.016 高过架顶，且长垂直段 + ORIENT_FWD 彻底调直残余倾斜），再高位
            # 水平移回架孔正上方（勺头离地高不碰架），最后竖直下探入孔。
            lift = (gp[0], gp[1], H)          # ① 原位竖直提到安全高位（调直）
            above_h = (sh[0], sh[1], H)       # ② 高位水平移到架孔正上方（对准）
            print(f"[return] lift_first: lift=({lift[0]:.3f},{lift[1]:.3f},{lift[2]:.3f}) "
                  f"above_h=({above_h[0]:.3f},{above_h[1]:.3f},{above_h[2]:.3f}) home_z={home[2]:.3f}")
            return [
                MoveAction(self.engine, lift, dwell=self.dwell, orient=orient_q),
                MoveAction(self.engine, above_h, dwell=self.dwell, orient=orient_q),
                MoveAction(self.engine, home, dwell=self.dwell, orient=orient_q),
                GripAction(self.engine, GRIP_OPEN, 25),
                MoveAction(self.engine, top, dwell=self.dwell, orient=orient_q),
            ]
        print(f"[return] orient=ORIENT_FWD [{orient_q[0]:.4f},{orient_q[1]:.4f},"
              f"{orient_q[2]:.4f},{orient_q[3]:.4f}] "
              f"above=({gp[0]:.3f},{gp[1]:.3f},{gp[2]:.3f}) home_z={home[2]:.3f}")
        return [
            MoveAction(self.engine, (sh[0], sh[1], gp[2]), dwell=self.dwell, orient=orient_q),
            MoveAction(self.engine, home, dwell=self.dwell, orient=orient_q),
            GripAction(self.engine, GRIP_OPEN, 25),
            MoveAction(self.engine, top, dwell=self.dwell, orient=orient_q),
        ]

    def forward(self, state):
        if self._steps is None:
            self._steps = self._build(state["joint_positions"], state["gripper_position"])
        step = self._steps[self._idx]
        cmd = step.forward(state["joint_positions"], state["gripper_position"], self.grip_target)
        if step.is_done():
            if isinstance(step, GripAction):
                self.grip_target = step.width
            self._idx += 1
            if self._idx >= len(self._steps):
                self._done = True
        self._frame += 1
        return cmd

    def is_done(self):
        return self._done
