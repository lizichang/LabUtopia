"""⑭ ReturnSpatula：药匙放回试管架中心孔（⑬ 倒粉后，把药匙还回架里）。

用户 2026-08-25（逐字）：「现在加动作，把药匙放回试管架」；
同日修正（逐字）：「完全不对，你还没有把药匙完全竖直，你需要把药匙水平移到架子上方
的时候需要药匙调整竖直，位置数据要对」——⑬ 去掉强制竖直后实际法兰有残余倾斜，
**不能**首帧 FK 采样当前朝向保持（会把倾斜带回架里），必须显式用 ORIENT_FWD
（pick ①-④ 已验证：该朝向 ⇔ 药匙世界 = REST，竖直、头下柄上、勺面沿 X）调直。
水平移动段带 orient=ORIENT_FWD → IK 边平移边把 joint7 转到竖直，到位时药匙完全竖直。

位置数据（pxr 实测场景 d2s_water_solubility.usd，2026-08-25 核对）：药匙原点
(0.6993,0.3608,0.828) = SPAT_XY、抓点 z0.94 = SPAT_GRASP_Z；试管架 (0.6803,0.3607)；
试管口 (0.659,0.241,0.9593)。常量均与场景一致。

- 4 段（先水平后竖直，避免斜线下探碰架）：
  ① 水平 MoveAction(orient=ORIENT_FWD) → 架孔正上方 (0.6993, 0.3608, 当前 z)，边平移边
     调竖直（z 锁；勺尖 ~0.9653 高于架顶 0.917 净空 4.8cm，不会碰架）
  ② 竖直 MoveAction(orient=ORIENT_FWD) → 抓点 (0.6993, 0.3608, 0.94)（= pick ② 的
     SPAT_GRASP，勺尖入孔至静止位 z 0.806；x/y 锁）
  ③ GripAction 张开到 GRIP_OPEN 放回 —— task._update_spatula 检测到 opening>0.03 →
     spatula_state attached→released、药匙 _rest_matrix() 归位、隐藏勺上粉
  ④ 竖直 MoveAction → 抬回安全高位 H（夹爪张开撤离，药匙留在架里）

复用 MoveAction（到达冻结 + dwell）/ GripAction，夹爪每帧发 grip_target（controller 从
⑬ 传播 GRIP_SPATULA 进来；③ 张开后 grip_target=GRIP_OPEN，④ 撤离保持张开）。
"""
import numpy as np

from controllers.atomic_actions.flametest.move_action import MoveAction
from controllers.atomic_actions.flametest.grip_action import GripAction

from .constants import H, SPAT_XY, SPAT_GRASP_Z, GRIP_OPEN, GRIP_SPATULA, ORIENT_FWD


class ReturnSpatula:
    """⑬ 后把药匙平移回试管架孔（水平段同时调竖直）并竖直放下、松爪放回。"""

    def __init__(self, engine, dwell=20):
        self.engine = engine
        self.dwell = int(dwell)
        self.grip_target = GRIP_SPATULA   # 进入时仍夹着药匙（controller 会从上一动作传播）
        self.reset()

    def reset(self):
        self._steps = None
        self._idx = 0
        self._frame = 0
        self._done = False

    def _build(self, joints, gripper_pos):
        # 朝向 = ORIENT_FWD（药匙竖直 REST）——不用 FK 采样当前朝向（⑬ 有残余倾斜，
        # 采样=保持倾斜，用户 2026-08-25「需要药匙调整竖直」）。水平段由 IK 边平移边调直。
        orient_q = np.asarray(ORIENT_FWD, dtype=float)
        gp = np.asarray(gripper_pos, dtype=float)
        above = (SPAT_XY[0], SPAT_XY[1], gp[2])          # ① 架孔正上方（锁当前 z）
        home = (SPAT_XY[0], SPAT_XY[1], SPAT_GRASP_Z)    # ② 抓点高度（= pick ② SPAT_GRASP）
        top = (SPAT_XY[0], SPAT_XY[1], H)                # ④ 撤离到安全高位
        print(f"[return] orient=ORIENT_FWD [{orient_q[0]:.4f},{orient_q[1]:.4f},"
              f"{orient_q[2]:.4f},{orient_q[3]:.4f}] "
              f"above=({above[0]:.3f},{above[1]:.3f},{above[2]:.3f}) home_z={home[2]:.3f}")
        return [
            MoveAction(self.engine, above, dwell=self.dwell, orient=orient_q),
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
