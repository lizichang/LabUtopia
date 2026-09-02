"""焰色反应控制器：整个实验 = 顺序执行 10 个元动作。

v44 分层重构（贴合物理规律：无瞬移、无空抓、无悬空、无乱动）：
  - atomic_actions/flametest/（IK 驱动小动作：MoveAction / GripAction / HoldAction
    + IkMotionEngine，见 diag_rmp.py：RMP 对焰色场景发散，故弃用 RMP 改用 Lula IK）
  - flametest_meta_actions/（10 个元动作，一类一文件，各自组合一串小动作）
  - 本控制器：只做"整个实验"——实例化 10 个元动作，_step_collect 按序 forward()，
    当前元动作 is_done() 后进下一个，全部完成 → success。

保留 v21-v43 已验证的行为契约：Lula IK 求解 + FK 验证 + 关节钳制 + 到达冻结 +
dwell 停留 + 夹爪每帧显式发送（v41）；_step_infer / is_success / get_language_instruction
/data_collector 接口不变，factory 注册名 "flametest" 不变。
"""
import os
import numpy as np
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.extensions import get_extension_path_from_name

from .base_controller import BaseController as TaskBaseController
from .atomic_actions.flametest import IkMotionEngine
from .flametest_meta_actions import (
    OpenHclStopper, DripHclAcid, IgniteLamp, DipWireAcid, BurnClean,
    RepeatDipBurn, Cool, DipPowder, BurnStain, Extinguish,
)
from .flametest_meta_actions.constants import GRIP_OPEN


class FlameTestTaskController(TaskBaseController):
    """Composite controller: 整个实验 = 10 个元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[flametest] controller VERSION v44 (layered: atomic_actions + 10 meta-actions, IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 v31）：精确关节控制替代 RMP（RMP 对远距离低 z 目标发散）
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 10 个元动作（见 flametest_meta_actions/，一类一文件）
        self.meta_classes = [
            OpenHclStopper, DripHclAcid, IgniteLamp, DipWireAcid, BurnClean,
            RepeatDipBurn, Cool, DipPowder, BurnStain, Extinguish,
        ]
        self.meta_names = [
            "P1 open hcl stopper", "P2 drip 3 drops", "P3 ignite alcohol lamp",
            "P4 dip wire in acid", "P5 burn (no color)", "P6 repeat dip+burn x3",
            "P7 cool 5s", "P8 dip powder", "P9 burn 2-5s (stain)",
            "P10 extinguish",
        ]
        self.meta_actions = [C(self.engine) for C in self.meta_classes]
        self._meta_idx = 0
        self._h5_sample = 0
        self._start = True

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        if self.mode == "collect":
            self._meta_idx = 0
            self._h5_sample = 0
            self._start = True
            for m in self.meta_actions:
                m.reset()
            self.rmp_controller.reset()
        else:
            self.inference_engine.reset()

    def step(self, state):
        self.state = state
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state):
        if self._meta_idx >= len(self.meta_actions):
            print("[flametest] all 10 meta-actions done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        if self._start:
            # 首帧：只发夹爪打开（稳定握姿再开始），臂不动
            self._start = False
            target = np.full(state["joint_positions"].shape[0], np.nan)
            target[7] = GRIP_OPEN / get_stage_units()
            target[8] = GRIP_OPEN / get_stage_units()
            action = ArticulationAction(joint_positions=target)
        else:
            meta = self.meta_actions[self._meta_idx]
            action = meta.forward(state)
            if meta.is_done():
                print(f"[flametest] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1
                if self._meta_idx < len(self.meta_actions):
                    # 修 bug6：跨元动作夹爪目标传递——铂丝跨 ④⑤⑥⑦⑧⑨ 持握，
                    # 无 GripAction 的元动作继承上一段的夹爪状态，否则灼烧/冷却中
                    # 爪子张开、铂丝悬空吸附（用户报"夹紧后又松开"）。
                    self.meta_actions[self._meta_idx].grip_target = meta.grip_target

        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            self.data_collector.cache_step(
                camera_images=state["camera_data"],
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )
        return action, False, False

    def _step_infer(self, state):
        if self._meta_idx >= len(self.meta_actions):
            self.reset_needed = True
            return None, True, self._last_success

        language_instruction = self.get_language_instruction()
        state["language_instruction"] = language_instruction
        action = self.inference_engine.step_inference(state)

        return action, False, self.is_success()

    def is_success(self):
        return self._meta_idx >= len(self.meta_actions)

    def get_language_instruction(self):
        return ("Open the dilute hydrochloric acid bottle, drip 2-3 drops with the "
                "dropper onto the watch glass, ignite the alcohol lamp with a match, "
                "dip the platinum wire in the acid, burn it in the lamp "
                "flame 3-4 times until no characteristic color, cool for 5 s, dip "
                "the solid sample powder, burn for 2-5 s to observe the flame "
                "color, extinguish the flame with the cap, rinse the wire and return it")
