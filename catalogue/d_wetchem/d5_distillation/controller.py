# -*- coding: utf-8 -*-
"""D5 蒸馏分离控制器：段 1 跑点火元动作，段 2 等 task 相态机推完蒸馏再报成功。

两段式（同 B2）：
  段 1  LightFlamePass：拿火柴点燃酒精灯（机械臂唯一动作，装置预组装）。
  段 2  点火完成后机械臂回显保持（夹爪停在 GRIP_OPEN，火柴已放回），等 task 相态机
        自行推 ignited → heating（气泡渐起）→ boiling（沸腾）→ distilling（馏出液滴落
        收集）→ finishing（火焰熄灭）→ done。读到 phase=="done" → 上报成功并请求 reset。

与 d3l/b2/d9 同构分层（Lula IK + 元动作组合）。动作级契约（grip 每帧发送、到达冻结、
dwell、跨元动作 grip_target 传播）沿用 flametest/d2s。
"""
import os
import numpy as np
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.extensions import get_extension_path_from_name

from controllers.base_controller import BaseController as TaskBaseController
from controllers.atomic_actions.flametest import IkMotionEngine
from .meta_actions import LightFlamePass
from .meta_actions.constants import GRIP_OPEN


class D5DistillationTaskController(TaskBaseController):
    """Composite controller: 段 1 点火元动作，段 2 等 task 蒸馏相态推完再报成功。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d5] controller VERSION v1 (LightFlamePass + phase watch: "
              "heat -> boil -> distill -> collect)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l/d9/b2）
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 段 1：点火元动作（唯一元动作）；段 2 等 task 相态机推完蒸馏
        self.meta_classes = [LightFlamePass]
        self.meta_names = ["S1 ignite the alcohol lamp with a match (apparatus pre-assembled)"]
        self.meta_actions = [C(self.engine) for C in self.meta_classes]
        self._meta_idx = 0
        self._h5_sample = 0
        self._start = True
        self._done = False

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)
        self._done = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self._meta_idx = 0
        self._start = True
        self._done = False
        self._h5_sample = 0
        if self.mode == "collect":
            for m in self.meta_actions:
                m.reset()
            self.rmp_controller.reset()
        else:
            self.inference_engine.reset()

    def step(self, state):
        self.state = state
        if self.mode == "collect":
            return self._step_collect(state)
        return self._step_infer(state)

    def _cache_h5(self, state):
        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            obs_names = getattr(getattr(self.cfg, "infer", None), "obs_names", None) or {}
            camera_images = (
                {k: v for k, v in state["camera_data"].items() if k in obs_names}
                if obs_names else state["camera_data"]
            )
            self.data_collector.cache_step(
                camera_images=camera_images,
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )

    def _step_collect(self, state):
        if self._done:
            print("[d5] all phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        jp = state.get("joint_positions")

        # 段 2：点火完成，机械臂回显保持，等 task 相态机推完蒸馏 → phase=="done" 报成功
        if self._meta_idx >= len(self.meta_actions):
            if state.get("phase") == "done":
                self._done = True
            action = (ArticulationAction(joint_positions=np.array(jp, dtype=float))
                      if jp is not None else None)
            self._cache_h5(state)
            return action, False, False

        if self._start:
            # 首帧：只发夹爪打开（稳定握姿再开始），臂不动
            self._start = False
            target = np.full(jp.shape[0], np.nan)
            target[7] = GRIP_OPEN / get_stage_units()
            target[8] = GRIP_OPEN / get_stage_units()
            action = ArticulationAction(joint_positions=target)
        else:
            meta = self.meta_actions[self._meta_idx]
            action = meta.forward(state)
            if meta.is_done():
                print(f"[d5] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1

        self._cache_h5(state)
        return action, False, False

    def _step_infer(self, state):
        if self._done:
            self.reset_needed = True
            return None, True, self._last_success
        if state.get("phase") == "done":
            self._done = True
            self._last_success = True
        state["language_instruction"] = self.get_language_instruction()
        action = self.inference_engine.step_inference(state)
        return action, False, self.is_success()

    def is_success(self):
        return self._done

    def get_language_instruction(self):
        return ("Light the alcohol lamp with a match; the distillation apparatus is "
                "pre-assembled, so the liquid in the flask heats to boiling, the vapor "
                "condenses in the condenser, and the distillate drips into the receiving "
                "flask until collection is complete.")
