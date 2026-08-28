"""E3「密度测定」控制器：整个实验 = 顺序执行 4 个元动作。

v44 同构分层（与 flametest/d2s/e1 相同：Lula IK + 元动作组合）：
  - atomic_actions/flametest/（IK 原子动作，RMP 对低 z 下探发散，用 Lula IK）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

动作序列（v2 加天平完整测密度；量筒预置天平称盘上不动、样品瓶预开盖、移液管+洗耳球插架）：
  ① PickPipette     夹取移液管（抓点 = 尖端上方 0.09m，v2 压缩管身）
  ② DrawPipette     尖端伸入样品瓶吸液（模拟挤压洗耳球）
  ③ TransferPipette 移液管移到天平上量筒放液（放液后天平屏 m1→m2+ρ）
  ④ ReturnPipette   移液管放回架孔

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest。
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
from .meta_actions import PickPipette, DrawPipette, TransferPipette, ReturnPipette
from .meta_actions.constants import GRIP_OPEN


class E3DensityTaskController(TaskBaseController):
    """Composite controller: 整个 E3 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[e3] controller VERSION v2 "
              "(meta-actions: PickPipette→DrawPipette→TransferPipette→ReturnPipette, "
              "IK-driven, balance density readout)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest v31）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        self.meta_classes = [PickPipette, DrawPipette, TransferPipette, ReturnPipette]
        self.meta_names = [
            "S1 pick pipette (grip 0.09m above tip)",
            "S2 draw 5mL from sample bottle",
            "S3 transfer 5mL into cylinder on balance (m1->m2+rho)",
            "S4 return pipette to stand",
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
            print("[e3] all meta-actions done. success.")
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
                print(f"[e3] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1
                if self._meta_idx < len(self.meta_actions):
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

        state["language_instruction"] = self.get_language_instruction()
        action = self.inference_engine.step_inference(state)
        return action, False, self.is_success()

    def is_success(self):
        return self._meta_idx >= len(self.meta_actions)

    def get_language_instruction(self):
        return ("Use the pipette with the rubber bulb to draw 5 mL of liquid from the "
                "sample bottle, then transfer it into the graduated cylinder on the "
                "analytical balance to measure the liquid density.")
