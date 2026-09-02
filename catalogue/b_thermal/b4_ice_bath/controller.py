"""B4 冰浴/冷却 —— 控制器：整个实验 = 顺序执行元动作。

2026-08-30 用户逐字：「现在加动作，机械臂往里面挤入液体要真实（可参考a3），然后
烧杯里面的冰块浮起来（符合物理现象），然后原位放回洗瓶。然后竖直提取试管（参考d3l）并浸入冰水」。

元动作链（一个用户步骤 = 一个元动作，一类一文件，v44 同构分层同 d2s）：
  ① PickWashBottle      水平横夹洗瓶肚子 → 竖直提起 15cm → 向 +X 移 15cm（红嘴尖到烧杯口上方）
  ② SqueezeWashBottle   挤压洗瓶身出水（夹爪 0.030→0.020→0.030；task 检测开度发水流 +
                         烧杯内液面上涨 + 冰块浮起，a3 同款）
  ③ ReturnWashBottle    把挤完水的洗瓶原位放回（水平移回 → 竖直下探 → 开爪松放 → 抬回）
  ④ PickTube            竖直提取试管（手指朝下，d3l 同款）→ 提出架顶
  ⑤ ImmerseTube         试管移到烧杯上方 → 竖直下探浸入冰水（冷却 15s）→ 提出看现象停留 8s
  ⑥ ReturnTube          把冰浴后的试管放回架孔（水平移回 → 竖直下探 → 开爪松放 → 抬回）

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest。
持握状态跨元动作：洗瓶在 ②③ 连续持握 → 每个元动作末尾 grip_target 传播到下一个
（③ 无 GripAction 时首帧不开爪，瓶子吸附不挂空，坑 28）。
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
from .meta_actions import (PickWashBottle, SqueezeWashBottle,
                           ReturnWashBottle, PickTube, ImmerseTube, ReturnTube)
from .meta_actions.constants import GRIP_OPEN


class B4IceBathTaskController(TaskBaseController):
    """Composite controller: 整个 B4 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b4] controller VERSION v4 (meta-actions: pick-wash→squeeze→return→pick-tube→immerse→return-tube, IK-driven)")
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

        # 元动作链：① 夹洗瓶提起移 +X → ② 挤水入烧杯 → ③ 放回洗瓶 → ④ 水平夹试管提出
        self.meta_classes = [PickWashBottle, SqueezeWashBottle, ReturnWashBottle, PickTube,
                             ImmerseTube, ReturnTube]
        self.meta_names = [
            "S1 pick wash bottle (x-offset descent, horizontal grip, lift 15cm, +X 15cm)",
            "S2 squeeze wash bottle into beaker (grip 0.030->0.020->0.030, water stream + liquid rise + ice float)",
            "S3 return wash bottle to original position (back + down + open + retreat)",
            "S4 pick test tube vertically (finger-down, d3l-style) and lift out of rack",
            "S5 move test tube above beaker and immerse into ice water (cool 15s, then lift out, observe 8s)",
            "S6 return test tube to rack (back + down + open + retreat)",
        ]
        self.meta_actions = [cls(self.engine) for cls in self.meta_classes]
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
            print("[b4] all meta-actions done. success.")
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
                print(f"[b4] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1
                if self._meta_idx < len(self.meta_actions):
                    # 持握状态跨元动作传播（洗瓶在 ②③ 连续持握，坑 28）：
                    # 下一个元动作继承上一个的夹爪开度，无 GripAction 的段不开爪
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
        return ("Pick up the wash bottle horizontally and move it +X, squeeze water into "
                "the beaker (ice cubes float up), return the wash bottle to its original "
                "position, then vertically pick up the test tube, move it above the beaker, "
                "immerse it into the ice water, observe it for 5 seconds, and return it to "
                "the rack.")
