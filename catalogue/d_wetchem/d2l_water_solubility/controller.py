"""D2-L 液体样品水溶性测试控制器：整个实验 = 顺序执行元动作。

与 d4l/d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用
IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

第一步（v1）：仅 SAMPLE_PASS（取样滴管吸样品液→滴入试管，cfg.sample_cycles 遍）。
后续步骤（v2）：WashBottlePass（洗瓶注水）→ TubeShakePass（拿起试管震荡）+ 现象
三档（cfg.mixing），顺序追加到 meta_actions。

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用
flametest/d2s/d3l/d4l。
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
from .meta_actions import (SamplePass, PickWashBottle, SqueezeWater,
                           ReturnWashBottle, TubeShakePass)
from .meta_actions.constants import GRIP_OPEN


class D2LWaterSolubilityTaskController(TaskBaseController):
    """Composite controller: 整个 D2-L 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d2l] controller VERSION v1 (sample dropper aspirate+drip, IK-driven)")
        # 引擎默认朝向 = 手指朝下（euler(0,π,0)）：正向持握。
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l/d4l）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作（整个 D2-L 实验，顺序执行）：
        #   ① SAMPLE_PASS      取样滴管吸样品→滴入试管（一次持握内循环「吸液-滴液」
        #                       cfg.sample_cycles 遍，中途不松开）
        #   ② PICK_WASH_BOTTLE 抓洗瓶（手指朝前 ORIENT_FWD 横夹肚子，红嘴朝 +X）
        #      SQUEEZE_WATER    挤水（夹爪进一步合拢挤瓶身，水流从红嘴入试管）
        #      RETURN_WASH_BOTTLE 放回洗瓶（逆抓取轨迹归位松爪）
        #   ③ TUBE_SHAKE_PASS  拿起试管震荡来回 cfg.shake_cycles 下→放回（现象三档分化）
        sample_cycles = max(1, int(getattr(cfg, "sample_cycles", 1)))
        shake_cycles = max(1, int(getattr(cfg, "shake_cycles", 5)))
        self.meta_classes = [SamplePass, PickWashBottle, SqueezeWater,
                             ReturnWashBottle, TubeShakePass]
        self.meta_names = [
            f"S sample aspirate+drip into tube x{sample_cycles}",
            "W pick wash bottle",
            "W squeeze water into tube",
            "W return wash bottle",
            f"T shake tube x{shake_cycles}",
        ]
        self.meta_actions = [
            SamplePass(self.engine, cycles=sample_cycles),
            PickWashBottle(self.engine),
            SqueezeWater(self.engine),
            ReturnWashBottle(self.engine),
            TubeShakePass(self.engine, cycles=shake_cycles),
        ]
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
            print("[d2l] all meta-actions done. success.")
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
                print(f"[d2l] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Aspirate the liquid sample with the sample dropper, drip it into the "
                "test tube in the rack, squeeze distilled water from the wash bottle "
                "into the tube, then pick up and shake the tube")
