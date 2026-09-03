"""D1 酸碱滴定 P1（加指示剂）控制器：整个动作 = 顺序执行一个元动作 IndicatorPass。

与 d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（IndicatorPass = P1 整个动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

P1 注册单个 IndicatorPass：抓滴管（捏胶头）→ 提出 → 伸进指示剂瓶液面下吸酚酞 →
移到锥形瓶（清点 W）口上挤胶头滴 3 滴（坠滴动画）→ 样液无色→粉 → 放回滴管架孔。
滴管持握的 grip_target 由元动作内 GripAction 全程管理，task 生命周期检测
attached/squeezed/filled/dropped/released 五态。
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
from .meta_actions import IndicatorPass
from .meta_actions.constants import GRIP_OPEN


class D1AcidBaseTitrationTaskController(TaskBaseController):
    """Composite controller: P1 加指示剂 = IndicatorPass 一次持握吸滴 + 放回。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d1] controller VERSION v1 (IndicatorPass 吸酚酞→滴锥形瓶→放回, IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作：① INDICATOR_PASS（抓滴管吸酚酞→滴入锥形瓶 W→放回），一次持握完成。
        # 一次挤胶头 = 成串 3 滴（task DropperDrop/Drop_0..2 动画），样液无色→粉。
        indicator_cycles = max(1, int(getattr(cfg, "indicator_cycles", 1)))
        self.meta_classes = [IndicatorPass]
        self.meta_names = [
            f"INDICATOR pass: aspirate phenolphthalein, drip into flask W, return x{indicator_cycles}",
        ]
        self.meta_actions = [
            IndicatorPass(self.engine, cycles=indicator_cycles),
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
            print("[d1] all meta-actions done. success.")
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
                print(f"[d1] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pick up the dropper by its bulb, dip it below the surface of the "
                "phenolphthalein indicator bottle, aspirate, then drip 2-3 drops into "
                "the conical flask W so the colorless NaOH solution turns pink, and "
                "return the dropper to its rack hole")
