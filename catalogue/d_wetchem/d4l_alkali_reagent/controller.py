"""D4-L 碱性试剂滴加反应控制器：整个实验 = 顺序执行元动作（两支滴管吸液滴入 → 震荡）。

与 d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册 ①SamplePass（取样滴管吸样品液→滴入试管）→ ②AlkaliPass（加碱滴管吸碱→滴入
试管，加碱混合触发现象）→ ③TubeShakePass（抓起试管震荡）顺序执行。碱瓶橡胶塞由
gen 静态倒放桌面（同 D3-L 瓶盖模式，无机械臂拔塞动作——用户 2026-08-24）。
动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用
flametest/d2s/d3l。
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
from .meta_actions import (
    SamplePass, AlkaliPass, TubeShakePass,
)
from .meta_actions.constants import GRIP_OPEN


class D4LAlkaliReagentTaskController(TaskBaseController):
    """Composite controller: 整个 D4-L 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d4l] controller VERSION v2 (Sample/Alkali droppers + tube shake, IK-driven)")
        # 引擎默认朝向 = 手指朝下（euler(0,π,0)）：正向持握。
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

        # 元动作：①SAMPLE_PASS（取样滴管吸样滴入）→ ②ALKALI_PASS（加碱滴管吸碱滴入，
        # 加碱触发现象）→ ③TUBE_SHAKE_PASS（抓起试管震荡混合），顺序执行。碱瓶橡胶塞
        # 由 gen 静态倒放桌面，无机械臂拔塞动作（用户 2026-08-24）。各滴管一次持握内循环
        # 「吸液-滴液」cfg.{sample,alkali}_cycles 遍（抓一次→多遍滴→放回一次，中途不松开；
        # 管内多积几滴液体，液面逐滴升高）。
        sample_cycles = max(1, int(getattr(cfg, "sample_cycles", 1)))
        alkali_cycles = max(1, int(getattr(cfg, "alkali_cycles", 1)))
        shake_cycles = max(1, int(getattr(cfg, "shake_cycles", 3)))
        self.meta_classes = [SamplePass, AlkaliPass, TubeShakePass]
        self.meta_names = [
            f"S sample aspirate+drip into tube x{sample_cycles}",
            f"A alkali aspirate+drip into tube x{alkali_cycles}",
            f"T shake tube x{shake_cycles}",
        ]
        self.meta_actions = [
            SamplePass(self.engine, cycles=sample_cycles),
            AlkaliPass(self.engine, cycles=alkali_cycles),
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
            print("[d4l] all meta-actions done. success.")
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
                print(f"[d4l] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Aspirate the liquid sample with the sample dropper and the alkali "
                "reagent with the alkali dropper, drip both into the same test tube "
                "in the rack, then pick up the tube and shake it to mix")
