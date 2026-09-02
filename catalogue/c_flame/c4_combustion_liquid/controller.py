"""C4 燃烧试验（液体样品）控制器：整个实验 = 顺序执行元动作（滴管吸液→滴入燃烧匙
→点燃酒精灯→水平横夹燃烧匙入外焰→燃烧放回→盖帽灭火）。

与 d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册 ①DripSpoonPass（抓滴管 → 药品瓶吸液 → 燃烧匙碗口上方挤胶头滴入 → 放回）一次
持握内循环 cfg.drip_cycles 遍 → ②LightFlamePass（取火柴→触灯芯点燃酒精灯→放回）
→ ③SpoonToFlamePass（水平横夹燃烧匙杆身→碗口入外焰燃烧 4s→+y 5cm 观察 10s→放回原位）
→ ⑤CapLampPass（取灯帽→下扣盖灭火焰）。动作级契约（grip 每帧发送、到达冻结、dwell、
跨元动作 grip_target 传播）沿用 flametest/d2s/d3l。
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
from .meta_actions import DripSpoonPass, LightFlamePass, SpoonToFlamePass, CapLampPass
from .meta_actions.constants import GRIP_OPEN


class C4CombustionLiquidTaskController(TaskBaseController):
    """Composite controller: 整个 C4 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[c4] controller VERSION v5 (DripSpoonPass + LightFlamePass + SpoonToFlamePass 入外焰燃烧4s→+y5cm观察10s放回 + CapLampPass 盖帽灭火 + 燃烧/沸腾现象+火焰flicker, IK-driven)")
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
        print("[c4] Lula IK solver ready")

        # 元动作：①DRIP_SPOON_PASS（抓滴管→药品瓶吸液→燃烧匙碗滴入）→
        #         ②LIGHT_FLAME_PASS（取火柴→触灯芯点燃酒精灯→放回火柴），顺序执行。
        # ①一次持握内循环「吸液-滴液」cfg.drip_cycles 遍（抓一次→多遍滴→放回一次，
        # 中途不松开；碗液逐滴升高）；②照 B2 LightFlamePass（火柴横移灯 -X 侧，头 +X
        # 触灯芯 WICK）。task 滴管/火柴生命周期已就绪。
        drip_cycles = max(1, int(getattr(cfg, "drip_cycles", 1)))
        self.meta_classes = [DripSpoonPass, LightFlamePass, SpoonToFlamePass, CapLampPass]
        self.meta_names = [
            f"DRIP aspirate from bottle + drip into spoon x{drip_cycles}",
            "LIGHT flame with match (ignite alcohol lamp)",
            "SPOON to outer flame (grip spoon, liquid ignites & burns or boils, 4s + observe 10s, return)",
            "CAP lamp to extinguish flame",
        ]
        self.meta_actions = [
            DripSpoonPass(self.engine, cycles=drip_cycles),
            LightFlamePass(self.engine),
            SpoonToFlamePass(self.engine),
            CapLampPass(self.engine),
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
            print("[c4] all meta-actions done. success.")
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
                print(f"[c4] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Aspirate the liquid sample with the dropper from the sample bottle, "
                "drip it into the bowl of the combustion spoon resting on the "
                "test tube rack, then pick up the match and light the alcohol lamp, "
                "grip the combustion spoon by its handle and hold the bowl in the "
                "outer flame for 10 seconds so the liquid ignites and burns (or "
                "boils without burning if it is non-combustible), return the spoon "
                "to the rack, then cover the alcohol lamp with its cap to "
                "extinguish the flame")
