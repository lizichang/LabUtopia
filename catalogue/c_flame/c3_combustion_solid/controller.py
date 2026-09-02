"""C3 燃烧试验（固体样品）控制器：顺序执行元动作（挖粉→倒粉→放回药匙→点火→入外焰）。

v44 同构分层（与 d2s/flametest 相同的 Lula IK + 元动作组合）：
  - atomic_actions/flametest/（IK 原子动作，RMP 对低 z 下探发散，弃用 RMP 用 Lula IK）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册 ①PickSpatula（横夹药匙 → 挖粉 → 倒燃烧匙前段 ⑪⑫⑬⑭ 同步倒粉）→
②ReturnSpatula（倒粉后原位提 H 调直 → 高位移回架孔 → 竖直下探入孔 → 松爪放回）→
③LightFlamePass（取火柴 → 高位运移 → 下探触灯芯点燃酒精灯 → 抬离绕焰 → 放回）→
④SpoonToFlamePass（横夹燃烧匙把手 → 碗口入外焰停留 → 放回原位）→
⑤CapLampPass（取灯帽 → 高位运到灯口上方 → 下扣盖灭火焰）。动作级契约
（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest/d2s。
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
from .meta_actions import (PickSpatula, ReturnSpatula, LightFlamePass,
                           SpoonToFlamePass, CapLampPass)
from .meta_actions.constants import GRIP_OPEN


class C3CombustionSolidTaskController(TaskBaseController):
    """Composite controller: C3 燃烧试验 = 元动作的顺序执行（本阶段 = 挖粉）。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[c3] controller VERSION v2 (PickSpatula 挖粉倒粉 + ReturnSpatula + "
              "LightFlamePass 点火 + SpoonToFlamePass 入外焰, IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest v31 / d2s）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作：① 横夹药匙挖粉 → ⑪⑫⑬⑭ 倒燃烧匙（同步倒粉+粉末下落）→
        #         ② 放回药匙（原位提 H 调直 → 高位移回架孔 → 下探入孔 → 松爪）→
        #         ③ 火柴点燃酒精灯 → ④ 燃烧匙入外焰（停留→放回）。顺序执行。
        self.meta_classes = [PickSpatula, ReturnSpatula, LightFlamePass,
                             SpoonToFlamePass, CapLampPass]
        self.meta_names = [
            "S1 pick spatula + flange roll -45° + align powder x=0.537 "
            "+ lower 24.5cm + shift -y 16cm + scoop flange -45°→-90° "
            "+ lift to safe height + down 17cm + shift +y 31cm + shift +x 5cm "
            "+ sync roll vertical + shift -y 18cm (pour)",
            "S2 return spatula to rack (lift to H + align + lower into hole + release)",
            "S3 light alcohol lamp with match (carry high 0.96 + descend to wick + lift away)",
            "S4 grip combustion spoon handle + move bowl into outer flame (dwell) + return",
            "S5 pick lamp cap + carry over lamp mouth + cover down to extinguish flame",
        ]
        self.meta_actions = [
            PickSpatula(self.engine),
            ReturnSpatula(self.engine),
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
            print("[c3] all meta-actions done. success.")
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
                print(f"[c3] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pick up the spatula from the rack, scoop up the solid sample powder "
                "from the dish, pour it into the bowl of the combustion spoon, return "
                "the spatula to the rack, then pick up the match and light the alcohol "
                "lamp, grip the combustion spoon by its handle and move its bowl into "
                "the outer flame so the sample heats, then return the spoon, and finally "
                "pick up the lamp cap and cover the lamp mouth to extinguish the flame")
