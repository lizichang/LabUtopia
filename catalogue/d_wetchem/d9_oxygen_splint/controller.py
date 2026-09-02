"""D9 氧气检验控制器：顺序执行 7 个元动作（带火星木条复燃）。

与 d2s/flametest/e2/d6/d7 同构（Lula IK + 元动作组合）：
  - atomic_actions/flametest/（IK 原子动作，Lula IK 驱动）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

2026-09-01 用户动作链（逐字）：「摘开酒精灯帽 → 拿火柴点燃酒精灯 → 拿木条点燃 → 快速摆动
机械臂让它熄灭（甩灭明火留余烬）→ 火星现象 → 悬停氧气试管口上方（不伸进去）→ 复燃 → 取出
归位 → 盖灯帽（熄灯）」对应 8 元动作：
  ① CapOffPass     摘灯帽：帽从灯口 → 桌面 CAP_REST
  ② IgniteLamp     火柴点燃酒精灯（照 B2 LightFlamePass）
  ③ PickSplint     夹木条（手指朝下默认朝向横夹杆身）
  ④ LightSplint    木条端伸入灯焰点燃（task 端近灯焰 → SplintFlame 显）
  ⑤ BlowOutSplint  快速摆动熄火（shake 基元，task 明火灭 → 余烬火星点显）
  ⑥ HoverSplint    火星端悬停氧气试管口上方 15mm（不伸入，task 复燃/不复燃）
  ⑦ ReturnSplint   木条取出归位
  ⑧ CapOnPass      盖灯帽：帽从桌面 → 灯口（task 帽回灯上 + 火焰熄）
全程默认朝向（手指朝下），木条/火柴水平横躺轴 +X（B2 火柴/盖帽同款，已验证）。
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
from .meta_actions import (CapOffPass, IgniteLamp, PickSplint, LightSplint,
                           BlowOutSplint, HoverSplint, ReturnSplint, CapOnPass)
from .meta_actions.constants import GRIP_OPEN


class D9OxygenSplintTaskController(TaskBaseController):
    """Composite controller: 整个 D9 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d9] controller VERSION v2 (meta-actions: CapOff/IgniteLamp/PickSplint/"
              "LightSplint/BlowOutSplint/HoverSplint/ReturnSplint/CapOn, Lula IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 d2s/flametest/e2/d6/d7）
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        self.meta_classes = [CapOffPass, IgniteLamp, PickSplint, LightSplint,
                             BlowOutSplint, HoverSplint, ReturnSplint, CapOnPass]
        self.meta_names = [
            "S1 remove the alcohol-lamp cap to the desk",
            "S2 ignite the alcohol lamp with a match",
            "S3 pick up the wooden splint",
            "S4 light the splint tip in the lamp flame",
            "S5 shake the arm to blow out the flame, leaving a glowing ember",
            "S6 hover the ember tip above the oxygen tube mouth (not inserted)",
            "S7 return the splint to the desk",
            "S8 cap the alcohol lamp to extinguish the flame",
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
            print("[d9] all meta-actions done. success.")
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
                print(f"[d9] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Remove the alcohol-lamp cap, light the lamp with a match, light a wooden "
                "splint in the flame, quickly shake the arm to blow out the flame leaving a "
                "glowing ember, hover the ember tip just above the mouth of the oxygen test "
                "tube without inserting it so the ember reignites, return the splint, then "
                "cap the lamp to extinguish the flame.")
