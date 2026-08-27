"""B1 酒精灯加热（固体样品）控制器：本批次 = 三个元动作顺序执行（用户指定先只写这三个）。

用户 2026-08-27 逐字：「先写咬粉末咬进试管里面，然后拿起酒精灯盖儿放到一边儿，再拿起火柴点燃
酒精灯，这几个过程先只写这些我来验收。」→ 本控制器只编排这三段；拿试管→外焰预热（倾斜 10-15°）
→集中加热→熄灭→归位留待验收后接续。

与 d2s/d3s 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册顺序：
  ①PickSpatula（挖粉倒粉，home=None → d2s SPAT_XY，B1 场景复刻 D2S 坐标逐字）
  ②ReturnSpatula（药匙放回，home=None → d2s SPAT_XY）
  ③OpenCapPass（拿起酒精灯盖放到一边：取帽 → 提起 → 横移 → 落台面 → 松爪归位）
  ④LightFlamePass（取火柴点燃酒精灯：抓杆 → 触灯芯 → 点燃 → 放回火柴；B1 无温度模型，
    flame_lit 置位即 reveal 火焰）

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest/d2s。
跨元动作 grip_target 传播：①② 之间（药匙放回后爪子张开）与 ③④ 之间（灯帽释放后爪子张开）
由各元动作末尾 GripAction 置 GRIP_OPEN，持握状态不出单个元动作，无需额外传播。
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
from .meta_actions import PickSpatula, ReturnSpatula, OpenCapPass, LightFlamePass
from .meta_actions.constants import GRIP_OPEN


class B1AlcoholHeatSolidTaskController(TaskBaseController):
    """Composite controller: B1 本批次 = 挖粉倒粉 → 放回药匙 → 开灯帽 → 点火四个元动作顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b1] controller VERSION v1 (meta-actions: PickSpatula->ReturnSpatula->OpenCapPass->LightFlamePass, IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3s）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作：
        # ①PickSpatula（挖粉倒粉）→ ②ReturnSpatula（药匙放回）→ ③OpenCapPass（开灯帽放一边）
        # → ④LightFlamePass（取火柴点燃）。挖粉用 d2s 默认 home（None → d2s SPAT_XY (0.6993,
        # 0.3608)）：用户先决条件「表面皿、粉末和机械臂坐标一定要复刻 D2S，粉末才挖得准」，
        # B1 场景已逐字复刻 D2S，故不传 home/参数与 d2s 行为完全一致。
        self.meta_classes = [PickSpatula, ReturnSpatula, OpenCapPass, LightFlamePass]
        self.meta_names = [
            "P1 pick spatula + scoop powder + pour into tube (d2s, coords unchanged)",
            "P2 return spatula to rack (d2s)",
            "C open alcohol lamp cap and set it aside (pure-translation cap hold)",
            "L pick up match and light the alcohol lamp (match tip to wick, direct flame reveal)",
        ]
        self.meta_actions = [
            PickSpatula(self.engine),     # home=None → d2s SPAT_XY（B1 复刻 D2S 坐标）
            ReturnSpatula(self.engine),   # home=None → d2s SPAT_XY
            OpenCapPass(self.engine),
            LightFlamePass(self.engine),
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
            print("[b1] all meta-actions done. success.")
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
                print(f"[b1] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Scoop the solid powder sample with the spatula and pour it into the "
                "test tube in the rack, return the spatula, open the alcohol lamp cap "
                "and set it aside, then pick up the match and light the alcohol lamp")
