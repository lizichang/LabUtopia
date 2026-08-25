"""D2-S 水溶性测试控制器：整个实验 = 顺序执行元动作（舀取倒入 → 加蒸馏水 → 振荡）。

v44 同构分层（与 flametest_controller 相同的 Lula IK + 元动作组合）：
  - atomic_actions/flametest/（IK 原子动作，RMP 对低 z 下探发散，弃用 RMP 用 Lula IK）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

已实现 ①PickSpatula（横向夹取药匙 → 竖直提起 → 法兰转 -45° → 对齐粉堆 x=0.537 → 竖直降 24.5cm →
往 -y 平移 16cm → 法兰 -45°→-90° 挖粉 → 抬升到管口上方 14cm → 往 +y 平移 24cm → 往 +x 平移 11cm → 法兰回卷 -90°→0° 同时往 -y 移 14cm → ⑭ 药匙放回试管架（ReturnSpatula，水平移回同时 ORIENT_FWD 调竖直→降回→松爪→撤离，2026-08-25 用户「现在加动作，把药匙放回试管架」）→
S3 夹洗瓶肚子（PickWashBottle，2026-08-25 用户「现在加动作，机械臂像加药匙的方法一样水平横着夹住wash bottle的肚子（就是能挤压的部分）」：x 偏移下探避前壁吸管 → 水平移入瓶身中心 → 横夹肚子 → 竖直提起，到此结束）。
本阶段只注册该元动作（用户 2026-08-20 先要求删掉法兰转后的旧动作，
再给出新步骤：法兰转后机械臂移动到粉堆 x 绝对位置 0.537、yz 不变、夹爪世界绝对朝向不变、
药匙 -45° 夹着 → ⑥ AlignPowderX；2026-08-22 追加 ⑦ LowerPowder：夹爪竖直向下移动 22cm
（定 22cm，2026-08-23 改回 20cm、2026-08-24 再降 3→23、再 2→25 但 25cm 会穿皿沿、用户改选 24.5cm）、
x/y 与朝向不变、只变 z，以及 ⑧ ShiftYNeg：夹爪往 -y 移动 16cm
（2026-08-22 定 15cm、08-23 改 18→23；2026-08-24 皿+粉 +Y 6.5cm 后改回 16cm，终点脱离贴底座失效区）、
x/z 与朝向严格不变；2026-08-17 曾加"水平往 -X 对齐粉末"、2026-08-20 曾试 DipToPowder 碰粉，
均已删/弃）。
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
from .meta_actions import PickSpatula, ReturnSpatula, PickWashBottle, SqueezeWater, ReturnWashBottle
from .meta_actions.constants import GRIP_OPEN


class D2SWaterSolubilityTaskController(TaskBaseController):
    """Composite controller: 整个 D2-S 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d2s] controller VERSION v1 (meta-actions: PickSpatula, IK-driven)")
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

        # 元动作：① 全流程舀粉倒入试管 → ⑭ 药匙放回试管架 → S3 夹洗瓶肚子 → S4 挤水 → S5 放回洗瓶
        self.meta_classes = [PickSpatula, ReturnSpatula, PickWashBottle, SqueezeWater, ReturnWashBottle]
        self.meta_names = ["S1 pick spatula + flange roll -45° + align powder x + lower 24.5cm + shift -y 16cm + scoop flange -45°→-90° + lift to tube mouth +14cm + shift +y 24cm + shift +x 11cm + roll flange -90°→0° + shift -y 14cm + pour powder",
                           "S2 return spatula to rack",
                           "S3 pick wash bottle belly (x-offset descent, horizontal grip, lift)",
                           "S4 squeeze wash bottle (water stream into tube)",
                           "S5 return wash bottle"]
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
            print("[d2s] all meta-actions done. success.")
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
                print(f"[d2s] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pick up the spatula from the rack horizontally "
                "(grip the handle, lift it sideways)")
