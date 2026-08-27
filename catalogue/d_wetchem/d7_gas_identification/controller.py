"""D7 气体鉴定控制器：顺序执行 3 个元动作（橡皮塞预装，机械臂不夹取/拔塞）。

与 d2s/flametest/e2/d6 同构（Lula IK + 元动作组合）：
  - atomic_actions/flametest/（IK 原子动作，Lula IK 驱动）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

2026-08-27 用户方案（检测试剂统一为液体 + 单输入入口 + ③④ 合并 + 摆装饰瓶）；
**2026-08-27 二改：跳过夹取塞子（①PickStopper）与拔塞（⑤RemoveStopper），橡皮塞预装塞紧
产气试管口 → 仅 3 个元动作**：
  ① DipGasTube    取检验试管 → 移到下浸孔下放使末端浸入液面下 15mm
  ② HoldDetect    保持 2.5s 通气观察（task 驱动气泡上升；③④ 合并）
  ③ ReturnTube    检验试管归位
试管全程侧面横夹（手指朝前 ORIENT_FWD），避导气管竖段/试管架顶板穿模（2026-08-27 用户）。
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
from .meta_actions import DipGasTube, HoldDetect, ReturnTube
from .meta_actions.constants import GRIP_OPEN


class D7GasIdentificationTaskController(TaskBaseController):
    """Composite controller: 整个 D7 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d7] controller VERSION v2 (meta-actions: DipGasTube/HoldDetect/ReturnTube, "
              "stopper pre-plugged, Lula IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 d2s/flametest/e2/d6）
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        self.meta_classes = [DipGasTube, HoldDetect, ReturnTube]
        self.meta_names = [
            "S1 pick the detection tube and lower it so the tube end submerges 15mm in liquid",
            "S2 hold 2.5s for gas to bubble through the detection liquid",
            "S3 return the detection tube to the rack",
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
            print("[d7] all meta-actions done. success.")
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
                print(f"[d7] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pick up the detection tube and move it forward so the delivery-tube end submerges "
                "about 1 cm into the liquid, hold for 2.5 seconds to let the gas bubble through the "
                "liquid, and finally return the detection tube to the rack.")
