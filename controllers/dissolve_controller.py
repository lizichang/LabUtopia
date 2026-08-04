"""蒸馏水溶解性测试控制器（D2）：4 个 phase 串联原子动作。

Phase 序列：
    1. SCOOP_POWDER  ScoopController 抓起药匙（kinematic）→ 勺头插入粉末堆
                    （dwell，勺上粉末 reveal）→ 转移进试管口（dwell，粉末入管）
                    → 药匙放回桌面（任务检测 spoon_state=released）
    2. POUR_WATER   ScoopController 抓起洗瓶 → 举到试管口上方（dwell，液面
                    reveal）→ 放回桌面（任务检测 wash_state=released）
    3. SHAKE        ShakeController 在试管口上方振荡（任务检测 dissolved）
    4. OBSERVE      无控制器：等待任务检测 obs_done（溶解后持续观察）
"""
import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.scoop_controller import ScoopController
from .atomic_actions.shake_controller import ShakeController
from .base_controller import BaseController


class Phase(Enum):
    SCOOP_POWDER = "scoop_powder"
    POUR_WATER = "pour_water"
    SHAKE = "shake"
    OBSERVE = "observe"
    FINISHED = "finished"


class DissolveTaskController(BaseController):
    """Composite controller for the solubility-in-water test."""

    def __init__(self, cfg, robot):
        """Initialize the dissolve task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.SCOOP_POWDER
        self.initial_object_z = None

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        # 1+2. 药匙 / 洗瓶（同一动作：抓取 -> 蘸取位 -> 转移位 -> 放回）
        self.scoop_controller = ScoopController(
            name="scoop_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.03, 0.004, 0.004, 0.04, 0.004, 0.004, 0.05, 0.05],
        )

        # 3. 振荡（振荡位在 __init__ 后由 task 参考点设置，见 reset()）
        self.shake_controller = ShakeController(
            name="shake_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.02, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.015],
        )

        self.active_controller = self.scoop_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.SCOOP_POWDER
        self.initial_object_z = None

        if self.mode == "collect":
            self.active_controller = self.scoop_controller
            self.scoop_controller.reset()
            self.shake_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task)."""
        if self.current_phase == Phase.SCOOP_POWDER:
            # 药匙完成舀取+转移并放回桌面（kinematic 检测）
            return self.state.get('spoon_state') == 'released'
        elif self.current_phase == Phase.POUR_WATER:
            # 洗瓶完成倒水并放回桌面（kinematic 检测）
            return self.state.get('wash_state') == 'released'
        elif self.current_phase == Phase.SHAKE:
            # 夹爪在试管口上方振荡 dwell，任务检测溶解完成
            return bool(self.state.get('dissolved'))
        elif self.current_phase == Phase.OBSERVE:
            # 溶解后持续观察 N 帧
            return bool(self.state.get('obs_done'))
        return False

    def step(self, state):
        """Execute one step of control.

        Args:
            state: Current state dictionary containing sensor data and robot state.

        Returns:
            Tuple containing action, done flag, and success flag.
        """
        self.state = state
        if self.initial_object_z is None:
            self.initial_object_z = self.state['object_position'][2]
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state):
        """Execute collection mode step."""
        success = self._check_phase_success()

        if self.current_phase == Phase.FINISHED:
            self.reset_needed = True
            return None, True, self._last_success

        # 无控制器（OBSERVE 等待环节）或控制器运动序列未完成
        controller_done = self.active_controller is None or self.active_controller.is_done()

        action = None
        if not controller_done:
            action = self._phase_action(state)

        if 'camera_data' in state:
            self.data_collector.cache_step(
                camera_images=state['camera_data'],
                joint_angles=state['joint_positions'][:-1],
                language_instruction=self.get_language_instruction(),
            )

        if not controller_done:
            return action, False, False

        # 控制器完成（或等待环节）-> 检查 phase 目标
        if success:
            return self._advance_phase(state)

        print(f"{self.current_phase.value} failed!")
        self.data_collector.clear_cache()
        self._last_success = False
        self.current_phase = Phase.FINISHED
        return None, True, False

    def _phase_action(self, state):
        """Produce the gripper action for the current phase's atomic controller."""
        if self.current_phase == Phase.SCOOP_POWDER:
            return self.scoop_controller.forward(
                grasp_position=state['spoon_grasp_position'],
                scoop_position=state['powder_scoop_position'],
                transfer_position=state['tube_transfer_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
                grasp_distance=0.006,
            )
        elif self.current_phase == Phase.POUR_WATER:
            return self.scoop_controller.forward(
                grasp_position=state['wash_grasp_position'],
                scoop_position=state['wash_grasp_position'] + np.array([0.0, 0.0, 0.10]),
                transfer_position=state['wash_pour_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
                grasp_distance=0.018,
            )
        elif self.current_phase == Phase.SHAKE:
            return self.shake_controller.forward(
                current_joint_positions=state['joint_positions'],
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )
        return None

    def _advance_phase(self, state):
        """Phase goal achieved: switch to the next phase (or finish)."""
        if self.current_phase == Phase.SCOOP_POWDER:
            print("Scoop success! Powder transferred into the tube. Pouring water...")
            self.current_phase = Phase.POUR_WATER
            self.active_controller = self.scoop_controller
            return None, False, False
        elif self.current_phase == Phase.POUR_WATER:
            print("Water poured! Shaking the tube...")
            # 振荡位 = 试管口上方（从 task 参考点）
            self.shake_controller.set_initial_position(state['shake_position'])
            self.current_phase = Phase.SHAKE
            self.active_controller = self.shake_controller
            return None, False, False
        elif self.current_phase == Phase.SHAKE:
            print("Shake success! The powder is dissolving. Observing...")
            self.current_phase = Phase.OBSERVE
            self.active_controller = None
            return None, False, False
        elif self.current_phase == Phase.OBSERVE:
            print("Observation done! Task complete.")
            self.data_collector.write_cached_data(state['joint_positions'][:-1])
            self._last_success = True
            self.current_phase = Phase.FINISHED
            return None, True, True
        return None, False, False

    def _step_infer(self, state):
        """Execute inference mode step."""
        self.state = state
        if self.current_phase == Phase.FINISHED:
            self.reset_needed = True
            return None, True, self._last_success

        language_instruction = self.get_language_instruction()
        state['language_instruction'] = language_instruction
        action = self.inference_engine.step_inference(state)

        return action, False, self.is_success()

    def is_success(self):
        """Task succeeds when the observation is done after dissolving."""
        return self.state.get('obs_done')

    def get_language_instruction(self):
        return "Scoop the sample powder into the test tube, add distilled water, shake, and observe the dissolution"
