"""焰色反应控制器：夹取铂丝 → 移到本生灯口停留（点火）→ 移开。

动作序列（PickWireController，9 段）：
    Phase 0: 移动到铂丝手柄上方（pre-grasp）
    Phase 1: 下降到手柄抓取位
    Phase 2: 等待动力学稳定
    Phase 3: 闭合夹爪（任务检测到"attached"后铂丝跟随）
    Phase 4: 上提铂丝
    Phase 5: 横移到本生灯口点火位
    Phase 6: 在点火位停留（任务检测 dwell 后 reveal 火焰并变色）
    Phase 7: 后撤离开灯口
    Phase 8: 完成
"""
import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.pickwire_controller import PickWireController
from .base_controller import BaseController


class Phase(Enum):
    PICKWIRE = "pickwire"
    FINISHED = "finished"


class FlameTestTaskController(BaseController):
    """Composite controller for the flame test task.

    The wire and the flame are managed by the task (FlameTestTask); this
    controller produces the gripper motion via the PickWireController.
    """

    def __init__(self, cfg, robot):
        """Initialize the flame test task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.PICKWIRE

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        self.pickwire_controller = PickWireController(
            name="pickwire_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.03, 0.004, 0.006],
        )

        self.active_controller = self.pickwire_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.PICKWIRE

        if self.mode == "collect":
            self.active_controller = self.pickwire_controller
            self.pickwire_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task)."""
        # 火焰被任务点燃（变色 reveal）即代表焰色反应完成
        return bool(self.state.get('flame_on'))

    def step(self, state):
        """Execute one step of control.

        Args:
            state: Current state dictionary containing sensor data and robot state.

        Returns:
            Tuple containing action, done flag, and success flag.
        """
        self.state = state
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

        if not self.active_controller.is_done():
            action = self.pickwire_controller.forward(
                wire_grasp_position=state['wire_grasp_position'],
                ignite_position=state['ignite_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )

            if 'camera_data' in state:
                self.data_collector.cache_step(
                    camera_images=state['camera_data'],
                    joint_angles=state['joint_positions'][:-1],
                    language_instruction=self.get_language_instruction(),
                )

            return action, False, False

        # The pickwire sequence has finished.
        if success:
            print(f"Flame test success! Flame color: {self.state.get('flame_color', 'unknown')}")
            self.data_collector.write_cached_data(state['joint_positions'][:-1])
            self._last_success = True
            self.current_phase = Phase.FINISHED
            return None, True, True

        print("Flame test failed!")
        self.data_collector.clear_cache()
        self._last_success = False
        self.current_phase = Phase.FINISHED
        return None, True, False

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
        """Task succeeds when the flame is on (the flame test has been performed)."""
        return bool(self.state.get('flame_on'))

    def get_language_instruction(self):
        return "Pick up the platinum wire and hold its tip in the bunsen burner flame"
