import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.uncap_controller import UncapController
from .atomic_actions.ignite_controller import IgniteController
from .base_controller import BaseController


class Phase(Enum):
    UNCAPPING = "uncapping"
    IGNITING = "igniting"
    FINISHED = "finished"


class IgniteLampTaskController(BaseController):
    """Composite controller for the "ignite the alcohol lamp" task.

    The task has two stages:
    1. UNCAPPING: remove the cap from the alcohol lamp and set it aside.
    2. IGNITING: move the gripper beside the exposed wick and dwell there to
       (abstractly) ignite it. The task reveals the flame when it detects the
       gripper dwelling beside the wick.

    The cap and the flame are managed by the task (IgniteLampTask); this
    controller only produces gripper actions via the two atomic controllers.
    """

    def __init__(self, cfg, robot):
        """Initialize the ignite-lamp task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.UNCAPPING

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        self.uncap_controller = UncapController(
            name="uncap_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.002, 0.002, 0.02, 0.05, 0.006, 0.006, 0.05, 0.006],
        )

        self.ignite_controller = IgniteController(
            name="ignite_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.004, 0.006],
        )

        self.active_controller = self.uncap_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.UNCAPPING

        if self.mode == "collect":
            self.active_controller = self.uncap_controller
            self.uncap_controller.reset()
            self.ignite_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task)."""
        if self.current_phase == Phase.UNCAPPING:
            # The task marks the cap as 'placed' once the gripper releases it.
            return self.state.get('cap_state') == 'placed'
        elif self.current_phase == Phase.IGNITING:
            # The task turns the flame on when the gripper dwells beside the wick.
            return bool(self.state.get('flame_on'))
        return False

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
            action = None
            if self.current_phase == Phase.UNCAPPING:
                action = self.uncap_controller.forward(
                    cap_position=state['cap_position'],
                    cap_rest_position=state['cap_rest_position'],
                    current_joint_positions=state['joint_positions'],
                    gripper_position=state['gripper_position'],
                    gripper_control=self.gripper_control,
                    end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
                )
            elif self.current_phase == Phase.IGNITING:
                action = self.ignite_controller.forward(
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

        # The active controller has finished its motion sequence.
        if success:
            if self.current_phase == Phase.UNCAPPING:
                print("Uncap success! Switching to ignite...")
                self.current_phase = Phase.IGNITING
                self.active_controller = self.ignite_controller
                return None, False, False
            elif self.current_phase == Phase.IGNITING:
                print("Ignite success! The alcohol lamp is burning.")
                self.data_collector.write_cached_data(state['joint_positions'][:-1])
                self._last_success = True
                self.current_phase = Phase.FINISHED
                return None, True, True

        # Controller finished but the phase goal was not reached -> failure.
        print(f"{self.current_phase.value} failed!")
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
        """Task succeeds when the cap is off and the flame is on."""
        return bool(self.state.get('flame_on')) and self.state.get('cap_state') == 'placed'

    def get_language_instruction(self):
        return "Remove the cap from the alcohol lamp and ignite the wick"
