import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.dropper_controller import DropperController
from .base_controller import BaseController


class Phase(Enum):
    PICK_DROPPER = "pick_dropper"
    FILL_DROPPER = "fill_dropper"
    DRIP = "drip"
    FINISHED = "finished"


class DropperDripTaskController(BaseController):
    """Composite controller for the "drip liquid from a dropper" task.

    The task has three stages:
    1. PICK_DROPPER: grasp the standing dropper by its bulb (top part; the
       task attaches it kinematically once the gripper closes near the grasp
       point).
    2. FILL_DROPPER: move to the reagent bottle mouth, squeeze the bulb to
       expel air, dip the tip into the liquid, release the bulb to aspirate.
    3. DRIP: move to the test tube mouth and squeeze the bulb to drip.

    The dropper state machine is owned by the task (DropperDripTask); this
    controller only produces gripper actions via the DropperController and
    reads dropper_state from the state dict to decide phase success.
    """

    def __init__(self, cfg, robot):
        """Initialize the dropper-drip task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.PICK_DROPPER

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        self.dropper_controller = DropperController(
            name="dropper_controller",
            cspace_controller=self.rmp_controller,
        )

        self.active_controller = self.dropper_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.PICK_DROPPER

        if self.mode == "collect":
            self.active_controller = self.dropper_controller
            self.dropper_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task).

        The task exposes sticky per-phase flags ("picked"/"filled"/"dropped").
        The atomic controller runs the whole pick->fill->drip sequence in one
        pass, so by the time it reports done the current dropper_state has
        already advanced to "dropped" — the flags record whether each stage
        actually happened during the pass.
        """
        if self.current_phase == Phase.PICK_DROPPER:
            return self.state.get('picked')
        elif self.current_phase == Phase.FILL_DROPPER:
            return self.state.get('filled')
        elif self.current_phase == Phase.DRIP:
            return self.state.get('dropped')
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
            action = self.dropper_controller.forward(
                grasp_position=state['grasp_position'],
                dip_position=state['dip_position'],
                target_position=state['target_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 180, 0])).as_quat(),
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
            if self.current_phase == Phase.PICK_DROPPER:
                print("Dropper attached! Switching to fill...")
                self.current_phase = Phase.FILL_DROPPER
                return None, False, False
            elif self.current_phase == Phase.FILL_DROPPER:
                print("Dropper filled! Switching to drip...")
                self.current_phase = Phase.DRIP
                return None, False, False
            elif self.current_phase == Phase.DRIP:
                print("Drip success! Liquid dropped into the tube.")
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
        """Task succeeds when the dropper has dropped the liquid."""
        return self.state.get('dropper_state') == 'dropped'

    def get_language_instruction(self):
        return "Aspirate liquid from the reagent bottle with the dropper and drip it into the test tube"
