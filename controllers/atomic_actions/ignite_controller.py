from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class IgniteController(BaseController):
    """A state machine controller for the abstract "ignition" of an alcohol lamp.

    The ignition is abstract: the gripper moves to a point beside the exposed wick,
    dwells there for a moment (the "ignition" gesture), and the task reveals the flame.
    No physical fire-starting mechanism is modeled.

    Phases:
    - Phase 0: Move end effector to an approach point (offset away from the wick).
    - Phase 1: Move end effector to the ignition point (beside the wick tip).
    - Phase 2: Dwell at the ignition point (the abstract ignition gesture). While the
      gripper dwells here, the task detects it and reveals the flame.
    - Phase 3: Retract the end effector away from the lamp.
    - Phase 4: Complete the sequence.

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller returning ArticulationAction.
        events_dt (List[float], optional): Duration for each phase. Must have 5 elements.
            events_dt[2] controls the dwell length (smaller = longer dwell).
        position_threshold (float): Distance threshold for position-based phase transitions.
    """

    def __init__(
        self,
        name: str,
        cspace_controller: BaseController,
        events_dt: typing.Optional[typing.List[float]] = None,
        position_threshold: float = 0.01,
    ) -> None:
        super().__init__(name=name)
        self._event = 0
        self._t = 0

        if events_dt is None:
            # Phase 2 (dwell) = 0.02 -> ~50 steps at 60 Hz, long enough for the
            # task to detect the gripper dwelling and reveal the flame.
            self._events_dt = [0.004, 0.004, 0.02, 0.004, 0.006]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 5:
                raise Exception(f"events_dt length must be 5, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        ignite_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        approach_offset: typing.Optional[np.ndarray] = None,
        retract_offset: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Computes the joint positions for the current ignition phase.

        Args:
            ignite_position (np.ndarray): World position of the ignition point
                (beside the wick tip) where the gripper dwells to "ignite".
            current_joint_positions (np.ndarray): Current joint positions of the robot.
            gripper_position (np.ndarray): Current position of the gripper (TCP).
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation (quat).
                Defaults to pointing straight down.
            approach_offset (np.ndarray, optional): Offset from ignite_position for the
                approach point. Defaults to [0.05, 0, 0.05].
            retract_offset (np.ndarray, optional): Offset from ignite_position for the
                retract point. Defaults to [0.07, 0, 0.07].

        Returns:
            ArticulationAction: Joint positions for the robot to execute.
        """
        if self._start:
            self._start = False
            # Keep the gripper open during ignition.
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.04 / get_stage_units()
            target_joint_positions[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
        if approach_offset is None:
            approach_offset = np.array([0.05, 0.0, 0.05])
        if retract_offset is None:
            retract_offset = np.array([0.07, 0.0, 0.07])

        target_joint_positions = self._execute_phase(
            ignite_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            approach_offset,
            retract_offset,
        )

        if self._event < len(self._events_dt):
            self._t += self._events_dt[self._event]
            if self._t >= 1.0:
                self._event += 1
                self._t = 0

        return target_joint_positions

    def _execute_phase(
        self,
        ignite_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        approach_offset,
        retract_offset,
    ):
        """Executes the current phase of the ignition sequence."""
        if self._event == 0:
            # Move to the approach point (offset away from the wick).
            target_position = ignite_position + approach_offset
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            xy_distance = np.linalg.norm(gripper_position[:2] - target_position[:2])
            if xy_distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 1:
            # Move to the ignition point (beside the wick tip).
            target_position = ignite_position.copy()
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 2:
            # Dwell at the ignition point (the abstract ignition gesture).
            # Hold position; the task reveals the flame while we dwell here.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=ignite_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 3:
            # Retract away from the lamp.
            target_position = ignite_position + retract_offset
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        else:
            # Done.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

    def get_phase(self) -> int:
        """Returns the current phase index (0-4)."""
        return self._event

    def reset(
        self,
        events_dt: typing.Optional[typing.List[float]] = None,
    ) -> None:
        """Resets the controller to the initial phase."""
        super().reset()
        self._cspace_controller.reset()
        self._event = 0
        self._t = 0
        self._start = True

        if events_dt is not None:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 5:
                raise Exception(f"events_dt length must be 5, got {len(self._events_dt)}")

    def is_done(self) -> bool:
        """Checks if the ignition sequence is complete."""
        return self._event >= len(self._events_dt)
