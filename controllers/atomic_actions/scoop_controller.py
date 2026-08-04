from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class ScoopController(BaseController):
    """A state machine controller for scooping powder and transferring it.

    The object (spoon / wash bottle) is driven kinematically by the task: the
    task detects "gripper near the grasp point AND gripper closed" and then
    mirrors the gripper motion onto the object prim. This controller only
    produces the gripper motion; it walks through these phases:

    - Phase 0: Move end effector above the grasp point (pre-grasp).
    - Phase 1: Lower down to the grasp point.
    - Phase 2: Wait for the dynamics to settle.
    - Phase 3: Close the gripper (the task detects "attached" here).
    - Phase 4: Lift straight up.
    - Phase 5: Move to the scoop position (powder heap / lift-off point).
    - Phase 6: Dwell at the scoop position (the dipping gesture; the task
      reveals the powder on the spoon here).
    - Phase 7: Move above the transfer position (tube mouth).
    - Phase 8: Lower into the transfer position (the task detects the powder
      transfer / water pour here).
    - Phase 9: Dwell at the transfer position.
    - Phase 10: Open the gripper and lift away (the task settles the object).

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller returning ArticulationAction.
        events_dt (List[float], optional): Duration for each phase. Must have 11 elements.
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
            self._events_dt = [0.004, 0.004, 0.02, 0.03, 0.004, 0.004, 0.04, 0.004, 0.004, 0.05, 0.05]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 11:
                raise Exception(f"events_dt length must be 11, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        grasp_position: np.ndarray,
        scoop_position: np.ndarray,
        transfer_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.12,
        lift_z: float = 0.10,
        grasp_distance: float = 0.006,
        retract_offset: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Computes the joint positions for the current scoop phase.

        Args:
            grasp_position (np.ndarray): World position of the grasp point
                (spoon handle center / wash bottle neck).
            scoop_position (np.ndarray): World position where the gripper holds
                the spoon head into the powder heap (or the lift-off point for
                the wash bottle).
            transfer_position (np.ndarray): World position where the gripper
                holds the spoon head into the tube mouth (or above the tube
                mouth for the wash bottle).
            current_joint_positions (np.ndarray): Current robot joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above grasp_position for the approach.
            lift_z (float): Height to lift after grasping.
            grasp_distance (float): Gripper finger distance to grasp the object.
            retract_offset (np.ndarray, optional): Offset for the final retract.

        Returns:
            ArticulationAction: Joint positions for the robot to execute.
        """
        if self._start:
            return self._handle_start_state(current_joint_positions)

        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
        if retract_offset is None:
            retract_offset = np.array([0.05, 0.0, 0.10])

        target_joint_positions = self._execute_phase(
            grasp_position,
            scoop_position,
            transfer_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            pre_offset_z,
            lift_z,
            grasp_distance,
            retract_offset,
        )

        if self._event < len(self._events_dt):
            self._t += self._events_dt[self._event]
            if self._t >= 1.0:
                self._event += 1
                self._t = 0

        return target_joint_positions

    def _handle_start_state(self, current_joint_positions):
        """Opens the gripper before starting the scoop sequence."""
        self._start = False
        target_joint_positions = [None] * current_joint_positions.shape[0]
        target_joint_positions[7] = 0.04 / get_stage_units()
        target_joint_positions[8] = 0.04 / get_stage_units()
        return ArticulationAction(joint_positions=target_joint_positions)

    def _execute_phase(
        self,
        grasp_position,
        scoop_position,
        transfer_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
        grasp_distance,
        retract_offset,
    ):
        """Executes the current phase of the scoop sequence."""
        if self._event == 0:
            # Move directly above the grasp point.
            target_position = grasp_position + np.array([0.0, 0.0, pre_offset_z])
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
            # Lower down to the grasp point.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=grasp_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - grasp_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 2:
            # Wait for dynamics to settle.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

        elif self._event == 3:
            # Close the gripper (the task detects "attached" while closed).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = grasp_distance / get_stage_units()
            target_joint_positions[8] = grasp_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 4:
            # Lift straight up (the object follows kinematically).
            target_position = grasp_position.copy()
            target_position[2] += lift_z
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 5:
            # Move to the scoop position (spoon head into the powder heap).
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=scoop_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - scoop_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 6:
            # Dwell at the scoop position (the scooping gesture).
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=scoop_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 7:
            # Move above the transfer position.
            target_position = transfer_position + np.array([0.0, 0.0, pre_offset_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            xy_distance = np.linalg.norm(gripper_position[:2] - target_position[:2])
            if xy_distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 8:
            # Lower into the transfer position (spoon head into the tube mouth).
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=transfer_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - transfer_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 9:
            # Dwell at the transfer position (the transfer / pour gesture).
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=transfer_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        else:
            # Open the gripper and lift away (the task settles the object).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.04 / get_stage_units()
            target_joint_positions[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

    def get_phase(self) -> int:
        """Returns the current phase index (0-10)."""
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
            if len(self._events_dt) != 11:
                raise Exception(f"events_dt length must be 11, got {len(self._events_dt)}")

    def is_done(self) -> bool:
        """Checks if the scoop sequence is complete."""
        return self._event >= len(self._events_dt)
