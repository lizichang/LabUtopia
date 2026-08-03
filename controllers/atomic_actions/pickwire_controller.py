"""PickWireController：夹取铂丝并移到本生灯口停留（焰色反应的机械臂动作）。

铂丝本身由任务（FlameTestTask）做 kinematic 跟随：夹爪靠近手柄并闭合后，
任务让铂丝镜像夹爪的运动。本控制器只产生夹爪运动序列。
"""
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class PickWireController(BaseController):
    """State machine controller for grasping the platinum wire and holding it
    at the burner mouth.

    Phases:
    - Phase 0: Move above the wire handle (pre-grasp).
    - Phase 1: Lower to the handle grasp position.
    - Phase 2: Wait for dynamics to settle.
    - Phase 3: Close the gripper (the task detects the grasp and attaches the wire).
    - Phase 4: Lift the wire straight up.
    - Phase 5: Move to the ignition point beside the burner mouth.
    - Phase 6: Dwell at the ignition point (the task reveals a colored flame).
    - Phase 7: Retract away from the burner.
    - Phase 8: Complete.

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller.
        events_dt (List[float], optional): Duration for each phase (9 elements).
        position_threshold (float): Distance threshold for phase transitions.
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
            self._events_dt = [0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.03, 0.004, 0.006]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 9:
                raise Exception(f"events_dt length must be 9, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        wire_grasp_position: np.ndarray,
        ignite_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.08,
        lift_z: float = 0.07,
        retract_offset: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Computes the joint positions for the current phase.

        Args:
            wire_grasp_position (np.ndarray): World position of the wire handle
                (where the gripper closes on the wire).
            ignite_position (np.ndarray): World position where the gripper dwells
                to hold the wire tip in the flame (the abstract ignition point).
            current_joint_positions (np.ndarray): Current joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above the handle for the pre-grasp point.
            lift_z (float): Height to lift the wire above its grasp position.
            retract_offset (np.ndarray, optional): Offset for the retract point.

        Returns:
            ArticulationAction: Joint positions for the robot to execute.
        """
        if self._start:
            self._start = False
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.04 / get_stage_units()
            target_joint_positions[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
        if retract_offset is None:
            retract_offset = np.array([0.05, 0.0, 0.05])

        target_joint_positions = self._execute_phase(
            wire_grasp_position,
            ignite_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            pre_offset_z,
            lift_z,
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
        wire_grasp_position,
        ignite_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
        retract_offset,
    ):
        """Executes the current phase of the pick-wire sequence."""
        if self._event == 0:
            # Move to the pre-grasp point (above the handle).
            target_position = wire_grasp_position + np.array([0.0, 0.0, pre_offset_z])
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
            # Lower to the handle grasp position.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=wire_grasp_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - wire_grasp_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 2:
            # Wait for dynamics to settle.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=wire_grasp_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 3:
            # Close the gripper (the task attaches the wire on detection).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.03 / get_stage_units()
            target_joint_positions[8] = 0.03 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 4:
            # Lift the wire straight up.
            target_position = wire_grasp_position + np.array([0.0, 0.0, lift_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 5:
            # Move to the ignition point beside the burner mouth.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=ignite_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - ignite_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 6:
            # Dwell at the ignition point; the task reveals the colored flame.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=ignite_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 7:
            # Retract away from the burner.
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
        """Returns the current phase index (0-8)."""
        return self._event

    def reset(self) -> None:
        """Resets the controller to the initial phase."""
        super().reset()
        self._cspace_controller.reset()
        self._event = 0
        self._t = 0
        self._start = True

    def is_done(self) -> bool:
        """Checks if the pick-wire sequence is complete."""
        return self._event >= len(self._events_dt)
