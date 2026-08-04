"""DipController：蘸取动作（焰色反应用）。

铂丝已由任务吸附在夹爪上（kinematic 跟随），本控制器只产生夹爪运动：
夹爪把铂丝 loop 端降到目标物（盐酸瓶口 / 样品皿粉末堆）上方 → 浸入停留 →
上提 → 移开。

Phases:
- Phase 0: 移到目标位上方（pre-dip）
- Phase 1: 下降到蘸取位（铂丝 loop 触到目标物）
- Phase 2: 停留（蘸取 gesture）
- Phase 3: 上提
- Phase 4: 横移移开（远离目标物）
- Phase 5: 完成
"""
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class DipController(BaseController):
    """State machine controller for the dipping gesture (flame test).

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller.
        events_dt (List[float], optional): Duration for each phase (6 elements).
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
            self._events_dt = [0.004, 0.004, 0.03, 0.004, 0.004, 0.006]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 6:
                raise Exception(f"events_dt length must be 6, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        dip_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.12,
        lift_z: float = 0.10,
        retract_offset: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Computes the joint positions for the current dip phase.

        Args:
            dip_position (np.ndarray): World position where the gripper holds the
                wire loop into the target (already accounts for the wire geometry).
            current_joint_positions (np.ndarray): Current joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above dip_position for the approach point.
            lift_z (float): Height to lift the wire after dipping.
            retract_offset (np.ndarray, optional): Offset for the retract point.

        Returns:
            ArticulationAction: Joint positions for the robot to execute.
        """
        if self._start:
            self._start = False
            # Keep the gripper closed (the wire stays attached) during dipping.
            target_joint_positions = [None] * current_joint_positions.shape[0]
            return ArticulationAction(joint_positions=target_joint_positions)

        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
        if retract_offset is None:
            retract_offset = np.array([0.05, 0.0, 0.10])

        target_joint_positions = self._execute_phase(
            dip_position,
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
        dip_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
        retract_offset,
    ):
        """Executes the current phase of the dip sequence."""
        if self._event == 0:
            # Move above the dip position.
            target_position = dip_position + np.array([0.0, 0.0, pre_offset_z])
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
            # Lower so the wire loop dips into the target.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=dip_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - dip_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 2:
            # Dwell at the dip position (the dipping gesture).
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=dip_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 3:
            # Lift the wire straight up.
            target_position = dip_position + np.array([0.0, 0.0, lift_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 4:
            # Retract away from the target.
            target_position = dip_position + retract_offset
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        else:
            # Done.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

    def get_phase(self) -> int:
        """Returns the current phase index (0-5)."""
        return self._event

    def reset(self) -> None:
        """Resets the controller to the initial phase."""
        super().reset()
        self._cspace_controller.reset()
        self._event = 0
        self._t = 0
        self._start = True

    def is_done(self) -> bool:
        """Checks if the dip sequence is complete."""
        return self._event >= len(self._events_dt)
