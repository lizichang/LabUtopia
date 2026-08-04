"""CapController：盖帽动作（酒精灯用毕盖灭）。

UncapController 的逆操作：夹爪到桌面帽位抓帽（任务吸附，kinematic 跟随）→
上提 → 移到灯口正上方 → 下降盖帽（任务检测 cap 落位 → capped，火焰熄灭）→
打开夹爪释放。

Phases:
- Phase 0: 移到桌面帽位上方（pre-grasp）
- Phase 1: 下降到帽抓取位
- Phase 2: 等待动力学稳定
- Phase 3: 闭合夹爪（任务吸附 cap，跟随夹爪）
- Phase 4: 上提 cap
- Phase 5: 移到灯口正上方
- Phase 6: 下降盖帽（cap 落回灯口，任务判定 capped）
- Phase 7: 打开夹爪（cap 固定在灯口，火焰熄灭）
- Phase 8: 完成
"""
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class CapController(BaseController):
    """State machine controller for recapping the alcohol lamp.

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
            self._events_dt = [0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.004, 0.05, 0.006]
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
        cap_rest_position: np.ndarray,
        cap_closed_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.08,
        lift_z: float = 0.07,
        grasp_distance: float = 0.019,
    ) -> ArticulationAction:
        """Computes the joint positions for the current cap phase.

        Args:
            cap_rest_position (np.ndarray): World position of the cap on the table
                (grasp point).
            cap_closed_position (np.ndarray): World position where the cap sits on
                the lamp mouth when closed (the seating target).
            current_joint_positions (np.ndarray): Current joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above the cap for the pre-grasp position.
            lift_z (float): Height to lift the cap above its grasp position.
            grasp_distance (float): Gripper finger distance to grasp the cap.

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

        target_joint_positions = self._execute_phase(
            cap_rest_position,
            cap_closed_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            pre_offset_z,
            lift_z,
            grasp_distance,
        )

        if self._event < len(self._events_dt):
            self._t += self._events_dt[self._event]
            if self._t >= 1.0:
                self._event += 1
                self._t = 0

        return target_joint_positions

    def _execute_phase(
        self,
        cap_rest_position,
        cap_closed_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
        grasp_distance,
    ):
        """Executes the current phase of the cap sequence."""
        if self._event == 0:
            # Move directly above the cap on the table.
            target_position = cap_rest_position.copy()
            target_position[2] += pre_offset_z
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
            # Lower to the cap grasp position.
            target_position = cap_rest_position.copy()
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
            # Wait for dynamics to settle.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

        elif self._event == 3:
            # Close the gripper to grasp the cap (task attaches on detection).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = grasp_distance / get_stage_units()
            target_joint_positions[8] = grasp_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 4:
            # Lift the cap straight up (task drives the cap to follow).
            target_position = cap_rest_position.copy()
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
            # Move to a point above the lamp mouth.
            target_position = cap_closed_position.copy()
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

        elif self._event == 6:
            # Lower the cap onto the lamp mouth (task detects capped on seating).
            target_position = cap_closed_position.copy()
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 7:
            # Open the gripper (the cap stays seated on the lamp).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.04 / get_stage_units()
            target_joint_positions[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

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
        """Checks if the cap sequence is complete."""
        return self._event >= len(self._events_dt)
