"""MatchIgniteController：持火柴点燃酒精灯（用户确认的"持火柴点燃"方式）。

火柴由任务吸附在夹爪上（kinematic 跟随，同铂丝机制），本控制器只产生
夹爪运动序列：抓火柴 → 上提 → 移到灯芯旁点燃位 → 停留（任务点亮灯焰并
给火柴头加小火苗）→ 移回桌面 → 松手（火柴放回桌面熄灭）。

Phases:
- Phase 0: 移到火柴上方（pre-grasp）
- Phase 1: 下降到火柴抓取位
- Phase 2: 等待动力学稳定
- Phase 3: 闭合夹爪（任务检测到吸附，火柴跟随）
- Phase 4: 上提火柴
- Phase 5: 移到点燃位（火柴头在灯芯旁）
- Phase 6: 停留（任务检测 dwell → 灯焰 reveal + 火柴头火焰）
- Phase 7: 移回桌面放回位（任务把火柴 settle 贴桌）
- Phase 8: 打开夹爪（释放火柴）
- Phase 9: 完成
"""
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class MatchIgniteController(BaseController):
    """State machine controller for igniting the alcohol lamp with a match.

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller.
        events_dt (List[float], optional): Duration for each phase (10 elements).
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
            self._events_dt = [0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.03, 0.01, 0.05, 0.006]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 10:
                raise Exception(f"events_dt length must be 10, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        match_grasp_position: np.ndarray,
        match_ignite_position: np.ndarray,
        match_rest_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.08,
        lift_z: float = 0.07,
    ) -> ArticulationAction:
        """Computes the joint positions for the current match-ignite phase.

        Args:
            match_grasp_position (np.ndarray): World position of the match stick
                (where the gripper closes to pick it up).
            match_ignite_position (np.ndarray): World position where the match head
                is held beside the lamp wick (the abstract ignition point).
            match_rest_position (np.ndarray): World position where the match is put
                back on the table after lighting.
            current_joint_positions (np.ndarray): Current joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above the match for the pre-grasp point.
            lift_z (float): Height to lift the match above its grasp position.

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
            match_grasp_position,
            match_ignite_position,
            match_rest_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            pre_offset_z,
            lift_z,
        )

        if self._event < len(self._events_dt):
            self._t += self._events_dt[self._event]
            if self._t >= 1.0:
                self._event += 1
                self._t = 0

        return target_joint_positions

    def _execute_phase(
        self,
        match_grasp_position,
        match_ignite_position,
        match_rest_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
    ):
        """Executes the current phase of the match-ignite sequence."""
        if self._event == 0:
            # Move above the match.
            target_position = match_grasp_position + np.array([0.0, 0.0, pre_offset_z])
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
            # Lower to the match grasp position.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=match_grasp_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - match_grasp_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 2:
            # Wait for dynamics to settle.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=match_grasp_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 3:
            # Close the gripper (the task attaches the match on detection).
            # 闭合目标 0.02 必须 < 任务检测阈值 gripper_closed_threshold(0.025)。
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.02 / get_stage_units()
            target_joint_positions[8] = 0.02 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 4:
            # Lift the match straight up.
            target_position = match_grasp_position + np.array([0.0, 0.0, lift_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 5:
            # Move to the ignition point beside the wick.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=match_ignite_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - match_ignite_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 6:
            # Dwell at the ignition point; the task lights the lamp and the match.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=match_ignite_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            return target_joint_positions

        elif self._event == 7:
            # Move the match back to its rest position on the table.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=match_rest_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - match_rest_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 8:
            # Open the gripper (the task settles the match on the table and hides
            # the match flame).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = 0.04 / get_stage_units()
            target_joint_positions[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        else:
            # Done.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

    def get_phase(self) -> int:
        """Returns the current phase index (0-9)."""
        return self._event

    def reset(self) -> None:
        """Resets the controller to the initial phase."""
        super().reset()
        self._cspace_controller.reset()
        self._event = 0
        self._t = 0
        self._start = True

    def is_done(self) -> bool:
        """Checks if the match-ignite sequence is complete."""
        return self._event >= len(self._events_dt)
