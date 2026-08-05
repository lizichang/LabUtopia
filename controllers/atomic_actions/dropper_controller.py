"""DropperController：胶头滴管滴加动作（抓取→吸液→转移→滴加）。

滴管由任务吸附在夹爪上（kinematic 跟随，同 dip 的铂丝模式、scoop 的勺子
模式），本控制器只产生夹爪运动与夹爪开合。挤压/松开胶头通过"夹爪开合 +
停留"表达，task 根据夹爪距离检测状态：

- joint7 < 0.005  -> "squeeze"（挤压胶头：排空气 / 滴加）
- 0.005 < joint7 < 0.025 -> 仍吸附（未触发释放）
- joint7 > 0.03  -> 释放（task 判定滴管脱离）

Phases:
- Phase 0: 移到抓取位上方（pre-grasp）
- Phase 1: 下降到抓取位（管身）
- Phase 2: 等待动力学稳定
- Phase 3: 闭合夹爪（任务吸附滴管，跟随夹爪）
- Phase 4: 上提滴管
- Phase 5: 移到液瓶口上方（预挤压位）
- Phase 6: 挤压胶头排空气（闭到 squeeze_distance）
- Phase 7: 下探浸入液面（尖嘴触液）
- Phase 8: 松开胶头吸液（张到 release_distance，仍保持吸附）
- Phase 9: 上提离开液面
- Phase 10: 移到目标（试管口）上方
- Phase 11: 下探到滴加位
- Phase 12: 挤压胶头滴加
- Phase 13: 完成
"""
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
import typing


class DropperController(BaseController):
    """State machine controller for the dropper dripping gesture.

    Args:
        name (str): Identifier for the controller.
        cspace_controller (BaseController): Cartesian space controller.
        events_dt (List[float], optional): Duration for each phase (14 elements).
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
            self._events_dt = [0.004, 0.004, 0.02, 0.05, 0.004, 0.004, 0.05, 0.004, 0.05, 0.004, 0.004, 0.004, 0.05, 0.006]
        else:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, (np.ndarray, list)):
                raise Exception("events_dt must be a list or numpy array")
            if isinstance(self._events_dt, np.ndarray):
                self._events_dt = events_dt.tolist()
            if len(self._events_dt) != 14:
                raise Exception(f"events_dt length must be 14, got {len(self._events_dt)}")

        self._cspace_controller = cspace_controller
        self._start = True
        self._position_threshold = position_threshold

    def forward(
        self,
        grasp_position: np.ndarray,
        dip_position: np.ndarray,
        target_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        gripper_control,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        pre_offset_z: float = 0.12,
        lift_z: float = 0.10,
        grasp_distance: float = 0.008,
        squeeze_distance: float = 0.002,
        release_distance: float = 0.015,
    ) -> ArticulationAction:
        """Computes the joint positions for the current dropper phase.

        Args:
            grasp_position (np.ndarray): World position of the dropper body grasp
                point.
            dip_position (np.ndarray): World position where the dropper tip
                touches the liquid surface (the aspiration point).
            target_position (np.ndarray): World position where the dropper tip
                delivers the drops (above the target tube mouth).
            current_joint_positions (np.ndarray): Current joint positions.
            gripper_position (np.ndarray): Current gripper (TCP) position.
            gripper_control: Gripper controller instance.
            end_effector_orientation (np.ndarray, optional): Target orientation.
            pre_offset_z (float): Height above a position for the approach point.
            lift_z (float): Height to lift the dropper after grasping/aspirating.
            grasp_distance (float): Gripper finger distance to grasp the dropper
                body (0.008 matches the pick table entry for "pipette").
            squeeze_distance (float): Gripper finger distance when squeezing the
                rubber bulb (< 0.005 so the task detects "squeeze").
            release_distance (float): Gripper finger distance when releasing the
                bulb to aspirate (keeps the dropper attached: < 0.025).

        Returns:
            ArticulationAction: Joint positions for the robot to execute.
        """
        if self._start:
            return self._handle_start_state(current_joint_positions)

        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))

        target_joint_positions = self._execute_phase(
            grasp_position,
            dip_position,
            target_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
            pre_offset_z,
            lift_z,
            grasp_distance,
            squeeze_distance,
            release_distance,
        )

        if self._event < len(self._events_dt):
            self._t += self._events_dt[self._event]
            if self._t >= 1.0:
                self._event += 1
                self._t = 0

        return target_joint_positions

    def _handle_start_state(self, current_joint_positions):
        """Opens the gripper before starting the dropper sequence."""
        self._start = False
        target_joint_positions = [None] * current_joint_positions.shape[0]
        target_joint_positions[7] = 0.04 / get_stage_units()
        target_joint_positions[8] = 0.04 / get_stage_units()
        return ArticulationAction(joint_positions=target_joint_positions)

    def _execute_phase(
        self,
        grasp_position,
        dip_position,
        target_position,
        end_effector_orientation,
        current_joint_positions,
        gripper_position,
        pre_offset_z,
        lift_z,
        grasp_distance,
        squeeze_distance,
        release_distance,
    ):
        """Executes the current phase of the dropper sequence."""
        if self._event == 0:
            # Move directly above the grasp point.
            target_position_cmd = grasp_position + np.array([0.0, 0.0, pre_offset_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position_cmd,
                target_end_effector_orientation=end_effector_orientation,
            )
            xy_distance = np.linalg.norm(gripper_position[:2] - target_position_cmd[:2])
            if xy_distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 1:
            # Lower down to the grasp point (dropper body).
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
            # Close the gripper on the dropper body (the task detects "attached").
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = grasp_distance / get_stage_units()
            target_joint_positions[8] = grasp_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 4:
            # Lift the dropper straight up.
            target_position_cmd = grasp_position.copy()
            target_position_cmd[2] += lift_z
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position_cmd,
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position_cmd)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 5:
            # Move to a point above the liquid bottle mouth (pre-squeeze point).
            target_position_cmd = dip_position + np.array([0.0, 0.0, pre_offset_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position_cmd,
                target_end_effector_orientation=end_effector_orientation,
            )
            xy_distance = np.linalg.norm(gripper_position[:2] - target_position_cmd[:2])
            if xy_distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 6:
            # Squeeze the bulb to expel the air (the task detects "squeezed").
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = squeeze_distance / get_stage_units()
            target_joint_positions[8] = squeeze_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 7:
            # Lower the dropper tip into the liquid.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=dip_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - dip_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 8:
            # Release the bulb to aspirate (the task detects the liquid in).
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = release_distance / get_stage_units()
            target_joint_positions[8] = release_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        elif self._event == 9:
            # Lift the dropper out of the liquid.
            target_position_cmd = dip_position.copy()
            target_position_cmd[2] += lift_z
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position_cmd,
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position_cmd)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 10:
            # Move above the target position (tube mouth).
            target_position_cmd = target_position + np.array([0.0, 0.0, pre_offset_z])
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position_cmd,
                target_end_effector_orientation=end_effector_orientation,
            )
            xy_distance = np.linalg.norm(gripper_position[:2] - target_position_cmd[:2])
            if xy_distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 11:
            # Lower to the drip position.
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position.copy(),
                target_end_effector_orientation=end_effector_orientation,
            )
            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
            return target_joint_positions

        elif self._event == 12:
            # Squeeze the bulb to drip (the task detects "dropped").
            target_joint_positions = [None] * current_joint_positions.shape[0]
            target_joint_positions[7] = squeeze_distance / get_stage_units()
            target_joint_positions[8] = squeeze_distance / get_stage_units()
            return ArticulationAction(joint_positions=target_joint_positions)

        else:
            # Done.
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])

    def get_phase(self) -> int:
        """Returns the current phase index (0-13)."""
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
            if len(self._events_dt) != 14:
                raise Exception(f"events_dt length must be 14, got {len(self._events_dt)}")

    def is_done(self) -> bool:
        """Checks if the dropper sequence is complete."""
        return self._event >= len(self._events_dt)
