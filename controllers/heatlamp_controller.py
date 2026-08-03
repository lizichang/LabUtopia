"""酒精灯加热液体控制器：夹取烧杯 → 悬停到酒精灯火焰上方（点火）→ 完成。

两阶段：
1. PICKBEAKER：用 PickController 从工作区抓起烧杯（物理抓取）。
2. HEATING：用 IgniteController 把夹爪（夹着烧杯）移到灯口上方悬停位并停留，
   任务检测到 dwell 后 reveal 火焰和蒸汽（抽象加热）。
"""
import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.pick_controller import PickController
from .atomic_actions.ignite_controller import IgniteController
from .base_controller import BaseController


class Phase(Enum):
    PICKBEAKER = "pickbeaker"
    HEATING = "heating"
    FINISHED = "finished"


class HeatLampTaskController(BaseController):
    """Composite controller for heating a beaker of liquid over an alcohol lamp."""

    def __init__(self, cfg, robot):
        """Initialize the heat-lamp task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.PICKBEAKER
        self.initial_object_z = None

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        self.pick_controller = PickController(
            name="pick_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.002, 0.002, 0.005, 0.02, 0.05, 0.01, 0.02],
        )

        self.ignite_controller = IgniteController(
            name="ignite_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.004, 0.006],
        )

        self.active_controller = self.pick_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.PICKBEAKER
        self.initial_object_z = None

        if self.mode == "collect":
            self.active_controller = self.pick_controller
            self.pick_controller.reset()
            self.ignite_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task)."""
        if self.current_phase == Phase.PICKBEAKER:
            # 烧杯被夹爪提起足够高度即算抓起
            return self.state['object_position'][2] > self.initial_object_z + 0.1
        elif self.current_phase == Phase.HEATING:
            # 任务在 dwell 检测到加热（火焰 + 蒸汽 reveal）
            return bool(self.state.get('steaming'))
        return False

    def step(self, state):
        """Execute one step of control.

        Args:
            state: Current state dictionary containing sensor data and robot state.

        Returns:
            Tuple containing action, done flag, and success flag.
        """
        self.state = state
        if self.initial_object_z is None:
            self.initial_object_z = self.state['object_position'][2]
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
            if self.current_phase == Phase.PICKBEAKER:
                action = self.pick_controller.forward(
                    picking_position=state['object_position'],
                    current_joint_positions=state['joint_positions'],
                    object_size=state['object_size'],
                    object_name=state['object_name'],
                    gripper_control=self.gripper_control,
                    gripper_position=state['gripper_position'],
                    end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
                    pre_offset_x=0.05,
                    pre_offset_z=0.05,
                )
            elif self.current_phase == Phase.HEATING:
                action = self.ignite_controller.forward(
                    ignite_position=state['heat_position'],
                    current_joint_positions=state['joint_positions'],
                    gripper_position=state['gripper_position'],
                    gripper_control=self.gripper_control,
                    end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
                    retract_offset=np.array([0.02, 0.0, 0.02]),
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
            if self.current_phase == Phase.PICKBEAKER:
                print("Pick beaker success! Moving to the alcohol lamp...")
                self.current_phase = Phase.HEATING
                self.active_controller = self.ignite_controller
                return None, False, False
            elif self.current_phase == Phase.HEATING:
                print("Heating success! The liquid is boiling.")
                self.data_collector.write_cached_data(state['joint_positions'][:-1])
                self._last_success = True
                self.current_phase = Phase.FINISHED
                return None, True, True

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
        """Task succeeds when the liquid is steaming over the lit lamp."""
        return bool(self.state.get('steaming'))

    def get_language_instruction(self):
        return "Pick up the beaker and hold it over the alcohol lamp flame to heat the liquid"
