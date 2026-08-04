"""酒精灯加热液体控制器（level4 式复合实验）：8 个 phase 串联多个原子动作。

Phase 序列（严格还原酒精灯加热操作规范）：
    1. PICK_BEAKER    PickController 从工作区抓起烧杯（物理抓取）
    2. PLACE_ON_STAND PlaceController 把烧杯放到铁架台石棉网上
    3. UNCAP_LAMP     UncapController 摘下灯帽放到桌面（任务检测 cap_state=placed）
    4. IGNITE_MATCH   MatchIgniteController 持火柴点燃酒精灯（任务检测 flame_on）
    5. HEAT           无控制器：等待任务检测蒸汽（flame_on + 烧杯在 stand 位）
    6. OBSERVE        无控制器：等待任务检测 obs_done（蒸汽持续观察）
    7. CAP_LAMP       CapController 抓帽盖回灯口（任务检测 cap_state=capped，
                      同时火焰/蒸汽熄灭）
    8. FINISHED       完成

HEAT/OBSERVE 两个等待环节没有原子控制器（active_controller=None），
每次 step 直接检查任务状态，成功后切换下一 phase。
"""
import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from .atomic_actions.pick_controller import PickController
from .atomic_actions.place_controller import PlaceController
from .atomic_actions.uncap_controller import UncapController
from .atomic_actions.match_ignite_controller import MatchIgniteController
from .atomic_actions.cap_controller import CapController
from .base_controller import BaseController


class Phase(Enum):
    PICK_BEAKER = "pick_beaker"
    PLACE_ON_STAND = "place_on_stand"
    UNCAP_LAMP = "uncap_lamp"
    IGNITE_MATCH = "ignite_match"
    HEAT = "heat"
    OBSERVE = "observe"
    CAP_LAMP = "cap_lamp"
    FINISHED = "finished"


class HeatLampTaskController(BaseController):
    """Composite controller for heating a beaker over a lit alcohol lamp."""

    def __init__(self, cfg, robot):
        """Initialize the heat-lamp task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)
        self.current_phase = Phase.PICK_BEAKER
        self.initial_object_z = None

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)

        # 1. 抓烧杯
        self.pick_controller = PickController(
            name="pick_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.002, 0.002, 0.005, 0.02, 0.05, 0.01, 0.02],
        )

        # 2. 放铁架台石棉网
        self.place_controller = PlaceController(
            name="place_controller",
            cspace_controller=self.rmp_controller,
            gripper=robot.gripper,
            events_dt=[0.003, 0.008, 1, 0.05, 0.01, 1],
        )

        # 3. 摘灯帽
        self.uncap_controller = UncapController(
            name="uncap_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.002, 0.002, 0.02, 0.05, 0.006, 0.006, 0.05, 0.006],
        )

        # 4. 持火柴点燃
        self.match_ignite_controller = MatchIgniteController(
            name="match_ignite_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.03, 0.01, 0.05, 0.006],
        )

        # 7. 盖帽灭灯
        self.cap_controller = CapController(
            name="cap_controller",
            cspace_controller=self.rmp_controller,
            events_dt=[0.004, 0.004, 0.02, 0.05, 0.006, 0.01, 0.004, 0.05, 0.006],
        )

        self.active_controller = self.pick_controller

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    def reset(self):
        """Reset controller state and phase."""
        super().reset()
        self.current_phase = Phase.PICK_BEAKER
        self.initial_object_z = None

        if self.mode == "collect":
            self.active_controller = self.pick_controller
            self.pick_controller.reset()
            self.place_controller.reset()
            self.uncap_controller.reset()
            self.match_ignite_controller.reset()
            self.cap_controller.reset()
        else:
            self.inference_engine.reset()

    def _check_phase_success(self):
        """Check if the current phase goal has been achieved (reported by the task)."""
        if self.current_phase == Phase.PICK_BEAKER:
            # 烧杯被夹爪提起足够高度即算抓起
            return self.state['object_position'][2] > self.initial_object_z + 0.1
        elif self.current_phase == Phase.PLACE_ON_STAND:
            # 烧杯中心落在铁架台 stand 位（石棉网上）
            return bool(self.state.get('on_stand'))
        elif self.current_phase == Phase.UNCAP_LAMP:
            # 任务把帽放到桌面（kinematic 检测）
            return self.state.get('cap_state') == 'placed'
        elif self.current_phase == Phase.IGNITE_MATCH:
            # 火柴头在灯芯旁 dwell，任务点亮灯焰
            return bool(self.state.get('flame_on'))
        elif self.current_phase == Phase.HEAT:
            # 火焰亮 + 烧杯在 stand 位 dwell，任务 reveal 蒸汽
            return bool(self.state.get('steaming'))
        elif self.current_phase == Phase.OBSERVE:
            # 蒸汽持续观察 N 帧
            return bool(self.state.get('obs_done'))
        elif self.current_phase == Phase.CAP_LAMP:
            # 帽盖回灯口，任务熄灭火焰/蒸汽
            return self.state.get('cap_state') == 'capped'
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

        # 无控制器（HEAT/OBSERVE 等待环节）或控制器运动序列未完成
        controller_done = self.active_controller is None or self.active_controller.is_done()

        action = None
        if not controller_done:
            action = self._phase_action(state)

        if 'camera_data' in state:
            self.data_collector.cache_step(
                camera_images=state['camera_data'],
                joint_angles=state['joint_positions'][:-1],
                language_instruction=self.get_language_instruction(),
            )

        if not controller_done:
            return action, False, False

        # 控制器完成（或等待环节）-> 检查 phase 目标
        if success:
            return self._advance_phase(state)

        print(f"{self.current_phase.value} failed!")
        self.data_collector.clear_cache()
        self._last_success = False
        self.current_phase = Phase.FINISHED
        return None, True, False

    def _phase_action(self, state):
        """Produce the gripper action for the current phase's atomic controller."""
        if self.current_phase == Phase.PICK_BEAKER:
            return self.pick_controller.forward(
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
        elif self.current_phase == Phase.PLACE_ON_STAND:
            return self.place_controller.forward(
                place_position=state['stand_position'],
                current_joint_positions=state['joint_positions'],
                gripper_control=self.gripper_control,
                gripper_position=state['gripper_position'],
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )
        elif self.current_phase == Phase.UNCAP_LAMP:
            return self.uncap_controller.forward(
                cap_position=state['cap_position'],
                cap_rest_position=state['cap_rest_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )
        elif self.current_phase == Phase.IGNITE_MATCH:
            return self.match_ignite_controller.forward(
                match_grasp_position=state['match_grasp_position'],
                match_ignite_position=state['match_ignite_position'],
                match_rest_position=state['match_rest_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )
        elif self.current_phase == Phase.CAP_LAMP:
            return self.cap_controller.forward(
                cap_rest_position=state['cap_rest_position'],
                cap_closed_position=state['cap_closed_position'],
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                gripper_control=self.gripper_control,
                end_effector_orientation=R.from_euler('xyz', np.radians([0, 90, 30])).as_quat(),
            )
        return None

    def _advance_phase(self, state):
        """Phase goal achieved: switch to the next phase (or finish)."""
        if self.current_phase == Phase.PICK_BEAKER:
            print("Pick beaker success! Placing on the stand...")
            self.current_phase = Phase.PLACE_ON_STAND
            self.active_controller = self.place_controller
            return None, False, False
        elif self.current_phase == Phase.PLACE_ON_STAND:
            print("Beaker on stand! Uncapping the lamp...")
            self.current_phase = Phase.UNCAP_LAMP
            self.active_controller = self.uncap_controller
            return None, False, False
        elif self.current_phase == Phase.UNCAP_LAMP:
            print("Uncap success! Lighting the lamp with a match...")
            self.current_phase = Phase.IGNITE_MATCH
            self.active_controller = self.match_ignite_controller
            return None, False, False
        elif self.current_phase == Phase.IGNITE_MATCH:
            print("Ignite success! The alcohol lamp is burning. Heating...")
            self.current_phase = Phase.HEAT
            self.active_controller = None
            return None, False, False
        elif self.current_phase == Phase.HEAT:
            print("Heating success! The liquid is steaming. Observing...")
            self.current_phase = Phase.OBSERVE
            self.active_controller = None
            return None, False, False
        elif self.current_phase == Phase.OBSERVE:
            print("Observation done! Capping the lamp...")
            self.current_phase = Phase.CAP_LAMP
            self.active_controller = self.cap_controller
            return None, False, False
        elif self.current_phase == Phase.CAP_LAMP:
            print("Lamp capped! Task complete.")
            self.data_collector.write_cached_data(state['joint_positions'][:-1])
            self._last_success = True
            self.current_phase = Phase.FINISHED
            return None, True, True
        return None, False, False

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
        """Task succeeds when the lamp is capped after heating."""
        return self.state.get('cap_state') == 'capped'

    def get_language_instruction(self):
        return "Place the beaker on the iron stand, ignite the alcohol lamp with a match, heat the liquid, and cap the lamp when done"
