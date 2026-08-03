"""酒精灯加热液体任务：夹取烧杯 → 悬停到酒精灯火焰上方 → 点火 → 沸腾（蒸汽）。

场景（lab_003.usd）须包含：
    <lamp>       酒精灯引用（/World/AlcoholLamp，桌面 z=0.762）
    <lamp>/flame_outer, flame_inner  火焰 Cone（初始隐藏，加热后 reveal）
    <steam>      蒸汽 Cone（初始隐藏，加热后 reveal，代表液体沸腾）
烧杯为物理物体（真抓取），由 pick 原子动作抓起；本任务只管理加热判定与
火焰/蒸汽的 reveal（抽象加热：夹爪在火焰上方停留 N 帧 → 开始加热）。
"""
import numpy as np
from pxr import Usd
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class HeatLampTask(BaseTask):
    """Task definition for heating a beaker of liquid over an alcohol lamp."""

    def __init__(self, cfg, world, stage, robot):
        """Initialize the heat-lamp task.

        Args:
            cfg: Configuration object for the task.
            world: The simulation world instance.
            stage: The USD stage for the simulation.
            robot: The robot instance used in the task.
        """
        super().__init__(cfg, world, stage, robot)

        self.lamp_path = cfg.lamp_path
        self.steam_path = cfg.steam_path
        self.flame_paths = [
            f"{self.lamp_path}/flame_outer",
            f"{self.lamp_path}/flame_inner",
        ]

        # 夹爪悬停位（加热位）= 灯根 + heat_offset（beaker 底部进入火焰区）
        self.heat_offset = np.array(getattr(cfg, "heat_offset", [0.0, 0.0, 0.27]), dtype=float)

        # 检测阈值
        self.heat_xy_threshold = getattr(cfg, "heat_xy_threshold", 0.03)
        self.heat_dwell_frames = int(getattr(cfg, "heat_dwell_frames", 25))

        # Episode state
        self.current_obj_path = None
        self.heat_position = None
        self.flame_on = False
        self.steaming = False
        self.flame_counter = 0

    def reset(self):
        """Reset the task state: beaker randomized, flame and steam hidden."""
        super().reset()
        self.robot.initialize()

        # 随机化烧杯位置（obj_paths[0]），其余物体隐藏
        if len(self.obj_configs) > 0:
            self.current_obj_path = self.place_objects_with_visibility_management(self.current_obj_idx)

        # 火焰与蒸汽隐藏
        self._set_flame_visible(False)
        self._set_steam_visible(False)

        self.flame_on = False
        self.steaming = False
        self.flame_counter = 0

        # 加热位（夹爪悬停位）= 灯根 + 偏移
        lamp_position = self.object_utils.get_object_xform_position(self.lamp_path)
        self.heat_position = lamp_position + self.heat_offset

    def step(self):
        """Execute one simulation step.

        Returns:
            dict: State dictionary, or None if not ready.
        """
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        self._update_heating()

        return self.get_basic_state_info(
            object_path=self.current_obj_path,
            additional_info={
                "heat_position": self.heat_position,
                "flame_on": self.flame_on,
                "steaming": self.steaming,
            },
        )

    def _update_heating(self):
        """Reveal the flame and steam when the gripper dwells over the lamp mouth.

        This is the abstract heating gesture: the gripper (holding the beaker)
        hovers above the burner mouth; after heat_dwell_frames consecutive frames
        of dwelling there, the flame ignites and the liquid starts steaming.
        """
        if self.flame_on:
            return

        gripper_pos = self.robot.get_gripper_position()
        if gripper_pos is None:
            return

        if np.linalg.norm(gripper_pos - self.heat_position) < self.heat_xy_threshold:
            self.flame_counter += 1
            if self.flame_counter >= self.heat_dwell_frames:
                self.flame_on = True
                self.steaming = True
                self._set_flame_visible(True)
                self._set_steam_visible(True)
        else:
            self.flame_counter = 0

    def _set_flame_visible(self, visible: bool) -> None:
        """Show or hide the lamp flame prims."""
        for flame_path in self.flame_paths:
            prim = self.stage.GetPrimAtPath(flame_path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)

    def _set_steam_visible(self, visible: bool) -> None:
        """Show or hide the steam prim."""
        prim = self.stage.GetPrimAtPath(self.steam_path)
        if prim.IsValid():
            set_prim_visibility(prim, visible)
