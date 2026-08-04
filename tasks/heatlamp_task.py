"""酒精灯加热液体任务（level4 式复合实验）：抓烧杯 → 放铁架台石棉网 → 摘灯帽 →
持火柴点燃酒精灯 → 加热（蒸汽）→ 观察 → 灯帽盖灭（火焰熄灭）。

场景（lab_003.usd）须包含：
    <lamp>          酒精灯引用（/World/AlcoholLamp，桌面 z=0.762）
    <lamp>/cap      灯帽（kinematic，任务驱动跟随夹爪）
    <lamp>/wick_tip 灯芯（点火参考点）
    <lamp>/flame_outer, flame_inner  火焰 Cone（初始隐藏）
    <steam>         蒸汽（/World/Steam，初始隐藏；reset 时移到烧杯口上方）
    <stand>         铁架台石棉网（/World/AsbestosGauze，须带碰撞，烧杯放置位）
    <match>         火柴（/World/Match，kinematic，任务驱动跟随夹爪）
    <match_flame>   火柴头火焰（/World/MatchFlame，初始隐藏，跟随火柴头）

任务端状态机（全部由 gripper 可观测状态驱动，与 IgniteLampTask 同一机制）：
    cap_state:   "closed"(灯口) -> "attached"(跟随) -> "placed"(桌面, settle)
                 -> "attached"(再次跟随, 桌面为原点) -> "capped"(回灯口, 灭火灭汽)
    match_state: "rest" -> "attached"(跟随) -> "released"(桌面, settle)
    flame_on:    夹爪持火柴在灯芯旁 dwell ignite_dwell_frames 帧后 reveal 灯焰
    steaming:    flame_on 且烧杯在 stand 位 dwell heat_dwell_frames 帧后 reveal 蒸汽
    obs_done:    steaming 后持续 obs_dwell_frames 帧（观察环节完成）

控制器（HeatLampTaskController）按 phase 读取这些状态切换环节。
"""
import numpy as np
from pxr import Usd, UsdGeom
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
        self.cap_path = f"{self.lamp_path}/cap"
        self.wick_path = f"{self.lamp_path}/wick_tip"
        self.flame_paths = [
            f"{self.lamp_path}/flame_outer",
            f"{self.lamp_path}/flame_inner",
        ]
        self.steam_path = cfg.steam_path
        self.stand_path = cfg.stand_path
        self.match_path = cfg.match_path
        self.match_flame_path = cfg.match_flame_path

        # 可配置偏移（相对各自参考点）
        self.cap_rest_offset = np.array(getattr(cfg, "cap_rest_offset", [0.12, 0.06, 0.02]), dtype=float)
        # 火柴头目标相对灯芯顶的偏移（火柴头要伸到灯芯旁火焰区）
        self.ignite_offset = np.array(getattr(cfg, "ignite_offset", [0.0, 0.02, 0.02]), dtype=float)
        # 火柴抓取点相对火柴场景 translate 的偏移（棍中部）
        self.match_grasp_offset = np.array(getattr(cfg, "match_grasp_offset", [0.04, 0.0, 0.0]), dtype=float)
        # 火柴头相对夹爪的偏移（+x 端；夹爪抓在棍中部）
        self.match_head_offset = np.array(getattr(cfg, "match_head_offset", [0.0578, 0.0, 0.0]), dtype=float)

        # 检测阈值
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.ignite_xy_threshold = getattr(cfg, "ignite_xy_threshold", 0.02)
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 25))
        self.heat_dwell_frames = int(getattr(cfg, "heat_dwell_frames", 40))
        self.obs_dwell_frames = int(getattr(cfg, "obs_dwell_frames", 30))
        self.on_stand_threshold = getattr(cfg, "on_stand_threshold", 0.03)

        # 帽在灯口的 translate（__init__ 读一次；reset 时复位 + 盖帽终点）
        self.cap_closed_translate = self._read_translate(self.cap_path)
        # 火柴场景 translate（__init__ 读一次；reset 时复位 + 跟随原点）
        self.match_orig_translate = self._read_translate(self.match_path)

        # Episode state（reset 时重新设置）
        self.cap_state = "closed"
        self.cap_orig_translate = self.cap_closed_translate.copy()
        self.attach_gripper_pos = None
        self.match_state = "rest"
        self.match_attach_gripper_pos = None
        self.flame_on = False
        self.steaming = False
        self.obs_done = False
        self.flame_counter = 0
        self.heat_counter = 0
        self.obs_counter = 0
        self.cap_grasp_pos = None
        self.cap_closed_position = None
        self.cap_rest_position = None
        self.wick_top_pos = None
        self.match_head_target = None
        self.match_ignite_position = None
        self.match_grasp_position = None
        self.stand_position = None
        self.table_z = None

    def reset(self):
        """Reset the task state: beaker randomized, cap on lamp, match on table,
        flame/steam/match-flame hidden."""
        super().reset()
        self.robot.initialize()

        # 烧杯随机化（obj_paths[0]）
        if len(self.obj_configs) > 0:
            self.current_obj_path = self.place_objects_with_visibility_management(self.current_obj_idx)

        # 帽回灯口、火柴回原位、火焰/蒸汽/火柴火隐藏
        self.object_utils.set_object_position(
            object_path=self.cap_path, position=self.cap_closed_translate.copy()
        )
        self.object_utils.set_object_position(
            object_path=self.match_path, position=self.match_orig_translate.copy()
        )
        self._set_flame_visible(False)
        self._set_steam_visible(False)
        self._set_match_flame_visible(False)

        self.cap_state = "closed"
        self.cap_orig_translate = self.cap_closed_translate.copy()
        self.attach_gripper_pos = None
        self.match_state = "rest"
        self.match_attach_gripper_pos = None
        self.flame_on = False
        self.steaming = False
        self.obs_done = False
        self.flame_counter = 0
        self.heat_counter = 0
        self.obs_counter = 0

        # --- 静态世界参考点（本 episode 不变）---
        # 帽中心（灯口位）既是摘帽抓取位，也是盖帽落位
        self.cap_grasp_pos = self.object_utils.get_geometry_center(object_path=self.cap_path)
        self.cap_closed_position = self.cap_grasp_pos.copy()
        # 灯芯顶（点火参考）
        self.wick_top_pos = self.object_utils.get_geometry_center(object_path=self.wick_path)
        # 桌面高度 = 酒精灯根 z
        lamp_position = self.object_utils.get_object_xform_position(self.lamp_path)
        self.table_z = lamp_position[2]

        self.cap_rest_position = lamp_position + self.cap_rest_offset

        # 火柴：抓取位（棍中部）、点燃位（夹爪位 = 火柴头目标 - 头偏移）、放回位（原位）
        self.match_grasp_position = self.match_orig_translate + self.match_grasp_offset
        self.match_head_target = self.wick_top_pos + self.ignite_offset
        self.match_ignite_position = self.match_head_target - self.match_head_offset
        self.match_rest_position = self.match_grasp_position.copy()

        # 铁架台石棉网：网顶 z（烧杯落位）、stand 位（烧杯中心目标）
        stand_prim = self.stage.GetPrimAtPath(self.stand_path)
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        gauze_bound = bbox_cache.ComputeWorldBound(stand_prim).ComputeAlignedRange()
        gauze_center = self.object_utils.get_object_xform_position(self.stand_path)
        self.gauze_top_z = gauze_bound.GetMax()[2]
        beaker_size = self.object_utils.get_object_size(object_path=self.current_obj_path)
        self.stand_position = np.array([
            gauze_center[0], gauze_center[1], self.gauze_top_z + beaker_size[2] / 2.0
        ])

        # 蒸汽移到烧杯口上方（stand 位上方）
        steam_position = np.array([
            gauze_center[0], gauze_center[1], self.gauze_top_z + beaker_size[2] + 0.005
        ])
        self.object_utils.set_object_position(object_path=self.steam_path, position=steam_position)

    def step(self):
        """Execute one simulation step.

        Returns:
            dict: State dictionary, or None if not ready.
        """
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        self._update_world_state()

        return self.get_basic_state_info(
            object_path=self.current_obj_path,
            additional_info={
                "stand_position": self.stand_position,
                "on_stand": self._beaker_on_stand(),
                "cap_position": self.cap_grasp_pos,
                "cap_closed_position": self.cap_closed_position,
                "cap_rest_position": self.cap_rest_position,
                "cap_state": self.cap_state,
                "wick_position": self.wick_top_pos,
                "match_grasp_position": self.match_grasp_position,
                "match_ignite_position": self.match_ignite_position,
                "match_rest_position": self.match_rest_position,
                "match_state": self.match_state,
                "flame_on": self.flame_on,
                "steaming": self.steaming,
                "obs_done": self.obs_done,
            },
        )

    # ------------------------------------------------------------------
    # 世界状态更新
    # ------------------------------------------------------------------
    def _update_world_state(self):
        """Drive cap/match kinematic following, match flame, and the
        flame/steam/observe reveal logic."""
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]  # finger distance (small = closed)

        self._update_cap(gripper_pos, gripper_opening)
        self._update_match(gripper_pos, gripper_opening)
        self._update_heating()

    def _update_cap(self, gripper_pos, gripper_opening):
        """Cap lifecycle: closed -> attached -> placed -> attached -> capped.

        - "closed":  cap sits on the lamp. Gripper near the cap (lamp mouth)
          and closed -> "attached" (mirror origin = cap-on-lamp translate).
        - "attached": cap mirrors gripper delta. Gripper opens:
            * near the lamp mouth -> "capped" (cap snaps to lamp mouth,
              flame & steam extinguished);
            * otherwise (over the table) -> "placed" (cap settles on table).
        - "placed": cap rests on the table. Gripper near the cap there and
          closed -> "attached" again (mirror origin = current table translate).
        """
        if self.cap_state == "closed":
            near_cap = np.linalg.norm(gripper_pos - self.cap_grasp_pos) < self.grasp_xy_threshold
            if near_cap and gripper_opening < self.gripper_closed_threshold:
                self.cap_state = "attached"
                self.attach_gripper_pos = gripper_pos.copy()
                self.cap_orig_translate = self.cap_closed_translate.copy()

        elif self.cap_state == "attached":
            delta = gripper_pos - self.attach_gripper_pos
            cap_translate = self.cap_orig_translate + delta
            self.object_utils.set_object_position(object_path=self.cap_path, position=cap_translate)
            if gripper_opening > self.gripper_open_threshold:
                if np.linalg.norm(gripper_pos - self.cap_closed_position) < 0.05:
                    # 盖帽：帽精确落回灯口，火焰与蒸汽熄灭
                    self.cap_state = "capped"
                    self.object_utils.set_object_position(
                        object_path=self.cap_path, position=self.cap_closed_translate.copy()
                    )
                    self.flame_on = False
                    self._set_flame_visible(False)
                    self._set_steam_visible(False)
                else:
                    self.cap_state = "placed"
                    self._settle_cap_on_table()

        elif self.cap_state == "placed":
            near_cap = np.linalg.norm(gripper_pos - self.cap_rest_position) < self.grasp_xy_threshold
            if near_cap and gripper_opening < self.gripper_closed_threshold:
                self.cap_state = "attached"
                self.attach_gripper_pos = gripper_pos.copy()
                # 第二次吸附：原点换成当前桌面位（否则帽会跳回灯口）
                self.cap_orig_translate = self._read_translate(self.cap_path)

        # "capped": cap stays seated; nothing to do.

    def _update_match(self, gripper_pos, gripper_opening):
        """Match lifecycle: rest -> attached -> released, plus the match flame.

        - "rest": match lies on the table. Gripper near the grasp point and
          closed -> "attached" (mirror origin = match table translate).
        - "attached": match mirrors gripper delta; a small flame prim follows
          the match head. Gripper opens -> "released" (match settles on table).
        - The match flame is hidden once the lamp flame is revealed.
        """
        if self.match_state == "rest":
            near_match = np.linalg.norm(gripper_pos - self.match_grasp_position) < self.grasp_xy_threshold
            if near_match and gripper_opening < self.gripper_closed_threshold:
                self.match_state = "attached"
                self.match_attach_gripper_pos = gripper_pos.copy()
                self._set_match_flame_visible(True)

        elif self.match_state == "attached":
            delta = gripper_pos - self.match_attach_gripper_pos
            match_translate = self.match_orig_translate + delta
            self.object_utils.set_object_position(object_path=self.match_path, position=match_translate)
            # 火柴头火焰跟随（火柴头 = 夹爪 + 头偏移，略抬高）
            if not self.flame_on:
                flame_pos = gripper_pos + self.match_head_offset
                flame_pos[2] += 0.005
                self.object_utils.set_object_position(object_path=self.match_flame_path, position=flame_pos)
            else:
                # 灯已点燃，火柴功成身退
                self._set_match_flame_visible(False)
            if gripper_opening > self.gripper_open_threshold:
                self.match_state = "released"
                self._set_match_flame_visible(False)
                self._settle_match_on_table()

    def _update_heating(self):
        """Reveal the steam when the lamp flame is on AND the beaker rests on
        the stand; then mark observation done after a dwell."""
        # 点火（火柴头在灯芯旁 dwell）
        if not self.flame_on:
            gripper_pos = self.robot.get_gripper_position()
            if gripper_pos is not None:
                if np.linalg.norm(gripper_pos - self.match_ignite_position) < self.ignite_xy_threshold:
                    self.flame_counter += 1
                    if self.flame_counter >= self.ignite_dwell_frames:
                        self.flame_on = True
                        self._set_flame_visible(True)
                else:
                    self.flame_counter = 0

        # 加热（火焰亮 + 烧杯在 stand 位）
        if self.flame_on and not self.steaming:
            if self._beaker_on_stand():
                self.heat_counter += 1
                if self.heat_counter >= self.heat_dwell_frames:
                    self.steaming = True
                    self._set_steam_visible(True)
            else:
                self.heat_counter = 0

        # 观察完成（蒸汽亮后持续 dwell）
        if self.steaming and not self.obs_done:
            self.obs_counter += 1
            if self.obs_counter >= self.obs_dwell_frames:
                self.obs_done = True

    def _beaker_on_stand(self) -> bool:
        """True when the beaker center is close to the stand position."""
        obj_pos = self.object_utils.get_geometry_center(object_path=self.current_obj_path)
        return np.linalg.norm(obj_pos - self.stand_position) < self.on_stand_threshold

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _settle_cap_on_table(self):
        """Snap the cap vertically so its bottom rests exactly on the table."""
        prim = self.stage.GetPrimAtPath(self.cap_path)
        if not prim.IsValid():
            return
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        world_bottom = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]
        dz = self.table_z - world_bottom
        current = self.object_utils.get_object_xform_position(self.cap_path)
        settled = current.copy()
        settled[2] = current[2] + dz
        self.object_utils.set_object_position(object_path=self.cap_path, position=settled)

    def _settle_match_on_table(self):
        """Snap the match vertically so its bottom rests exactly on the table."""
        prim = self.stage.GetPrimAtPath(self.match_path)
        if not prim.IsValid():
            return
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        world_bottom = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]
        dz = self.table_z - world_bottom
        current = self.object_utils.get_object_xform_position(self.match_path)
        settled = current.copy()
        settled[2] = current[2] + dz
        self.object_utils.set_object_position(object_path=self.match_path, position=settled)

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

    def _set_match_flame_visible(self, visible: bool) -> None:
        """Show or hide the match-head flame prim."""
        prim = self.stage.GetPrimAtPath(self.match_flame_path)
        if prim.IsValid():
            set_prim_visibility(prim, visible)

    def _read_translate(self, object_path: str) -> np.ndarray:
        """Read the translate xformOp value of a prim (defaults to zeros)."""
        prim = self.stage.GetPrimAtPath(object_path)
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    return np.array(op.Get(), dtype=float)
        return np.zeros(3)
