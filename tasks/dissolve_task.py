"""蒸馏水溶解性测试任务（D2）：用药匙舀取样品粉末 → 转移进试管 → 洗瓶加水 →
振荡 → 观察溶解。

场景（lab_004.usd）须包含：
    <spoon>         药匙（/World/Spoon，kinematic，任务驱动跟随夹爪；原点=手柄中点）
    <sample_powder> 样品粉末堆（/World/SamplePowder，静态）
    <test_tube>     试管（/World/TestTube，静态，插在试管架孔中，原点=管底）
    <test_tube_rack> 试管架（/World/TestTubeRack，静态视觉）
    <wash_bottle>   蒸馏水洗瓶（/World/WashBottle，kinematic；原点=瓶底）
    <powder_on_spoon> 勺上粉末（/World/PowderOnSpoon，初始隐藏，跟随勺头）
    <tube_sample>   试管内粉末（/World/TubeSample，初始隐藏）
    <tube_water>    试管内液面（/World/TubeWater，初始隐藏）

任务端状态机（全部由 gripper 可观测状态驱动，与 IgniteLampTask 同一机制）：
    spoon_state: "rest" -> "attached"(跟随) -> "scooping"(勺头插进粉末堆 dwell，
                 勺上粉末 reveal) -> "transferred"(勺头进试管口 dwell，粉末入管)
                 -> "released"(打开，settle 回桌面)
    wash_state:  "rest" -> "attached"(跟随) -> "pouring"(洗瓶举到试管口上方 dwell，
                 液面 reveal) -> "released"(打开，settle 回桌面)
    dissolved:   夹爪在试管口上方振荡（shake 区域）dwell N 帧后 True，
                 试管内粉末隐藏（已溶解）
    obs_done:    dissolved 后持续 dwell（观察环节完成）

控制器（DissolveTaskController）按 phase 读取这些状态切换环节。
"""
import numpy as np
from pxr import UsdGeom
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class DissolveTask(BaseTask):
    """Task definition for the solubility-in-water test (scoop -> tube -> shake)."""

    def __init__(self, cfg, world, stage, robot):
        """Initialize the dissolve task.

        Args:
            cfg: Configuration object for the task.
            world: The simulation world instance.
            stage: The USD stage for the simulation.
            robot: The robot instance used in the task.
        """
        super().__init__(cfg, world, stage, robot)

        self.spoon_path = cfg.spoon_path
        self.powder_path = cfg.powder_path
        self.tube_path = cfg.tube_path
        self.wash_path = cfg.wash_path
        self.powder_on_spoon_path = cfg.powder_on_spoon_path
        self.tube_sample_path = cfg.tube_sample_path
        self.tube_water_path = cfg.tube_water_path

        # 药匙几何：勺头中心相对夹爪（= 手柄中点，attach 后平移跟随）的偏移
        self.spoon_head_offset = np.array(getattr(cfg, "spoon_head_offset", [0.045, 0.0, 0.0]), dtype=float)

        # 可配置偏移（相对各自参考点）
        # 勺头插入粉末堆的深度（相对堆中心，负=插入）
        self.scoop_insert_offset = np.array(getattr(cfg, "scoop_insert_offset", [0.0, 0.0, -0.002]), dtype=float)
        # 洗瓶举到试管口上方的额外高度（相对试管口）
        self.wash_pour_lift = getattr(cfg, "wash_pour_lift", 0.06)
        # 振荡位 = 试管口 + 该偏移
        self.shake_offset = np.array(getattr(cfg, "shake_offset", [0.0, 0.0, 0.05]), dtype=float)
        # 洗瓶抓取位 = 瓶底 + 该偏移（瓶颈，直径 20mm）
        self.wash_grasp_offset = np.array(getattr(cfg, "wash_grasp_offset", [0.0, 0.0, 0.090]), dtype=float)

        # 检测阈值
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.scoop_dwell_frames = int(getattr(cfg, "scoop_dwell_frames", 25))
        self.transfer_dwell_frames = int(getattr(cfg, "transfer_dwell_frames", 25))
        self.pour_dwell_frames = int(getattr(cfg, "pour_dwell_frames", 25))
        self.shake_dwell_frames = int(getattr(cfg, "shake_dwell_frames", 60))
        self.obs_dwell_frames = int(getattr(cfg, "obs_dwell_frames", 30))
        self.shake_region_radius = getattr(cfg, "shake_region_radius", 0.15)

        # 药匙/洗瓶的场景 translate（__init__ 读一次；reset 时复位 + 跟随原点）
        self.spoon_orig_translate = self._read_translate(self.spoon_path)
        self.wash_orig_translate = self._read_translate(self.wash_path)

        # Episode state（reset 时重新设置）
        self.spoon_state = "rest"
        self.spoon_attach_gripper_pos = None
        self._powder_revealed = False
        self.wash_state = "rest"
        self.wash_attach_gripper_pos = None
        self.water_added = False
        self.dissolved = False
        self.obs_done = False
        self.scoop_counter = 0
        self.transfer_counter = 0
        self.pour_counter = 0
        self.shake_counter = 0
        self.obs_counter = 0

        # 世界参考点（reset 时计算）
        self.spoon_grasp_pos = None
        self.powder_center = None
        self.powder_scoop_pos = None
        self.tube_mouth_pos = None
        self.tube_transfer_pos = None
        self.wash_grasp_pos = None
        self.wash_lift_pos = None
        self.wash_pour_pos = None
        self.shake_pos = None
        self.table_z = None

    def reset(self):
        """Reset the task state: spoon/wash back to origin, samples hidden."""
        super().reset()
        self.robot.initialize()

        self.object_utils.set_object_position(
            object_path=self.spoon_path, position=self.spoon_orig_translate.copy()
        )
        self.object_utils.set_object_position(
            object_path=self.wash_path, position=self.wash_orig_translate.copy()
        )
        self._set_prim_visible(self.powder_on_spoon_path, False)
        self._set_prim_visible(self.tube_sample_path, False)
        self._set_prim_visible(self.tube_water_path, False)

        self.spoon_state = "rest"
        self.spoon_attach_gripper_pos = None
        self._powder_revealed = False
        self.wash_state = "rest"
        self.wash_attach_gripper_pos = None
        self.water_added = False
        self.dissolved = False
        self.obs_done = False
        self.scoop_counter = 0
        self.transfer_counter = 0
        self.pour_counter = 0
        self.shake_counter = 0
        self.obs_counter = 0

        # --- 静态世界参考点（本 episode 不变）---
        # 桌面高度 = 粉末堆根 z（堆底坐在桌面上）
        self.table_z = self.object_utils.get_object_xform_position(self.powder_path)[2]
        # 药匙抓取位 = 手柄中点（asset 原点 + 手柄半高；不用 bbox 中心，避免勺头拉偏）
        spoon_pos = self.object_utils.get_object_xform_position(self.spoon_path)
        self.spoon_grasp_pos = spoon_pos + np.array([0.0, 0.0, 0.0025])
        # 粉末堆中心（堆顶 = 根 z + 0.007）
        powder_pos = self.object_utils.get_object_xform_position(self.powder_path)
        self.powder_center = powder_pos + np.array([0.0, 0.0, 0.0035])
        # 勺头插入粉末堆 = 堆中心 + 插入偏移 - 勺头偏移（gripper 目标）
        self.powder_scoop_pos = self.powder_center + self.scoop_insert_offset - self.spoon_head_offset
        # 试管口内一点（管口 z = 管底 z + 0.118）
        tube_pos = self.object_utils.get_object_xform_position(self.tube_path)
        self.tube_mouth_pos = tube_pos + np.array([0.0, 0.0, 0.115])
        # 勺头进试管口 = 管口 - 勺头偏移（gripper 目标）
        self.tube_transfer_pos = self.tube_mouth_pos - self.spoon_head_offset
        # 洗瓶抓取位（瓶颈）、举高位、试管口上方的倒水位
        self.wash_grasp_pos = self.wash_orig_translate + self.wash_grasp_offset
        self.wash_lift_pos = self.wash_grasp_pos + np.array([0.0, 0.0, 0.10])
        self.wash_pour_pos = self.tube_mouth_pos + np.array([0.0, 0.0, self.wash_pour_lift])
        # 振荡位（夹爪在试管口上方）
        self.shake_pos = self.tube_mouth_pos + self.shake_offset

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
            object_path=self.tube_path,
            additional_info={
                "spoon_grasp_position": self.spoon_grasp_pos,
                "powder_scoop_position": self.powder_scoop_pos,
                "tube_transfer_position": self.tube_transfer_pos,
                "wash_grasp_position": self.wash_grasp_pos,
                "wash_pour_position": self.wash_pour_pos,
                "shake_position": self.shake_pos,
                "spoon_state": self.spoon_state,
                "wash_state": self.wash_state,
                "water_added": self.water_added,
                "dissolved": self.dissolved,
                "obs_done": self.obs_done,
            },
        )

    # ------------------------------------------------------------------
    # 世界状态更新
    # ------------------------------------------------------------------
    def _update_world_state(self):
        """Drive spoon/wash kinematic following, sample reveal logic."""
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]  # finger distance (small = closed)

        self._update_spoon(gripper_pos, gripper_opening)
        self._update_wash(gripper_pos, gripper_opening)
        self._update_dissolving(gripper_pos)

    def _update_spoon(self, gripper_pos, gripper_opening):
        """Spoon lifecycle: rest -> attached -> scooping -> transferred -> released.

        - "rest": spoon lies on the table. Gripper near the handle center and
          closed -> "attached" (mirror origin = spoon table translate).
        - "attached": spoon mirrors the gripper's 3D motion. When the spoon
          head (gripper + head offset) dips into the powder heap for
          scoop_dwell_frames, the powder-on-spoon prim is revealed
          (state -> "scooping"). When the spoon head enters the tube mouth
          for transfer_dwell_frames, the powder moves into the tube
          (state -> "transferred").
        - "transferred": gripper opens -> "released" (spoon settles on table).
        """
        if self.spoon_state == "rest":
            near_spoon = np.linalg.norm(gripper_pos - self.spoon_grasp_pos) < self.grasp_xy_threshold
            if near_spoon and gripper_opening < self.gripper_closed_threshold:
                self.spoon_state = "attached"
                self.spoon_attach_gripper_pos = gripper_pos.copy()

        elif self.spoon_state == "attached":
            delta = gripper_pos - self.spoon_attach_gripper_pos
            spoon_translate = self.spoon_orig_translate + delta
            self.object_utils.set_object_position(object_path=self.spoon_path, position=spoon_translate)

            spoon_head = gripper_pos + self.spoon_head_offset
            if not self._powder_revealed:
                # 舀取检测：勺头插入粉末堆
                if np.linalg.norm(spoon_head - self.powder_center) < 0.02:
                    self.scoop_counter += 1
                    if self.scoop_counter >= self.scoop_dwell_frames:
                        self._powder_revealed = True
                        self._set_prim_visible(self.powder_on_spoon_path, True)
                else:
                    self.scoop_counter = 0
            elif self.spoon_state == "attached":
                # 转移检测：勺头进入试管口
                if np.linalg.norm(spoon_head - self.tube_mouth_pos) < 0.02:
                    self.transfer_counter += 1
                    if self.transfer_counter >= self.transfer_dwell_frames:
                        self.spoon_state = "transferred"
                        self._set_prim_visible(self.tube_sample_path, True)
                        self._set_prim_visible(self.powder_on_spoon_path, False)
                else:
                    self.transfer_counter = 0

            # 勺上粉末跟随勺头
            if self._powder_revealed and self.spoon_state == "attached":
                powder_pos = spoon_head - np.array([0.0, 0.0, 0.002])
                self.object_utils.set_object_position(object_path=self.powder_on_spoon_path, position=powder_pos)

            # 打开即释放
            if gripper_opening > self.gripper_open_threshold:
                self.spoon_state = "released"
                self._set_prim_visible(self.powder_on_spoon_path, False)
                self._return_to_origin(self.spoon_path)

        # "released"/"transferred": nothing to do (transferred keeps the powder in tube)

    def _update_wash(self, gripper_pos, gripper_opening):
        """Wash bottle lifecycle: rest -> attached -> pouring -> released.

        - "rest": bottle stands on the table. Gripper near the neck and closed
          -> "attached" (mirror origin = bottle table translate).
        - "attached": bottle mirrors the gripper's 3D motion. When the gripper
          dwells at the pour position (above the tube mouth) for
          pour_dwell_frames, the water level in the tube is revealed
          (state -> "pouring").
        - "pouring": gripper opens -> "released" (bottle settles on table).
        """
        if self.wash_state == "rest":
            near_wash = np.linalg.norm(gripper_pos - self.wash_grasp_pos) < self.grasp_xy_threshold
            if near_wash and gripper_opening < self.gripper_closed_threshold:
                self.wash_state = "attached"
                self.wash_attach_gripper_pos = gripper_pos.copy()

        elif self.wash_state == "attached":
            delta = gripper_pos - self.wash_attach_gripper_pos
            wash_translate = self.wash_orig_translate + delta
            self.object_utils.set_object_position(object_path=self.wash_path, position=wash_translate)

            # 倒水检测：夹爪在试管口上方的倒水位 dwell
            if not self.water_added:
                if np.linalg.norm(gripper_pos - self.wash_pour_pos) < 0.03:
                    self.pour_counter += 1
                    if self.pour_counter >= self.pour_dwell_frames:
                        self.water_added = True
                        self.wash_state = "pouring"
                        self._set_prim_visible(self.tube_water_path, True)
                else:
                    self.pour_counter = 0

            if gripper_opening > self.gripper_open_threshold:
                self.wash_state = "released"
                self._return_to_origin(self.wash_path)

        # "released": nothing to do.

    def _update_dissolving(self, gripper_pos):
        """Reveal dissolution after shaking, then observation done."""
        # 振荡：加水完成（water_added）后才生效——避免药匙转移阶段
        # gripper 停在试管口旁时误触发溶解
        if not self.dissolved and self.water_added:
            if np.linalg.norm(gripper_pos - self.shake_pos) < self.shake_region_radius:
                self.shake_counter += 1
                if self.shake_counter >= self.shake_dwell_frames:
                    self.dissolved = True
                    # 粉末已溶解：试管内粉末隐藏，液面保留
                    self._set_prim_visible(self.tube_sample_path, False)
            else:
                self.shake_counter = 0

        # 观察完成（溶解后持续 dwell）
        if self.dissolved and not self.obs_done:
            self.obs_counter += 1
            if self.obs_counter >= self.obs_dwell_frames:
                self.obs_done = True

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _return_to_origin(self, object_path):
        """Put an object back to its origin table position (used on release)."""
        if object_path == self.spoon_path:
            target = self.spoon_orig_translate.copy()
        elif object_path == self.wash_path:
            target = self.wash_orig_translate.copy()
        else:
            return
        self.object_utils.set_object_position(object_path=object_path, position=target)

    def _set_prim_visible(self, object_path, visible: bool) -> None:
        """Show or hide a prim."""
        prim = self.stage.GetPrimAtPath(object_path)
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
