"""焰色反应任务：夹取铂丝 → 移到本生灯口 → 停留点火 → 火焰按物质变色。

场景须包含（见 assets/chemistry_lab/bunsen_burner.usd / platinum_wire.usd）：
    <burner>/flame_outer_{color}   6 色外焰 Cone 变体（yellow/purple/green/red/
                                  orange/blue，各自绑定固定颜色材质，初始隐藏）
    <burner>/flame_inner           内焰 Cone（白热芯，点火后 reveal）
    焰色实现：点火时仅显示 flame_outer_{flame_color} 变体（visibility 切换）。
    注意：不要在运行时改 Shader 的 diffuseColor/emissiveColor —— UsdPreviewSurface
    input 改动不会传导到 RTX 渲染器，火焰会渲染成白色（2026-08 实测验证）。
    铂丝为 kinematic：任务检测夹爪靠近手柄并闭合后，让铂丝跟随夹爪移动
    （与 IgniteLampTask 的 cap 跟随同一机制）。
"""
import numpy as np
from pxr import Usd, UsdGeom
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class FlameTestTask(BaseTask):
    """Task definition for the flame test (焰色反应) on a bunsen burner."""

    # 常见金属离子焰色表（LLM 实验日志中记录的观察颜色）
    FLAME_COLORS = {
        "yellow": (1.00, 0.85, 0.30),   # Na 钠：亮黄
        "purple": (0.80, 0.45, 1.00),   # K 钾：紫色（隔钴玻璃观察）
        "green": (0.35, 0.95, 0.40),    # Cu 铜：绿色
        "red": (1.00, 0.35, 0.25),      # Ca 钙：砖红 / Sr 锶：洋红
        "orange": (1.00, 0.60, 0.15),   # 默认橙黄火焰
        "blue": (0.30, 0.60, 1.00),     # 冷色火焰（本生灯无色焰）
    }

    def __init__(self, cfg, world, stage, robot):
        """Initialize the flame test task.

        Args:
            cfg: Configuration object for the task.
            world: The simulation world instance.
            stage: The USD stage for the simulation.
            robot: The robot instance used in the task.
        """
        super().__init__(cfg, world, stage, robot)

        self.burner_path = cfg.burner_path
        self.wire_path = cfg.wire_path

        # 铂丝手柄中心相对铂丝场景 translate 的偏移（夹爪抓手柄）
        self.grasp_offset = np.array(getattr(cfg, "grasp_offset", [0.0, 0.0, 0.06]), dtype=float)
        # 夹爪点火位相对本生灯根的偏移（丝顶端伸到灯口火焰区）
        self.ignite_offset = np.array(getattr(cfg, "ignite_offset", [0.03, 0.0, 0.088]), dtype=float)
        # 铂丝放回桌面的水平偏移（相对初始位；z 由 settle 自动校正）
        self.wire_rest_offset = np.array(getattr(cfg, "wire_rest_offset", [0.0, -0.12, 0.0]), dtype=float)

        # 焰色（yaml 配置，如 yellow/purple/green/red）
        self.flame_color = getattr(cfg, "flame_color", "yellow")

        # 检测阈值
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.ignite_xy_threshold = getattr(cfg, "ignite_xy_threshold", 0.025)
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 25))

        # Episode state
        self.wire_state = "rest"          # rest -> attached -> released
        # 铂丝场景 translate（根，即桌面落点；在 __init__ 读取一次，reset 时复位用）
        self.wire_orig_translate = self._read_translate(self.wire_path)
        self.attach_gripper_pos = None
        self.flame_on = False
        self.flame_counter = 0
        self.wire_grasp_position = None
        self.wire_rest_position = None
        self.ignite_position = None
        self.table_z = None

    def reset(self):
        """Reset the task state: wire back to rest, flame hidden."""
        super().reset()
        self.robot.initialize()

        # 铂丝放回初始位，火焰隐藏
        self.object_utils.set_object_position(
            object_path=self.wire_path, position=self.wire_orig_translate.copy()
        )
        self._set_flame_visible(False)

        self.wire_state = "rest"
        self.attach_gripper_pos = None
        self.flame_on = False
        self.flame_counter = 0

        # 静态参考点（wire_orig_translate 在 __init__ 读取一次，reset 里勿重读，否则会把复位前已移动的位置当原点）
        self.wire_grasp_position = self.wire_orig_translate + self.grasp_offset
        self.wire_rest_position = self.wire_orig_translate + self.wire_rest_offset
        # 本生灯根位置（桌面高度）
        burner_position = self.object_utils.get_object_xform_position(self.burner_path)
        self.table_z = burner_position[2]
        self.ignite_position = burner_position + self.ignite_offset

    def step(self):
        """Execute one simulation step.

        Returns:
            dict: State dictionary, or None if not ready.
        """
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        self._update_wire_and_flame()

        return self.get_basic_state_info(
            object_path=self.wire_path,
            additional_info={
                "wire_grasp_position": self.wire_grasp_position,
                "wire_rest_position": self.wire_rest_position,
                "wire_state": self.wire_state,
                "ignite_position": self.ignite_position,
                "flame_on": self.flame_on,
                "flame_color": self.flame_color,
            },
        )

    def _update_wire_and_flame(self):
        """Drive the wire with the gripper; reveal a colored flame on dwell.

        Wire lifecycle (kinematic, same as the lamp cap):
          - "rest" -> "attached": gripper near the handle AND gripper closed.
          - "attached": the wire translate mirrors the gripper's delta motion.
          - "attached" -> "released": the gripper opens (wire settles to table).

        Flame: revealed (with the configured flame color) once the gripper
        dwells near the burner mouth for ignite_dwell_frames consecutive frames.
        """
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]

        # --- Wire lifecycle ---
        if self.wire_state == "rest":
            near_handle = np.linalg.norm(gripper_pos - self.wire_grasp_position) < self.grasp_xy_threshold
            gripper_closed = gripper_opening < self.gripper_closed_threshold
            if near_handle and gripper_closed:
                self.wire_state = "attached"
                self.attach_gripper_pos = gripper_pos.copy()

        elif self.wire_state == "attached":
            delta = gripper_pos - self.attach_gripper_pos
            wire_translate = self.wire_orig_translate + delta
            self.object_utils.set_object_position(
                object_path=self.wire_path, position=wire_translate
            )
            if gripper_opening > self.gripper_open_threshold:
                self.wire_state = "released"
                self._settle_wire_on_table()

        # --- Flame reveal (abstract ignition, with flame color) ---
        if not self.flame_on:
            if np.linalg.norm(gripper_pos - self.ignite_position) < self.ignite_xy_threshold:
                self.flame_counter += 1
                if self.flame_counter >= self.ignite_dwell_frames:
                    self.flame_on = True
                    # 焰色通过显示预制的颜色变体 flame_outer_{color} 实现
                    # （运行时改 Shader input 不会传导到渲染器）
                    self._set_flame_visible(True)
            else:
                self.flame_counter = 0

    def _settle_wire_on_table(self):
        """Snap the wire vertically so its bottom rests exactly on the table."""
        prim = self.stage.GetPrimAtPath(self.wire_path)
        if not prim.IsValid():
            return
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        world_bottom = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]
        dz = self.table_z - world_bottom
        current = self.object_utils.get_object_xform_position(self.wire_path)
        settled = current.copy()
        settled[2] = current[2] + dz
        self.object_utils.set_object_position(object_path=self.wire_path, position=settled)

    def _set_flame_visible(self, visible: bool) -> None:
        """Show or hide the flame prims.

        The flame color is realized by switching pre-authored color variants
        (flame_outer_{color}) instead of editing Shader inputs at runtime —
        Isaac Sim does not propagate runtime UsdPreviewSurface input changes
        to the RTX renderer (verified 2026-08: diffuseColor/emissiveColor
        edits left the flame white), while prim visibility does take effect.
        The inner white-hot core is shown together with the outer flame.
        """
        for color in self.FLAME_COLORS:
            prim = self.stage.GetPrimAtPath(f"{self.burner_path}/flame_outer_{color}")
            if prim.IsValid():
                set_prim_visibility(prim, False)

        inner = self.stage.GetPrimAtPath(f"{self.burner_path}/flame_inner")
        if inner.IsValid():
            set_prim_visibility(inner, visible)

        if visible:
            prim = self.stage.GetPrimAtPath(
                f"{self.burner_path}/flame_outer_{self.flame_color}"
            )
            if prim.IsValid():
                set_prim_visibility(prim, True)

    def _read_translate(self, object_path: str) -> np.ndarray:
        """Read the translate xformOp value of a prim (defaults to zeros)."""
        prim = self.stage.GetPrimAtPath(object_path)
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    return np.array(op.Get(), dtype=float)
        return np.zeros(3)
