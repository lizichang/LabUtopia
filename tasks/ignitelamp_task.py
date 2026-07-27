import numpy as np
from pxr import Usd, UsdGeom
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class IgniteLampTask(BaseTask):
    """Task definition for "ignite the alcohol lamp".

    The scene must contain an alcohol lamp asset (see assets/chemistry_lab/alcohol_lamp.usd)
    referenced into the scene USD. The lamp provides these prims (relative to the lamp root):
        - <lamp>/cap          the removable cap (driven kinematically by this task)
        - <lamp>/body         the lamp body (static visual)
        - <lamp>/wick_tip     the exposed wick (ignition reference point)
        - <lamp>/flame_outer  outer flame cone (hidden until ignition)
        - <lamp>/flame_inner  inner flame cone (hidden until ignition)

    This task owns all world state for the lamp:
        - It drives the cap to follow the gripper while the gripper holds it
          (kinematic driving, detected from the gripper's position + openness).
        - It reveals the flame when the gripper dwells beside the wick.

    The robot controller (IgniteLampTaskController) only produces gripper motion;
    it reads cap_state / flame_on from the state dict to decide phase success.
    """

    def __init__(self, cfg, world, stage, robot):
        """Initialize the ignite-lamp task.

        Args:
            cfg: Configuration object for the task.
            world: The simulation world instance.
            stage: The USD stage for the simulation.
            robot: The robot instance used in the task.
        """
        super().__init__(cfg, world, stage, robot)

        # Lamp prim paths (the lamp is referenced into the scene at cfg.lamp_path).
        self.lamp_path = cfg.lamp_path
        self.cap_path = f"{self.lamp_path}/cap"
        self.body_path = f"{self.lamp_path}/body"
        self.wick_path = f"{self.lamp_path}/wick_tip"
        self.flame_paths = [
            f"{self.lamp_path}/flame_outer",
            f"{self.lamp_path}/flame_inner",
        ]

        # Configurable placement offsets (relative to the lamp / wick).
        self.cap_rest_offset = np.array(getattr(cfg, "cap_rest_offset", [0.12, 0.06, 0.02]), dtype=float)
        self.ignite_offset = np.array(getattr(cfg, "ignite_offset", [0.03, 0.0, 0.01]), dtype=float)

        # Detection thresholds.
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.ignite_xy_threshold = getattr(cfg, "ignite_xy_threshold", 0.02)
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 25))

        # The cap's translate when it is seated on the lamp (read once, before any edits).
        self.cap_closed_translate = self._read_translate(self.cap_path)

        # Episode state (set properly in reset()).
        self.cap_state = "closed"
        self.cap_orig_translate = self.cap_closed_translate.copy()
        self.attach_gripper_pos = None
        self.flame_on = False
        self.flame_counter = 0
        self.cap_grasp_pos = None
        self.wick_top_pos = None
        self.cap_rest_position = None
        self.ignite_position = None
        self.table_z = None

    def reset(self):
        """Reset the task state, cap position, and flame visibility."""
        super().reset()
        self.robot.initialize()

        # Put the cap back on the lamp and hide the flame for the new episode.
        self.object_utils.set_object_position(
            object_path=self.cap_path, position=self.cap_closed_translate.copy()
        )
        self._set_flame_visible(False)

        self.cap_state = "closed"
        self.cap_orig_translate = self.cap_closed_translate.copy()
        self.attach_gripper_pos = None
        self.flame_on = False
        self.flame_counter = 0

        # Static world-space reference points for this episode.
        # cap_grasp_pos: where the gripper descends to grasp the cap (cap center, seated).
        self.cap_grasp_pos = self.object_utils.get_geometry_center(object_path=self.cap_path)
        # wick_top_pos: top of the exposed wick (ignition reference).
        self.wick_top_pos = self.object_utils.get_geometry_center(object_path=self.wick_path)
        # lamp_position: world position of the lamp base (== table surface height).
        lamp_position = self.object_utils.get_object_xform_position(self.lamp_path)
        # The lamp base sits on the table, so its z is the table surface height.
        self.table_z = lamp_position[2]

        self.cap_rest_position = lamp_position + self.cap_rest_offset
        self.ignite_position = self.wick_top_pos + self.ignite_offset

    def step(self):
        """Execute one simulation step.

        Returns:
            dict: A dictionary containing simulation state data, or None if not ready.
        """
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        # Advance the lamp's world state (cap following + flame reveal).
        self._update_cap_and_flame()

        return self.get_basic_state_info(
            object_path=self.cap_path,
            additional_info={
                # Fixed grasp target for the gripper (cap center while seated).
                "cap_position": self.cap_grasp_pos,
                # Where the gripper should set the cap down.
                "cap_rest_position": self.cap_rest_position,
                # Cap lifecycle state: "closed" -> "attached" -> "placed".
                "cap_state": self.cap_state,
                # Wick top and the abstract ignition point beside it.
                "wick_position": self.wick_top_pos,
                "ignite_position": self.ignite_position,
                # Whether the flame has been revealed.
                "flame_on": self.flame_on,
            },
        )

    def _update_cap_and_flame(self):
        """Drive the cap with the gripper and reveal the flame on a dwell beside the wick.

        The cap lifecycle is detected purely from the gripper's observable state:
          - "closed"  -> "attached": gripper is near the cap AND the gripper has closed.
          - "attached": the cap's translate mirrors the gripper's motion (delta from attach).
          - "attached" -> "placed": the gripper has opened (releasing the cap).

        The flame is revealed once the gripper dwells near the ignition point for
        ignite_dwell_frames consecutive frames (this is the abstract ignition gesture).
        """
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]  # finger distance (small = closed)

        # --- Cap lifecycle ---
        if self.cap_state == "closed":
            near_cap = np.linalg.norm(gripper_pos - self.cap_grasp_pos) < self.grasp_xy_threshold
            gripper_closed = gripper_opening < self.gripper_closed_threshold
            if near_cap and gripper_closed:
                self.cap_state = "attached"
                self.attach_gripper_pos = gripper_pos.copy()

        elif self.cap_state == "attached":
            # The cap mirrors the gripper's full 3D motion (delta from the attach
            # moment). The gripper lifts straight up over the lamp, moves clear
            # horizontally, and only descends once over the table, so the cap can
            # safely follow down without ever being pushed into the lamp.
            delta = gripper_pos - self.attach_gripper_pos
            cap_translate = self.cap_orig_translate + delta
            self.object_utils.set_object_position(
                object_path=self.cap_path, position=cap_translate
            )
            # Release when the gripper opens.
            if gripper_opening > self.gripper_open_threshold:
                self.cap_state = "placed"
                # The cap is kinematic (no gravity), so it would freeze mid-air.
                # Settle it so its bottom sits exactly on the table surface.
                self._settle_cap_on_table()

        # "placed": the cap stays where it was set down; nothing to do.

        # --- Flame reveal (abstract ignition) ---
        if not self.flame_on:
            if np.linalg.norm(gripper_pos - self.ignite_position) < self.ignite_xy_threshold:
                self.flame_counter += 1
                if self.flame_counter >= self.ignite_dwell_frames:
                    self.flame_on = True
                    self._set_flame_visible(True)
            else:
                self.flame_counter = 0

    def _settle_cap_on_table(self):
        """Snap the cap vertically so its bottom rests exactly on the table.

        The cap is kinematic (no gravity/collision), so when the gripper releases
        it would otherwise freeze at whatever height the gripper left it. This
        measures the cap's current world-space bottom and shifts its translate op
        down (or up) so the bottom lands precisely on the table surface.
        """
        prim = self.stage.GetPrimAtPath(self.cap_path)
        if not prim.IsValid():
            return

        # Current world-space bottom of the cap geometry.
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        world_bottom = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]

        # Vertical correction to bring the cap bottom to the table surface.
        dz = self.table_z - world_bottom
        current_translate = self._read_translate(self.cap_path)
        settled_translate = current_translate.copy()
        settled_translate[2] = current_translate[2] + dz
        self.object_utils.set_object_position(
            object_path=self.cap_path, position=settled_translate
        )

    def _set_flame_visible(self, visible: bool) -> None:
        """Show or hide the flame prims."""
        for flame_path in self.flame_paths:
            prim = self.stage.GetPrimAtPath(flame_path)
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
