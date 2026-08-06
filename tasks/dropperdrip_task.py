import numpy as np
from pxr import Usd, UsdGeom
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class DropperDripTask(BaseTask):
    """Task definition for "drip liquid from a dropper into a test tube".

    The scene (assets/chemistry_lab/lab_005/lab_005.usd) contains:
        - <World>/Dropper        the dropper asset (stand-up, tip bottom at z=0)
        - <World>/HClBottle      the reagent bottle (aspiration source, kept from lab_001)
        - <World>/TestTube       the target tube (mouth z = translate.z + tube_height)
        - <World>/BottleLiquid   the liquid surface inside the bottle (always visible)
        - <World>/TubeDrops      the drops inside the tube (hidden until "dropped")

    This task owns the dropper's world state:
        - It drives the dropper to follow the gripper while the gripper holds it
          (kinematic following, same as the cap in IgniteLampTask).
        - It detects the dropper lifecycle purely from the gripper's observable
          state (finger distance joint7 + TCP position):
            rest -> attached : joint7 < 0.025 AND gripper near the grasp point
            attached -> squeezed : joint7 < 0.005 AND TCP xy near the bottle mouth
                                   (squeeze to expel air, performed above the mouth)
            squeezed -> filled : 0.005 <= joint7 < 0.025 AND TCP xy near the mouth
                                   (release the bulb to aspirate liquid)
            filled -> dropped : joint7 < 0.005 AND TCP xy near the tube mouth
                                   (squeeze to drip, position distinguishes this
                                    from the expel squeeze)
            dropped -> released : joint7 > 0.03 (gripper opens, dropper detached)

    IMPORTANT: both squeezes use joint7 < 0.005, so the task MUST gate them by
    TCP position (bottle mouth vs tube mouth) to tell them apart.
    """

    def __init__(self, cfg, world, stage, robot):
        """Initialize the dropper-drip task.

        Args:
            cfg: Configuration object for the task.
            world: The simulation world instance.
            stage: The USD stage for the simulation.
            robot: The robot instance used in the task.
        """
        super().__init__(cfg, world, stage, robot)

        # Prim paths.
        self.dropper_path = cfg.dropper_path
        self.bottle_path = cfg.bottle_path
        self.tube_path = cfg.tube_path
        self.tube_drops_path = cfg.tube_drops_path

        # Reference-point derivation offsets.
        # The gripper grasps the dropper by its bulb (the top part, z=0.13 of
        # the bulb range 0.115-0.15) at height grasp_height above the tip
        # (tip bottom = asset origin). While held, the tip hangs that far
        # below the TCP, so dip/target TCP heights = surface height + grasp_height.
        self.grasp_offset = np.array(getattr(cfg, "grasp_offset", [0.0, 0.0, 0.13]), dtype=float)
        self.dip_inset = getattr(cfg, "dip_inset", 0.005)      # tip goes this deep into the liquid
        self.tube_height = getattr(cfg, "tube_height", 0.12)   # tube mouth above its translate z
        self.drip_inset = getattr(cfg, "drip_inset", 0.005)    # tip goes this deep into the tube

        # Detection thresholds.
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.squeeze_close_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)
        self.release_threshold = getattr(cfg, "release_threshold", 0.025)
        self.squeeze_xy_threshold = getattr(cfg, "squeeze_xy_threshold", 0.05)

        # Episode state (set properly in reset()).
        self.dropper_state = "rest"
        # Phase-completion flags: set once a stage is truly reached and kept
        # until reset. The composite controller checks these instead of the
        # current dropper_state, because the atomic controller runs the whole
        # pick->fill->drip sequence in one pass (by the time it reports done,
        # dropper_state has advanced to "dropped").
        self.flag_picked = False
        self.flag_filled = False
        self.flag_dropped = False
        self.dropper_orig_translate = None
        self.attach_gripper_pos = None
        self.grasp_position = None
        self.dip_position = None
        self.target_position = None

    def reset(self):
        """Reset the task state, dropper position, and drops visibility."""
        super().reset()
        self.robot.initialize()

        # Put the dropper back to its standing position and hide the drops.
        if self.dropper_orig_translate is not None:
            self.object_utils.set_object_position(
                object_path=self.dropper_path, position=self.dropper_orig_translate.copy()
            )
        self._set_tube_drops_visible(False)

        self.dropper_state = "rest"
        self.flag_picked = False
        self.flag_filled = False
        self.flag_dropped = False
        self.attach_gripper_pos = None

        # --- Static world-space reference points for this episode ---
        # Grasp point: the dropper body at grasp_offset above its standing origin.
        dropper_pos = self.object_utils.get_object_xform_position(self.dropper_path)
        self.dropper_orig_translate = dropper_pos.copy()
        self.grasp_position = dropper_pos + self.grasp_offset

        # Dip point (TCP target): the bottle mouth center. The tip hangs
        # grasp_offset.z below the TCP, so TCP z = bottle_top - inset + grasp_height.
        bottle_center, bottle_top = self._world_bbox_center_top(self.bottle_path)
        self.dip_position = np.array(
            [bottle_center[0], bottle_center[1],
             bottle_top - self.dip_inset + self.grasp_offset[2]],
            dtype=float,
        )

        # Target point (TCP target): the tube mouth. The tube is referenced with
        # its bottom at translate.z, so mouth z = translate.z + tube_height.
        tube_pos = self.object_utils.get_object_xform_position(self.tube_path)
        tube_mouth_z = tube_pos[2] + self.tube_height
        self.target_position = np.array(
            [tube_pos[0], tube_pos[1], tube_mouth_z - self.drip_inset + self.grasp_offset[2]],
            dtype=float,
        )

    def step(self):
        """Execute one simulation step.

        Returns:
            dict: A dictionary containing simulation state data, or None if not ready.
        """
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        # Advance the dropper's world state (following + lifecycle detection).
        self._update_dropper()

        return self.get_basic_state_info(
            object_path=self.dropper_path,
            additional_info={
                "grasp_position": self.grasp_position,
                "dip_position": self.dip_position,
                "target_position": self.target_position,
                # Dropper lifecycle: rest -> attached -> squeezed -> filled -> dropped -> released.
                "dropper_state": self.dropper_state,
                # Phase-completion flags (sticky until reset).
                "picked": self.flag_picked,
                "filled": self.flag_filled,
                "dropped": self.flag_dropped,
            },
        )

    def _update_dropper(self):
        """Drive the dropper with the gripper and detect the lifecycle states.

        The dropper lifecycle is detected purely from the gripper's observable
        state (finger distance joint7 + TCP position):
          - "rest" -> "attached": gripper near the grasp point AND gripper closed.
          - "attached": the dropper's translate mirrors the gripper's motion.
          - "attached" -> "squeezed": gripper squeezed (< 0.005) near the bottle mouth.
          - "squeezed" -> "filled": gripper released (0.005-0.025) near the bottle mouth.
          - "filled" -> "dropped": gripper squeezed (< 0.005) near the tube mouth.
          - "dropped" -> "released": gripper opened (> 0.03); dropper returns to origin.
        """
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]  # finger distance (small = closed)
        near_grasp = np.linalg.norm(gripper_pos - self.grasp_position) < self.grasp_xy_threshold
        near_bottle = np.linalg.norm(gripper_pos[:2] - self.dip_position[:2]) < self.squeeze_xy_threshold
        near_tube = np.linalg.norm(gripper_pos[:2] - self.target_position[:2]) < self.squeeze_xy_threshold

        # The dropper mirrors the gripper's full 3D motion (delta from attach)
        # for the WHOLE held period (attached -> squeezed -> filled -> dropped),
        # not only while the state is "attached". Freezing it at the squeeze
        # point left the dropper floating over the bottle while the gripper
        # moved on to dip and drip.
        if self.dropper_state in ("attached", "squeezed", "filled", "dropped"):
            delta = gripper_pos - self.attach_gripper_pos
            self.object_utils.set_object_position(
                object_path=self.dropper_path,
                position=self.dropper_orig_translate + delta,
            )

        if self.dropper_state == "rest":
            if near_grasp and gripper_opening < self.gripper_closed_threshold:
                self.dropper_state = "attached"
                self.flag_picked = True
                self.attach_gripper_pos = gripper_pos.copy()

        elif self.dropper_state == "attached":
            # Expel squeeze happens above the bottle mouth (same xy, higher z).
            if near_bottle and gripper_opening < self.squeeze_close_threshold:
                self.dropper_state = "squeezed"

        elif self.dropper_state == "squeezed":
            # Release the bulb to aspirate: still attached (finger distance in
            # 0.005-0.025), and still above the bottle mouth.
            if near_bottle and self.squeeze_close_threshold <= gripper_opening < self.release_threshold:
                self.dropper_state = "filled"
                self.flag_filled = True

        elif self.dropper_state == "filled":
            # The drip squeeze happens above the TUBE mouth (position-gated).
            if near_tube and gripper_opening < self.squeeze_close_threshold:
                self.dropper_state = "dropped"
                self.flag_dropped = True
                self._set_tube_drops_visible(True)

        elif self.dropper_state == "dropped":
            # The gripper opens to release the dropper.
            if gripper_opening > self.gripper_open_threshold:
                self.dropper_state = "released"
                # Put the kinematic dropper back to its standing position.
                self.object_utils.set_object_position(
                    object_path=self.dropper_path, position=self.dropper_orig_translate.copy()
                )

        # "released": the dropper rests at its origin; nothing to do.

    def _world_bbox_center_top(self, object_path):
        """Return (world-space xy-center, max-z) of a prim's world bounding box."""
        prim = self.stage.GetPrimAtPath(object_path)
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = rng.GetMin(), rng.GetMax()
        center = np.array([(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2], dtype=float)
        return center, float(mx[2])

    def _set_tube_drops_visible(self, visible: bool) -> None:
        """Show or hide the drops prim inside the tube."""
        prim = self.stage.GetPrimAtPath(self.tube_drops_path)
        if prim.IsValid():
            set_prim_visibility(prim, visible)
