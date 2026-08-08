"""焰色反应任务（V7 文档 C1，13 步，一步一事件）。

13 步（严格按 LabUtopia_Action_Catalogue_v7 文档 C1）：
  P1 取表面皿置中央      P2 旋开稀盐酸瓶磨口塞   P3 滴管吸盐酸滴 2-3 滴（滴完盖塞归位）
  P4 点燃本生灯（蓝焰）  P5 铂丝蘸酸            P6 外焰灼烧
  P7 反复蘸酸+灼烧 3 次  P8 冷却 5s             P9 开样品瓶蘸粉末
  P10 灼烧 2-5s（受染）  P11 灯帽盖灭           P12 冲洗擦干归位
  P13 表面皿清洗归位

v16 修正（试管架布局 + per-object 夹爪阈值）：
  - 铂丝改为竖直放在试管架上（rotateY=0°），origin 在手柄底部(0.536,-0.14,0.812)
  - 滴管竖直放在试管架孔中(0.488,-0.14,0.812)，origin 在管口
  - WIRE_TIP_OFFSET 改为竖直方向：(0, 0, 0.1095)
  - DROPPER_NOZZLE_OFFSET 改为：(0, 0, -0.06)
  - 夹爪阈值收紧到物体直径：dish 0.005, stopper 0.015, dropper 0.005, match 0.003, cap 0.020, wire 0.005
  - 修复 stoppers 双重偏移 bug：stopper 局部几何中心在 z=0.0735
  - FLAME_Z 适配缩小后火焰（外焰 z[0.958,1.004]）
  - stain 锥由 controller 定位到铂丝尖端，仅尖端周围 1.2cm 黄色光晕
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class FlameTestTask(BaseTask):
    """Task definition for the 13-step flame test (焰色反应) on a bunsen burner."""

    FLAME_COLORS = {
        "yellow": (1.00, 0.85, 0.30),
        "purple": (0.80, 0.45, 1.00),
        "green": (0.35, 0.95, 0.40),
        "red": (1.00, 0.35, 0.25),
        "orange": (1.00, 0.60, 0.15),
        "blue": (0.30, 0.60, 1.00),
    }

    TABLE_Z = 0.80

    # ---- 铂丝（rotateY=120°，手柄穿过试管架孔板，丝/环挂在下方）----
    # origin 在手柄底部(0.488,-0.14,0.895)，在孔板顶上方 8mm
    # sin120=0.866, cos120=-0.5
    # 手柄中心 local(0,0,0.056) → 旋转后(0.0485,0,-0.028) → 世界(0.537,-0.14,0.867)
    # loop local(0,0,0.1655) → 旋转后(0.1433,0,-0.0828) → 世界(0.631,-0.14,0.812)
    WIRE_GRASP = np.array([0.537, -0.14, 0.867])
    WIRE_REST = np.array([0.488, -0.14, 0.895])
    WIRE_HELD_OFFSET = WIRE_REST - WIRE_GRASP  # (-0.049, 0, 0.028)
    # loop 相对夹爪 = (0.1433-0.0485, 0, -0.0828+0.028) = (0.0948, 0, -0.0548)
    WIRE_TIP_OFFSET = np.array([0.095, 0.0, -0.055])

    # ---- 各物体抓取点（夹爪 TCP 位置，世界坐标）----
    # 滴管：origin 在管口(0.536,-0.14,0.812)，玻璃管 z[0.812,0.932]，夹在 z=0.872
    # 瓶塞：世界中心 z=0.8735，夹在近顶部 z=0.877
    # 火柴(rotY=180)：杆中心世界 (0.4585, 0.24, 0.803)
    # 灯帽：世界中心 (0.46,0.28,0.82)，夹在近顶部 z=0.825
    GRASP_POINTS = {
        "dish":           np.array([0.32,   -0.22, 0.803]),
        "hcl_stopper":    np.array([0.12,    0.02, 0.877]),
        "dropper":        np.array([0.536,  -0.14, 0.872]),
        "sample_stopper": np.array([0.20,    0.12, 0.877]),
        "match":          np.array([0.4585,  0.24, 0.803]),
        "cap":            np.array([0.46,    0.28, 0.825]),
    }

    # ---- 物体静止位置（世界坐标，对于子物体指几何中心）----
    REST_POS = {
        "dish":           np.array([0.32,   -0.22, 0.80]),
        "hcl_stopper":    np.array([0.12,    0.02, 0.8735]),
        "dropper":        np.array([0.536,  -0.14, 0.812]),
        "sample_stopper": np.array([0.20,    0.12, 0.8735]),
        "match":          np.array([0.50,    0.24, 0.8013]),
        "cap":            np.array([0.46,    0.28, 0.82]),
    }

    # ---- 子物体局部几何中心偏移（raw mesh center relative to prim origin）----
    # stoppers 的 mesh 顶点在局部 z[0.068,0.079]，中心 0.0735；cap 的圆柱以原点为中心
    LOCAL_GEOM_OFFSET = {
        "dish":           np.array([0.0, 0.0, 0.0]),
        "hcl_stopper":    np.array([0.0, 0.0, 0.0735]),
        "dropper":        np.array([0.0, 0.0, 0.0]),
        "sample_stopper": np.array([0.0, 0.0, 0.0735]),
        "match":          np.array([0.0, 0.0, 0.0]),
        "cap":            np.array([0.0, 0.0, 0.0]),
    }

    # ---- 夹持偏移：物体 prim 原点相对夹爪的位置 = REST_POS - GRASP_POINT ----
    HELD_OFFSETS = {
        "dish":           REST_POS["dish"]           - GRASP_POINTS["dish"],
        "hcl_stopper":    REST_POS["hcl_stopper"]    - GRASP_POINTS["hcl_stopper"],
        "dropper":        REST_POS["dropper"]        - GRASP_POINTS["dropper"],
        "sample_stopper": REST_POS["sample_stopper"] - GRASP_POINTS["sample_stopper"],
        "match":          REST_POS["match"]          - GRASP_POINTS["match"],
        "cap":            REST_POS["cap"]            - GRASP_POINTS["cap"],
    }

    # ---- 每物体夹爪闭合阈值（joint7 单指位移 < 此值才算夹紧）----
    # controller 设置 joint7 = grip_val；总宽 = 2*joint7
    # 阈值 = grip值 + 0.002~0.005 裕量，确保 controller 设 grip 后 task 能检测到"夹紧"
    # grip 值：dish 0.002, stopper 0.011, dropper 0.003, match 0.0015, cap 0.015, wire 0.003
    GRIP_CLOSED_THRESH = {
        "dish":           0.005,   # 表面皿 ~3mm, grip=0.002
        "hcl_stopper":    0.015,   # 瓶塞 ~25mm, grip=0.011
        "dropper":        0.005,   # 滴管 8mm, grip=0.003
        "sample_stopper": 0.015,   # 瓶塞 ~25mm, grip=0.011
        "match":          0.003,   # 火柴 3mm, grip=0.0015
        "cap":            0.020,   # 灯帽 ~34mm, grip=0.015
        "wire":           0.005,   # 铂丝手柄 8mm, grip=0.003
    }

    # ---- 关键点 ----
    BURNER_POS = np.array([0.36, 0.18, 0.80])
    # 缩小后火焰：外焰 z[0.958, 1.004]，外焰有效区 z[0.968, 1.000]
    FLAME_Z = (0.968, 1.000)
    IGNITE_POS = np.array([0.36, 0.18, 0.96])
    # 火柴 rotY=180：头在 origin -x 方向 0.0894；HELD_OFFSET x=0.0415
    # 头相对夹爪 = 0.0415 - 0.0894 = -0.0479
    MATCH_TIP_OFFSET = np.array([-0.048, 0.0, 0.0])
    # 滴管管口在 origin(z=0.812)，夹在 z=0.872，管口相对夹爪 z = -0.06
    DROPPER_NOZZLE_OFFSET = np.array([0.0, 0.0, -0.06])
    # 铂丝 WIRE_TIP_OFFSET=(0.095,0,-0.055)，环中心 = gripper + offset
    # 火柴 MATCH_TIP_OFFSET 不变
    DISH_CENTER = np.array([0.20, 0.02])
    HCL_MOUTH = np.array([0.12, 0.02])
    SAMPLE_MOUTH = np.array([0.20, 0.12])
    WASH_NOZZLE = np.array([0.40, -0.10])
    JET_POS = np.array([0.40, -0.10, 0.885])
    # 盖灭后灯帽放在灯管上（帽底 z=0.958，中心 z=0.968）
    CAP_SETTLED_POS = np.array([0.36, 0.18, 0.968])

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

        self.dish_path = cfg.dish_path
        self.hcl_path = cfg.hcl_path
        self.hcl_stopper_path = self.hcl_path + "/stopper"
        self.dropper_path = cfg.dropper_path
        self.wire_path = cfg.wire_path
        self.sample_path = cfg.sample_path
        self.sample_stopper_path = self.sample_path + "/stopper"
        self.burner_path = cfg.burner_path
        self.cap_path = self.burner_path + "/cap"
        self.match_path = cfg.match_path
        self.droplet_path = cfg.droplet_path
        self.jet_path = cfg.jet_path
        self.dish_acid_path = cfg.dish_acid_path

        self.flame_color = getattr(cfg, "flame_color", "yellow")

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 15))
        self.stain_dwell_frames = int(getattr(cfg, "stain_dwell_frames", 150))
        self.extinguish_dwell_frames = int(getattr(cfg, "extinguish_dwell_frames", 15))
        self.drop_dwell_frames = int(getattr(cfg, "drop_dwell_frames", 20))
        self.drop_interval_frames = int(getattr(cfg, "drop_interval_frames", 30))
        self.n_drops = int(getattr(cfg, "n_drops", 3))

        self.kin_objs = {
            "dish":           {"path": self.dish_path,           "parent": None},
            "hcl_stopper":    {"path": self.hcl_stopper_path,    "parent": self.hcl_path},
            "dropper":        {"path": self.dropper_path,        "parent": None},
            "sample_stopper": {"path": self.sample_stopper_path, "parent": self.sample_path},
            "match":          {"path": self.match_path,          "parent": None},
            "cap":            {"path": self.cap_path,            "parent": self.burner_path},
        }
        self.wire_state = "rest"
        self._reset_kin_states()

    def _reset_kin_states(self):
        for name in self.kin_objs:
            self.kin_objs[name]["state"] = "rest"

    def reset(self):
        super().reset()
        self.robot.initialize()

        self.object_utils.set_object_position(
            object_path=self.wire_path, position=self.WIRE_REST.copy()
        )
        for name in self.kin_objs:
            self._set_obj_world(name, self.REST_POS[name])
        self._set_flame_visible(False)
        self._set_stain(False)
        self._set_visibility(self.droplet_path, False)
        self._set_visibility(self.jet_path, False)
        self._set_visibility(self.dish_acid_path, False)
        self._set_visibility(self.dish_path + "/powder", False)

        self.wire_state = "rest"
        self.flame_on = False
        self.stain_on = False
        self.ignite_counter = 0
        self.stain_counter = 0
        self.extinguish_counter = 0
        self.drop_counter = 0
        self.drop_interval = 0
        self.n_dropped = 0
        self.droplet_flash = 0
        self.jet_counter = 0
        self.jet_on = False
        self._reset_kin_states()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_objects_and_events()
        return self.get_basic_state_info(
            object_path=self.wire_path,
            additional_info={
                "flame_on": self.flame_on,
                "stain_on": self.stain_on,
                "flame_color": self.flame_color,
                "n_dropped": self.n_dropped,
                "wire_state": self.wire_state,
            },
        )

    # ------------------------------------------------------------------
    # 每帧更新
    # ------------------------------------------------------------------
    def _update_objects_and_events(self):
        gripper_pos = self.robot.get_gripper_position()
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]

        self._update_kin_objects(gripper_pos, gripper_opening)
        self._update_wire(gripper_pos, gripper_opening)
        self._update_effects(gripper_pos)

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.03):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _update_kin_objects(self, gripper_pos, gripper_opening):
        for name, obj in self.kin_objs.items():
            if obj["state"] == "settled":
                continue

            if obj["state"] == "rest":
                grasp = self.GRASP_POINTS[name]
                closed_thresh = self.GRIP_CLOSED_THRESH[name]
                if self._near_grasp(gripper_pos, grasp) and gripper_opening < closed_thresh:
                    obj["state"] = "attached"
                    self._set_obj_world(name, gripper_pos + self.HELD_OFFSETS[name])
                    print(f"[flametest] attached {name} (grip={gripper_opening:.4f} < {closed_thresh})")

            elif obj["state"] == "attached":
                self._set_obj_world(name, gripper_pos + self.HELD_OFFSETS[name])
                if gripper_opening > self.gripper_open_threshold:
                    obj["state"] = "released"
                    self._settle_object(name)
                    print(f"[flametest] released {name}")

    def _update_wire(self, gripper_pos, gripper_opening):
        if self.wire_state == "rest":
            if (self._near_grasp(gripper_pos, self.WIRE_GRASP)
                    and gripper_opening < self.GRIP_CLOSED_THRESH["wire"]):
                self.wire_state = "attached"
                self.object_utils.set_object_position(
                    object_path=self.wire_path,
                    position=gripper_pos + self.WIRE_HELD_OFFSET,
                )
                print("[flametest] wire attached (snap-to-grip)")

        elif self.wire_state == "attached":
            self.object_utils.set_object_position(
                object_path=self.wire_path,
                position=gripper_pos + self.WIRE_HELD_OFFSET,
            )
            if (gripper_opening > self.gripper_open_threshold
                    and np.linalg.norm(gripper_pos - self.WIRE_GRASP) < 0.05):
                self.object_utils.set_object_position(
                    object_path=self.wire_path, position=self.WIRE_REST.copy()
                )
                self.wire_state = "rest"
                print("[flametest] wire returned to rest")

    def _update_effects(self, gripper_pos):
        # ---- 1. 点燃 ----
        if not self.flame_on:
            if self.kin_objs["match"]["state"] == "attached":
                tip = self._match_tip(gripper_pos)
                if np.linalg.norm(tip - self.IGNITE_POS) < 0.035:
                    self.ignite_counter += 1
                    if self.ignite_counter >= self.ignite_dwell_frames:
                        self.flame_on = True
                        self._set_flame_visible(True)
                        print("[flametest] flame ignited (blue)")
                else:
                    self.ignite_counter = 0
            else:
                self.ignite_counter = 0

        # ---- 2. 滴液 ----
        if self.kin_objs["dropper"]["state"] == "attached":
            nozzle = self._nozzle(gripper_pos)
            above_dish = (np.linalg.norm(nozzle[:2] - self.DISH_CENTER) < 0.035
                          and abs(nozzle[2] - 0.85) < 0.04)
            if above_dish:
                self.drop_counter += 1
                if self.drop_counter >= self.drop_dwell_frames and self.n_dropped < self.n_drops:
                    self.drop_interval += 1
                    if self.drop_interval >= self.drop_interval_frames:
                        self.n_dropped += 1
                        self.drop_interval = 0
                        self.droplet_flash = 8
                        self._set_obj_world_plain(
                            self.droplet_path, nozzle + np.array([0.0, 0.0, -0.02]))
                        if self.n_dropped >= self.n_drops:
                            self._set_visibility(self.dish_acid_path, True)
                        print(f"[flametest] drop {self.n_dropped}/{self.n_drops}")
            else:
                self.drop_counter = 0
                self.drop_interval = 0
        else:
            self.drop_counter = 0
            self.drop_interval = 0
        if self.droplet_flash > 0:
            self.droplet_flash -= 1
        self._set_visibility(self.droplet_path, self.droplet_flash > 0)

        # ---- 3. 受染：铂丝尖端在外焰 -> 局部黄色光晕跟随尖端 ----
        if self.flame_on and self.wire_state == "attached":
            tip = gripper_pos + self.WIRE_TIP_OFFSET
            in_flame = (np.linalg.norm(tip[:2] - self.BURNER_POS[:2]) < 0.035
                        and self.FLAME_Z[0] < tip[2] < self.FLAME_Z[1])
            if in_flame:
                self.stain_counter += 1
                if self.stain_counter >= self.stain_dwell_frames:
                    if not self.stain_on:
                        self.stain_on = True
                        self._set_stain(True)
                        print(f"[flametest] stain {self.flame_color} revealed")
                    self._position_stain_at_tip(tip)
            else:
                if self.stain_counter > 0 or self.stain_on:
                    self.stain_counter = 0
                    if self.stain_on:
                        self.stain_on = False
                        self._set_stain(False)
        elif self.stain_on:
            self.stain_on = False
            self._set_stain(False)

        # ---- 4. 灭焰 ----
        if self.flame_on and self.kin_objs["cap"]["state"] == "attached":
            if np.linalg.norm(gripper_pos - np.array([0.36, 0.18, 0.97])) < 0.04:
                self.extinguish_counter += 1
                if self.extinguish_counter >= self.extinguish_dwell_frames:
                    self.flame_on = False
                    self.stain_on = False
                    self._set_flame_visible(False)
                    self._set_stain(False)
                    self._set_obj_world("cap", self.CAP_SETTLED_POS)
                    self.kin_objs["cap"]["state"] = "settled"
                    print("[flametest] flame extinguished, cap on burner")
            else:
                self.extinguish_counter = 0

        # ---- 5. 水柱 ----
        jet_cond = False
        if self.wire_state == "attached":
            tip = gripper_pos + self.WIRE_TIP_OFFSET
            if (np.linalg.norm(tip[:2] - self.WASH_NOZZLE) < 0.035
                    and tip[2] < 0.88):
                jet_cond = True
        if (self.kin_objs["dish"]["state"] == "attached"
                and np.linalg.norm(gripper_pos[:2] - self.WASH_NOZZLE) < 0.035):
            jet_cond = True
        if jet_cond:
            self.jet_counter += 1
            if self.jet_counter >= 5 and not self.jet_on:
                self.jet_on = True
                self._set_obj_world_plain(self.jet_path, self.JET_POS)
                self._set_visibility(self.jet_path, True)
        else:
            self.jet_counter = 0
            if self.jet_on:
                self.jet_on = False
                self._set_visibility(self.jet_path, False)

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _match_tip(self, gripper_pos):
        return gripper_pos + self.MATCH_TIP_OFFSET

    def _nozzle(self, gripper_pos):
        return gripper_pos + self.DROPPER_NOZZLE_OFFSET

    def _get_obj_world(self, name):
        """获取物体几何中心世界坐标（对子物体补偿 local_geom_offset）。"""
        obj = self.kin_objs[name]
        world_origin = self.object_utils.get_object_xform_position(obj["path"])
        if world_origin is None:
            return self.REST_POS[name].copy()
        return world_origin + self.LOCAL_GEOM_OFFSET[name]

    def _set_obj_world(self, name, world):
        """设置物体几何中心世界坐标（对子物体减去 local_geom_offset）。"""
        obj = self.kin_objs[name]
        if obj["parent"] is None:
            self.object_utils.set_object_position(object_path=obj["path"], position=world)
        else:
            parent_t = self.object_utils.get_object_xform_position(obj["parent"])
            local_t = world - parent_t - self.LOCAL_GEOM_OFFSET[name]
            self.object_utils.set_object_position(
                object_path=obj["path"], position=local_t)

    def _set_obj_world_plain(self, path, world):
        self.object_utils.set_object_position(object_path=path, position=world)

    def _settle_object(self, name):
        if name == "dish":
            cur = self._get_obj_world(name)
            cur[2] = self.TABLE_Z
            self._set_obj_world(name, cur)
            self._set_visibility(self.dish_acid_path, False)
        elif name == "hcl_stopper":
            cur = self._get_obj_world(name)
            if np.linalg.norm(cur[:2] - self.HCL_MOUTH) < 0.03:
                cur[:] = self.REST_POS[name]
            else:
                cur[2] = self.TABLE_Z + 0.006
            self._set_obj_world(name, cur)
        elif name == "sample_stopper":
            cur = self._get_obj_world(name)
            if np.linalg.norm(cur[:2] - self.SAMPLE_MOUTH) < 0.03:
                cur[:] = self.REST_POS[name]
            else:
                cur[2] = self.TABLE_Z + 0.006
            self._set_obj_world(name, cur)
        elif name in ("dropper", "match"):
            self._set_obj_world(name, self.REST_POS[name])
        self.kin_objs[name]["state"] = "rest"

    def _set_flame_visible(self, visible: bool) -> None:
        for prim_name in ("flame_outer", "flame_inner"):
            prim = self.stage.GetPrimAtPath(f"{self.burner_path}/{prim_name}")
            if prim.IsValid():
                set_prim_visibility(prim, visible)

    def _position_stain_at_tip(self, tip_world):
        """将受染锥中心定位到铂丝尖端（burner 局部坐标）。
        stain xform op 顺序为 [translate, scale]，中心 = translate。"""
        local = tip_world - self.BURNER_POS
        prim = self.stage.GetPrimAtPath(
            f"{self.burner_path}/flame_stain_{self.flame_color}")
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(float(local[0]), float(local[1]), float(local[2])))
                    break

    def _set_stain(self, visible: bool) -> None:
        for color in self.FLAME_COLORS:
            prim = self.stage.GetPrimAtPath(f"{self.burner_path}/flame_stain_{color}")
            if prim.IsValid():
                set_prim_visibility(prim, False)
        if visible:
            prim = self.stage.GetPrimAtPath(
                f"{self.burner_path}/flame_stain_{self.flame_color}")
            if prim.IsValid():
                set_prim_visibility(prim, True)

    def _set_visibility(self, path, visible):
        try:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass
