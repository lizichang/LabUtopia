"""焰色反应任务（V7 文档 C1，13 步，一步一事件）。

13 步（严格按 LabUtopia_Action_Catalogue_v7 文档 C1）：
  P1 取表面皿置中央      P2 旋开稀盐酸瓶磨口塞   P3 滴管吸盐酸滴 2-3 滴（滴完盖塞归位）
  P4 点燃本生灯（蓝焰）  P5 铂丝蘸酸            P6 外焰灼烧
  P7 反复蘸酸+灼烧 3 次  P8 冷却 5s             P9 开样品瓶蘸粉末
  P10 灼烧 2-5s（受染）  P11 灯帽盖灭           P12 冲洗擦干归位
  P13 表面皿清洗归位

v21 修正（修复多物体同时附着 + 一次只抓一个 + 收紧 z 阈值）：
  - _near_grasp z_thresh 从 0.03 收紧至 0.015（1.5cm）
  - 新增 _any_obj_attached()：任何 kin_obj 或 wire 处于 attached 时禁止新附着
  - GRIP_CLOSED_THRESH 收紧裕量（从 grip+2mm 改为 grip+1mm）
  - 修复：抓铂丝时滴管同时附着（两者在试管架上仅相距 5mm）
  - 修复：伸入盐酸瓶时瓶塞误附着（瓶塞阈值 0.015 过宽松）
  - max_steps 从 15000 增至 30000（确保 13 phase 全部完成）

v22 修正（场景结构 + 可及性 + 夹爪宽度，配套 scripts/fix_flametest_v17.py）：
  - v17 USD 修复为 defaultPrim token "World"（否则 kit 的 add_reference 解析失败）
  - 子 prim 路径对齐 v17 实际命名：stopper_020 / stopper_021 / cap_004_011 /
    powder_002_002
  - 表面皿从不可及 x=0.6682 移回 (0.32,-0.22)（超出 Franka 桌面高度工作半径）
  - 夹爪宽度按 mesh extent 实测修正：dish 6.5mm、wire 11mm（原误记 8mm）

v23 修正（焰色可见性）：
  - 受染时火焰本体变成本色（yellow），不再只显示尖端染色锥——原染色锥
    r=6mm 被不透明白/蓝火焰（该高度 r≈8-10mm）完全包住，相机里看不到黄色
  - 熄灭/离开火焰/重置时恢复蓝色（FLAME_BASE_COLORS 常量）

v20 修正（试管架移入工作空间 + 夹爪开合 = 物体直径）：
  - controller joint7 = 物体直径 / 2（从 USD mesh extent 精确提取）
  - 场景用 lab_flametest_v17.usd（含 TestTubeRack at (0.38,-0.14,0.80)）
  - 铂丝 rotateY(120°) 斜置：origin 在手柄底部 (0.368,-0.14,0.895)
    手柄中心 world=(0.417,-0.14,0.867)，环中心 world=(0.511,-0.14,0.812)
  - 滴管竖直放在试管架孔中：origin 在管口 (0.416,-0.14,0.812)
  - WIRE_TIP_OFFSET=(0.095,0,-0.055)：环中心 = gripper + offset
  - DROPPER_NOZZLE_OFFSET=(0,0,-0.06)：管口 = gripper + offset
  - stain 锥由 controller 定位到铂丝尖端，仅尖端周围 1.2cm 黄色光晕
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class FlameTestTask(BaseTask):
    """Task definition for the 13-step flame test (焰色反应) on a bunsen burner."""

    FLAME_COLORS = {
        "yellow": (1.00, 0.72, 0.12),   # v23：钠焰饱和黄（原 0.85,0.30 渲染后偏白）
        "purple": (0.80, 0.45, 1.00),
        "green": (0.35, 0.95, 0.40),
        "red": (1.00, 0.35, 0.25),
        "orange": (1.00, 0.60, 0.15),
        "blue": (0.30, 0.60, 1.00),
    }
    # 火焰基色（与 v17 移植的 flame_outer_mat / flame_inner_mat 一致）
    FLAME_BASE_COLORS = {
        "outer": (0.35, 0.65, 1.0),
        "inner": (0.75, 0.9, 1.0),
    }

    TABLE_Z = 0.80

    # ---- 铂丝（rotateY=120°，手柄穿过试管架孔板，丝/环挂在下方）----
    # origin 在手柄底部(0.368,-0.14,0.895)，在孔板顶上方 8mm
    # sin120=0.866, cos120=-0.5
    # 手柄中心 local(0,0,0.056) → 旋转后(0.0485,0,-0.028) → 世界(0.417,-0.14,0.867)
    # loop local(0,0,0.1655) → 旋转后(0.1433,0,-0.0828) → 世界(0.511,-0.14,0.812)
    WIRE_REST  = np.array([0.3977, -0.0201, 0.9756])
    WIRE_GRASP = np.array([0.3977, -0.0201, 0.9476])
    WIRE_HELD_OFFSET = WIRE_REST - WIRE_GRASP  # (-0.049, 0, 0.028)
    # loop 相对夹爪 = (0.1433-0.0485, 0, -0.0828+0.028) = (0.0948, 0, -0.0548)
    WIRE_TIP_OFFSET = np.array([0.095, 0.0, -0.055])

    # ---- 各物体抓取点（夹爪 TCP 位置，世界坐标）----
    # v22 修正：表面皿从 v17 USD 的 (0.6682,-0.22) 移回 (0.32,-0.22)。
    # 原因：x=0.6682 超出 Franka（base x=-0.3）在桌面高度的实际工作半径
    # （约 x≤0.5），RMP 会卡在 x≈0.48 无法到位，导致 P1 抓不到表面皿。
    # 滴管：origin 在管口(0.3591,-0.0205,0.812)，玻璃管 z[0.812,0.932]，夹在 z=0.872
    # 瓶塞：世界中心 z=0.8735，夹在近顶部 z=0.877
    # 火柴(rotY=180)：杆中心世界 (0.5000, 0.24, 0.803)
    # 灯帽：世界中心 (0.46,0.28,0.81)，夹在近顶部 z=0.815
    GRASP_POINTS = {
        "dish":           np.array([0.3200, -0.2200, 0.8030]),
        "hcl_stopper":    np.array([0.1200,  0.0200, 0.8770]),
        "dropper":        np.array([0.3591, -0.0205, 0.8720]),
        "sample_stopper": np.array([0.2000,  0.1200, 0.8770]),
        "match":          np.array([0.5000,  0.2400, 0.8030]),
        "cap":            np.array([0.4600,  0.2800, 0.8150]),
    }

    # ---- 物体静止位置（世界坐标，对于子物体指几何中心）----
    REST_POS = {
        "dish":           np.array([0.3200, -0.2200, 0.8000]),
        "hcl_stopper":    np.array([0.1200,  0.0200, 0.8735]),
        "dropper":        np.array([0.3591, -0.0205, 0.8120]),
        "sample_stopper": np.array([0.2000,  0.1200, 0.8735]),
        "match":          np.array([0.5000,  0.2400, 0.8013]),
        "cap":            np.array([0.4600,  0.2800, 0.8100]),
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
    # controller 设置 joint7 = grip_val（= 物体直径/2）；总宽 = 2*joint7
    # 阈值 = grip值 + 1mm 裕量（v21 收紧，原 2mm 导致多物体误触发）
    # grip 值：dish 0.0035, stopper 0.0126, dropper 0.004, match 0.0015, cap 0.017, wire 0.0055
    # v22 实测修正（按 v17 USD mesh extent，总宽 = 2*grip）：
    #   dish  slab 厚 6.5mm（旧 4.2mm），grip=0.0035
    #   wire  手柄直径 11mm（旧误记 8mm），grip=0.0055
    GRIP_CLOSED_THRESH = {
        "dish":           0.0045,
        "hcl_stopper":    0.0136,
        "dropper":        0.005,
        "sample_stopper": 0.0136,
        "match":          0.0025,
        "cap":            0.018,
        "wire":           0.0065,
    }

    # ---- 关键点 ----
    BURNER_POS = np.array([0.36, 0.18, 0.80])
    # 缩小后火焰：外焰 z[0.958, 1.004]，外焰有效区 z[0.968, 1.000]
    FLAME_Z = (0.960, 1.005)   # v23：放宽 0.5cm，容忍 RMP 到位抖动
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
        self.hcl_stopper_path = self.hcl_path + "/stopper_020"
        self.dropper_path = cfg.dropper_path
        self.wire_path = cfg.wire_path
        self.sample_path = cfg.sample_path
        self.sample_stopper_path = self.sample_path + "/stopper_021"
        self.burner_path = cfg.burner_path
        self.cap_path = self.burner_path + "/cap_004_011"
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
        self._set_visibility(self.dish_path + "/powder_002_002", False)

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

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _any_obj_attached(self):
        """检查是否已有任何 kin_obj 或 wire 处于 attached 状态（一次只抓一个）。"""
        if self.wire_state == "attached":
            return True
        for name, obj in self.kin_objs.items():
            if obj["state"] == "attached":
                return True
        return False

    def _find_closest_graspable(self, gripper_pos):
        """v21: 找到离夹爪最近的可抓取物体（rest 状态的 kin_obj 或 wire）。
        返回 (type, name) 或 (None, None)。type 为 "kin" 或 "wire"。
        当滴管和铂丝在试管架上仅相距 5mm 时，确保只抓最近的那个。
        """
        candidates = []
        for name, obj in self.kin_objs.items():
            if obj["state"] == "rest":
                grasp = self.GRASP_POINTS[name]
                dist = np.linalg.norm(gripper_pos - grasp)
                candidates.append((dist, "kin", name))
        if self.wire_state == "rest":
            dist = np.linalg.norm(gripper_pos - self.WIRE_GRASP)
            candidates.append((dist, "wire", None))
        if not candidates:
            return None, None
        candidates.sort(key=lambda c: c[0])
        return candidates[0][1], candidates[0][2]

    def _update_kin_objects(self, gripper_pos, gripper_opening):
        for name, obj in self.kin_objs.items():
            if obj["state"] == "settled":
                continue

            if obj["state"] == "rest":
                # v21: 一次只抓一个物体——已有物体附着时不抓新的
                if self._any_obj_attached():
                    continue
                # v21: 只抓最近的物体（防止滴管/铂丝误抓）
                closest_type, closest_name = self._find_closest_graspable(gripper_pos)
                if closest_type != "kin" or closest_name != name:
                    continue
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
            # v21: 一次只抓一个——已有 kin_obj 附着时不抓 wire
            if self._any_obj_attached():
                return
            # v21: 只抓最近的物体（防止抓铂丝时误抓滴管）
            closest_type, _ = self._find_closest_graspable(gripper_pos)
            if closest_type != "wire":
                return
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
            in_flame = (np.linalg.norm(tip[:2] - self.BURNER_POS[:2]) < 0.05
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
            # v23：受染时火焰本身变成本色（焰色反应的核心视觉）。
            # 之前只有尖端染色锥（r=6mm），被不透明白/蓝火焰（该高度 r≈8-10mm）
            # 完全包住，相机里黄色不可见。
            self._set_flame_color(self.FLAME_COLORS[self.flame_color])
        else:
            self._set_flame_color(self.FLAME_BASE_COLORS["outer"], kind="outer")
            self._set_flame_color(self.FLAME_BASE_COLORS["inner"], kind="inner")

    # ---- 火焰材质颜色（v23 新增：受染变黄 / 恢复蓝）----
    # 基色用 FLAME_BASE_COLORS 常量（与场景材质定义一致）

    def _flame_shader(self, kind: str):
        path = f"{self.burner_path}/flame_{kind}_mat/Shader"
        prim = self.stage.GetPrimAtPath(path)
        return prim if prim.IsValid() else None

    def _set_flame_color(self, color, kind=None) -> None:
        for k in (("outer", "inner") if kind is None else (kind,)):
            shader = self._flame_shader(k)
            if shader is None:
                continue
            for attr_name in ("inputs:diffuseColor", "inputs:emissiveColor"):
                attr = shader.GetAttribute(attr_name)
                if attr is not None:
                    attr.Set(Gf.Vec3f(*color))

    def _set_visibility(self, path, visible):
        try:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass
