"""B4 冰浴/冷却 —— 任务：洗瓶挤水入烧杯（水流 + 液面上涨 + 冰块浮起）+ 放回洗瓶 +
竖直提取试管并浸入冰水。

2026-08-30 用户逐字：「现在加动作，机械臂往里面挤入液体要真实（可参考a3），然后
烧杯里面的冰块浮起来（符合物理现象），然后原位放回洗瓶。然后水平夹起试管（模仿b1）」。

洗瓶生命周期（rest → attached → squeezing → released，d2s/a3 同款）：
  rest     近抓点+合拢（opening < 0.034）→ attached（锁 _T_HELD_WASHB = rest·tool⁻¹，
           纯平移持握，零跳变）
  attached 随夹爪平移；夹爪从持握 0.030 进一步合到 <0.025 挤压瓶身 → 挤水（水流 + 液面
           上涨 + 冰块浮起）；松回 0.030 → 停止发射、液面定高（water_added 只触发一次）
  released opening > 0.038 开爪 → 瓶回表位（洗瓶底 0.80 = 台面顶）

挤水水流（仿 a3 SqueezeWater / d2s）：挤水期间每 WATER_STAGGER 帧从红嘴尖发射一颗水滴，
沿抛物线（x/y 线性、z t² 重力加速）坠入烧杯口中心；液面 BeakerLiquid 随落定水滴数上涨；
冰块 /World/Ice_0..5 随液面上涨浮到水面（冰底 = max(rest_z, 烧杯内底 + 液面高 − 冰高
× ICE_FLOAT_SUBMERGE)，冰密度 ~0.90 < 水 → ~10% 露出水面）。

试管生命周期（rest → attached → released，d3l 纯平移持握）：竖直提取（手指朝下）→ 合拢
（opening < 0.019）→ attached（试管世界 = tool 平移 + (0,0,−TUBE_HELD_Z)，只随夹爪平移、
保竖立；管内药品液柱 /World/TubeDrug 随管平移）→ 移到烧杯上方浸入冰水（观察 5s）→
放回架孔（回抓点 + 开爪 >0.03 → released 回 rest）。

场景 prim（b4_ice_bath.usd，scripts/gen_b4_scene.py 生成，2026-08-29 用户定稿）：
  烧杯 beaker.usd (0.45,0.10,0.80) 内装 6 冰块 + 洗瓶 wash_bottle.usd (0.20,0.10,0.80)
  rot180 红嘴朝 +X + 试管 test_tube.usd 立架前排左孔 (0.279,0.241,0.806) 管内预装药品
  /World/TubeDrug；无温度计/试管夹。洗瓶为静态碰撞体，持握期关碰撞。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from catalogue._shared.base_task import CatalogueBaseTask
from .meta_actions.constants import (
    WASH_GRASP, GRIP_WASHBOT,
    WASH_SQUEEZE_CLOSED, SPOUT_TIP_OFFSET,
    WATER_DROPS, WATER_STAGGER, WATER_FALL, WATER_LAND_FULL,
    BEAKER_MOUTH_TOP,
    BEAKER_LIQUID_PATH, BEAKER_LIQUID_R, BEAKER_LIQUID_H0, BEAKER_LIQUID_H_MAX,
    BEAKER_INNER_BOTTOM_Z,
    ICE_PATHS, ICE_HEIGHT, ICE_FLOAT_SUBMERGE,
    TUBE_XY, TUBE_REST_Z, TUBE_GRASP_TCP, TUBE_HELD_Z, TUBE_DRUG_OFFSET_Z,
)

# 试管持握 = 纯平移（d3l 同款）：手指朝下竖直吊管，管底在夹爪下方 TUBE_HELD_Z（0.1393m），
# 只随夹爪平移、不随旋转（保竖立）。见 _held_tube_matrix()：管底世界 z = TCP z − TUBE_HELD_Z。

# 管内药品液柱相对管底的局部偏移（液柱中心 = 管底 + TUBE_DRUG_OFFSET_Z，与 gen rest 中心
# 0.826 = 管底 0.806 + 0.020 一致）；随管同一矩阵刚性跟随，不再悬在原架里。
_TUBE_DRUG_OFFSET = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, TUBE_DRUG_OFFSET_Z, 1.0)

# 晶体层/外壁雾层相对管底的局部偏移（晶体中心 = 管底 + 0.005、雾中心 = 管底 + 0.06，
# 与 gen_b4_scene.py CRYSTAL_CZ/FOG_CZ 一致），随管同一矩阵刚性跟随。
_TUBE_CRYSTAL_OFFSET = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0,
                                   0.0, 0.0, 0.005, 1.0)
_TUBE_FOG_OFFSET = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.06, 1.0)

# 现象变体颜色集（gen_b4_scene.py 预烘焙，task 按 cfg.liquid_color/crystal_color 拼路径）
_LIQUID_COLORS = ["clear", "white", "red", "blue", "green", "purple"]
_CRYSTAL_COLORS = ["none", "white", "red", "blue", "green", "purple"]
_FOG_PATHS = ["/World/TubeFog_1", "/World/TubeFog_2", "/World/TubeFog_3"]


class B4IceBathTask(CatalogueBaseTask):
    """B4 冰浴/冷却任务：洗瓶挤水（水流+液面上涨+冰块浮起）→ 放回 → 竖直提取试管浸入冰水。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array(WASH_GRASP)                 # 抓点（tool_center，瓶身中部 z=0.88）
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004           # 夹紧阈值：grip 0.030 + 4mm 裕量 = 0.034
    WASH_GRIP_OPEN = 0.038                            # 松开阈值（同 d2s/a3；>0.038 才算松开）

    TUBE = "/World/TestTube"
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)             # 抓点（管口下 14mm，手指朝下竖直提取）
    TUBE_GRIP_CLOSED = 0.019                          # 夹紧阈值（b1 修法：0.0096+4mm=0.0136 太紧
                                                      #   手指贴合管壁读回略高即永不吸附 → 放宽）

    WATER_STREAM = "/World/WaterStream"
    LIQUID_PATH = BEAKER_LIQUID_PATH
    WATER_TARGET = np.array(BEAKER_MOUTH_TOP)         # 水落点 = 烧杯口顶中心
    WATER_START_OFFSET = np.array(SPOUT_TIP_OFFSET)   # 红嘴尖相对瓶原点世界偏移（纯平移持握恒定）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.washbottle_path = self.WASH_PATH
        # 洗瓶/试管是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.washbottle_path)
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 洗瓶
        self.washbottle_state = "rest"   # rest / attached / released
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None

        # 挤水 / 液面 / 冰块
        self.squeezing = False           # 挤水进行中（持续发射水滴 + 液面上涨）
        self.water_added = False         # 已挤入水（液面定高，只触发一次）
        self._water_queue = []           # 在飞水滴队列（prim/t）
        self._water_next_prim = 0        # 下一颗水滴用哪个 Drop_i（round-robin 复用池）
        self._water_spawn = 0            # 距下次发射水滴的倒计时帧
        self._water_landed = 0           # 已落定水滴数（驱动液面上涨 + 冰块浮起）

        # 冰块 rest 世界位（烧杯内 6 块；浮起只动 z、保 x/y 与 yaw）
        self._ice_rest = self._read_ice_rest()

        # 试管
        self.tube_state = "rest"         # rest / attached / released
        self._tube_near_frames = 0

        # 现象（冷却析出 + 外壁起雾）—— cfg 色驱动预烘焙变体路径（2026-08-30 用户
        # 「浑浊渐变特别慢、冰浴时间要长、拿出后浑浊渐褪回澄清但晶体留下」）
        self.liquid_color = getattr(cfg, "liquid_color", "clear")
        self.crystal_color = getattr(cfg, "crystal_color", "white")
        if self.liquid_color not in _LIQUID_COLORS:
            self.liquid_color = "clear"
        if self.crystal_color not in _CRYSTAL_COLORS:
            self.crystal_color = "white"
        self.TUBE_DRUG = f"/World/TubeDrug_{self.liquid_color}"       # 澄清液柱（溶液本色）
        self.TUBE_CLOUD = [f"/World/TubeCloud_{self.liquid_color}_{lv}" for lv in (1, 2, 3)]  # 浑浊 3 档
        self.TUBE_CRYSTAL = (f"/World/TubeCrystal_{self.crystal_color}"
                             if self.crystal_color != "none" else None)   # 晶体（none=不析出）
        self.TUBE_FOG = list(_FOG_PATHS)                              # 外壁雾层 3 档
        self._phase = "idle"          # idle / cooling / cooled / warming / warmed
        self._cloud_level = 0         # 当前浑浊档 0(澄清)..3(最浑浊)
        self._cool_progress = 0.0     # 冷却进度 0..1（浑浊渐显）
        self._warm_progress = 0.0     # 升温进度 0..1（浑浊渐隐）
        self._crystal_shown = False   # 晶体是否已析出
        self._fog_frac = 0.0          # 外壁起雾进度 0..1
        self._fog_shown = 0           # 已显示外壁雾档 0..3
        # 现象判定：烧杯 xy（BEAKER_MOUTH_TOP 前两维）、水面 z（内底 + 液面高）
        self._beaker_xy = np.array(BEAKER_MOUTH_TOP)[:2]
        self._water_surface_z = BEAKER_INNER_BOTTOM_Z + BEAKER_LIQUID_H_MAX
        self._cool_frames = 600       # 冷却渐显帧数（浸冰 900 帧内完成，慢）
        self._warm_frames = 420       # 升温渐隐帧数（观察 480 帧内完成，慢）
        self._fog_frames = 300        # 外壁起雾帧数（观察 480 帧内完成）
        self._crystal_at = 0.5        # 冷却到一半时析出晶体

    def reset(self):
        # 必须显式初始化机器人 articulation（全 catalogue 约定同 d2s/b3/e1/a3）。
        super().reset()
        self.robot.initialize()
        # 洗瓶复位
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self._set_washbottle_world(_washbottle_rest_matrix())
        # 挤水/液面/冰块复位
        self.squeezing = False
        self.water_added = False
        self._water_queue = []
        self._water_next_prim = 0
        self._water_spawn = 0
        self._water_landed = 0
        self._set_visibility(self.WATER_STREAM, False)
        for i in range(WATER_DROPS):
            self._set_visibility(f"{self.WATER_STREAM}/Drop_{i}", False)
        self._set_visibility(self.LIQUID_PATH, False)
        self._set_liquid_height(BEAKER_LIQUID_H0)
        for i, rest in enumerate(self._ice_rest):
            self.object_utils.set_object_position(f"/World/Ice_{i}", rest.copy())
        # 试管复位（含管内现象变体）
        self.tube_state = "rest"
        self._tube_near_frames = 0
        self._reset_phenomenon()
        self._set_tube_world(_tube_rest_matrix())

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_washbottle(gripper_pos, opening)
        self._update_tube(gripper_pos, opening)
        self._step_water_anim()
        self._step_phenomenon()
        return self.get_basic_state_info(additional_info={
            "washbottle_state": self.washbottle_state,
            "squeezing": self.squeezing,
            "water_added": self.water_added,
            "tube_state": self.tube_state,
        })

    def on_task_complete(self, success):
        print(f"[b4] episode done success={success} washbottle={self.washbottle_state} "
              f"water_added={self.water_added} tube={self.tube_state}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧洗瓶持握（纯平移，d2s/a3 同款）：rest → 近抓点+合拢 → attached（随夹爪平移）
    #   → 挤水（opening <0.025 挤压瓶身 → 水流+液面上涨+冰块浮起）→ released（回表位）
    # ------------------------------------------------------------------
    def _update_washbottle(self, gripper_pos, opening):
        if self.washbottle_state == "rest":
            if self._near_grasp(gripper_pos, self.WASH_GRASP):
                self._wb_near_frames += 1
            else:
                self._wb_near_frames = 0
            if (self._wb_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.WASH_GRIP_CLOSED):
                self.washbottle_state = "attached"
                # 动态持握变换：抓取时刻瓶子正好在静止位 → 锁 (静止 · tool⁻¹)，零跳变。
                self._T_HELD_WASHB = _washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[b4] washbottle attached (grip={opening:.4f})")

        elif self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            # 挤水：夹爪从持握 0.030 进一步合到 0.020 挤压瓶身 → 水流 + 液面上涨 + 冰块浮起；
            # 松回 0.030 → 停止发射、液面定高（water_added 只触发一次）。
            if not self.water_added:
                if not self.squeezing and opening < WASH_SQUEEZE_CLOSED:
                    self.squeezing = True
                    self._water_spawn = WATER_STAGGER   # 下一帧立即发射首滴
                    self._set_visibility(self.WATER_STREAM, True)
                    self._set_visibility(self.LIQUID_PATH, True)
                    print(f"[b4] washbottle squeezing (grip={opening:.4f}) water stream")
                elif self.squeezing and opening >= WASH_SQUEEZE_CLOSED:
                    self.squeezing = False
                    self.water_added = True
                    print(f"[b4] water added to beaker (grip={opening:.4f})")
            if opening > self.WASH_GRIP_OPEN:   # 完全开爪才算松开（>0.038）
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(_washbottle_rest_matrix())
                print(f"[b4] washbottle released to table (grip={opening:.4f})")
        # released：已回表位，不再跟随

    # ------------------------------------------------------------------
    # 每帧试管持握（d3l 纯平移持握）：rest → 近抓点+合拢 → attached（试管世界 =
    #   tool 平移 + (0,0,−TUBE_HELD_Z)，只随夹爪平移、保竖立）→ released（回架，防御性）
    # ------------------------------------------------------------------
    def _update_tube(self, gripper_pos, opening):
        if self.tube_state == "rest":
            if self._near_grasp(gripper_pos, self.TUBE_GRASP):
                self._tube_near_frames += 1
            else:
                self._tube_near_frames = 0
            if (self._tube_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.TUBE_GRIP_CLOSED):
                self.tube_state = "attached"
                self._set_tube_world(self._held_tube_matrix())
                print(f"[b4] tube attached (grip={opening:.4f})")

        elif self.tube_state == "attached":
            self._set_tube_world(self._held_tube_matrix())
            # 松爪且回抓点才写回原位（ReturnTube ② 下探到抓点 + ③ 开爪触发；防远端开爪误释放）
            if (opening > self.gripper_open_threshold
                    and self._near_grasp(gripper_pos, self.TUBE_GRASP)):
                self.tube_state = "released"
                self._set_tube_world(_tube_rest_matrix())
                print(f"[b4] tube released to rack (grip={opening:.4f})")
        # released：已回架位，不再跟随

    # ------------------------------------------------------------------
    # 挤水水流（仿 a3 SqueezeWater / d2s）：挤水期间每 WATER_STAGGER 帧发射一颗水滴，
    # 沿抛物线（x/y 线性、z t² 重力加速）从红嘴尖坠入烧杯口中心；松爪后停止发射、让在飞
    # 水滴落完再隐藏父节点。水滴池 round-robin 复用。液面随落定水滴数上涨、冰块随液面浮起。
    # ------------------------------------------------------------------
    def _washbottle_world_matrix(self):
        prim = self.stage.GetPrimAtPath(self.washbottle_path)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _spout_tip_pos(self):
        """红嘴尖世界坐标（水流出点）：瓶原点 + 固定世界偏移（纯平移持握瓶朝向恒定）。"""
        wm = self._washbottle_world_matrix()
        origin = np.array(wm.ExtractTranslation())
        return origin + self.WATER_START_OFFSET

    def _step_water_anim(self):
        if self.squeezing:
            self._water_spawn += 1
            if self._water_spawn >= WATER_STAGGER:
                self._water_spawn = 0
                idx = self._water_next_prim % WATER_DROPS
                self._water_next_prim += 1
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{idx}", True)
                self.object_utils.set_object_position(
                    f"{self.WATER_STREAM}/Drop_{idx}", self._spout_tip_pos())
                self._water_queue.append({"prim": idx, "t": 0})
        if not self._water_queue:
            return
        remaining = []
        start = self._spout_tip_pos()          # 每帧取当前红嘴尖（瓶不动时恒定）
        target = self.WATER_TARGET
        for d in self._water_queue:
            d["t"] += 1
            if d["t"] >= WATER_FALL:
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{d['prim']}", False)
                self._water_landed += 1
                continue
            frac = d["t"] / WATER_FALL
            x = start[0] + (target[0] - start[0]) * frac
            y = start[1] + (target[1] - start[1]) * frac
            z = start[2] - (start[2] - target[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{self.WATER_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._water_queue = remaining
        # 液面随落定水滴数上涨（落满 WATER_LAND_FULL 滴 → 最终液面高度），冰块随液面浮起
        if self._water_landed > 0:
            frac = min(1.0, self._water_landed / WATER_LAND_FULL)
            h = max(BEAKER_LIQUID_H0, BEAKER_LIQUID_H_MAX * frac)
            self._set_liquid_height(h)
            self._update_ice_float(h)
        if not remaining and not self.squeezing:
            self._set_visibility(self.WATER_STREAM, False)

    def _set_liquid_height(self, h):
        """烧杯内液面圆柱：更新半径/高度 + 中心 z（底贴烧杯内底，随 h 上移）。"""
        prim = self.stage.GetPrimAtPath(self.LIQUID_PATH)
        if not prim.IsValid():
            return
        cyl = UsdGeom.Cylinder(prim)
        cyl.GetRadiusAttr().Set(BEAKER_LIQUID_R)
        cyl.GetHeightAttr().Set(h)
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpName() != "xformOp:translate":
                continue
            v = op.Get()
            op.Set(Gf.Vec3d(v[0], v[1], BEAKER_INNER_BOTTOM_Z + h / 2.0))
            return

    def _update_ice_float(self, h):
        """冰块随液面上涨浮到水面（冰密度 ~0.90 < 水，~10% 露出水面）：冰底 = max(rest_z,
        烧杯内底 + 液面高 − 冰高 × ICE_FLOAT_SUBMERGE)。只动 translate z、保 x/y 与 yaw。"""
        if not self._ice_rest:
            return
        float_z = BEAKER_INNER_BOTTOM_Z + h - ICE_HEIGHT * ICE_FLOAT_SUBMERGE
        for i, rest in enumerate(self._ice_rest):
            z = max(float(rest[2]), float_z)
            self.object_utils.set_object_position(
                f"/World/Ice_{i}", np.array([rest[0], rest[1], z]))

    def _read_ice_rest(self):
        """读 6 块冰块的世界静止位（烧杯内底层 4 块 0.802 / 二层 2 块 0.8135）。"""
        rest = []
        for path in ICE_PATHS:
            pos = self.object_utils.get_object_xform_position(path)
            if pos is not None:
                rest.append(np.asarray(pos, dtype=float))
        return rest

    # ------------------------------------------------------------------
    # 位姿工具（写世界矩阵到 prim；行向量约定同 d2s/a3/b1）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _write_world(self, path, world_matrix):
        """把 prim 写到给定世界位姿（局部 = 世界 · 父世界逆，单 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 d2s，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        self._write_world(self.washbottle_path, world_matrix)

    def _held_tube_matrix(self):
        """试管世界位姿 = 纯平移持握（d3l 同款）：手指朝下，试管竖直吊在夹爪下方
        TUBE_HELD_Z，只随夹爪平移、不随旋转（保竖立）。管底世界 z = TCP z − TUBE_HELD_Z。"""
        t = self._tool_world()
        p = t.ExtractTranslation()
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           p[0], p[1], p[2] - TUBE_HELD_Z, 1.0)

    def _set_tube_world(self, world_matrix):
        """试管 + 管内现象变体（澄清液柱 + 当前浑浊档 + 晶体 + 外壁雾层）一起写世界矩阵
        （各层相对管底偏移刚性跟随、随管转——不再悬在原架里）。澄清液柱始终跟随（溶液
        本色，浑浊档 opacity 1.0 同几何遮住它）。"""
        self._write_world(self.TUBE, world_matrix)
        self._write_world(self.TUBE_DRUG, world_matrix * _TUBE_DRUG_OFFSET)
        if self._cloud_level > 0:
            self._write_world(self.TUBE_CLOUD[self._cloud_level - 1],
                              world_matrix * _TUBE_DRUG_OFFSET)
        if self.TUBE_CRYSTAL:
            self._write_world(self.TUBE_CRYSTAL, world_matrix * _TUBE_CRYSTAL_OFFSET)
        for fog in self.TUBE_FOG:
            self._write_world(fog, world_matrix * _TUBE_FOG_OFFSET)

    # ------------------------------------------------------------------
    # 现象状态机（2026-08-30 用户「1 冷却析出（溶液浑浊+晶体析出）、2 外壁冷凝起雾」）：
    # 读试管当前世界位置，管底浸入冰水（水面 z 下、水平在烧杯内）→ 冷却进度累积 →
    # 澄清 TubeDrug 切浑浊 TubeCloud + 晶体 TubeCrystal 析出；管底提出水面后 → 起雾
    # 进度累积 → 外壁雾层 3 档依次切换（visibility 模拟冷凝渐浓，headless 下 runtime
    # 改 opacity 不渲染）。
    # ------------------------------------------------------------------
    def _step_phenomenon(self):
        pos = self.object_utils.get_object_xform_position(self.TUBE)
        if pos is None:
            return
        x, y, z = pos[0], pos[1], pos[2]
        immersed = (z < self._water_surface_z
                    and abs(x - self._beaker_xy[0]) < 0.05
                    and abs(y - self._beaker_xy[1]) < 0.05)
        if immersed:
            # 降温段（浸冰）：浑浊渐显（慢）+ 晶体析出
            if self._phase in ("idle", "cooling"):
                self._phase = "cooling"
                self._cool_progress = min(1.0, self._cool_progress + 1.0 / self._cool_frames)
                level = min(3, int(self._cool_progress * 3.0 + 1e-6))
                self._set_cloud_level(level)
                if self._cool_progress >= self._crystal_at and not self._crystal_shown:
                    self._crystal_shown = True
                    if self.TUBE_CRYSTAL:
                        self._set_visibility(self.TUBE_CRYSTAL, True)
                        print(f"[b4] crystal precipitated ({self.crystal_color})")
                if self._cool_progress >= 1.0:
                    self._phase = "cooled"
                    print("[b4] cooled (cloudy + crystal)")
        else:
            # 升温段（提出）：浑浊渐隐（慢）→ 溶液恢复澄清、晶体保留；外壁冷凝起雾
            if self._phase == "cooled":
                self._warm_progress = min(1.0, self._warm_progress + 1.0 / self._warm_frames)
                level = max(0, 3 - int(self._warm_progress * 3.0 + 1e-6))
                self._set_cloud_level(level)
                if self._warm_progress >= 1.0:
                    self._phase = "warmed"
                    print("[b4] warmed: clear again (crystal stays)")
            if self._phase in ("cooled", "warmed") and self._fog_frac < 1.0:
                self._fog_frac = min(1.0, self._fog_frac + 1.0 / self._fog_frames)
                fog_now = min(3, int(self._fog_frac * 3.0))
                if fog_now != self._fog_shown:
                    for i in range(3):
                        self._set_visibility(self.TUBE_FOG[i], i == fog_now - 1)
                    self._fog_shown = fog_now
                    if fog_now > 0:
                        print(f"[b4] fog level {fog_now}/3")

    def _set_cloud_level(self, level):
        """切换浑浊档 visibility（只显示当前档，level=0 全隐=澄清）。"""
        if level == self._cloud_level:
            return
        self._cloud_level = level
        for i in range(3):
            self._set_visibility(self.TUBE_CLOUD[i], i + 1 == level)

    def _reset_phenomenon(self):
        """复位现象状态 + 变体 visibility（澄清液柱显示、浑浊 3 档/晶体/雾隐藏）。"""
        self._phase = "idle"
        self._cloud_level = 0
        self._cool_progress = 0.0
        self._warm_progress = 0.0
        self._crystal_shown = False
        self._fog_frac = 0.0
        self._fog_shown = 0
        for c in _LIQUID_COLORS:
            self._set_visibility(f"/World/TubeDrug_{c}", c == self.liquid_color)
            for lv in (1, 2, 3):
                self._set_visibility(f"/World/TubeCloud_{c}_{lv}", False)
        for c in _CRYSTAL_COLORS:
            if c != "none":
                self._set_visibility(f"/World/TubeCrystal_{c}", False)
        for fog in self.TUBE_FOG:
            self._set_visibility(fog, False)

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _disable_collision(self, root):
        prim = self.stage.GetPrimAtPath(root)
        if not prim.IsValid():
            return
        stack = [prim]
        while stack:
            p = stack.pop()
            if UsdPhysics.CollisionAPI(p):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
            for c in p.GetChildren():
                stack.append(c)

    def _set_visibility(self, path, visible):
        try:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass


def _washbottle_rest_matrix():
    """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵，pxr 实测 2026-08-29）：
    rotateXYZ(0,0,-180) + translate (0.20,0.10,0.80) 烘平后即下行序。
    行 0 = (-1,0,0,0) → 局部 +X 朝世界 -X；行 1 = (0,-1,0,0) → +Y 朝 -Y（红嘴尖朝 +X）。
    """
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.20, 0.10, 0.80, 1.0)


def _tube_rest_matrix():
    """试管架内竖插位姿（b1 同款）：恒等旋转 + 平移 (TUBE_XY, TUBE_REST_Z)。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       TUBE_XY[0], TUBE_XY[1], TUBE_REST_Z, 1.0)
