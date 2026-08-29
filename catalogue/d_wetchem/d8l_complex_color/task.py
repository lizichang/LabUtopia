"""D8-L 络合/显色试剂滴加反应任务：三支滴管各自的「吸液→滴入试管」生命周期 + 液体效果。

持握照 flametest（v24-v46 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——滴管
全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），效果 prim（DropperFill）
只需 position 跟随尖嘴即可。

生命周期（每支滴管，gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（液柱被吸进尖嘴）
  - filled   跟随；试管口区挤胶头（<GRIP_SQUEEZED）→ dropped → DropperFill 隐藏
             + TubeDrops 显示且液面逐滴升高
  - dropped  跟随；cycle 未结束回瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（一次持握内
             循环）；末遍滴完回架松开（>gripper_open）→ released → rest

三支滴管生命周期句柄（sample/reagent1/reagent2）共用一套状态机，各自参考点不同；
controller 顺序执行 ①SamplePass → ②Reagent1Pass → ③Reagent2Pass → ④TubeShakePass。

液体变色 3 段（用户 2026-08-28 多输入）：样品滴入 → initial_color 段、试剂 1 滴入 →
color_after_reagent1 段、试剂 2 滴入 → color_after_reagent2 段（后段盖前段）。试剂 2
（最后一支）滴入触发沉淀（cfg.has_precipitate）+ 气泡（cfg.has_bubbles），震荡停后
分层成形（cfg.has_layer）。气泡复刻 d3l（气泡颜色跟随最终色 = 最后非 clear 段）。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
    DROP_REAGENT1_REST, DROP_REAGENT1_GRASP,
    DROP_REAGENT2_REST, DROP_REAGENT2_GRASP,
    SAMPLE_BOTTLE_XY, REAGENT1_BOTTLE_XY, REAGENT2_BOTTLE_XY,
    TUBE_XY, TUBE_GRASP_TCP,
    SHAKE_TOP_Z, SHAKE_STOP_EPS, SHAKE_STILL_FRAMES, PHENOMENA_FRAMES,
    EFFECT_TUBE_DROPS, EFFECT_PRECIPITATE, EFFECT_PRECIPITATE_CLOUD, EFFECT_LAYER,
    EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])

# 试管持握（步9 抓起试管震荡）：抓管身中段（管口下 14mm），管底 0.139m 吊在夹爪下方。
TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], 0.806])
TUBE_HELD_OFFSET = np.array([0.0, 0.0, TUBE_ORIG[2] - TUBE_GRASP_TCP[2]])   # ≈(0,0,-0.139)


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    所对瓶口 xy（排空气/浸液区，z 不区分）
      tube_xy      试管口 xy（滴液区）
    """

    def __init__(self, task, name, path, orig, grasp, bottle_xy, tube_xy,
                 fill_path=None):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.bottle_xy = np.array(bottle_xy)
        self.tube_xy = np.array(tube_xy)
        self.fill_path = fill_path
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.squeezed = False
        self.filled = False
        self.dropped = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.squeezed = self.filled = self.dropped = self.released = False
        self.task._set_obj_world(self.path, self.orig)
        if self.fill_path:
            self.task._set_visibility(self.fill_path, False)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + HELD_OFFSET
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d8l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d8l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d8l] {self.name} filled (aspirated)")
        elif self.state == "filled":
            if self.fill_path:
                self.task._set_fill_follow(self)
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.tube_xy, gripper_pos)):
                self.state = "dropped"
                self.dropped = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, False)
                self.task._on_drop(self)
                print(f"[d8l] {self.name} dropped into tube")
        elif self.state == "dropped":
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[d8l] {self.name} re-squeeze at bottle (cycle)")
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d8l] {self.name} released to rack -> rest")


class _TubeLifecycle:
    """试管生命周期：rest → attached（跟随 + 管内液柱随管平移）→ released。"""

    def __init__(self, task, path, orig, grasp):
        self.task = task
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.released = False
        self.task._set_obj_world(self.path, self.orig)

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + TUBE_HELD_OFFSET
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._vigor = 1.0   # 抓起试管震荡=再爆发（模拟震荡释放溶解气体）
                self.task._set_obj_world(self.path, held)
                print(f"[d8l] tube attached (grip={opening:.4f})")
            return

        held = gripper_pos + TUBE_HELD_OFFSET
        self.task._set_obj_world(self.path, held)
        self.task._follow_tube_liquid(held)
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.grasp, gripper_pos)):
            self.released = True
            self.task._set_obj_world(self.path, self.orig)
            self.task._follow_tube_liquid(self.orig)
            self.state = "rest"
            print(f"[d8l] tube released to rack -> rest")


class D8LComplexColorTask(BaseTask):
    """D8-L 络合/显色试剂滴加任务：三支滴管吸液→滴入试管 + 3 段变色 + 沉淀 + 分层。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 管内液体逐滴生长（底面贴管底 0.806；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    DROP_LEVEL_STEP = 0.004   # 每滴落定后液面升高 4mm（视觉夸张）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5
    DROP_FALL = 16
    DROP_STAGGER = 6

    # 现象时序：震荡停止检测阈值（管已抓起 + 高位 + 连续静止帧）。分层/沉淀沉降共用。
    SHAKE_TOP_Z = SHAKE_TOP_Z
    SHAKE_STOP_EPS = SHAKE_STOP_EPS
    SHAKE_STILL_FRAMES = SHAKE_STILL_FRAMES
    PHENOMENA_FRAMES = PHENOMENA_FRAMES

    # 沉淀现象（同 d3l 2026-08-19 真实感改造）：逐滴增厚 + 先浊后沉 + 震荡再悬浮。
    # 2026-08-29 用户：沉淀看不到→加厚；随后"太高太快"→ 减半 0.0065 + 放缓；本日
    # "沉淀再变厚1cm" → 6.5mm 基线 +1cm = 16.5mm（PRECIP_MAX 0.0165），增速同步提到
    # 0.000020（780 帧提起窗口刚好长到上限）；沉降斜坡 PRECIP_FADE_RATE 0.008→0.005
    # 让停震后沉淀层"逐渐变厚"的观感更慢更自然（震荡中颗粒悬浮成灰色浑浊云盖住总量变化）。
    PRECIP_DROP_STEP = 0.0008
    PRECIP_MAX = 0.0165          # 最终沉降层最大厚度（6.5mm + 用户"+1cm" = 16.5mm）
    PRECIP_GROW_RATE = 0.000020  # 提起期间每帧沉淀增厚量（≈780 帧震荡+悬停 → +0.0156）
    PRECIP_RESUSPEND_RATE = 0.008
    PRECIP_FADE_RATE = 0.005     # 停震沉降斜坡（0.008→0.005，沉淀层渐厚更慢更自然）
    PRECIP_RESUSPEND_FLOOR = 0.1
    PRECIP_SPAWN_MARGIN = 0.0015   # 气泡生成高度 = 沉降层顶 + 该值

    # 气泡动画（复刻 d3l 2026-08-19 真实感改造，中等档）：小球池从管底分散区（gen 烘焙
    # 40 个基准 x/y：中心盘 30 + 近壁环 10）生成 → 穿液柱上升 → 到当前液面消失（"破掉"）。
    # 小泡(Ø4.4mm)×40、快(~0.06m/s)、蛇形摆动上飘 + 每颗速度差异、每滴试剂触发爆发后按
    # _vigor 衰减（先猛后衰）。N_BUBBLES 必须与 gen BUBBLES 一致(40)，gen.verify() 会断言。
    N_BUBBLES = 40
    # 生成高度：无沉淀=试管最底部圆底点（管底收敛点 z=0.806）；有沉淀时白柱盖住底部
    # 0.806..0.809，若仍从 0.806 生成会被白柱遮住 → __init__ 按 has_precipitate 自动抬到
    # 沉淀柱顶之上。
    BUBBLE_SPAWN_Z = 0.806
    BUBBLE_SPAWN_Z_PRECIP = 0.810     # 沉淀柱顶 0.809 + 1mm
    BUBBLE_POP_MARGIN = 0.002  # 离当前液面下方一点即消失（"破在液面"）
    BUBBLE_RISE = 0.0010       # 每帧上升量（m，@60Hz ≈ 0.06m/s，中等档）
    BUBBLE_SPAWN_INTERVAL = 4  # 基础生成间隔帧（@60Hz ≈ 15 颗/s；实际间隔 = 本值/_vigor）
    BUBBLE_WOBBLE_AMP = 0.0012  # 上升蛇形摆动振幅（±1.2mm）
    BUBBLE_MAX_RADIUS = 0.006   # 气泡中心离管轴最大半径（管内缘 0.009 − 泡半径 0.0022）
    VIGOR_DECAY = 0.994         # 反应强度每帧衰减系数（@60Hz ≈3.8s 从 1.0 降到 0.25）
    VIGOR_FLOOR = 0.25          # 衰减下限（保持轻微余泡，不突变归零）

    DROPPER_SAMPLE = "/World/DropperSample"
    DROPPER_REAGENT1 = "/World/DropperReagent1"
    DROPPER_REAGENT2 = "/World/DropperReagent2"
    TUBE = "/World/TestTube"

    TUBE_DROPS = EFFECT_TUBE_DROPS
    PRECIPITATE = EFFECT_PRECIPITATE
    PRECIPITATE_CLOUD = EFFECT_PRECIPITATE_CLOUD
    LAYER = EFFECT_LAYER
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    # 3 段液体变色（2026-08-28）：每段×候选色预烘焙液柱 TubeDropsColor_<stage>_<色>，
    # task 按三段输入 show 对应一根，逐滴推进 _color_frac。三段半径逐段递增嵌套
    # （sample 0.0084 < reagent1 0.0085 < reagent2 0.0086），后段在外层盖前段。
    COLOR_STEP = 0.25          # 每滴推进变色进度步长（4 滴即满）
    TUBE_DROPS_COLOR = "/World/TubeDropsColor_{stage}_{color}"
    COLOR_STAGES = ("sample", "reagent1", "reagent2")

    # 分层（2026-08-28）：震荡停后底部重相成形，占液面高 LAYER_FRACTION，LAYER_FORM_FRAMES 内长成
    LAYER_FRACTION = 0.40
    LAYER_FORM_FRAMES = PHENOMENA_FRAMES   # 180 帧（3s）内渐渐成形

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管/试管是静态碰撞体：吸附期关碰撞
        self._disable_collision(self.DROPPER_SAMPLE)
        self._disable_collision(self.DROPPER_REAGENT1)
        self._disable_collision(self.DROPPER_REAGENT2)
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        self.has_layer = bool(getattr(cfg, "has_layer", False))
        self.has_precipitate = bool(getattr(cfg, "has_precipitate", False))
        self.has_bubbles = bool(getattr(cfg, "has_bubbles", False))

        # 3 段变色（用户 2026-08-28 三输入；clear=该段不变色，保持上一段颜色）
        self.initial_color = str(getattr(cfg, "initial_color", "clear")).strip().lower()
        self.color_after_reagent1 = str(getattr(cfg, "color_after_reagent1", "clear")).strip().lower()
        self.color_after_reagent2 = str(getattr(cfg, "color_after_reagent2", "clear")).strip().lower()
        stage_colors = {
            "sample": self.initial_color,
            "reagent1": self.color_after_reagent1,
            "reagent2": self.color_after_reagent2,
        }
        self._color_paths = {}
        self._color_fracs = {}
        for stage_key in self.COLOR_STAGES:
            color = stage_colors[stage_key]
            self._color_paths[stage_key] = (
                self.TUBE_DROPS_COLOR.format(stage=stage_key, color=color)
                if color != "clear" else None)
            self._color_fracs[stage_key] = 0.0

        # 气泡组：颜色跟随最终变色（复刻 d3l 2026-08-24 用户）——最终色 = 最后非 clear 段
        # （reagent2 → reagent1 → sample 依次回退，全 clear 则 clear）。gen 预烘焙
        # Bubbles_<色> 五组，task 按最终色选一组 show（headless 下运行时改材质不渲染）。
        self._bubbles_path = f"/World/Bubbles_{self._final_color()}"
        # 气泡生成高度：无沉淀=管底圆底点；有沉淀=抬到沉淀柱顶之上（避免被白柱遮住）。
        self.BUBBLE_SPAWN_Z = (self.BUBBLE_SPAWN_Z_PRECIP if self.has_precipitate
                               else self.BUBBLE_SPAWN_Z)

        # 三支滴管各自的生命周期句柄（参考点已 pxr 实测）
        self.droppers = {
            "sample": _DropperLifecycle(
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
            "reagent1": _DropperLifecycle(
                self, "reagent1", self.DROPPER_REAGENT1, DROP_REAGENT1_REST, DROP_REAGENT1_GRASP,
                REAGENT1_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
            "reagent2": _DropperLifecycle(
                self, "reagent2", self.DROPPER_REAGENT2, DROP_REAGENT2_REST, DROP_REAGENT2_GRASP,
                REAGENT2_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_drop_count = 0      # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []          # 滴落动画队列
        self.tube = _TubeLifecycle(self, self.TUBE, TUBE_ORIG, TUBE_GRASP_TCP)

        # 现象时序状态：震荡停检测 + 分层成形 + 沉淀沉降
        self._precip_rack = None
        r = self._get_obj_world(self.PRECIPITATE)
        if r is not None:
            self._precip_rack = np.asarray(r, dtype=float)
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        # 气泡消退时序（复刻 d3l）：震荡停后 PHENOMENA_FRAMES 隐藏气泡（沉淀/分层不随消退）
        self._phenomena_fade_frame = None
        self._phenomena_faded = False

        # 沉淀状态
        self._precip_total = 0.0
        self._precip_settled = 0.0
        self._cloud_frac = 0.0
        self._precip_prev = None

        # 分层状态
        self._layer_frac = 0.0        # 成形进度 0..1（震荡停后 LAYER_FORM_FRAMES 内长成）
        self._layer_formed = False

        # 气泡动画状态（复刻 d3l）：基准 x/y（gen 烘焙的局部 translate）保持，每帧只动子球
        # 局部 z；_bubble_active[i] 标记第 i 颗是否在飞（池内小球复用，升起→液面消失）
        self._bubbles_visible = False
        self._bubble_bases = []     # [(x, y), ...] 相对试管原位的局部基准
        self._bubble_z = []         # 当前 z（上升中）
        self._bubble_active = []    # 每颗是否在飞（True 显示中，False 空闲待复用）
        self._bubble_age = []       # 每颗已上升帧数（蛇形摆动相位）
        self._bubble_speed = []     # 每颗速度系数（0.85~1.15，确定性由 index 派生）
        self._bubble_phase = []     # 每颗蛇形摆动相位偏移（确定性由 index 派生）
        self._spawn_timer = 0       # 距下次生成剩余帧数
        self._vigor = 1.0           # 反应强度：每滴试剂滴入/抓起试管时复位 1.0，逐帧衰减
        self._init_bubble_anim()

        self._liquid_shader_cache = None
        # 从 TubeDrops 材质实读"清澈基线"（避免与 gen 的 WATER 配方重复维护）
        self._liquid_clear_color = (0.72, 0.85, 1.0)
        self._liquid_clear_opacity = 0.50
        sh = self._liquid_shader()
        if sh is not None:
            dc = sh.GetInput('diffuseColor')
            op = sh.GetInput('opacity')
            if dc is not None and dc.Get() is not None:
                self._liquid_clear_color = tuple(float(x) for x in dc.Get())
            if op is not None and op.Get() is not None:
                self._liquid_clear_opacity = float(op.Get())

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_drop_count = 0
        self._drop_queue = []
        for d in self.droppers.values():
            d.reset()
        self.tube.reset()
        for p in (self.TUBE_DROPS, self.PRECIPITATE, self.PRECIPITATE_CLOUD,
                  self.LAYER, self._bubbles_path, self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visibility(p, False)
        # 3 段变色复位：进度清零、变色液柱隐藏 + 高度归零
        for stage_key in self.COLOR_STAGES:
            self._color_fracs[stage_key] = 0.0
            path = self._color_paths[stage_key]
            if path:
                self._set_visibility(path, False)
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    UsdGeom.Cylinder(prim).GetHeightAttr().Set(0.0)
        # 现象时序复位：气泡消退帧/已消退标志清零
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        self._phenomena_fade_frame = None
        self._phenomena_faded = False
        # 气泡复位：隐藏、z 回生成位、Xform 回原点（=气泡在管内架位）
        self._bubbles_visible = False
        self._reset_bubble_anim()
        self.object_utils.set_object_position(self._bubbles_path, (0.0, 0.0, 0.0))
        # 沉淀复位
        self._precip_total = 0.0
        self._precip_settled = 0.0
        self._cloud_frac = 0.0
        self._precip_prev = None
        sh = self._liquid_shader()
        if sh is not None:
            sh.GetInput('diffuseColor').Set(Gf.Vec3f(*self._liquid_clear_color))
            sh.GetInput('opacity').Set(self._liquid_clear_opacity)
        pprim = self.stage.GetPrimAtPath(self.PRECIPITATE)
        if pprim.IsValid():
            UsdGeom.Cylinder(pprim).GetHeightAttr().Set(0.0)
        cprim = self.stage.GetPrimAtPath(self.PRECIPITATE_CLOUD)
        if cprim.IsValid():
            UsdGeom.Cylinder(cprim).GetHeightAttr().Set(0.0)
        if self._precip_rack is not None:
            self.object_utils.set_object_position(self.PRECIPITATE, self._precip_rack)
        # 分层复位
        self._layer_frac = 0.0
        self._layer_formed = False
        lprim = self.stage.GetPrimAtPath(self.LAYER)
        if lprim.IsValid():
            UsdGeom.Cylinder(lprim).GetHeightAttr().Set(0.0)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._step_drop_anim()              # 滴落动画独立推进
        self._step_bubble_anim()            # 气泡上升循环（可见时推进）
        for d in self.droppers.values():
            d.step(gripper_pos, opening)
        self.tube.step(gripper_pos, opening)  # 试管跟随/震荡（步9）
        self._step_phenomena(gripper_pos)     # 震荡停检测 + 气泡消退排定
        self._step_precipitate(gripper_pos)   # 沉淀：逐滴增厚/先浊后沉/震荡再悬浮
        self._step_color_liquid()             # 3 段变色液柱：顶贴液面向下扩散
        self._step_layer()                    # 分层：震荡停后底部重相渐渐成形
        return self.get_basic_state_info(additional_info={
            "sample_attached": self.droppers["sample"].attached,
            "sample_filled": self.droppers["sample"].filled,
            "sample_dropped": self.droppers["sample"].dropped,
            "reagent1_attached": self.droppers["reagent1"].attached,
            "reagent1_filled": self.droppers["reagent1"].filled,
            "reagent1_dropped": self.droppers["reagent1"].dropped,
            "reagent2_attached": self.droppers["reagent2"].attached,
            "reagent2_filled": self.droppers["reagent2"].filled,
            "reagent2_dropped": self.droppers["reagent2"].dropped,
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[d8l] episode done success={success} "
              f"sample_dropped={self.droppers['sample'].dropped} "
              f"reagent1_dropped={self.droppers['reagent1'].dropped} "
              f"reagent2_dropped={self.droppers['reagent2'].dropped} "
              f"tube_released={self.tube.released}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 滴管位姿
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            self.object_utils.set_object_position(path, np.asarray(position, dtype=float))

    def _ease_obj_world(self, path, target, k=0.18):
        cur = self._get_obj_world(path)
        if cur is None:
            return
        nxt = cur + (target - cur) * k
        self._set_obj_world(path, nxt)

    def _liquid_shader(self):
        if self._liquid_shader_cache is not None:
            return self._liquid_shader_cache
        sh = None
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            rel = prim.GetRelationship("material:binding")
            targets = rel.GetTargets() if rel else []
            if targets:
                mat = self.stage.GetPrimAtPath(targets[0])
                for c in mat.GetChildren():
                    if c.GetTypeName() == "Shader":
                        sh = UsdShade.Shader(c)
                        break
        self._liquid_shader_cache = sh
        return sh

    def _set_fill_follow(self, dropper):
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(self.DROPPER_FILL, tip)

    def _follow_tube_liquid(self, tube_pos):
        """试管被拿起时管内效果随管平移：液柱（TubeDrops）底贴管底、高度不变；气泡（Bubbles
        父 Xform）整体平移——现象在**管内**，不能留在空架孔处。沉淀/变色/分层由各自 _step_*
        每帧以试管当前管底为基准定位，不在此处。tube_pos = 试管管底世界坐标。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            h = float(UsdGeom.Cylinder(prim).GetHeightAttr().Get() or 0.0)
            self.object_utils.set_object_position(
                self.TUBE_DROPS, (tube_pos[0], tube_pos[1], tube_pos[2] + h / 2))
        delta = np.asarray(tube_pos, dtype=float) - TUBE_ORIG   # 随管位移（回架=0）
        if self.has_bubbles:
            # Bubbles Xform 原点即"气泡在架内管内"位（球有各自 translate），整体平移 delta
            self.object_utils.set_object_position(self._bubbles_path, delta)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """任一滴加：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴。每滴落定后管内液面长高。
        试剂 2（最后一支）滴入触发沉淀；每支滴入推进对应段变色进度（见 _grow_tube_level）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._tube_drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            target = np.array([TUBE_XY[0], TUBE_XY[1],
                               self.TUBE_BOTTOM_Z + level - 0.003])
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,
                "t": 0,
                "start": start.copy(), "target": target,
                "level": level, "name": dropper.name,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._tube_drop_count += self.DROPS_PER_SQUEEZE
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[d8l] squeeze ({dropper.name}) -> {self.DROPS_PER_SQUEEZE} drops spawned")

    def _step_drop_anim(self):
        if not self._drop_queue:
            return
        remaining = []
        for d in self._drop_queue:
            if d["delay"] > 0:
                d["delay"] -= 1
                remaining.append(d)
                continue
            d["t"] += 1
            if d["t"] <= d["hang"]:
                pos = d["start"]
            elif d["t"] <= d["hang"] + d["fall"]:
                frac = (d["t"] - d["hang"]) / d["fall"]
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_tube_level(d["level"], d["name"])
                continue
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    def _grow_tube_level(self, h, name):
        """液滴落定：管内液面长到高度 h；试剂 2 触发现象 + 各支推进对应段变色。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self.TUBE_DROPS,
                (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z + h / 2))
        self._set_visibility(self.TUBE_DROPS, True)
        # 沉淀 + 气泡：最后一支试剂（reagent2）滴入触发（样品+试剂1+试剂2 全部混合后）
        if name == "reagent2":
            if self.has_precipitate:
                self._set_visibility(self.PRECIPITATE, True)
                self._precip_total = min(self.PRECIP_MAX,
                                         self._precip_total + self.PRECIP_DROP_STEP)
                self._precip_settled = self._precip_total
            if self.has_bubbles:
                self._set_visibility(self._bubbles_path, True)
                self._bubbles_visible = True
                self._vigor = 1.0   # 每滴试剂滴入=一次小爆发（反应强度复位，随后逐帧衰减）
        # 3 段变色：任一滴入推进对应段进度
        if name in self.COLOR_STAGES:
            path = self._color_paths[name]
            if path:
                self._color_fracs[name] = min(1.0, self._color_fracs[name] + self.COLOR_STEP)
                self._set_visibility(path, True)
        print(f"[d8l] tube liquid level h={h:.3f}")

    # ------------------------------------------------------------------
    # 气泡上升动画（复刻 d3l 2026-08-16：气泡要有移动上升效果）
    # ------------------------------------------------------------------
    def _final_color(self):
        """气泡跟随的最终液体颜色：最后非 clear 段（reagent2 → reagent1 → sample 依次
        回退，全 clear 则 clear）。气泡在 reagent2 滴入时触发，此时三段已全部滴入。"""
        for c in (self.color_after_reagent2, self.color_after_reagent1, self.initial_color):
            if c != "clear":
                return c
        return "clear"

    def _bubble_spawn_z(self):
        """气泡生成高度：有沉淀时从沉降层顶上方冒出（跟 _precip_settled 走，白柱长高
        生成位也抬升、震荡荡起变矮生成位随之降低），无沉淀时固定管底圆底点。"""
        if self.has_precipitate:
            return self.TUBE_BOTTOM_Z + self._precip_settled + self.PRECIP_SPAWN_MARGIN
        return self.BUBBLE_SPAWN_Z

    def _init_bubble_anim(self):
        """读 N_BUBBLES 颗气泡的烘焙局部 translate 作基准 x/y；初始全隐藏、待生成。
        每颗派生确定性的速度系数与摆动相位（不引 random，保证 reset 可复现）。"""
        self._bubble_bases = []
        self._bubble_z = []
        self._bubble_active = []
        self._bubble_age = []
        self._bubble_speed = []
        self._bubble_phase = []
        for i in range(self.N_BUBBLES):
            prim = self.stage.GetPrimAtPath(f"{self._bubbles_path}/Bubble_{i}")
            if not prim.IsValid():
                continue
            t = prim.GetAttribute("xformOp:translate")
            if not t or not t.HasValue():
                continue
            v = t.Get()
            self._bubble_bases.append((float(v[0]), float(v[1])))
            self._bubble_z.append(self.BUBBLE_SPAWN_Z)
            self._bubble_active.append(False)
            self._bubble_age.append(0)
            # 速度 0.85~1.15、相位 0~2π，均由 index 确定性派生（线性同余散步）
            self._bubble_speed.append(0.85 + 0.3 * ((i * 37) % 100) / 100.0)
            self._bubble_phase.append((i * 0.7) % (2.0 * np.pi))
            self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", False)
        self._spawn_timer = 0

    def _reset_bubble_anim(self):
        """复位小球池：全部隐藏、z 回生成位、年龄/反应强度清零（reset/重跑用，基准 x/y
        与每颗速度/相位不变）。"""
        for i in range(len(self._bubble_active)):
            self._bubble_active[i] = False
            self._bubble_z[i] = self._bubble_spawn_z()
            self._bubble_age[i] = 0
            self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", False)
        self._spawn_timer = 0
        self._vigor = 1.0

    def _step_bubble_anim(self):
        """气泡动画：小球池按"间隔 = BUBBLE_SPAWN_INTERVAL / _vigor"从管底分散区生成一颗、
        逐帧上升（每颗速度差异 + 蛇形摆动）、到当前液面即隐藏（"破掉"）——不是循环归位，
        形成散布+摆动的反应冒泡。

        反应强度 _vigor：每滴试剂滴入/抓起试管复位 1.0 → 逐帧衰减到 VIGOR_FLOOR（先猛后衰）。
        只写子球局部 translate（x/y = 基准 + 摆动、z = 上升，x/y 离轴钳到 BUBBLE_MAX_RADIUS
        防插壁）；随管平移由 Bubbles 父 Xform 承担。仅气泡可见时推进。"""
        if not (self.has_bubbles and self._bubbles_visible):
            return
        # 反应强度衰减（先猛后衰）
        self._vigor = max(self.VIGOR_FLOOR, self._vigor * self.VIGOR_DECAY)
        # 当前液面 = 破灭高度（液面随滴落长高，贴实时液面；留 margin 破在液面下方）
        level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        pop_z = self.TUBE_BOTTOM_Z + level - self.BUBBLE_POP_MARGIN
        # 生成：间隔随 _vigor 加密（vigor 1.0 → 每 4 帧一颗；0.25 → 每 16 帧）
        if self._spawn_timer <= 0:
            for i, active in enumerate(self._bubble_active):
                if not active:
                    self._bubble_active[i] = True
                    self._bubble_z[i] = self._bubble_spawn_z()
                    self._bubble_age[i] = 0
                    self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", True)
                    break
            self._spawn_timer = max(1, round(self.BUBBLE_SPAWN_INTERVAL / self._vigor))
        else:
            self._spawn_timer -= 1
        # 推进在飞气泡：上升（速度差异）+ 蛇形摆动，到液面消失（隐藏，留作复用）
        for i, (bx, by) in enumerate(self._bubble_bases):
            if not self._bubble_active[i]:
                continue
            age = self._bubble_age[i]
            z = self._bubble_z[i] + self.BUBBLE_RISE * self._bubble_speed[i]
            if z >= pop_z:
                self._bubble_active[i] = False
                self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", False)
                continue
            self._bubble_z[i] = z
            # 蛇形摆动（x/y 错相正弦），随后把离轴距离钳到 BUBBLE_MAX_RADIUS 防插管壁
            ph = self._bubble_phase[i]
            wob = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.15 + ph)
            woy = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.13 + ph + 1.7)
            cx, cy = bx + wob, by + woy
            dx, dy = cx - TUBE_XY[0], cy - TUBE_XY[1]
            r = np.hypot(dx, dy)
            if r > self.BUBBLE_MAX_RADIUS:
                s = self.BUBBLE_MAX_RADIUS / r
                cx, cy = TUBE_XY[0] + dx * s, TUBE_XY[1] + dy * s
            self._bubble_age[i] = age + 1
            prim = self.stage.GetPrimAtPath(f"{self._bubbles_path}/Bubble_{i}")
            if prim.IsValid():
                t = prim.GetAttribute("xformOp:translate")
                if t:
                    t.Set(Gf.Vec3d(cx, cy, z))

    def _step_phenomena(self, gripper_pos):
        """震荡停检测 + 气泡消退（复刻 d3l）：加试剂滴入即出现气泡 → 随管震荡持续 →
        **震荡停止后 PHENOMENA_FRAMES（3s）消退**（隐藏气泡；沉淀/分层不随消退）。

        检测"震荡停"：试管已抓起（tube.attached）+ 夹爪在震荡/停留高度区（z ≥ SHAKE_TOP_Z）
        + 连续 SHAKE_STILL_FRAMES 帧夹爪位移 < SHAKE_STOP_EPS。一旦判停 → 排定消退帧
        （当前帧 + PHENOMENA_FRAMES），到帧隐藏气泡。结果存 _shake_stop_frames（
        _step_precipitate 判浑浊褪去、_step_layer 判分层成形共用）。"""
        if not (self.has_bubbles or self.has_precipitate or self.has_layer):
            return
        if self.has_bubbles and self._phenomena_faded:
            return                       # 气泡已消退过（停留期内不反复重排）
        if self.has_bubbles and self._phenomena_fade_frame is not None:
            if self.frame_idx >= self._phenomena_fade_frame:
                # 消退只隐藏气泡；沉淀/分层保留（沉淀留管、分层成形）
                self._set_visibility(self._bubbles_path, False)
                self._bubbles_visible = False
                self._phenomena_faded = True
                self._phenomena_fade_frame = None
                print(f"[d8l] phenomena faded (frame {self.frame_idx})")
            return                       # 倒计时进行中，等消退
        # —— 检测震荡是否已停（尚未排定消退）——
        if not self.tube.attached or gripper_pos[2] < self.SHAKE_TOP_Z:
            self._prev_gripper_pos = None
            self._shake_stop_frames = 0
            return
        move = 0.0
        if self._prev_gripper_pos is not None:
            move = float(np.linalg.norm(
                np.asarray(gripper_pos, dtype=float) - self._prev_gripper_pos))
        self._prev_gripper_pos = np.asarray(gripper_pos, dtype=float)
        if move < self.SHAKE_STOP_EPS:
            self._shake_stop_frames += 1
            if (self.has_bubbles and self._shake_stop_frames >= self.SHAKE_STILL_FRAMES
                    and self._phenomena_fade_frame is None):
                self._phenomena_fade_frame = self.frame_idx + self.PHENOMENA_FRAMES
                print(f"[d8l] shake stopped -> phenomena fade at "
                      f"frame {self._phenomena_fade_frame} "
                      f"(now {self.frame_idx}, +{self.PHENOMENA_FRAMES})")
        else:
            self._shake_stop_frames = 0

    def _step_precipitate(self, gripper_pos):
        """沉淀每帧推进：震荡再悬浮 / 停震先快后慢沉降 + 渲染（柱高=settled、液浊度）。
        2026-08-29 用户：试管被提起期间沉淀总量按 PRECIP_GROW_RATE 逐渐增厚（震荡+悬停都长，
        封顶 PRECIP_MAX=1.65cm），停震沉降时显现为厚实沉淀层；震荡中颗粒悬浮成灰色浑浊云。"""
        if not self.has_precipitate or self._precip_total <= 0:
            return
        stopped = self._shake_stop_frames >= self.SHAKE_STILL_FRAMES
        lifted = (self.tube.attached and gripper_pos[2] >= self.SHAKE_TOP_Z)
        oscillating = False
        if lifted:
            if self._precip_prev is not None:
                horiz = float(np.linalg.norm(
                    np.asarray(gripper_pos[:2], dtype=float) - self._precip_prev[:2]))
                oscillating = (horiz > self.SHAKE_STOP_EPS) and not stopped
            self._precip_prev = np.asarray(gripper_pos, dtype=float)
        else:
            self._precip_prev = None
        if oscillating:
            # 震荡中：反应继续 → 沉淀总量逐渐增厚（封顶 PRECIP_MAX）；颗粒大多悬浮成浑浊云
            self._precip_total = min(self.PRECIP_MAX,
                                     self._precip_total + self.PRECIP_GROW_RATE)
            target = self._precip_total * self.PRECIP_RESUSPEND_FLOOR
            self._precip_settled += (target - self._precip_settled) \
                * self.PRECIP_RESUSPEND_RATE
            self._cloud_frac += (1.0 - self._cloud_frac) * self.PRECIP_RESUSPEND_RATE
        else:
            # 未震荡（高位悬停沉降 / 放回）：颗粒回沉、浑浊云渐褪 → 沉降层变厚；提起期间仍增厚
            if lifted:
                self._precip_total = min(self.PRECIP_MAX,
                                         self._precip_total + self.PRECIP_GROW_RATE)
            self._precip_settled += (self._precip_total - self._precip_settled) \
                * self.PRECIP_FADE_RATE
            self._cloud_frac += (0.0 - self._cloud_frac) * self.PRECIP_FADE_RATE
        settled = max(self._precip_settled, 0.0)
        tube_now = self._get_obj_world(self.TUBE)
        base = tube_now if tube_now is not None else TUBE_ORIG
        prim = self.stage.GetPrimAtPath(self.PRECIPITATE)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(settled)
            self.object_utils.set_object_position(
                self.PRECIPITATE, (base[0], base[1], base[2] + settled / 2))
        liquid_level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        cloud_h = liquid_level * self._cloud_frac
        cprim = self.stage.GetPrimAtPath(self.PRECIPITATE_CLOUD)
        if cprim.IsValid():
            UsdGeom.Cylinder(cprim).GetHeightAttr().Set(cloud_h)
            self.object_utils.set_object_position(
                self.PRECIPITATE_CLOUD, (base[0], base[1], base[2] + cloud_h / 2))
            self._set_visibility(self.PRECIPITATE_CLOUD, cloud_h > 0.0005)

    def _step_color_liquid(self):
        """3 段变色液柱每帧渲染：每段变色柱高度 = 当前液面 × 该段 _color_frac，顶贴液面
        向下扩散。三段半径逐段递增嵌套，后段在外层盖前段——最终色=最后非 clear 段色。
        定位以试管当前管底为基准（随管平移）。"""
        liquid_level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        tube_now = self._get_obj_world(self.TUBE)
        base = tube_now if tube_now is not None else TUBE_ORIG
        for stage_key in self.COLOR_STAGES:
            path = self._color_paths[stage_key]
            frac = self._color_fracs[stage_key]
            if not path or frac <= 0:
                continue
            h_color = liquid_level * frac
            if h_color <= 0.0005:
                self._set_visibility(path, False)
                continue
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                UsdGeom.Cylinder(prim).GetHeightAttr().Set(h_color)
                self.object_utils.set_object_position(
                    path, (base[0], base[1], base[2] + liquid_level - h_color / 2))
                self._set_visibility(path, True)

    def _step_layer(self):
        """分层现象：震荡停止后（_shake_stop_frames 判停）底部重相渐渐成形——琥珀层从
        **沉降层顶**（无沉淀=管底）向上长到 LAYER_FRACTION × 沉降层之上的液面高。震荡中
        不显示（混合均匀无分层），停震后 LAYER_FORM_FRAMES 内渐渐成形。2026-08-29 用户
        "没看到琥珀色"→ 琥珀层上移到沉降层之上，避免被沉淀圆柱（r 0.0088 > 琥珀 0.0087）
        从外圈遮挡。"""
        if not self.has_layer:
            return
        stopped = self._shake_stop_frames >= self.SHAKE_STILL_FRAMES
        if stopped and not self._layer_formed:
            self._layer_frac += 1.0 / self.LAYER_FORM_FRAMES
            if self._layer_frac >= 1.0:
                self._layer_frac = 1.0
                self._layer_formed = True
        liquid_level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        sed = self._precip_settled if self.has_precipitate else 0.0
        band = max(0.0, liquid_level - sed)
        h_layer = band * self.LAYER_FRACTION * self._layer_frac
        if h_layer <= 0.0005:
            self._set_visibility(self.LAYER, False)
            return
        tube_now = self._get_obj_world(self.TUBE)
        base = tube_now if tube_now is not None else TUBE_ORIG
        prim = self.stage.GetPrimAtPath(self.LAYER)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h_layer)
            self.object_utils.set_object_position(
                self.LAYER, (base[0], base[1], base[2] + sed + h_layer / 2))
            self._set_visibility(self.LAYER, True)

    # ------------------------------------------------------------------
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
