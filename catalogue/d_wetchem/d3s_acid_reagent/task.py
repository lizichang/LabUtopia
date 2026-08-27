# -*- coding: utf-8 -*-
"""D3-S 固体样品 + 酸性试剂滴加反应任务：药匙挖粉 + 酸滴管吸酸滴酸 + 试管震荡 + 酸现象。

D3-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加酸性试剂」。挖粉动作（①PickSpatula →
②ReturnSpatula，d2s 元动作包复用，药匙/表面皿/粉末/试管/试管架坐标逐字一致）与试管
震荡（④TubeShakePass，d2s 复用）都不变；酸滴管（③AcidPass）照 B2 水平横夹 + d3l 单段
滴加。本任务负责三块生命周期/效果：

  1) 药匙（d2s 同款）：每帧把药匙世界位姿写为 _T_HELD · tool_world（6-DOF 随夹爪旋转：
     竖直提起 → 过架顶 → 法兰转 → 回卷倒粉）。勺尖 = 夹爪 + 0.134·tool+X，挖粉/倒粉
     判定照 d2s（_scoop_starting/_vertical_over_mouth），粉末下落动画（PowderDrop 父 +
     14 粒）与 d2s 逐字一致。
  2) 酸滴管（B2 同款水平横夹）：滴管是静态碰撞体，吸附期逐帧把世界 4x4 矩阵写为
     _T_HELD_DROPPER · tool_world（沿 tool+X 伸 0.13m，手指朝前 ORIENT_FWD = 滴管竖直挂
     夹爪下、尖嘴朝下）。尖嘴 = 夹爪 + 0.13·tool+X。生命周期 rest → attached → squeezed
     → filled → dropped → released（一次持握内循环吸酸-滴酸，acid_cycles 遍，中途不松）。
  3) 试管震荡（d2s 同款纯平移）+ 酸现象（d3l 同款）：拿起试管震荡后，管内固体粉末
     （TubeSample，白色）随管平移；酸液柱 TubeDrops 逐滴生长 + 气泡（Bubbles_<色>）
     + 沉淀（Precipitate）/浑浊云（PrecipitateCloud）+ 液体变色（TubeDropsInput_<色> 输入色
     → 震荡后 TubeDropsColor_<色> 输出色，几何实现，headless 下运行时改材质不渲染）。
     加酸滴入即出现输入色，震荡停止后 PHENOMENA_FRAMES 消退（气泡隐藏、沉淀柱保留管内）。

驱动 prim（d3s_acid_reagent.usd，scripts/gen_d3s_scene.py 生成）：
  /World/Spatula / SurfaceDish / SamplePowder / TestTubeRack / TestTube  挖粉同 d2s
  /World/DropperAcid / HClBottle  酸试剂（滴管进主试管架空孔与药匙同架 + 盐酸瓶）
  /World/PowderOnSpoon / PowderDrop  药匙上粉堆 / 药粉下落（task 动画）
  /World/TubeSample  管内白色固体粉末（⑬ 倒粉后显示，白粉固定）
  /World/TubeDrops  管内酸液柱（滴入后逐滴生长）
  /World/Precipitate / PrecipitateCloud  沉淀 / 浑浊云（cfg.has_precipitate）
  /World/Bubbles_<clear|red|blue|green|purple>  气泡组 ×5（颜色跟随液体变色）
  /World/TubeDropsInput_<red|blue|green|purple>  输入色液柱（cfg.input_color，滴入后震荡前）
  /World/TubeDropsColor_<red|blue|green|purple>  输出色液柱（cfg.liquid_color，震荡反应后）
  /World/DropperDrop  挤胶头滴落串（task 动画坠落）
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_ACID_REST, DROP_ACID_GRASP, ACID_BOTTLE_XY,
    EFFECT_TUBE_DROPS, EFFECT_PRECIPITATE, EFFECT_PRECIPITATE_CLOUD,
    EFFECT_DROPPER_DROP, EFFECT_TUBE_SAMPLE,
    SPAT_GRASP,   # D3-S 药匙家用：第一列第3排 (0.659,0.3209,0.94)（非 d2s 第二列第4排）
)
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.constants import (
    GRIP_SPATULA, SPAT_HEAD_DIST,
    POWDER_TOP_Z, POWDER_X, DISH_XY,
    GRIP_TUBE, TUBE_GRASP_TCP, TUBE_ORIG_Z, TUBE_HELD_OFFSET_Z,
    TUBE_XY, TUBE_MOUTH_Z,
)

# 药匙相对夹爪持握矩阵（d2s 同款）：平移 (0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、
# toolZ→(-1,0,0)）。平移在最后一行（USD 行向量约定）。合成 = _T_HELD · tool_world（先作用药匙
# 局部系、再 tool_world 到世界；写反旋转作用到世界系 → 药匙翻走/飞桌面下）。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)

# 酸滴管相对夹爪持握矩阵（B2 同款水平横夹）：旋转同药匙、平移沿 tool+X 伸 TIP_OFFSET=0.13。
# 手指朝前 ORIENT_FWD（tool+X=世界 -Z）→ 滴管竖直挂夹爪下、尖嘴朝下。合成同 _T_HELD。
_T_HELD_DROPPER = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                              0.0, -1.0, 0.0, 0.0,
                              -1.0, 0.0, 0.0, 0.0,
                              TIP_OFFSET, 0.0, 0.0, 1.0)

# 试管架内竖插位姿（管底 = 原点）：平移 (0.659,0.241,0.806)（同 d2s TUBE_ORIG_Z）
TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z])


class _DropperLifecycle:
    """单支酸滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    B2 同款水平横夹：滴管沿 tool+X 伸出 0.13m，随夹爪旋转（ORIENT_FWD 竖直挂下、尖嘴朝下）。
    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    盐酸瓶口 xy（排空气/浸液区，z 不区分）
      tube_xy      试管口 xy（滴液区）
    """

    def __init__(self, task, name, path, orig, grasp, bottle_xy, tube_xy):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.bottle_xy = np.array(bottle_xy)
        self.tube_xy = np.array(tube_xy)
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
        self.task._set_dropper_world(_dropper_rest_matrix())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._dropper_held_matrix()
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（d2s 同款全矩阵 ease；
            # 附着手腕已回正→旋转差≈0，只 eases 平移，消除闭合瞬间闪现吸附）。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_dropper_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_dropper_world(held)
                print(f"[d3s] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（矩阵持握：滴管随夹爪旋转，全程 ORIENT_FWD 竖直挂下）
        self.task._set_dropper_world(self.task._dropper_held_matrix())
        tip = self.task._dropper_tip_pos()

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d3s] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸酸
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "filled"
                self.filled = True
                print(f"[d3s] {self.name} filled (aspirated)")
        elif self.state == "filled":
            # 试管口区挤胶头滴酸（尖嘴在管口上方 25mm，液滴可见坠落）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.tube_xy, tip)):
                self.state = "dropped"
                self.dropped = True
                self.task._on_drop(self)
                print(f"[d3s] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的 cycle
            # 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "squeezed"
                print(f"[d3s] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_dropper_world(_dropper_rest_matrix())
                self.state = "rest"
                print(f"[d3s] {self.name} released to rack -> rest")


class D3SAcidReagentTask(BaseTask):
    """D3-S 固体样品 + 酸性试剂滴加反应任务：药匙挖粉 → 酸滴管滴酸 → 试管震荡 + 酸现象。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 药匙（d2s 同款）
    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array(SPAT_GRASP)
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量
    POWDER_EFFECT = "/World/PowderOnSpoon"
    POWDER_DROP = "/World/PowderDrop"
    POWDER_DROPS = 14
    POWDER_STAGGER = 3
    POWDER_HANG = 4
    POWDER_FALL = 14
    POWDER_LAND_Z = 0.818   # 药粉落点=粉堆顶面（LIQUID_BASE_Z，2026-08-26 粉末贴管底平铺）

    # 酸滴管（B2 同款水平横夹）
    DROPPER = "/World/DropperAcid"
    TUBE_DROPS = EFFECT_TUBE_DROPS
    PRECIPITATE = EFFECT_PRECIPITATE
    PRECIPITATE_CLOUD = EFFECT_PRECIPITATE_CLOUD
    DROPPER_DROP = EFFECT_DROPPER_DROP
    TUBE_DROPS_COLOR = "/World/TubeDropsColor_{name}"
    TUBE_DROPS_INPUT = "/World/TubeDropsInput_{name}"

    # 试管（d2s 同款纯平移）
    TUBE = "/World/TestTube"
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)
    TUBE_GRIP_CLOSED = GRIP_TUBE + 0.004   # 夹紧阈值：grip 0.0096 + 4mm 裕量
    TUBE_SAMPLE = EFFECT_TUBE_SAMPLE       # 管内白色固体粉末
    # 2026-08-26 用户"试管还没摇晃时看不到粉末沉淀"：粉末柱贴管底平铺（0.806..0.818），
    # rest 中心 = 0.806 + 0.012/2 = 0.812。酸液柱底面改到粉顶（LIQUID_BASE_Z），白粉始终
    # 露在酸液下方，倒粉后立即可见（旧 0.84 悬在酸柱中部被不透明酸盖住）。
    TUBE_SAMPLE_REST = np.array([TUBE_XY[0], TUBE_XY[1], 0.812])   # 粉末柱 rest（gen BUILTIN）
    TUBE_SAMPLE_H = 0.012                   # 粉末柱高（与 gen TUBE_SAMPLE_H 对齐）
    TUBE_MOUTH = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z])

    # 管内酸液柱逐滴生长（底面=白粉顶面 LIQUID_BASE_Z；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    LIQUID_BASE_Z = TUBE_BOTTOM_Z + TUBE_SAMPLE_H      # 0.818：酸液底面 = 粉堆顶面
    LIQUID_OFFSET = LIQUID_BASE_Z - TUBE_BOTTOM_Z      # 0.012：酸液相对管底的抬升量
    DROP_LEVEL_STEP = 0.004
    DROP_LEVEL_MAX = 0.060

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5
    DROP_FALL = 16
    DROP_STAGGER = 6

    # 现象时序（d3l 同款）：加酸滴入即出现 → 随管震荡持续 → 震荡停止后 PHENOMENA_FRAMES 消退
    SHAKE_TOP_Z = 1.02
    SHAKE_STOP_EPS = 0.0005
    SHAKE_STILL_FRAMES = 20
    PHENOMENA_FRAMES = 180

    # 气泡动画（d3l 同款中等档）：小球池从白粉顶面（反应位）生成 → 穿液柱上升 → 到液面消失
    N_BUBBLES = 40
    BUBBLE_SPAWN_Z = LIQUID_BASE_Z            # 0.818：无沉淀时泡在粉堆顶面冒出
    BUBBLE_SPAWN_Z_PRECIP = LIQUID_BASE_Z + 0.0015   # 0.8195：有沉淀时在沉层上沿
    BUBBLE_POP_MARGIN = 0.002
    BUBBLE_RISE = 0.0010
    BUBBLE_SPAWN_INTERVAL = 4
    BUBBLE_WOBBLE_AMP = 0.0012
    BUBBLE_MAX_RADIUS = 0.006
    VIGOR_DECAY = 0.994
    VIGOR_FLOOR = 0.25

    # 沉淀现象（d3l 同款）：逐滴增厚 + 先浊后沉 + 震荡再悬浮
    PRECIP_DROP_STEP = 0.0008
    PRECIP_MAX = 0.008
    PRECIP_RESUSPEND_RATE = 0.008
    PRECIP_FADE_RATE = 0.008
    PRECIP_RESUSPEND_FLOOR = 0.1
    PRECIP_SPAWN_MARGIN = 0.0015

    # 滴加酸后液体变色（d3l 同款）：输入色液柱逐滴长高（顶贴液面向下扩散），震荡时渐渐
    # 过渡到输出色（输入色↓、输出色↑）。COLOR_STEP=每滴酸输入色进度步长；COLOR_FADE_RATE=
    # 震荡过渡每帧步长（"震荡之后液体才慢慢变成输出色"，每帧 ~1.2% → 约 80 帧≈1.3s 渐满）
    COLOR_STEP = 0.25
    COLOR_FADE_RATE = 0.012

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 药匙/滴管/试管是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被
        # 物理干扰），与 d2s/d3l/B2 同模式。
        self.spatula_path = self.SPATULA_PATH
        self._disable_collision(self.spatula_path)
        self._disable_collision(self.DROPPER)
        self._disable_collision(self.TUBE)

        # 阈值（config 可调，d2s/B2/d3l 同款默认）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.0035)

        # 药匙状态（d2s 同款）
        self._near_frames = 0
        self.spatula_state = "rest"     # rest / attached / released
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None        # 上一帧法兰角（joint7 索引 6），判定⑨挖粉旋转开始

        # 酸滴管生命周期（B2 同款水平横夹）
        self.dropper = _DropperLifecycle(
            self, "acid", self.DROPPER, DROP_ACID_REST, DROP_ACID_GRASP,
            ACID_BOTTLE_XY, TUBE_XY)
        self._tube_drop_count = 0       # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []           # 滴落动画队列（含 delay/t/hang/fall）

        # 试管震荡状态（d2s 同款纯平移）
        self.tube_state = "rest"        # rest / attached / released
        self._tube_near_frames = 0
        self._tube_pos = TUBE_ORIG.copy()

        # 酸现象（d3l 同款）：气泡 / 沉淀 / 液体变色
        self.has_bubbles = bool(getattr(cfg, "has_bubbles", False))
        self.has_precipitate = bool(getattr(cfg, "has_precipitate", False))
        self.liquid_color = str(getattr(cfg, "liquid_color", "clear")).strip().lower()
        self._color_path = (self.TUBE_DROPS_COLOR.format(name=self.liquid_color)
                            if self.liquid_color != "clear" else None)
        self._color_frac = 0.0
        # 输入酸溶液颜色（滴入后、震荡反应前）：滴酸阶段逐滴长高输入色液柱，震荡后渐渐
        # 过渡到输出色（_color_path）。clear=无输入色（酸无色，直接等震荡出输出色）
        self.input_color = str(getattr(cfg, "input_color", "clear")).strip().lower()
        self._input_path = (self.TUBE_DROPS_INPUT.format(name=self.input_color)
                            if self.input_color != "clear" else None)
        self._input_frac = 0.0
        self._transition_started = False   # 震荡一旦开始即锁 true，渐变持续到完成
        self._oscillating = False          # 本帧是否在水平摇晃（_step_oscillation 写）
        # 气泡组颜色跟随液体变色（clear=原浅天蓝，其余=目标色；gen 预烘焙 5 组，运行时 show 一组）
        self._bubbles_path = (f"/World/Bubbles_{self.liquid_color}"
                              if self.liquid_color != "clear" else "/World/Bubbles_clear")
        self.BUBBLE_SPAWN_Z = (self.BUBBLE_SPAWN_Z_PRECIP if self.has_precipitate
                               else self.BUBBLE_SPAWN_Z)
        self._bubbles_visible = False
        self._bubble_bases = []
        self._bubble_z = []
        self._bubble_active = []
        self._bubble_age = []
        self._bubble_speed = []
        self._bubble_phase = []
        self._spawn_timer = 0
        self._vigor = 1.0
        self._init_bubble_anim()

        # 沉淀状态：_precip_total=已析出总量；_precip_settled=当前沉降层高
        self._precip_total = 0.0
        self._precip_settled = 0.0
        self._cloud_frac = 0.0
        self._precip_prev = None        # 摇晃判定用上一帧夹爪位置
        # 现象时序：震荡停检测 + 消退倒计时
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        self._phenomena_fade_frame = None
        self._phenomena_faded = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        # 药匙复位（d2s 同款）
        self.spatula_state = "rest"
        self._near_frames = 0
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None
        self._set_spatula_world(_spatula_rest_matrix())
        self._set_visibility(self.POWDER_EFFECT, False)
        self._set_visibility(self.POWDER_DROP, False)
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        # 还原粉堆尺寸（上一集 _shrink_powder_blob 缩到 12%）
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005)
            cyl.GetHeightAttr().Set(0.005)
        # 酸滴管复位
        self.dropper.reset()
        self._tube_drop_count = 0
        self._drop_queue = []
        for p in (self.TUBE_DROPS, self.PRECIPITATE, self.PRECIPITATE_CLOUD,
                  self._bubbles_path, self.DROPPER_DROP):
            self._set_visibility(p, False)
        # 变色复位（输入色 + 输出色）：进度清零、液柱隐藏 + 高度归零、过渡锁复位
        self._color_frac = 0.0
        if self._color_path:
            self._set_visibility(self._color_path, False)
            p = self.stage.GetPrimAtPath(self._color_path)
            if p.IsValid():
                UsdGeom.Cylinder(p).GetHeightAttr().Set(0.0)
        self._input_frac = 0.0
        if self._input_path:
            self._set_visibility(self._input_path, False)
            p = self.stage.GetPrimAtPath(self._input_path)
            if p.IsValid():
                UsdGeom.Cylinder(p).GetHeightAttr().Set(0.0)
        self._transition_started = False
        self._oscillating = False
        # 现象时序复位：气泡回原点、沉淀清零
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        self._phenomena_fade_frame = None
        self._phenomena_faded = False
        self._bubbles_visible = False
        self._reset_bubble_anim()
        self.object_utils.set_object_position(self._bubbles_path, (0.0, 0.0, 0.0))
        self._precip_total = 0.0
        self._precip_settled = 0.0
        self._cloud_frac = 0.0
        self._precip_prev = None
        pprim = self.stage.GetPrimAtPath(self.PRECIPITATE)
        if pprim.IsValid():
            UsdGeom.Cylinder(pprim).GetHeightAttr().Set(0.0)
        cprim = self.stage.GetPrimAtPath(self.PRECIPITATE_CLOUD)
        if cprim.IsValid():
            UsdGeom.Cylinder(cprim).GetHeightAttr().Set(0.0)
        # 试管震荡复位（d2s 同款）：回架 + 粉末回 rest + 尺寸还原
        self.tube_state = "rest"
        self._tube_near_frames = 0
        self._tube_pos = TUBE_ORIG.copy()
        self._set_tube_world(TUBE_ORIG)
        self._set_visibility(self.TUBE_SAMPLE, False)
        self._set_tube_column(self.TUBE_SAMPLE, self.TUBE_SAMPLE_H, self.TUBE_SAMPLE_REST, r=0.006)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_spatula()              # ①/② 药匙挖粉/倒粉（d2s 同款）
        self._step_powder_anim()            # 药粉下落动画独立推进
        self.dropper.step(gripper_pos, opening)   # ③ 酸滴管吸酸/滴酸（B2 同款）
        self._step_drop_anim()              # 滴落动画独立推进
        self._step_bubble_anim()            # 气泡上升循环（可见时推进）
        self._update_tube()                 # ④ 试管震荡持握（d2s 同款纯平移）
        self._step_phenomena(gripper_pos)   # 现象时序：震荡停→3s 后消退
        self._step_oscillation(gripper_pos) # 摇晃检测（沉淀/变色共用 _oscillating）
        self._step_precipitate(gripper_pos) # 沉淀：逐滴增厚/先浊后沉/震荡再悬浮
        self._step_color_liquid(gripper_pos) # 变色：输入色→输出色渐变（震荡后触发）
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "powder_on_spoon": self.powder_on_spoon,
            "poured": self.poured,
            "acid_attached": self.dropper.attached,
            "acid_filled": self.dropper.filled,
            "acid_dropped": self.dropper.dropped,
            "tube_state": self.tube_state,
        })

    def on_task_complete(self, success):
        print(f"[d3s] episode done success={success} "
              f"spatula={self.spatula_state} poured={self.poured} "
              f"acid_dropped={self.dropper.dropped} tube={self.tube_state}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 药匙持握 / 效果（d2s 同款）
    # ------------------------------------------------------------------
    def _update_spatula(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        # 法兰（joint7，索引 6）是否在旋转：⑨ 挖粉起判定信号
        flange_rotating = (self._prev_flange is not None
                           and abs(joints[6] - self._prev_flange) > 0.005)
        self._prev_flange = float(joints[6])

        if self.spatula_state == "rest":
            if self._near_grasp(gripper_pos, self.SPAT_GRASP):
                self._near_frames += 1
            else:
                self._near_frames = 0
            if self._near_grasp(gripper_pos, self.SPAT_GRASP) and opening < self.gripper_open_threshold:
                self._ease_spatula_to_gripper(gripper_pos)
            if (self._near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.SPAT_GRIP_CLOSED):
                self.spatula_state = "attached"
                self._set_spatula_from_gripper()
                print(f"[d3s] spatula attached (grip={opening:.4f})")

        elif self.spatula_state == "attached":
            self._set_spatula_from_gripper()
            tip = self._spoon_tip_pos(gripper_pos)
            if not self.powder_on_spoon and self._scoop_starting(tip, flange_rotating):
                self.powder_on_spoon = True
                self._set_visibility(self.POWDER_EFFECT, True)
                print(f"[d3s] powder on spoon (tip={np.round(tip, 3)})")
            if self.powder_on_spoon and not self.poured:
                self.object_utils.set_object_position(
                    self.POWDER_EFFECT, tip + np.array([0.0, 0.0, 0.003]))
                if not self.powder_falling and self._vertical_over_mouth(tip, joints):
                    self.powder_falling = True
                    self._start_powder_fall(tip)
            if opening > self.gripper_open_threshold:
                self.spatula_state = "released"
                self._set_spatula_world(_spatula_rest_matrix())
                self._set_visibility(self.POWDER_EFFECT, False)
                print("[d3s] spatula released to rack")

    def _set_spatula_from_gripper(self):
        self._set_spatula_world(_T_HELD * self._tool_world())

    def _set_spatula_world(self, world_matrix):
        prim = self.stage.GetPrimAtPath(self.spatula_path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _ease_spatula_to_gripper(self, gripper_pos, k=0.18):
        target = _T_HELD * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.spatula_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_spatula_world(_blend_world(cur, target, k))

    def _spoon_tip_pos(self, gripper_pos):
        """勺尖 = 夹爪 + 0.134 × 夹爪局部 +X（勺头方向）世界方向。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]
        return np.asarray(gripper_pos, dtype=float) + SPAT_HEAD_DIST * x_dir

    def _scoop_starting(self, tip, flange_rotating):
        """⑨ 法兰开始旋转（挖粉起）判定：法兰正在旋转 且 勺尖在粉丘附近（松带）。"""
        near = (abs(tip[0] - POWDER_X) < 0.04
                and abs(tip[1] - DISH_XY[1]) < 0.08
                and tip[2] < POWDER_TOP_Z + 0.02)
        return flange_rotating and near

    def _vertical_over_mouth(self, tip, joints):
        """勺尖水平近管口且已降到回卷中段高度 → 开始倒粉（⑬ 进行到约一半掉入试管）。"""
        above = tip[2] < TUBE_MOUTH_Z + 0.05
        near = np.linalg.norm(tip[:2] - np.array([TUBE_XY[0], TUBE_XY[1]])) < 0.06
        return above and near

    def _start_powder_fall(self, tip):
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        start = tip + np.array([0.0, 0.0, -0.004])
        for i in range(self.POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * self.POWDER_STAGGER,
                "t": 0,
                "start": start.copy(),
                "target": np.array([TUBE_XY[0], TUBE_XY[1], self.POWDER_LAND_Z]),
                "hang": self.POWDER_HANG, "fall": self.POWDER_FALL,
            })
        self._set_visibility(self.POWDER_DROP, True)
        print(f"[d3s] powder fall started from {np.round(start, 3)}")

    def _step_powder_anim(self):
        if not self._powder_queue:
            return
        remaining = []
        landed = self.POWDER_DROPS - len(self._powder_queue)
        for d in self._powder_queue:
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
                self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", False)
                landed += 1
                continue
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.POWDER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._powder_queue = remaining
        self._shrink_powder_blob(landed / self.POWDER_DROPS)
        if not remaining:
            self._set_visibility(self.POWDER_DROP, False)
            if self.powder_falling:
                self.powder_falling = False
                if not self.poured:
                    self.poured = True
                    self._set_visibility(self.POWDER_EFFECT, False)
                    self._set_visibility(self.TUBE_SAMPLE, True)
                    print("[d3s] powder poured into tube")

    def _shrink_powder_blob(self, landed_frac):
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005 * remain)
            cyl.GetHeightAttr().Set(0.005 * remain)

    # ------------------------------------------------------------------
    # 酸滴管矩阵持握 / 判定 / 滴落动画（B2 同款水平横夹 + d3l 现象）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dropper_held_matrix(self):
        return _T_HELD_DROPPER * self._tool_world()

    def _set_dropper_world(self, world_matrix):
        prim = self.stage.GetPrimAtPath(self.DROPPER)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _dropper_tip_pos(self):
        """滴管尖嘴（局部原点）世界坐标 = 物理夹爪 + 0.13×tool+X 方向（B2 同款混合数据源）。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]
        gripper_pos = self.robot.get_gripper_position()
        return np.asarray(gripper_pos, dtype=float) + TIP_OFFSET * x_dir

    def _ease_dropper_world(self, target, k=0.18):
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.DROPPER)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_dropper_world(_blend_world(cur, target, k))

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        tip = self._dropper_tip_pos()
        start = tip + np.array([0.0, 0.0, -0.005])
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._tube_drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            # 液滴落在当前液面（底面=白粉顶 LIQUID_BASE_Z），沉入 3mm 与液柱融合
            target = np.array([TUBE_XY[0], TUBE_XY[1],
                               self.LIQUID_BASE_Z + level - 0.003])
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
        print(f"[d3s] squeeze ({dropper.name}) -> {self.DROPS_PER_SQUEEZE} drops spawned")

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
        """液滴落定：管内酸液面长到高度 h（从白粉顶面 LIQUID_BASE_Z 起算），加酸时触发
        现象（气泡/沉淀/变色）。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self.TUBE_DROPS, (TUBE_XY[0], TUBE_XY[1], self.LIQUID_BASE_Z + h / 2))
        self._set_visibility(self.TUBE_DROPS, True)
        if name == "acid":
            if self.has_bubbles:
                self._set_visibility(self._bubbles_path, True)
                self._bubbles_visible = True
                self._vigor = 1.0
            if self.has_precipitate:
                self._set_visibility(self.PRECIPITATE, True)
                self._precip_total = min(self.PRECIP_MAX,
                                         self._precip_total + self.PRECIP_DROP_STEP)
                self._precip_settled = self._precip_total
            if self._input_path:
                self._input_frac = min(1.0, self._input_frac + self.COLOR_STEP)
                self._set_visibility(self._input_path, True)
        print(f"[d3s] tube liquid level h={h:.3f}")

    # ------------------------------------------------------------------
    # 试管震荡持握（d2s 同款纯平移）+ 管内效果跟随
    # ------------------------------------------------------------------
    def _update_tube(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        held = np.asarray(gripper_pos, dtype=float) + np.array([0.0, 0.0, TUBE_HELD_OFFSET_Z])

        if self.tube_state == "rest":
            if self._near_grasp(gripper_pos, self.TUBE_GRASP):
                self._tube_near_frames += 1
            else:
                self._tube_near_frames = 0
            if (self._tube_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.TUBE_GRIP_CLOSED):
                self.tube_state = "attached"
                self._set_tube_world(held)
                print(f"[d3s] tube attached (grip={opening:.4f})")
        elif self.tube_state == "attached":
            self._set_tube_world(held)
            self._follow_tube_effects(held)
            if opening > self.gripper_open_threshold:
                self.tube_state = "released"
                self._set_tube_world(TUBE_ORIG)
                self._follow_tube_effects(TUBE_ORIG)
                print("[d3s] tube released to rack")

    def _set_tube_world(self, pos):
        self._tube_pos = np.asarray(pos, dtype=float)
        self.object_utils.set_object_position(self.TUBE, self._tube_pos)

    def _follow_tube_effects(self, tube_pos):
        """试管被拿起时管内固体粉末 + 酸液柱 + 气泡随管平移（保持相对管底偏移）。"""
        delta = np.asarray(tube_pos, dtype=float) - TUBE_ORIG
        self.object_utils.set_object_position(self.TUBE_SAMPLE, self.TUBE_SAMPLE_REST + delta)
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            h = float(UsdGeom.Cylinder(prim).GetHeightAttr().Get() or 0.0)
            self.object_utils.set_object_position(
                self.TUBE_DROPS, (tube_pos[0], tube_pos[1],
                                  tube_pos[2] + self.LIQUID_OFFSET + h / 2))
        if self.has_bubbles:
            self.object_utils.set_object_position(self._bubbles_path, delta)

    # ------------------------------------------------------------------
    # 酸现象（d3l 同款）：气泡上升 / 沉淀 / 液体变色
    # ------------------------------------------------------------------
    def _bubble_spawn_z(self):
        # 泡从白粉顶面（反应位）冒出；有沉淀时再浮到沉层上沿
        if self.has_precipitate:
            return self.LIQUID_BASE_Z + self._precip_settled + self.PRECIP_SPAWN_MARGIN
        return self.BUBBLE_SPAWN_Z

    def _init_bubble_anim(self):
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
            self._bubble_speed.append(0.85 + 0.3 * ((i * 37) % 100) / 100.0)
            self._bubble_phase.append((i * 0.7) % (2.0 * np.pi))
            self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", False)
        self._spawn_timer = 0

    def _reset_bubble_anim(self):
        for i in range(len(self._bubble_active)):
            self._bubble_active[i] = False
            self._bubble_z[i] = self._bubble_spawn_z()
            self._bubble_age[i] = 0
            self._set_visibility(f"{self._bubbles_path}/Bubble_{i}", False)
        self._spawn_timer = 0
        self._vigor = 1.0

    def _step_bubble_anim(self):
        if not (self.has_bubbles and self._bubbles_visible):
            return
        self._vigor = max(self.VIGOR_FLOOR, self._vigor * self.VIGOR_DECAY)
        level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        pop_z = self.LIQUID_BASE_Z + level - self.BUBBLE_POP_MARGIN
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
        """现象时序（d3l 同款）：加酸滴入即出现 → 随管震荡持续 → 震荡停止后 PHENOMENA_FRAMES
        消退（气泡隐藏、沉淀柱保留管内）。"""
        if not (self.has_bubbles or self.has_precipitate):
            return
        if self._phenomena_faded:
            return
        if self._phenomena_fade_frame is not None:
            if self.frame_idx >= self._phenomena_fade_frame:
                self._set_visibility(self._bubbles_path, False)
                self._bubbles_visible = False
                self._phenomena_faded = True
                self._phenomena_fade_frame = None
                print(f"[d3s] phenomena faded (frame {self.frame_idx})")
            return
        if self.tube_state != "attached" or gripper_pos[2] < self.SHAKE_TOP_Z:
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
            if self._shake_stop_frames >= self.SHAKE_STILL_FRAMES:
                self._phenomena_fade_frame = self.frame_idx + self.PHENOMENA_FRAMES
                print(f"[d3s] shake stopped -> phenomena fade at "
                      f"frame {self._phenomena_fade_frame} (now {self.frame_idx})")
        else:
            self._shake_stop_frames = 0

    def _step_oscillation(self, gripper_pos):
        """检测试管是否正在水平摇晃（沉淀再悬浮 + 变色过渡共用）。复用沉淀判定：试管已拎起
        到震荡高度 + 上一帧有水平位移 + 未判停。结果存 self._oscillating（本帧有效）。"""
        stopped = self._shake_stop_frames >= self.SHAKE_STILL_FRAMES
        lifted = (self.tube_state == "attached" and gripper_pos[2] >= self.SHAKE_TOP_Z)
        oscillating = False
        if lifted:
            if self._precip_prev is not None:
                horiz = float(np.linalg.norm(
                    np.asarray(gripper_pos[:2], dtype=float) - self._precip_prev[:2]))
                oscillating = (horiz > self.SHAKE_STOP_EPS) and not stopped
            self._precip_prev = np.asarray(gripper_pos, dtype=float)
        else:
            self._precip_prev = None
        self._oscillating = oscillating

    def _step_precipitate(self, gripper_pos):
        """沉淀每帧推进（d3l 同款）：震荡再悬浮 / 停震先快后慢沉降 + 渲染。定位以试管当前
        管底（self._tube_pos）为基准，随管平移。摇晃判定复用 _step_oscillation 的 _oscillating。"""
        if not self.has_precipitate or self._precip_total <= 0:
            return
        oscillating = self._oscillating
        if oscillating:
            target = self._precip_total * self.PRECIP_RESUSPEND_FLOOR
            self._precip_settled += (target - self._precip_settled) \
                * self.PRECIP_RESUSPEND_RATE
            self._cloud_frac += (1.0 - self._cloud_frac) * self.PRECIP_RESUSPEND_RATE
        else:
            self._precip_settled += (self._precip_total - self._precip_settled) \
                * self.PRECIP_FADE_RATE
            self._cloud_frac += (0.0 - self._cloud_frac) * self.PRECIP_FADE_RATE
        settled = max(self._precip_settled, 0.0)
        base = self._tube_pos
        liquid_base = base[2] + self.LIQUID_OFFSET   # 沉淀/云底面 = 白粉顶面（反应位）
        prim = self.stage.GetPrimAtPath(self.PRECIPITATE)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(settled)
            self.object_utils.set_object_position(
                self.PRECIPITATE, (base[0], base[1], liquid_base + settled / 2))
        liquid_level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        cloud_h = liquid_level * self._cloud_frac
        cprim = self.stage.GetPrimAtPath(self.PRECIPITATE_CLOUD)
        if cprim.IsValid():
            UsdGeom.Cylinder(cprim).GetHeightAttr().Set(cloud_h)
            self.object_utils.set_object_position(
                self.PRECIPITATE_CLOUD, (base[0], base[1], liquid_base + cloud_h / 2))
            self._set_visibility(self.PRECIPITATE_CLOUD, cloud_h > 0.0005)

    def _set_color_column(self, path, h, center):
        """写变色液柱：height=h + 平移 center（圆柱 bottom-center 约定）+ 显示。"""
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(path, np.asarray(center, dtype=float))
            self._set_visibility(path, True)

    def _step_color_liquid(self, gripper_pos):
        """液体变色（输入色 → 输出色）：滴酸阶段输入色液柱逐滴长高（顶贴液面、向下扩散，
        酸从管口滴入）；震荡一旦开始（_oscillating）锁 _transition_started，此后每帧
        输入色↓、输出色↑直到 input 归零、output 满——实现"震荡之后液体才慢慢变成输出色"。
        输入柱顶贴液面向下、输出柱从管底向上生长（粉末在管底反应），两者错开不叠影。"""
        if not (self._input_path or self._color_path):
            return
        # 震荡触发 input→output 渐变（一旦触发持续到完成，不因震荡停/放回而中断）
        if self._oscillating and not self._transition_started:
            self._transition_started = True
            print(f"[d3s] shaking started -> liquid color transition "
                  f"{self.input_color} -> {self.liquid_color}")
        if self._transition_started:
            if self._input_path and self._input_frac > 0:
                self._input_frac = max(0.0, self._input_frac - self.COLOR_FADE_RATE)
            if self._color_path and self._color_frac < 1.0:
                self._color_frac = min(1.0, self._color_frac + self.COLOR_FADE_RATE)
        liquid_level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        base = self._tube_pos
        liquid_base = base[2] + self.LIQUID_OFFSET   # 液柱底面 = 白粉顶面（酸从粉顶滴入）
        # 输入色液柱：顶贴液面向下（滴酸逐滴长高 / 震荡缩回）
        if self._input_path:
            h_in = liquid_level * self._input_frac
            if h_in <= 0.0005:
                self._set_visibility(self._input_path, False)
            else:
                self._set_color_column(
                    self._input_path, h_in,
                    (base[0], base[1], liquid_base + liquid_level - h_in / 2))
        # 输出色液柱：从白粉顶面向上（震荡反应逐帧长高）
        if self._color_path:
            h_out = liquid_level * self._color_frac
            if h_out <= 0.0005:
                self._set_visibility(self._color_path, False)
            else:
                self._set_color_column(
                    self._color_path, h_out,
                    (base[0], base[1], liquid_base + h_out / 2))

    # ------------------------------------------------------------------
    # 辅助（d2s/d3l 同款）
    # ------------------------------------------------------------------
    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _set_tube_column(self, path, h, center, r=None):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetHeightAttr().Set(h)
            if r is not None:
                cyl.GetRadiusAttr().Set(r)
        self.object_utils.set_object_position(path, np.asarray(center, dtype=float))

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


def _spatula_rest_matrix():
    """药匙架内竖插位姿（2026-08-26 药匙移到第一列第3排）：与世界 /World/Spatula 矩阵一致
    (translate (0.659,0.3209,0.828)，rotateXYZ(0,0,-180) 烘平后即下行序)。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.659, 0.3209, 0.828, 1.0)


def _dropper_rest_matrix():
    """酸滴管架内竖插静止位姿（与场景 /World/DropperAcid 世界矩阵一致：立放，尖嘴=原点
    DROP_ACID_REST）。平移在最后一行（行向量），否则 AddTransformOp 读出的世界平移是
    (0,0,0) → 滴管被 reset 到原点=桌面下不可见。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       DROP_ACID_REST[0], DROP_ACID_REST[1], DROP_ACID_REST[2], 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
