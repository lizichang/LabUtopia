"""D2-L 液体样品水溶性测试任务：①取样滴管吸样品滴入 → ②洗瓶注水 → ③拿管震荡 + 现象三档。

与 D4-L 同构（持握照 flametest 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**
写为 TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——
滴管全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方）。

洗瓶（②）与药匙同款 6-DOF 持握：attach 时动态锁定 _T_HELD_WASHB = 静止矩阵 · tool_world^-1
（行向量约定：先 _T_HELD_WASHB 再 tool_world 右乘），瓶子随夹爪平移、保持静止朝向
（红嘴朝 +X），attach 瞬间零跳变（D2-S 已验证）。挤水由夹爪开度驱动（opening <
WASH_SQUEEZE_CLOSED → 水流 WaterStream，回升 → 管内显水）。

试管（③）纯平移持握（TUBE_HELD_OFFSET 保竖立），管内水/样品两根液柱 + 浑浊云随管平移
（_render_layers 每帧以试管当前管底为基准渲染）；震荡时按 cfg.mixing 分化三档现象
（miscible 扩散均一 / layered 两层保持 / cloudy 乳白浑浊云盖满后停震褪去）。

生命周期（gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  滴管 rest → attached → squeezed → filled → dropped → released（同 D3-L）
  洗瓶 rest → attached（动态 _T_HELD_WASHB）→ released
  试管 rest → attached（液柱随管）→ released

controller 顺序：SAMPLE_PASS → PICK_WASH_BOTTLE → SQUEEZE_WATER → RETURN_WASH_BOTTLE
→ TUBE_SHAKE_PASS（现象三档在震荡步骤分化）。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
    SAMPLE_BOTTLE_XY, TUBE_XY,
    WASH_GRASP, GRIP_WASHBOT, WASH_SQUEEZE_CLOSED,
    TUBE_GRASP_TCP, GRIP_TUBE,
    SHAKE_TOP_Z, SHAKE_STOP_EPS,
    WATER_START,
    EFFECT_TUBE_DROPS, EFFECT_LAYER_COLUMN, EFFECT_MIXED_LIQUID, EFFECT_CLOUD,
    EFFECT_WATER_STREAM, EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
    SAMPLE_COLOR_NAMES, EFFECT_SAMPLE_LIQUID,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])

# 试管持握（③ 抓起试管震荡）：抓管身中段（管口下 14mm），管底 0.139m 吊在夹爪下方，
# 管内液柱随管平移（_render_layers 以试管当前管底为基准渲染）。TUBE_ORIG = 架内竖插位姿
# （管底=原点 z=0.806）。
TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], 0.806])
TUBE_HELD_OFFSET = np.array([0.0, 0.0, TUBE_ORIG[2] - TUBE_GRASP_TCP[2]])   # ≈(0,0,-0.139)


def _washbottle_rest_matrix():
    """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵，同 D2-S）：rotateXYZ(0,0,-180)
    + translate (0.370,0.525,0.80) 烘平后即下行序。行 0 = (-1,0,0,0) → +X 朝世界 -X；
    行 1 = (0,-1,0,0) → +Y 朝 -Y（红嘴朝 +X）。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.370, 0.525, 0.80, 1.0)


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    样品瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
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
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（flametest v28 同款，
            # 消除闭合瞬间闪现吸附）。只在 near 时 ease，避免合爪未遂拖离原位。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d2l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d2l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d2l] {self.name} filled (aspirated)")
        elif self.state == "filled":
            # 液柱跟随尖嘴
            if self.fill_path:
                self.task._set_fill_follow(self)
            # 试管口区挤胶头滴液
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.tube_xy, gripper_pos)):
                self.state = "dropped"
                self.dropped = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, False)
                self.task._on_drop(self)
                print(f"[d2l] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[d2l] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d2l] {self.name} released to rack -> rest")


class _TubeLifecycle:
    """试管生命周期：rest → attached（跟随 + 管内液柱随管平移）→ released。

    参考点（TCP 世界坐标）：
      grasp    试管口下抓点（管身中段，夹爪 z = 管口下 14mm）
    持握 = TCP + TUBE_HELD_OFFSET(0,0,-0.139)（管底 0.139m 吊在夹爪下方，纯平移
    保竖立）。震荡由 ShakeAction 驱动（task 只逐帧跟随；液柱随管平移由 _render_layers
    每帧以试管当前管底为基准渲染）。
    """

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
            # 夹爪开始合拢且已进近窗：先把试管平滑拉向持握位（同滴管，消闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d2l] tube attached (grip={opening:.4f})")
            return

        # 吸附期：试管逐帧跟随夹爪（纯平移保竖立）；液柱随管平移由 _render_layers 承担
        held = gripper_pos + TUBE_HELD_OFFSET
        self.task._set_obj_world(self.path, held)
        # 回架内松开：写回竖插位姿并复位 rest（液柱随管回原位）
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.grasp, gripper_pos)):
            self.released = True
            self.task._set_obj_world(self.path, self.orig)
            self.state = "rest"
            print(f"[d2l] tube released to rack -> rest")


class D2LWaterSolubilityTask(BaseTask):
    """D2-L 液体样品水溶性测试任务：①吸样品滴入 → ②洗瓶注水 → ③拿管震荡 + 现象三档。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 管内液体逐滴生长（底面贴管底 0.806；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    DROP_LEVEL_STEP = 0.004   # 每滴落定后液面升高 4mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数
    DROP_FALL = 16       # 每滴加速坠落帧数（重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    # 洗瓶持握（②）：夹持阈值 = GRIP_WASHBOT + 4mm 裕量；松开阈值 0.038（< 满开 0.04，
    # 保证开爪能越过它触发 released——0.04=满开永不触发，D2-S 已踩坑修复 2026-08-25）。
    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array(WASH_GRASP)
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004
    WASH_GRIP_OPEN = 0.038
    WASH_SQUEEZE_CLOSED = WASH_SQUEEZE_CLOSED

    # 挤水水流（②）：父 WaterStream + 16 颗水滴球沿抛物线从红嘴坠入试管口（D2-S 同款）
    WATER_STREAM = EFFECT_WATER_STREAM
    WATER_DROPS = 16           # 水滴池大小（与 gen WATER_DROPS 对齐）
    WATER_STAGGER = 2          # 相邻水滴发射间隔帧（错成连续水流）
    WATER_FALL = 12            # 每滴沿抛物线坠落帧数（重力加速视觉）
    WATER_START = np.array(WATER_START)   # 红嘴尖（水平射出起点）
    TUBE_MOUTH = np.array([TUBE_XY[0], TUBE_XY[1], 0.9593])   # 管口中心（抛物线终点）
    WATER_LEVEL = 0.048        # 注水后管内水层高度（洗瓶挤水一次，比样品层多）

    # 试管震荡（③）：抓点 + 纯平移持握（GRIP_TUBE≈Ø19.2/2）
    TUBE = "/World/TestTube"
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)

    # 现象三档（cfg.mixing，③ 震荡分化，2026-08-24 用户确认震荡才分化）
    MIX_FRAMES = 300           # miscible 扩散动画帧数（5s @60Hz；用户 2026-08-25 反馈 120=2s 太快）
    CLOUD_RISE_RATE = 0.02     # cloudy 浑浊云升起速率/帧（震荡 5s 内渐渐盖满）
    CLOUD_FADE_RATE = 0.02     # cloudy 浑浊云褪去速率/帧（停震后渐渐褪去）
    SHAKE_TOP_Z = SHAKE_TOP_Z
    SHAKE_STOP_EPS = SHAKE_STOP_EPS
    SHAKE_STILL_FRAMES = 20    # 连续静止帧数判「震荡停」（同 D3-L）

    DROPPER_SAMPLE = "/World/DropperSample"

    TUBE_DROPS = EFFECT_TUBE_DROPS       # 水色柱（②注水后显示：分层态下层水）
    CLOUD = EFFECT_CLOUD                 # 浑浊云（cloudy 档震荡盖满、停震褪去）
    # LAYER_COLUMN / DROPPER_FILL / DROPPER_DROP 随 sample_color 变体，在 __init__ 里
    # 拼 <色> 后缀设为实例属性（headless 运行时改材质不渲染，须 gen 预烘焙 + visibility 切换）。

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管/洗瓶/试管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会
        # 被物理干扰）
        self._disable_collision(self.DROPPER_SAMPLE)
        self._disable_collision(self.WASH_PATH)
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        # 样品液颜色（2026-08-25 用户：输入决定，同 d3l）。headless 运行时改材质不渲染，
        # gen 预烘焙 <色> 变体，task 拼 <色> 后缀 show 对应变体。默认 blue（非黄色）。
        # 须在 droppers（fill_path=self.DROPPER_FILL）之前设置，否则 AttributeError。
        self.sample_color = str(getattr(cfg, "sample_color", "blue")).strip().lower()
        if self.sample_color not in SAMPLE_COLOR_NAMES:
            self.sample_color = "blue"
        self.SAMPLE_LIQUID = f"{EFFECT_SAMPLE_LIQUID}_{self.sample_color}"
        self.LAYER_COLUMN = f"{EFFECT_LAYER_COLUMN}_{self.sample_color}"
        self.MIXED_LIQUID = f"{EFFECT_MIXED_LIQUID}_{self.sample_color}"
        self.DROPPER_FILL = f"{EFFECT_DROPPER_FILL}_{self.sample_color}"
        self.DROPPER_DROP = f"{EFFECT_DROPPER_DROP}_{self.sample_color}"

        # 取样滴管生命周期句柄（参考点已 pxr 实测）
        self.droppers = {
            "sample": _DropperLifecycle(
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_drop_count = 0      # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []          # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）

        # 洗瓶持握（②）：rest / attached / released；attach 时动态锁定 _T_HELD_WASHB
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self.squeezing = False         # 挤水进行中（持续发射水滴）
        self.water_in_tube = False     # 已挤入水（管内显水层，只触发一次）
        self._water_queue = []         # 在飞水滴队列（prim/t）
        self._water_next_prim = 0      # 下一颗水滴用哪个 Drop_i（round-robin 复用池）
        self._water_spawn = 0          # 距下次发射水滴的倒计时帧

        # 试管震荡（③）：纯平移持握，液柱随管平移
        self.tube = _TubeLifecycle(self, self.TUBE, TUBE_ORIG, self.TUBE_GRASP)

        # 现象三档状态：两层液柱高度 + 混合/浑浊进度 + 震荡停检测
        self.mixing = str(getattr(cfg, "mixing", "miscible")).strip().lower()
        if self.mixing not in ("miscible", "layered", "cloudy"):
            self.mixing = "miscible"
        self._sample_level = 0.0       # 样品层高（滴样涨，上限 DROP_LEVEL_MAX）
        self._water_level = 0.0        # 水层高（注水后 = WATER_LEVEL）
        self._mix_frac = 0.0           # miscible 扩散进度 0..1
        self._cloud_frac = 0.0         # cloudy 浑浊云覆盖 0..1
        self._mix_prev = None          # 摇晃判定用上一帧夹爪位置（自维护）
        self._prev_gripper_pos = None  # 震荡停检测用上一帧夹爪位置
        self._shake_stop_frames = 0    # 连续"几乎不动"帧数

    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_drop_count = 0
        self._drop_queue = []
        for d in self.droppers.values():
            d.reset()
        self.tube.reset()
        # 洗瓶复位：回静止位姿 + 水流/管内水状态清零
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self._set_washbottle_world(_washbottle_rest_matrix())
        self.squeezing = False
        self.water_in_tube = False
        self._water_queue = []
        self._water_next_prim = 0
        self._water_spawn = 0
        # 现象三档复位：层高/进度清零、液柱回架内位姿
        self._sample_level = 0.0
        self._water_level = 0.0
        self._mix_frac = 0.0
        self._cloud_frac = 0.0
        self._mix_prev = None
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        # 样品色变体：只显示选中色的 SampleLiquid（瓶内液体恒显），其余样品色 prim 全隐藏
        # （headless 运行时改材质不渲染，须 gen 预烘焙 <色> 变体 + task 切换 visibility）。
        for name in SAMPLE_COLOR_NAMES:
            for base in (EFFECT_LAYER_COLUMN, EFFECT_MIXED_LIQUID,
                         EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP):
                self._set_visibility(f"{base}_{name}", False)
            self._set_visibility(f"{EFFECT_SAMPLE_LIQUID}_{name}", name == self.sample_color)
            # 分层柱/混液变体高度复位（防换色残留；选中/未选中都复位）
            for base in (EFFECT_LAYER_COLUMN, EFFECT_MIXED_LIQUID):
                prim = self.stage.GetPrimAtPath(f"{base}_{name}")
                if prim.IsValid():
                    UsdGeom.Cylinder(prim).GetHeightAttr().Set(0.0)
        # 效果 prim 全部隐藏 + 高度/位置复位
        for p in (self.TUBE_DROPS, self.LAYER_COLUMN, self.MIXED_LIQUID, self.CLOUD,
                  self.WATER_STREAM, self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visibility(p, False)
        for i in range(self.WATER_DROPS):
            self._set_visibility(f"{self.WATER_STREAM}/Drop_{i}", False)
        for p in (self.TUBE_DROPS, self.LAYER_COLUMN, self.MIXED_LIQUID, self.CLOUD):
            prim = self.stage.GetPrimAtPath(p)
            if prim.IsValid():
                UsdGeom.Cylinder(prim).GetHeightAttr().Set(0.0)
        for p in (self.LAYER_COLUMN, self.MIXED_LIQUID):
            self.object_utils.set_object_position(
                p, (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z))
        self.object_utils.set_object_position(
            self.CLOUD, (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z))

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._step_drop_anim()              # 滴落动画独立推进（与抓取/滴加并行）
        self._update_washbottle()           # ② 洗瓶持握（rest/attached/released）
        self._step_water_anim()             # ② 挤水水流动画（挤水发射水滴、松爪收尾）
        for d in self.droppers.values():
            d.step(gripper_pos, opening)
        self.tube.step(gripper_pos, opening)   # ③ 试管跟随/震荡
        self._step_shake_stop(gripper_pos)     # 震荡停检测（cloudy 褪去用）
        self._step_mixing(gripper_pos)         # ③ 现象三档（miscible/layered/cloudy）
        self._render_layers()                  # 每帧按当前层高渲染（液柱随管平移）
        return self.get_basic_state_info(additional_info={
            "sample_attached": self.droppers["sample"].attached,
            "sample_filled": self.droppers["sample"].filled,
            "sample_dropped": self.droppers["sample"].dropped,
            "sample_released": self.droppers["sample"].released,
            "washbottle_state": self.washbottle_state,
            "water_in_tube": self.water_in_tube,
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[d2l] episode done success={success} "
              f"sample_dropped={self.droppers['sample'].dropped} "
              f"sample_released={self.droppers['sample'].released} "
              f"washbottle={self.washbottle_state} water_in_tube={self.water_in_tube} "
              f"tube_released={self.tube.released}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 滴管位姿
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        """物体原点世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保竖立姿态）。

        flametest 同款：不 ClearXformOpOrder、不写 4x4 矩阵——烘平场景里滴管/试管只有
        xformOp:translate 一个 op，set_object_position 改首 op 即平移，姿态不变。
        """
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            self.object_utils.set_object_position(path, np.asarray(position, dtype=float))

    def _ease_obj_world(self, path, target, k=0.18):
        """把物体逐帧向 target 平滑移动（flametest v28：抓取时消除闪现吸附）。"""
        cur = self._get_obj_world(path)
        if cur is None:
            return
        nxt = cur + (target - cur) * k
        self._set_obj_world(path, nxt)

    def _set_fill_follow(self, dropper):
        """DropperFill 样品液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴，+Z 收窄→加宽
        贴合玻璃体）。尖嘴在夹爪下 0.13m（保竖立），液柱从尖嘴向上 60mm（几何见 gen 脚本）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(self.DROPPER_FILL, tip)

    # ------------------------------------------------------------------
    # 洗瓶持握（② 与 D2-S 药匙同款 6-DOF：动态锁定 _T_HELD_WASHB）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 D2-S 药匙，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        """把洗瓶写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.WASH_PATH)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _update_washbottle(self):
        """每帧洗瓶持握：rest → 近抓点+合拢 → attached（动态 _T_HELD_WASHB 跟随）→
        released（回表位）。挤水（opening < WASH_SQUEEZE_CLOSED → 水流，回升 → 管内显水）。"""
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]

        if self.washbottle_state == "rest":
            if self._near(self.WASH_GRASP, gripper_pos):
                self._wb_near_frames += 1
            else:
                self._wb_near_frames = 0
            if (self._wb_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.WASH_GRIP_CLOSED):
                self.washbottle_state = "attached"
                # 动态持握变换：抓取时刻瓶子正好在静止位 → 锁 (静止 · tool^-1)，attach 零跳变
                self._T_HELD_WASHB = _washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[d2l] washbottle attached (grip={opening:.4f})")

        elif self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            # 挤水：夹爪从持握 0.030 合到 0.020 挤压瓶身 → 水流；松回 → 水流结束、管内显水
            if not self.water_in_tube:
                if not self.squeezing and opening < self.WASH_SQUEEZE_CLOSED:
                    self.squeezing = True
                    self._water_spawn = self.WATER_STAGGER   # 下一帧立即发射首滴
                    self._set_visibility(self.WATER_STREAM, True)
                    print(f"[d2l] washbottle squeezing (grip={opening:.4f}) water stream")
                elif self.squeezing and opening >= self.WASH_SQUEEZE_CLOSED:
                    # 松爪：停止发射，让在飞水滴由 _step_water_anim 收尾坠落，再显管内水层
                    self.squeezing = False
                    self.water_in_tube = True
                    self._water_level = self.WATER_LEVEL
                    print("[d2l] water in tube")
            if opening > self.WASH_GRIP_OPEN:   # 完全开爪才算松开（见类常量注释）
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(_washbottle_rest_matrix())
                print("[d2l] washbottle released to table")

    def _step_water_anim(self):
        """挤水水流：挤水期间每 WATER_STAGGER 帧发射一颗水滴，沿抛物线（x/y 线性、z t²
        重力加速）从红嘴尖坠入试管口中心；松爪后停止发射、让在飞水滴落完再隐藏父节点。

        抛物线：x = x0+(x1-x0)·t，z = z0-(z0-z1)·t²（t∈[0,1]），起点 WATER_START=红嘴尖、
        终点 TUBE_MOUTH=管口中心。水滴池 WATER_DROPS 颗 round-robin 复用。"""
        if self.squeezing:
            self._water_spawn += 1
            if self._water_spawn >= self.WATER_STAGGER:
                self._water_spawn = 0
                idx = self._water_next_prim % self.WATER_DROPS
                self._water_next_prim += 1
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{idx}", True)
                self.object_utils.set_object_position(
                    f"{self.WATER_STREAM}/Drop_{idx}", self.WATER_START.copy())
                self._water_queue.append({"prim": idx, "t": 0})
        if not self._water_queue:
            return
        remaining = []
        for d in self._water_queue:
            d["t"] += 1
            if d["t"] >= self.WATER_FALL:
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{d['prim']}", False)
                continue
            frac = d["t"] / self.WATER_FALL
            x = self.WATER_START[0] + (self.TUBE_MOUTH[0] - self.WATER_START[0]) * frac
            y = self.WATER_START[1] + (self.TUBE_MOUTH[1] - self.WATER_START[1]) * frac
            z = self.WATER_START[2] - (self.WATER_START[2] - self.TUBE_MOUTH[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{self.WATER_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._water_queue = remaining
        if not remaining and not self.squeezing:
            self._set_visibility(self.WATER_STREAM, False)

    # ------------------------------------------------------------------
    # 现象三档（③ 震荡分化）
    # ------------------------------------------------------------------
    def _step_shake_stop(self, gripper_pos):
        """检测震荡是否已停（连续 SHAKE_STILL_FRAMES 帧水平位移 < SHAKE_STOP_EPS），供
        cloudy 褪去判定。试管未抓起或夹爪不在震荡高度 → 复位计数。"""
        if not self.tube.attached or gripper_pos[2] < self.SHAKE_TOP_Z:
            self._prev_gripper_pos = None
            self._shake_stop_frames = 0
            return
        move = 0.0
        if self._prev_gripper_pos is not None:
            move = float(np.linalg.norm(
                np.asarray(gripper_pos[:2], dtype=float) - self._prev_gripper_pos[:2]))
        self._prev_gripper_pos = np.asarray(gripper_pos, dtype=float)
        if move < self.SHAKE_STOP_EPS:
            self._shake_stop_frames += 1
        else:
            self._shake_stop_frames = 0

    def _step_mixing(self, gripper_pos):
        """现象三档（cfg.mixing）：震荡才分化。
          miscible → 样品柱长满整管（扩散均一，_mix_frac→1）
          layered  → 两层保持（样品层在水层之上，不变化）
          cloudy   → 乳白浑浊云盖满液柱（摇晃中升起），停震后渐渐褪去
        摇晃判定（同 D3-L）：试管已抓起 + 夹爪在震荡高度 + 水平位移 > eps 且未判停
        （拎起是垂直运动 dx≈0、震荡是水平运动，浑浊云不会在提起来瞬间冒出）。"""
        if not self.water_in_tube or self.mixing == "layered":
            return
        lifted = self.tube.attached and gripper_pos[2] >= self.SHAKE_TOP_Z
        oscillating = False
        if lifted:
            if self._mix_prev is not None:
                horiz = float(np.linalg.norm(
                    np.asarray(gripper_pos[:2], dtype=float) - self._mix_prev[:2]))
                oscillating = (horiz > self.SHAKE_STOP_EPS
                               and self._shake_stop_frames < self.SHAKE_STILL_FRAMES)
            self._mix_prev = np.asarray(gripper_pos, dtype=float)
        else:
            self._mix_prev = None
        if self.mixing == "miscible":
            if oscillating and self._mix_frac < 1.0:
                self._mix_frac = min(1.0, self._mix_frac + 1.0 / self.MIX_FRAMES)
        elif self.mixing == "cloudy":
            target = 1.0 if oscillating else 0.0
            rate = self.CLOUD_RISE_RATE if oscillating else self.CLOUD_FADE_RATE
            self._cloud_frac += (target - self._cloud_frac) * rate

    def _set_column(self, path, h, center):
        """写一根圆柱高度 + 位置（bottom-center 约定：center = 底面 + h/2）。"""
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(path, np.asarray(center, dtype=float))

    def _render_layers(self):
        """按当前水/样品层高与混合进度渲染液柱（水层 TubeDrops、纯样品层 LayerColumn、
        稀释混液 MixedLiquid）+ 浑浊云（Cloud）。两层模型（bottom-center，底面贴管底）。

        miscible 物理模型（2026-08-25 用户：混液颜色应**渐渐出现**，勿终点跳变）：样品（顶）
        向下溶入水（底），稀释混液从水-样界面双向扩展，三区高度随 _mix_frac 线性变化——
          水层   TubeDrops   高 hw·(1-f)，底贴管底（被混液从顶向下吃掉）
          混液   MixedLiquid 高 T·f，底 = 管底 + hw·(1-f)（从界面双向长，稀释色渐现）
          样品层 LayerColumn 高 hs·(1-f)，底 = 管底 + hw + hs·f（被混液从底向上吃掉）
        layered：两层保持；cloudy：两层保持 + Cloud 乳白柱盖液柱（_cloud_frac）。
        定位以试管**当前**管底为基准（随管平移，不写死 TUBE_XY）。"""
        hw = self._water_level
        hs = self._sample_level
        tube_now = self._get_obj_world(self.TUBE)
        base = np.asarray(tube_now, dtype=float) if tube_now is not None else TUBE_ORIG
        bx, by, bz = base
        if self.mixing == "miscible":
            # 三层随 _mix_frac 线性收缩/扩展（T=整液柱高，f=扩散进度 0→1）
            f = self._mix_frac
            T = hs + hw
            # 水层（底）：hw·(1-f)
            h_w = hw * (1.0 - f)
            if h_w > 1e-4:
                self._set_column(self.TUBE_DROPS, h_w, (bx, by, bz + h_w / 2))
                self._set_visibility(self.TUBE_DROPS, True)
            else:
                self._set_visibility(self.TUBE_DROPS, False)
            # 稀释混液（界面双向扩展）：高 T·f，底 = 管底 + hw·(1-f)
            h_m = T * f
            if h_m > 1e-4:
                self._set_column(self.MIXED_LIQUID, h_m, (bx, by, bz + hw * (1.0 - f) + h_m / 2))
                self._set_visibility(self.MIXED_LIQUID, True)
            else:
                self._set_visibility(self.MIXED_LIQUID, False)
            # 纯样品层（顶）：高 hs·(1-f)，底 = 管底 + hw + hs·f
            h_s = hs * (1.0 - f)
            if h_s > 1e-4:
                self._set_column(self.LAYER_COLUMN, h_s, (bx, by, bz + hw + hs * f + h_s / 2))
                self._set_visibility(self.LAYER_COLUMN, True)
            else:
                self._set_visibility(self.LAYER_COLUMN, False)
        else:
            # 水层 TubeDrops（底层）
            if hw > 0:
                self._set_column(self.TUBE_DROPS, hw, (bx, by, bz + hw / 2))
                self._set_visibility(self.TUBE_DROPS, True)
            else:
                self._set_visibility(self.TUBE_DROPS, False)
            # 纯样品层 LayerColumn（顶层）
            if hs > 0:
                self._set_column(self.LAYER_COLUMN, hs, (bx, by, bz + hw + hs / 2))
                self._set_visibility(self.LAYER_COLUMN, True)
            else:
                self._set_visibility(self.LAYER_COLUMN, False)
            self._set_visibility(self.MIXED_LIQUID, False)
        # 浑浊云（cloudy）：白柱盖液柱，_cloud_frac 驱动高度
        if self.mixing == "cloudy":
            cloud_h = (hw + hs) * self._cloud_frac
            if cloud_h > 0.0005:
                self._set_column(self.CLOUD, cloud_h, (bx, by, bz + cloud_h / 2))
                self._set_visibility(self.CLOUD, True)
            else:
                self._set_visibility(self.CLOUD, False)
        else:
            self._set_visibility(self.CLOUD, False)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """任一滴加：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴（尖嘴下逐滴错落坠落）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（管口上方 25mm，液滴可见坠落）
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._tube_drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            target = np.array([TUBE_XY[0], TUBE_XY[1],
                               self.TUBE_BOTTOM_Z + level - 0.003])  # 落定在管内液面
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(), "target": target,
                "level": level, "name": dropper.name,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._tube_drop_count += self.DROPS_PER_SQUEEZE
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[d2l] squeeze ({dropper.name}) -> {self.DROPS_PER_SQUEEZE} drops spawned")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该球+涨样品层）。"""
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
                pos = d["start"]                         # 悬停：看得见滴挂在尖嘴
            elif d["t"] <= d["hang"] + d["fall"]:
                frac = (d["t"] - d["hang"]) / d["fall"]  # 重力加速（t² 缓入）
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                # 落定：隐藏这颗、涨样品层，移出队列
                self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_tube_level(d["level"], d["name"])
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    def _grow_tube_level(self, h, name):
        """液滴落定：涨对应层高。sample → 样品层（LayerColumn），water → 水层（TubeDrops）。
        实际渲染统一走 _render_layers（两层模型，随管平移）。"""
        if name == "sample":
            self._sample_level = max(self._sample_level, h)
        else:
            self._water_level = max(self._water_level, h)
        self._render_layers()
        print(f"[d2l] tube {name} level h={h:.3f}")

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
