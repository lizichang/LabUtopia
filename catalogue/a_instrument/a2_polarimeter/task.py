# -*- coding: utf-8 -*-
"""A2 旋光仪测量任务：试管预装粉 → 洗瓶加水 → 震荡溶解 → 胶头滴管吸 3 次挤进旋光管
→ 放导轨 → 按启动键读数。

2026-08-27 滴管转移改造（用户：倒液时机械臂乱动转圈，改用胶头滴管移动液体，不倒液体）：
  - 删倒液周期（PickTestTube 再拿试管 + 倒置 PourStream 倒液），试管只被拿一次（震荡溶解）；
  - 新增 DropperTransferPass：胶头滴管从试管吸液 drop_cycles=3 次，每次水平 −x 移 10cm
    挤进旋光管加液口。task 检测滴管下探到浸液点 → 试管液变矮（吸出）、下探到加液口上 →
    成串液滴坠入 + 旋光管内液柱（TubeLiquid）生长；满 3 次试管液吸空隐藏。

三类物体三种持握（同 d2s/d3s 约定）：
  试管（/World/TestTube）    旋转跟随 _T_HELD_TUBE（管底吊夹爪下 0.1393m），一个吸附周期
                            （TubeShakePass 震荡溶解；释放回架时粉溶尽隐藏，溶液留管内待吸）
  洗瓶（/World/WashBottle）  动态锁 _T_HELD_WASHB = 静止矩阵 · tool^-1，横夹肚子随夹爪平移；
                            开合 < 0.025 判挤水（WaterStream 水流进试管）、> 0.038 松开回表位
  滴管（/World/Dropper）     _T_HELD_DROPPER 沿 tool+X 伸 0.13（同 d3s 酸滴管，尖嘴吊夹爪下；
                            持握后滴管保持竖直），吸试管内液 3 次、每次挤进旋光管加液口
  旋光管（/World/PolarimeterTube） 纯平移持握（set_object_position，保横放管轴沿 x），
                            释放到导轨 rest (0.51,-0.24,1.0075)；TubeLiquid 是它的子 prim
                            随管移动（滴入时在桌面显液柱，随后跟着管一起上导轨）。

测量键（/World/Polarimeter/start_button）：idle → measuring（按下，ScreenMeasuring 红进度条
  ~4s）→ result（ScreenGlow_<rotation> 读数定格）→ releasing（爪子抬离后缓慢弹回）→ released。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_TUBE, TUBE_XY, TUBE_ORIG_Z, TUBE_GRASP_TCP,
    GRIP_WASHBOT, WASH_GRASP, WASH_GRIP_OPEN, WASH_SQUEEZE_CLOSED,
    GRIP_PTUBE, PTUBE_GRASP, PTUBE_REST, PTUBE_PLACE_CENTER, PTUBE_HELD_OFFSET_Z,
    FILL_TOP_Z, WATER_START, WATER_END,
    GRIP_DROPPER, DROP_XY, DROP_GRASP, DROP_REST,
    DROPPER_DIP_TCP, DROPPER_DROP_TCP, TIP_OFFSET,
    TUBE_LIQUID_H, DROP_CYCLES,
    START_BUTTON_PRESS_TCP, BUTTON_PATH, BUTTON_REST_Z, BUTTON_PRESSED_Z,
    BUTTON_LIFT_Z, BUTTON_SPRING_STEP,
    LID_PATH, LID_OPEN_DEG, LID_PUSH_X, LID_PUSH_Y0, LID_PUSH_Y1,
    EFFECT_TUBE_POWDER, EFFECT_TUBE_WATER, EFFECT_TUBE_LIQUID,
    EFFECT_WATER_STREAM, EFFECT_DROPPER_DRIP,
    EFFECT_SCREEN_MEASURING_TPL, EFFECT_SCREEN_RESULT_TPL, PROGRESS_STEPS,
    ROTATION_DEFAULT, ROTATION_KEY,
)

# 试管相对夹爪的持握矩阵（行向量约定，平移在最后一行）：
#   tube+X → tool+Z、tube+Y → tool+Y、tube+Z → -tool+X，管底原点在 +0.1393 tool+X。
# ORIENT_FWD（tool+Z=+X、tool+X=-Z）下：管底 = gripper-0.1393·Z（吊夹爪下方）、管口朝上。
_T_HELD_TUBE = Gf.Matrix4d(0.0, 0.0, 1.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           -1.0, 0.0, 0.0, 0.0,
                           0.1393, 0.0, 0.0, 1.0)

# 滴管相对夹爪的持握矩阵（与 _T_HELD_TUBE 同旋转：滴管+Z → tool-X；ORIENT_FWD 下 tool-X=世界+Z
# → 滴管保持竖直），原点（尖嘴）在 +0.13 tool+X：尖嘴 = 夹爪 + 0.13·tool+X（世界 -Z，吊夹爪下方）。
# 抓取时夹爪在尖嘴上方 0.13（= 胶头位），持握后尖嘴继续吊夹爪下 0.13（d3s 酸滴管同款）。
_T_HELD_DROPPER = Gf.Matrix4d(0.0, 0.0, 1.0, 0.0,
                              0.0, 1.0, 0.0, 0.0,
                              -1.0, 0.0, 0.0, 0.0,
                              0.13, 0.0, 0.0, 1.0)


class A2PolarimeterTask(BaseTask):
    """A2 旋光仪测量任务：洗瓶注水 + 试管震荡 + 滴管吸液挤入 + 旋光管平移 + 屏幕读数。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # ---- 试管（单吸附周期：震荡溶解，粉溶尽隐藏）----
    TUBE = "/World/TestTube"
    TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z])    # 管底架内竖插位
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)                          # 抓点（管口下 14mm）
    TUBE_GRIP_CLOSED = GRIP_TUBE + 0.004                           # 夹紧阈值 0.0136
    # 管内效果 rest 位（gen_a2_scene.py 已预制，TUBE_BOT+0.034 / +0.049）
    POWDER_REST = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z + 0.034])
    TUBE_WATER_REST = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z + 0.049])
    TUBE_WATER_H = TUBE_LIQUID_H                                  # 试管液初始高 0.035

    # ---- 洗瓶（d2s 同款横夹肚子）----
    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array(WASH_GRASP)
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004                        # 夹紧阈值 0.034
    WASH_GRIP_OPEN = WASH_GRIP_OPEN                                # 松开阈值 0.038（<满开 0.04）

    # ---- 滴管（吸试管内液 3 次挤进加液口；一次持握，task 跟随 + 现象动画）----
    DROP_PATH = "/World/Dropper"
    DROP_GRASP = np.array(DROP_GRASP)                              # 抓点（尖嘴底 0.806 + 0.13 = 0.936）
    DROP_REST = np.array(DROP_REST)                                # 架内竖插静止位（尖嘴底 0.806）
    DROP_GRIP_CLOSED = GRIP_DROPPER + 0.004                        # 夹紧阈值 0.0095
    drop_cycles = DROP_CYCLES                                      # 吸液循环数（__init__ 读 cfg 覆盖）
    # 滴液动画：尖嘴（加液口上 25mm）→ 加液口顶，成串 4 滴错帧坠入（读 DropperDrip 16 球池）
    DRIP_START = np.array(DROPPER_DROP_TCP) - np.array([0.0, 0.0, TIP_OFFSET])  # 尖嘴 (0.559,0.241,0.855)
    DRIP_END = np.array([DRIP_START[0], DRIP_START[1], FILL_TOP_Z])            # 加液口顶 0.830
    DRIP_POOL = 16
    DRIP_BURST = 4
    DRIP_STAGGER = 2
    DRIP_FALL = 10

    # ---- 旋光管（纯平移持握）----
    PTUBE_PATH = "/World/PolarimeterTube"
    PTUBE_REST = np.array(PTUBE_REST)                              # 桌面横放 (0.5265,0.241,0.811)
    PTUBE_RAILS = np.array(PTUBE_PLACE_CENTER)                     # 导轨落座 (0.51,-0.24,1.0075)
    PTUBE_GRASP = np.array(PTUBE_GRASP)
    PTUBE_GRIP_CLOSED = GRIP_PTUBE + 0.004                         # 夹紧阈值 0.0105
    PTUBE_HELD_OFFSET = np.array([0.0, 0.0, PTUBE_HELD_OFFSET_Z])  # 管中心=TCP-0.019
    PTUBE_DROP_FRAMES = 20         # 松爪后管「落下」导轨的帧数（0.33s @60fps）
    # 旋光管内液柱（TubeLiquid 子 prim：管局部系轴 Y 圆柱，原长 0.10、局部 y=0 → 底 -0.05）
    TUBE_LIQ_LEN = 0.10
    TUBE_LIQ_BOT = -0.05

    # ---- 挤水水流（洗瓶红嘴 → 试管口，抛物线坠入）----
    WATER_START = np.array(WATER_START)                            # 嘴尖 0.994
    WATER_END = np.array(WATER_END)                                # 管口 0.9593
    WATER_DROPS = 16
    WATER_STAGGER = 2
    WATER_FALL = 12

    # ---- 屏幕读数（进度条 ~4s 走完 → 结果屏定格）----
    MEASURE_FRAMES = 240

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 静态碰撞体持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰，d2s 同款）
        self._disable_collision(self.TUBE)
        self._disable_collision(self.WASH_PATH)
        self._disable_collision(self.DROP_PATH)
        self._disable_collision(self.PTUBE_PATH)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.squeeze_close_threshold = getattr(cfg, "squeeze_close_threshold", 0.025)
        self.drop_cycles = max(1, int(getattr(cfg, "drop_cycles", DROP_CYCLES)))

        # 旋转角度读数（experiment_result 框架 --result rotation_angle=<> / TTY 交互写回
        # cfg.rotation_angle，default "+12.5"）→ 选对应预烘焙读数屏 ScreenGlow_<key>
        # （headless 下运行时改材质不渲染 → 按档位预烘焙，a1 同款）
        self.rotation_angle = str(getattr(cfg, "rotation_angle", ROTATION_DEFAULT)).strip()
        self._result_screen = EFFECT_SCREEN_RESULT_TPL.format(key=ROTATION_KEY(self.rotation_angle))

        # ---- 试管状态（单吸附周期：震荡溶解）----
        self.tube_state = "rest"      # rest / attached / released(→回 rest 可再抓)
        self._tube_near_frames = 0

        # ---- 洗瓶状态 ----
        self.washbottle_state = "rest"  # rest / attached / released
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self.squeezing = False        # 挤水进行中（持续发射水滴）
        self.water_in_tube = False    # 已挤入水（管内水显示，只触发一次）
        self._water_queue = []        # 在飞水滴（prim/t）
        self._water_next_prim = 0
        self._water_spawn = 0

        # ---- 滴管状态（吸液挤液 transfer）----
        self.dropper_state = "rest"   # rest / attached / released
        self._drop_near_frames = 0
        self.transfer_count = 0       # 已滴入旋光管份数（0..drop_cycles）
        self.aspirate_count = 0       # 已吸试管液份数（0..drop_cycles）
        self._at_dip = False          # 吸液下探边沿（防同窗重复计数）
        self._at_drop = False         # 滴液下探边沿
        self._drip_queue = []         # 在飞滴液滴（prim/t）
        self._drip_next = 0

        # ---- 旋光管状态 ----
        self.ptube_state = "rest"     # rest / attached / dropping / released
        self._ptube_near_frames = 0
        self._ptube_drop_from = None  # 松爪瞬间管中心（下落起点）
        self._ptube_drop_t = 0        # 下落已进行帧数

        # ---- 测量键状态（a1 同款）----
        self.button_state = "idle"    # idle / measuring / result / releasing / released
        self.button_pressed = False
        self.reading = False
        self._measure_frames = 0
        self._step_shown = -1

        # ---- 翻盖状态（⑩ CloseLidPass 拨回闭合）----
        self.lid_state = "open"       # open / closed
        self._lid_push_seen = False   # 夹爪是否进入过推盖带（近侧 y∈带 且向 −y，防放导轨等误提前闭合）
        self._lid_angle = LID_OPEN_DEG  # 当前 lid 转轴角（单调递减：只合拢不回弹）
        self._prev_lid_y = None       # 上一帧夹爪 y（检测向 −y 移动 = 正在推盖）

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        # 试管回架 + 粉/水回 rest（粉可见：预装白粉初始状态）
        self.tube_state = "rest"
        self._tube_near_frames = 0
        self._set_tube_world(_tube_rest_matrix())
        self._set_visibility(EFFECT_TUBE_POWDER, True)
        self._set_visibility(EFFECT_TUBE_WATER, False)
        self._reset_tube_water()
        self.object_utils.set_object_position(EFFECT_TUBE_POWDER, self.POWDER_REST)
        # 洗瓶回表位
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self.squeezing = False
        self.water_in_tube = False
        self._water_queue = []
        self._water_next_prim = 0
        self._water_spawn = 0
        self._set_washbottle_world(_washbottle_rest_matrix())
        self._set_visibility(EFFECT_WATER_STREAM, False)
        for i in range(self.WATER_DROPS):
            self._set_visibility(f"{EFFECT_WATER_STREAM}/Drop_{i}", False)
        # 滴管回架 + 转移计数清零 + 滴液动画复位
        self.dropper_state = "rest"
        self._drop_near_frames = 0
        self.transfer_count = 0
        self.aspirate_count = 0
        self._at_dip = False
        self._at_drop = False
        self._drip_queue = []
        self._drip_next = 0
        self._set_dropper_world(_dropper_rest_matrix())
        self._set_visibility(EFFECT_DROPPER_DRIP, False)
        for i in range(self.DRIP_POOL):
            self._set_visibility(f"{EFFECT_DROPPER_DRIP}/Drop_{i}", False)
        # 旋光管回桌面 + 管内液柱复位（隐藏、长 0.10、局部 y=0）
        self.ptube_state = "rest"
        self._ptube_near_frames = 0
        self._ptube_drop_from = None
        self._ptube_drop_t = 0
        self._set_ptube_pos(self.PTUBE_REST)
        self._reset_tube_liquid()
        # 测量键复位
        self.button_state = "idle"
        self.button_pressed = False
        self.reading = False
        self._measure_frames = 0
        self._step_shown = -1
        for i in range(PROGRESS_STEPS):
            self._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=i), False)
        self._set_visibility(self._result_screen, False)
        self._set_button_z(BUTTON_REST_Z)
        # 翻盖回掀开位（新一集从头拨盖）
        self.lid_state = "open"
        self._lid_push_seen = False
        self._lid_angle = LID_OPEN_DEG
        self._prev_lid_y = None
        self._set_lid_angle(LID_OPEN_DEG)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_washbottle(gripper_pos, opening)   # 洗瓶持握 + 挤水判定
        self._update_tube(gripper_pos, opening)         # 试管持握（震荡）
        self._update_dropper(gripper_pos, opening)      # 滴管持握 + 吸液/滴液判定
        self._update_ptube(gripper_pos, opening)        # 旋光管持握
        self._update_button(gripper_pos)                # 测量键生命周期
        self._update_lid(gripper_pos)                   # 拨回翻盖（⑩ 联动 lid 闭合）
        self._step_water_anim()                         # 挤水水流
        self._step_drip_anim()                          # 滴管滴液
        return self.get_basic_state_info(additional_info={
            "washbottle_state": self.washbottle_state,
            "tube_state": self.tube_state,
            "water_in_tube": self.water_in_tube,
            "dropper_state": self.dropper_state,
            "transfer_count": self.transfer_count,
            "aspirate_count": self.aspirate_count,
            "ptube_state": self.ptube_state,
            "button_state": self.button_state,
            "button_pressed": self.button_pressed,
            "reading": self.reading,
            "lid_state": self.lid_state,
        })

    def on_task_complete(self, success):
        print(f"[a2] episode done success={success} "
              f"washbottle={self.washbottle_state} tube={self.tube_state} "
              f"dropper={self.dropper_state} transfer={self.transfer_count}/{self.drop_cycles} "
              f"ptube={self.ptube_state} "
              f"button_pressed={self.button_pressed} reading={self.reading}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧洗瓶持握（S3 夹肚子）：rest → 近抓点+合拢 → attached（动态锁随夹爪）→
    #   挤压（开合<0.025 → WaterStream 水流）→ 松开（>0.038）→ released（回表位）
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
                # 动态持握变换：抓取时刻瓶子正好在静止位 → 锁 (静止 · tool^-1)，
                # 瓶子保持静止朝向、随夹爪平移，attach 瞬间零跳变（d2s 同款）。
                self._T_HELD_WASHB = _washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[a2] washbottle attached (grip={opening:.4f})")
            return

        if self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            # 挤水：夹爪从持握 0.030 进一步合到 0.020 挤压瓶身 → 水流显示；
            # 松回 0.030 → 水流结束、管内显水（只触发一次）。
            if not self.water_in_tube:
                if not self.squeezing and opening < self.squeeze_close_threshold:
                    self.squeezing = True
                    self._water_spawn = self.WATER_STAGGER
                    self._set_visibility(EFFECT_WATER_STREAM, True)
                    print(f"[a2] washbottle squeezing (grip={opening:.4f}) water stream")
                elif self.squeezing and opening >= self.squeeze_close_threshold:
                    self.squeezing = False
                    self.water_in_tube = True
                    self._set_visibility(EFFECT_TUBE_WATER, True)
                    print("[a2] water in tube")
            if opening > self.WASH_GRIP_OPEN:
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(_washbottle_rest_matrix())
                print("[a2] washbottle released to table")

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 _set_spatula_from_gripper，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        """把洗瓶写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.WASH_PATH)
        if not prim.IsValid():
            return
        parent_xf = UsdGeom.Xformable(self.stage.GetPrimAtPath("/World")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    # ------------------------------------------------------------------
    # 每帧试管持握（单吸附周期）：rest → 近抓点+合拢 → attached（旋转跟随 _T_HELD_TUBE）
    #   → released（回架 rest）。释放隐藏管内粉（震荡溶解进溶液），溶液留管内待滴管吸。
    # ------------------------------------------------------------------
    def _update_tube(self, gripper_pos, opening):
        if self.tube_state == "rest":
            near = self._near_grasp(gripper_pos, self.TUBE_GRASP)
            self._tube_near_frames = self._tube_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_tube_to_gripper(gripper_pos)
            if (near and self._tube_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.TUBE_GRIP_CLOSED):
                self.tube_state = "attached"
                self._set_tube_world(_T_HELD_TUBE * self._tool_world())
                print(f"[a2] tube attached (grip={opening:.4f})")
            return

        if self.tube_state == "attached":
            self._set_tube_world(_T_HELD_TUBE * self._tool_world())
            self._follow_tube_effects()
            if opening > self.gripper_open_threshold:
                # 释放回架：震荡后粉溶进溶液（隐藏白粉），溶液留在管内待滴管吸
                self._set_visibility(EFFECT_TUBE_POWDER, False)
                print("[a2] tube released to rack (powder dissolved into solution)")
                self.tube_state = "rest"
                self._set_tube_world(_tube_rest_matrix())
                self._follow_tube_effects()

    def _ease_tube_to_gripper(self, gripper_pos, k=0.18):
        """夹爪合拢期间试管逐帧平滑拉向持握位（消除闪现吸附，旋转一致 → 纯平移插值足够）。"""
        target = _T_HELD_TUBE * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.TUBE)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_tube_world(_blend_world(cur, target, k))

    def _tube_world_matrix(self):
        """读试管当前世界矩阵（管内效果跟随偏移用）。"""
        prim = self.stage.GetPrimAtPath(self.TUBE)
        if not prim.IsValid():
            return Gf.Matrix4d()
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _tube_origin(self):
        """试管当前世界原点（管底，旋转跟随下 = 管底中心）。"""
        return np.array(self._tube_world_matrix().ExtractTranslation(), dtype=float)

    def _set_tube_world(self, world_matrix):
        """把试管写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op，同药匙）。"""
        prim = self.stage.GetPrimAtPath(self.TUBE)
        if not prim.IsValid():
            return
        parent_xf = UsdGeom.Xformable(self.stage.GetPrimAtPath("/World")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _follow_tube_effects(self):
        """试管被拿起时管内粉/水随管平移（保持相对管底偏移）。"""
        delta = self._tube_origin() - self.TUBE_ORIG
        self.object_utils.set_object_position(EFFECT_TUBE_POWDER, self.POWDER_REST + delta)
        self.object_utils.set_object_position(EFFECT_TUBE_WATER, self.TUBE_WATER_REST + delta)

    # ------------------------------------------------------------------
    # 每帧滴管持握（一次持握吸液挤液）：rest → 近抓点+合拢 → attached（跟随 _T_HELD_DROPPER）
    #   → 吸液/滴液边沿检测 → 松开（>0.03）→ released（回架 rest）。
    # ------------------------------------------------------------------
    def _update_dropper(self, gripper_pos, opening):
        if self.dropper_state == "rest":
            near = self._near_grasp(gripper_pos, self.DROP_GRASP)
            self._drop_near_frames = self._drop_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_dropper_to_gripper(gripper_pos)
            if (near and self._drop_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.DROP_GRIP_CLOSED):
                self.dropper_state = "attached"
                self._set_dropper_world(_T_HELD_DROPPER * self._tool_world())
                print(f"[a2] dropper attached (grip={opening:.4f})")
            return

        if self.dropper_state == "attached":
            self._set_dropper_world(_T_HELD_DROPPER * self._tool_world())
            self._step_dropper_transfer_detect(gripper_pos)
            if opening > self.gripper_open_threshold:
                self.dropper_state = "released"
                self._set_dropper_world(_dropper_rest_matrix())
                print(f"[a2] dropper released to rack (transfer={self.transfer_count})")
        # released：已回架，不再跟随

    def _ease_dropper_to_gripper(self, gripper_pos, k=0.18):
        """夹爪合拢期间滴管逐帧平滑拉向持握位（消除闪现吸附，旋转一致 → 纯平移插值足够）。"""
        target = _T_HELD_DROPPER * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.DROP_PATH)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_dropper_world(_blend_world(cur, target, k))

    def _set_dropper_world(self, world_matrix):
        """把滴管写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.DROP_PATH)
        if not prim.IsValid():
            return
        parent_xf = UsdGeom.Xformable(self.stage.GetPrimAtPath("/World")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    # ------------------------------------------------------------------
    # 吸液/滴液边沿检测（吸试管内液 → 挤进加液口，每遍循环一次）：
    #   下探到浸液点（DROPPER_DIP_TCP）→ 吸液（试管液变矮）；下探到加液口上
    #   （DROPPER_DROP_TCP）→ 滴液（成串液滴 + TubeLiquid 生长）。离开该点才复位边沿。
    # ------------------------------------------------------------------
    def _step_dropper_transfer_detect(self, gripper_pos):
        near_dip = self._near_grasp(gripper_pos, np.array(DROPPER_DIP_TCP), z_thresh=0.02)
        if near_dip and not self._at_dip:
            self._at_dip = True
            self.aspirate_count += 1
            self._do_aspirate(self.drop_cycles - self.aspirate_count)
            print(f"[a2] dropper aspirate {self.aspirate_count}/{self.drop_cycles}")
        elif not near_dip:
            self._at_dip = False
        near_drop = self._near_grasp(gripper_pos, np.array(DROPPER_DROP_TCP), z_thresh=0.02)
        if near_drop and not self._at_drop:
            self._at_drop = True
            self._do_drip(self.transfer_count + 1)
        elif not near_drop:
            self._at_drop = False

    def _do_aspirate(self, remaining):
        """第 (drop_cycles-remaining) 次吸液：试管液变矮（液底固定向上缩），吸空隐藏。"""
        prim = self.stage.GetPrimAtPath(EFFECT_TUBE_WATER)
        if prim.IsValid():
            h = max(0.0, self.TUBE_WATER_H * remaining / self.drop_cycles)
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            bottom = self.TUBE_WATER_REST[2] - self.TUBE_WATER_H / 2.0
            self.object_utils.set_object_position(
                EFFECT_TUBE_WATER,
                np.array([self.TUBE_WATER_REST[0], self.TUBE_WATER_REST[1], bottom + h / 2.0]))
            if h <= 1e-4:
                self._set_visibility(EFFECT_TUBE_WATER, False)

    def _do_drip(self, count):
        """第 count 次滴液：从尖嘴发射成串液滴坠入加液口 + 旋光管内液柱生长。"""
        self.transfer_count = count
        self._set_visibility(EFFECT_DROPPER_DRIP, True)
        # 预排成串 DRIP_BURST 滴（DRIP_STAGGER 帧错开），_step_drip_anim 逐帧下落
        for i in range(self.DRIP_BURST):
            idx = (self._drip_next + i) % self.DRIP_POOL
            self._set_visibility(f"{EFFECT_DROPPER_DRIP}/Drop_{idx}", True)
            self.object_utils.set_object_position(
                f"{EFFECT_DROPPER_DRIP}/Drop_{idx}", self.DRIP_START.copy())
            self._drip_queue.append({"prim": idx, "t": -i * self.DRIP_STAGGER})
        self._drip_next = (self._drip_next + self.DRIP_BURST) % self.DRIP_POOL
        self._set_tube_liquid_level(count)
        print(f"[a2] drip {count}/{self.drop_cycles} -> liquid in polarimeter tube")

    def _set_tube_liquid_level(self, count):
        """按滴入份数生长旋光管内液柱（TubeLiquid 子 prim：管局部系轴 Y 圆柱）。
        h = 0.10*count/cycles，translate 局部 y = TUBE_LIQ_BOT + h/2 → 液面从管身底部固定向上长。"""
        prim = self.stage.GetPrimAtPath(EFFECT_TUBE_LIQUID)
        if not prim.IsValid():
            return
        h = self.TUBE_LIQ_LEN * count / self.drop_cycles
        UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                v = op.Get()
                op.Set(Gf.Vec3d(v[0], self.TUBE_LIQ_BOT + h / 2.0, v[2]))
                return

    def _reset_tube_liquid(self):
        prim = self.stage.GetPrimAtPath(EFFECT_TUBE_LIQUID)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(self.TUBE_LIQ_LEN)
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetName() == "xformOp:translate":
                    v = op.Get()
                    op.Set(Gf.Vec3d(v[0], 0.0, v[2]))
                    break
        self._set_visibility(EFFECT_TUBE_LIQUID, False)

    def _reset_tube_water(self):
        prim = self.stage.GetPrimAtPath(EFFECT_TUBE_WATER)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(self.TUBE_WATER_H)
            self.object_utils.set_object_position(EFFECT_TUBE_WATER, self.TUBE_WATER_REST)
        self._set_visibility(EFFECT_TUBE_WATER, False)

    # ------------------------------------------------------------------
    # 滴液动画：成串液滴从尖嘴（加液口上 25mm）抛物线坠入加液口顶，落完隐藏
    # ------------------------------------------------------------------
    def _step_drip_anim(self):
        if not self._drip_queue:
            return
        remaining = []
        for d in self._drip_queue:
            d["t"] += 1
            if d["t"] < 0:
                remaining.append(d)      # 错帧：未到发射时刻，停在起点
                continue
            if d["t"] >= self.DRIP_FALL:
                self._set_visibility(f"{EFFECT_DROPPER_DRIP}/Drop_{d['prim']}", False)
                continue
            frac = d["t"] / self.DRIP_FALL
            z = self.DRIP_START[2] - (self.DRIP_START[2] - self.DRIP_END[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{EFFECT_DROPPER_DRIP}/Drop_{d['prim']}",
                np.array([self.DRIP_START[0], self.DRIP_START[1], z]))
            remaining.append(d)
        self._drip_queue = remaining
        if not remaining:
            self._set_visibility(EFFECT_DROPPER_DRIP, False)

    # ------------------------------------------------------------------
    # 每帧旋光管持握（纯平移）：rest → 近抓点+合拢 → attached（管中心=TCP+偏移）→
    #   松开（>0.03）→ dropping（PTUBE_DROP_FRAMES 帧落下）→ released（导轨落座，终态）
    # ------------------------------------------------------------------
    def _update_ptube(self, gripper_pos, opening):
        if self.ptube_state == "rest":
            near = self._near_grasp(gripper_pos, self.PTUBE_GRASP)
            self._ptube_near_frames = self._ptube_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_ptube_to_gripper(gripper_pos)
            if (near and self._ptube_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.PTUBE_GRIP_CLOSED):
                self.ptube_state = "attached"
                self._set_ptube_pos(gripper_pos + self.PTUBE_HELD_OFFSET)
                print(f"[a2] polarimeter tube attached (grip={opening:.4f})")
            return

        if self.ptube_state == "attached":
            self._set_ptube_pos(gripper_pos + self.PTUBE_HELD_OFFSET)
            if opening > self.gripper_open_threshold:
                self.ptube_state = "dropping"
                self._ptube_drop_from = self.object_utils.get_object_xform_position(self.PTUBE_PATH)
                self._ptube_drop_t = 0
                print(f"[a2] polarimeter tube released, dropping to rails (grip={opening:.4f})")

        if self.ptube_state == "dropping":
            # 松爪后管从当前高度「落下」导轨（用户 2026-08-28：不再深下探，直接松爪让管落下去）
            self._ptube_drop_t += 1
            k = min(1.0, self._ptube_drop_t / self.PTUBE_DROP_FRAMES)
            start = self.PTUBE_RAILS if self._ptube_drop_from is None else np.asarray(self._ptube_drop_from, dtype=float)
            self._set_ptube_pos(start + (self.PTUBE_RAILS - start) * k)
            if k >= 1.0:
                self.ptube_state = "released"
                print("[a2] polarimeter tube landed on rails")
        # released：已落座导轨，不再跟随

    def _ease_ptube_to_gripper(self, gripper_pos, k=0.18):
        cur = self.object_utils.get_object_xform_position(self.PTUBE_PATH)
        if cur is None:
            return
        target = gripper_pos + self.PTUBE_HELD_OFFSET
        self._set_ptube_pos(np.asarray(cur, dtype=float) + (target - cur) * k)

    def _set_ptube_pos(self, pos):
        """把旋光管写到给定世界位置（只写 translate，保横放泡朝上姿态）。"""
        self.object_utils.set_object_position(self.PTUBE_PATH, np.asarray(pos, dtype=float))

    # ------------------------------------------------------------------
    # 每帧测量键（a1 同款）：idle → measuring（按下，红进度条逐帧切显）→ result（读数屏
    #   定格）→ releasing（爪子抬离后缓慢弹回）→ released（读数保持）。
    # ------------------------------------------------------------------
    def _update_button(self, gripper_pos):
        if self.button_state == "idle":
            if self._near(np.array(START_BUTTON_PRESS_TCP), gripper_pos, z_thresh=0.012):
                self.button_state = "measuring"
                self.button_pressed = True
                self._step_shown = 0
                self._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=0), True)
                self._set_button_z(BUTTON_PRESSED_Z)   # 按钮下沉（按下效果）
                print("[a2] start button pressed (measuring… red progress bar 0%)")
        elif self.button_state == "measuring":
            self._measure_frames += 1
            step = min(int(self._measure_frames * PROGRESS_STEPS / self.MEASURE_FRAMES),
                       PROGRESS_STEPS - 1)
            if step != self._step_shown:
                if self._step_shown >= 0:
                    self._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=self._step_shown), False)
                self._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=step), True)
                self._step_shown = step
            if self._measure_frames >= self.MEASURE_FRAMES:
                self.button_state = "result"
                self.reading = True
                if self._step_shown >= 0:
                    self._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=self._step_shown), False)
                    self._step_shown = -1
                self._set_visibility(self._result_screen, True)   # 完成：旋光角读数
                print(f"[a2] measurement done -> rotation {self.rotation_angle} shown")
        elif self.button_state == "result":
            if gripper_pos[2] > BUTTON_LIFT_Z:
                self.button_state = "releasing"
                print("[a2] button releasing (gripper lifted, slow spring back)")
        elif self.button_state == "releasing":
            z = self._get_button_z()
            z = min(z + BUTTON_SPRING_STEP, BUTTON_REST_Z)
            self._set_button_z(z)
            if z >= BUTTON_REST_Z:
                self.button_state = "released"
                print("[a2] button back to rest")
        # released：按钮回位，读数保持定格

    # ------------------------------------------------------------------
    # 拨回翻盖（⑩ CloseLidPass 夹爪在机身近侧 x0.51 向 −y 推板时，按夹爪 y 进度把 lid
    #   从掀开 120° 转到闭合 0°。门控防误闭：须 ptube 已释放（放导轨后）且夹爪 x≈LID_PUSH_X
    #   且此刻从上往下越过 Y0（−0.15）——即 CloseLidPass ④ 推盖段首次入带（PlaceOnRails
    #   放管时 ptube 未释放 / 释放后 y 恒 −0.24 无越界，绝不误锁存）。
    # ------------------------------------------------------------------
    def _update_lid(self, gripper_pos):
        if self.lid_state == "closed":
            return
        # 门控（防「放管瞬间闪现闭合」）：只在此帧夹爪 y 从上往下第一次越过 LID_PUSH_Y0
        # （−0.15，触板近面）时锁存 _lid_push_seen。旧「带内 + moving−y」判据在放管后夹爪
        # 已深在 y−0.24 时可能一帧直接满足 → progress 跳 1 → lid 闪现闭合；越界判据保证
        # 只从 Y0 起随推盖 −y 逐步合拢（y≤Y0 progress=0 全开 → y≤Y1 全闭），有翻下流程。
        # x 门控（LID_PUSH_X=0.51）：滴管挤液 x0.553/震荡 x0.53 在带内但 y0.241 不越 Y0 不误锁存。
        if (abs(gripper_pos[0] - LID_PUSH_X) < 0.05
                and self.ptube_state == "released"
                and self._prev_lid_y is not None
                and self._prev_lid_y > LID_PUSH_Y0
                and gripper_pos[1] <= LID_PUSH_Y0):
            self._lid_push_seen = True
        self._prev_lid_y = gripper_pos[1]
        if not self._lid_push_seen:
            return
        progress = max(0.0, min(1.0, (LID_PUSH_Y0 - gripper_pos[1]) / (LID_PUSH_Y0 - LID_PUSH_Y1)))
        angle = LID_OPEN_DEG * (1.0 - progress)
        # 单调递减：lid 只继续合拢、绝不随抬升回弹（CloseLidPass ⑤ 抬回 +y 时 angle 保持 0）
        self._lid_angle = min(self._lid_angle, angle)
        self._set_lid_angle(self._lid_angle)
        if self._lid_angle <= 0.5:
            self.lid_state = "closed"
            print("[a2] lid flipped closed")

    def _set_lid_angle(self, deg):
        """写 lid 的 rotateXYZ.Y（掀 120°→0° 闭合；旋转轴沿世界±X 过铰链 y−0.184）。"""
        prim = self.stage.GetPrimAtPath(LID_PATH)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:rotateXYZ":
                v = op.Get()
                op.Set(Gf.Vec3f(v[0], float(deg), v[2]))
                return

    # ------------------------------------------------------------------
    # 挤水水流（洗瓶红嘴 → 试管口，抛物线坠入；d2s 同款）
    # ------------------------------------------------------------------
    def _step_water_anim(self):
        if self.squeezing:
            self._water_spawn += 1
            if self._water_spawn >= self.WATER_STAGGER:
                self._water_spawn = 0
                idx = self._water_next_prim % self.WATER_DROPS
                self._water_next_prim += 1
                self._set_visibility(f"{EFFECT_WATER_STREAM}/Drop_{idx}", True)
                self.object_utils.set_object_position(
                    f"{EFFECT_WATER_STREAM}/Drop_{idx}", self.WATER_START.copy())
                self._water_queue.append({"prim": idx, "t": 0})
        if not self._water_queue:
            return
        remaining = []
        for d in self._water_queue:
            d["t"] += 1
            if d["t"] >= self.WATER_FALL:
                self._set_visibility(f"{EFFECT_WATER_STREAM}/Drop_{d['prim']}", False)
                continue
            frac = d["t"] / self.WATER_FALL
            x = self.WATER_START[0] + (self.WATER_END[0] - self.WATER_START[0]) * frac
            y = self.WATER_START[1] + (self.WATER_END[1] - self.WATER_START[1]) * frac
            z = self.WATER_START[2] - (self.WATER_START[2] - self.WATER_END[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{EFFECT_WATER_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._water_queue = remaining
        if not remaining and not self.squeezing:
            self._set_visibility(EFFECT_WATER_STREAM, False)

    # ------------------------------------------------------------------
    # 判定 / 位姿 / 辅助
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _set_button_z(self, z):
        """写按钮 /World/Polarimeter/start_button 的 translate z（按下下沉/弹回）。
        只改 translate 的 z 分量，保留 x/y（局部，Polarimeter 原点贴台面）。"""
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                v = op.Get()
                op.Set(Gf.Vec3d(v[0], v[1], float(z)))
                return

    def _get_button_z(self):
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return BUTTON_REST_Z
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                return float(op.Get()[2])
        return BUTTON_REST_Z

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


def _tube_rest_matrix():
    """试管架内竖插位姿（场景 /World/TestTube 世界矩阵：translate (0.659,0.241,0.806)，无旋转）。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z, 1.0)


def _washbottle_rest_matrix():
    """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵：rotZ -180 + translate (0.370,0.525,0.80)
    烘平后即下行序；行 0=(-1,0,0) → 局部 +X 朝世界 -X、红嘴朝 +X（对试管方向））。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.370, 0.525, 0.80, 1.0)


def _dropper_rest_matrix():
    """滴管架内竖插位姿（场景 /World/Dropper 世界矩阵：translate DROP_REST，无旋转）。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       DROP_REST[0], DROP_REST[1], DROP_REST[2], 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切，d2s 同款）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
