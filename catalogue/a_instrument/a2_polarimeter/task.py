# -*- coding: utf-8 -*-
"""A2 旋光仪测量任务：试管预装粉 → 洗瓶加水 → 震荡溶解 → 倒进旋光管 → 放导轨 → 按启动键读数。

三类物体三种持握（同 d2s/d3s 约定）：
  试管（/World/TestTube）    旋转跟随 _T_HELD_TUBE（管底吊夹爪下 0.1393m；ORIENT_FWD 管口朝上、
                            ORIENT_POUR 倒置管口朝下），**两个吸附周期**：
                              周期1 = TubeShakePass 震荡溶解（管内粉/水随管平移，释放回架时
                                      粉溶尽隐藏）；
                              周期2 = PickTestTube+PourToTube+ReturnTestTube 倒液进旋光管
                                      （倒置 + 近倒液点 → 隐藏管内水 + 发射 PourStream → 液
                                      进旋光管显 TubeLiquid，空管放回架孔）。
  洗瓶（/World/WashBottle）  动态锁 _T_HELD_WASHB = 静止矩阵 · tool^-1，横夹肚子随夹爪平移；
                            开合 < 0.025 判挤水（WaterStream 水流进试管）、> 0.038 松开回表位
  旋光管（/World/PolarimeterTube） 纯平移持握（set_object_position，保横放泡朝上），
                            释放到导轨 rest (0.30,-0.03,1.0075)；TubeLiquid 是它的子 prim
                            随管移动（倒液时在桌面显液柱，随后跟着管一起上导轨）。

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
    POUR_TCP, POUR_MOUTH, FILL_XY, FILL_TOP_Z, WATER_START, WATER_END,
    START_BUTTON_PRESS_TCP, BUTTON_PATH, BUTTON_REST_Z, BUTTON_PRESSED_Z,
    BUTTON_LIFT_Z, BUTTON_SPRING_STEP,
    EFFECT_TUBE_POWDER, EFFECT_TUBE_WATER, EFFECT_TUBE_LIQUID,
    EFFECT_WATER_STREAM, EFFECT_POUR_STREAM,
    EFFECT_SCREEN_MEASURING_TPL, EFFECT_SCREEN_RESULT_TPL, PROGRESS_STEPS,
    ROTATION_DEFAULT, ROTATION_KEY,
)

# 试管相对夹爪的持握矩阵（行向量约定，平移在最后一行）：
#   tube+X → tool+Z、tube+Y → tool+Y、tube+Z → -tool+X，管底原点在 +0.1393 tool+X。
# ORIENT_FWD（tool+Z=+X、tool+X=-Z）下：管底 = gripper-0.1393·Z（吊夹爪下方）、管口朝上；
# ORIENT_POUR（tool+X=+Z）下：管底在夹爪上方、管口朝下（倒置）。管倒置判定读管世界矩阵
# row2（tube+Z 世界方向）的 z 分量 < -0.5。
_T_HELD_TUBE = Gf.Matrix4d(0.0, 0.0, 1.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           -1.0, 0.0, 0.0, 0.0,
                           0.1393, 0.0, 0.0, 1.0)


class A2PolarimeterTask(BaseTask):
    """A2 旋光仪测量任务：洗瓶注水 + 试管两周期持握（震荡/倒液）+ 旋光管平移 + 屏幕读数。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # ---- 试管（两个吸附周期，见文件头）----
    TUBE = "/World/TestTube"
    TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z])    # 管底架内竖插位
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)                          # 抓点（管口下 14mm）
    TUBE_GRIP_CLOSED = GRIP_TUBE + 0.004                           # 夹紧阈值 0.0136
    # 管内效果 rest 位（gen_a2_scene.py 已预制，TUBE_BOT+0.034 / +0.049）
    POWDER_REST = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z + 0.034])
    TUBE_WATER_REST = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z + 0.049])

    # ---- 洗瓶（d2s 同款横夹肚子）----
    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array(WASH_GRASP)
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004                        # 夹紧阈值 0.034
    WASH_GRIP_OPEN = WASH_GRIP_OPEN                                # 松开阈值 0.038（<满开 0.04）

    # ---- 旋光管（纯平移持握）----
    PTUBE_PATH = "/World/PolarimeterTube"
    PTUBE_REST = np.array(PTUBE_REST)                              # 桌面横放 (0.70,0.30,0.811)
    PTUBE_RAILS = np.array(PTUBE_PLACE_CENTER)                     # 导轨落座 (0.30,-0.03,1.0075)
    PTUBE_GRASP = np.array(PTUBE_GRASP)
    PTUBE_GRIP_CLOSED = GRIP_PTUBE + 0.004                         # 夹紧阈值 0.0105
    PTUBE_HELD_OFFSET = np.array([0.0, 0.0, PTUBE_HELD_OFFSET_Z])  # 管中心=TCP-0.019

    # ---- 挤水水流（洗瓶红嘴 → 试管口，抛物线坠入）----
    WATER_START = np.array(WATER_START)                            # 嘴尖 0.994
    WATER_END = np.array(WATER_END)                                # 管口 0.9593
    WATER_DROPS = 16
    WATER_STAGGER = 2
    WATER_FALL = 12

    # ---- 倒液水流（倒置试管口 → 旋光管加液口）----
    POUR_MOUTH = np.array(POUR_MOUTH)                              # 倒置管口 0.846
    POUR_FILL = np.array([FILL_XY[0], FILL_XY[1], FILL_TOP_Z])     # 加液口顶 0.830
    POUR_DROPS = 16
    POUR_STAGGER = 2
    POUR_FALL = 10

    # ---- 屏幕读数（进度条 ~4s 走完 → 结果屏定格）----
    MEASURE_FRAMES = 240

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 静态碰撞体持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰，d2s 同款）
        self._disable_collision(self.TUBE)
        self._disable_collision(self.WASH_PATH)
        self._disable_collision(self.PTUBE_PATH)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.squeeze_close_threshold = getattr(cfg, "squeeze_close_threshold", 0.025)

        # 旋转角度读数（experiment_result 框架 --result rotation_angle=<> / TTY 交互写回
        # cfg.rotation_angle，default "+12.5"）→ 选对应预烘焙读数屏 ScreenGlow_<key>
        # （headless 下运行时改材质不渲染 → 按档位预烘焙，a1 同款）
        self.rotation_angle = str(getattr(cfg, "rotation_angle", ROTATION_DEFAULT)).strip()
        self._result_screen = EFFECT_SCREEN_RESULT_TPL.format(key=ROTATION_KEY(self.rotation_angle))

        # ---- 试管两周期状态 ----
        self.tube_state = "rest"      # rest / attached / released(→回 rest 可再抓)
        self.tube_cycle = 0           # 1=震荡周期、2=倒液周期（attach 时递增）
        self._tube_near_frames = 0
        self.poured = False           # 粘性标志：液已进旋光管（倒液完成）
        self.pouring = False          # 倒液中（持续发射 PourStream）
        self._pour_queue = []         # 在飞倒液滴（prim/t）
        self._pour_spawn = 0          # 距下次发射倒液滴倒计时
        self._pour_next = 0           # round-robin 池游标

        # ---- 洗瓶状态 ----
        self.washbottle_state = "rest"  # rest / attached / released
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self.squeezing = False        # 挤水进行中（持续发射水滴）
        self.water_in_tube = False    # 已挤入水（管内水显示，只触发一次）
        self._water_queue = []        # 在飞水滴（prim/t）
        self._water_next_prim = 0
        self._water_spawn = 0

        # ---- 旋光管状态 ----
        self.ptube_state = "rest"     # rest / attached / released
        self._ptube_near_frames = 0

        # ---- 测量键状态（a1 同款）----
        self.button_state = "idle"    # idle / measuring / result / releasing / released
        self.button_pressed = False
        self.reading = False
        self._measure_frames = 0
        self._step_shown = -1

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        # 试管回架 + 两周期清零 + 粉/水回 rest（粉可见：预装白粉初始状态）
        self.tube_state = "rest"
        self.tube_cycle = 0
        self._tube_near_frames = 0
        self.poured = False
        self.pouring = False
        self._pour_queue = []
        self._pour_spawn = 0
        self._pour_next = 0
        self._set_tube_world(_tube_rest_matrix())
        self._set_visibility(EFFECT_TUBE_POWDER, True)
        self._set_visibility(EFFECT_TUBE_WATER, False)
        self.object_utils.set_object_position(EFFECT_TUBE_POWDER, self.POWDER_REST)
        self.object_utils.set_object_position(EFFECT_TUBE_WATER, self.TUBE_WATER_REST)
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
        # 旋光管回桌面 + 管内液柱隐藏
        self.ptube_state = "rest"
        self._ptube_near_frames = 0
        self._set_ptube_pos(self.PTUBE_REST)
        self._set_visibility(EFFECT_TUBE_LIQUID, False)
        self._set_visibility(EFFECT_POUR_STREAM, False)
        for i in range(self.POUR_DROPS):
            self._set_visibility(f"{EFFECT_POUR_STREAM}/Drop_{i}", False)
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
        self._update_tube(gripper_pos, opening)         # 试管两周期持握 + 倒液判定
        self._update_ptube(gripper_pos, opening)        # 旋光管持握
        self._update_button(gripper_pos)                # 测量键生命周期
        self._step_water_anim()                         # 挤水水流
        self._step_pour_anim()                          # 倒液水流
        return self.get_basic_state_info(additional_info={
            "washbottle_state": self.washbottle_state,
            "tube_state": self.tube_state,
            "tube_cycle": self.tube_cycle,
            "water_in_tube": self.water_in_tube,
            "poured": self.poured,
            "ptube_state": self.ptube_state,
            "button_state": self.button_state,
            "button_pressed": self.button_pressed,
            "reading": self.reading,
        })

    def on_task_complete(self, success):
        print(f"[a2] episode done success={success} "
              f"washbottle={self.washbottle_state} tube={self.tube_state} cycle={self.tube_cycle} "
              f"poured={self.poured} ptube={self.ptube_state} "
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
    # 每帧试管持握（两个吸附周期）：rest → 近抓点+合拢 → attached（旋转跟随 _T_HELD_TUBE）
    #   → released（回架 rest，可再抓）。周期1释放隐藏管内粉（溶尽）；周期2倒置+近倒液点
    #   触发倒液（隐藏管内水 + PourStream 水流），倒完液进旋光管显 TubeLiquid。
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
                self.tube_cycle += 1
                self._set_tube_world(_T_HELD_TUBE * self._tool_world())
                print(f"[a2] tube attached cycle={self.tube_cycle} (grip={opening:.4f})")
            return

        if self.tube_state == "attached":
            self._set_tube_world(_T_HELD_TUBE * self._tool_world())
            self._follow_tube_effects()
            self._step_pour_detect(gripper_pos)
            if opening > self.gripper_open_threshold:
                # 释放回架（可再抓）：周期1=震荡后粉溶尽隐藏；周期2=倒完液空管回架
                if self.tube_cycle == 1:
                    self._set_visibility(EFFECT_TUBE_POWDER, False)
                    print("[a2] tube cycle1 released -> powder dissolved into water")
                self.pouring = False
                self.tube_state = "rest"
                self._set_tube_world(_tube_rest_matrix())
                self._follow_tube_effects()
                print(f"[a2] tube released to rack cycle={self.tube_cycle}")

    def _ease_tube_to_gripper(self, gripper_pos, k=0.18):
        """夹爪合拢期间试管逐帧平滑拉向持握位（消除闪现吸附，旋转一致 → 纯平移插值足够）。"""
        target = _T_HELD_TUBE * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.TUBE)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_tube_world(_blend_world(cur, target, k))

    def _tube_world_matrix(self):
        """读试管当前世界矩阵（管口朝向判定用）。"""
        prim = self.stage.GetPrimAtPath(self.TUBE)
        if not prim.IsValid():
            return Gf.Matrix4d()
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _tube_origin(self):
        """试管当前世界原点（管底，旋转跟随下 = 管底中心）。"""
        return np.array(self._tube_world_matrix().ExtractTranslation(), dtype=float)

    def _tube_inverted(self):
        """试管是否倒置（管口朝下）：管局部 +Z（管口方向）世界 z 分量 < -0.5。"""
        return self._tube_world_matrix()[2][2] < -0.5

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
        """试管被拿起时管内粉/水随管平移（保持相对管底偏移）。粉全程跟随（周期1震荡中可见）；
        水只在不倒置时跟随（倒置 = 倒液中，水已倒出、_step_pour_detect 负责隐藏）。"""
        delta = self._tube_origin() - self.TUBE_ORIG
        self.object_utils.set_object_position(EFFECT_TUBE_POWDER, self.POWDER_REST + delta)
        if not self._tube_inverted():
            self.object_utils.set_object_position(EFFECT_TUBE_WATER, self.TUBE_WATER_REST + delta)

    # ------------------------------------------------------------------
    # 倒液（周期2）：试管倒置 + 近倒液点 → 开始倒液（隐藏管内水 + PourStream 水流）；
    # 试管离开倒液点 → 停止发射，在飞滴落完 → poured（液进旋光管，TubeLiquid 显）。
    # ------------------------------------------------------------------
    def _step_pour_detect(self, gripper_pos):
        inverted = self._tube_inverted()
        near_pour = self._near_grasp(gripper_pos, np.array(POUR_TCP), z_thresh=0.02)
        if self.poured:
            return
        if not self.pouring and inverted and near_pour:
            self.pouring = True
            self._set_visibility(EFFECT_TUBE_WATER, False)   # 水已倒出
            self._set_visibility(EFFECT_POUR_STREAM, True)
            print(f"[a2] pour started (inverted near fill port, cycle={self.tube_cycle})")
        elif self.pouring and not (inverted and near_pour):
            self.pouring = False
            print("[a2] pour ended (tube lifted away, draining stream)")

    def _step_pour_anim(self):
        """倒液水流：倒液中每 POUR_STAGGER 帧发射一颗水滴，抛物线从倒置管口坠入加液口；
        停止发射后让在飞滴落完 → 液进旋光管（TubeLiquid 显、PourStream 隐藏）。"""
        if self.pouring:
            self._pour_spawn += 1
            if self._pour_spawn >= self.POUR_STAGGER:
                self._pour_spawn = 0
                idx = self._pour_next % self.POUR_DROPS
                self._pour_next += 1
                self._set_visibility(f"{EFFECT_POUR_STREAM}/Drop_{idx}", True)
                self.object_utils.set_object_position(
                    f"{EFFECT_POUR_STREAM}/Drop_{idx}", self.POUR_MOUTH.copy())
                self._pour_queue.append({"prim": idx, "t": 0})
        if not self._pour_queue:
            return
        remaining = []
        for d in self._pour_queue:
            d["t"] += 1
            if d["t"] >= self.POUR_FALL:
                self._set_visibility(f"{EFFECT_POUR_STREAM}/Drop_{d['prim']}", False)
                continue
            frac = d["t"] / self.POUR_FALL
            x = self.POUR_MOUTH[0] + (self.POUR_FILL[0] - self.POUR_MOUTH[0]) * frac
            y = self.POUR_MOUTH[1] + (self.POUR_FILL[1] - self.POUR_MOUTH[1]) * frac
            z = self.POUR_MOUTH[2] - (self.POUR_MOUTH[2] - self.POUR_FILL[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{EFFECT_POUR_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._pour_queue = remaining
        if not remaining and not self.pouring and not self.poured:
            self._set_visibility(EFFECT_POUR_STREAM, False)
            self.poured = True
            self._set_visibility(EFFECT_TUBE_LIQUID, True)   # 液已进旋光管（子 prim 随管移动）
            print("[a2] pour complete -> liquid in polarimeter tube")

    # ------------------------------------------------------------------
    # 每帧旋光管持握（纯平移）：rest → 近抓点+合拢 → attached（管中心=TCP+偏移）→
    #   松开（>0.03）→ released（导轨落座，终态）
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
                self.ptube_state = "released"
                self._set_ptube_pos(self.PTUBE_RAILS)
                print(f"[a2] polarimeter tube released on rails (grip={opening:.4f})")
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
    """试管架内竖插位姿（场景 /World/TestTube 世界矩阵：translate (0.799,0.43,0.806)，无旋转）。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z, 1.0)


def _washbottle_rest_matrix():
    """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵：rotZ -180 + translate (-0.10,0.60,0.80)
    烘平后即下行序；行 0=(-1,0,0) → 局部 +X 朝世界 -X、红嘴朝 +X（对试管方向））。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       -0.10, 0.60, 0.80, 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切，d2s 同款）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
