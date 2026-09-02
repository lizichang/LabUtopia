# -*- coding: utf-8 -*-
"""A3 电导率测量任务（v9：夹皿提起 → 移烧杯上方 → 倾斜倒粉 → 放回空皿 → 夹洗瓶 → 挤水
→ 放回洗瓶 → 夹玻璃棒移烧杯上方 → 下降插入烧杯搅拌 → 放回玻璃棒 → 竖直提起电极）。

表面皿生命周期：rest → 近抓点+合爪 → attached（皿随夹爪 6-DOF 持握，含旋转——倒粉需倾斜）
→ 开爪 → released（皿+粉回 rest）。洗瓶生命周期：rest → 近抓点+合爪 → attached（纯平移持握）
→ 挤水（夹爪 <0.025 挤压瓶身 → 水流 + 烧杯液面上涨）→ 开爪 → released（回表位）。
玻璃棒生命周期：rest → 近抓点+合爪 → attached（纯平移持握，棒底 = TCP − 0.20；下探/搅拌/
提出/放回架全程跟随）→ 开爪 → released（回架位）。电极生命周期：rest → 近抓点+合爪 →
attached（纯 z 平移持握，电极 prim 写 (0,0,lift_z)）→ 开爪 → released（回 rest lift=0）。
后续步骤（电极浸入烧杯 / 读数）逐步追加。

皿持握 = 6-DOF（d2s 药匙同款）：皿世界 = _T_HELD_DISH · tool_center 世界矩阵。_T_HELD_DISH
旋转 = R_y(π)（皿 +z 朝上、tool +z 朝下翻 180°）、平移 = 皿原点在 tool 局部 +z 0.0046m
（皿 prim 原点在皿底；TCP=tool_center 比指端高 0.027，指端 0.825 进天平机身顶 15mm——无碰撞仅
接近时短暂穿入；皿底 0.8474 在指端上方 22.4mm——五改再往下伸 1cm；attach 时皿原点 0.8520−0.0046
= 0.8474 = rest，零跳变）。手腕绕 Y 轴倾斜时皿随 tool 旋转 → 皿 -X 侧下降、粉末沿 -X 滑出。
粉堆 = 程序化圆柱 /World/PowderOnDish（Ø22×6，可 shrink「倒下」），随皿 6-DOF（粉堆中心 = 皿原点
+ 0.0096 在皿局部 +z），皿倾斜时粉堆同倾斜，倒粉时随粉粒落定逐渐缩小。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    DISH_XY, DISH_ORIG_REST_Z, DISH_GRASP_Z, GRIP_DISH,
    DISH_GRIP_OPEN, DISH_HELD_OFFSET_Z,
    POWDER_PATH, POWDER_ORIG_REST_Z, POWDER_HELD_OFFSET_Z,
    POWDER_BLOB_R, POWDER_BLOB_H,
    POWDER_LAND, BEAKER_POWDER_PATH,
    POWDER_DROPS, POWDER_STAGGER, POWDER_HANG, POWDER_FALL,
    WASH_XY, WASH_GRASP_Y, WASH_GRASP_Z, GRIP_WASHBOT,
    WASH_SQUEEZE_CLOSED, SPOUT_TIP_OFFSET,
    WATER_DROPS, WATER_STAGGER, WATER_FALL, WATER_LAND_FULL,
    BEAKER_MOUTH_TOP,
    BEAKER_LIQUID_PATH, BEAKER_LIQUID_R, BEAKER_LIQUID_H0, BEAKER_LIQUID_H_MAX,
    ROD_GRASP, ROD_REST_POS, GRIP_ROD, ROD_TIP_TO_GRIP,
    ELECTRODE_PATH, ELECTRODE_GRASP, GRIP_ELECTRODE,
    ELECTRODE_CAP_TOP, CABLE_PATH,
    BTN_PRESS, BUTTON_PATH, BUTTON_REST_Z, BUTTON_PRESSED_Z,
    BUTTON_LIFT_Z, BUTTON_SPRING_STEP,
    EFFECT_SCREEN_MEASURING_TPL, EFFECT_SCREEN_RESULT_TPL,
    PROGRESS_STEPS, CONDUCTIVITY_DEFAULT,
)
from .dynamic_cable import DynamicCable

# 皿相对夹爪的固定变换（6-DOF 持握）：旋转 R_y(π)（皿 +z 朝上 ↔ tool +z 朝下）+ 平移
# (0,0,-DISH_HELD_OFFSET_Z)=(0,0,0.0046)（皿原点在 tool 局部 +z）。行向量约定：先
# _T_HELD_DISH（皿局部→夹爪局部）再 tool_world（局部→世界），写反会把旋转作用到世界系。
# pxr 已验证：朝下时 dish 世界旋转=单位（+z 朝上）、平移=(0,0,-0.0046) 与纯平移一致。
_T_HELD_DISH = Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0, 0.0,
                            0.0, 0.0, -DISH_HELD_OFFSET_Z, 1.0)


class _ButtonLifecycle:
    """机顶「开始」键状态（idle/measuring/result/releasing/released）。按下 = 爪子垂直下探到按钮顶
    (0.3549,-0.133,0.938)，检测爪子近按钮顶 → 按钮下沉 BUTTON_PRESSED_Z + 显示测量进度条
    （ScreenMeasuring_<i> 预烘焙多帧，红条 0%→100%，MEASURE_FRAMES≈4s 内逐帧切显）→ 走完切
    result 读数屏 ScreenGlow_<key>（绿满条 + cfg.conductivity 档位读数，定格）→ 爪子抬离
    > BUTTON_LIFT_Z → 按钮缓慢弹回 BUTTON_REST_Z（releasing → released，读数保持）。按钮无碰撞，
    非物理下压（折光仪 A1 _ButtonLifecycle 同款）。"""

    def __init__(self, task):
        self.task = task
        self.state = "idle"
        self.pressed = False
        self.reading = False
        self._frames = 0
        self._step_shown = -1   # 当前显示的进度条帧（-1 = 无）

    def reset(self):
        self.state = "idle"
        self.pressed = self.reading = False
        self._frames = 0
        self._step_shown = -1
        for i in range(PROGRESS_STEPS):
            self.task._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=i), False)
        self.task._set_visibility(self.task._result_screen, False)
        self.task._set_button_z(BUTTON_REST_Z)

    def _hide_progress(self):
        if self._step_shown >= 0:
            self.task._set_visibility(
                EFFECT_SCREEN_MEASURING_TPL.format(step=self._step_shown), False)
            self._step_shown = -1

    def step(self, gripper_pos):
        if self.state == "idle":
            if self.task._near_grasp(gripper_pos, self.task.START_BUTTON_PRESS,
                                     z_thresh=0.012):
                self.state = "measuring"
                self.pressed = True
                self._step_shown = 0
                self.task._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=0), True)
                self.task._set_button_z(BUTTON_PRESSED_Z)   # 按钮下沉（按下效果）
                print("[a3] start button pressed (measuring… red progress bar 0%)")
        elif self.state == "measuring":
            self._frames += 1
            step = min(int(self._frames * PROGRESS_STEPS / self.task.MEASURE_FRAMES),
                       PROGRESS_STEPS - 1)
            if step != self._step_shown:
                if self._step_shown >= 0:
                    self.task._set_visibility(
                        EFFECT_SCREEN_MEASURING_TPL.format(step=self._step_shown), False)
                self.task._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=step), True)
                self._step_shown = step
            if self._frames >= self.task.MEASURE_FRAMES:
                self.state = "result"
                self.reading = True
                self._hide_progress()
                self.task._set_visibility(self.task._result_screen, True)
                print(f"[a3] measurement done -> {self.task.conductivity} mS/cm shown (green bar)")
        elif self.state == "result":
            # 爪子抬离按钮（z > BUTTON_LIFT_Z）→ 按钮缓慢弹回（读数保持显示）
            if gripper_pos[2] > BUTTON_LIFT_Z:
                self.state = "releasing"
                print("[a3] button releasing (gripper lifted, slow spring back)")
        elif self.state == "releasing":
            z = self.task._get_button_z()
            z = min(z + BUTTON_SPRING_STEP, BUTTON_REST_Z)
            self.task._set_button_z(z)
            if z >= BUTTON_REST_Z:
                self.state = "released"
                print("[a3] button back to rest")
        # released：按钮回位，读数保持定格


class A3ConductivityTask(BaseTask):
    """A3 电导率测量任务（v9：夹皿提起 → 移烧杯上方 → 倾斜倒粉 → 放回空皿 → 夹洗瓶 → 挤水 → 放回洗瓶 → 夹玻璃棒移烧杯上方 → 搅拌 → 放回玻璃棒 → 竖直提起电极）。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3
    # 倒粉判定：tool+Z（手指方向）从朝下 (0,0,-1) 转朝 +X 斜下，x 分量超过此阈值判已倾斜。
    # 2026-08-30 用户「粉末应在倒粉的最后一个旋转时倒出来」：最终 tool+Z x=0.866，阈值 0.5 太早
    # （倾斜 ~58% 就落粉）→ 0.82（倾斜 ~95% 近末段才落）。
    TILT_X_THRESH = 0.82

    DISH = "/World/SurfaceDish"
    DISH_ORIG = np.array([DISH_XY[0], DISH_XY[1], DISH_ORIG_REST_Z])   # 皿 prim 原点 rest
    DISH_GRASP = np.array([DISH_XY[0], DISH_XY[1], DISH_GRASP_Z])      # 抓点（tool_center，指端 0.825 进机身顶 15mm）
    DISH_GRIP_CLOSED = GRIP_DISH + 0.004                                # 夹紧阈值 0.031
    DISH_HELD_OFFSET = np.array([0.0, 0.0, DISH_HELD_OFFSET_Z])         # 皿原点 = TCP + 偏移（纯平移参考）
    POWDER_ORIG = np.array([DISH_XY[0], DISH_XY[1], POWDER_ORIG_REST_Z])
    POWDER_OFFSET = np.array([0.0, 0.0, POWDER_HELD_OFFSET_Z])          # 粉原点 = 皿原点 + 偏移（皿局部 +z）

    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array([WASH_XY[0], WASH_GRASP_Y, WASH_GRASP_Z])     # 抓点（tool_center，瓶身中上部偏 -y 0.5cm，与 controller 夹点一致）
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004                             # 夹紧阈值：grip 0.030 + 4mm 裕量
    WASH_GRIP_OPEN = 0.038                                              # 松开阈值（同 d2s；>0.038 才算松开）
    # 挤水（⑥）：夹爪从持握 0.030 进一步合到 0.020 挤压瓶身 → 水流 + 液面上涨
    #（夹爪开度作触发信号，d2s SqueezeWater 同款；持握 0.030 > 0.025 不会误触）
    WASH_SQUEEZE_CLOSED = WASH_SQUEEZE_CLOSED
    WATER_STREAM = "/World/WaterStream"
    WATER_DROPS = WATER_DROPS
    WATER_STAGGER = WATER_STAGGER
    WATER_FALL = WATER_FALL
    WATER_LAND_FULL = WATER_LAND_FULL
    WATER_START_OFFSET = np.array(SPOUT_TIP_OFFSET)      # 红嘴尖相对瓶原点世界偏移（纯平移持握恒定）
    WATER_TARGET = np.array(BEAKER_MOUTH_TOP)            # 水落点 = 烧杯口顶中心
    LIQUID_PATH = BEAKER_LIQUID_PATH                     # 烧杯内液面圆柱（淡蓝半透明）
    LIQUID_R = BEAKER_LIQUID_R
    LIQUID_H0 = BEAKER_LIQUID_H0
    LIQUID_H_MAX = BEAKER_LIQUID_H_MAX

    ROD_PATH = "/World/GlassRod"                         # 玻璃棒（试管架内）
    ROD_GRASP = np.array(ROD_GRASP)                      # 抓点（tool_center，棒底上方 0.20m）
    ROD_GRIP_CLOSED = GRIP_ROD + 0.004                   # 夹紧阈值：grip 0.003 + 4mm 裕量
    ROD_GRIP_OPEN = 0.038                                # 松开阈值（同皿/洗瓶；>0.038 才算松开）

    ELECTRODE_PATH = ELECTRODE_PATH                      # 导电率仪电极 prim（/World/Meter/electrode）
    ELECTRODE_GRASP = np.array(ELECTRODE_GRASP)          # 抓点（tool_center，cap 中心 z=0.945）
    ELECTRODE_GRIP_CLOSED = GRIP_ELECTRODE + 0.004       # 夹紧阈值：grip 0.010 + 4mm 裕量
    ELECTRODE_GRIP_OPEN = 0.038                          # 松开阈值（同皿/洗瓶/棒；>0.038 才算松开）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰，d2s 同款）
        self._disable_collision(self.DISH)
        self._disable_collision(POWDER_PATH)
        self.washbottle_path = self.WASH_PATH
        self._disable_collision(self.washbottle_path)
        self.rod_path = self.ROD_PATH
        self._disable_collision(self.rod_path)
        self.electrode_path = self.ELECTRODE_PATH
        self._disable_collision(self.electrode_path)
        # 动态电缆：电极被夹走时逐帧重算 Catmull-Rom 样条让软线跟随（移动端 = cap 顶）
        self.cable = DynamicCable(self.stage, CABLE_PATH)
        self._electrode_cap_top = np.array(ELECTRODE_CAP_TOP, dtype=float)
        # cap 顶比抓点 TCP 高的固定 z 偏移（cap 顶 0.965 − 抓点 0.9625 = 0.0025；电缆移动端随夹爪）
        self._cap_top_dz = float(ELECTRODE_CAP_TOP[2] - self.ELECTRODE_GRASP[2])

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        self.dish_state = "rest"   # rest / attached / released
        self._dish_near_frames = 0
        self.poured = False            # 已倒粉（粉末落入烧杯）
        self.powder_falling = False    # 粉末下落动画进行中
        self._powder_queue = []        # 下落动画队列

        self.washbottle_state = "rest"   # rest / attached / released
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None

        self.rod_state = "rest"          # rest / attached / released
        self._rod_near_frames = 0

        self.electrode_state = "rest"    # rest / attached / released
        self._electrode_near_frames = 0

        # 导电率读数（experiment_result 框架 --result conductivity=<> / TTY 交互写回 cfg.conductivity，
        # default 1.413）→ 选对应预烘焙读数屏 ScreenGlow_<key>（A1 nD 同款）。按下机顶开始键后
        # _ButtonLifecycle 驱动测量进度条 → 结果屏。
        self.conductivity = str(getattr(cfg, "conductivity", CONDUCTIVITY_DEFAULT)).strip()
        self._result_screen = EFFECT_SCREEN_RESULT_TPL.format(key=self.conductivity.replace(".", "_"))
        self.button = _ButtonLifecycle(self)
        self.START_BUTTON_PRESS = np.array(BTN_PRESS)     # 按下触发点（闭合指端压按钮顶 TCP 0.938）
        self.MEASURE_FRAMES = 240                         # 按下 → 读数的模拟测量时长（~4s@60fps）

        self.squeezing = False           # 挤水进行中（持续发射水滴 + 液面上涨）
        self.water_added = False         # 已挤入水（液面定高，只触发一次）
        self._water_queue = []           # 在飞水滴队列（prim/t）
        self._water_next_prim = 0        # 下一颗水滴用哪个 Drop_i（round-robin 复用池）
        self._water_spawn = 0            # 距下次发射水滴的倒计时帧
        self._water_landed = 0           # 已落定水滴数（驱动液面上涨）

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.dish_state = "rest"
        self._dish_near_frames = 0
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._set_dish_world(self._dish_rest_matrix())
        self._set_powder_from_dish()
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self._set_washbottle_world(self._washbottle_rest_matrix())
        self.rod_state = "rest"
        self._rod_near_frames = 0
        self.object_utils.set_object_position(self.ROD_PATH, np.array(ROD_REST_POS))
        self.electrode_state = "rest"
        self._electrode_near_frames = 0
        self.object_utils.set_object_position(self.ELECTRODE_PATH, np.array([0.0, 0.0, 0.0]))
        self.cable.update(Gf.Vec3d(*(self._electrode_cap_top)))
        # 挤水复位：水流父+单粒隐藏、液面隐藏 + 高度还原
        self.squeezing = False
        self.water_added = False
        self._water_queue = []
        self._water_next_prim = 0
        self._water_spawn = 0
        self._water_landed = 0
        self._set_visibility(self.WATER_STREAM, False)
        for i in range(self.WATER_DROPS):
            self._set_visibility(f"{self.WATER_STREAM}/Drop_{i}", False)
        self._set_visibility(self.LIQUID_PATH, False)
        self._set_liquid_height(self.LIQUID_H0)
        # 现象复位：皿内粉显、烧杯粉隐、下落父+粉粒隐；粉堆尺寸还原（上集 shrink 缩到 12%）
        prim = self.stage.GetPrimAtPath(POWDER_PATH)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(POWDER_BLOB_R)
            cyl.GetHeightAttr().Set(POWDER_BLOB_H)
        self._set_visibility(POWDER_PATH, True)
        self._set_visibility(BEAKER_POWDER_PATH, False)
        self._set_visibility(self.POWDER_DROP_PATH, False)
        for i in range(POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{i}", False)
        # 按钮/屏幕复位：进度条+结果屏全隐藏、按钮弹回静止位
        self.button.reset()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_dish(gripper_pos, opening)
        self._update_washbottle(gripper_pos, opening)
        self._update_rod(gripper_pos, opening)
        self._update_electrode(gripper_pos, opening)
        self._step_powder_anim()
        self._step_water_anim()
        self.button.step(gripper_pos)
        return self.get_basic_state_info(additional_info={
            "dish_state": self.dish_state,
            "poured": self.poured,
            "washbottle_state": self.washbottle_state,
            "squeezing": self.squeezing,
            "water_added": self.water_added,
            "rod_state": self.rod_state,
            "electrode_state": self.electrode_state,
            # 按钮生命周期状态（idle/measuring/result/releasing/released）：controller 据此
            # 在动作序列完成后保持 episode 存活，等仪器测量显示（进度条→结果屏→按钮回位）走完
            "button_state": self.button.state,
        })

    def on_task_complete(self, success):
        print(f"[a3] episode done success={success} dish={self.dish_state} poured={self.poured} "
              f"washbottle={self.washbottle_state} water_added={self.water_added} "
              f"rod={self.rod_state} electrode={self.electrode_state} "
              f"button_pressed={self.button.pressed} reading={self.button.reading} "
              f"conductivity={self.conductivity} mS/cm")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧皿持握（6-DOF）：rest → 近抓点+合拢 → attached（皿随 tool 6-DOF、粉堆随皿）
    #   → 倾斜倒粉（tool+Z 朝 +X 斜下）→ 松开（>0.038）→ released（皿+粉回 rest）
    # ------------------------------------------------------------------
    def _update_dish(self, gripper_pos, opening):
        if self.dish_state == "rest":
            near = self._near_grasp(gripper_pos, self.DISH_GRASP)
            self._dish_near_frames = self._dish_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_dish_to_gripper()
            if (near and self._dish_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.DISH_GRIP_CLOSED):
                self.dish_state = "attached"
                self._set_dish_from_gripper()
                print(f"[a3] dish attached (grip={opening:.4f})")
            return

        if self.dish_state == "attached":
            self._set_dish_from_gripper()
            # 倒粉：皿倾斜到位（tool+Z 朝 +X 斜下）且尚未倒 → 触发粉末下落（只一次）
            if not self.poured and not self.powder_falling and self._is_tilted():
                self.powder_falling = True
                self._start_powder_fall()
            if opening > DISH_GRIP_OPEN:   # 0.038：皿刚性触壁 opening≈0.030 不会误判松开
                self.dish_state = "released"
                self._set_dish_world(self._dish_rest_matrix())
                self._set_powder_from_dish()
                print(f"[a3] dish released to balance (grip={opening:.4f})")
        # released：已回 rest，不再跟随

    def _ease_dish_to_gripper(self, k=0.18):
        """夹爪合拢期间皿逐帧平滑拉向持握位（6-DOF：平移线性 + 旋转 slerp）。"""
        cur = self._dish_world_matrix()
        target = _T_HELD_DISH * self._tool_world()
        self._set_dish_world(self._blend_world(cur, target, k))
        self._set_powder_from_dish()

    def _set_dish_from_gripper(self):
        """皿跟随夹爪 6-DOF（皿世界 = _T_HELD_DISH · tool_world），粉堆随皿。"""
        self._set_dish_world(_T_HELD_DISH * self._tool_world())
        self._set_powder_from_dish()

    def _set_powder_from_dish(self):
        """粉堆随皿 6-DOF：粉世界 = 皿世界 · 平移(0,0,POWDER_HELD_OFFSET_Z)（皿局部 +z）。"""
        dish_world = self._dish_world_matrix()
        offset = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, POWDER_HELD_OFFSET_Z, 1.0)
        self._set_powder_world(dish_world * offset)

    # ------------------------------------------------------------------
    # 每帧洗瓶持握（纯平移，d2s 同款）：rest → 近抓点+合拢 → attached（随夹爪平移）→ released（回表位）
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
                self._T_HELD_WASHB = self._washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[a3] washbottle attached (grip={opening:.4f})")

        elif self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            # 挤水（⑥）：夹爪从持握 0.030 进一步合到 0.020 挤压瓶身 → 水流显示 + 液面上涨；
            # 松回 0.030 → 停止发射、液面定高（water_added 只触发一次，d2s SqueezeWater 同款）。
            if not self.water_added:
                if not self.squeezing and opening < self.WASH_SQUEEZE_CLOSED:
                    self.squeezing = True
                    self._water_spawn = self.WATER_STAGGER   # 下一帧立即发射首滴
                    self._set_visibility(self.WATER_STREAM, True)
                    self._set_visibility(self.LIQUID_PATH, True)
                    print(f"[a3] washbottle squeezing (grip={opening:.4f}) water stream")
                elif self.squeezing and opening >= self.WASH_SQUEEZE_CLOSED:
                    self.squeezing = False
                    self.water_added = True
                    print(f"[a3] water added to beaker (grip={opening:.4f})")
            if opening > self.WASH_GRIP_OPEN:   # 完全开爪才算松开（>0.038）
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(self._washbottle_rest_matrix())
                print(f"[a3] washbottle released to table (grip={opening:.4f})")
        # released：已回表位，不再跟随

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 _set_dish_from_gripper，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        """把洗瓶写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        self._write_world(self.washbottle_path, world_matrix)

    def _washbottle_rest_matrix(self):
        """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵，pxr 实测 2026-08-29）：
        rotateXYZ(0,0,180) + translate (0.3536,0.3062,0.80) 烘平后即下行序。
        行 0 = (-1,0,0,0) → 局部 +X 朝世界 -X；行 1 = (0,-1,0,0) → +Y 朝 -Y（红嘴尖朝 +X）。
        """
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           WASH_XY[0], WASH_XY[1], 0.80, 1.0)

    # ------------------------------------------------------------------
    # 每帧玻璃棒持握（纯平移，E1 同款）：rest → 近抓点+合拢 → attached（棒随夹爪平移，
    #   棒底 = TCP − 0.20）→ 开爪 → released（棒回架位）
    # ------------------------------------------------------------------
    def _update_rod(self, gripper_pos, opening):
        if self.rod_state == "rest":
            if self._near_grasp(gripper_pos, self.ROD_GRASP):
                self._rod_near_frames += 1
            else:
                self._rod_near_frames = 0
            if (self._rod_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.ROD_GRIP_CLOSED):
                self.rod_state = "attached"
                print(f"[a3] rod attached (grip={opening:.4f})")

        elif self.rod_state == "attached":
            # 平移跟随：棒底 = 抓点 - (0,0,ROD_TIP_TO_GRIP)（棒原点在棒底，纯平移）
            rod_pos = np.asarray(gripper_pos, dtype=float) - np.array(
                [0.0, 0.0, ROD_TIP_TO_GRIP])
            self.object_utils.set_object_position(self.ROD_PATH, rod_pos)
            if opening > self.ROD_GRIP_OPEN:   # 完全开爪才算松开（>0.038）
                self.rod_state = "released"
                self.object_utils.set_object_position(self.ROD_PATH, np.array(ROD_REST_POS))
                print(f"[a3] rod released to rack (grip={opening:.4f})")
        # released：已回架位，不再跟随

    # ------------------------------------------------------------------
    # 每帧电极持握（纯 z 平移）：rest → 近抓点+合拢 → attached（电极随夹爪竖直平移，
    #   lift_z = TCP z − cap 抓点 z）→ 开爪 → released（电极回 rest lift=0）
    # 电极 prim 无 xform op（mesh 烘焙在 meter 局部系），meter 仅 rotZ90 → 局部 +z = 世界
    # +z，竖直提 = 对电极 prim 写 (0,0,lift_z) 纯 z 平移（不写旋转）。
    # ------------------------------------------------------------------
    def _update_electrode(self, gripper_pos, opening):
        if self.electrode_state == "rest":
            if self._near_grasp(gripper_pos, self.ELECTRODE_GRASP):
                self._electrode_near_frames += 1
            else:
                self._electrode_near_frames = 0
            if (self._electrode_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.ELECTRODE_GRIP_CLOSED):
                self.electrode_state = "attached"
                print(f"[a3] electrode attached (grip={opening:.4f})")

        elif self.electrode_state == "attached":
            # 3-DOF 平移跟随：世界位移 (wx,wy,wz) = gripper − 抓点；电极 prim 局部（meter 局部，
            # rotZ90）写 (wy,−wx,wz)（rotZ90⁻¹：local_x=world_y, local_y=−world_x）。attach 时
            # gripper=抓点 → (0,0,0) 零跳变。
            ex = float(self.ELECTRODE_GRASP[0])
            ey = float(self.ELECTRODE_GRASP[1])
            wx = float(gripper_pos[0]) - ex
            wy = float(gripper_pos[1]) - ey
            wz = float(gripper_pos[2]) - self.ELECTRODE_GRASP[2]
            self.object_utils.set_object_position(self.ELECTRODE_PATH,
                                                  np.array([wy, -wx, wz]))
            # 电缆跟随：移动端 = cap 顶世界位 = gripper + (0,0,_cap_top_dz)
            cap_top = np.array([float(gripper_pos[0]), float(gripper_pos[1]),
                                float(gripper_pos[2]) + self._cap_top_dz])
            self.cable.update(Gf.Vec3d(*cap_top))
            if opening > self.ELECTRODE_GRIP_OPEN:   # 完全开爪才算松开（>0.038）
                self.electrode_state = "released"
                # 松爪后电极**留在烧杯内**（不再回 meter）：不重置位置、电缆停在当前形状
                # （reset() 下一 episode 才把电极 + 电缆整体回 rest）。
                print(f"[a3] electrode released into beaker (grip={opening:.4f})")
        # released：电极 + 电缆冻结在松爪瞬间的位姿（浸在烧杯液体内），不再跟随

    # ------------------------------------------------------------------
    # 挤水水流（⑥，仿 d2s SqueezeWater）：挤水期间每 WATER_STAGGER 帧发射一颗水滴，
    # 沿抛物线（x/y 线性、z t² 重力加速）从红嘴尖（随瓶动态算）坠入烧杯口中心；
    # 松爪后停止发射、让在飞水滴落完再隐藏父节点。水滴池 round-robin 复用：复用周期
    # = 池大小×发射间隔 ≫ 单滴坠落帧数，无同 prim 碰撞。烧杯内液面随落定水滴数上涨。
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
            if self._water_spawn >= self.WATER_STAGGER:
                self._water_spawn = 0
                idx = self._water_next_prim % self.WATER_DROPS
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
            if d["t"] >= self.WATER_FALL:
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{d['prim']}", False)
                self._water_landed += 1
                continue
            frac = d["t"] / self.WATER_FALL
            x = start[0] + (target[0] - start[0]) * frac
            y = start[1] + (target[1] - start[1]) * frac
            z = start[2] - (start[2] - target[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{self.WATER_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._water_queue = remaining
        # 液面随落定水滴数上涨（落满 WATER_LAND_FULL 滴 → 最终液面高度）
        if self._water_landed > 0:
            frac = min(1.0, self._water_landed / self.WATER_LAND_FULL)
            h = max(self.LIQUID_H0, self.LIQUID_H_MAX * frac)
            self._set_liquid_height(h)
        if not remaining and not self.squeezing:
            self._set_visibility(self.WATER_STREAM, False)

    def _set_liquid_height(self, h):
        """烧杯内液面圆柱：更新半径/高度 + 中心 z（底贴烧杯内底 TABLE_Z，随 h 上移）。"""
        prim = self.stage.GetPrimAtPath(self.LIQUID_PATH)
        if not prim.IsValid():
            return
        cyl = UsdGeom.Cylinder(prim)
        cyl.GetRadiusAttr().Set(self.LIQUID_R)
        cyl.GetHeightAttr().Set(h)
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpName() != "xformOp:translate":
                continue
            v = op.Get()
            op.Set(Gf.Vec3d(v[0], v[1], self.TABLE_Z + h / 2.0))
            return

    # ------------------------------------------------------------------
    # 位姿工具（6-DOF：读 tool_world / 写 world 矩阵到 prim，d2s 同款）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dish_world_matrix(self):
        prim = self.stage.GetPrimAtPath(self.DISH)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dish_rest_matrix(self):
        """皿静止位姿（秤盘上平放，+z 朝上，无旋转）。"""
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           DISH_XY[0], DISH_XY[1], DISH_ORIG_REST_Z, 1.0)

    def _set_dish_world(self, world_matrix):
        self._write_world(self.DISH, world_matrix)

    def _set_powder_world(self, world_matrix):
        self._write_world(POWDER_PATH, world_matrix)

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

    def _blend_world(self, a, b, k):
        """两个世界位姿的刚性插值（平移线性 + 旋转 slerp）。"""
        qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
        qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
        m = Gf.Matrix4d()
        m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
        m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
        return m

    # ------------------------------------------------------------------
    # 倒粉判定 + 粉末下落动画（仿 d2s PowderDrop：粉粒从皿低侧滑出坠入烧杯口）
    # ------------------------------------------------------------------
    def _is_tilted(self):
        """皿是否已倾斜到位：tool+Z（手指方向）从朝下转到朝 +X 斜下，x 分量超阈值。"""
        wm = self._tool_world()
        z_dir = np.array([wm[2][0], wm[2][1], wm[2][2]])   # 行向量约定 tool+Z = 第 2 行
        return z_dir[0] > self.TILT_X_THRESH

    def _dish_mouth_low_pos(self):
        """皿倾斜后低侧（-X）口沿世界位置 = 皿中心 + tool+X 方向 × 皿半径（粉末滑出起点）。

        pxr 已验证：TILT_ORIENT 让 tool+Z→(0.866,0,-0.5)，皿上法线 dish+z = -tool+z =
        (-0.866,0,0.5) 朝 -X 斜上 → 皿 -X 侧下降、粉末沿 -X 滑出。tool+x（行 0）=(-0.5,0,-0.866)
        恰好指向皿 -x 低侧（dish+x = -tool+x），故用 tool+x 方向 × 皿半径取低侧口沿。
        """
        wm = self._tool_world()
        x_dir = np.array([wm[0][0], wm[0][1], wm[0][2]])   # tool+X 世界方向（行向量第 0 行）= 皿 -x 低侧
        dish_center = np.array(self._dish_world_matrix().ExtractTranslation())
        # 皿中心 = 皿原点 + 皿 +z（= tool -z）方向 × 0.00335
        z_dir = np.array([wm[2][0], wm[2][1], wm[2][2]])
        center = dish_center + (-z_dir) * 0.00335
        return center + x_dir * 0.03   # 皿半径 30mm

    def _start_powder_fall(self):
        """倒粉起：皿低侧口沿正下方生成一串粉粒，delay 错帧坠入烧杯口（仿 d2s）。"""
        # 先清掉上一集残留的可见粉粒（父隐藏 ≠ 单粒 visibility 复位）
        for i in range(POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{i}", False)
        start = self._dish_mouth_low_pos() + np.array([0.0, 0.0, -0.005])
        target = np.array(POWDER_LAND, dtype=float)
        for i in range(POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * POWDER_STAGGER,
                "t": 0,
                "start": start.copy(),
                "target": target.copy(),
            })
        self._set_visibility(self.POWDER_DROP_PATH, True)   # 父显，单粒才渲染（visibility 与父 AND）
        print(f"[a3] powder fall started from {np.round(start, 3)}")

    def _step_powder_anim(self):
        """推进下落串：delay 错帧 → 悬停 → 加速坠落 → 落定隐藏（皿上粉堆随落定缩小）；
        全部落定 → 烧杯显粉。"""
        if not self._powder_queue:
            return
        remaining = []
        landed = POWDER_DROPS - len(self._powder_queue)   # 已落定粒数（本帧循环内递增）
        for d in self._powder_queue:
            if d["delay"] > 0:
                d["delay"] -= 1
                remaining.append(d)
                continue
            d["t"] += 1
            if d["t"] <= POWDER_HANG:
                pos = d["start"]
            elif d["t"] <= POWDER_HANG + POWDER_FALL:
                frac = (d["t"] - POWDER_HANG) / POWDER_FALL
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", False)
                landed += 1
                continue
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._powder_queue = remaining
        self._shrink_powder_blob(landed / POWDER_DROPS)
        if not remaining:
            self._set_visibility(self.POWDER_DROP_PATH, False)   # 落完收父隐藏
            if self.powder_falling:
                self.powder_falling = False
                self.poured = True
                # 皿里粉堆隐藏、烧杯内显粉
                self._set_visibility(POWDER_PATH, False)
                self._set_visibility(BEAKER_POWDER_PATH, True)
                print("[a3] powder poured into beaker")

    def _shrink_powder_blob(self, landed_frac):
        """皿上粉堆随下落进度缩小（粉粒落定越多、皿上剩得越少，避免整块粉堆闪现消失）。"""
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(POWDER_PATH)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(POWDER_BLOB_R * remain)
            cyl.GetHeightAttr().Set(POWDER_BLOB_H * remain)

    POWDER_DROP_PATH = "/World/PowderDrop"

    # ------------------------------------------------------------------
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
            from isaacsim.core.utils.prims import set_prim_visibility
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass

    def _set_button_z(self, z):
        """写按钮 /World/Meter/start_button 的 translate z（按下下沉/弹回）。只改 translate 的
        z 分量，保留 x=0.08、y=0.08（Meter 局部系）。"""
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                v = op.Get()
                op.Set(Gf.Vec3d(v[0], v[1], float(z)))
                return

    def _get_button_z(self):
        """读按钮 translate z（弹回动画驱动，从当前位逐帧上抬）。找不到 op 时回退静止位。"""
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return BUTTON_REST_Z
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                return float(op.Get()[2])
        return BUTTON_REST_Z
