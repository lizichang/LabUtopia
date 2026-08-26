"""A1 折光率测量任务：取瓶塞（直拔）+ 取滴管吸样→滴样到棱镜 + 液体效果。

与 d4l 同构（持握照 flametest v24-v46 已验证）：瓶塞与滴管是静态碰撞体，吸附期逐帧把
**世界位置**写为 TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform
op 表）——滴管全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方）；瓶塞抓近顶、
直拔离瓶口后倒放桌面。

A1 相对 d4l 的差异：
  ① 滴样目标是**折光仪棱镜面**（/World/PrismDrop 液滴落在棱镜顶 0.9175），非试管口。
  ② 样品瓶白塞由**机械臂直拔**（取瓶塞 = 第一个动作，无取瓶/无旋转开盖），
     d4l 碱瓶塞是 gen 静态倒放、无机械臂动作。
  ③ 瓶塞是瓶的子 prim（/World/SampleBottle/stopper），读写世界位置需补偿父平移
     与局部几何中心偏移（同 flametest 子物体约定）。

生命周期：
  瓶塞 _CapLifecycle：rest（瓶口）→ attached（跟随）→ released（倒放桌面 CAP_DESK）。
  滴管 _DropperLifecycle：rest → attached → squeezed（瓶口挤空气）→ filled（浸液吸液，
  DropperFill 显）→ dropped（棱镜上方挤胶头，液滴坠落到棱镜 + PrismDrop 显）→ released
  （回架松开 → rest）。
  测量键 _ButtonLifecycle：idle → measuring（按下，ScreenMeasuring 红进度条）
  → result（MEASURE_FRAMES 帧后，ScreenGlow_<key> 显示 cfg.n_d 档位读数 + 20.0°C 绿满条，
  读数定格）→ releasing（爪子抬离后按钮缓慢弹回）→ released（按钮回位，读数保持）。

controller 顺序：①PICK_CAP_PASS（取瓶塞直拔倒放桌面）→ ②SAMPLE_PASS（取滴管吸样→
滴样到棱镜）。
"""
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    CAP_PATH, CAP_PARENT_T, CAP_LOCAL_OFFSET, CAP_GRASP, CAP_REST, CAP_HELD_OFFSET, CAP_DESK,
    DROP_PATH, DROP_REST, DROP_GRASP,
    BOTTLE_XY, PRISM_XY, PRISM_TOP_Z,
    COVER_PATH, COVER_HINGE_XY, COVER_OPEN_ANGLE, COVER_CLOSED_ANGLE,
    COVER_PUSH_TRIGGER_Y, COVER_PUSH_Z_MIN, COVER_PUSH_Z_MAX,
    START_BUTTON_PRESS_TCP, BUTTON_PATH, BUTTON_REST_Z, BUTTON_PRESSED_Z,
    BUTTON_LIFT_Z, BUTTON_SPRING_STEP,
    EFFECT_PRISM_DROP, EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
    EFFECT_SCREEN_MEASURING_TPL, EFFECT_SCREEN_RESULT_TPL, N_D_DEFAULT, PROGRESS_STEPS,
)

# 滴管相对夹爪的持握偏移（flametest/d4l 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])


class _CapLifecycle:
    """瓶塞状态机（rest/attached/released）。瓶塞是瓶的子 prim，读写世界几何中心需
    补偿父平移 CAP_PARENT_T 与局部几何中心偏移 CAP_LOCAL_OFFSET（flametest 同款）。"""

    def __init__(self, task):
        self.task = task
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.released = False
        self.task._set_cap_world(np.array(CAP_REST, dtype=float))

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(np.array(CAP_GRASP), gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + CAP_HELD_OFFSET
            # 夹爪开始合拢且已进近窗：先把瓶塞平滑拉向持握位（消闪现吸附）。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_cap_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_cap_world(held)
                print(f"[a1] cap attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：逐帧跟随夹爪（纯平移）
            self.task._set_cap_world(gripper_pos + CAP_HELD_OFFSET)
            if opening > self.task.gripper_open_threshold:
                self.state = "released"
                self.released = True
                self.task._set_cap_world(np.array(CAP_DESK, dtype=float))
                print(f"[a1] cap released to desk (grip={opening:.4f})")
            return

        # released：已倒放桌面，不再跟随夹爪。
        # 旧 bug：released 态仍走上面「吸附期逐帧跟随」，把瓶塞每帧拉回爪子中间。
        return


class _DropperLifecycle:
    """滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp      架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy  瓶口 xy（排空气/浸液区，z 不区分）
      prism_xy   棱镜 xy（滴样区）
    """

    def __init__(self, task):
        self.task = task
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
        self.task._set_obj_world(DROP_PATH, np.array(DROP_REST, dtype=float))
        self.task._set_visibility(EFFECT_DROPPER_FILL, False)

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(np.array(DROP_GRASP), gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + HELD_OFFSET
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(DROP_PATH, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(DROP_PATH, held)
                print(f"[a1] dropper attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(DROP_PATH, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(np.array(BOTTLE_XY), gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print("[a1] dropper squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(np.array(BOTTLE_XY), gripper_pos)):
                self.state = "filled"
                self.filled = True
                self.task._set_visibility(EFFECT_DROPPER_FILL, True)
                print("[a1] dropper filled (aspirated)")
        elif self.state == "filled":
            # 液柱跟随尖嘴
            self.task._set_fill_follow()
            # 棱镜区挤胶头滴样
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(np.array(PRISM_XY), gripper_pos)):
                self.state = "dropped"
                self.dropped = True
                self.task._set_visibility(EFFECT_DROPPER_FILL, False)
                self.task._on_drop()
                print("[a1] dropper dropped onto prism")
        elif self.state == "dropped":
            # 回架松开：写回竖插位姿并复位 rest
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(np.array(DROP_GRASP), gripper_pos)):
                self.released = True
                self.task._set_obj_world(DROP_PATH, np.array(DROP_REST, dtype=float))
                self.state = "rest"
                print("[a1] dropper released to rack -> rest")


class _CoverLifecycle:
    """盖子状态机（open/closing/closed）。合盖 = 爪子从 +y 侧往 -y 推圆盘前缘，
    爪子 y 越过 COVER_PUSH_TRIGGER_Y（前缘 0.1075 稍外）→ 触发 rotateX 从 -55 平滑转 0
    = 自动合平。盖无碰撞，task 检测爪子位置触发动画（非物理推动）。"""

    CLOSE_STEP = 2.5        # 每帧转 2.5°（-55→0 共 22 帧 ≈ 0.37s）

    def __init__(self, task):
        self.task = task
        self.state = "open"
        self.angle = COVER_OPEN_ANGLE
        self.closed = False

    def reset(self):
        self.state = "open"
        self.angle = COVER_OPEN_ANGLE
        self.closed = False
        self.task._set_cover_angle(COVER_OPEN_ANGLE)

    def step(self, gripper_pos):
        if self.state == "open":
            if (abs(gripper_pos[0] - COVER_HINGE_XY[0]) < 0.03
                    and gripper_pos[1] < COVER_PUSH_TRIGGER_Y
                    and COVER_PUSH_Z_MIN < gripper_pos[2] < COVER_PUSH_Z_MAX):
                self.state = "closing"
                print("[a1] cover closing triggered (gripper pushed -y over front edge)")
            return

        if self.state == "closing":
            self.angle += self.CLOSE_STEP
            if self.angle >= COVER_CLOSED_ANGLE:
                self.angle = COVER_CLOSED_ANGLE
                self.state = "closed"
                self.closed = True
                print("[a1] cover closed flat over well")
            self.task._set_cover_angle(self.angle)
            return

        # closed：已合平，不再动


class _ButtonLifecycle:
    """测量键状态（idle/measuring/result/releasing/released）。按下 = 爪子垂直下探到按钮顶
    (0.30,0.05,0.921)，检测爪子近按钮顶 → 按钮下沉到 BUTTON_PRESSED_Z + 显示测量进度条
    （ScreenMeasuring_<i> 预烘焙多帧，红条 0%→100%，MEASURE_FRAMES≈4s 内逐帧切显）→ 走完
    切 result 读数屏 ScreenGlow_<key>（绿满条 + cfg.n_d 档位读数，读数定格）→ 爪子抬离
    > BUTTON_LIFT_Z → 按钮缓慢弹回 BUTTON_REST_Z（releasing → released，读数保持显示）。
    按钮无碰撞，非物理下压。"""

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
        """隐藏当前进度条帧（测量结束切结果/复位时用）。"""
        if self._step_shown >= 0:
            self.task._set_visibility(
                EFFECT_SCREEN_MEASURING_TPL.format(step=self._step_shown), False)
            self._step_shown = -1

    def step(self, gripper_pos):
        if self.state == "idle":
            if self.task._near(np.array(START_BUTTON_PRESS_TCP), gripper_pos,
                               z_thresh=0.012):
                self.state = "measuring"
                self.pressed = True
                self._step_shown = 0
                self.task._set_visibility(EFFECT_SCREEN_MEASURING_TPL.format(step=0), True)
                self.task._set_button_z(BUTTON_PRESSED_Z)   # 按钮下沉（按下效果）
                print("[a1] start button pressed (measuring… red progress bar 0%)")
        elif self.state == "measuring":
            self._frames += 1
            # 进度条动画：按进度切到对应预烘焙帧（红条 0%→100%，~4s 走完）
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
                self.task._set_visibility(self.task._result_screen, True)  # 完成：nD 读数
                print(f"[a1] measurement done -> nD {self.task.n_d} shown (green bar)")
        elif self.state == "result":
            # 爪子抬离按钮（z > BUTTON_LIFT_Z）→ 按钮缓慢弹回（读数保持显示）
            if gripper_pos[2] > BUTTON_LIFT_Z:
                self.state = "releasing"
                print("[a1] button releasing (gripper lifted, slow spring back)")
        elif self.state == "releasing":
            # 每帧上抬 BUTTON_SPRING_STEP，到静止位复位
            z = self.task._get_button_z()
            z = min(z + BUTTON_SPRING_STEP, BUTTON_REST_Z)
            self.task._set_button_z(z)
            if z >= BUTTON_REST_Z:
                self.state = "released"
                print("[a1] button back to rest")
        # released：按钮回位，读数保持定格


class A1RefractometerTask(BaseTask):
    """A1 折光率测量任务：取瓶塞 + 取滴管吸样→滴样到棱镜 + 液体效果 prim + 合盖 + 按测量键。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内液柱 60mm 很满，一挤该是一串滴不是一滴——用户 2026-08-14）。每滴
    # delay 错帧起落 → 悬停成形 → 加速坠落，落定后显示 PrismDrop（棱镜面液滴）。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.02m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）
    DROP_RADIUS = 0.003  # 液滴球半径（与 gen DROP_BALL_R 一致）
    MEASURE_FRAMES = 240  # 按下测量键 → nD 读数的模拟测量时长（帧，240≈4s@60fps。
    # 用户 2026-08-26：先显示进度条，~4s 走完最后显示结果；进度条按 PROGRESS_STEPS 预烘焙逐帧切显）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 瓶塞/滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(CAP_PATH)
        self._disable_collision(DROP_PATH)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        # 折光率读数（experiment_result 框架 --result n_d=<> / TTY 交互写回 cfg.n_d，default 1.4000）
        # → 选对应预烘焙读数屏 ScreenGlow_<key>（headless 下运行时改材质不渲染 → 按档位预烘焙）
        self.n_d = str(getattr(cfg, "n_d", N_D_DEFAULT)).strip()
        self._result_screen = EFFECT_SCREEN_RESULT_TPL.format(key=self.n_d.replace(".", "_"))

        # 瓶塞 / 滴管 / 盖子 / 测量键各自的生命周期句柄（参考点已 pxr 实测）
        self.cap = _CapLifecycle(self)
        self.dropper = _DropperLifecycle(self)
        self.cover = _CoverLifecycle(self)
        self.button = _ButtonLifecycle(self)

        # 滴落动画状态
        self._drop_queue = []
        self._prism_drop_shown = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._drop_queue = []
        self._prism_drop_shown = False
        self.cap.reset()
        self.dropper.reset()
        self.cover.reset()
        self.button.reset()
        self._set_visibility(EFFECT_PRISM_DROP, False)
        self._set_visibility(EFFECT_DROPPER_FILL, False)
        self._set_visibility(EFFECT_DROPPER_DROP, False)

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
        self.cap.step(gripper_pos, opening)
        self.dropper.step(gripper_pos, opening)
        self.cover.step(gripper_pos)
        self.button.step(gripper_pos)
        return self.get_basic_state_info(additional_info={
            "cap_attached": self.cap.attached,
            "cap_released": self.cap.released,
            "dropper_attached": self.dropper.attached,
            "dropper_filled": self.dropper.filled,
            "dropper_dropped": self.dropper.dropped,
            "cover_closed": self.cover.closed,
            "button_pressed": self.button.pressed,
            "reading": self.button.reading,
            # 按钮生命周期状态（idle/measuring/result/releasing/released）：controller 据此
            # 在动作序列完成后保持 episode 存活，等仪器测量显示（进度条→结果屏→按钮回位）走完
            "button_state": self.button.state,
        })

    def on_task_complete(self, success):
        print(f"[a1] episode done success={success} "
              f"cap_released={self.cap.released} "
              f"dropper_dropped={self.dropper.dropped} "
              f"cover_closed={self.cover.closed} "
              f"button_pressed={self.button.pressed} "
              f"reading={self.button.reading}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 位姿读写（滴管顶层 + 瓶塞子 prim）
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保竖立姿态）。

        flametest/d4l 同款：不 ClearXformOpOrder、不写 4x4 矩阵——烘平场景里顶层 prim
        （滴管）只有 xformOp:translate 一个 op，set_object_position 改首 op 即平移。
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

    def _get_cap_world(self):
        """瓶塞几何中心世界坐标（prim 原点 + 局部几何中心偏移）。"""
        origin = self._get_obj_world(CAP_PATH)
        if origin is None:
            return np.array(CAP_REST, dtype=float)
        return np.asarray(origin, dtype=float) + CAP_LOCAL_OFFSET

    def _set_cap_world(self, world):
        """把瓶塞几何中心写到世界位置（子 prim：局部 = 世界 - 父平移 - 局部偏移）。"""
        local_t = np.asarray(world, dtype=float) - CAP_PARENT_T - CAP_LOCAL_OFFSET
        self._set_obj_world(CAP_PATH, local_t)

    def _ease_cap_world(self, target, k=0.18):
        cur = self._get_cap_world()
        nxt = cur + (np.asarray(target, dtype=float) - cur) * k
        self._set_cap_world(nxt)

    # ------------------------------------------------------------------
    # 效果
    # ------------------------------------------------------------------
    def _set_fill_follow(self):
        """DropperFill 截锥液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴）。尖嘴在夹爪
        下 0.13m（保竖立），液柱从尖嘴向上 60mm（几何见 gen 脚本）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(EFFECT_DROPPER_FILL, tip)

    def _on_drop(self):
        """挤胶头滴样：在尖嘴正下方生成一串液滴（DropperDrop 父 Xform 的 Drop_0.._N 球），
        delay 错帧起落形成连续「滴-滴-滴」。每滴落到棱镜顶；全部落定后显示 PrismDrop。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（棱镜顶上方 25mm，可见坠落）
        target = np.array([PRISM_XY[0], PRISM_XY[1], PRISM_TOP_Z + self.DROP_RADIUS])
        for i in range(self.DROPS_PER_SQUEEZE):
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(), "target": target,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._set_visibility(EFFECT_DROPPER_DROP, True)
        print(f"[a1] squeeze -> {self.DROPS_PER_SQUEEZE} drops spawned onto prism")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该球）；全部落定后
        显示 PrismDrop（棱镜面液滴）。"""
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
                # 落定：隐藏这颗，移出队列
                self._set_visibility(f"{EFFECT_DROPPER_DROP}/Drop_{d['idx']}", False)
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{EFFECT_DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{EFFECT_DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(EFFECT_DROPPER_DROP, False)
            # 全部液滴落定 → 棱镜面显示液滴（样品就位）
            if not self._prism_drop_shown:
                self._set_visibility(EFFECT_PRISM_DROP, True)
                self._prism_drop_shown = True
                print("[a1] prism drop shown (sample on prism)")

    # ------------------------------------------------------------------
    # 判定 / 辅助
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

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

    def _set_cover_angle(self, angle):
        """写盖子 /World/Refractometer/Cover 的 xformOp:rotateX（合盖动画驱动）。
        只改 rotateX op，保留 translate（铰链位）与板子其他 op。"""
        prim = self.stage.GetPrimAtPath(COVER_PATH)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if "rotateX" in op.GetName():
                op.Set(float(angle))
                return

    def _set_button_z(self, z):
        """写按钮 /World/Refractometer/start_button 的 translate z（按下下沉/弹回）。
        只改 translate 的 z 分量，保留 x=0、y=0.05。"""
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                v = op.Get()
                op.Set(Gf.Vec3d(v[0], v[1], float(z)))
                return

    def _get_button_z(self):
        """读按钮 /World/Refractometer/start_button 的 translate z（弹回动画驱动，
        从当前位逐帧上抬）。找不到 op 时回退静止位。"""
        prim = self.stage.GetPrimAtPath(BUTTON_PATH)
        if not prim.IsValid():
            return BUTTON_REST_Z
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetName() == "xformOp:translate":
                return float(op.Get()[2])
        return BUTTON_REST_Z
