"""D6 试纸气体检测（通用）任务：试纸夹预夹好试纸，机械臂只润湿 + 移试管 + 观察。

2026-08-26 用户重新设计：专用试纸夹（test_paper_holder.usd）预夹好试纸，机械臂**不碰试纸**，
只做「取蒸馏水滴管润湿试纸 → 取反应试管移到试纸下方 → 保持观察变色 → 试管归位」。
本 task 只做 4 类驱动（与 e2 同构，纯平移持握 + visibility 变体切换）：

  1. 滴管生命周期：rest → attached（抓胶头）→ 挤胶头润湿（squeeze，触发水滴坠落动画）→
     松回架 released。持握 = 纯平移（滴管 translate=底/尖嘴 = tool_center + (0,0,-0.13)）。
  2. 试管生命周期：rest → attached（夹管身）→ released。持握 = 纯平移（管底 =
     tool_center + (0,0,-0.139)），管内液体（TubeSolution 父 prim）逐帧随管平移。
  3. 试纸变色：试纸 4 变体（oxidative/alkaline × blue/negative）预制，task 按 cfg.gas_result
     在检测时（试管到位）切换 visibility——检测前显示对应 paper_type 的 negative 变体
     （未反应基色），检出变蓝就切到 gas_result 变体（湿润端变蓝，头部底色不变）。
  4. 试管内液体颜色：6 色变体预制，task 按 cfg.liquid_color 显示其一，父 prim 随管平移。
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    DROPPER_XY, DROPPER_BOTTOM_Z, DROPPER_GRASP, DROPPER_HELD_OFFSET,
    TUBE_XY, TUBE_BOTTOM_Z, TUBE_GRASP, TUBE_HELD_OFFSET,
    PAPER_WET_XY, PAPER_Z,
)

# 滴管/试管持握偏移（纯平移，保竖立；同 d3l）。抓点 = 立放位 + (0,0,0.13/0.139)，
# 故 translate(=底/尖嘴) = tool_center - 0.13 / -0.139，抓点处=rest 位零跳变。
DROPPER_ORIG = np.array([DROPPER_XY[0], DROPPER_XY[1], DROPPER_BOTTOM_Z])
TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_BOTTOM_Z])

# 试纸 4 变体 = 试纸类型（oxidative 淀粉碘化钾/alkaline 红石蕊）× 是否变蓝
PAPER_VARIANTS = ["oxidative_blue", "oxidative_negative",
                  "alkaline_blue", "alkaline_negative"]
PAPER_TYPE = {"oxidative_blue": "oxidative", "oxidative_negative": "oxidative",
              "alkaline_blue": "alkaline", "alkaline_negative": "alkaline"}
LIQUID_COLORS = ["colorless", "blue", "red", "green", "yellow", "purple"]


class _DropperLifecycle:
    """蒸馏水滴管状态机：rest → attached（抓胶头）→ 挤胶头润湿 → released（回架松开）。

    挤胶头判定：opening < squeeze_threshold 且在试纸湿润端 xy 区（润湿点）→ 一次性触发
    润湿动画（水滴坠落，由 task._spawn_wet_drops 驱动）。挤完松回 attached（opening 回升
    但 > squeeze_threshold），回到架内再松开（> open_threshold）。
    """

    def __init__(self, task):
        self.task = task
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.wet_done = False   # 本次持握是否已润湿（一次性）

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.released = self.wet_done = False
        self.task._set_obj_world(self.task.DROPPER, DROPPER_ORIG)

    def step(self, gripper_pos, opening):
        grasp = np.array(DROPPER_GRASP)
        if self.state == "rest":
            near = self.task._near(grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + np.array(DROPPER_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.task.DROPPER, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.task.DROPPER, held)
                print(f"[d6] dropper attached (grip={opening:.4f})")
            return

        if self.state != "attached":
            return   # released：已放回，不再跟随

        # 吸附期：逐帧跟随（纯平移保竖立）
        held = gripper_pos + np.array(DROPPER_HELD_OFFSET)
        self.task._set_obj_world(self.task.DROPPER, held)
        # 挤胶头润湿（试纸湿润端 xy 区 + 开度压到 squeeze 阈值）→ 一次性触发水滴动画
        if (opening < self.task.gripper_squeezed_threshold
                and self.task._near_xy(PAPER_WET_XY, gripper_pos)
                and not self.wet_done):
            self.wet_done = True
            self.task._spawn_wet_drops()
            print(f"[d6] dropper squeezed -> wet paper (grip={opening:.4f})")
        # 回架内松开
        if (opening > self.task.gripper_open_threshold
                and self.task._near(grasp, gripper_pos)):
            self.state = "released"
            self.released = True
            self.task._set_obj_world(self.task.DROPPER, DROPPER_ORIG)
            print("[d6] dropper released to rack")


class _TubeLifecycle:
    """反应试管状态机：rest → attached（夹管身，液体随管平移）→ released（回架松开）。"""

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
        self.task._set_obj_world(self.task.TUBE, TUBE_ORIG)
        self.task._set_obj_world(self.task.TUBE_SOLUTION, TUBE_ORIG)

    def step(self, gripper_pos, opening):
        grasp = np.array(TUBE_GRASP)
        if self.state == "rest":
            near = self.task._near(grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + np.array(TUBE_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.task.TUBE, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.task.TUBE, held)
                self.task._set_obj_world(self.task.TUBE_SOLUTION, held)
                print(f"[d6] tube attached (grip={opening:.4f})")
            return

        if self.state != "attached":
            return   # released：已放回，不再跟随

        # 吸附期：试管 + 管内液体逐帧跟随（纯平移保竖立）
        held = gripper_pos + np.array(TUBE_HELD_OFFSET)
        self.task._set_obj_world(self.task.TUBE, held)
        self.task._set_obj_world(self.task.TUBE_SOLUTION, held)
        # 回架内松开
        if (opening > self.task.gripper_open_threshold
                and self.task._near(grasp, gripper_pos)):
            self.state = "released"
            self.released = True
            self.task._set_obj_world(self.task.TUBE, TUBE_ORIG)
            self.task._set_obj_world(self.task.TUBE_SOLUTION, TUBE_ORIG)
            print("[d6] tube released to rack")


class D6TestpaperGasTask(BaseTask):
    """D6 试纸气体检测任务：润湿试纸 + 移试管观察变色（试纸预夹，不碰试纸）。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    DROPPER = "/World/Dropper"
    TUBE = "/World/TestTube"
    TUBE_SOLUTION = "/World/TubeSolution"
    DROPPER_DROP = "/World/DropperDrop"
    TESTPAPER = "/World/TestPaper"

    # 润湿水滴坠落动画（试纸湿润端上方 2cm，滴管尖 1.01 → 试纸 0.99）
    WET_DROPS = 2
    WET_HANG = 4        # 每滴在尖嘴悬停成形帧数
    WET_FALL = 10       # 每滴加速坠落帧数（2cm）
    WET_STAGGER = 5     # 相邻两滴起落间隔帧数

    # 检测变色：试管到位（湿润端正下方）后 DETECT_DELAY 帧切换试纸 visibility
    # （气体上升需片刻；检出变蓝则湿润端变蓝，未检出保持基色）。
    DETECT_DELAY = 30
    DETECT_XY_THRESH = 0.03

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管/试管是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER)
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        # 试纸检测结果（main.py 已按 experiment_result schema 把 CLI/交互结果写回 cfg.gas_result）
        self.gas_result = str(getattr(cfg, "gas_result", "oxidative_blue")).strip().lower()
        if self.gas_result not in PAPER_VARIANTS:
            self.gas_result = "oxidative_blue"
        self._paper_before = f"{PAPER_TYPE[self.gas_result]}_negative"   # 未反应基色变体

        # 试管内液体颜色（写回 cfg.liquid_color）
        self.liquid_color = str(getattr(cfg, "liquid_color", "blue")).strip().lower()
        if self.liquid_color not in LIQUID_COLORS:
            self.liquid_color = "blue"

        self.dropper = _DropperLifecycle(self)
        self.tube = _TubeLifecycle(self)

        # 润湿水滴动画队列 + 检测变色状态
        self._wet_queue = []
        self._detect_frames = 0
        self._paper_swapped = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.dropper.reset()
        self.tube.reset()
        # 试纸：检测前显示未反应基色变体（paper_type 的 negative）
        self._show_paper(self._paper_before)
        # 液体：显示 cfg.liquid_color 变体，父贴管底 rest 位
        self._show_liquid(self.liquid_color)
        self._set_obj_world(self.TUBE_SOLUTION, TUBE_ORIG)
        # 水滴动画复位
        self._wet_queue = []
        self._detect_frames = 0
        self._paper_swapped = False
        self._set_visibility(self.DROPPER_DROP, False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._step_wet_anim()                       # 水滴坠落动画独立推进
        self.dropper.step(gripper_pos, opening)
        self.tube.step(gripper_pos, opening)
        self._step_detect(gripper_pos)              # 试管到位 → 试纸变色
        return self.get_basic_state_info(additional_info={
            "dropper_attached": self.dropper.attached,
            "dropper_released": self.dropper.released,
            "dropper_wet": self.dropper.wet_done,
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[d6] episode done success={success} "
              f"dropper_wet={self.dropper.wet_done} "
              f"tube_released={self.tube.released} "
              f"gas_result={self.gas_result} liquid_color={self.liquid_color}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 持握 / 判定
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
        self._set_obj_world(path, cur + (np.asarray(target) - cur) * k)

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - np.asarray(center_xy)) < self.grasp_xy_threshold

    # ------------------------------------------------------------------
    # 润湿水滴动画
    # ------------------------------------------------------------------
    def _spawn_wet_drops(self):
        """挤胶头瞬间在滴管尖正下方生成 WET_DROPS 滴蒸馏水，错帧坠到试纸湿润端。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) \
            + np.array(DROPPER_HELD_OFFSET)
        start = tip + np.array([0.0, 0.0, -0.003])   # 尖嘴正下方（试纸上方 2cm）
        target = np.array([PAPER_WET_XY[0], PAPER_WET_XY[1], PAPER_Z + 0.0005])
        for i in range(self.WET_DROPS):
            self._wet_queue.append({
                "idx": i,
                "delay": i * self.WET_STAGGER,
                "t": 0,
                "start": start.copy(), "target": target,
                "hang": self.WET_HANG, "fall": self.WET_FALL,
            })
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[d6] wet -> {self.WET_DROPS} drops spawned")

    def _step_wet_anim(self):
        if not self._wet_queue:
            return
        remaining = []
        for d in self._wet_queue:
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
                self._set_visibility(f"{self.DROPPER_DROP}/drop_{d['idx']}", False)
                continue
            self._set_visibility(f"{self.DROPPER_DROP}/drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/drop_{d['idx']}", pos)
            remaining.append(d)
        self._wet_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    # ------------------------------------------------------------------
    # 试纸变色 + 液体变体切换
    # ------------------------------------------------------------------
    def _step_detect(self, gripper_pos):
        """试管到位（湿润端正下方）后连续 DETECT_DELAY 帧 → 试纸切换到最终 gas_result
        变体（检出变蓝：湿润端由基色变蓝；未检出：保持基色不变）。已换过则不再重复。"""
        if self._paper_swapped:
            return
        gp = np.asarray(gripper_pos, dtype=float)
        under_paper = (self.tube.attached
                       and np.linalg.norm(gp[:2] - np.asarray(PAPER_WET_XY))
                       < self.DETECT_XY_THRESH)
        if under_paper:
            self._detect_frames += 1
        else:
            self._detect_frames = 0
        if self._detect_frames >= self.DETECT_DELAY:
            self._show_paper(self.gas_result)
            self._paper_swapped = True
            print(f"[d6] paper changed -> {self.gas_result}")

    def _show_paper(self, key):
        for v in PAPER_VARIANTS:
            self._set_visibility(f"{self.TESTPAPER}/{v}", v == key)

    def _show_liquid(self, key):
        for c in LIQUID_COLORS:
            self._set_visibility(f"{self.TUBE_SOLUTION}/liquid_{c}", c == key)

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
