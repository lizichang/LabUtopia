"""D7 气体鉴定任务：带导气管橡皮塞预装塞紧产气试管口 + 检验试管下浸通气。

2026-08-27 用户方案：检测试剂统一表现为液体（无浑浊/变色变体），仅初始颜色由输入 liquid_color
决定（6 色变体，默认 colorless）；通气+识别合并为单个 HoldDetect；
**2026-08-27 二改：跳过夹取/拔塞（橡皮塞预装塞紧产气试管口），机械臂只做检验试管的下浸→
通气→归位三动作**。

本 task 只做 3 类驱动（与 d6 同构，纯平移持握 + visibility 变体切换 + 气泡动画）：

  1. 检验试管生命周期：rest（架内）→ attached（夹管身）→ released（回架松开）。持握 = 纯平移
     （管底 = tool_center + (0,0,-0.139)），管内检测液（TubeSolution 父 prim）逐帧随管平移。
  2. 检测液颜色：6 色变体预制，task 按 cfg.liquid_color 显示其一（父 prim 随管平移）。
  3. 气泡动画：HoldDetect 期间（试管下浸到位）GasBubbles 气泡从导气管末端 1.024 连续上升到
     检测液面 1.039（气体通入检测液的可视化）。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TEST_TUBE_XY, TUBE_BOTTOM_Z, TUBE_GRASP, TUBE_HELD_OFFSET,
    DIP_XY, DIP_GRASP_Z, DIP_SURFACE_Z, FREE_END_Z,
)

# 持握偏移（纯平移，保竖立；同 d3l/d6）。抓点 = 立放位 + (0,0,0.139)，
# 故 translate(=管底) = tool_center - 0.139，抓点处 = rest 位零跳变。
TUBE_ORIG = np.array([TEST_TUBE_XY[0], TEST_TUBE_XY[1], TUBE_BOTTOM_Z])   # (0.300,0.160,0.806)

LIQUID_COLORS = ["colorless", "blue", "red", "green", "yellow", "purple"]


class _TubeLifecycle:
    """检验试管状态机：rest → attached（夹管身，检测液随管平移）→ released（回架松开）。"""

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
                print(f"[d7] tube attached (grip={opening:.4f})")
            return

        if self.state != "attached":
            return   # released：已放回，不再跟随

        # 吸附期：试管 + 管内检测液逐帧跟随（纯平移保竖立）
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
            print("[d7] tube released to rack")


class D7GasIdentificationTask(BaseTask):
    """D7 气体鉴定任务：带导气管橡皮塞预装 + 检验试管下浸通气观察气泡。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    TUBE = "/World/TestTube"
    TUBE_SOLUTION = "/World/TubeSolution"
    GAS_BUBBLES = "/World/GasBubbles"

    # 气泡动画（HoldDetect 通气）：末端 1.024 → 液面 1.039 连续上升
    BUBBLE_COUNT = 8
    BUBBLE_RISE = 15       # 单气泡上升帧数
    BUBBLE_STAGGER = 8     # 相邻气泡起落间隔
    DETECT_XY_THRESH = 0.03

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 试管是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）。
        # 橡皮塞预装塞紧（静态不动，无持握），无需关碰撞。
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 检测液颜色（main.py 已按 experiment_result schema 把 CLI/交互结果写回 cfg.liquid_color）
        self.liquid_color = str(getattr(cfg, "liquid_color", "colorless")).strip().lower()
        if self.liquid_color not in LIQUID_COLORS:
            self.liquid_color = "colorless"

        self.tube = _TubeLifecycle(self)

        # 气泡动画状态
        self._bubble_queue = []
        self._bubble_spawned = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.tube.reset()
        # 检测液：显示 cfg.liquid_color 变体，父贴管底 rest 位
        self._show_liquid(self.liquid_color)
        self._set_obj_world(self.TUBE_SOLUTION, TUBE_ORIG)
        # 气泡动画复位（隐藏）
        self._bubble_queue = []
        self._bubble_spawned = False
        self._set_visibility(self.GAS_BUBBLES, False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self.tube.step(gripper_pos, opening)
        self._step_bubbles(gripper_pos)             # 试管下浸到位 → 气泡连续上升
        return self.get_basic_state_info(additional_info={
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[d7] episode done success={success} "
              f"tube_released={self.tube.released} "
              f"liquid_color={self.liquid_color}")
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
    # 气泡动画（HoldDetect 通气）
    # ------------------------------------------------------------------
    def _spawn_bubble_wave(self):
        """一轮 BUBBLE_COUNT 个气泡，错帧从导气管末端 1.024 连续上升到检测液面 1.039。"""
        for i in range(self.BUBBLE_COUNT):
            self._bubble_queue.append({
                "idx": i,
                "delay": i * self.BUBBLE_STAGGER,
                "t": 0,
                "rise": self.BUBBLE_RISE,
                "z0": FREE_END_Z,
                "z1": DIP_SURFACE_Z,
            })
        self._set_visibility(self.GAS_BUBBLES, True)

    def _step_bubble_anim(self):
        if not self._bubble_queue:
            return
        remaining = []
        for b in self._bubble_queue:
            if b["delay"] > 0:
                b["delay"] -= 1
                remaining.append(b)
                continue
            b["t"] += 1
            if b["t"] <= b["rise"]:
                frac = b["t"] / b["rise"]
                z = b["z0"] + (b["z1"] - b["z0"]) * frac
                self._set_visibility(f"{self.GAS_BUBBLES}/bubble_{b['idx']}", True)
                # 气泡父 Xform 在下浸点 (0.44,0.079,0)：local z 即 world z
                self.object_utils.set_object_position(
                    f"{self.GAS_BUBBLES}/bubble_{b['idx']}", np.array([0.0, 0.0, z]))
            else:
                self._set_visibility(f"{self.GAS_BUBBLES}/bubble_{b['idx']}", False)
                continue
            remaining.append(b)
        self._bubble_queue = remaining

    def _step_bubbles(self, gripper_pos):
        """试管下浸到位（attached 且夹爪近下浸抓点）时连续通气冒泡；离开则清空隐藏。"""
        gp = np.asarray(gripper_pos, dtype=float)
        bubbling = (self.tube.attached
                    and self._near_xy(DIP_XY, gp)
                    and abs(gp[2] - DIP_GRASP_Z) < 0.02)
        if not bubbling:
            self._bubble_queue = []
            self._bubble_spawned = False
            self._set_visibility(self.GAS_BUBBLES, False)
            return
        if not self._bubble_spawned:
            self._spawn_bubble_wave()
            self._bubble_spawned = True
        self._step_bubble_anim()
        if not self._bubble_queue:
            # 一轮冒完仍在通气 → 再来一轮（连续冒泡）
            self._spawn_bubble_wave()

    # ------------------------------------------------------------------
    # 检测液颜色变体切换
    # ------------------------------------------------------------------
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
