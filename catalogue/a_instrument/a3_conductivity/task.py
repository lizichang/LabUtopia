# -*- coding: utf-8 -*-
"""A3 电导率测量任务（v1：第一步=竖直夹住玻璃皿提起来）。

当前只实现表面皿生命周期：rest → 近抓点+合爪 → attached（皿原点随 TCP 纯平移、
粉堆随皿同位移）→ 开爪 → released（皿+粉回 rest）。后续步骤（称量配液 / 电极浸入 /
读数）逐步追加。

皿持握 = 纯平移（a2 旋光管同款）：皿无旋转、保持平放，set_object_position 写 translate op，
皿原点 = TCP − 0.0146（皿 prim 原点在皿底；TCP=tool_center 比指端高 0.027，指端 0.835 进天平机身顶
5mm——无碰撞仅接近时短暂穿入；皿底 0.8474 在指端上方 12.4mm、皿中心高出指腹正中——四改让皿偏上；
attach 时皿原点 = 0.8620−0.0146 = 0.8474 = rest，零跳变）。
粉堆 = 独立 /World prim（SamplePowder，scale 0.25 半大小），随皿同位移（粉原点 = 皿原点 + 0.004525）。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    DISH_XY, DISH_ORIG_REST_Z, DISH_GRASP_Z, GRIP_DISH,
    DISH_GRIP_OPEN, DISH_HELD_OFFSET_Z,
    POWDER_PATH, POWDER_ORIG_REST_Z, POWDER_HELD_OFFSET_Z,
)


class A3ConductivityTask(BaseTask):
    """A3 电导率测量任务（v1：竖直夹住玻璃皿提起来）。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    DISH = "/World/SurfaceDish"
    DISH_ORIG = np.array([DISH_XY[0], DISH_XY[1], DISH_ORIG_REST_Z])   # 皿 prim 原点 rest
    DISH_GRASP = np.array([DISH_XY[0], DISH_XY[1], DISH_GRASP_Z])      # 抓点（tool_center，指端 0.835 进机身顶 5mm）
    DISH_GRIP_CLOSED = GRIP_DISH + 0.004                                # 夹紧阈值 0.031
    DISH_HELD_OFFSET = np.array([0.0, 0.0, DISH_HELD_OFFSET_Z])         # 皿原点 = TCP + 偏移
    POWDER_ORIG = np.array([DISH_XY[0], DISH_XY[1], POWDER_ORIG_REST_Z])
    POWDER_OFFSET = np.array([0.0, 0.0, POWDER_HELD_OFFSET_Z])          # 粉原点 = 皿原点 + 偏移

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰，d2s 同款）
        self._disable_collision(self.DISH)
        self._disable_collision(POWDER_PATH)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        self.dish_state = "rest"   # rest / attached / released
        self._dish_near_frames = 0

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.dish_state = "rest"
        self._dish_near_frames = 0
        self.object_utils.set_object_position(self.DISH, self.DISH_ORIG)
        self.object_utils.set_object_position(POWDER_PATH, self.POWDER_ORIG)

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
        return self.get_basic_state_info(additional_info={
            "dish_state": self.dish_state,
        })

    def on_task_complete(self, success):
        print(f"[a3] episode done success={success} dish={self.dish_state}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧皿持握（纯平移）：rest → 近抓点+合拢 → attached（皿中心随 TCP、粉堆随皿）
    #   → 松开（>0.03）→ released（皿+粉回 rest，可再抓）
    # ------------------------------------------------------------------
    def _update_dish(self, gripper_pos, opening):
        if self.dish_state == "rest":
            near = self._near_grasp(gripper_pos, self.DISH_GRASP)
            self._dish_near_frames = self._dish_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_dish_to_gripper(gripper_pos)
            if (near and self._dish_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.DISH_GRIP_CLOSED):
                self.dish_state = "attached"
                self._set_dish_held(gripper_pos)
                print(f"[a3] dish attached (grip={opening:.4f})")
            return

        if self.dish_state == "attached":
            self._set_dish_held(gripper_pos)
            if opening > DISH_GRIP_OPEN:   # 0.038：皿刚性触壁 opening≈0.030 不会误判松开
                self.dish_state = "released"
                self.object_utils.set_object_position(self.DISH, self.DISH_ORIG)
                self.object_utils.set_object_position(POWDER_PATH, self.POWDER_ORIG)
                print(f"[a3] dish released to balance (grip={opening:.4f})")
        # released：已回 rest，不再跟随

    def _ease_dish_to_gripper(self, gripper_pos, k=0.18):
        """夹爪合拢期间皿逐帧平滑拉向持握位（消除闪现吸附，纯平移插值足够）。"""
        cur = self.object_utils.get_object_xform_position(self.DISH)
        if cur is None:
            return
        target = gripper_pos + self.DISH_HELD_OFFSET
        self.object_utils.set_object_position(
            self.DISH, np.asarray(cur, dtype=float) + (target - cur) * k)
        self._set_powder_follow()

    def _set_dish_held(self, gripper_pos):
        """皿跟随夹爪（皿原点 = TCP + 偏移），粉堆同位移跟随。"""
        self.object_utils.set_object_position(self.DISH, gripper_pos + self.DISH_HELD_OFFSET)
        self._set_powder_follow()

    def _set_powder_follow(self):
        dish = self.object_utils.get_object_xform_position(self.DISH)
        if dish is None:
            return
        self.object_utils.set_object_position(POWDER_PATH, dish + self.POWDER_OFFSET)

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
