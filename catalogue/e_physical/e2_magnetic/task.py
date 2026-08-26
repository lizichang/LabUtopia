"""E2 磁性检测任务：磁铁纯平移持握 + 磁性颗粒吸起效果。

2026-08-26 用户简化设计：待测固体已预先取出铺在表面皿上（DishPowder 常显），删掉
药匙/试管架/样品瓶/瓶盖 → 只剩「夹磁铁 → 下探检测 → 归位」。本 task 只做：
  - 磁铁 100×15×15mm 平放（长轴 X），纯平移持握（只写 translate 保平放，不写 4x4）：
    磁铁 translate=底中心 = 抓点 + (0,0,-0.03)。抓点 z=0.83（tool_center，手指朝下指尖
    0.805 罩住磁铁上段），抓点处 translate=0.80 零跳变（手指朝下，非 ORIENT_FWD 横夹）。
  - 磁性颗粒（MagnetGrains）：磁铁靠近皿上方 + cfg.magnetic=magnetic 时吸起上浮，撤走回落。
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_MAGNET, MAGNET_GRASP, MAGNET_HELD_OFFSET,
    DISH_XY, DISH_POWDER_TOP_Z,
)


class E2MagneticTask(BaseTask):
    """E2 磁性检测任务：夹磁铁检测的持握与效果驱动（待测固体已铺在皿上）。"""

    TABLE_Z = 0.80

    MAGNET_PATH = "/World/BarMagnet"
    DISH_POWDER = "/World/DishPowder"
    MAGNET_GRAINS = "/World/MagnetGrains"

    MAGNET_GRASP = np.array(MAGNET_GRASP)
    MAGNET_GRIP_CLOSED = GRIP_MAGNET + 0.004   # 夹紧阈值：grip 0.0075 + 4mm 裕量
    GRIP_OPEN_THRESH = 0.03                    # 松开阈值（与 d2s/flametest 一致）

    # 磁性颗粒动画参数（与 gen_e2_scene 对齐：父 prim rest z=DISH_POWDER_TOP_Z=0.8106，
    # 约 110 颗 r1.5mm 细铁粉随机散布，一开始可见；有磁性时被吸起上浮到磁铁底 0.83，无磁性停留）
    GRAIN_PARENT_Z = DISH_POWDER_TOP_Z         # 0.8106
    GRAIN_RISE = 0.016                         # 吸起上浮高度（球心 0.8121→0.8285，贴磁铁底 0.83）
    GRAIN_RISE_RATE = 0.02                     # 上升速率/帧
    GRAIN_FALL_RATE = 0.03                     # 回落速率/帧

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 磁铁是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self.magnet_path = self.MAGNET_PATH
        self._disable_collision(self.magnet_path)

        self.GRASP_NEAR_FRAMES = 3
        self._magnet_near_frames = 0
        self.magnet_state = "rest"
        self._grain_frac = 0.0                 # 磁性颗粒吸起进度 0..1
        self._detect_active = False

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 磁性检测结果（main.py 已按 experiment_result schema 把 CLI/交互结果写回 cfg.magnetic）
        self.magnetic = getattr(cfg, "magnetic", "magnetic") == "magnetic"

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.magnet_state = "rest"
        self._magnet_near_frames = 0
        self.object_utils.set_object_position(
            self.MAGNET_PATH, np.array([MAGNET_GRASP[0], MAGNET_GRASP[1], self.TABLE_Z]))
        self._set_visibility(self.DISH_POWDER, True)     # 待测固体预铺，常显
        self._set_visibility(self.MAGNET_GRAINS, True)   # 铁粉颗粒一开始可见（物质明显在皿上）
        self._grain_frac = 0.0
        self._detect_active = False

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_magnet()
        self._step_grains()
        return self.get_basic_state_info(additional_info={
            "magnet_state": self.magnet_state,
        })

    def on_task_complete(self, success):
        print(f"[e2] episode done success={success} magnet={self.magnet_state} "
              f"magnetic={self.magnetic}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧磁铁持握（纯平移）+ 检测判定
    # ------------------------------------------------------------------
    def _update_magnet(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        held = np.asarray(gripper_pos, dtype=float) + np.array(MAGNET_HELD_OFFSET)

        if self.magnet_state == "rest":
            if self._near_grasp(gripper_pos, self.MAGNET_GRASP):
                self._magnet_near_frames += 1
            else:
                self._magnet_near_frames = 0
            if (self._magnet_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.MAGNET_GRIP_CLOSED):
                self.magnet_state = "attached"
                self.object_utils.set_object_position(self.MAGNET_PATH, held)
                print(f"[e2] magnet attached (grip={opening:.4f})")

        elif self.magnet_state == "attached":
            self.object_utils.set_object_position(self.MAGNET_PATH, held)
            # 检测判定：磁铁在皿上方（近 DISH_XY）且降到检测高度（z<0.90）→ 驱动磁性颗粒吸起
            gp = np.asarray(gripper_pos, dtype=float)
            self._detect_active = (np.linalg.norm(gp[:2] - np.array(DISH_XY)) < 0.05
                                   and gp[2] < 0.90)
            if opening > self.gripper_open_threshold:
                self.magnet_state = "released"
                self._detect_active = False
                self.object_utils.set_object_position(
                    self.MAGNET_PATH, np.array([MAGNET_GRASP[0], MAGNET_GRASP[1], self.TABLE_Z]))
                print("[e2] magnet released to table")

    # ------------------------------------------------------------------
    def _step_grains(self):
        """铁粉颗粒吸起/回落：颗粒一开始可见（待测固体的铁粉表层，物质明显在皿上）。
        cfg.magnetic=magnetic 且磁铁检测中 → 父 prim z 上浮 GRAIN_RISE（贴磁铁底）；
        磁铁撤走 → 回落皿上。non_magnetic → 留在皿上不动（不吸附）。"""
        if not self.magnetic:
            # 无磁性：颗粒留在皿上（rest 位，可见，不吸附）
            self.object_utils.set_object_position(
                self.MAGNET_GRAINS, np.array([DISH_XY[0], DISH_XY[1], self.GRAIN_PARENT_Z]))
            self._set_visibility(self.MAGNET_GRAINS, True)
            self._grain_frac = 0.0
            return
        if self._detect_active and self._grain_frac < 1.0:
            self._grain_frac = min(1.0, self._grain_frac + self.GRAIN_RISE_RATE)
        elif not self._detect_active and self._grain_frac > 0.0:
            self._grain_frac = max(0.0, self._grain_frac - self.GRAIN_FALL_RATE)
        z = self.GRAIN_PARENT_Z + self.GRAIN_RISE * self._grain_frac
        self.object_utils.set_object_position(
            self.MAGNET_GRAINS, np.array([DISH_XY[0], DISH_XY[1], z]))
        self._set_visibility(self.MAGNET_GRAINS, True)

    # ------------------------------------------------------------------
    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

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
