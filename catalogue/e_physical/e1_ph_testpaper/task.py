"""E1「pH 试纸检测」任务：玻璃棒平移跟随 + pH 色斑显色。

与 flametest 的 set_object_position 平移跟随同构。本实验无手腕翻转、且试纸条已默认
铺在白瓷板上（不再夹取/铺放），故 task 只对玻璃棒做平移跟随：每帧把被持握玻璃棒写到
TCP 相对位：
  玻璃棒  棒底 = 抓点 - (0,0,ROD_TIP_TO_GRIP)（抓点在棒底上方 0.20m）

pH 变色接口（预留）：试纸中央 3 个同几何不同色斑变体（spot_acidic/neutral/alkaline）
初始全隐藏，task 在棒尖点触试纸中央时按 cfg.ph_result 显示对应变体（坑 27：RTX 下
改 Shader 不刷新，须预制变体 + visibility 切换）。
"""
import numpy as np
from pxr import UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_ROD,
    ROD_GRASP, ROD_TIP_TO_GRIP, ROD_REST_POS,
    PLATE_XY, TOUCH_TIP_Z,
)


class E1PhTestpaperTask(BaseTask):
    """E1 pH 试纸检测：玻璃棒蘸取待测液 → 点触试纸显色 → 归位（试纸已预铺白瓷板）。"""

    ROD_PATH = "/World/GlassRod"

    ROD_GRASP = np.array(ROD_GRASP)
    ROD_GRIP_CLOSED = GRIP_ROD + 0.004       # 夹紧阈值：grip 0.003 + 4mm 裕量
    GRIP_OPEN_THRESH = 0.03                  # 松开阈值（与 flametest/d2s 一致）

    # pH 色斑变体（gen_e1_scene.py 建的 3 个隐藏 prim）
    SPOT_PATHS = {key: f"/World/TestPaper/spot_{key}"
                  for key in ("acidic", "neutral", "alkaline")}

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.rod_path = self.ROD_PATH
        # 静态几何：玻璃棒持握期关碰撞（逐帧 transform 覆写 + 手指闭合，避免物理干扰）
        self._disable_collision(self.rod_path)

        self.GRASP_NEAR_FRAMES = 3
        self._rod_near_frames = 0
        self.rod_state = "rest"      # rest / attached / released
        self.spot_shown = False

        # pH 变色接口：读 cfg.ph_result（experiment_result 框架 --result 写回）
        self.ph_result = getattr(cfg, "ph_result", "neutral")
        if self.ph_result not in self.SPOT_PATHS:
            self.ph_result = "neutral"

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.rod_state = "rest"
        self._rod_near_frames = 0
        self.spot_shown = False
        # 棒回静止位；3 个色斑变体全隐藏
        self.object_utils.set_object_position(self.ROD_PATH, np.array(ROD_REST_POS))
        for path in self.SPOT_PATHS.values():
            self._set_visibility(path, False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_rod()
        return self.get_basic_state_info(additional_info={
            "rod_state": self.rod_state,
            "ph_result": self.ph_result,
            "spot_shown": self.spot_shown,
        })

    def on_task_complete(self, success):
        print(f"[e1] episode done success={success} "
              f"rod={self.rod_state} spot_shown={self.spot_shown} ph_result={self.ph_result}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 玻璃棒持握（rest → attached → released）+ 点触显色
    # ------------------------------------------------------------------
    def _update_rod(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]

        if self.rod_state == "rest":
            if self._near_grasp(gripper_pos, self.ROD_GRASP):
                self._rod_near_frames += 1
            else:
                self._rod_near_frames = 0
            if (self._rod_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.ROD_GRIP_CLOSED):
                self.rod_state = "attached"
                print(f"[e1] rod attached (grip={opening:.4f})")

        elif self.rod_state == "attached":
            # 平移跟随：棒底 = 抓点 - (0,0,ROD_TIP_TO_GRIP)
            rod_pos = np.asarray(gripper_pos, dtype=float) - np.array(
                [0.0, 0.0, ROD_TIP_TO_GRIP])
            self.object_utils.set_object_position(self.ROD_PATH, rod_pos)
            # 点触显色：棒尖贴近试纸中央（transfer 下探到位）→ 按 cfg.ph_result 显示色斑
            if not self.spot_shown and self._rod_tip_near_paper(gripper_pos):
                self.spot_shown = True
                self._show_spot(self.ph_result)
                print(f"[e1] pH spot shown: {self.ph_result}")
            if opening > self.gripper_open_threshold:
                self.rod_state = "released"
                self.object_utils.set_object_position(self.ROD_PATH, np.array(ROD_REST_POS))
                print("[e1] rod released to rack")

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _rod_tip_near_paper(self, gripper_pos):
        """棒尖（底）贴近试纸中央（PLATE_XY）且降到纸面高度 → 点触成功。

        防误触发：蘸取时棒尖在试管孔(0.2787,0.1193)远离纸中央(near_xy 失败)；水平平移
        时棒尖在 z=1.00（gripper 1.20−0.20）高于纸面(near_z 失败)。仅 transfer 下探到位
        （棒尖 z→0.8070）时 near_xy+near_z 齐备 → 触发一次。

        near_z 裕量 +0.015（0.822）：MoveAction 冻结判定是 TCP 距目标 <1cm 即冻结，下探
        可能停在目标上方 ≤1cm 处（棒尖最高 0.817 > 旧阈值 0.812）→ 旧 0.005 裕量会因
        冻结竞态永不触发（用户报"颜色没变化"）。+0.015 覆盖最坏冻结位 0.817 仍余 5mm。
        """
        tip = np.asarray(gripper_pos, dtype=float) - np.array([0.0, 0.0, ROD_TIP_TO_GRIP])
        near_xy = np.linalg.norm(tip[:2] - np.array(PLATE_XY)) < 0.02
        near_z = tip[2] < TOUCH_TIP_Z + 0.015
        return near_xy and near_z

    def _show_spot(self, key):
        for k, path in self.SPOT_PATHS.items():
            self._set_visibility(path, k == key)

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
