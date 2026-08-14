"""D3-L 酸性试剂滴加反应任务：两支滴管各自的「吸液→滴入试管」生命周期 + 液体效果。

仿 d2s_water_solubility/task.py 的持握模式（关碰撞 + transform-op 覆写逐帧跟随）：
滴管是静态碰撞体，吸附期把世界位姿写为 tool_center 世界矩阵 · _T_HELD_DROPPER
（R_x(π) + t(0,0,0.13)）——滴管保持竖立（胶头上）、尖嘴 0.13m 吊在夹爪下方，全程
纯平移无旋转，所以效果 prim（DropperFill）只需 position 跟随尖嘴即可。

生命周期（每支滴管，gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（液柱被吸进尖嘴，"像不像水"的动态可视）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；试管口区挤胶头（<GRIP_SQUEEZED）
             → dropped → DropperFill 隐藏 + TubeDrops 显示
             （cfg.has_bubbles / cfg.has_precipitate → 同时显示 Bubbles/Precipitate）
  - dropped  跟随；回架松开（>gripper_open）→ released（写回架内竖插位姿）

两个滴管生命周期句柄（sample/acid）共用一套状态机，各自参考点不同；本阶段 controller
只跑 SAMPLE_PASS，ACID_PASS 下轮补（task 已就绪，无需改）。
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_XY, DROP_SAMPLE_GRASP,
    DROP_ACID_XY, DROP_ACID_GRASP,
    SAMPLE_BOTTLE_XY, ACID_BOTTLE_XY, TUBE_XY,
    EFFECT_TUBE_DROPS, EFFECT_PRECIPITATE, EFFECT_BUBBLES, EFFECT_DROPPER_FILL,
)

# 滴管相对夹爪：R_x(π) + 平移 (0,0,0.13)（滴管 local +Z = 胶头方向）。
# 平移必须在最后一行（USD 行向量约定）；写在每行第 4 个参数会变成非仿射矩阵，
# 持握时滴管相对夹爪偏移错乱（d2s 药匙同坑）。
_T_HELD = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      0.0, 0.0, -1.0, 0.0,
                      0.0, 0.0, TIP_OFFSET, 1.0)


def _rest_matrix(orig):
    """滴管架内竖插位姿（无旋转，只平移，与场景 /World/DropperX 一致）。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       orig[0], orig[1], orig[2], 1.0)


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    所对瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
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
        self.task._set_obj_world(self.path, _rest_matrix(self.orig))
        if self.fill_path:
            self.task._set_visibility(self.fill_path, False)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            if self.task._near(self.grasp, gripper_pos):
                self._near_frames += 1
            else:
                self._near_frames = 0
            if (self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_from_gripper(self.path)
                print(f"[d3l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪
        self.task._set_obj_from_gripper(self.path)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d3l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d3l] {self.name} filled (aspirated)")
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
                print(f"[d3l] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 回架松开
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.state = "released"
                self.released = True
                self.task._set_obj_world(self.path, _rest_matrix(self.orig))
                print(f"[d3l] {self.name} released to rack")


class D3LAcidReagentTask(BaseTask):
    """D3-L 酸性试剂滴加任务：两支滴管吸液→滴入试管 + 液体效果 prim。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    DROPPER_SAMPLE = "/World/DropperSample"
    DROPPER_ACID = "/World/DropperAcid"

    TUBE_DROPS = EFFECT_TUBE_DROPS
    PRECIPITATE = EFFECT_PRECIPITATE
    BUBBLES = EFFECT_BUBBLES
    DROPPER_FILL = EFFECT_DROPPER_FILL

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER_SAMPLE)
        self._disable_collision(self.DROPPER_ACID)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        self.has_bubbles = bool(getattr(cfg, "has_bubbles", False))
        self.has_precipitate = bool(getattr(cfg, "has_precipitate", False))

        # 两支滴管各自的生命周期句柄（参考点已 pxr 实测）
        self.droppers = {
            "sample": _DropperLifecycle(
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_XY, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
            "acid": _DropperLifecycle(
                self, "acid", self.DROPPER_ACID, DROP_ACID_XY, DROP_ACID_GRASP,
                ACID_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_dropped = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_dropped = False
        for d in self.droppers.values():
            d.reset()
        for p in (self.TUBE_DROPS, self.PRECIPITATE, self.BUBBLES, self.DROPPER_FILL):
            self._set_visibility(p, False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        for d in self.droppers.values():
            d.step(gripper_pos, opening)
        return self.get_basic_state_info(additional_info={
            "sample_attached": self.droppers["sample"].attached,
            "sample_filled": self.droppers["sample"].filled,
            "sample_dropped": self.droppers["sample"].dropped,
            "acid_attached": self.droppers["acid"].attached,
            "acid_filled": self.droppers["acid"].filled,
            "acid_dropped": self.droppers["acid"].dropped,
        })

    def on_task_complete(self, success):
        print(f"[d3l] episode done success={success} "
              f"sample_dropped={self.droppers['sample'].dropped} "
              f"acid_dropped={self.droppers['acid'].dropped}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 滴管位姿
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_obj_from_gripper(self, path):
        self._set_obj_world(path, self._tool_world() * _T_HELD)

    def _set_obj_world(self, path, world_matrix):
        """把物体写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _set_fill_follow(self, dropper):
        """DropperFill 液柱跟随滴管尖嘴：尖嘴在夹爪下 TIP_OFFSET，柱中心再上 9mm。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        z_dir = wm_np[:3, 2]
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) - TIP_OFFSET * z_dir
        center = tip - 0.009 * z_dir
        self.object_utils.set_object_position(self.DROPPER_FILL, center)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """任意一支滴管完成滴加：管内液滴显示；experiment_result 驱动气泡/沉淀。"""
        if not self._tube_dropped:
            self._tube_dropped = True
            self._set_visibility(self.TUBE_DROPS, True)
            print("[d3l] TubeDrops visible (first drop into tube)")
        if self.has_bubbles:
            self._set_visibility(self.BUBBLES, True)
        if self.has_precipitate:
            self._set_visibility(self.PRECIPITATE, True)

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
