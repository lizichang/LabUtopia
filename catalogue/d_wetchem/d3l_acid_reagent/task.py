"""D3-L 酸性试剂滴加反应任务：两支滴管各自的「吸液→滴入试管」生命周期 + 液体效果。

持握照 flametest（v24-v46 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——滴管
全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），效果 prim（DropperFill）
只需 position 跟随尖嘴即可。

生命周期（每支滴管，gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（液柱被吸进尖嘴，"像不像水"的动态可视）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；试管口区挤胶头（<GRIP_SQUEEZED）
             → dropped → DropperFill 隐藏 + TubeDrops 显示且液面逐滴升高
             （每滴 +DROP_LEVEL_STEP，上限 DROP_LEVEL_MAX）
  - 现象（Bubbles/Precipitate）只在**加酸滴管**滴加（样品+酸混合）时按 cfg 触发；
    本阶段只跑取样滴管，无反应现象
  - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（**一次
             持握内循环**吸液-滴液，不松开）；末遍滴完回架松开（>gripper_open）
             → released（写回架内竖插位姿）并复位 rest

两个滴管生命周期句柄（sample/acid）共用一套状态机，各自参考点不同；本阶段 SAMPLE_PASS
**一次持握内循环吸液-滴液 cfg.sample_cycles 遍**（抓一次→多遍滴→放回一次），ACID_PASS
下轮补。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
    DROP_ACID_REST, DROP_ACID_GRASP,
    SAMPLE_BOTTLE_XY, ACID_BOTTLE_XY, TUBE_XY,
    EFFECT_TUBE_DROPS, EFFECT_PRECIPITATE, EFFECT_BUBBLES, EFFECT_DROPPER_FILL,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])


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
        self.task._set_obj_world(self.path, self.orig)
        if self.fill_path:
            self.task._set_visibility(self.fill_path, False)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + HELD_OFFSET
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（flametest v28 同款，
            # 消除闭合瞬间闪现吸附）。只在 near 时 ease，避免合爪未遂拖离原位。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d3l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

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
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[d3l] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d3l] {self.name} released to rack -> rest")


class D3LAcidReagentTask(BaseTask):
    """D3-L 酸性试剂滴加任务：两支滴管吸液→滴入试管 + 液体效果 prim。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 管内液体逐滴生长（底面贴管底 0.806；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    DROP_LEVEL_STEP = 0.008   # 每滴液面升高 8mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm（≈7 滴）

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
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
            "acid": _DropperLifecycle(
                self, "acid", self.DROPPER_ACID, DROP_ACID_REST, DROP_ACID_GRASP,
                ACID_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_drop_count = 0

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_drop_count = 0
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
    def _get_obj_world(self, path):
        """物体尖嘴（原点）世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保竖立姿态）。

        flametest 同款：不 ClearXformOpOrder、不写 4x4 矩阵——烘平场景里滴管/试管只有
        xformOp:translate 一个 op，set_object_position 改首 op 即平移，姿态不变。
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

    def _set_fill_follow(self, dropper):
        """DropperFill 截锥液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴，+Z 收窄→加宽贴合
        玻璃体）。尖嘴在夹爪下 0.13m（保竖立），液柱从尖嘴向上 40mm，整体在玻璃体内。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(self.DROPPER_FILL, tip)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """任一滴加：管内液面逐滴升高（圆柱高+上移，底面贴管底）。

        反应现象（气泡/沉淀）只在**加酸**（样品+酸混合）时出现——本阶段只跑取样滴管
        滴加（加酸下轮补），管里先积样品液、无反应物，故不触发任何现象。
        """
        self._tube_drop_count += 1
        h = min(self.DROP_LEVEL_STEP * self._tube_drop_count, self.DROP_LEVEL_MAX)
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self.TUBE_DROPS,
                (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z + h / 2))
        self._set_visibility(self.TUBE_DROPS, True)
        if dropper.name == "acid":
            if self.has_bubbles:
                self._set_visibility(self.BUBBLES, True)
            if self.has_precipitate:
                self._set_visibility(self.PRECIPITATE, True)
        print(f"[d3l] drop #{self._tube_drop_count} ({dropper.name}) -> tube liquid level h={h:.3f}")

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
