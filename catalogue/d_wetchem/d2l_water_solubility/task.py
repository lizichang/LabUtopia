"""D2-L 液体样品水溶性测试任务（第一步：取样滴管 吸样品→滴入试管）。

与 D4-L 同构（持握照 flametest 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**
写为 TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——
滴管全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方）。

D2-L 相对 D4-L 的差异：单支滴管（无碱滴管/碱瓶）；滴入的是液体样品（样品色）；
无气泡/沉淀/变色（现象三档 mixing 在震荡步骤才分化，第一步先只做滴样）。

生命周期（滴管，gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（样品液柱被吸进尖嘴）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；试管口区挤胶头（<GRIP_SQUEEZED）
             → dropped → DropperFill 隐藏 + LayerColumn（样品色柱）显示且液面逐滴升高
  - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（一次
             持握内循环吸液-滴液，不松开）；末遍滴完回架松开（>gripper_open）
             → released（写回架内竖插位姿）并复位 rest

controller 顺序（第一步）：SAMPLE_PASS（取样滴管吸样品→滴入试管，cfg.sample_cycles 遍）。
注水/震荡步骤（WashBottlePass/TubeShakePass + 现象三档）后续步骤补。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
    SAMPLE_BOTTLE_XY, TUBE_XY,
    EFFECT_TUBE_DROPS, EFFECT_LAYER_COLUMN,
    EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    样品瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
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
                print(f"[d2l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d2l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d2l] {self.name} filled (aspirated)")
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
                print(f"[d2l] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[d2l] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d2l] {self.name} released to rack -> rest")


class D2LWaterSolubilityTask(BaseTask):
    """D2-L 液体样品水溶性测试任务（第一步：取样滴管 吸样品→滴入试管）。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 管内液体逐滴生长（底面贴管底 0.806；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    DROP_LEVEL_STEP = 0.004   # 每滴落定后液面升高 4mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内样品液柱 60mm 很满，一挤该是一串滴不是一滴——用户 2026-08-14）。每滴
    # delay 错帧起落 → 悬停成形 → 加速坠落，落定才长液面（4 滴/挤=16mm/挤）。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数（成串时挂短点整体才连贯）
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.13m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    DROPPER_SAMPLE = "/World/DropperSample"
    TUBE = "/World/TestTube"

    TUBE_DROPS = EFFECT_TUBE_DROPS       # 水色柱（v2 注水后显示：分层态下层水）
    LAYER_COLUMN = EFFECT_LAYER_COLUMN   # 样品色柱（v1 滴样显示：整管样品；v2 分层态上层）
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER_SAMPLE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        # 取样滴管生命周期句柄（参考点已 pxr 实测）
        self.droppers = {
            "sample": _DropperLifecycle(
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_drop_count = 0      # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []          # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）

    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_drop_count = 0
        self._drop_queue = []
        for d in self.droppers.values():
            d.reset()
        for p in (self.TUBE_DROPS, self.LAYER_COLUMN, self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visibility(p, False)
        lc = self.stage.GetPrimAtPath(self.LAYER_COLUMN)
        if lc.IsValid():
            UsdGeom.Cylinder(lc).GetHeightAttr().Set(0.0)  # 样品色柱高度复位
            self.object_utils.set_object_position(
                self.LAYER_COLUMN, (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z))

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._step_drop_anim()              # 滴落动画独立推进（与抓取/滴加并行）
        for d in self.droppers.values():
            d.step(gripper_pos, opening)
        return self.get_basic_state_info(additional_info={
            "sample_attached": self.droppers["sample"].attached,
            "sample_filled": self.droppers["sample"].filled,
            "sample_dropped": self.droppers["sample"].dropped,
            "sample_released": self.droppers["sample"].released,
        })

    def on_task_complete(self, success):
        print(f"[d2l] episode done success={success} "
              f"sample_dropped={self.droppers['sample'].dropped} "
              f"sample_released={self.droppers['sample'].released}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 滴管位姿
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        """物体尖嘴（原点）世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保竖立姿态）。

        flametest 同款：不 ClearXformOpOrder、不写 4x4 矩阵——烘平场景里滴管只有
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
        """DropperFill 样品液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴，+Z 收窄→加宽
        贴合玻璃体）。尖嘴在夹爪下 0.13m（保竖立），液柱从尖嘴向上 60mm（几何见 gen 脚本），
        整体在玻璃体直管段内、不顶到胶头。"""
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
        """任一滴加：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴（尖嘴下逐滴错落坠落）。

        挤胶头瞬间在尖嘴正下方生成一串样品色液滴（DropperDrop 父 Xform 的 Drop_0.._N 球，
        每滴一格），delay 错帧起落形成连续"滴-滴-滴"。每滴落定后管内液面长高
        DROP_LEVEL_STEP。现象三档（mixing）在震荡步骤才分化，此处不触发。
        """
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（管口上方 25mm，液滴可见坠落）
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._tube_drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            target = np.array([TUBE_XY[0], TUBE_XY[1],
                               self.TUBE_BOTTOM_Z + level - 0.003])  # 落定在管内液面
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(), "target": target,
                "level": level, "name": dropper.name,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._tube_drop_count += self.DROPS_PER_SQUEEZE
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[d2l] squeeze ({dropper.name}) -> {self.DROPS_PER_SQUEEZE} drops spawned")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该球+长液面）。"""
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
                # 落定：隐藏这颗、长液面，移出队列
                self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_tube_level(d["level"], d["name"])
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    def _grow_tube_level(self, h, name):
        """液滴落定：管内液面长到高度 h（圆柱高+上移，底面贴管底）。

        D2-L 双柱现象设计（2026-08-24）：name 决定长哪根柱——
          sample → LayerColumn（样品色柱，第一步滴样品：整管样品色，非水色）
          water  → TubeDrops（水色柱，第二步注水：分层态下层水，WashBottlePass 用）
        注水步骤（v2）未实现前 SamplePass 恒走 sample 分支，TubeDrops 保持隐藏。"""
        cyl_path = self.LAYER_COLUMN if name == "sample" else self.TUBE_DROPS
        prim = self.stage.GetPrimAtPath(cyl_path)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                cyl_path,
                (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z + h / 2))
        self._set_visibility(cyl_path, True)
        print(f"[d2l] tube liquid level h={h:.3f} (col={name})")

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
