# -*- coding: utf-8 -*-
"""B2 沸点测定任务（酒精灯加热试管内液体 → 温度计读数 → 沸腾 → 记录沸点）。

v2 = v1 自动加热观测 + 阶段B 滴加（V7 步骤 2-3 的滴加部分）：
    机械臂（controller DropperDripPass）抓滴管吸样品瓶液 → 滴入试管出液柱，
    本任务负责滴管生命周期 + 液滴坠落动画 + 管内液柱逐滴生长，完成后才放行
    相态机点火加热。

温度模型（v1 保留，config 可调）：T 从 room_temp 开始，ignite 停留后按 heat_rate
（°C/帧）升温，到达 cfg.boiling_point（水 100.0）后保持。相态机：
    idle（灯焰隐藏，T=room_temp，等滴加完成）→ ignited（灯焰 reveal）
    → heating（T 按 heat_rate 升温，温度计毛细柱随实时温度爬升）
    → boiling（到 cfg.boiling_point，气泡+蒸汽 reveal）→ done（保持
    boil_dwell_frames 后完成，上报沸点）。全部 phase 通过
    get_basic_state_info(additional_info) 上报，controller 读到 phase=="done" 即报成功。

滴管生命周期（照 d3l _DropperLifecycle，gripper 开度 = joint[7]，判定纯关节+TCP）：
    rest → attached → squeezed → filled → dropped → released
    - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
    - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
    - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
               → DropperFill 显示（液柱被吸进尖嘴）
    - filled   跟随（DropperFill 逐帧跟随尖嘴）；试管口区挤胶头（<GRIP_SQUEEZED）
               → dropped → DropperFill 隐藏 + 一次成串液滴坠落 + 管内液面逐滴升高
    - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（一次
               持握内循环吸液-滴液）；末遍滴完回架松开（>gripper_open）→ released
               （写回架内竖插位姿）并复位 rest

持握照 flametest（v24-v46 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET(0,0,-0.13)（只写 xformOp:translate，不写旋转矩阵、不清 xform op
表）——滴管全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），效果 prim
（DropperFill）只需 position 跟随尖嘴即可。

温度计读数（capillary_liquid 锚定缩放，pxr 已验证，v1 保留）：
    毛细红液柱 mesh z[0.005,0.245]（全量程 -20..110°C 刻度区顶）。每帧写单一
    transform op：M = T(0,0,-0.005) · S(1,1,s) · T(0,0,+0.005)，s = (z_of(T)-0.005)/0.24，
    底锚 z=0.005 不动、柱顶 = z_of(T) 随温度爬升（行向量约定：pxr 中 A·B 表示 A 先作用）。

驱动 prim（b2_alcohol_heat_liquid.usd，由 scripts/gen_b2_scene.py 生成）：
    /World/AlcoholLamp/flame_outer|flame_inner   火焰（Cone，初始隐藏）
    /World/Thermometer/Thermometer/capillary_liquid  温度计毛细红液柱（锚定缩放）
    /World/TestTubeBubbles/bubble_{0..5}         沸腾气泡（球组，初始隐藏，上升动画）
    /World/Steam/puff_{0..4}                     蒸汽（球组，初始隐藏，上浮动画）
    /World/Dropper / DropperFill / DropperDrop / TestTubeLiquid  滴加效果（阶段B）
"""
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_REST, DROP_GRASP,
    SAMPLE_BOTTLE_XY, TUBE_XY,
    EFFECT_TUBE_DROPS, EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
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
            self.task._set_visible(self.fill_path, False)

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
                print(f"[b2] dropper attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[b2] dropper squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visible(self.fill_path, True)
                print(f"[b2] dropper filled (aspirated)")
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
                    self.task._set_visible(self.fill_path, False)
                self.task._on_drop(self)
                print(f"[b2] dropper dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[b2] dropper re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[b2] dropper released to rack -> rest")


class B2AlcoholHeatLiquidTask(BaseTask):
    """B2 沸点测定任务：滴加出液柱（阶段B）→ 加热 → 沸腾 → 记录沸点。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 试管底（坐石棉网上，b2_tmp pxr 实测）/ 管内液体逐滴生长
    TUBE_BOTTOM_Z = 0.9206
    DROP_LEVEL_STEP = 0.004   # 每滴落定后液面升高 4mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm（3 遍 × 4 滴 = 48mm，接近上限）

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内液柱 60mm 很满，一挤该是一串滴不是一滴——d3l 用户 2026-08-14）。每滴
    # delay 错帧起落 → 悬停成形 → 加速坠落，落定才长液面（4 滴/挤=16mm/挤）。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数（成串时挂短点整体才连贯）
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.13m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    # 温度计毛细柱锚定：底 z=0.005，全量程 z[0.005,0.245]（-20..110°C 刻度区顶）
    CAP_BOTTOM = 0.005
    CAP_FULL_H = 0.245 - 0.005        # 0.24，柱顶 = CAP_BOTTOM + CAP_FULL_H*s

    # 刻度映射：z(T) = 0.02 + (T+20)/130*0.22（T=-20..110 -> z 0.02..0.24）
    T_MIN, T_MAX = -20.0, 110.0
    Z_LO, Z_HI = 0.02, 0.24

    # 效果动画速度（m/帧）与越界复位高度
    BUBBLE_SPEED = 0.004
    STEAM_SPEED = 0.003
    STEAM_CEIL = 1.20                 # 蒸汽越过复位到基础位（管口上方消散高度）

    # 滴加效果 prim 路径
    DROPPER = "/World/Dropper"
    TUBE_DROPS = EFFECT_TUBE_DROPS
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 温度模型参数（config 顶层可调，v1 保留）
        self.room_temp = float(getattr(cfg, "room_temp", 25.0))
        self.heat_rate = float(getattr(cfg, "heat_rate", 0.08))
        self.boiling_point = float(getattr(cfg, "boiling_point", 100.0))
        self.idle_dwell_frames = int(getattr(cfg, "idle_dwell_frames", 20))
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.boil_dwell_frames = int(getattr(cfg, "boil_dwell_frames", 240))

        # 气泡/蒸汽球组（骨架 Sphere，排除 bubble_mat/steam_mat 材质 prim）
        self.bubble_prims = self._children("/World/TestTubeBubbles")
        self.steam_prims = self._children("/World/Steam")
        self.bubble_base = [self._read_translate(p) for p in self.bubble_prims]
        self.steam_base = [self._read_translate(p) for p in self.steam_prims]

        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._cap_op = None            # 毛细柱 transform op（首次惰性创建，之后只 Set）

        # 阶段B 滴加：阈值（config 可调）+ 滴管生命周期 + 液滴动画状态
        self.sample_cycles = max(1, int(getattr(cfg, "sample_cycles", 3)))
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.0035)
        # 滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER)
        self.dropper = _DropperLifecycle(
            self, "dropper", self.DROPPER, DROP_REST, DROP_GRASP,
            SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL)
        self._drop_count = 0           # 已生成的液滴总数（每 +DROPS_PER_SQUEEZE）
        self._drop_queue = []          # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）
        self._liquid_added = False     # 滴加完成（idle 门控：全 cycles 滴完才允许点火）
        self._liquid_level = 0.0       # 当前管内液面高（气泡破灭高度跟随）

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._drop_count = 0
        self._drop_queue = []
        self._liquid_added = False
        self._liquid_level = 0.0
        self._set_visible(self._flame_paths(), False)
        self._set_visible(self.bubble_prims, False)
        self._set_visible(self.steam_prims, False)
        for p, base in zip(self.bubble_prims, self.bubble_base):
            self._set_translate(p, base)
        for p, base in zip(self.steam_prims, self.steam_base):
            self._set_translate(p, base)
        self._set_capillary(self.room_temp)
        # 滴加复位：滴管回架、液柱/填充/滴落效果隐藏、管柱高度归零
        self.dropper.reset()
        for p in (self.TUBE_DROPS, self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visible(p, False)
        lq = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if lq.IsValid():
            UsdGeom.Cylinder(lq).GetHeightAttr().Set(0.0)

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
        self.dropper.step(gripper_pos, opening)
        self._liquid_added = (self._drop_count
                              >= self.sample_cycles * self.DROPS_PER_SQUEEZE)
        self._update_experiment()
        return self.get_basic_state_info(additional_info={
            "phase": self.phase,
            "temperature": round(self.temperature, 1),
            "boiling_point": self.boiling_point,
            "flame_on": self.phase != "idle",
            "dropper_attached": self.dropper.attached,
            "dropper_filled": self.dropper.filled,
            "dropper_dropped": self.dropper.dropped,
        })

    # ------------------------------------------------------------------
    # 相态机：idle（等滴加完成）→ ignited → heating → boiling → done
    # ------------------------------------------------------------------
    def _update_experiment(self):
        if self.phase == "idle":
            if (self._liquid_added
                    and self.frame_idx >= 5 + self.idle_dwell_frames):
                self.phase = "ignited"
                self._set_visible(self._flame_paths(), True)
                print(f"[b2] ignite: flame on @ frame {self.frame_idx}")

        elif self.phase == "ignited":
            if self.frame_idx >= 5 + self.idle_dwell_frames + self.ignite_dwell_frames:
                self.phase = "heating"
                print(f"[b2] heating start T={self.temperature:.1f}")

        elif self.phase == "heating":
            self.temperature = min(self.boiling_point, self.temperature + self.heat_rate)
            if self.temperature >= self.boiling_point:
                self.phase = "boiling"
                self._set_visible(self.bubble_prims, True)
                self._set_visible(self.steam_prims, True)
                print(f"[b2] boiling at T={self.temperature:.1f}")

        elif self.phase == "boiling":
            self._boil_frames += 1
            self._animate_bubbles()
            self._animate_steam()
            if self._boil_frames >= self.boil_dwell_frames:
                self.phase = "done"
                print(f"[b2] done: boiling point {self.boiling_point:.1f}°C recorded")

        # 温度计读数每帧跟随温度
        self._set_capillary(self.temperature)

    def _animate_bubbles(self):
        # 气泡破灭高度 = 当前液面（滴加后液面从 0 长到 DROP_LEVEL_MAX，跟随实时）
        pop_z = self.TUBE_BOTTOM_Z + self._liquid_level - 0.002
        for p, base in zip(self.bubble_prims, self.bubble_base):
            t = self._read_translate(p)
            t[2] += self.BUBBLE_SPEED
            if t[2] > pop_z:
                t = list(base)
            self._set_translate(p, t)

    def _animate_steam(self):
        for p, base in zip(self.steam_prims, self.steam_base):
            t = self._read_translate(p)
            t[2] += self.STEAM_SPEED
            if t[2] > self.STEAM_CEIL:
                t = list(base)
            self._set_translate(p, t)

    # ------------------------------------------------------------------
    # 温度计读数：毛细柱锚定缩放（底 z=0.005 不动，柱顶随温度爬升）
    # ------------------------------------------------------------------
    def _z_of(self, T):
        return self.Z_LO + (T - self.T_MIN) / (self.T_MAX - self.T_MIN) * (self.Z_HI - self.Z_LO)

    def _set_capillary(self, T):
        prim = self.stage.GetPrimAtPath("/World/Thermometer/Thermometer/capillary_liquid")
        if not prim.IsValid():
            return
        s = (self._z_of(T) - self.CAP_BOTTOM) / self.CAP_FULL_H
        s = min(1.0, max(0.0, s))
        # M = T(0,0,-0.005) · S(1,1,s) · T(0,0,+0.005)：底锚 z=0.005，柱顶 = CAP_BOTTOM + CAP_FULL_H*s = z_of(T)
        S = Gf.Matrix4d().SetScale(Gf.Vec3d(1, 1, s))
        Td = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, -self.CAP_BOTTOM))
        Tu = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, self.CAP_BOTTOM))
        M = Td * S * Tu
        xf = UsdGeom.Xformable(prim)
        if self._cap_op is None:
            self._cap_op = xf.AddTransformOp()
        self._cap_op.Set(M)

    # ------------------------------------------------------------------
    # 滴管位姿 / 判定 / 滴落动画
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
        """DropperFill 截锥液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(self.DROPPER_FILL, tip)

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """任一滴加：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴（尖嘴下逐滴错落坠落）。

        挤胶头瞬间在尖嘴正下方生成一串亮蓝液滴（DropperDrop 父 Xform 的 Drop_0.._N 球，
        每滴一格），delay 错帧起落形成连续"滴-滴-滴"（液柱 60mm 很满，一挤该是一串滴
        不是一滴——d3l 用户 2026-08-14）。每滴落定后管内液面长高 DROP_LEVEL_STEP。
        """
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（管口上方 25mm，液滴可见坠落）
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._drop_count + i + 1
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
        self._drop_count += self.DROPS_PER_SQUEEZE
        self._set_visible(self.DROPPER_DROP, True)
        print(f"[b2] squeeze -> {self.DROPS_PER_SQUEEZE} drops spawned")

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
                self._set_visible(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_tube_level(d["level"], d["name"])
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visible(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visible(self.DROPPER_DROP, False)

    def _grow_tube_level(self, h, name):
        """液滴落定：管内液面长到高度 h（圆柱高+上移，底面贴管底），并记液面高。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self.TUBE_DROPS,
                (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z + h / 2))
        self._set_visible(self.TUBE_DROPS, True)
        self._liquid_level = h
        print(f"[b2] tube liquid level h={h:.3f}")

    # ------------------------------------------------------------------
    # 辅助（v1 保留）
    # ------------------------------------------------------------------
    def _flame_paths(self):
        return ["/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner"]

    def _children(self, root):
        prim = self.stage.GetPrimAtPath(root)
        if not prim.IsValid():
            return []
        return [str(c.GetPath()) for c in prim.GetChildren()
                if c.GetTypeName() in ("Sphere", "Cone", "Mesh", "Cylinder")]

    def _read_translate(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return [0.0, 0.0, 0.0]
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                return [float(v[0]), float(v[1]), float(v[2])]
        return [0.0, 0.0, 0.0]

    def _set_translate(self, path, t):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(*t))
        else:
            xf.AddTranslateOp().Set(Gf.Vec3d(*t))

    def _set_visible(self, paths, visible):
        if isinstance(paths, str):
            paths = [paths]
        for path in paths:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)

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

    def on_task_complete(self, success):
        print(f"[b2] episode done success={success} phase={self.phase} "
              f"dropper_dropped={self.dropper.dropped} "
              f"boiling_point={self.boiling_point:.1f}°C temp={self.temperature:.1f}°C")
        super().on_task_complete(success)
