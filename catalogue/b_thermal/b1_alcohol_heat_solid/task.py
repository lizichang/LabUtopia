# -*- coding: utf-8 -*-
"""B1 酒精灯加热（固体样品）任务：药匙挖粉倒进试管 → 打开灯帽 → 取火柴点燃酒精灯。

用户 2026-08-27 逐字：「先写咬粉末咬进试管里面，然后拿起酒精灯盖儿放到一边儿，
再拿起火柴点燃酒精灯，这几个过程先只写这些我来验收。」——本批次只实现这三个
过程的 task 生命周期；拿试管→外焰预热（倾斜 10-15°）→集中加热→熄灭→归位留待验收后。

用户先决条件（场景复刻 D2S 挖粉坐标）：「表面皿、粉末和机械臂坐标一定要复刻 D2S
这样粉末才能挖得准；去掉试管夹，用机械臂的夹爪直接代替。」→ 挖粉逐字复用 d2s
元动作（PickSpatula/ReturnSpatula 默认参数即 d2s 家用），本任务管三块生命周期：

  1) 药匙（d3s 同款）：每帧把药匙世界位姿写为 _T_HELD · tool_world（6-DOF 随夹爪旋转）。
     勺尖 = 夹爪 + 0.134·tool+X，挖粉/倒粉判定照 d2s（_scoop_starting/_vertical_over_mouth），
     粉末下落动画（PowderDrop 父 + 14 粒）与 d2s 逐字一致；⑬ 倒粉完成 show TubePowder
     （/World/TubePowder，白色粉末柱，gen BUILTIN 预摆 rest 中心 0.84）。
  2) 灯帽（新，纯平移持握）：/World/AlcoholLamp/cap 是灯的子 prim（ops=translate+
     rotateXYZ(90)+scale(0.01)，lamp 恒等旋转），帽中心世界 = 灯原心 + local_t +
     (0,0,0.09152)。持握 = 帽中心 = 夹爪（纯平移：task 写帽 local translate =
     center − CAP_CENTER_REST，姿态不变）。生命周期 rest → attached → released →
     rest：rest 在灯口（local_t=0）；attached 跟随夹爪；松爪须同时满足「夹爪近
     CAP_ASIDE_TCP」+「开爪 > open」（类沸石，防下探未到位提前释放帽悬空）→ released
     写回放一边静止中心（帽底贴台面 0.80）。
  3) 火柴（b2 逐字）：纯平移 offset（MATCH_HELD_OFFSET），火柴全程水平头朝 +X。
     点火检测 _step_match_ignite：火柴头（夹爪 + MATCH_TIP_OFFSET）近灯芯 WICK 连续
     MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit=True，B1 无温度模型 → 直接 reveal
     flame_outer/flame_inner。

驱动 prim（b1_alcohol_heat_solid.usd，scripts/gen_b1_scene.py 生成，2026-08-27 重建含
PowderDrop 14 粒）：
  /World/Spatula / SurfaceDish / SamplePowder / TestTubeRack / TestTube  挖粉同 d2s
  /World/AlcoholLamp/cap  灯帽（纯平移持握）/ flame_outer|flame_inner 火焰（初始隐藏）
  /World/Match  火柴（头朝灯芯，抬高 12mm）
  /World/PowderOnSpoon / PowderDrop / TubePowder  药匙上粉堆 / 药粉下落 / 管内白粉柱
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_SPATULA, SPAT_HEAD_DIST,
    POWDER_TOP_Z, POWDER_X, DISH_XY,
    TUBE_XY, TUBE_MOUTH_Z,
    CAP_GRASP, CAP_CENTER_REST, CAP_ASIDE_TCP, CAP_ASIDE_CENTER,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
)

# 药匙相对夹爪持握矩阵（d3s/d2s 同款）：平移 (0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、
# toolZ→(-1,0,0)）。平移在最后一行（USD 行向量约定）。合成 = _T_HELD · tool_world（先作用药匙
# 局部系、再 tool_world 到世界；写反旋转作用到世界系 → 药匙翻走/飞桌面下）。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)

# 试管内粉末柱 rest 中心（gen_b1_scene BUILTIN 预摆：/World/TubePowder translate z=0.84）
TUBE_POWDER_REST = np.array([TUBE_XY[0], TUBE_XY[1], 0.84])
TUBE_POWDER_H = 0.012


class _CapLifecycle:
    """灯帽状态机（rest → attached → released → rest，纯平移持握）。

    持握 = 纯平移：帽中心 = 夹爪（task._set_cap_center 写帽 local translate =
    center − CAP_CENTER_REST，只动 translate op，姿态恒等不随夹爪旋转）。这与药匙的
    矩阵持握不同——帽是盖在灯口的钟罩，从灯口竖直端起再横移到放一边，全程帽姿态不变。
    释放时写回放一边静止中心（帽底贴台面 0.80），之后不再跟手。
    参考点（gripper/TCP 世界坐标）：
      grasp        灯口帽中心抓点（CAP_GRASP，TCP=帽中心，两指夹帽壁）
      rest_center  帽盖在灯上的静止中心（CAP_CENTER_REST，local_t=0）
      aside_tcp    放一边下探 TCP（CAP_ASIDE_TCP = 帽中心落位，帽底贴台面）
    """

    def __init__(self, task, name, path, rest_center, grasp, aside_tcp):
        self.task = task
        self.name = name
        self.path = path
        self.rest_center = np.array(rest_center)
        self.grasp = np.array(grasp)
        self.aside_tcp = np.array(aside_tcp)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.task._set_cap_center(self.rest_center)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            # 夹爪开始合拢且已进近窗：先把帽中心平滑拉向持握位（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_cap_center(gripper_pos)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_cap_center(gripper_pos)
                print(f"[b1] cap attached (grip={opening:.4f})")
            return

        # 吸附期：帽中心 = 夹爪（纯平移跟随）
        self.task._set_cap_center(gripper_pos)
        # 松爪判定：夹爪到放一边下探位且 grip 打开（⑧）→ 帽写回 aside 静止中心，复位 rest
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.aside_tcp, gripper_pos)):
            self.released = True
            self.task._set_cap_center(self.task._cap_aside_center())
            self.state = "rest"
            print(f"[b1] cap released to aside -> rest")


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段③取火柴点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （b2 逐字；火柴杆横躺，夹爪手指朝下竖直夹其杆身）。释放时写回台面静止位。
    参考点（gripper/TCP 世界坐标）：
      grasp   杆身中部抓点（MATCH_GRASP）
      rest    火柴原点台面静止位（MATCH_XY + MATCH_REST_Z）
    """

    def __init__(self, task, name, path, rest, grasp):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp = np.array(grasp)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.task._set_match_world(self.task._match_rest_pos())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_match_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_match_world(held)
                print(f"[b1] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位 MATCH_LIFT_Z）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[b1] match released to rest")


class B1AlcoholHeatSolidTask(BaseTask):
    """B1 酒精灯加热（固体样品）任务：药匙挖粉倒进试管 → 打开灯帽 → 取火柴点燃酒精灯。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 药匙（d2s/d3s 同款）
    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array([0.6993, 0.3608, 0.94])   # d2s 家用（B1 场景逐字）
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量
    POWDER_EFFECT = "/World/PowderOnSpoon"
    POWDER_DROP = "/World/PowderDrop"
    POWDER_DROPS = 14
    POWDER_STAGGER = 3
    POWDER_HANG = 4
    POWDER_FALL = 14
    POWDER_LAND_Z = 0.84   # 药粉落点 = 管内白粉柱中心（/World/TubePowder rest 0.84）
    TUBE_SAMPLE = "/World/TubePowder"   # 管内白色粉末柱（⑬ 倒粉后 reveal）

    # 灯帽（纯平移持握）
    CAP = "/World/AlcoholLamp/cap"
    CAP_MESH_OFFSET = np.array([0.0, 0.0, 0.09152])   # 帽 mesh 中心相对帽局部原点（R90+S0.01）

    # 火柴（b2 逐字）/ 火焰
    MATCH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest/b2）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.spatula_path = self.SPATULA_PATH
        # 药匙/灯帽/火柴是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被
        # 物理干扰），与 d2s/d3s/b2 同模式。
        self._disable_collision(self.spatula_path)
        self._disable_collision(self.CAP)
        self._disable_collision(self.MATCH)

        # 阈值（config 可调，d2s/B2 同款默认）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        # 药匙状态（d2s/d3s 同款）
        self._near_frames = 0
        self.spatula_state = "rest"     # rest / attached / released
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None        # 上一帧法兰角（joint7 索引 6），判定⑨挖粉旋转开始

        # 灯帽生命周期（纯平移持握）
        self.cap = _CapLifecycle(
            self, "cap", self.CAP, CAP_CENTER_REST, CAP_GRASP, CAP_ASIDE_TCP)

        # 火柴生命周期（b2 逐字）
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH, match_rest, MATCH_GRASP)
        self.flame_lit = False         # 火柴触灯芯点燃（点火即 reveal 火焰）
        self.match_ignite_counter = 0  # 火柴头近灯芯连续帧计数

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        # 药匙复位（d2s/d3s 同款）
        self.spatula_state = "rest"
        self._near_frames = 0
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None
        self._set_spatula_world(_spatula_rest_matrix())
        self._set_visibility(self.POWDER_EFFECT, False)
        self._set_visibility(self.POWDER_DROP, False)
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        # 还原粉堆尺寸（上一集 _shrink_powder_blob 缩到 12%）
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005)
            cyl.GetHeightAttr().Set(0.005)
        # 试管内白粉柱复位：回 rest、还原尺寸、隐藏（下一集重新倒粉）
        self._set_tube_column(self.TUBE_SAMPLE, TUBE_POWDER_H, TUBE_POWDER_REST, r=0.006)
        self._set_visibility(self.TUBE_SAMPLE, False)
        # 灯帽复位：回灯口（local_t=0）
        self.cap.reset()
        # 火柴复位：回台面静止位
        self.match.reset()
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._set_visibility(self._flame_paths(), False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_spatula()              # ① 药匙挖粉/倒粉（d2s/d3s 同款）
        self._step_powder_anim()            # 药粉下落动画独立推进
        self.cap.step(gripper_pos, opening)          # ② 灯帽取放（纯平移）
        self.match.step(gripper_pos, opening)        # ③ 火柴取放（b2）
        self._step_match_ignite(gripper_pos)         # ③ 点火检测（火柴头触灯芯 → flame_lit）
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "powder_on_spoon": self.powder_on_spoon,
            "poured": self.poured,
            "cap_attached": self.cap.attached,
            "cap_released": self.cap.released,
            "match_attached": self.match.attached,
            "flame_lit": self.flame_lit,
        })

    def on_task_complete(self, success):
        print(f"[b1] episode done success={success} "
              f"spatula={self.spatula_state} poured={self.poured} "
              f"cap_released={self.cap.released} "
              f"match={self.match.state} flame_lit={self.flame_lit}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 药匙持握 / 效果（d2s/d3s 同款）
    # ------------------------------------------------------------------
    def _update_spatula(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        # 法兰（joint7，索引 6）是否在旋转：⑨ 挖粉起判定信号
        flange_rotating = (self._prev_flange is not None
                           and abs(joints[6] - self._prev_flange) > 0.005)
        self._prev_flange = float(joints[6])

        if self.spatula_state == "rest":
            if self._near_grasp(gripper_pos, self.SPAT_GRASP):
                self._near_frames += 1
            else:
                self._near_frames = 0
            if self._near_grasp(gripper_pos, self.SPAT_GRASP) and opening < self.gripper_open_threshold:
                self._ease_spatula_to_gripper(gripper_pos)
            if (self._near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.SPAT_GRIP_CLOSED):
                self.spatula_state = "attached"
                self._set_spatula_from_gripper()
                print(f"[b1] spatula attached (grip={opening:.4f})")

        elif self.spatula_state == "attached":
            self._set_spatula_from_gripper()
            tip = self._spoon_tip_pos(gripper_pos)
            if not self.powder_on_spoon and self._scoop_starting(tip, flange_rotating):
                self.powder_on_spoon = True
                self._set_visibility(self.POWDER_EFFECT, True)
                print(f"[b1] powder on spoon (tip={np.round(tip, 3)})")
            if self.powder_on_spoon and not self.poured:
                self.object_utils.set_object_position(
                    self.POWDER_EFFECT, tip + np.array([0.0, 0.0, 0.003]))
                if not self.powder_falling and self._vertical_over_mouth(tip, joints):
                    self.powder_falling = True
                    self._start_powder_fall(tip)
            if opening > self.gripper_open_threshold:
                self.spatula_state = "released"
                self._set_spatula_world(_spatula_rest_matrix())
                self._set_visibility(self.POWDER_EFFECT, False)
                print("[b1] spatula released to rack")

    def _set_spatula_from_gripper(self):
        self._set_spatula_world(_T_HELD * self._tool_world())

    def _set_spatula_world(self, world_matrix):
        prim = self.stage.GetPrimAtPath(self.spatula_path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _ease_spatula_to_gripper(self, gripper_pos, k=0.18):
        target = _T_HELD * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.spatula_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_spatula_world(_blend_world(cur, target, k))

    def _spoon_tip_pos(self, gripper_pos):
        """勺尖 = 夹爪 + 0.134 × 夹爪局部 +X（勺头方向）世界方向。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]
        return np.asarray(gripper_pos, dtype=float) + SPAT_HEAD_DIST * x_dir

    def _scoop_starting(self, tip, flange_rotating):
        """⑨ 法兰开始旋转（挖粉起）判定：法兰正在旋转 且 勺尖在粉丘附近（松带）。"""
        near = (abs(tip[0] - POWDER_X) < 0.04
                and abs(tip[1] - DISH_XY[1]) < 0.08
                and tip[2] < POWDER_TOP_Z + 0.02)
        return flange_rotating and near

    def _vertical_over_mouth(self, tip, joints):
        """勺尖水平近管口且已降到回卷中段高度 → 开始倒粉（⑬ 进行到约一半掉入试管）。"""
        above = tip[2] < TUBE_MOUTH_Z + 0.05
        near = np.linalg.norm(tip[:2] - np.array([TUBE_XY[0], TUBE_XY[1]])) < 0.06
        return above and near

    def _start_powder_fall(self, tip):
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        start = tip + np.array([0.0, 0.0, -0.004])
        for i in range(self.POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * self.POWDER_STAGGER,
                "t": 0,
                "start": start.copy(),
                "target": np.array([TUBE_XY[0], TUBE_XY[1], self.POWDER_LAND_Z]),
                "hang": self.POWDER_HANG, "fall": self.POWDER_FALL,
            })
        self._set_visibility(self.POWDER_DROP, True)
        print(f"[b1] powder fall started from {np.round(start, 3)}")

    def _step_powder_anim(self):
        if not self._powder_queue:
            return
        remaining = []
        landed = self.POWDER_DROPS - len(self._powder_queue)
        for d in self._powder_queue:
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
                self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", False)
                landed += 1
                continue
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.POWDER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._powder_queue = remaining
        self._shrink_powder_blob(landed / self.POWDER_DROPS)
        if not remaining:
            self._set_visibility(self.POWDER_DROP, False)
            if self.powder_falling:
                self.powder_falling = False
                if not self.poured:
                    self.poured = True
                    self._set_visibility(self.POWDER_EFFECT, False)
                    self._set_visibility(self.TUBE_SAMPLE, True)
                    print("[b1] powder poured into tube")

    def _shrink_powder_blob(self, landed_frac):
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005 * remain)
            cyl.GetHeightAttr().Set(0.005 * remain)

    # ------------------------------------------------------------------
    # 灯帽纯平移持握（cap 是灯的子 prim，local translate = center − CAP_CENTER_REST）
    # ------------------------------------------------------------------
    def _cap_aside_center(self):
        return np.array(CAP_ASIDE_CENTER, dtype=float)

    def _set_cap_center(self, center):
        """帽中心落到 center（世界）：写帽 local translate = center − CAP_CENTER_REST。
        只动现有 translate op（帽 ops = translate+rotateXYZ(90)+scale(0.01)，姿态不变）。"""
        local_t = np.asarray(center, dtype=float) - np.array(CAP_CENTER_REST, dtype=float)
        prim = self.stage.GetPrimAtPath(self.CAP)
        if not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*local_t))
                return
        xf.AddTranslateOp().Set(Gf.Vec3d(*local_t))

    def _cap_center(self):
        """帽当前世界中心 = cap 世界矩阵平移 + CAP_MESH_OFFSET（读回 easing 用）。"""
        prim = self.stage.GetPrimAtPath(self.CAP)
        if not prim.IsValid():
            return np.array(CAP_CENTER_REST, dtype=float)
        wm = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(wm.ExtractTranslation()) + self.CAP_MESH_OFFSET

    def _ease_cap_center(self, target, k=0.18):
        """夹爪合拢期间帽中心逐帧平滑移向持握位（消除闭合瞬间闪现吸附）。"""
        cur = self._cap_center()
        nxt = cur + (np.asarray(target, dtype=float) - cur) * k
        self._set_cap_center(nxt)

    # ------------------------------------------------------------------
    # 火柴纯平移持握（b2 逐字：火柴横躺水平头朝 +X，只跟夹爪平移不随旋转）
    # ------------------------------------------------------------------
    def _match_rest_pos(self):
        """火柴原点台面静止位（MATCH_XY + MATCH_REST_Z）。"""
        return np.array([MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z])

    def _set_match_world(self, position):
        """把火柴写到给定世界位置（纯平移，火柴水平头朝 +X 姿态不变）。"""
        self._set_obj_world(self.MATCH, position)

    def _ease_match_world(self, target, k=0.18):
        """夹爪合拢期间火柴逐帧平滑移向持握位（消除闪现吸附）。"""
        self._ease_obj_world(self.MATCH, target, k)

    def _match_tip(self, gripper_pos):
        """火柴头中心世界坐标 = 夹爪 + MATCH_TIP_OFFSET（头在夹爪 +X 0.0494，水平朝前）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)

    def _step_match_ignite(self, gripper_pos):
        """点火检测（仿 flametest/b2）：火柴 attached 期间头近灯芯连续
        MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit=True。B1 无温度模型 → 置位即直接
        reveal 火焰（flame_outer/flame_inner）。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    self._set_visibility(self._flame_paths(), True)
                    print(f"[b1] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    # ------------------------------------------------------------------
    # 辅助（d2s/d3s/b2 同款）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _get_obj_world(self, path):
        """物体世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保姿态）。"""
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

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _flame_paths(self):
        return ["/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner"]

    def _set_tube_column(self, path, h, center, r=None):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetHeightAttr().Set(h)
            if r is not None:
                cyl.GetRadiusAttr().Set(r)
        self.object_utils.set_object_position(path, np.asarray(center, dtype=float))

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
        """设置单个 prim（或 prim 路径列表）可见性。

        2026-08-27 修「火焰不显示」：调用方传 _flame_paths() 列表，而
        GetPrimAtPath 只收字符串 → ArgumentError 被 except 吞掉 → 永不 reveal。
        改为统一按列表遍历，单字符串也兼容。
        """
        paths = path if isinstance(path, (list, tuple)) else [path]
        for p in paths:
            try:
                prim = self.stage.GetPrimAtPath(p)
                if prim.IsValid():
                    set_prim_visibility(prim, visible)
            except Exception:
                pass


def _spatula_rest_matrix():
    """药匙架内竖插位姿（d2s 家用）：与世界 /World/Spatula 矩阵一致
    (translate (0.6993,0.3608,0.828)，rotateXYZ(0,0,-180) 烘平后即下行序)。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.6993, 0.3608, 0.828, 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
