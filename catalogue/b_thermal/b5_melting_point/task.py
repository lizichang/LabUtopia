# -*- coding: utf-8 -*-
"""B5 熔点测定（提勒管法）任务：夹毛细管端部拎起 → 插粉丘 → 放回 → 夹另一端拎起 → 震实。

用户 2026-08-31 逐字：「算了还是换一种方法吧，同时把表面皿的位置放回去；重新写：首先你夹
这个毛细管还是以同样的方式夹但是夹的位置变了……夹起来的时候因为重力水平细管就自动变成竖直
了就像把它拎起来了……然后再毛细管放回桌面（有倒在桌面上变成水平的）……第二次夹毛细管的 -x
端，还是拎起来，这样子自动数值就可以上下快速移动把粉末从一端搞到另一端」——弃程序化旋转，
改夹端部拎起自动竖直，装样段实现五段 task 生命周期（pivot 持握：夹点钉 TCP、管身绕夹点
摆转，两次夹取按 phase 切夹点）：
  ① PickCapillarySealedEnd  夹封口端(-X)拎起 → 开口端朝下（蘸粉准备）
  ② DipCapillaryIntoPowder  水平移到粉丘上方→竖直下探开口端沉入粉丘 5mm
  ③ ReturnCapillaryToTable  蘸粉后放回桌面，毛细管倒成水平（松爪）
  ④ PickCapillaryOpenEnd    夹开口端(+X)拎起 → 封口端朝下（抖粉准备）
  ⑤ TampCapillary           保持封口端朝下，竖直方向上下快速来回 10 次震实
挂温度计/入提勒管加热/观察熔点留待验收后接续。

驱动 prim（b5_melting_point.usd，scripts/gen_b5_scene.py 生成，2026-08-30 建）：
  /World/CapillaryTube  毛细管（熔点管 Ø1.5×100mm，rot(0,90,0) 后局部 +Z → 世界 +X 水平躺台面，
    闭口端=xform 原点 (0.1710,0.2704,0.813)、开口端 (0.2710,0.2704,0.813)；中心抬高 12mm 防夹爪扎桌面）
  /World/CapillaryTube/SamplePlug  管内样品柱（开口端内白粉柱，默认 invisible，蘸粉后 task 显示）
  /World/ThieleTube     提勒管（侧管 V 顶点=加热点，静态）
  /World/ParaffinOil    石蜡油载热液柱（静态，本就装在主管内腔）
  /World/SurfaceDish + SamplePowder  表面皿 (0.4433,0.1488,0.80) + 粉丘 (0.4451,0.1430)
  /World/Match          火柴（点火用，后续动作）
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from tasks.base_task import BaseTask
from .meta_actions.constants import (CAP_REST, GRASP_SEALED, GRASP_OPEN,
                                     SWING_THRESHOLD_Z, SWING_FRAMES,
                                     POWDER_XY, POWDER_TOP_Z, END_OFFSET,
                                     THERMO_REST, THERMO_GRASP,
                                     CAP_MID, STICK_SEALED)


class _CapillaryLifecycle:
    """毛细管状态机（rest → attached → released → rest，夹端部拎起 pivot 持握，三次夹取）。

    持握 = pivot 持握（夹端部拎起自动竖直）：夹点钉在 TCP（平移跟随，offset=抓点−TCP_attach），
    管身绕夹点（世界 Y 轴）摆转水平↔竖直 θ=swing_sign·90°·swing_frac。swing_frac 由 TCP
    高度阈值 SWING_THRESHOLD_Z 驱动（高于→竖直、低于→水平，每帧线性逼近），机械臂只纯竖直
    拎起、不旋转夹爪（用户 2026-09-01「机械臂就只是直上直下不要其他变化」+「拎起来毛细管
    要完全竖直」）。世界矩阵（Gf 行向量约定，已验证零跳变/精确 90°）：
        cap_world = M_rest · T(G_rest)⁻¹ · RotY(θ) · T(G(t))
        G(t) = TCP + offset（offset = 抓点 − TCP_attach，attach 时反解）
    三次夹取（用户 2026-08-31「夹端部拎起…放回…夹另一端」+ 2026-09-02「拎起一端…贴温度计」）：
    ① 夹封口端 GRASP_SEALED 蘸粉（_phase=0 → swing_sign=+1 开口端朝下），松爪放回 _phase→1；
    ② 夹开口端 GRASP_OPEN 抖粉（swing_sign=−1 封口端朝下），松爪放回 _phase→2；
    ③ 夹封口端 GRASP_SEALED 拎起竖直贴泡（swing_sign=+1 封口端朝上、开口端垂下），贴泡吸附后
    松爪 _phase→3（此后不再监听）。release 时 swing_frac 回 0、写回静止矩阵。
    参考点（gripper/TCP 世界坐标）：
      grasp_points [封口端 GRASP_SEALED, 开口端 GRASP_OPEN]（phase 0/2→封口端、1→开口端）
      rest    闭口端（xform 原点）台面静止位（CAP_REST）
    """

    def __init__(self, task, name, path, rest, grasp_points):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp_points = [np.array(g) for g in grasp_points]  # [封口端, 开口端]
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self._M_rest = None        # 管静止世界矩阵（attach 前从 task 读，缓存）
        self._grasp_offset = None  # 抓点相对 TCP 世界偏移（attach 时反解）
        self._swing_frac = 0.0     # 摆转进度 0=水平 1=完全竖直
        self._swing_sign = 1.0     # +1 封口端夹(开口端朝下) / −1 开口端夹(封口端朝下)
        self._phase = 0            # 0=第一次夹封口端蘸粉，1=第二次夹开口端抖粉，2=第三次夹封口端贴泡
        self._sample_shown = False  # 蘸粉后管内样品柱已显示（持久，reset 才清）

    @property
    def grasp(self):
        """当前夹点：phase 0/2 夹封口端(GRASP_SEALED)、phase 1 夹开口端(GRASP_OPEN)。"""
        return self.grasp_points[0 if self._phase in (0, 2) else 1]

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self._grasp_offset = None
        self._swing_frac = 0.0
        self._swing_sign = 1.0
        self._phase = 0
        self._sample_shown = False
        self.task._set_sample_visible(False)
        self.task._set_capillary_world(self.task._capillary_rest_matrix())

    def _rest_matrix(self):
        """毛细管静止世界矩阵（首次读取后缓存，避免与 task 初始化顺序耦合）。"""
        if self._M_rest is None:
            self._M_rest = self.task._capillary_rest_matrix()
        return self._M_rest

    def _pivot_matrix(self, gripper_pos):
        """夹端部拎起 pivot：夹点钉 TCP、管身绕夹点摆转。世界 = M_rest·T(G_rest)⁻¹·RotY(θ)·T(G)。

        G(t) = TCP + offset（offset = 抓点 − TCP_attach，纯平移下常量）。θ=swing_sign·90°·swing_frac。
        Gf 行向量约定：A*B = A 先 B 后；已验证零跳变（θ=0,G=G_rest → M_rest）与精确 90°。"""
        G_rest = self.grasp
        G = np.asarray(gripper_pos, dtype=float) + self._grasp_offset
        theta_deg = self._swing_sign * 90.0 * self._swing_frac
        T_G = Gf.Matrix4d(1.0)
        T_G.SetTranslateOnly(Gf.Vec3d(*G))
        T_Gr = Gf.Matrix4d(1.0)
        T_Gr.SetTranslateOnly(Gf.Vec3d(*G_rest))
        R = Gf.Matrix4d(1.0)
        R.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), theta_deg))
        return self._rest_matrix() * T_Gr.GetInverse() * R * T_G

    def _update_swing(self, gripper_pos):
        """摆转进度：TCP 高于阈值→竖直(→1)，低于→水平(→0)，每帧线性逼近一步，精确到 0/1。"""
        target = 1.0 if gripper_pos[2] > SWING_THRESHOLD_Z else 0.0
        step = 1.0 / SWING_FRAMES
        d = target - self._swing_frac
        if abs(d) <= step:
            self._swing_frac = target
        else:
            self._swing_frac += step if d > 0 else -step

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        # 三次夹取（封口端蘸粉→放回、开口端抖粉→放回、封口端拎起贴泡→吸附）完成后 phase=3，
        # 本 pivot 生命周期不再监听；中间 ⑦⑧ 中部水平蘸油由 _CapillaryHoldLifecycle 负责。
        if self._phase >= 3:
            return
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            # pivot 零跳变：attach 时 θ=0、G=G_rest → cap_world=M_rest，无需 easing，
            # 毛细管静置等夹爪闭合即可，闭合即 attached。
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self._grasp_offset = (np.asarray(self.grasp, dtype=float)
                                      - np.asarray(gripper_pos, dtype=float))
                self._swing_frac = 0.0
                self._swing_sign = 1.0 if self._phase in (0, 2) else -1.0
                self.task._set_capillary_world(self._pivot_matrix(gripper_pos))
                print(f"[b5] capillary attached (grip={opening:.4f} sign={self._swing_sign:+.0f})")
            return

        # 吸附期：更新摆转进度（TCP 高度阈值驱动），管身 pivot 跟随（夹点钉 TCP、绕夹点摆转）
        self._update_swing(gripper_pos)
        self.task._set_capillary_world(self._pivot_matrix(gripper_pos))
        # 蘸粉检测：phase0 夹封口端竖直后，开口端（夹点正下方 END_OFFSET）沉入粉丘顶以下
        # 且 x/y 对齐粉丘中心 → 管内样品柱显示（持久，reset 才清）。
        if self._phase == 0 and not self._sample_shown:
            grasp = np.asarray(gripper_pos, dtype=float) + self._grasp_offset
            free_end_z = grasp[2] - END_OFFSET
            if (free_end_z < POWDER_TOP_Z
                    and abs(grasp[0] - POWDER_XY[0]) < 0.02
                    and abs(grasp[1] - POWDER_XY[1]) < 0.02):
                self._sample_shown = True
                self.task._set_sample_visible(True)
                print(f"[b5] sample dipped: plug visible (open end z={free_end_z:.4f})")
        # 竖贴泡检测：phase2 夹封口端竖直后，封口端（=TCP+offset）贴近倒插泡 +Y 侧 STICK_SEALED
        # → 记录相对变换吸附（持久，reset 才清）。吸附后 _update_stuck_capillary 每帧接管位置。
        if self._phase == 2 and not self.task._capillary_stuck:
            sealed = np.asarray(gripper_pos, dtype=float) + self._grasp_offset
            if (abs(sealed[0] - STICK_SEALED[0]) < 0.012
                    and abs(sealed[1] - STICK_SEALED[1]) < 0.012
                    and abs(sealed[2] - STICK_SEALED[2]) < 0.012):
                self.task._stick_capillary()
                print("[b5] capillary sealed-end stuck to bulb (vertical)")
        # 松爪：写回台面静止位，复位 rest；swing_frac 回 0、_phase+1 进入下一次夹取。
        # grip 全程闭合（GRIP_CAPILLARY），不会误触发。
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_capillary_world(self.task._capillary_rest_matrix())
            self.state = "rest"
            self._swing_frac = 0.0
            self._phase += 1
            print(f"[b5] capillary released to rest (phase -> {self._phase})")


class _ThermometerLifecycle:
    """温度计状态机（rest → attached → released → rest，矩阵持握，泡朝下）。

    2026-09-02 用户定案「可以倒着插温度计」：温度计倒插试管架（泡朝上，见 gen_b5_scene.py），
    手指朝前 ORIENT_FWD 水平横夹竖直杆身（d2s 夹药匙同款，夹点 THERMO_GRASP），竖直提出后
    只用法兰（panda_joint7）滚 FLANGE_ANGLE=−166°（限位 ±166°）把泡翻朝下，再 IK 校直剩余
    ~14°（ORIENT_VERT=Rx(180°)·ORIENT_FWD 泡朝下精确朝向）。温度计 6-DOF 刚性跟随夹爪
    （矩阵持握，同 b2 _THERMO_HELD）。单次夹取。release 时写回静止矩阵。
    吸附（毛细管贴泡）不再在此检测——改由 _CapillaryHoldLifecycle（毛细管持水平移到泡处贴）
    触发 task._stick_capillary()。
    """

    # 持握矩阵不硬编码：attach 时从 ground truth 反解 self._held = 静止矩阵 · tool_world⁻¹，
    # 保证任何底座朝向下零跳变。2026-09-01 用户报「抓住一瞬间温度计瞬移，玻璃泡那一端直接
    # 瞬移到爪子上」根因 = 旧硬编码 _THERMO_HELD（行 [0 0 1]/[0 1 0]/[-1 0 0]）假设手指朝下时
    # tool+X=世界-X、tool+Z=世界-Z，但实测 tool_center 世界旋转 = RotZ(-35°)·diag(1,-1,-1)
    # （底座带 ~35° 偏航），与假设差 ~145° 绕 tool-X，attach 即瞬移、后续旋转几圈对不齐管口。
    # 逐帧反解捕获后：attach 时 _held·tool_world = 静止矩阵（零跳变），持握期 6-DOF 刚性跟随
    # 夹爪（原地旋转 ORIENT_VERT 时温度计随之竖直泡朝下，同 pivot 生命周期 attach 反解 offset）。

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
        self._M_rest = None
        self._held = None          # attach 时反解的持握矩阵（rest · tool_world⁻¹）

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self._held = None
        self.task._set_thermometer_world(self.task._thermometer_rest_matrix())

    def _rest_matrix(self):
        if self._M_rest is None:
            self._M_rest = self.task._thermometer_rest_matrix()
        return self._M_rest

    def _held_matrix(self):
        """矩阵持握：温度计世界 = _held · tool_world（6-DOF 刚性跟随夹爪，随夹爪旋转）。"""
        return self._held * self.task._tool_world()

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            # 零跳变反解：attach 时温度计仍静止，_held = 静止矩阵 · tool_world⁻¹ →
            # _held·tool_world = 静止矩阵。无需 easing，闭合即 attached。
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self._held = self._rest_matrix() * self.task._tool_world().GetInverse()
                self.task._set_thermometer_world(self._held_matrix())
                print(f"[b5] thermometer attached (grip={opening:.4f}, matrix hold zero-jump)")
            return

        # 持握期：6-DOF 刚性跟随夹爪（夹爪原地旋转立起来时温度计随之竖直泡朝下）
        self.task._set_thermometer_world(self._held_matrix())
        # 松爪：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_thermometer_world(self.task._thermometer_rest_matrix())
            self.state = "rest"
            print("[b5] thermometer released to rest")


class _CapillaryHoldLifecycle:
    """毛细管水平持握状态机（rest → held → rest，矩阵持握，中部夹起蘸油后放回）。

    2026-09-02 用户改流程定案：震实放回桌面后，从毛细管**中部**水平夹起保持水平（矩阵持握，
    非 pivot 竖直）→ 移到油皿封口端蘸油 → **放回桌面**。贴泡改由 pivot 生命周期第三次夹取
    （夹封口端拎起竖直贴泡，见 _CapillaryLifecycle phase=2 + task._stick_capillary）。
    持握矩阵不硬编码：attach 时从 ground truth 反解 self._held = 静止矩阵 · tool_world⁻¹
    （夹中部零跳变）。封口端世界 = _held_matrix().Transform((0,0,0)) = 夹点 − 0.05·世界+X
    ——attach 时反解捕获，落点与底座偏航无关（旧硬编码 _CAP_HELD 假设手指朝下 tool+X→−X，
    实际 tool_center 带 ~35° 底座偏航，2026-09-01 用户报「沾油爪子没到皿中心、只有一段沾上油」
    根因）。单次夹取（⑦ 夹中部 → ⑧ 蘸油 → ⑧' 放回）。
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
        self._M_rest = None
        self._held = None          # attach 时反解的持握矩阵（rest · tool_world⁻¹）

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self._held = None
        self.task._set_capillary_world(self.task._capillary_rest_matrix())

    def _rest_matrix(self):
        if self._M_rest is None:
            self._M_rest = self.task._capillary_rest_matrix()
        return self._M_rest

    def _held_matrix(self):
        """矩阵持握：毛细管世界 = _held · tool_world（水平 6-DOF 刚性跟随夹爪）。"""
        return self._held * self.task._tool_world()

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            # 零跳变反解：attach 时毛细管仍静止，_held = 静止矩阵 · tool_world⁻¹ →
            # _held·tool_world = 静止矩阵。无需 easing，闭合即 held。
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "held"
                self.attached = True
                self._held = self._rest_matrix() * self.task._tool_world().GetInverse()
                self.task._set_capillary_world(self._held_matrix())
                print(f"[b5] capillary middle-held (grip={opening:.4f}, matrix hold zero-jump)")
            return

        if self.state == "held":
            # 持握期：水平 6-DOF 刚性跟随夹爪（横移蘸油全程保持水平）
            self.task._set_capillary_world(self._held_matrix())
            # 松爪（⑧' 蘸油后放回桌面）：写回原位复位 rest（贴泡检测已移至 pivot phase2）
            if opening > self.task.gripper_open_threshold:
                self.released = True
                self.task._set_capillary_world(self.task._capillary_rest_matrix())
                self.state = "rest"
                print("[b5] capillary middle-hold released to rest")
            return


class B5MeltingPointTask(BaseTask):
    """B5 熔点测定（提勒管法）任务：拿起毛细管 → 转竖直 → 插入粉丘。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    CAPILLARY = "/World/CapillaryTube"
    SAMPLE_PLUG = "/World/CapillaryTube/SamplePlug"
    THERMOMETER = "/World/MainThermometer"

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.capillary_path = self.CAPILLARY
        # 毛细管是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰），
        # 与 d2s/d3s/b2 同模式。
        self._disable_collision(self.capillary_path)
        self._disable_collision(self.THERMOMETER)

        # 阈值（config 可调，b1/b2 同款默认）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        # 毛细管静止矩阵（首次从 stage 读，reset 写回用；rotY90 + CAP_REST）
        self._cap_rest_matrix = self._get_capillary_world_matrix()
        # 温度计静止矩阵（倒插试管架泡朝上 + THERMO_REST）
        self._thermo_rest_matrix = self._get_thermometer_world_matrix()

        # 毛细管生命周期（pivot 持握；两次夹取：封口端蘸粉 → 放回 → 开口端抖粉）
        self.capillary = _CapillaryLifecycle(self, "capillary", self.CAPILLARY,
                                             CAP_REST, [GRASP_SEALED, GRASP_OPEN])
        # 毛细管水平持握生命周期（矩阵持握；单次夹中部蘸油贴泡）
        self.capillary_hold = _CapillaryHoldLifecycle(self, "capillary_hold", self.CAPILLARY,
                                                      CAP_REST, CAP_MID)
        # 温度计生命周期（矩阵持握；单次横夹倒插竖直杆身，法兰翻转泡朝下）
        self.thermometer = _ThermometerLifecycle(self, "thermometer", self.THERMOMETER,
                                                 THERMO_REST, THERMO_GRASP)
        # 毛细管吸附标记（⑨ 泡贴封口端后置位，随温度计整组移动，reset 清）+ 相对变换
        self._capillary_stuck = False
        self._capillary_stick_rel = None

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.capillary.reset()
        self.capillary_hold.reset()
        self.thermometer.reset()
        self._capillary_stuck = False
        self._capillary_stick_rel = None

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self.capillary.step(gripper_pos, opening)          # ①-⑥ + ⑨'⑩ 毛细管（pivot 拎起蘸粉/抖粉/竖贴泡）
        self.capillary_hold.step(gripper_pos, opening)     # ⑦⑧⑧' 毛细管中部水平持握蘸油后放回
        self.thermometer.step(gripper_pos, opening)        # ⑪⑫ 温度计（矩阵持握，法兰滚166°泡朝下）
        self._update_stuck_capillary()                     # ⑩ 吸附后毛细管随温度计整组移动
        return self.get_basic_state_info(additional_info={
            "capillary_state": self.capillary.state,
            "capillary_attached": self.capillary.attached,
            "capillary_released": self.capillary.released,
            "capillary_hold_state": self.capillary_hold.state,
            "capillary_hold_attached": self.capillary_hold.attached,
            "thermometer_state": self.thermometer.state,
            "thermometer_attached": self.thermometer.attached,
            "capillary_stuck": self._capillary_stuck,
        })

    def on_task_complete(self, success):
        print(f"[b5] episode done success={success} "
              f"capillary={self.capillary.state} attached={self.capillary.attached}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 毛细管矩阵持握（药匙/试管同款：清 op 序写单一 transform op，随矩阵旋转）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _capillary_rest_matrix(self):
        """毛细管台面静止矩阵（__init__ 读的 stage 初始值：rotY90 + CAP_REST）。"""
        return self._cap_rest_matrix

    def _get_capillary_world_matrix(self):
        """毛细管当前世界 4x4 矩阵。"""
        prim = self.stage.GetPrimAtPath(self.CAPILLARY)
        if not prim.IsValid():
            return None
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_capillary_world(self, world_matrix):
        """把毛细管写到给定世界矩阵（清 op 序 + AddTransformOp，6-DOF 随矩阵旋转）。"""
        prim = self.stage.GetPrimAtPath(self.CAPILLARY)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _set_sample_visible(self, visible):
        """管内样品柱可见性开关（蘸粉后显示）。样品柱是 CapillaryTube 子 prim，
        随管 pivot 矩阵一起动，此处只切 visibility。"""
        prim = self.stage.GetPrimAtPath(self.SAMPLE_PLUG)
        if not prim.IsValid():
            return
        UsdGeom.Imageable(prim).GetVisibilityAttr().Set(
            UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)

    # ------------------------------------------------------------------
    # 温度计矩阵持握 + 毛细管吸附（与毛细管同款：清 op 序写单一 transform op）
    # ------------------------------------------------------------------
    def _get_thermometer_world_matrix(self):
        prim = self.stage.GetPrimAtPath(self.THERMOMETER)
        if not prim.IsValid():
            return None
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_thermometer_world(self, world_matrix):
        prim = self.stage.GetPrimAtPath(self.THERMOMETER)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _thermometer_rest_matrix(self):
        """温度计台面静止矩阵（__init__ 读的 stage 初始值：倒插试管架泡朝上 + THERMO_REST）。"""
        return self._thermo_rest_matrix

    def _stick_capillary(self):
        """吸附：毛细管封口端粘上温度计泡（记录相对变换，持久，reset 才清）。"""
        if self._capillary_stuck:
            return
        cap_world = self._get_capillary_world_matrix()
        thermo_world = self._get_thermometer_world_matrix()
        if cap_world is None or thermo_world is None:
            return
        # 记录毛细管相对温度计的变换（Gf 行向量：cap_world = stick_rel · thermo_world）。
        # 此时温度计倒插试管架、毛细管竖直封口端贴泡，之后温度计法兰翻转竖直/插管全程相对不变。
        self._capillary_stick_rel = cap_world * thermo_world.GetInverse()
        self._capillary_stuck = True
        print("[b5] capillary stuck to thermometer bulb (rel transform recorded)")

    def _update_stuck_capillary(self):
        """吸附后每帧把毛细管钉在温度计上（相对变换），随温度计整组移动（旋转竖直/插管全程贴泡）。"""
        if not self._capillary_stuck or self._capillary_stick_rel is None:
            return
        thermo_world = self._get_thermometer_world_matrix()
        if thermo_world is None:
            return
        self._set_capillary_world(self._capillary_stick_rel * thermo_world)

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    # ------------------------------------------------------------------
    # 辅助（d2s/d3s/b2 同款）
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
