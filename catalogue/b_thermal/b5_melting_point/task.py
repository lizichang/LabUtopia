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
import math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from tasks.base_task import BaseTask
from .meta_actions.constants import (CAP_REST, GRASP_SEALED, GRASP_OPEN,
                                     SWING_THRESHOLD_Z, SWING_FRAMES,
                                     POWDER_XY, POWDER_TOP_Z, END_OFFSET,
                                     THERMO_REST, THERMO_GRASP,
                                     CAP_MID, STICK_SEALED,
                                     TUBE_XY, INSERT_THERMO_ORIGIN_Z, DROP_FRAMES,
                                     MATCH_XY, MATCH_REST_Z, MATCH_GRASP,
                                     MATCH_HELD_OFFSET, MATCH_TIP_OFFSET,
                                     WICK, FLAME_GRPS, FLAME_PRIMS,
                                     LAMP_XY, LAMP_REST_Z, LAMP_GRASP_OFFSET,
                                     LAMP_GRASP, LAMP_CLOSED_THRESHOLD,
                                     LAMP_OPEN_THRESHOLD, LAMP_TARGET,
                                     HEAT_SWAY_AMP, HEAT_SWAY_CYCLES, HEAT_SWAY_PERIOD,
                                     LAMPCAP_CENTER_DZ, LAMPCAP_REST, LAMPCAP_GRASP,
                                     LAMPCAP_HIGH, LAMPCAP_HELD_OFFSET, LAMPCAP_BURNER,
                                     LAMPCAP_CLOSED_THRESHOLD, LAMPCAP_COVER_NEAR,
                                     LAMPCAP_EXTINGUISH_XY, LAMPCAP_EXTINGUISH_Z)


# 酒精灯相对夹爪的持握矩阵 _LAMP_HELD（⑭⑮ 夹灯加热摆动 + 移灯，照 B2/B3）。
# 旋转与 _T_HELD 同款（灯 Z 朝上→tool -X、X→tool -Z、Y→tool -Y），平移 = LAMP_GRASP_OFFSET
# 0.0448（抓点 z=0.845 − 灯原点 z=0.8002）。Orient_FWD 手指朝前下 tool+X=世界-Z → tool -X=世界
# +Z，故灯局部 +Z（朝上）映射世界 +Z，灯保持竖直 R180 与场景一致 → 零跳变。平移沿 tool+X → 灯
# 原点在夹爪正下方 0.0448。行向量：对象世界 = _LAMP_HELD · tool_world（先夹爪后灯相对夹爪）。
_LAMP_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                         0.0, -1.0, 0.0, 0.0,
                         -1.0, 0.0, 0.0, 0.0,
                         LAMP_GRASP_OFFSET, 0.0, 0.0, 1.0)


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
    """温度计状态机（rest → attached → dropping → inserted，矩阵持握，泡朝下）。

    2026-09-02 用户定案「可以倒着插温度计」：温度计倒插试管架（泡朝上，见 gen_b5_scene.py），
    手指朝前 ORIENT_FWD 水平横夹竖直杆身（d2s 夹药匙同款，夹点 THERMO_GRASP），竖直提出后
    只用法兰（panda_joint7）滚 FLANGE_ANGLE=−166°（限位 ±166°）把泡翻朝下，再 IK 校直剩余
    ~14°（ORIENT_VERT=Rx(180°)·ORIENT_FWD 泡朝下精确朝向）。温度计 6-DOF 刚性跟随夹爪
    （矩阵持握，同 b2 _THERMO_HELD）。单次夹取。
    松爪（release）不再写回静止矩阵——改触发**落体**（用户 2026-09-02「高位对准后直接松爪让
    温度计落进去、塞子正好对准」）：state→dropping，DROP_FRAMES 帧加速下落（z 加速、xy 线性
    收敛到管口轴心 TUBE_XY），终点 origin z=INSERT_THERMO_ORIGIN_Z=0.941（塞中心 0.137 封管口
    1.078），旋转保持泡朝下。
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
        self._drop_t = 0           # 落体动画帧计数（松爪后自由落体插管）
        self._drop_start = None    # 落体起点（松爪时刻温度计 world 矩阵，旋转泡朝下保留）

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self._held = None
        self._drop_t = 0
        self._drop_start = None
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

        # 已插管：温度计停在插管终态（origin z=0.941 泡朝下），不再持握/落体。
        # 缺失此分支会落到下方「持握期」分支，把温度计写回夹爪位 → 插管位↔夹爪位来回闪现。
        if self.state == "inserted":
            return

        # 落体动画（松爪后温度计靠重力落进提勒管，塞子卡口；持握已止，每帧推进 z）
        if self.state == "dropping":
            self._drop_t += 1
            frac = min(1.0, self._drop_t / DROP_FRAMES)
            k = frac * frac                       # 加速下落（自由落体感）
            tx, ty = TUBE_XY
            ez = INSERT_THERMO_ORIGIN_Z
            sx, sy, sz = self._drop_start.ExtractTranslation()
            x = sx + (tx - sx) * frac             # xy 线性收敛到管口轴心（塞子正好对准）
            y = sy + (ty - sy) * frac
            z = sz - (sz - ez) * k                # z 加速下落到插管终态
            M = Gf.Matrix4d(self._drop_start)
            M.SetRow(3, Gf.Vec4d(x, y, z, 1.0))
            self.task._set_thermometer_world(M)
            if frac >= 1.0:
                self.state = "inserted"
                print("[b5] thermometer dropped into thiele tube, stopper sealed mouth")
            return

        # 持握期：6-DOF 刚性跟随夹爪（夹爪原地旋转立起来时温度计随之竖直泡朝下）
        self.task._set_thermometer_world(self._held_matrix())
        # 松爪：触发落体（温度计靠重力落进管口、塞子卡口），不再写回静止矩阵
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.state = "dropping"
            self._drop_t = 0
            self._drop_start = self.task._get_thermometer_world_matrix()
            print("[b5] thermometer released -> dropping into thiele tube")


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


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，⑬ 点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转（照 B2/C4——
    火柴杆横躺，夹爪手指朝下竖直夹其杆身）。释放时写回台面静止位。
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
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
                print(f"[b5] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（低位 MATCH_LIFT_Z）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print("[b5] match released to rest")


class _LampLifecycle:
    """单支酒精灯状态机（rest → attached → released，⑭ 夹灯加热摆动 15s + 移灯 -X 5cm）。

    持握 = 矩阵 _LAMP_HELD · tool_world（水平横夹灯体宽处 z=0.845，ORIENT_FWD 手指朝前，
    d2s 夹药匙同款矩阵持握）：灯随夹爪，垂直姿态保持（xz/朝向不变），只跟夹爪水平平移（X 摆动）。
    移灯终点松爪 → released：灯锁移灯位（不再跟随夹爪），火焰跟随灯锁定；task 读 lamp.released
    → phase cap_lamp。reset 时写回原位（移灯前的位置）。
    参考点（gripper/TCP 世界坐标）：
      grasp   灯体宽处抓点（LAMP_GRASP，z=0.845 Ø76.8mm）
      rest    灯底座中心静止位（LAMP_XY + LAMP_REST_Z）
      target  移灯终点夹爪（LAMP_TARGET；松爪判定）
    """

    def __init__(self, task, name, path, rest, grasp, target):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp = np.array(grasp)
        self.target = np.array(target)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.task._set_lamp_world(self.task._lamp_rest_matrix())
        self.task._set_flame_lamp_x(LAMP_XY[0])

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._lamp_held_matrix()
            # 夹爪开始合拢且已进近窗：先把灯平滑拉向持握位（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_lamp_world(held)
            # 灯专用 attach 阈值 lamp_closed_threshold（灯体 Ø76.8 宽，合不到常规 0.025）
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.lamp_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_lamp_world(held)
                print(f"[b5] lamp attached (grip={opening:.4f})")
            return

        # 吸附期（attached）：灯逐帧跟随夹爪（矩阵持握，灯保持竖直只跟平移）；火焰跟随灯（x=灯原点 x）
        if self.state == "attached":
            self.task._set_lamp_world(self.task._lamp_held_matrix())
            self.task._set_flame_lamp_x(gripper_pos[0])
            # 松爪判定：夹爪到移灯终点且 grip 打开 → released，灯锁移灯位（火焰也锁移灯位）。
            # 用 lamp_open_threshold（>GRIP_LAMP 0.038 才真松爪，同 B2/B3）：常规
            # gripper_open_threshold 0.03 在移灯时开度保持 0.038 已超标 → 提前 released。
            if (opening > self.task.lamp_open_threshold
                    and self.task._near(self.target, gripper_pos)):
                self.released = True
                self.state = "released"
                self.task._set_lamp_world(self.task._lamp_target_matrix())
                self.task._set_flame_lamp_x(self.target[0])
                print(f"[b5] lamp released at target (grip={opening:.4f})")
            return

        # released：灯锁移灯位（不再跟随夹爪），火焰锁定不移（夹爪已退走）。
        # 必须早退（同 B2 四改）：released 后若落进「灯逐帧跟随夹爪」分支，盖帽动作一起臂灯就
        # 跟着走；帽是灯子 prim，世界位随灯漂（「帽还跟着乱动」）。空状态，复位靠 task.reset()。


class _CapLifecycle:
    """单支灯帽状态机（rest → attached → settled，⑯ 移灯后盖灯帽灭火）。

    持握 = 纯平移 offset（LAMPCAP_HELD_OFFSET）：帽全程竖直开口朝下，不随夹爪旋转（与火柴同款
    纯平移持握，非矩阵持握）。帽是灯的子 prim，吸附期逐帧把帽写到夹爪持握位（帽中心 = 夹爪 +
    LAMPCAP_HELD_OFFSET，经 _set_cap_world 换算成帽相对灯的 local translate）。盖到位（夹爪近
    LAMPCAP_BURNER 连续帧）→ settled：火焰熄灭、帽锁灯口。
    参考点（gripper/TCP 世界坐标）：
      grasp   帽静止位夹点（LAMPCAP_GRASP=LAMPCAP_REST 同水平，帽顶下 7mm；移灯期间 task 每帧
             _set_cap_world(LAMPCAP_REST) 把帽钉在静止位，不随灯滑到移灯位）
      cover   盖灯口夹爪（LAMPCAP_BURNER=0.900，帽中心 0.8917 盖严实，同资产原始帽位）
    """

    def __init__(self, task, name, path, grasp, cover):
        self.task = task
        self.name = name
        self.path = path
        self.grasp = np.array(grasp)
        self.cover = np.array(cover)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.settled = False
        self.extinguish_counter = 0

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.settled = False
        self.extinguish_counter = 0
        self.task._set_cap_rest()

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos, dtype=float) + np.array(LAMPCAP_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_cap_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.cap_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_cap_world(held)
                print(f"[b5] cap attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：帽跟随夹爪（纯平移），帽中心 = 夹爪 + LAMPCAP_HELD_OFFSET
            held = np.asarray(gripper_pos, dtype=float) + np.array(LAMPCAP_HELD_OFFSET)
            self.task._set_cap_world(held)
            # 下落即熄火：帽盖到灯口下降途中火焰熄（z 门控 LAMPCAP_EXTINGUISH_Z=1.00，帽底 0.976
            # 罩过火焰尖 0.9795 才灭）。xy 门控防误触：⑥ 水平运帽在高位不触发、⑤ 提帽在静止位旁。
            if (not self.task.flame_extinguished
                    and np.linalg.norm(np.asarray(gripper_pos[:2]) - self.cover[:2]) < LAMPCAP_EXTINGUISH_XY
                    and gripper_pos[2] < LAMPCAP_EXTINGUISH_Z):
                self.task._extinguish_flame()
            # 盖到位：夹爪近盖灯口位 LAMPCAP_BURNER 连续帧 → settled → 火焰熄灭、帽锁灯口
            if np.linalg.norm(np.asarray(gripper_pos) - self.cover) < self.task.cap_cover_near:
                self.extinguish_counter += 1
                if self.extinguish_counter >= self.task.cap_dwell_frames:
                    self.state = "settled"
                    self.settled = True
                    self.task._on_cap_settled(held)
                    print("[b5] cap settled, flame extinguished")
            else:
                self.extinguish_counter = 0
            return

        # settled：帽锁灯口（不再跟随夹爪，帽已停在盖灭位），火焰已熄（_on_cap_settled 处理）


class B5MeltingPointTask(BaseTask):
    """B5 熔点测定（提勒管法）任务：拿起毛细管 → 转竖直 → 插入粉丘。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    CAPILLARY = "/World/CapillaryTube"
    SAMPLE_PLUG = "/World/CapillaryTube/SamplePlug"
    SAMPLE_MELT = "/World/CapillaryTube/SampleMelt"
    THERMOMETER = "/World/MainThermometer"
    # 温度计红液柱锚定缩放（2026-09-03 用户「最开始温度计不该是满的，红色液柱应缓慢上升」）：
    # 毛细红液柱烘焙为**全量程满柱** asset z[0.005,0.245]（build_thermometer 原样 → 显示"满"），
    # 任务每帧用锚定缩放 transform（同 B2 _set_capillary）压到当前温度读数。柱顶读数刻度映射
    # z_scale(T)=0.02+(T+20)/130*0.22（asset 刻度 -20..110°C）；mp≤~107°C 柱顶=熔点真刻度，
    # 更大则封顶 LIQ_TOP_CAP（近刻度顶）→ mp=158 默认即此：初始室温=低位，加热 15s 内红色从低位
    # 爬满可视刻度段，熔点时柱顶最高。
    THERMO_LIQUID = "/World/MainThermometer/Thermometer/capillary_liquid"
    LIQ_ANCHOR_Z = 0.005        # 柱底锚（asset 局部 z：底固定、柱顶随温度爬升）
    LIQ_FULL_H = 0.245 - 0.005  # 柱全高（烘焙 z[0.005,0.245]，即 anchor..0.245）
    LIQ_TMIN, LIQ_TMAX = -20.0, 110.0
    LIQ_SCALE_LO, LIQ_SCALE_HI = 0.02, 0.24   # 刻度区 z（z_scale 的区间端点）
    LIQ_TOP_CAP = 0.235          # mp 超量程时的柱顶封顶 z（近刻度顶 0.24，留白不顶玻璃）
    MATCH = "/World/Match"
    LAMP = "/World/AlcoholLamp"
    CAP = "/World/AlcoholLamp/cap"

    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest/B2/C4）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近

    # 石蜡油对流气泡（⑭ 加热期油浴对流，照 B2 _step_bubble_anim 动态池）
    OIL_BUBBLES = "/World/OilBubbles"
    N_OIL_BUBBLES = 16
    OIL_BUBBLE_RISE = 0.0018         # 每帧上升量（m，@60Hz ≈ 0.108m/s）
    OIL_BUBBLE_SPAWN_INTERVAL = 4    # 全速生成间隔帧（实际间隔 = 本值/_oil_bubble_vigor）
    OIL_BUBBLE_WOBBLE_AMP = 0.0012   # 上升蛇形摆动振幅（±1.2mm）
    OIL_BUBBLE_MAX_RADIUS = 0.006    # 气泡中心离管轴最大半径（油柱 r0.010 − 泡 r0.002）
    OIL_TOP_Z = 1.066                # 油面 z（gen OIL_TOP_Z，气泡到此下方破灭）
    OIL_BUBBLE_POP_DZ = 0.004        # 气泡到油面下方 4mm 破灭
    OIL_TUBE_XY = (0.400, 0.0029)    # 提勒管主管轴 x,y（gen TUBE_X/TUBE_Y，气泡离轴钳位基准）

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

        # 火柴生命周期（⑬ 点燃酒精灯：纯平移持握，头朝 +X）+ 点火状态
        self._disable_collision(self.MATCH)
        self.match = _MatchLifecycle(self, "match", self.MATCH,
                                     (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z), MATCH_GRASP)
        self.flame_lit = False          # 火柴触灯芯点燃（火焰 reveal）
        self.match_ignite_counter = 0   # 火柴头近灯芯连续帧计数
        self.flame_prims = self._flame_paths()   # 4 个火焰 prim（外/内焰各 球+锥，可见性用）
        # 火焰组初始原点（组 translate = 火焰底锚点 LAMP_X；移灯/摆动时 x 跟随灯，reset 写回）。
        # 注意：子 prim（球/锥）的 translate 是纯几何偏移 (0,0,zc)/(0,0,zc+h/2)，绝不能被 x 跟随改写，
        # 否则火焰会被甩离灯体几十厘米（2026-09-03 夹灯瞬间火焰消失根因）。flicker 只动组 scale/rotate，
        # 组 translate 由 _set_flame_lamp_x 独占。
        self.flame_grp_base = [self._read_translate(p) for p in FLAME_GRPS]

        # 温度模型（B5 熔点测定：熔点由输入决定，加热 15s 内线性升到熔点，与 15s 摆动同步）
        self.room_temp = float(getattr(cfg, "room_temp", 25.0))
        self.melting_point = float(getattr(cfg, "melting_point", 158.0))
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.cap_dwell_frames = int(getattr(cfg, "cap_dwell_frames", 15))
        self.cap_result_hold_frames = int(getattr(cfg, "cap_result_hold_frames", 120))
        self.debug_heat_move = bool(getattr(cfg, "debug_heat_move", False))
        self.debug_cap_lamp = bool(getattr(cfg, "debug_cap_lamp", False))
        # heat_rate = 15s（HEAT_SWAY_CYCLES×HEAT_SWAY_PERIOD=900 帧）内从室温线性升到熔点
        self.heat_rate = (self.melting_point - self.room_temp) / float(
            HEAT_SWAY_CYCLES * HEAT_SWAY_PERIOD)
        self.temperature = self.room_temp
        self._cap_op = None              # 温度计红液柱缩放 transform op 缓存（首帧 AddTransformOp，见 _set_thermo_liquid）
        self.phase = "idle"
        self._ignite_frame = 0           # 进入 ignited 相的帧号（dwell 计时）
        self.flame_extinguished = False  # 盖帽熄火标记（幂等，_extinguish_flame 只灭一次）
        self._melted = False             # 样品熔点相变标记（T>=熔点 → SamplePlug 隐藏、SampleMelt 显示）
        self._cap_done_frames = 0

        # 灯/帽专用阈值（灯体宽、帽 Ø37mm，同 B2/B3）
        self.lamp_closed_threshold = LAMP_CLOSED_THRESHOLD
        self.lamp_open_threshold = LAMP_OPEN_THRESHOLD
        self.cap_closed_threshold = LAMPCAP_CLOSED_THRESHOLD
        self.cap_cover_near = LAMPCAP_COVER_NEAR

        # 酒精灯（⑭ 夹灯加热摆动 15s + 移灯 -X 5cm）：静态碰撞体，吸附期关碰撞
        self._disable_collision(self.LAMP)
        lamp_rest = (LAMP_XY[0], LAMP_XY[1], LAMP_REST_Z)
        self.lamp = _LampLifecycle(self, "lamp", self.LAMP, lamp_rest, LAMP_GRASP, LAMP_TARGET)
        # 灯帽（⑯ 盖帽灭火）：帽是灯子 prim（碰撞已随灯 disable）；grasp=帽静止位夹点、
        # cover=盖灯口夹爪。帽静止 local translate 读场景（reset 写回）。
        self.cap = _CapLifecycle(self, "cap", self.CAP, LAMPCAP_GRASP, LAMPCAP_BURNER)
        self.cap_rest_translate = self._read_translate(self.CAP)

        # 石蜡油对流气泡（⑭ 加热期油浴对流）：读场景烘焙基准 + 动态池状态
        self.oil_bubble_prims = [f"{self.OIL_BUBBLES}/bubble_{i}" for i in range(self.N_OIL_BUBBLES)]
        self._oil_bubble_bases = [self._read_translate(p) for p in self.oil_bubble_prims]
        self._oil_bubble_active = [False] * self.N_OIL_BUBBLES
        self._oil_bubble_z = [b[2] for b in self._oil_bubble_bases]
        self._oil_bubble_age = [0] * self.N_OIL_BUBBLES
        self._oil_bubble_speed = [0.7 + 0.6 * ((i * 37) % 10) / 10.0 for i in range(self.N_OIL_BUBBLES)]
        self._oil_bubble_phase = [(i * 0.7) % (2.0 * np.pi) for i in range(self.N_OIL_BUBBLES)]
        self._oil_bubble_spawn_timer = 0
        self._oil_bubble_vigor = 0.0

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.capillary.reset()
        self.capillary_hold.reset()
        self.thermometer.reset()
        self._capillary_stuck = False
        self._capillary_stick_rel = None
        self.match.reset()
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._set_flame_visible(False)
        # 灯/帽/温度/相态/气泡复位（⑭⑮⑯ 加热/移灯/盖帽段）
        self.phase = "idle"
        self.temperature = self.room_temp
        self._set_thermo_liquid(self.temperature)   # 红液柱写回室温低位（防"初始满柱"，同 B2 每帧驱动）
        self._ignite_frame = 0
        self.flame_extinguished = False
        self._melted = False
        self._cap_done_frames = 0
        self._oil_bubble_vigor = 0.0
        self._oil_bubble_spawn_timer = 0
        self._set_visibility(self.SAMPLE_MELT, False)
        for i in range(self.N_OIL_BUBBLES):
            self._oil_bubble_active[i] = False
            self._oil_bubble_z[i] = self._oil_bubble_bases[i][2]
            self._oil_bubble_age[i] = 0
            self._set_visibility(self.oil_bubble_prims[i], False)
            self._set_translate(self.oil_bubble_prims[i], self._oil_bubble_bases[i])
        self.lamp.reset()
        self.cap.reset()
        for p, base in zip(FLAME_GRPS, self.flame_grp_base):
            self._set_translate(p, base)

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
        self.match.step(gripper_pos, opening)              # ⑬ 火柴（纯平移持握，触灯芯点火）
        self.lamp.step(gripper_pos, opening)               # ⑭⑮ 酒精灯（矩阵持握摆动 15s + 移灯 -X 5cm）
        self.cap.step(gripper_pos, opening)                # ⑯ 灯帽（纯平移持握盖帽熄火）
        self._step_match_ignite(gripper_pos)               # ⑬ 点火检测（头近灯芯→flame_lit→火焰 reveal）
        self._step_flame_anim()                            # ⑬ 火焰每帧 flicker（点燃后，水滴形动起来）
        self._update_experiment()                          # 温度模型 + 相态机 + 油浴对流 + 熔点相变
        self._set_thermo_liquid(self.temperature)          # 温度计红液柱随实时温度锚定缩放（室温低位→熔点爬升）
        return self.get_basic_state_info(additional_info={
            "capillary_state": self.capillary.state,
            "capillary_attached": self.capillary.attached,
            "capillary_released": self.capillary.released,
            "capillary_hold_state": self.capillary_hold.state,
            "capillary_hold_attached": self.capillary_hold.attached,
            "thermometer_state": self.thermometer.state,
            "thermometer_attached": self.thermometer.attached,
            "capillary_stuck": self._capillary_stuck,
            "match_state": self.match.state,
            "match_attached": self.match.attached,
            "flame_lit": self.flame_lit,
            "phase": self.phase,
            "temperature": round(self.temperature, 1),
            "melting_point": self.melting_point,
            "lamp_state": self.lamp.state,
            "lamp_attached": self.lamp.attached,
            "lamp_released": self.lamp.released,
            "cap_state": self.cap.state,
            "cap_attached": self.cap.attached,
            "cap_settled": self.cap.settled,
            "flame_extinguished": self.flame_extinguished,
            "melted": self._melted,
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

    # ------------------------------------------------------------------
    # 火柴位姿 + 点火检测 + 火焰 flicker（⑬ 点燃酒精灯）
    # ------------------------------------------------------------------
    def _match_rest_pos(self):
        """火柴原点台面静止位（MATCH_XY + MATCH_REST_Z）。"""
        return np.array([MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z])

    def _set_match_world(self, position):
        """把火柴写到给定世界位置（只改 translate op，火柴水平头朝 +X 姿态不变，照 C4）。"""
        self.object_utils.set_object_position(self.MATCH, np.asarray(position, dtype=float))

    def _match_tip(self, gripper_pos):
        """火柴头中心世界坐标 = 夹爪 + MATCH_TIP_OFFSET（头在夹爪 +X 0.0494，水平朝前）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)

    def _step_match_ignite(self, gripper_pos):
        """点火检测（仿 flametest/B2/C4 _step_match_ignite）：火柴 attached 期间头近灯芯连续
        MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit=True → 火焰 reveal。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    self._set_flame_visible(True)
                    print(f"[b5] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    def _flame_paths(self):
        """火焰迁到 /World 顶层 grp（gen rebuild_flames 水滴形 = 底半球 Sphere + 上部 Cone，
        每焰两 prim 挂 /World/<名>_grp 组下，pivot=火焰底）。gen 初始隐藏，task 点着翻 visible；
        组本身每帧 flicker（scale/rotate），见 _step_flame_anim。"""
        return list(FLAME_PRIMS)

    def _set_flame_visible(self, visible):
        """点火 reveal / 熄火隐藏全部火焰 prim（外/内焰各 球+锥 共 4 个）。"""
        for p in self.flame_prims:
            self._set_visibility(p, visible)

    def _set_visibility(self, path, visible):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Imageable(prim).GetVisibilityAttr().Set(
                UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)

    def _smooth_noise(self, t, seed):
        """确定性平滑噪声（每 3 帧一个随机值，相邻 smoothstep 插值）。火焰 flicker 用，
        同输入同输出（无随机抖动，录像可复现）。照 C4（修过 20Hz 爆闪 bug）。"""
        span = 3.0
        i = math.floor(t / span)
        f = (t - i * span) / span

        def _rand(n):
            n = (n * 2654435761 + 12345) & 0xFFFFFFFF
            n ^= n >> 13
            return ((n % 1000) / 500.0) - 1.0

        v0, v1 = _rand(i + seed * 131), _rand(i + 1 + seed * 131)
        f = f * f * (3.0 - 2.0 * f)   # smoothstep
        return v0 + (v1 - v0) * f

    def _apply_flame_flicker(self, grp_path, base_pos=None, seed=0):
        """对火焰组做每帧 flicker：scale(高/宽) + rotateXYZ(侧摆)，pivot=组原点=火焰底
        （gen 组 op 序 translate→rotate→scale：先 scale 后 rotate 再 translate = 绕底不漂移）。
        base_pos 给则同时写 translate（B5 酒精灯火焰固定，恒 None 不动 translate）。"""
        prim = self.stage.GetPrimAtPath(grp_path)
        if not prim.IsValid():
            return
        t = float(self.frame_idx)
        h = 1.0 + 0.13 * self._smooth_noise(t, seed) + 0.06 * math.sin(t * 0.35 + seed)
        w = 1.0 + 0.10 * self._smooth_noise(t, seed + 7) + 0.04 * math.sin(t * 0.53 + seed)
        lean = 7.0 * self._smooth_noise(t, seed + 13)      # 侧摆度数（±~7°）
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            ot = op.GetOpType()
            if ot == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(w, w, h))
            elif ot == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3f(lean, 0.0, 0.0))
            elif base_pos is not None and ot == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*base_pos))

    def _step_flame_anim(self):
        """酒精灯火焰每帧 flicker（点燃后，用户 2026-09-02「火焰应该是水滴型然后在动比较逼真，
        模仿 C3/C4」）。"""
        if not self.flame_lit:
            return
        self._apply_flame_flicker(FLAME_GRPS[0], None, seed=1)
        self._apply_flame_flicker(FLAME_GRPS[1], None, seed=2)

    # ------------------------------------------------------------------
    # 温度模型 + 相态机 + 油浴对流 + 熔点相变（⑭⑮⑯ 加热/移灯/盖帽）
    #   idle → ignited（火焰点亮）→ heating（夹灯摆动 15s，温度升到熔点、样品熔化、油浴
    #   对流气泡上升）→ cap_lamp（移灯 -X 5cm 后盖帽熄火）→ done
    # ------------------------------------------------------------------
    def _update_experiment(self):
        if self.phase == "idle":
            # 调试：跳过段1全部，直进 heating 相（只跑 LampHeatMovePass 抓灯摆动 + 移灯）
            if self.debug_heat_move:
                self._set_flame_visible(True)
                self.flame_lit = True
                self.phase = "heating"
                self._ignite_frame = self.frame_idx
                print("[b5] debug_heat_move: skip to heating phase (flame on)")
            # 调试：跳过段1+加热+移灯，灯预摆移灯位 + 火焰点亮，直进 cap_lamp 相（只跑 CapLampPass）
            elif self.debug_cap_lamp:
                self._set_lamp_world(self._lamp_target_matrix())
                self._set_flame_lamp_x(LAMP_TARGET[0])
                self._set_flame_visible(True)
                self.flame_lit = True
                self._set_cap_world(LAMPCAP_REST)
                self.lamp.state = "released"
                self.lamp.released = True
                self.phase = "cap_lamp"
                print("[b5] debug_cap_lamp: lamp pre-moved + flame on -> cap_lamp phase")
            elif self.flame_lit:
                self.phase = "ignited"
                self._ignite_frame = self.frame_idx
                print(f"[b5] ignite: flame on @ frame {self.frame_idx}")

        elif self.phase == "ignited":
            # 点火后停留 ignite_dwell_frames 帧（火焰稳定）→ 进入加热
            if self.frame_idx - self._ignite_frame >= self.ignite_dwell_frames:
                self.phase = "heating"
                print(f"[b5] heating start T={self.temperature:.1f}")

        elif self.phase == "heating":
            # 温度线性上升（heat_rate 保证 15s 摆动结束正好到熔点）；油浴对流气泡随温度进度加密
            self.temperature = min(self.melting_point, self.temperature + self.heat_rate)
            progress = (self.temperature - self.room_temp) / max(
                1e-6, self.melting_point - self.room_temp)
            self._oil_bubble_vigor = progress
            if progress > 0:
                self._step_oil_bubble_anim()
            # 熔点相变：到达熔点 → 样品固体转液体（SamplePlug 隐藏、SampleMelt 显示）
            if not self._melted and self.temperature >= self.melting_point:
                self._melted = True
                self._set_sample_visible(False)
                self._set_visibility(self.SAMPLE_MELT, True)
                print(f"[b5] sample melted at T={self.temperature:.1f} "
                      f"(melting point {self.melting_point:.1f})")
            # 移灯完成（LampHeatMovePass 松爪后 lamp.released）→ 盖帽相
            if self.lamp.released:
                self.phase = "cap_lamp"
                print("[b5] heating done, lamp moved -X -> cap lamp phase")

        elif self.phase == "cap_lamp":
            # 机械臂 CapLampPass 盖帽中：帽被夹走 → 火焰仍亮（跟随灯）；帽盖到位 settled →
            # 火焰熄灭（_on_cap_settled）。熄火后再停留 cap_result_hold_frames 帧才 done
            if self.cap.settled:
                self._cap_done_frames += 1
                if self._cap_done_frames >= self.cap_result_hold_frames:
                    self.phase = "done"
                    print(f"[b5] done: cap covers lamp, flame extinguished, "
                          f"melting point {self.melting_point:.1f}°C recorded")

    # ------------------------------------------------------------------
    # 温度计红色液柱锚定缩放（同 B2 _set_capillary，2026-09-03 用户「最开始温度计不该是满的，
    # 红色液柱应缓慢上升」）。见类常量 THERMO_LIQUID 注释：毛细柱烘焙为满柱，每帧按温度压到对应
    # 读数刻度。底锚 LIQ_ANCHOR_Z=0.005 不动（贴泡储液段），柱顶 = z_men 随温度爬升。
    # 注：capillary_liquid 还带 gen 的 RotateXYZ(0,0,270)（刻度朝向）——本缩放 M 只绕竖轴 z 伸缩，
    # 与 Rz 可交换，互不干扰。
    # ------------------------------------------------------------------
    def _z_thermo_scale(self, T):
        """温度读数 T（°C）→ asset 刻度区 z（-20..110°C 映射 z=0.02..0.24，同 build_thermometer
        z_of：z=0.02+(T+20)/130*0.22）。"""
        return self.LIQ_SCALE_LO + (T - self.LIQ_TMIN) / (self.LIQ_TMAX - self.LIQ_TMIN) \
            * (self.LIQ_SCALE_HI - self.LIQ_SCALE_LO)

    def _set_thermo_liquid(self, T):
        """把温度计红液柱顶压到温度 T 的读数刻度（每帧调，室温低位 → 加热爬到熔点）。"""
        prim = self.stage.GetPrimAtPath(self.THERMO_LIQUID)
        if not prim.IsValid():
            return
        # 柱顶区间：室温读数（低位）→ 熔点读数（mp≤~107°C 用真刻度；mp 超量程封顶 LIQ_TOP_CAP）。
        z_lo = self._z_thermo_scale(self.room_temp)
        z_hi = min(self._z_thermo_scale(self.melting_point), self.LIQ_TOP_CAP)
        if z_hi < z_lo:
            z_hi = z_lo
        span = max(1e-6, self.melting_point - self.room_temp)
        frac = min(1.0, max(0.0, (T - self.room_temp) / span))
        z_men = z_lo + frac * (z_hi - z_lo)
        s = (z_men - self.LIQ_ANCHOR_Z) / self.LIQ_FULL_H
        s = min(1.0, max(0.0, s))
        # M = T(0,0,-anchor) · S(1,1,s) · T(0,0,+anchor)：底锚固定，柱顶 = anchor + FULL_H*s
        S = Gf.Matrix4d().SetScale(Gf.Vec3d(1, 1, s))
        Td = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, -self.LIQ_ANCHOR_Z))
        Tu = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, self.LIQ_ANCHOR_Z))
        M = Td * S * Tu
        xf = UsdGeom.Xformable(prim)
        if self._cap_op is None:
            self._cap_op = xf.AddTransformOp()
        self._cap_op.Set(M)

    def _step_oil_bubble_anim(self):
        """油浴对流气泡动画（照 B2 _step_bubble_anim 动态池）：加热期小球池按「间隔 =
        OIL_BUBBLE_SPAWN_INTERVAL/_oil_bubble_vigor」从油底散布区生成一颗、逐帧上升（速度差异 +
        蛇形摆动）、到油面下方破灭复用。_oil_bubble_vigor 由加热相驱动 = 温度进度 0→1。只写子球
        translate（x/y = 基准 + 摆动、z = 上升，离轴钳到 OIL_BUBBLE_MAX_RADIUS 防插壁）。"""
        pop_z = self.OIL_TOP_Z - self.OIL_BUBBLE_POP_DZ
        # 生成：vigor>0 按间隔生成（vigor 越大越密）
        if self._oil_bubble_vigor > 0:
            self._oil_bubble_spawn_timer += self._oil_bubble_vigor
            if self._oil_bubble_spawn_timer >= self.OIL_BUBBLE_SPAWN_INTERVAL:
                self._oil_bubble_spawn_timer -= self.OIL_BUBBLE_SPAWN_INTERVAL
                for i, active in enumerate(self._oil_bubble_active):
                    if not active:
                        self._oil_bubble_active[i] = True
                        self._oil_bubble_z[i] = self._oil_bubble_bases[i][2]
                        self._oil_bubble_age[i] = 0
                        self._set_visibility(self.oil_bubble_prims[i], True)
                        break
        # 推进在飞气泡：上升（速度差异）+ 蛇形摆动，到油面消失（隐藏留复用）
        for i, (bx, by, bz) in enumerate(self._oil_bubble_bases):
            if not self._oil_bubble_active[i]:
                continue
            age = self._oil_bubble_age[i]
            z = self._oil_bubble_z[i] + self.OIL_BUBBLE_RISE * self._oil_bubble_speed[i]
            if z >= pop_z:
                self._oil_bubble_active[i] = False
                self._set_visibility(self.oil_bubble_prims[i], False)
                continue
            self._oil_bubble_z[i] = z
            ph = self._oil_bubble_phase[i]
            wob = self.OIL_BUBBLE_WOBBLE_AMP * np.sin(age * 0.15 + ph)
            woy = self.OIL_BUBBLE_WOBBLE_AMP * np.sin(age * 0.13 + ph + 1.7)
            cx, cy = bx + wob, by + woy
            dx, dy = cx - self.OIL_TUBE_XY[0], cy - self.OIL_TUBE_XY[1]
            r = np.hypot(dx, dy)
            if r > self.OIL_BUBBLE_MAX_RADIUS:
                s = self.OIL_BUBBLE_MAX_RADIUS / r
                cx, cy = self.OIL_TUBE_XY[0] + dx * s, self.OIL_TUBE_XY[1] + dy * s
            self._oil_bubble_age[i] = age + 1
            self._set_translate(self.oil_bubble_prims[i], (cx, cy, z))

    # ------------------------------------------------------------------
    # 酒精灯持握（⑭⑮ 矩阵持握：夹灯体宽处摆动 15s + 移灯 -X 5cm，照 B2/B3）
    # ------------------------------------------------------------------
    def _lamp_held_matrix(self):
        """灯当前持握世界矩阵 = _LAMP_HELD · tool_world（Orient_FWD 下灯保持竖直 R180，
        与场景世界旋转一致 → 附着手腕已回正零跳变；只跟夹爪水平平移）。"""
        return _LAMP_HELD * self._tool_world()

    def _set_lamp_world(self, world_matrix):
        """把灯写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.LAMP)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _lamp_rest_matrix(self):
        """灯原位静止位姿（底座中心 LAMP_XY + LAMP_REST_Z，世界旋转 R180 = 场景 EQUIP rot180
        一致）。平移写在最后一行（行向量）。"""
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           LAMP_XY[0], LAMP_XY[1], LAMP_REST_Z, 1.0)

    def _lamp_target_matrix(self):
        """灯移灯位静止位姿（底座中心 = LAMP_TARGET 夹爪正下方 0.0448，xz 不变、x -5cm）。"""
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           LAMP_TARGET[0], LAMP_TARGET[1],
                           LAMP_TARGET[2] - LAMP_GRASP_OFFSET, 1.0)

    def _ease_lamp_world(self, target, k=0.18):
        """夹爪合拢期间灯逐帧平滑移向持握位（消除闪现吸附）。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.LAMP)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_lamp_world(_blend_world(cur, target, k))

    def _set_flame_lamp_x(self, x):
        """火焰组原点 translate x 设为 x（组原点 = 火焰底锚点，灯位 LAMP_X）。移灯/摆动时火焰跟随灯
        原点 x，y/z 不变。必须写组 translate：子 prim（球/锥）translate 是纯几何偏移，若被写成世界 x
        会把火焰甩离灯（见 __init__ 注释，2026-09-03 夹灯瞬间火焰消失根因）。"""
        for p, base in zip(FLAME_GRPS, self.flame_grp_base):
            self._set_translate(p, (x, base[1], base[2]))

    # ------------------------------------------------------------------
    # 灯帽持握（⑯ 纯平移持握：帽中心 = 夹爪 + LAMPCAP_HELD_OFFSET，帽是灯子 prim）
    # ------------------------------------------------------------------
    def _set_cap_translate(self, t):
        """只写帽的 translate op（帽 xform op 序 translate→rotateXYZ→scale，不能动其它 op）。"""
        prim = self.stage.GetPrimAtPath(self.CAP)
        if not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*t))
                return
        xf.AddTranslateOp().Set(Gf.Vec3d(*t))

    def _set_cap_world(self, center):
        """把帽写到给定帽中心世界坐标（帽 = 灯子 prim，只写帽 translate op，保留 rotateXYZ+scale
        形状）。换算（pxr 实测，灯 R180Z）：cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+LAMPCAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−LAMPCAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world(self.LAMP)
        if lamp_pos is None:
            return
        cx, cy, cz = center
        tx = lamp_pos[0] - cx
        ty = lamp_pos[1] - cy
        tz = cz - lamp_pos[2] - LAMPCAP_CENTER_DZ
        self._set_cap_translate((tx, ty, tz))

    def _set_cap_rest(self):
        """帽回静止位（写回读自场景的帽 local translate，帽底贴台面、开口朝下）。"""
        if self.cap_rest_translate is not None:
            self._set_cap_translate(self.cap_rest_translate)

    def _ease_cap_world(self, target, k=0.18):
        """夹爪合拢期间帽逐帧平滑移向持握位（消除闪现吸附）。"""
        cur_origin = self._get_obj_world(self.CAP)
        if cur_origin is None:
            return
        cur_center = np.asarray(cur_origin, dtype=float) + np.array([0.0, 0.0, LAMPCAP_CENTER_DZ])
        nxt = cur_center + (np.asarray(target, dtype=float) - cur_center) * k
        self._set_cap_world(nxt)

    def _extinguish_flame(self):
        """熄灭火焰（幂等：下落即熄/盖到位都调，只灭一次）。"""
        if self.flame_extinguished:
            return
        self.flame_extinguished = True
        self._set_flame_visible(False)
        print(f"[b5] flame extinguished @ frame {self.frame_idx}")

    def _on_cap_settled(self, center):
        """帽盖到位：火焰熄灭、帽锁灯口（settled 态不再跟随夹爪，帽停在盖灭位）。"""
        self._extinguish_flame()

    # ------------------------------------------------------------------
    # 通用 xform 读/写（d2s/d3s/b2 同款）
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        """物体原点世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_visible(self, paths, visible):
        """批量可见性开关（str 或 list 皆可，委托 _set_visibility）。"""
        if isinstance(paths, str):
            paths = [paths]
        for path in paths:
            self._set_visibility(path, visible)

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


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
