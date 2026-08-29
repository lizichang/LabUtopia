# -*- coding: utf-8 -*-
"""B3 水浴加热任务（酒精灯加热烧杯水 → 水浴加热试管内固体样品 → 固体熔化/不熔化）。

B3 简化自 B2 沸点测定（用户 2026-08-29 指令）：
  - 无温度计、无滴管、无样品瓶、无试管架（试管预夹在水浴烧杯里）。
  - 加热容器 = 烧杯盛水坐石棉网上（水浴），试管预夹浸入烧杯水 5cm；机械臂只做三件事：
    阶段A 放固体样品（SolidTransferPass 依次夹两颗固体入试管）→ 阶段B 点燃酒精灯
    （LightFlamePass）→ 阶段C 盖灯帽灭火（CapLampPass，原位盖帽不移灯）。

温度模型（同 B2 简化）：T 从 room_temp 按 heat_rate 升温到 boiling_point（水 100.0）。
相态机：
    idle（火焰隐藏，等固体放好 + 火柴点燃）→ ignited（火焰 reveal）
    → heating（烧杯水按 heat_rate 升温；气泡随温度进度逐个 reveal，机械臂不动）
    → boiling（水到沸点，气泡全亮 30 泡；sample_phase=melted 时揭示熔化液柱 TubeMelt_<色>
      + 隐藏固体颗粒，保持 boil_dwell 5s）→ cap_lamp（机械臂 CapLampPass 原位盖帽灭火，
      盖帽期间气泡继续沸腾）→ done（帽盖到位火焰熄灭后气泡逐个渐熄，全部消失后完成）。

驱动 prim（b3_water_bath.usd，由 scripts/gen_b3_scene.py 生成）：
    /World/flame_outer|flame_inner(±_sphere)  火焰（水滴形，迁 /World 顶层，同 B2）
    /World/BeakerWater                         烧杯内水柱（水浴，可见 r0.031 h0.085）
    /World/BeakerBubbles/bubble_{0..29}        烧杯水浴气泡（球组 r2mm 亮白，初始隐藏，
                                                 环带避试管，上升动画）
    /World/TubeMelt_{clear,red,blue,green,purple}  试管内熔化液柱（r8mm h20mm 贴管底，
                                                 初始隐藏，sample_phase=melted 时揭示一根）
    /World/SolidSample / SolidSample2          固体样品白颗粒 ×2（复用 zeolite.usd）
    /World/Match                                火柴（抬高 12mm，头朝灯芯）
    /World/AlcoholLamp/cap                      灯帽（灯子 prim，静止位 CAP_REST）
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TUBE_XY,
    SOLID_GRASP, SOLID2_GRASP, SOLID_CENTER_OFFSET, SOLID_DROP_Z, SOLID_STACK_DZ,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    CAP_CENTER_DZ, CAP_GRASP, CAP_HELD_OFFSET, CAP_BURNER,
    CAP_CLOSED_THRESHOLD, CAP_COVER_NEAR, CAP_EXTINGUISH_XY, CAP_EXTINGUISH_Z,
    EFFECT_BEAKER_BUBBLES, EFFECT_TUBE_MELT,
)

# 固体相对夹爪的持握矩阵 _SOLID_HELD（阶段A 放固体，2026-08-29 复刻 B2 _ZEO_HELD 放沸石）。
# 旋转与 B2 滴管/温度计/沸石完全一致（固体 Z 朝上→tool -X、X→tool -Z、Y→tool -Y），平移
# SOLID_CENTER_OFFSET=0.0037（固体半高）。固体局部原点=底 z=0、+Z 朝上（zeolite.usd 白颗粒）。
# 竖直夹（默认朝向手指朝下，tool+X 朝世界 -Z）→ 固体底在夹爪下方 0.0037、中心落在夹爪处；
# 旋转 ORIENT_FWD 后固体随夹爪、位置不变（颗粒旋转对称）。固体中心 = 夹爪（tool 原点）。
_SOLID_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                          0.0, -1.0, 0.0, 0.0,
                          -1.0, 0.0, 0.0, 0.0,
                          SOLID_CENTER_OFFSET, 0.0, 0.0, 1.0)


class _SolidLifecycle:
    """单颗固体状态机（rest → attached → released → settled，阶段A 放固体入试管）。

    持握 = 矩阵 _SOLID_HELD · tool_world（固体中心落在夹爪处，颗粒随夹爪平移/旋转）。
    竖直夹（默认朝向手指朝下）→ 旋转手指朝前 → 水平伸到试管上方 → 松爪 → 固体从
    夹爪坠落进试管沉底（settled，锁管底位姿，不再跟随夹爪）。两颗固体各一实例：
    第一颗沉底、第二颗叠第一颗顶（settle_dz=SOLID_STACK_DZ，管底内径 Ø16.1mm 只容一颗
    并排）。两颗抓点仅距 2cm（< 近窗 3cm），rest 态用「离夹爪最近那颗」门禁防同时误抓。
    参考点（gripper/TCP 世界坐标）：
      grasp   皿上固体中心抓点（SOLID_GRASP / SOLID2_GRASP）
      drop    管口正上方放下位（夹爪 x/y = 管口、z = SOLID_DROP_Z，松爪判定）
      settle  固体中心沉底位（管口 xy、z = 管底 + 半高 + settle_dz）
    """

    def __init__(self, task, name, path, grasp, drop, settle_dz=0.0):
        self.task = task
        self.name = name
        self.path = path
        self.grasp = np.array(grasp)
        self.drop = np.array(drop)
        self.settle_dz = settle_dz
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.settled = False
        self._fall_t = 0
        self._fall_start = None

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.released = self.settled = False
        self._fall_t = 0
        self._fall_start = None
        self.task._set_solid_world(self.path, self.task._solid_rest_matrix(self.grasp))

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            # 两颗固体抓点仅 2cm 在近窗内：只允许「离夹爪最近」的 rest 固体附着（防同时误抓两颗）
            if not self.task._is_closest_solid(self):
                self._near_frames = 0
                return
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._solid_held_matrix()
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_solid_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_solid_world(self.path, held)
                print(f"[b3] solid attached {self.name} (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 松爪判定：夹爪到管口正上方放下位且 grip 打开（⑨）→ released，固体坠落
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(self.drop, gripper_pos)):
                self.state = "released"
                self.released = True
                self._fall_start = np.asarray(gripper_pos, dtype=float)  # 夹爪=固体中心
                self._fall_t = 0
                print(f"[b3] solid released over tube {self.name} (grip={opening:.4f})")
            else:
                self.task._set_solid_world(self.path, self.task._solid_held_matrix())
            return

        if self.state == "released":
            # 坠落动画：固体中心从放下位加速坠落到管底，落定转 settled 锁管底位姿
            self._fall_t += 1
            start = np.asarray(self._fall_start, dtype=float)
            target = self.task._solid_settle_center(self.settle_dz)
            if self._fall_t >= self.task.SOLID_FALL_FRAMES:
                self.state = "settled"
                self.settled = True
                self.task._set_solid_world(self.path, self.task._solid_settle_matrix(self.settle_dz))
                print(f"[b3] solid settled {self.name} at tube bottom")
            else:
                frac = self._fall_t / self.task.SOLID_FALL_FRAMES
                center = start + (target - start) * (frac * frac)   # 加速坠落（t²）
                self.task._set_solid_world(self.path, self.task._solid_translate_matrix(center))
            return

        # settled：固体沉在管底（穿过水面），锁管底位姿
        self.task._set_solid_world(self.path, self.task._solid_settle_matrix(self.settle_dz))


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段B 点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （火柴杆横躺，夹爪手指朝下竖直夹其杆身）。释放时写回台面静止位。
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
                print(f"[b3] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[b3] match released to rest")


class _CapLifecycle:
    """单支灯帽状态机（rest → attached → settled，阶段C 原位盖灯帽灭火）。

    持握 = 纯平移 offset（CAP_HELD_OFFSET）：帽全程竖直开口朝下，不随夹爪旋转。帽是灯的
    子 prim，吸附期逐帧把帽写到夹爪持握位（帽中心 = 夹爪 + CAP_HELD_OFFSET，经 _set_cap_world
    换算成帽相对灯的 local translate）。盖到位（夹爪近 CAP_BURNER 连续帧）→ settled：
    火焰熄灭、帽锁灯口。B3 灯不移走，CAP_BURNER 直指灯原位。
    参考点（gripper/TCP 世界坐标）：
      grasp   帽静止位夹点（CAP_GRASP=CAP_REST 同水平，帽顶下 7mm）
      cover   盖灯口夹爪（CAP_BURNER=0.900，帽中心 0.8917 盖严实）
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
            held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_cap_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.cap_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_cap_world(held)
                print(f"[b3] cap attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：帽跟随夹爪（纯平移），帽中心 = 夹爪 + CAP_HELD_OFFSET
            held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
            self.task._set_cap_world(held)
            # 下落即熄火（B2 同款）：帽底罩过火焰顶才灭，xy 门控防误触
            if (not self.task.flame_extinguished
                    and np.linalg.norm(np.asarray(gripper_pos[:2]) - self.cover[:2]) < CAP_EXTINGUISH_XY
                    and gripper_pos[2] < CAP_EXTINGUISH_Z):
                self.task._extinguish_flame()
            # 盖到位：夹爪近盖灯口位 CAP_BURNER 连续帧 → settled → 火焰熄灭、帽锁灯口
            if np.linalg.norm(np.asarray(gripper_pos) - self.cover) < self.task.cap_cover_near:
                self.extinguish_counter += 1
                if self.extinguish_counter >= self.task.cap_dwell_frames:
                    self.state = "settled"
                    self.settled = True
                    self.task._on_cap_settled(held)
                    print(f"[b3] cap settled, flame extinguished")
            else:
                self.extinguish_counter = 0
            return

        # settled：帽锁灯口（不再跟随夹爪，帽已停在盖灭位），火焰已熄（_on_cap_settled 处理）


class B3WaterBathTask(BaseTask):
    """B3 水浴加热任务：放固体入试管 → 点燃 → 水浴加热 → 固体熔化（或不熔化）→ 盖帽灭火。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 水浴几何（pxr 实测 b3_water_bath.usd 世界包围盒）
    BEAKER_BOTTOM_Z = 0.9205      # 烧杯底（坐石棉网顶）
    WATER_TOP_Z = 1.0055          # 烧杯水面（水浴）
    TUBE_BOTTOM_Z = 0.9555        # 试管底（浸入烧杯水 5cm）
    TUBE_MOUTH_Z = 1.1088         # 试管口

    # 气泡上升动画（照搬 B2/d3l 动态池：连续生成 + 速度差异 + 蛇形 + 破灭复用）。
    # 烧杯水浴比试管大 → 泡环带约束（避中心试管 + 避烧杯壁），见 gen _gen_beaker_bubbles。
    BUBBLE_RISE = 0.0010          # 每帧上升量（m，@60Hz ≈ 0.06m/s）
    BUBBLE_SPAWN_INTERVAL = 4     # 全速生成间隔帧（实际 = 本值 / _bubble_vigor）
    BUBBLE_WOBBLE_AMP = 0.0012    # 上升蛇形摆动振幅（±1.2mm）
    BUBBLE_MIN_RADIUS = 0.012     # 气泡中心离管轴最小半径（试管外径 0.0096 + 泡 0.002 余量）
    BUBBLE_MAX_RADIUS = 0.029     # 气泡中心离管轴最大半径（水柱 0.031 − 泡 0.002）
    BUBBLE_SPAWN_Z = BEAKER_BOTTOM_Z + 0.01   # 生成高度（烧杯底上方 10mm，网加热区）

    # prim 路径
    SOLID = "/World/SolidSample"
    SOLID2 = "/World/SolidSample2"
    SOLID_FALL_FRAMES = 18        # 固体坠落帧数（~15cm 加速坠落，仿 B2 沸石）
    MATCH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15  # 火柴头近灯芯连续帧数阈值（仿 B2/flametest）
    MATCH_IGNITE_DIST = 0.035      # 火柴头距灯芯 < 3.5cm 判定点火接近
    CAP = "/World/AlcoholLamp/cap"
    MELT_PREFIX = EFFECT_TUBE_MELT  # "/World/TubeMelt"（+ "_" + melt_color）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 温度模型参数（config 顶层可调，同 B2）
        self.room_temp = float(getattr(cfg, "room_temp", 25.0))
        self.heat_rate = float(getattr(cfg, "heat_rate", 0.15625))
        self.boiling_point = float(getattr(cfg, "boiling_point", 100.0))
        self.idle_dwell_frames = int(getattr(cfg, "idle_dwell_frames", 20))
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.boil_dwell_frames = int(getattr(cfg, "boil_dwell_frames", 300))
        self.bubble_fade_frames = int(getattr(cfg, "bubble_fade_frames", 240))
        # 灯帽：阈值 + 结果停留（同 B2）
        self.cap_closed_threshold = CAP_CLOSED_THRESHOLD     # 帽 attach 阈值（帽 Ø37mm）
        self.cap_dwell_frames = int(getattr(cfg, "cap_dwell_frames", 15))  # 盖到位连续帧
        self.cap_cover_near = CAP_COVER_NEAR                 # 夹爪距 CAP_BURNER 盖到位近窗
        self.cap_result_hold_frames = int(getattr(cfg, "cap_result_hold_frames", 120))
        self._cap_done_frames = 0
        self.flame_extinguished = False
        # 实验结果：sample_phase=melted/unchanged（固体加热后是否熔化）、melt_color=熔化液色
        self.sample_phase = str(getattr(cfg, "sample_phase", "melted")).strip().lower()
        self.melt_color = str(getattr(cfg, "melt_color", "clear")).strip().lower()
        self._melt_path = f"{self.MELT_PREFIX}_{self.melt_color}"
        self.melt_revealed = False

        # 夹爪阈值（同 B2）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 气泡球组（骨架 Sphere，排除 bubble_mat 材质 prim）
        self.bubble_prims = self._children(EFFECT_BEAKER_BUBBLES)
        self.bubble_base = [self._read_translate(p) for p in self.bubble_prims]
        self._bubble_bases = [(float(b[0]), float(b[1])) for b in self.bubble_base]
        self._bubble_z = [self.BUBBLE_SPAWN_Z] * len(self.bubble_prims)
        self._bubble_active = [False] * len(self.bubble_prims)
        self._bubble_age = [0] * len(self.bubble_prims)
        self._bubble_speed = [0.85 + 0.3 * ((i * 37) % 100) / 100.0
                              for i in range(len(self.bubble_prims))]
        self._bubble_phase = [(i * 0.7) % (2.0 * np.pi) for i in range(len(self.bubble_prims))]
        self._bubble_spawn_timer = 0
        self._bubble_vigor = 0.0
        # 火焰 prim（水滴形两焰×2=4，迁 /World 顶层）
        self.flame_prims = self._flame_paths()
        self.flame_base = [self._read_translate(p) for p in self.flame_prims]

        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._bubble_fade = 0

        # 固体（阶段A 放固体入试管）：静态碰撞体，吸附期关碰撞；grasp/drop 两点位姿
        self._disable_collision(self.SOLID)
        self._disable_collision(self.SOLID2)
        solid_drop = (TUBE_XY[0], TUBE_XY[1], SOLID_DROP_Z)
        self.solids = [
            _SolidLifecycle(self, "solid1", self.SOLID, SOLID_GRASP, solid_drop, 0.0),
            _SolidLifecycle(self, "solid2", self.SOLID2, SOLID2_GRASP, solid_drop, SOLID_STACK_DZ),
        ]
        self.solid_added = False      # 两颗固体都沉底（idle 门控）
        # 火柴（阶段B 点燃酒精灯）：静态碰撞体，吸附期关碰撞；rest/grasp 两点位姿
        self._disable_collision(self.MATCH)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH, match_rest, MATCH_GRASP)
        self.flame_lit = False        # 火柴触灯芯点燃（idle 门控：火焰 reveal）
        self.match_ignite_counter = 0
        # 灯帽（阶段C 盖帽灭火）：帽是灯子 prim（碰撞已随灯 disable）
        self.cap = _CapLifecycle(self, "cap", self.CAP, CAP_GRASP, CAP_BURNER)
        self.cap_rest_translate = self._read_translate(self.CAP)

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._bubble_fade = 0
        self._bubble_vigor = 0.0
        self._bubble_spawn_timer = 0
        self.solid_added = False
        self.flame_lit = False
        self.flame_extinguished = False
        self.melt_revealed = False
        self._cap_done_frames = 0
        self.match_ignite_counter = 0
        self._set_visible(self._flame_paths(), False)
        self._set_visible(self.bubble_prims, False)
        for i in range(len(self.bubble_prims)):
            self._bubble_active[i] = False
            self._bubble_z[i] = self.BUBBLE_SPAWN_Z
            self._bubble_age[i] = 0
            self._set_translate(self.bubble_prims[i], self.bubble_base[i])
        # 固体复位：回玻璃皿静止位（底贴皿顶）
        for s in self.solids:
            s.reset()
        self._set_visible([self.SOLID, self.SOLID2], True)
        # 火柴复位：回台面静止位
        self.match.reset()
        # 灯帽复位：回静止位（帽相对灯 local translate 写回）
        self.cap.reset()
        for p, base in zip(self.flame_prims, self.flame_base):
            self._set_translate(p, base)
        # 熔化液柱复位：全隐藏
        for name in ("clear", "red", "blue", "green", "purple"):
            self._set_visible(f"{self.MELT_PREFIX}_{name}", False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        for s in self.solids:
            s.step(gripper_pos, opening)
        self.match.step(gripper_pos, opening)
        self.cap.step(gripper_pos, opening)
        self._step_match_ignite(gripper_pos)   # 点火检测（火柴头触灯芯 → flame_lit）
        self.solid_added = all(s.settled for s in self.solids)
        self._update_experiment()
        return self.get_basic_state_info(additional_info={
            "phase": self.phase,
            "temperature": round(self.temperature, 1),
            "boiling_point": self.boiling_point,
            "flame_on": self.phase != "idle",
            "flame_lit": self.flame_lit,
            "solid_added": self.solid_added,
            "match_attached": self.match.attached,
            "cap_attached": self.cap.attached,
            "cap_settled": self.cap.settled,
            "flame_extinguished": self.flame_extinguished,
            "sample_phase": self.sample_phase,
            "melt_color": self.melt_color,
            "melt_revealed": self.melt_revealed,
        })

    # ------------------------------------------------------------------
    # 相态机：idle（等固体放好+点火）→ ignited → heating → boiling → cap_lamp → done
    # ------------------------------------------------------------------
    def _update_experiment(self):
        if self.phase == "idle":
            if self.solid_added and self.flame_lit:
                self.phase = "ignited"
                self._set_visible(self._flame_paths(), True)
                print(f"[b3] ignite: flame on (match lit) @ frame {self.frame_idx}")

        elif self.phase == "ignited":
            if self.frame_idx >= 5 + self.idle_dwell_frames + self.ignite_dwell_frames:
                self.phase = "heating"
                print(f"[b3] heating start T={self.temperature:.1f}")

        elif self.phase == "heating":
            self.temperature = min(self.boiling_point, self.temperature + self.heat_rate)
            progress = (self.temperature - self.room_temp) / max(
                1e-6, self.boiling_point - self.room_temp)
            self._bubble_vigor = progress
            if progress > 0:
                self._step_bubble_anim()
            if self.temperature >= self.boiling_point:
                self.phase = "boiling"
                self._on_melt()   # 水到沸点：固体熔化（或不熔化）
                print(f"[b3] boiling at T={self.temperature:.1f}"
                      f"{' -> melt ' + self.melt_color if self.melt_revealed else ' (solid unchanged)'}")

        elif self.phase == "boiling":
            self._boil_frames += 1
            self._bubble_vigor = 1.0
            self._step_bubble_anim()
            # 沸腾保持 boil_dwell_frames（config 300 = 5s @60fps）→ 进入盖帽相（气泡不熄）
            if self._boil_frames >= self.boil_dwell_frames:
                self.phase = "cap_lamp"
                print(f"[b3] boiling hold done -> cap lamp phase (grab cap, cover lamp in place)")

        elif self.phase == "cap_lamp":
            # 机械臂 CapLampPass 原位盖帽中：帽被夹走 → 火焰仍亮；帽盖到位 settled → 火焰熄灭
            # （_on_cap_settled）。熄火后气泡逐个渐熄（bubble_fade_frames），全熄 + 停留足够 → done。
            if self.cap.settled:
                self._cap_done_frames += 1
                self._bubble_fade += 1
                fade_progress = self._bubble_fade / max(1e-6, self.bubble_fade_frames)
                self._bubble_vigor = max(0.0, 1.0 - fade_progress)
                self._step_bubble_anim()
                if self._bubble_fade >= self.bubble_fade_frames:
                    self._set_visible(self.bubble_prims, False)
                if (self._cap_done_frames >= self.cap_result_hold_frames
                        and self._bubble_fade >= self.bubble_fade_frames):
                    self.phase = "done"
                    print(f"[b3] done: cap covers lamp, flame extinguished, "
                          f"sample_phase={self.sample_phase} melt_color={self.melt_color}")
            else:
                # 盖帽过程中（帽未盖到位）：气泡继续沸腾
                self._bubble_vigor = 1.0
                self._step_bubble_anim()

    def _on_melt(self):
        """水到沸点：sample_phase=melted 时揭示熔化液柱 + 隐藏固体（熔化消失）。"""
        if self.sample_phase == "melted":
            self.melt_revealed = True
            self._set_visible([self.SOLID, self.SOLID2], False)
            self._set_visible(self._melt_path, True)
        # unchanged：固体保持白颗粒，无熔化液柱

    def _step_bubble_anim(self):
        """气泡动画（照搬 B2/d3l 动态池，改烧杯水浴环带约束）：小球池按「间隔 =
        BUBBLE_SPAWN_INTERVAL / _bubble_vigor」从烧杯底生成一颗、逐帧上升（速度差异 + 蛇形
        摆动）、到水面隐藏（破灭）复用。只写子球 translate；离管轴半径钳到 [MIN,MAX] 防穿管/壁。"""
        pop_z = self.WATER_TOP_Z - 0.002
        if self._bubble_vigor > 0:
            if self._bubble_spawn_timer <= 0:
                for i, active in enumerate(self._bubble_active):
                    if not active:
                        self._bubble_active[i] = True
                        self._bubble_z[i] = self.BUBBLE_SPAWN_Z
                        self._bubble_age[i] = 0
                        self._set_visible(self.bubble_prims[i], True)
                        break
                self._bubble_spawn_timer = max(1, round(self.BUBBLE_SPAWN_INTERVAL / self._bubble_vigor))
            else:
                self._bubble_spawn_timer -= 1
        for i, (bx, by) in enumerate(self._bubble_bases):
            if not self._bubble_active[i]:
                continue
            age = self._bubble_age[i]
            z = self._bubble_z[i] + self.BUBBLE_RISE * self._bubble_speed[i]
            if z >= pop_z:
                self._bubble_active[i] = False
                self._set_visible(self.bubble_prims[i], False)
                continue
            self._bubble_z[i] = z
            ph = self._bubble_phase[i]
            wob = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.15 + ph)
            woy = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.13 + ph + 1.7)
            cx, cy = bx + wob, by + woy
            dx, dy = cx - TUBE_XY[0], cy - TUBE_XY[1]
            r = np.hypot(dx, dy)
            if r < self.BUBBLE_MIN_RADIUS:      # 避中心试管（Ø19.2mm 外径）
                s = self.BUBBLE_MIN_RADIUS / r
                cx, cy = TUBE_XY[0] + dx * s, TUBE_XY[1] + dy * s
            elif r > self.BUBBLE_MAX_RADIUS:    # 避烧杯壁（水柱 r0.031）
                s = self.BUBBLE_MAX_RADIUS / r
                cx, cy = TUBE_XY[0] + dx * s, TUBE_XY[1] + dy * s
            self._bubble_age[i] = age + 1
            self._set_translate(self.bubble_prims[i], (cx, cy, z))

    # ------------------------------------------------------------------
    # 固体矩阵持握（阶段A：固体中心落夹爪处，随夹爪平移/旋转 → 松爪坠落沉底）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _solid_held_matrix(self):
        """固体当前持握世界矩阵 = _SOLID_HELD · tool_world（固体中心落在夹爪处）。"""
        return _SOLID_HELD * self._tool_world()

    def _set_solid_world(self, path, world_matrix):
        """把固体写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _solid_translate_matrix(self, center_pos):
        """固体中心在 center_pos 的恒等旋转位姿（固体底 = 中心 - 半高 SOLID_CENTER_OFFSET）。"""
        cx, cy, cz = center_pos
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           cx, cy, cz - SOLID_CENTER_OFFSET, 1.0)

    def _solid_rest_matrix(self, grasp):
        """固体皿上静止位姿（中心在 grasp，底贴皿顶）。"""
        return self._solid_translate_matrix(grasp)

    def _solid_settle_center(self, settle_dz=0.0):
        """固体沉底后固体中心世界坐标（管口 xy、z = 管底 + 半高 + settle_dz，穿过水面沉底）。"""
        return np.array([TUBE_XY[0], TUBE_XY[1],
                         self.TUBE_BOTTOM_Z + SOLID_CENTER_OFFSET + settle_dz])

    def _solid_settle_matrix(self, settle_dz=0.0):
        """固体沉底位姿（恒等旋转，中心在管底 + 半高 + settle_dz）。"""
        return self._solid_translate_matrix(self._solid_settle_center(settle_dz))

    def _ease_solid_world(self, path, target, k=0.18):
        """夹爪合拢期间固体逐帧平滑移向持握位（消除闪现吸附）。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_solid_world(path, _blend_world(cur, target, k))

    def _is_closest_solid(self, solid):
        """这颗固体能否附着：离夹爪最近的 rest 态固体，且无其他固体正被夹/坠落。"""
        gripper_pos = self.robot.get_gripper_position()
        d_self = np.linalg.norm(np.asarray(gripper_pos) - solid.grasp)
        for other in self.solids:
            if other is solid:
                continue
            if other.state in ("attached", "released"):
                return False
            if other.state == "rest":
                if np.linalg.norm(np.asarray(gripper_pos) - other.grasp) < d_self:
                    return False
        return True

    # ------------------------------------------------------------------
    # 火柴纯平移持握（阶段B：火柴横躺水平头朝 +X，只跟夹爪平移不随旋转）
    # ------------------------------------------------------------------
    def _match_rest_pos(self):
        return np.array([MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z])

    def _set_match_world(self, position):
        self._set_obj_world(self.MATCH, position)

    def _ease_match_world(self, target, k=0.18):
        self._ease_obj_world(self.MATCH, target, k)

    def _match_tip(self, gripper_pos):
        """火柴头中心世界坐标 = 夹爪 + MATCH_TIP_OFFSET（头在夹爪 +X 0.0494）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)

    def _step_match_ignite(self, gripper_pos):
        """点火检测（仿 B2/flametest）：火柴 attached 期间头近灯芯连续帧 → flame_lit=True。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    print(f"[b3] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    # ------------------------------------------------------------------
    # 灯帽持握（阶段C：纯平移持握，帽中心 = 夹爪 + CAP_HELD_OFFSET，帽是灯子 prim）
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
        """把帽写到给定帽中心世界坐标（帽 = 灯子 prim，只写帽 translate op，保留 rotateXYZ
        +scale 形状）。换算（pxr 实测，灯 R180Z）：cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+CAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−CAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world("/World/AlcoholLamp")
        if lamp_pos is None:
            return
        cx, cy, cz = center
        tx = lamp_pos[0] - cx
        ty = lamp_pos[1] - cy
        tz = cz - lamp_pos[2] - CAP_CENTER_DZ
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
        cur_center = np.asarray(cur_origin, dtype=float) + np.array([0.0, 0.0, CAP_CENTER_DZ])
        nxt = cur_center + (np.asarray(target, dtype=float) - cur_center) * k
        self._set_cap_world(nxt)

    def _extinguish_flame(self):
        """熄灭火焰（幂等：下落即熄/盖到位都调，只灭一次）。"""
        if self.flame_extinguished:
            return
        self.flame_extinguished = True
        self._set_visible(self._flame_paths(), False)
        print(f"[b3] flame extinguished @ frame {self.frame_idx}")

    def _on_cap_settled(self, center):
        """帽盖到位：火焰熄灭、帽锁灯口（settled 态不再跟随夹爪，帽停在盖灭位）。"""
        self._extinguish_flame()

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _flame_paths(self):
        return ["/World/flame_outer", "/World/flame_outer_sphere",
                "/World/flame_inner", "/World/flame_inner_sphere"]

    def _get_obj_world(self, path):
        """物体原点世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            self.object_utils.set_object_position(path, np.asarray(position, dtype=float))

    def _ease_obj_world(self, path, target, k=0.18):
        cur = self._get_obj_world(path)
        if cur is None:
            return
        nxt = cur + (target - cur) * k
        self._set_obj_world(path, nxt)

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

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def on_task_complete(self, success):
        print(f"[b3] episode done success={success} phase={self.phase} "
              f"solid_added={self.solid_added} "
              f"flame_lit={self.flame_lit} "
              f"cap_settled={self.cap.settled} "
              f"flame_extinguished={self.flame_extinguished} "
              f"sample_phase={self.sample_phase} melt_color={self.melt_color} "
              f"melt_revealed={self.melt_revealed} temp={self.temperature:.1f}°C")
        super().on_task_complete(success)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
