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
    → boiling（到 cfg.boiling_point，气泡 reveal）→ done（保持
    boil_dwell_frames 后完成，上报沸点）。全部 phase 通过
    get_basic_state_info(additional_info) 上报，controller 读到 phase=="done" 即报成功。

滴管生命周期（照 d3l _DropperLifecycle，gripper 开度 = joint[7]，判定纯关节+TCP）：
    rest → attached → squeezed → filled → dropped → released
    - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
    - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
    - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
    - filled   跟随；试管口区挤胶头（<GRIP_SQUEEZED）→ dropped → 一次成串液滴坠落 +
               管内液面逐滴升高（无 DropperFill 固定液柱，2026-08-25 用户删）
    - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（一次
               持握内循环吸液-滴液）；末遍滴完回架松开（>gripper_open）→ released
               （写回架内竖插位姿）并复位 rest

持握照 d2s 夹药匙（2026-08-25 用户纠正：滴管水平横着夹在管身上避开挂钩支臂）：滴管是
静态碰撞体，吸附期逐帧把**世界 4x4 矩阵**写为 _T_HELD · tool_world（d2s 同款，清 xform
op 表 + 单 transform op 写矩阵）——滴管沿 tool+X 伸出、随夹爪旋转：手指朝前(ORIENT_FWD)=
滴管竖直挂夹爪下（尖嘴朝下）：吸液/低空横越/竖直滴液全程保持 ORIENT_FWD（2026-08-25
用户：滴加竖着在试管口滴，没必要倾斜，同 d2l）。尖嘴 = 夹爪 + 0.13·tool+X 方向
（d2s 混合数据源：物理位置+USD 方向），效果 prim 跟随即可。

温度计读数（capillary_liquid 锚定缩放，pxr 已验证，v1 保留）：
    毛细红液柱 mesh z[0.005,0.245]（全量程 -20..110°C 刻度区顶）。每帧写单一
    transform op：M = T(0,0,-0.005) · S(1,1,s) · T(0,0,+0.005)，s = (z_of(T)-0.005)/0.24，
    底锚 z=0.005 不动、柱顶 = z_of(T) 随温度爬升（行向量约定：pxr 中 A·B 表示 A 先作用）。

驱动 prim（b2_alcohol_heat_liquid.usd，由 scripts/gen_b2_scene.py 生成）：
    /World/flame_outer|flame_inner   火焰（Cone，迁到 /World 顶层——灯下引用子 prim RTX 不渲染）
    /World/Thermometer/Thermometer/capillary_liquid  温度计毛细红液柱（锚定缩放）
    /World/TestTubeBubbles/bubble_{0..7}         沸腾气泡（球组 r2.5mm 亮白，初始隐藏，上升动画）
    /World/Dropper / DropperDrop / TestTubeLiquid  滴加效果（阶段B；DropperFill 固定液柱
    已删，2026-08-25 用户「滴管里面固定竖直的液柱很奇怪+移动时浅色轨迹」→ 参考 d2l 无液柱）
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_REST, DROP_GRASP,
    SAMPLE_BOTTLE_XY, TUBE_XY, TUBE_MOUTH_Z,
    THERMO_XY, THERMO_REST_Z, THERMO_GRASP, THERMO_GRASP_OFFSET,
    THERMO_HANG,
    ZEO_GRASP, ZEO2_GRASP, ZEO_CENTER_OFFSET, ZEO_DROP_Z, ZEO_STACK_DZ,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    EFFECT_TUBE_DROPS, EFFECT_DROPPER_DROP,
)

# 滴管相对夹爪的持握矩阵 _T_HELD（d2s 夹药匙同款，2026-08-25 用户：滴管要像 d2s 药匙那样
# 水平横着夹在管身上、不能手指朝下竖直夹）。行向量约定：行=滴管局部轴在 tool 系的像、
# 平移在最后一行。滴管局部原点=尖嘴底、+Z=朝胶头。映射：滴管 X→tool -Z、Y→tool -Y、
# Z→tool -X，平移 TIP_OFFSET 沿 tool +X ——被夹物沿"手指侧面"伸出 0.13m、长轴与手指垂直
# （d2s 药匙同款 _T_HELD）。合成：滴管世界 = _T_HELD · tool_world（先 _T_HELD 作用滴管
# 局部系、再 tool_world 到世界；写反旋转作用到世界系 → 滴管翻走/飞桌面下）。
# 抓取 = d2s 药匙式：手指朝前(ORIENT_FWD)水平横着夹住竖直管身，attach 后滴管保持竖直
# （胶头朝上、尖嘴朝下）挂在夹爪前下方，与架内竖插姿态零跳变；尖嘴 = 夹爪 + 0.13·tool+X
# 方向（见 _dropper_tip_pos）。
#  手指朝前(ORIENT_FWD，tool+X=世界-Z) → 滴管竖直挂夹爪下、尖嘴朝下：吸液浸瓶口 /
#    低空横越（杆体 z = 夹爪-0.13 ~ 夹爪，全程压挂钩支臂 1.216 之下）/ 两段式滴液
#    （先到管口偏 -X 侧、再只水平移 x 到管口正上方，尖嘴在管口上 2mm——2026-08-25
#    用户弃 45° 倾斜 + 弃沉入管内，避免穿试管壁）
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      TIP_OFFSET, 0.0, 0.0, 1.0)

# 温度计相对夹爪的持握矩阵 _THERMO_HELD（阶段 D 段 1，2026-08-26 用户逐字：「首先
# 需要也是水平横着夹住温度计」——d2s 夹药匙同款，手指朝前 ORIENT_FWD 横着夹竖直杆身）。
# 旋转与 _T_HELD 完全一致（滴管/温度计都用同一「横夹竖直杆」持握），仅平移改抓点偏移
# THERMO_GRASP_OFFSET=0.20（温度计局部原点=泡尖 → 抓点沿杆身 +Z 0.20，杆身上段偏下；
# 旧 0.254 太靠近挂环，用户「夹的位置太高了」）。
# 手指朝前(ORIENT_FWD)时 tool+X=世界-Z → 温度计竖直挂夹爪下、泡尖朝下、挂环（抓点上方
# 0.0695）在夹爪之上。泡尖 = 夹爪 + 0.20·tool+X；挂环中心 = 夹爪 + 0.0695·tool-X。
# 段 1 倾斜 20° 靠朝向驱动（ORIENT_TILT_20，2026-08-26 用户 25°→10°→15°→20° 逐级），
# 矩阵随夹爪旋转（泡尖摆到试管上方落管口上方 5mm）；段 2（插入+挂环套钩棍）未实现。
_THERMO_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           -1.0, 0.0, 0.0, 0.0,
                           THERMO_GRASP_OFFSET, 0.0, 0.0, 1.0)

# 沸石相对夹爪的持握矩阵 _ZEO_HELD（阶段 C 放沸石，2026-08-27 用户新增）。旋转与
# _T_HELD/_THERMO_HELD 完全一致（沸石 Z 朝上→tool -X、X→tool -Z、Y→tool -Y），仅平移改
# ZEO_CENTER_OFFSET=0.0037（沸石半高）。沸石局部原点=底 z=0、+Z 朝上（新资产 2026-08-27
# 重建）。竖直夹（默认朝向手指朝下，tool+X 朝世界 -Z）→ 沸石底在夹爪下方 0.0037、
# 沸石中心落在夹爪处（夹爪两指夹住颗粒最宽处）；旋转 ORIENT_FWD 后沸石随夹爪、位置不变
# （颗粒旋转对称，朝向差异无视觉影响）。沸石中心 = 夹爪（tool 原点），抓点 = 沸石中心。
_ZEO_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                        0.0, -1.0, 0.0, 0.0,
                        -1.0, 0.0, 0.0, 0.0,
                        ZEO_CENTER_OFFSET, 0.0, 0.0, 1.0)


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    样品瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
      tube_xy      试管口 xy（滴液区）
    """

    def __init__(self, task, name, path, orig, grasp, bottle_xy, tube_xy):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.bottle_xy = np.array(bottle_xy)
        self.tube_xy = np.array(tube_xy)
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
        self.task._set_dropper_world(self.task._rest_matrix())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._dropper_held_matrix()
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（d2s 同款全矩阵 ease；
            # 附着手腕已回正→旋转差≈0，只 eases 平移，消除闭合瞬间闪现吸附）。
            # 只在 near 时 ease，避免合爪未遂拖离原位。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_dropper_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_dropper_world(held)
                print(f"[b2] dropper attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（矩阵持握：滴管随夹爪旋转，横越放平/45° 下倾滴液）
        self.task._set_dropper_world(self.task._dropper_held_matrix())
        tip = self.task._dropper_tip_pos()

        if self.state == "attached":
            # 瓶口区挤胶头排空气（尖嘴在瓶口）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[b2] dropper squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "filled"
                self.filled = True
                print(f"[b2] dropper filled (aspirated)")
        elif self.state == "filled":
            # 试管口区挤胶头滴液（尖嘴在管口上方 2mm）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.tube_xy, tip)):
                self.state = "dropped"
                self.dropped = True
                self.task._on_drop(self)
                print(f"[b2] dropper dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, tip)):
                self.state = "squeezed"
                print(f"[b2] dropper re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_dropper_world(self.task._rest_matrix())
                self.state = "rest"
                print(f"[b2] dropper released to rack -> rest")


class _ThermometerLifecycle:
    """单支温度计状态机（rest → attached → hung，阶段 D 挂温度计）。

    持握 = 矩阵 _THERMO_HELD · tool_world（d2s 夹药匙同款横夹竖直杆身，手指朝前），
    温度计随夹爪旋转：竖直提出 → 高位运到管口 -X 侧 → 原地倾斜 20° → 竖直下探泡尖
    落管口上方 5mm（段 1）→ 位置+朝向同步插值（ThermoInsertRotate ⑧，边下探边转垂直）
    → 挂环套铁架台钩、泡尖伸进试管（段 2）→ 松爪（grip 打开，⑨）→ hung：温度计
    自然下垂挂在钩上（泡尖在管内浸液面），锁挂位矩阵不再跟随夹爪。
    参考点（均为 gripper/TCP 世界坐标）：
      grasp   架右后排孔抓点（夹爪 z = THERMO_GRASP）
      rest    架内立放静止位（泡尖=原点，THERMO_XY + THERMO_REST_Z）
      hang    挂位夹爪（THERMO_HANG，松爪判定；挂环套钩后温度计锁 THERMO_HANG 挂位）
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
        self.hung = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.hung = False
        self.task._set_thermo_world(self.task._thermo_rest_matrix())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._thermo_held_matrix()
            # 夹爪合拢且进近窗：先把温度计平滑拉向持握位（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_thermo_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_thermo_world(held)
                print(f"[b2] thermometer attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 段 2 松爪判定：夹爪已到挂位（ThermoInsertRotate ⑧ 终点）且 grip 打开（⑨）
            # → 挂环套钩完成，转 hung 锁挂位矩阵（温度计不再跟随夹爪，自然下垂）
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(self.task._thermo_hang_grasp(), gripper_pos)):
                self.state = "hung"
                self.hung = True
                self.task._set_thermo_world(self.task._thermo_hang_matrix())
                print(f"[b2] thermometer hung on hook (grip={opening:.4f})")
            else:
                # 吸附期：逐帧跟随夹爪（矩阵持握，随夹爪旋转/倾斜：高位运到管口 → 倾斜 → 下探转垂直）
                self.task._set_thermo_world(self.task._thermo_held_matrix())
            return

        # hung：温度计挂在铁架台钩上自然下垂（泡尖在管内浸液面），锁挂位矩阵
        self.task._set_thermo_world(self.task._thermo_hang_matrix())


class _ZeoliteLifecycle:
    """单块沸石状态机（rest → attached → released → settled，阶段 C 放沸石）。

    持握 = 矩阵 _ZEO_HELD · tool_world（沸石中心落在夹爪处，颗粒随夹爪平移/旋转）。
    竖直夹（默认朝向手指朝下）→ 旋转手指朝前 → 水平伸到试管上方 → 松爪 → 沸石从
    夹爪坠落进试管沉底（settled，锁管底位姿，不再跟随夹爪）。两颗沸石各一实例：
    第一颗沉底、第二颗叠第一颗顶（settle_dz=ZEO_STACK_DZ，管底内 Ø11.5mm 只容一颗
    并排）。两颗抓点仅距 2cm（< 近窗 3cm），rest 态用「离夹爪最近那颗」门禁防同时误抓。
    参考点（gripper/TCP 世界坐标）：
      grasp   皿上沸石中心抓点（ZEO_GRASP / ZEO2_GRASP）
      drop    管口正上方放下位（夹爪 x/y = 管口、z = ZEO_DROP_Z，松爪判定）
      settle  沸石中心沉底位（管口 xy、z = 管底 + 半高 + settle_dz）
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
        self.task._set_zeolite_world(self.path, self.task._zeolite_rest_matrix(self.grasp))

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            # 两颗沸石抓点仅 2cm 在近窗内：只允许「离夹爪最近」的 rest 沸石附着（防同时误抓两颗）
            if not self.task._is_closest_zeolite(self):
                self._near_frames = 0
                return
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._zeolite_held_matrix()
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_zeolite_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_zeolite_world(self.path, held)
                print(f"[b2] zeolite attached {self.name} (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 松爪判定：夹爪到管口正上方放下位且 grip 打开（⑨）→ released，沸石坠落
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(self.drop, gripper_pos)):
                self.state = "released"
                self.released = True
                self._fall_start = np.asarray(gripper_pos, dtype=float)  # 夹爪=沸石中心
                self._fall_t = 0
                print(f"[b2] zeolite released over tube {self.name} (grip={opening:.4f})")
            else:
                self.task._set_zeolite_world(self.path, self.task._zeolite_held_matrix())
            return

        if self.state == "released":
            # 坠落动画：沸石中心从放下位加速坠落到管底，落定转 settled 锁管底位姿
            self._fall_t += 1
            start = np.asarray(self._fall_start, dtype=float)
            target = self.task._zeolite_settle_center(self.settle_dz)
            if self._fall_t >= self.task.ZEO_FALL_FRAMES:
                self.state = "settled"
                self.settled = True
                self.task._set_zeolite_world(self.path, self.task._zeolite_settle_matrix(self.settle_dz))
                print(f"[b2] zeolite settled {self.name} at tube bottom")
            else:
                frac = self._fall_t / self.task.ZEO_FALL_FRAMES
                center = start + (target - start) * (frac * frac)   # 加速坠落（t²）
                self.task._set_zeolite_world(self.path, self.task._zeolite_translate_matrix(center))
            return

        # settled：沸石沉在管底（穿过液面），锁管底位姿
        self.task._set_zeolite_world(self.path, self.task._zeolite_settle_matrix(self.settle_dz))


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段 E 点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （与滴管/温度计/沸石的矩阵持握不同——火柴杆横躺，夹爪手指朝下竖直夹其杆身）。
    释放时写回台面静止位（flametest 同款：高位松爪后火柴写回 rest）。
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
                print(f"[b2] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位 MATCH_LIFT_Z）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[b2] match released to rest")


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

    # 气泡上升速度（m/帧）
    BUBBLE_SPEED = 0.004

    # 水蒸气两段式（2026-08-27 用户「雾气太假：整个柱体超过试管；真实沸腾雾先只在试管
    # 范围内」→ 改两段）：steam_inner 管内雾（Cylinder r0.007 < 管内径，底锚液面，长到
    # 管口，只填试管内部）+ steam_plume 管口羽流（尖端朝下 Cone，apex 钉管口 1.0739，
    # 升出管口后扩散消散）。时序：管内雾先长满（雾先只在试管里），羽流才从管口升起。
    STEAM = "/World/TestTubeSteam"
    STEAM_MOUTH_Z = TUBE_MOUTH_Z      # 管口 1.0739（管内雾上限 / 羽流 apex）
    STEAM_INNER_R = 0.007             # 管内雾半径（< 管内径 Ø17/2=0.0085，被管壁约束）
    STEAM_PLUME_R = 0.016             # 管口上羽流顶半径（上升扩散）
    STEAM_PLUME_H = 0.12              # 羽流高（管口 → 1.1939，挂钩底 1.216 之下）
    STEAM_GROW_FRAMES = 90            # 两段总帧数（~1.5s @60fps）
    STEAM_INNER_SHARE = 0.6           # 前 60% 帧：管内雾先长满（雾先在试管范围内）

    # 滴加效果 prim 路径
    DROPPER = "/World/Dropper"
    TUBE_DROPS = EFFECT_TUBE_DROPS
    DROPPER_DROP = EFFECT_DROPPER_DROP
    # 温度计（阶段 D 挂温度计）：/World/Thermometer（外 translate (0.521,0.468,0.808)）
    THERMOMETER = "/World/Thermometer"
    # 沸石（阶段 C 放沸石）：/World/Zeolite、/World/Zeolite2（并排 ±1cm 沿 x，2026-08-27 两颗）
    ZEOLITE = "/World/Zeolite"
    ZEOLITE2 = "/World/Zeolite2"
    ZEO_FALL_FRAMES = 18   # 沸石坠落帧数（~18cm 加速坠落，仿液滴 t²）
    # 火柴（阶段 E 点燃酒精灯）：/World/Match（抬高 12mm，头朝灯芯）
    MATCH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 温度模型参数（config 顶层可调，v1 保留）
        self.room_temp = float(getattr(cfg, "room_temp", 25.0))
        self.heat_rate = float(getattr(cfg, "heat_rate", 0.08))
        self.boiling_point = float(getattr(cfg, "boiling_point", 100.0))
        self.idle_dwell_frames = int(getattr(cfg, "idle_dwell_frames", 20))
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.boil_dwell_frames = int(getattr(cfg, "boil_dwell_frames", 240))

        # 气泡球组（骨架 Sphere，排除 bubble_mat 材质 prim）
        self.bubble_prims = self._children("/World/TestTubeBubbles")
        self.bubble_base = [self._read_translate(p) for p in self.bubble_prims]
        # 水蒸气雾柱（同心双层 Cylinder，_children 已滤到几何）：初始隐藏高 0，沸腾相长高
        self.steam_prims = self._children(self.STEAM)
        self._steam_grow = 0

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
            SAMPLE_BOTTLE_XY, TUBE_XY)
        # 温度计（阶段 D 段 1）：静态碰撞体，吸附期关碰撞；rest/grasp 两点位姿
        self._disable_collision(self.THERMOMETER)
        thermo_rest = (THERMO_XY[0], THERMO_XY[1], THERMO_REST_Z)
        self.thermometer = _ThermometerLifecycle(
            self, "thermometer", self.THERMOMETER,
            thermo_rest, THERMO_GRASP)
        # 沸石（阶段 C 放沸石）：静态碰撞体，吸附期关碰撞；grasp/drop 两点位姿
        self._disable_collision(self.ZEOLITE)
        self._disable_collision(self.ZEOLITE2)
        zeo_drop = (TUBE_XY[0], TUBE_XY[1], ZEO_DROP_Z)
        self.zeolites = [
            _ZeoliteLifecycle(self, "zeolite1", self.ZEOLITE, ZEO_GRASP, zeo_drop, 0.0),
            _ZeoliteLifecycle(self, "zeolite2", self.ZEOLITE2, ZEO2_GRASP, zeo_drop, ZEO_STACK_DZ),
        ]
        self.zeolite_added = False     # 两颗沸石都沉底（idle 门控）
        # 火柴（阶段 E 点燃酒精灯）：静态碰撞体，吸附期关碰撞；rest/grasp 两点位姿
        self._disable_collision(self.MATCH)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH, match_rest, MATCH_GRASP)
        self.flame_lit = False         # 火柴触灯芯点燃（idle 门控：火焰 reveal）
        self.match_ignite_counter = 0  # 火柴头近灯芯连续帧计数
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
        self.zeolite_added = False
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._liquid_level = 0.0
        self._set_visible(self._flame_paths(), False)
        self._set_visible(self.bubble_prims, False)
        for p, base in zip(self.bubble_prims, self.bubble_base):
            self._set_translate(p, base)
        self._steam_grow = 0
        # 可见性父门控（同 DropperDrop 良方）：scene 中 steam_inner/plume 是默认可见、
        # 由父 Xform 隐藏；reset/沸腾 只 toggle 父即可（子继承），不可只藏子（父 invisible
        # 会阻挡所有后代渲染）。
        self._set_visible(self.STEAM, False)
        for p in self.steam_prims:
            prim = self.stage.GetPrimAtPath(p)
            if not prim.IsValid():
                continue
            if prim.GetName() == "steam_inner":
                UsdGeom.Cylinder(prim).GetHeightAttr().Set(0.0)
            elif prim.GetName() == "steam_plume":
                UsdGeom.Cone(prim).GetHeightAttr().Set(0.0)
                UsdGeom.Cone(prim).GetRadiusAttr().Set(self.STEAM_PLUME_R)
        self._set_capillary(self.room_temp)
        # 滴加复位：滴管回架、液滴/管柱效果隐藏、管柱高度归零
        self.dropper.reset()
        # 温度计复位：回架右后排孔立放位（挂环脱臂、泡尖回孔底）
        self.thermometer.reset()
        # 沸石复位：回玻璃皿静止位（底贴皿顶）
        for z in self.zeolites:
            z.reset()
        # 火柴复位：回台面静止位
        self.match.reset()
        for p in (self.TUBE_DROPS, self.DROPPER_DROP):
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
        for z in self.zeolites:
            z.step(gripper_pos, opening)
        self.thermometer.step(gripper_pos, opening)
        self.match.step(gripper_pos, opening)
        self._step_match_ignite(gripper_pos)   # 点火检测（火柴头触灯芯 → flame_lit）
        self._liquid_added = (self._drop_count
                              >= self.sample_cycles * self.DROPS_PER_SQUEEZE)
        self.zeolite_added = all(z.settled for z in self.zeolites)
        self._update_experiment()
        return self.get_basic_state_info(additional_info={
            "phase": self.phase,
            "temperature": round(self.temperature, 1),
            "boiling_point": self.boiling_point,
            "flame_on": self.phase != "idle",
            "flame_lit": self.flame_lit,
            "dropper_attached": self.dropper.attached,
            "dropper_filled": self.dropper.filled,
            "dropper_dropped": self.dropper.dropped,
            "thermometer_attached": self.thermometer.attached,
            "thermometer_hung": self.thermometer.hung,
            "zeolite_added": self.zeolite_added,
            "match_attached": self.match.attached,
        })

    # ------------------------------------------------------------------
    # 相态机：idle（等滴加完成）→ ignited → heating → boiling → done
    # ------------------------------------------------------------------
    def _update_experiment(self):
        if self.phase == "idle":
            if (self._liquid_added and self.zeolite_added and self.thermometer.hung
                    and self.flame_lit):
                self.phase = "ignited"
                self._set_visible(self._flame_paths(), True)
                print(f"[b2] ignite: flame on (match lit) @ frame {self.frame_idx}")

        elif self.phase == "ignited":
            if self.frame_idx >= 5 + self.idle_dwell_frames + self.ignite_dwell_frames:
                self.phase = "heating"
                print(f"[b2] heating start T={self.temperature:.1f}")

        elif self.phase == "heating":
            self.temperature = min(self.boiling_point, self.temperature + self.heat_rate)
            if self.temperature >= self.boiling_point:
                self.phase = "boiling"
                self._set_visible(self.bubble_prims, True)
                self._set_visible(self.STEAM, True)
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
        """水蒸气两段式（2026-08-27 用户「雾气要先在试管范围内」）：
        阶段A（前 60% 帧）steam_inner 管内雾从实际液面长到管口——雾先只在试管里；
        阶段B（后 40% 帧）steam_plume 管口羽流从管口升起扩散消散（apex 钉管口）。
        到顶后 ±3% 高度呼吸。管内雾半径 < 管内径，被管壁约束绝不超出试管。"""
        self._steam_grow += 1
        progress = min(1.0, self._steam_grow / self.STEAM_GROW_FRAMES)
        base = self.TUBE_BOTTOM_Z + self._liquid_level       # 实际液面（沸腾时已满）
        inner_full = self.STEAM_MOUTH_Z - base               # 液面→管口高度（管内雾上限）
        # 阶段A：管内雾 0→1（t² 缓入，雾积聚感）
        inner_p = min(1.0, progress / self.STEAM_INNER_SHARE)
        inner_h = inner_full * inner_p * inner_p
        # 阶段B：管口羽流（apex 钉管口，上升扩散；半径随高度同涨 = 越升越散）
        plume_p = max(0.0, (progress - self.STEAM_INNER_SHARE) / (1.0 - self.STEAM_INNER_SHARE))
        plume_h = self.STEAM_PLUME_H * plume_p * plume_p
        plume_r = self.STEAM_PLUME_R * plume_p
        if progress >= 1.0:                                  # 到顶后轻微呼吸
            resp = 1.0 + 0.03 * np.sin(self.frame_idx * 0.05)
            inner_h *= resp
            plume_h *= resp
        for p in self.steam_prims:
            prim = self.stage.GetPrimAtPath(p)
            if not prim.IsValid():
                continue
            name = prim.GetName()
            if name == "steam_inner":
                UsdGeom.Cylinder(prim).GetHeightAttr().Set(inner_h)
                UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(
                    Gf.Vec3d(TUBE_XY[0], TUBE_XY[1], base + inner_h / 2))
            elif name == "steam_plume":
                UsdGeom.Cone(prim).GetHeightAttr().Set(plume_h)
                UsdGeom.Cone(prim).GetRadiusAttr().Set(plume_r)
                # apex-down：translate 中心 = apex(管口) + 半高，apex 恒钉管口
                UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(
                    Gf.Vec3d(TUBE_XY[0], TUBE_XY[1], self.STEAM_MOUTH_Z + plume_h / 2))

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

    # ------------------------------------------------------------------
    # 滴管矩阵持握（d2s 同款：滴管是静态碰撞体，吸附期世界位姿 = 4x4 矩阵，
    # 随夹爪旋转——水平横越放平 / 45° 下倾滴液都由朝向驱动，见 constants docstring）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dropper_held_matrix(self):
        """滴管当前持握世界矩阵 = _T_HELD · tool_world（行向量：先 _T_HELD 作用滴管
        局部、再 tool_world 到世界。写反旋转作用到世界系 → 滴管翻走/飞桌面下）。"""
        return _T_HELD * self._tool_world()

    def _set_dropper_world(self, world_matrix):
        """把滴管写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.DROPPER)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _rest_matrix(self):
        """滴管架内竖插静止位姿（与场景 /World/Dropper 世界矩阵一致：立放，尖嘴=原点
        DROP_REST）。平移写在最后一行（行向量），否则 AddTransformOp 读出的世界平移是
        (0,0,0) → 滴管被 reset 到原点=桌面下不可见。"""
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           DROP_REST[0], DROP_REST[1], DROP_REST[2], 1.0)

    def _dropper_tip_pos(self):
        """滴管尖嘴（局部原点）世界坐标 = 物理夹爪 + 0.13×tool+X 方向（d2s 混合数据源）。

        位置取物理 gripper_pos、方向取 USD tool 矩阵——纯矩阵平移 ExtractTranslation 在
        headless 下 transform 可能滞后 → 试管口/瓶口判近不稳；物理位置 + USD 方向是 d2s
        已验证的写法（d2s _spoon_tip_pos 同款）。
        """
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]   # 行向量约定：tool +X = 旋转部分第 1 行 = 尖嘴方向（新 _T_HELD）
        gripper_pos = self.robot.get_gripper_position()
        return np.asarray(gripper_pos, dtype=float) + TIP_OFFSET * x_dir

    def _ease_dropper_world(self, target, k=0.18):
        """夹爪合拢期间滴管逐帧平滑移向持握位（消除闪现吸附）。target=持握矩阵；
        与 rest 旋转差≈0（附着手腕已回正）→ 平移 lerp 足够，_blend_world 保刚性。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.DROPPER)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_dropper_world(_blend_world(cur, target, k))

    # ------------------------------------------------------------------
    # 温度计矩阵持握（阶段 D：d2s 夹药匙同款横夹竖直杆身，随夹爪旋转/倾斜）
    # ------------------------------------------------------------------
    def _thermo_held_matrix(self):
        """温度计当前持握世界矩阵 = _THERMO_HELD · tool_world（与滴管持握同款）。"""
        return _THERMO_HELD * self._tool_world()

    def _set_thermo_world(self, world_matrix):
        """把温度计写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.THERMOMETER)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _thermo_rest_matrix(self):
        """温度计架右后排孔立放静止位姿（泡尖=原点，THERMO_XY + THERMO_REST_Z）。"""
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           THERMO_XY[0], THERMO_XY[1], THERMO_REST_Z, 1.0)

    def _thermo_hang_grasp(self):
        """挂位夹爪世界坐标（松爪判定参考点）= THERMO_HANG。"""
        return THERMO_HANG

    def _thermo_hang_matrix(self):
        """温度计挂位固定世界矩阵（挂环套铁架台钩、自然下垂；泡尖在管内浸液面）。

        旋转 = diag(-1,-1,1)：与 FWD 持握终点（_THERMO_HELD·tool_world）的世界旋转
        一致（实证：FWD 时温度计 local Z→世界 +Z 泡尖下挂环上、local X→世界 -X 挂环
        孔沿 X 套钩棍），松爪切换零跳变。平移 = 泡尖世界 = 夹爪 THERMO_HANG 正下方
        0.20（FWD tool+X=世界 -Z）：THERMO_HANG - (0,0,0.20)。
        """
        px = THERMO_HANG[0]
        py = THERMO_HANG[1]
        pz = THERMO_HANG[2] - THERMO_GRASP_OFFSET
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           px, py, pz, 1.0)

    def _ease_thermo_world(self, target, k=0.18):
        """夹爪合拢期间温度计逐帧平滑移向持握位（消除闪现吸附）。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.THERMOMETER)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_thermo_world(_blend_world(cur, target, k))

    # ------------------------------------------------------------------
    # 沸石矩阵持握（阶段 C：沸石中心落夹爪处，随夹爪平移/旋转 → 松爪坠落沉底）
    # ------------------------------------------------------------------
    def _zeolite_held_matrix(self):
        """沸石当前持握世界矩阵 = _ZEO_HELD · tool_world（沸石中心落在夹爪处）。"""
        return _ZEO_HELD * self._tool_world()

    def _set_zeolite_world(self, path, world_matrix):
        """把沸石写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _zeolite_translate_matrix(self, center_pos):
        """沸石中心在 center_pos 的恒等旋转位姿（沸石底 = 中心 - 半高 ZEO_CENTER_OFFSET）。"""
        cx, cy, cz = center_pos
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           cx, cy, cz - ZEO_CENTER_OFFSET, 1.0)

    def _zeolite_rest_matrix(self, grasp):
        """沸石皿上静止位姿（中心在 grasp，底贴皿顶）。"""
        return self._zeolite_translate_matrix(grasp)

    def _zeolite_settle_center(self, settle_dz=0.0):
        """沸石沉底后沸石中心世界坐标（管口 xy、z = 管底 + 半高 + settle_dz，穿过液面沉底）。"""
        return np.array([TUBE_XY[0], TUBE_XY[1],
                         self.TUBE_BOTTOM_Z + ZEO_CENTER_OFFSET + settle_dz])

    def _zeolite_settle_matrix(self, settle_dz=0.0):
        """沸石沉底位姿（恒等旋转，中心在管底 + 半高 + settle_dz）。"""
        return self._zeolite_translate_matrix(self._zeolite_settle_center(settle_dz))

    def _ease_zeolite_world(self, path, target, k=0.18):
        """夹爪合拢期间沸石逐帧平滑移向持握位（消除闪现吸附）。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_zeolite_world(path, _blend_world(cur, target, k))

    def _is_closest_zeolite(self, zeolite):
        """这颗沸石能否附着：离夹爪最近的 rest 态沸石，且无其他沸石正被夹/坠落。

        两颗抓点仅 2cm < 近窗 3cm，须双重门禁防同时误抓：
        1) 有别的沸石正 attached/released（正被夹持或坠落入管）→ 本颗绝不附着；
        2) 存在比本颗更近的 rest 沸石 → 本颗不附着。
        （2026-08-27 bug：第一颗 attach 后 state 离开 rest，第二颗失去「更近竞争者」、
        仍在 3cm 近窗内 + opening<closed，同帧连附 → 第一遍就抓走两颗，第二遍夹空。）
        """
        gripper_pos = self.robot.get_gripper_position()
        d_self = np.linalg.norm(np.asarray(gripper_pos) - zeolite.grasp)
        for other in self.zeolites:
            if other is zeolite:
                continue
            if other.state in ("attached", "released"):
                return False
            if other.state == "rest":
                if np.linalg.norm(np.asarray(gripper_pos) - other.grasp) < d_self:
                    return False
        return True


    # ------------------------------------------------------------------
    # 火柴纯平移持握（阶段 E：火柴横躺水平头朝 +X，只跟夹爪平移不随旋转）
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
        """点火检测（仿 flametest _update_ignite）：火柴 attached 期间头近灯芯连续
        MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit=True（idle 门控随后 reveal 火焰）。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    print(f"[b2] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

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
        tip = self._dropper_tip_pos()
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（滴液位尖嘴在管口上 2mm，start=尖嘴下 5mm 入管口内坠落）
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
        # 火焰迁到 /World 顶层（gen rebuild_flames）：灯下引用子 prim 在 RTX 不渲染，
        # 顶层 Cone 才渲染（flametest 已验证）。默认可见，任务 reset 熄、点着翻 visible。
        return ["/World/flame_outer", "/World/flame_inner"]

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
              f"thermometer_attached={self.thermometer.attached} "
              f"thermometer_hung={self.thermometer.hung} "
              f"zeolite_added={self.zeolite_added} "
              f"flame_lit={self.flame_lit} "
              f"boiling_point={self.boiling_point:.1f}°C temp={self.temperature:.1f}°C")
        super().on_task_complete(success)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
