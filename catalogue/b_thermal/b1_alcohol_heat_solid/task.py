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
     （/World/TubePowder，白色粉末柱，gen BUILTIN 预摆 rest 中心 0.809）。
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
  4) 试管（2026-08-27 用户新批拿管到火焰上方，矩阵持握）：每帧试管世界位姿 =
     _T_HELD_TUBE · tool_world（6-DOF 随夹爪旋转，药匙同款；用户逐字「爪子抓的东西
     应该也倾斜了呀」）——爪子转到朝下、试管跟着转水平，不再纯平移吊着竖立。管内
     白粉柱（/World/TubePowder）用同一矩阵平移粉柱相对管底偏移 (0,0,TUBE_POWDER_OFFSET_Z)
     刚性跟随，液体/粉末不再悬在原架里。

驱动 prim（b1_alcohol_heat_solid.usd，scripts/gen_b1_scene.py 生成，2026-08-27 重建含
PowderDrop 14 粒）：
  /World/Spatula / SurfaceDish / SamplePowder / TestTubeRack / TestTube  挖粉同 d2s
  /World/AlcoholLamp/cap  灯帽（纯平移持握）/ flame_outer|flame_inner 火焰（初始隐藏）
  /World/Match  火柴（头朝灯芯，抬高 12mm）
  /World/PowderOnSpoon_<色> / PowderDrop_<色> / TubePowder_<色> / TubePowderBlack
      药匙上粉堆 / 药粉下落 / 管内粉末柱 / 焦黑柱（2026-08-28 粉末颜色+现象输入驱动）
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_SPATULA, SPAT_HEAD_DIST,
    POWDER_TOP_Z, POWDER_X, DISH_XY,
    TUBE_XY, TUBE_MOUTH_Z, GRIP_TUBE, TUBE_REST_Z, TUBE_GRASP_TCP, TUBE_HELD_X,
    TUBE_POWDER_OFFSET_Z,
    CAP_GRASP, CAP_CENTER_REST, CAP_ASIDE_TCP, CAP_ASIDE_CENTER,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    POWDER_COLOR_NAMES, HEAT_PHENOMENON_NAMES,
)

# 药匙相对夹爪持握矩阵（d3s/d2s 同款）：平移 (0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、
# toolZ→(-1,0,0)）。平移在最后一行（USD 行向量约定）。合成 = _T_HELD · tool_world（先作用药匙
# 局部系、再 tool_world 到世界；写反旋转作用到世界系 → 药匙翻走/飞桌面下）。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)

# 试管相对夹爪持握矩阵（2026-08-27 用户逐字「爪子抓的东西应该也倾斜了呀」→ 矩阵持握）：
# 由 ground truth 反解：d2s 药匙 _T_HELD · tool_world(ORIENT_FWD) = 药匙静止（已实证零跳变）
# → tool_world(ORIENT_FWD) 旋转 = [0 0 -1; 0 1 0; 1 0 0]（toolX→(0,0,-1)）。试管抓点处 held
# 必须 = 管底静置（恒等旋转 + 原位）→ _T_HELD_TUBE 旋转 = tool_world(ORIENT_FWD)⁻¹ =
# [0 0 1; 0 1 0; -1 0 0]（toolX→(0,0,1)、toolY→(0,1,0)、toolZ→(-1,0,0)），平移 +TUBE_HELD_X
# 沿 tool-X（管底吊夹爪下 0.1393m）。合成 = _T_HELD_TUBE · tool_world。
# 2026-08-27 修「一夹试管直接翻转过来了」：旧矩阵旋转行 toolY→(0,-1,0)、toolZ→(1,0,0) 相对
# 正确旋转差 180°（约 toolX 翻转），抓点处 held 旋转 = diag(1,-1,-1) → 试管一夹就翻过去；
# 旧平移 −TUBE_HELD_X 把管底抬到 z=1.0846 而非原位 0.806（pxr 数值验证：旧=翻转+1.085 高位，
# 新=恒等旋转+原位 0.806 零跳变）。火焰上方（手指朝下）时试管水平、管轴朝 +X。
_T_HELD_TUBE = Gf.Matrix4d(0.0, 0.0, 1.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           -1.0, 0.0, 0.0, 0.0,
                           TUBE_HELD_X, 0.0, 0.0, 1.0)

# 管内白粉柱相对管底的局部偏移（粉柱中心 = 管底 + TUBE_POWDER_OFFSET_Z，与 gen BUILTIN
# rest 中心 0.809 一致；2026-08-27 用户「粉末只舀了一勺不可能那么多」→ 粉末坐管底 3mm）
_TUBE_POWDER_OFFSET = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, TUBE_POWDER_OFFSET_Z, 1.0)

# 试管内粉末柱 rest 中心（gen_b1_scene BUILTIN 预摆：/World/TubePowder translate z=0.809
# = 管底 0.806 + 3mm；2026-08-27 用户「粉末只舀了一勺不可能那么多」→ 缩小 r0.004 h0.006）
TUBE_POWDER_REST = np.array([TUBE_XY[0], TUBE_XY[1], 0.809])
TUBE_POWDER_H = 0.006


class _CapLifecycle:
    """灯帽状态机（rest → attached → released → rest，纯平移持握）。

    持握 = 纯平移：帽中心 = 夹爪（task._set_cap_center 写帽 local translate =
    center − CAP_CENTER_REST，只动 translate op，姿态恒等不随夹爪旋转）。这与药匙的
    矩阵持握不同——帽是盖在灯口的钟罩，从灯口竖直端起再横移到放一边，全程帽姿态不变。
    对称抓取（2026-08-28 加 CloseCapPass 合盖熄火）：rest 检测「夹爪近灯口 CAP_GRASP
    或近放一边 CAP_ASIDE_TCP」任一都吸附——开盖（OpenCapPass）从灯口端起、合盖
    （CloseCapPass）从放一边端起。吸附期帽中心近灯口即熄火（盖住即隔氧）；松爪按
    夹爪所在位置写回：近灯口 → 盖回灯口（local_t=0）+ 熄灭火焰；近放一边 → 写回
    放一边静止中心（帽底贴台面 0.80，OpenCapPass 原行为）。
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
            # 对称抓取：近灯口（开盖 OpenCapPass）或近放一边（合盖 CloseCapPass）任一吸附
            near_lamp = self.task._near(self.grasp, gripper_pos)
            near_aside = self.task._near(self.aside_tcp, gripper_pos)
            near = near_lamp or near_aside
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
        # 盖回灯口（夹爪近 CAP_GRASP、帽中心回到灯口）即熄灭火焰——物理上盖住即隔氧，
        # B1 无温度模型直接隐藏 flame_outer/flame_inner（flame_lit 作 guard 只灭一次；
        # 开盖吸附后朝远离灯口方向走、吸附前 flame_lit 未置位，都不会误触发）。
        if self.task.flame_lit and self.task._near(self.grasp, gripper_pos):
            self.task._extinguish_flame()
        # 松爪双条件（防下探未到位提前释放帽悬空）：近目标位 + grip 打开。按所在位置写回：
        if opening > self.task.gripper_open_threshold:
            if self.task._near(self.grasp, gripper_pos):
                # 盖回灯口：写回 CAP_CENTER_REST（local_t=0）+ 熄灭火焰
                self.released = True
                self.task._set_cap_center(self.rest_center)
                self.state = "rest"
                self.task._extinguish_flame()
                print("[b1] cap released to lamp -> rest (flame off)")
            elif self.task._near(self.aside_tcp, gripper_pos):
                # 放回台面放一边（OpenCapPass 原行为）
                self.released = True
                self.task._set_cap_center(self.task._cap_aside_center())
                self.state = "rest"
                print("[b1] cap released to aside -> rest")


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


class _TubeLifecycle:
    """试管状态机（rest → attached → released → rest，矩阵持握 6-DOF）。

    持握 = 矩阵持握（药匙同款 _T_HELD）：每帧试管世界位姿 = _T_HELD_TUBE · tool_world，
    试管随夹爪 6-DOF 刚性跟随——爪子转到朝下试管跟着转水平（2026-08-27 用户逐字：
    「爪子抓的东西应该也倾斜了呀」）。管内白粉柱（TubePowder）用同一矩阵平移粉柱相对
    管底偏移 (0,0,TUBE_POWDER_OFFSET_Z) 一起刚性跟随（液体/粉末不再悬在原架里）。task._set_tube_world
    清 op 序写单一 transform op（随矩阵旋转，同 _set_spatula_world）。

    _T_HELD_TUBE 由「抓点处试管=静止」反解（tube_rest · tool_grasp⁻¹），故在抓点
    held = 管底静置矩阵，即原位不动；被夹起后随夹爪转。

    参考点（gripper/TCP 世界坐标）：
      grasp  管口下 14mm 抓点（TUBE_GRASP_TCP = (0.659,0.241,0.9453)，ORIENT_FWD 水平横夹）
      rest   管底静置矩阵（(TUBE_XY, TUBE_REST_Z)，恒等旋转）
      orig   释放写回矩阵（= rest；本批次没有放回动作，保留判据防误释放）
    """

    def __init__(self, task, path, powder_path, rest_matrix, grasp):
        self.task = task
        self.path = path
        self.powder_path = powder_path
        self.rest_matrix = rest_matrix
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
        self.task._set_tube_world(self.rest_matrix)

    def _held_matrix(self):
        """试管世界位姿 = _T_HELD_TUBE · tool_world（6-DOF 随夹爪旋转，同药匙）。"""
        return _T_HELD_TUBE * self.task._tool_world()

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near_grasp(gripper_pos, self.grasp)
            self._near_frames = self._near_frames + 1 if near else 0
            # 夹爪开始合拢且已进近窗：先把试管平滑拉向持握矩阵（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_tube_to_gripper()
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_tube_world(self._held_matrix())
                print(f"[b1] tube attached (grip={opening:.4f})")
            return

        # 吸附期：试管 = _T_HELD_TUBE · tool_world（矩阵跟随，随夹爪转，含白粉柱）
        self.task._set_tube_world(self._held_matrix())
        # 松爪：须同时「夹爪近抓点」+「开爪」才写回原位（本批次没有放回动作，管在火焰上方
        # 悬着时远端开爪不误释放；近抓点开爪 = 后续放回批次接续时复位）
        if (opening > self.task.gripper_open_threshold
                and self.task._near_grasp(gripper_pos, self.grasp)):
            self.released = True
            self.task._set_tube_world(self.rest_matrix)
            self.state = "rest"
            print("[b1] tube released to rack -> rest")


class B1AlcoholHeatSolidTask(BaseTask):
    """B1 酒精灯加热（固体样品）任务：药匙挖粉倒进试管 → 打开灯帽 → 取火柴点燃酒精灯。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 药匙（d2s/d3s 同款）
    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array([0.6993, 0.3608, 0.94])   # d2s 家用（B1 场景逐字）
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量
    # 粉末效果 prim 路径现在按颜色变体动态算（__init__ 里 f"/World/PowderOnSpoon_{色}" 等），
    # 不再作类常量（2026-08-28 用户「加一个输入是表示粉末的颜色参考d2s,d3l」）。
    POWDER_DROPS = 14
    POWDER_STAGGER = 3
    POWDER_HANG = 4
    POWDER_FALL = 14
    POWDER_LAND_Z = 0.809   # 药粉落点 = 管内粉末柱中心（rest 0.809）

    # 加热现象（2026-08-28 用户「根据输入让你有什么现象就有什么现象」）
    HEAT_ZONE_Y = 0.20              # 夹爪 y < 0.20 = 试管进火焰加热区（⑨ 后 y=0.131，预热 y∈[0.111,0.151]）
    HEAT_PHENOMENON_FRAMES = 240    # 连续加热 4s（60fps）触发现象
    PHENOMENON_FADE_FRAMES = 180    # disappear 缩小消失动画 3s

    # 灯帽（纯平移持握）
    CAP = "/World/AlcoholLamp/cap"
    CAP_MESH_OFFSET = np.array([0.0, 0.0, 0.09152])   # 帽 mesh 中心相对帽局部原点（R90+S0.01）

    # 火柴（b2 逐字）/ 火焰
    MATCH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest/b2）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近

    # 试管（加热流第⑤步：水平横夹拿管 → 移到酒精灯火焰上方，矩阵持握随夹爪转）
    TUBE = "/World/TestTube"
    TUBE_GRIP_CLOSED = 0.019              # 夹紧阈值（2026-08-30 修：旧 0.0096+4mm=0.0136 只比指令开度
                                          #   宽 4mm，手指贴合管壁读回略高于阈值即永不吸附；放宽到 0.019
                                          #   手指一开始合拢即吸附，同 B3 修法）
    # 持握 = _T_HELD_TUBE · tool_world（模块常量，反解自「抓点处=静止」）；管内白粉柱
    # TUBE_SAMPLE 随管刚性跟随（同矩阵平移 (0,0,TUBE_POWDER_OFFSET_Z)），不再悬在原架里。

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.spatula_path = self.SPATULA_PATH
        # 药匙/灯帽/火柴是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被
        # 物理干扰），与 d2s/d3s/b2 同模式。
        self._disable_collision(self.spatula_path)
        self._disable_collision(self.CAP)
        self._disable_collision(self.MATCH)
        self._disable_collision(self.TUBE)

        # 阈值（config 可调，d2s/B2 同款默认）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        # 粉末颜色 + 加热现象（2026-08-28 用户「根据输入让你有什么现象就有什么现象，同时加一个
        # 输入是表示粉末的颜色参考d2s,d3l」）：config experiment_result 镜像写回顶层
        # cfg.powder_color / cfg.heat_phenomenon，task 读入并校验兜底。
        self.powder_color = getattr(cfg, "powder_color", "white")
        if self.powder_color not in POWDER_COLOR_NAMES:
            print(f"[b1] WARN unknown powder_color {self.powder_color!r} -> white")
            self.powder_color = "white"
        self.heat_phenomenon = getattr(cfg, "heat_phenomenon", "disappear")
        if self.heat_phenomenon not in HEAT_PHENOMENON_NAMES:
            print(f"[b1] WARN unknown heat_phenomenon {self.heat_phenomenon!r} -> disappear")
            self.heat_phenomenon = "disappear"
        # 粉末效果 prim 路径（颜色变体；gen 烘焙 PowderOnSpoon_<色>/PowderDrop_<色>/TubePowder_<色>）
        self.POWDER_EFFECT = f"/World/PowderOnSpoon_{self.powder_color}"
        self.POWDER_DROP = f"/World/PowderDrop_{self.powder_color}"
        self.TUBE_SAMPLE = f"/World/TubePowder_{self.powder_color}"
        self.TUBE_BLACK = "/World/TubePowderBlack"
        # 加热现象状态（reset 复位，现象只触发一次）
        self._heat_frames = 0
        self._phenomenon_done = False
        self._phenomenon_frames = 0

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

        # 试管生命周期（矩阵持握：试管 = _T_HELD_TUBE · tool_world，随夹爪 6-DOF 转；
        # 管内白粉柱随管刚性跟随）
        self.tube = _TubeLifecycle(self, self.TUBE, self.TUBE_SAMPLE,
                                   _tube_rest_matrix(), TUBE_GRASP_TCP)

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
        self._set_tube_column(self.TUBE_SAMPLE, TUBE_POWDER_H, TUBE_POWDER_REST, r=0.004)
        self._set_visibility(self.TUBE_SAMPLE, False)
        # 焦黑粉柱 + 加热现象状态复位（下一集重新出）
        self._set_tube_column(self.TUBE_BLACK, TUBE_POWDER_H, TUBE_POWDER_REST, r=0.004)
        self._set_visibility(self.TUBE_BLACK, False)
        self._heat_frames = 0
        self._phenomenon_done = False
        self._phenomenon_frames = 0
        # 灯帽复位：回灯口（local_t=0）
        self.cap.reset()
        # 火柴复位：回台面静止位
        self.match.reset()
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._set_visibility(self._flame_paths(), False)
        # 试管复位：回架内竖插静置矩阵（管底 0.806，含白粉柱）
        self.tube.reset()

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
        self.tube.step(gripper_pos, opening)         # ⑤ 试管取放到火焰上方（矩阵持握随爪转）
        self._step_heat_phenomenon(gripper_pos)      # ⑥ 加热现象（输入驱动：disappear/blacken/unchanged）
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "powder_on_spoon": self.powder_on_spoon,
            "poured": self.poured,
            "cap_attached": self.cap.attached,
            "cap_released": self.cap.released,
            "match_attached": self.match.attached,
            "flame_lit": self.flame_lit,
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[b1] episode done success={success} "
              f"spatula={self.spatula_state} poured={self.poured} "
              f"cap_released={self.cap.released} "
              f"match={self.match.state} flame_lit={self.flame_lit} "
              f"tube={self.tube.state} tube_attached={self.tube.attached}")
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

    def _extinguish_flame(self):
        """帽盖回灯口 → 熄灭火焰（B1 无温度模型：直接隐藏 flame_outer/flame_inner）。

        幂等：flame_lit 已 False 则 no-op（_CapLifecycle 吸附期每帧检近灯口，且松爪
        到灯口再调一次保险；同一次合盖只打印一次熄灭）。
        """
        if not self.flame_lit:
            return
        self.flame_lit = False
        self._set_visibility(self._flame_paths(), False)
        print(f"[b1] flame extinguished by cap @ frame {self.frame_idx}")

    def _step_heat_phenomenon(self, gripper_pos):
        """加热现象（2026-08-28 用户「根据输入让你有什么现象就有什么现象」）：
        火焰点燃 + 试管 attached + 夹爪进火焰加热区（y < HEAT_ZONE_Y）连续
        HEAT_PHENOMENON_FRAMES 帧后，按 cfg.heat_phenomenon 出效果：
          disappear = 管内粉末柱逐帧缩小至消失（分解/升华）
          blacken   = 粉末柱切换焦黑变体 TubePowderBlack（碳化）
          unchanged = 不变（无视觉变化）
        现象只触发一次（_phenomenon_done），reset 还原。"""
        if not self._phenomenon_done:
            if not (self.flame_lit and self.tube.attached):
                self._heat_frames = 0
                return
            if gripper_pos[1] >= self.HEAT_ZONE_Y:
                return
            self._heat_frames += 1
            if self._heat_frames < self.HEAT_PHENOMENON_FRAMES:
                return
            self._phenomenon_done = True
            if self.heat_phenomenon == "blacken":
                self._set_visibility(self.TUBE_SAMPLE, False)
                self._set_visibility(self.TUBE_BLACK, True)
                print(f"[b1] heat phenomenon: powder blackened (carbonized) @ frame {self.frame_idx}")
            elif self.heat_phenomenon == "disappear":
                print(f"[b1] heat phenomenon: powder disappearing (decompose) @ frame {self.frame_idx}")
            # unchanged：无视觉变化，仅置位停止累积
            return

        # 已触发：disappear 逐帧缩小（半径/高按剩余比例缩，0 → 隐藏）
        if self.heat_phenomenon != "disappear":
            return
        self._phenomenon_frames += 1
        if self._phenomenon_frames > self.PHENOMENON_FADE_FRAMES:
            return
        frac = self._phenomenon_frames / self.PHENOMENON_FADE_FRAMES
        remain = max(0.0, 1.0 - frac)
        prim = self.stage.GetPrimAtPath(self.TUBE_SAMPLE)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.004 * remain)
            cyl.GetHeightAttr().Set(TUBE_POWDER_H * remain)
        if self._phenomenon_frames >= self.PHENOMENON_FADE_FRAMES:
            self._set_visibility(self.TUBE_SAMPLE, False)

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

    # ------------------------------------------------------------------
    # 试管矩阵持握（药匙同款：清 op 序写单一 transform op，随矩阵旋转）
    # ------------------------------------------------------------------
    def _get_obj_world_matrix(self, path):
        """物体世界 4x4 矩阵；prim 缺失返回 None。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_obj_world_matrix(self, path, world_matrix):
        """把物体写到给定世界矩阵（清 op 序 + AddTransformOp，6-DOF 随矩阵旋转）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _set_tube_world(self, world_matrix):
        """试管 + 管内粉末柱（彩色 + 焦黑变体）一起写世界矩阵（粉柱相对管底偏移
        (0,0,TUBE_POWDER_OFFSET_Z) 刚性跟随，随管转——不再悬在原架里）。焦黑柱平时隐藏，
        位置同步只占一次写 op，开销可忽略。"""
        self._set_obj_world_matrix(self.TUBE, world_matrix)
        self._set_obj_world_matrix(self.TUBE_SAMPLE, world_matrix * _TUBE_POWDER_OFFSET)
        self._set_obj_world_matrix(self.TUBE_BLACK, world_matrix * _TUBE_POWDER_OFFSET)

    def _ease_tube_to_gripper(self, k=0.18):
        """夹爪合拢期间试管（含白粉柱）逐帧平滑拉向持握矩阵（消除闪现吸附）。"""
        cur = self._get_obj_world_matrix(self.TUBE)
        if cur is None:
            return
        self._set_tube_world(_blend_world(cur, _T_HELD_TUBE * self._tool_world(), k))

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


def _tube_rest_matrix():
    """试管架内竖插位姿：恒等旋转 + 平移 (TUBE_XY, TUBE_REST_Z)。"""
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       TUBE_XY[0], TUBE_XY[1], TUBE_REST_Z, 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
