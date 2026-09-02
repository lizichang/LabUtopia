"""C3 燃烧试验（固体样品）任务：药匙横夹持握 + 挖粉效果（本阶段）。

与 flametest 的关键差异（同 d2s）：药匙必须随夹爪**旋转**（竖直提起 → 法兰转 -45° → 挖粉
-45°→-90°），所以不是 set_object_position 平移跟随，而是每帧把药匙世界位姿写为
  药匙世界 = T_held · tool_center 世界矩阵
T_held = 平移(0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）——
药匙相对夹爪沿"局部 X = 手指侧面"伸出 0.112m、长轴与手指垂直。行向量约定：T_held 必须先
作用在夹爪局部系（右乘 tool_world），写反会把旋转作用到世界系 → 药匙翻到桌面下不可见。

药匙持握 = 关碰撞 + transform-op 覆写（药匙是静态碰撞体，逐帧传送会让物理干扰手指闭合；
关掉后与 flametest 铂丝同模式，手指按 grip_target=0.008 闭合、视觉贴杆）。

本阶段只覆盖「横夹药匙 + 挖起药粉」：药匙 rest→attached（近抓点+合拢）→ 逐帧跟随夹爪
→ ⑨ 法兰 -45°→-90° 旋转触发挖粉（powder_on_spoon 显示、逐帧跟随勺尖）→ 松开回架。
倒燃烧匙、夹燃烧匙、点火、燃烧现象、盖帽熄火已全部实现（本阶段完成全流程）。

场景 prim（assets/scenes/c_flame/c3_combustion_solid/c3_combustion_solid.usd，
scripts/gen_c3_scene.py 生成，2026-09-01 用户定稿，与 d2s 逐字对齐）：
  试管架 test_tube_rack.usd (0.6803,0.3607)；药匙 spatula.usd 立架中心孔
  (0.6993,0.3608,0.828) rotZ -180°；表面皿 sample_dish.usd (0.5365,0.105,0.80)
  内预置粉末 powder.usd（粉丘 bbox x[0.5188,0.5542] y[0.0814,0.1288]
  z[0.8021,0.8141]）；燃烧匙 combustion_spoon.usd (0.596,0.250,0.8068) 竖立靠架旁；
  酒精灯 (0.35,0.05)；火柴 (0.52,0.05,0.813)。效果 prim PowderOnSpoon 在药匙尖端
  (0.6993,0.3608,0.965)。
"""
import math
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_SPATULA, SPAT_GRASP, SPAT_HEAD_DIST,
    POWDER_TOP_Z, POWDER_X, DISH_XY,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    SPOON_GRASP, SPOON_HELD_OFFSET, FLAME_HOLD_TCP,
    CAP_GRASP, CAP_BURNER, CAP_CLOSED_THRESHOLD,
    CAP_COVER_NEAR, CAP_EXTINGUISH_XY, CAP_EXTINGUISH_Z,
    CAP_HELD_OFFSET, CAP_CENTER_DZ,
    SPOON_FLAME_NEAR, IGNITION_DELAY, BURN_FRAMES, BURN_OUT_AT,
    POWDER_FULL_H, POWDER_FULL_R, POWDER_RESIDUE_H, POWDER_RESIDUE_R,
    POWDER_WHITE, POWDER_ASH, POWDER_CARBON, CARBONIZE_DELAY,
    EFFECT_LAMP_FLAME_GRPS, EFFECT_SAMPLE_FLAME,
    EFFECT_SAMPLE_FLAME_CONE, EFFECT_SAMPLE_FLAME_SPHERE,
)

# 药匙相对夹爪：平移 (0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）。
# 药匙长轴 = 夹爪局部 X（手指侧面）、与手指垂直 → 手指水平时药匙竖直挂下、与架内姿态
# （rotZ -180°）零跳变。平移必须在最后一行（USD 行向量约定）。与 d2s 逐字一致。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段 ③点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （夹爪手指朝下竖直夹其杆身）。释放时写回台面静止位（flametest 同款：高位松爪后
    火柴写回 rest）。
    参考点（gripper/TCP 世界坐标）：
      grasp   杆身中部抓点（MATCH_GRASP=(0.56,0.05,0.8145)）
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
                print(f"[c3] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[c3] match released to rest")


class _SpoonLifecycle:
    """燃烧匙状态机（rest → attached → rest，阶段 ④ 横夹把手入外焰 → 放回原位）。

    纯平移持握（同火柴）：勺原点（碗口平面）= 夹爪 + SPOON_HELD_OFFSET，姿态不变。
    碗内样品（/World/PowderInBowl）随碗平移（勺原点下 5mm = content_offset，写 translate
    跟随；倒粉已落定显示，吸附期/入焰/放回都跟着碗走）。
    2026-09-01 用户「拿起燃烧匙移动到外焰上（仿照C4）」：碗入外焰停留后夹爪回把手抓点
    松爪 → 勺 + 碗内粉写回台面静止位并复位 rest。
    """

    def __init__(self, task, name, path, rest, grasp, content_path, content_offset):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(rest)
        self.grasp = np.array(grasp)
        self.content_path = content_path
        self.content_offset = np.array(content_offset)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.task._set_obj_world(self.path, self.orig)
        self.task._set_obj_world(self.content_path, self.orig + self.content_offset)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos) + np.array(SPOON_HELD_OFFSET)
            # 夹爪开始合拢且已进近窗：先把勺平滑拉向持握位（消闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                self.task._set_obj_world(self.content_path, held + self.content_offset)
                print(f"[c3] spoon attached (grip={opening:.4f})")
            return
        # 吸附期：勺 + 碗内样品跟随夹爪（纯平移保姿态）
        spoon_pos = np.asarray(gripper_pos) + np.array(SPOON_HELD_OFFSET)
        self.task._set_obj_world(self.path, spoon_pos)
        self.task._set_obj_world(self.content_path, spoon_pos + self.content_offset)
        # 放回：夹爪回把手抓点（勺原点=静止位）开爪 → 勺+碗内粉写回台面静止位、复位 rest
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.grasp, gripper_pos)):
            self.state = "rest"
            self.attached = False
            self.task._set_obj_world(self.path, self.orig)
            self.task._set_obj_world(self.content_path, self.orig + self.content_offset)
            print(f"[c3] spoon released to rest")


class _CapLifecycle:
    """灯帽状态机（rest → attached → settled，阶段 ⑤ 燃烧放回后盖灯帽灭火）。

    持握 = 纯平移 offset（CAP_HELD_OFFSET）：帽全程竖直开口朝下，不随夹爪旋转（同火柴
    纯平移持握）。帽是灯的子 prim，吸附期逐帧把帽写到夹爪持握位（帽中心 = 夹爪 +
    CAP_HELD_OFFSET，经 _set_cap_world 换算成帽相对灯的 local translate）。
    盖到位（夹爪近 CAP_BURNER 连续帧）→ settled：火焰熄灭、帽锁灯口。熄火时机照 B2
    十一改：帽下降罩过火焰顶（CAP_EXTINGUISH_Z 门控）才灭，不移动时早灭。
    参考点（gripper/TCP 世界坐标）：
      grasp   帽静止位夹点（CAP_GRASP=CAP_REST 同水平，帽顶下 7mm）
      cover   盖灯口夹爪（CAP_BURNER=0.900，帽中心 0.8915 盖严实，同资产原始帽位）
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
                print(f"[c3] cap attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：帽跟随夹爪（纯平移），帽中心 = 夹爪 + CAP_HELD_OFFSET
            held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
            self.task._set_cap_world(held)
            # 下落即熄火（B2 十一改）：帽底（夹爪−0.024）刚罩过火焰顶 0.936 才灭；
            # xy 门控防 ⑤ 提帽/⑥ 运帽时误触
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
                    print(f"[c3] cap settled, flame extinguished")
            else:
                self.extinguish_counter = 0
            return

        # settled：帽锁灯口（不再跟随夹爪，帽已停在盖灭位），火焰已熄（_on_cap_settled 处理）


class C3CombustionSolidTask(BaseTask):
    """C3 燃烧试验任务：药匙横夹 → 挖粉 → 倒燃烧匙 → 放回药匙 → 火柴点火 → 燃烧匙入外焰 → 盖帽熄火。"""

    # 样品燃烧火焰焰色 → emissive（照 C1 FLAME_COLORS，饱和色配方防渲染洗白）
    FLAME_COLORS = {
        "orange": (1.00, 0.60, 0.15),
        "yellow": (1.00, 0.72, 0.12),
        "red": (1.00, 0.35, 0.25),
        "blue": (0.30, 0.60, 1.00),
        "green": (0.35, 0.95, 0.40),
        "purple": (0.80, 0.45, 1.00),
    }

    TABLE_Z = 0.80

    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array(SPAT_GRASP)
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量（同 d2s）
    GRIP_OPEN_THRESH = 0.03                    # 松开阈值（与 flametest 一致）

    # 效果 prim（初始 invisible，task 挖粉后逐帧跟随勺尖）
    POWDER_EFFECT = "/World/PowderOnSpoon"

    # 燃烧匙（combustion_spoon.usd，用户定稿位置，靠在试管架旁）——倒粉目标 + 入外焰。
    SPOON_PATH = "/World/CombustionSpoon"
    SPOON_REST = np.array([0.596, 0.250, 0.8068])

    # 火柴 + 点火（阶段 ③ 点燃酒精灯）：火柴 rest/attached/released 生命周期 + 火焰 reveal
    MATCH_PATH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest/C4）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近
    # 酒精灯：臂在灯口上方作业（③ 火柴头近灯芯点火、④ 燃烧匙碗停外焰）——同 b2/b3/d9
    # 惯例，工作于灯区的任务须关灯碰撞，否则臂/持握物贴近灯口时物理接触会污染关节状态
    # （PhysX "Invalid PhysX transform" 连刷）。
    LAMP_PATH = "/World/AlcoholLamp"
    CAP = "/World/AlcoholLamp/cap"   # 灯帽（灯子 prim，阶段 ⑤ 盖帽熄火；碰撞随 LAMP_PATH 递归关）

    # 倒粉下落动画（仿 d2s PowderDrop）：⑭ 进行到一半（法兰旋转中 + TCP z≈0.98 高位 +
    # y 过中点）触发，粉粒从勺尖错帧坠入燃烧匙碗内落定（对角流），落定后勺上粉消失、
    # 碗内粉显示。
    POWDER_DROP = "/World/PowderDrop"
    POWDER_DROPS = 14
    POWDER_STAGGER = 3
    POWDER_HANG = 4
    POWDER_FALL = 14
    POWDER_LAND_Z = 0.802            # 落定 z：碗内样品位（碗口 0.8068 下 5mm，碗底≈0.797）
    POWDER_BOWL = "/World/PowderInBowl"
    SPOON_BOWL_XY = np.array([0.596, 0.250])   # 燃烧匙碗心（落点 xy，碗口 z=0.8068）
    POUR_TRIGGER_Y = 0.42            # ⑭ 触发阈值：TCP y<0.42（起点 0.5108→终点 0.3308 中点 0.4208）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.spatula_path = self.SPATULA_PATH
        # 药匙是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理
        # 干扰），与 flametest 铂丝/滴管同模式。
        self._disable_collision(self.spatula_path)

        self.GRASP_NEAR_FRAMES = 3
        self._near_frames = 0
        self.spatula_state = "rest"     # rest / attached / released
        self.powder_on_spoon = False
        self.powder_falling = False     # 倒粉下落动画进行中（⑭ 进行到一半触发）
        self._powder_queue = []         # 下落动画队列（delay/t/hang/fall/start/target）
        self._prev_flange = None        # 上一帧法兰角（joint7，索引 6），用于判定⑨挖粉旋转开始

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        # 火柴生命周期句柄 + 点火状态（阶段 ③ 点燃酒精灯）：火柴静态碰撞体吸附期关碰撞
        self._disable_collision(self.MATCH_PATH)
        # 酒精灯碰撞体：臂在灯口上方作业（③ 点火下探、④ 燃烧匙碗停外焰），关碰撞防
        # 物理接触污染关节状态（PhysX "Invalid PhysX transform" 连刷，同 b2/b3/d9）。
        self._disable_collision(self.LAMP_PATH)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH_PATH, match_rest, MATCH_GRASP)
        self.flame_lit = False
        self.match_ignite_counter = 0
        self.flame_prims = self._flame_paths()

        # 燃烧匙生命周期句柄（阶段 ④ 入外焰）：勺 + 碗内样品随勺平移（吸附期关碰撞）
        self._disable_collision(self.SPOON_PATH)
        self.spoon = _SpoonLifecycle(
            self, "spoon", self.SPOON_PATH, self.SPOON_REST, SPOON_GRASP,
            self.POWDER_BOWL, np.array([0.0, 0.0, -0.005]))

        # 灯帽生命周期句柄（阶段 ⑤ 盖帽熄火）：帽 = 灯子 prim，吸附期写帽 local translate
        # 跟随夹爪（纯平移持握）；帽碰撞已随 LAMP_PATH 递归关（帽是其子 prim，一并关）。
        self.cap_closed_threshold = CAP_CLOSED_THRESHOLD      # 帽 attach 阈值（帽 Ø37mm）
        self.cap_cover_near = CAP_COVER_NEAR                  # 夹爪距 CAP_BURNER 盖到位近窗
        self.cap_dwell_frames = int(getattr(cfg, "cap_dwell_frames", 15))  # 盖到位连续帧
        self.cap = _CapLifecycle(self, "cap", self.CAP, CAP_GRASP, CAP_BURNER)
        self.cap_rest_translate = self._read_cap_translate()  # 帽静止位 local translate
        self.flame_extinguished = False

        # 燃烧现象（阶段 ④ 碗入外焰 dwell 期间）：config combustion/flame_color 驱动
        # （用户 09-02「火焰颜色通过输入决定」+「燃烧后留残渣/几乎空」+「不可燃轻微碳化」）
        self.combustion = getattr(cfg, "combustion", "combustible")
        self.flame_color = getattr(cfg, "flame_color", "orange")
        self._burn_phase = "idle"       # idle/heating/ignited/burned_out(可燃) | carbonized(不可燃)
        self._burn_frames = 0
        self._burn_height = POWDER_FULL_H
        self._set_sample_flame_emissive()
        self._set_sample_flame_visible(False)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.spatula_state = "rest"
        self._near_frames = 0
        self.powder_on_spoon = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None
        self._set_spatula_world(_rest_matrix())
        self._set_visibility(self.POWDER_EFFECT, False)
        self._set_visibility(self.POWDER_BOWL, False)
        self._set_visibility(self.POWDER_DROP, False)
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        # 还原粉堆尺寸：上一集挖粉若被缩小（倒粉）需还原。
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005)
            cyl.GetHeightAttr().Set(0.005)
        # 火柴复位 + 火焰隐藏（阶段 ③ 点火前，同 C4 reset）
        self.match.reset()
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._set_flame_visible(False)
        # 燃烧匙复位（勺回台面、碗内粉回碗位；阶段 ④）
        self.spoon.reset()
        # 灯帽复位（帽回静止位；阶段 ⑤）+ 熄火状态复位
        self.cap.reset()
        self.flame_extinguished = False
        # 燃烧现象复位：粉末还原白/满高、样品火焰隐藏、焰色按输入重写
        self._burn_phase = "idle"
        self._burn_frames = 0
        self._burn_height = POWDER_FULL_H
        self._set_burn_powder(POWDER_FULL_H, POWDER_FULL_R, POWDER_WHITE)
        self._set_sample_flame_visible(False)
        self._set_sample_flame_emissive()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_spatula()
        self._step_powder_anim()
        self.match.step(gripper_pos, opening)
        self.spoon.step(gripper_pos, opening)
        self.cap.step(gripper_pos, opening)     # 盖帽阶段：帽跟随/下降熄火/盖到位锁灯口
        self._step_match_ignite(gripper_pos)   # 点火检测（火柴头触灯芯 → flame_lit → 火焰 reveal）
        self._step_burn_phenomenon(gripper_pos)  # 燃烧现象（碗入外焰点燃/碳化 + 样品火焰 flicker）
        self._step_flame_anim()                # 酒精灯火焰 flicker（点着后到熄火前）
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "powder_on_spoon": self.powder_on_spoon,
            "match_attached": self.match.attached,
            "match_released": self.match.released,
            "spoon_attached": self.spoon.attached,
            "flame_lit": self.flame_lit,
            "cap_attached": self.cap.attached,
            "cap_settled": self.cap.settled,
            "flame_extinguished": self.flame_extinguished,
            "combustion": self.combustion,
            "flame_color": self.flame_color,
        })

    def on_task_complete(self, success):
        print(f"[c3] episode done success={success} "
              f"spatula={self.spatula_state} powder_on_spoon={self.powder_on_spoon}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧药匙持握 / 挖粉效果
    # ------------------------------------------------------------------
    def _update_spatula(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        # 法兰（joint7，索引 6）是否在旋转：⑨ 挖粉起判定信号。⑥⑦⑧ 保持世界朝向（法兰恒定），
        # ⑤ 法兰旋转但勺尖在架位高位，⑨ 法兰旋转且勺尖在粉丘 → 仅⑨首帧满足全部条件。
        flange_rotating = (self._prev_flange is not None
                           and abs(joints[6] - self._prev_flange) > 0.005)
        self._prev_flange = float(joints[6])

        if self.spatula_state == "rest":
            if self._near_grasp(gripper_pos, self.SPAT_GRASP):
                self._near_frames += 1
            else:
                self._near_frames = 0
            # 夹爪开始合拢且够近：药匙平滑拉向夹爪持握位（消除闪现吸附）
            if self._near_grasp(gripper_pos, self.SPAT_GRASP) and opening < self.gripper_open_threshold:
                self._ease_spatula_to_gripper(gripper_pos)
            if (self._near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.SPAT_GRIP_CLOSED):
                self.spatula_state = "attached"
                self._set_spatula_from_gripper()
                print(f"[c3] spatula attached (grip={opening:.4f})")

        elif self.spatula_state == "attached":
            self._set_spatula_from_gripper()
            tip = self._spoon_tip_pos(gripper_pos)
            # ⑨ 法兰开始旋转（挖粉）→ 显示粉末，逐帧跟随勺尖（本阶段不倒入，粉随勺走）
            if not self.powder_on_spoon and self._scoop_starting(tip, flange_rotating):
                self.powder_on_spoon = True
                self._set_visibility(self.POWDER_EFFECT, True)
                print(f"[c3] powder on spoon (tip={np.round(tip, 3)})")
            if self.powder_on_spoon:
                self.object_utils.set_object_position(
                    self.POWDER_EFFECT, tip + np.array([0.0, 0.0, 0.003]))
            # ⑭ 进行到一半（法兰旋转中 + TCP z≈0.98 高位 + y 过中点 0.42）→ 触发粉末下落。
            # flange_rotating 排除 ⑥⑦⑧⑪⑫⑬ 平移段；⑤ 高位 z=1.15、⑨ 粉丘 z=0.905 都不在
            # 此 z 带——⑭ 从 z=0.98 起水平往 -y，y 过中点即半程。
            if (not self.powder_falling
                    and self.powder_on_spoon
                    and flange_rotating
                    and 0.95 < gripper_pos[2] < 1.01
                    and gripper_pos[1] < self.POUR_TRIGGER_Y):
                self.powder_falling = True
                self._start_powder_fall(self._spoon_tip_pos(gripper_pos))
                print(f"[c3] powder fall started (gripper y={gripper_pos[1]:.3f})")
            # 松开：回到架内竖插位姿
            if opening > self.gripper_open_threshold:
                self.spatula_state = "released"
                self._set_spatula_world(_rest_matrix())
                self._set_visibility(self.POWDER_EFFECT, False)
                print("[c3] spatula released to rack")

    # ------------------------------------------------------------------
    # 药匙位姿
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_spatula_from_gripper(self):
        # 行向量约定：先 _T_HELD（药匙局部→夹爪局部）再 tool_world（局部→世界）。
        # 写反成 tool_world * _T_HELD 会把 R_y(π) 作用到世界系，药匙原点算到
        # (-0.74,0.42,-0.83)（桌面下）→ 夹住瞬间"消失"（d2s 已 pxr 数值验证）。
        self._set_spatula_world(_T_HELD * self._tool_world())

    def _set_spatula_world(self, world_matrix):
        """把药匙写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
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
        """夹爪合拢期间药匙逐帧平滑移向持握位（消除闪现吸附）。"""
        target = _T_HELD * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.spatula_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_spatula_world(_blend_world(cur, target, k))

    def _spoon_tip_pos(self, gripper_pos):
        """勺尖 = 夹爪 + 0.134 × 夹爪局部 +X（勺头方向）世界方向。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]   # 行向量约定：tool +X = 旋转部分第 1 行 = 勺头方向
        return np.asarray(gripper_pos, dtype=float) + SPAT_HEAD_DIST * x_dir

    # ------------------------------------------------------------------
    # 通用物体位姿（火柴/燃烧匙/碗内粉用，纯平移持握；C4 同款）
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        """物体原点世界坐标；prim 缺失返回 None。"""
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
    # 火柴位姿 + 点火检测（阶段 ③ 点燃酒精灯，仿 C4）
    # ------------------------------------------------------------------
    def _match_rest_pos(self):
        """火柴原点台面静止位（MATCH_XY + MATCH_REST_Z）。"""
        return np.array([MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z])

    def _set_match_world(self, position):
        """把火柴写到给定世界位置（纯平移，火柴水平头朝 +X 姿态不变）。"""
        self._set_obj_world(self.MATCH_PATH, position)

    def _ease_match_world(self, target, k=0.18):
        """夹爪合拢期间火柴逐帧平滑移向持握位（消除闪现吸附）。"""
        self._ease_obj_world(self.MATCH_PATH, target, k)

    def _match_tip(self, gripper_pos):
        """火柴头中心世界坐标 = 夹爪 + MATCH_TIP_OFFSET（头在夹爪 +X 0.0494，水平朝前）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)

    def _step_match_ignite(self, gripper_pos):
        """点火检测（仿 flametest/C4）：火柴 attached 期间头近灯芯连续
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
                    print(f"[c3] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    def _flame_paths(self):
        # C3 火焰迁到 /World 顶层 + grp 包装（gen rebuild_flames 水滴形 = 底半球 Sphere +
        # 上部 Cone，每焰两 prim = <名>_sphere + <名>，grp pivot=火焰底供 flicker）；gen 初始
        # 隐藏，task 点着翻 visible。此处返回 grp 内子 prim（_set_flame_visible 翻 visible）。
        return ["/World/flame_outer_grp/flame_outer",
                "/World/flame_outer_grp/flame_outer_sphere",
                "/World/flame_inner_grp/flame_inner",
                "/World/flame_inner_grp/flame_inner_sphere"]

    def _set_flame_visible(self, visible):
        """点火 reveal / 熄火隐藏全部火焰 prim（外/内焰各 球+锥 共 4 个）。"""
        for p in self.flame_prims:
            self._set_visibility(p, visible)

    # ------------------------------------------------------------------
    # 火焰 flicker + 固体燃烧现象（阶段 ④ dwell，config combustion 双现象）
    # ------------------------------------------------------------------
    def _smooth_noise(self, t, seed):
        """确定性平滑噪声（每 3 帧一个随机值，相邻 smoothstep 插值）。火焰 flicker 用，
        同输入同输出（无随机抖动，录像可复现）。照 C4（span=3 防 20Hz 爆闪）。"""
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
        （gen 组 op 序 translate→rotate→scale：先 scale 后 rotate 再 translate = 绕底
        不漂移）。base_pos 给则同时写 translate（样品火焰跟随粉末顶），None 则不动 translate。"""
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
        """酒精灯火焰每帧 flicker（点火后到熄火前，仿 C4「火焰要动起来不然太假了」）。"""
        if not (self.flame_lit and not self.flame_extinguished):
            return
        self._apply_flame_flicker(EFFECT_LAMP_FLAME_GRPS[0], None, seed=1)
        self._apply_flame_flicker(EFFECT_LAMP_FLAME_GRPS[1], None, seed=2)

    def _set_sample_flame_visible(self, visible):
        """样品燃烧火焰 reveal/隐藏（锥+球两 prim，grp 内）。"""
        for p in (EFFECT_SAMPLE_FLAME_CONE, EFFECT_SAMPLE_FLAME_SPHERE):
            self._set_visibility(p, visible)

    def _set_sample_flame_emissive(self):
        """按 flame_color 输入写样品火焰 emissive（锥+球两 prim）。"""
        emissive = self.FLAME_COLORS.get(self.flame_color, self.FLAME_COLORS["orange"])
        for p in (EFFECT_SAMPLE_FLAME_CONE, EFFECT_SAMPLE_FLAME_SPHERE):
            sh = self.stage.GetPrimAtPath(f"{p}_mat/Shader")
            if sh.IsValid() and sh.GetTypeName() == "Shader":
                UsdShade.Shader(sh).GetInput("emissiveColor").Set(Gf.Vec3f(*emissive))

    def _set_burn_powder(self, h, r, color):
        """燃烧/碳化：更新碗内粉末高/半径 + diffuse 颜色（白→黑残渣/深灰碳化）。"""
        prim = self.stage.GetPrimAtPath(self.POWDER_BOWL)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetHeightAttr().Set(max(0.0, h))
            cyl.GetRadiusAttr().Set(max(0.0, r))
        sh = self.stage.GetPrimAtPath(f"{self.POWDER_BOWL}_mat/Shader")
        if sh.IsValid() and sh.GetTypeName() == "Shader":
            UsdShade.Shader(sh).GetInput("diffuseColor").Set(Gf.Vec3f(*color))

    def _set_sample_flame_pos(self):
        """样品火焰写 translate 到当前粉末顶 + flicker（可燃点燃期间每帧调用）。"""
        spoon_pos = np.asarray(self.robot.get_gripper_position()) + np.array(SPOON_HELD_OFFSET)
        top_z = spoon_pos[2] + self.spoon.content_offset[2] + self._burn_height / 2.0
        self._apply_flame_flicker(EFFECT_SAMPLE_FLAME,
                                  base_pos=(spoon_pos[0], spoon_pos[1], top_z), seed=3)

    def _step_burn_phenomenon(self, gripper_pos):
        """碗内固体燃烧现象（阶段 ④ dwell 期间）：combustible=粉末点燃火焰（焰色=输入）烧尽
        留黑色残渣；non_combustible=轻微碳化变黑（无火焰）。触发=勺 attached 且夹爪近
        FLAME_HOLD_TCP（碗在外焰中停留）。"""
        in_flame = (self.spoon.attached and np.linalg.norm(
            np.asarray(gripper_pos) - np.array(FLAME_HOLD_TCP)) < SPOON_FLAME_NEAR)
        if not in_flame:
            if self._burn_phase in ("heating", "ignited"):
                # 离开火焰：隐藏样品火焰（粉末保持当前燃烧态，不回滚）
                self._set_sample_flame_visible(False)
            self._burn_phase = "idle"
            self._burn_frames = 0
            return
        if self._burn_phase == "idle":
            self._burn_phase = "heating"
            self._burn_frames = 0
        if self.combustion == "combustible":
            self._step_burn_combustible()
        else:
            self._step_burn_non_combustible()

    def _step_burn_combustible(self):
        """可燃：heating（受热升温）→ ignited（粉末顶点燃火焰 + flicker）→ 粉末缩小变黑 →
        烧尽留残渣熄火。"""
        self._burn_frames += 1
        if self._burn_phase == "heating":
            if self._burn_frames >= IGNITION_DELAY:
                self._burn_phase = "ignited"
                self._set_sample_flame_visible(True)
                print(f"[c3] powder ignited ({self.flame_color}) @ frame {self.frame_idx}")
            return
        if self._burn_phase == "ignited":
            self._set_sample_flame_pos()   # 火焰跟随粉末顶 + flicker
            frac = min(1.0, (self._burn_frames - IGNITION_DELAY) / float(BURN_FRAMES))
            self._burn_height = POWDER_FULL_H + (POWDER_RESIDUE_H - POWDER_FULL_H) * frac
            r = POWDER_FULL_R + (POWDER_RESIDUE_R - POWDER_FULL_R) * frac
            self._set_burn_powder(self._burn_height, r,
                                  _lerp_color(POWDER_WHITE, POWDER_ASH, frac))
            if self._burn_height <= BURN_OUT_AT:
                self._burn_phase = "burned_out"
                self._set_sample_flame_visible(False)
                self._set_burn_powder(POWDER_RESIDUE_H, POWDER_RESIDUE_R, POWDER_ASH)
                print(f"[c3] powder burned out (residue) @ frame {self.frame_idx}")
            return
        # burned_out：黑色残渣在火焰中，无效果

    def _step_burn_non_combustible(self):
        """不可燃：heating（受热升温）→ carbonized（粉末轻微变黑碳化，无火焰、体积基本不变）。"""
        self._burn_frames += 1
        if self._burn_phase == "heating":
            if self._burn_frames >= CARBONIZE_DELAY:
                self._burn_phase = "carbonized"
                self._burn_height = POWDER_FULL_H * 0.85
                r = POWDER_FULL_R * 0.92
                self._set_burn_powder(self._burn_height, r, POWDER_CARBON)
                print(f"[c3] powder carbonized (non_combustible) @ frame {self.frame_idx}")
            return
        # carbonized：保持碳化态（无火焰，轻微变黑不再变）

    # ------------------------------------------------------------------
    # 灯帽位姿 + 熄火（阶段 ⑤ 盖帽灭火，照 C4/B2）
    # ------------------------------------------------------------------
    def _read_cap_translate(self):
        """读帽当前 local translate（场景静止位，帽=灯子 prim 的 translate op）。"""
        prim = self.stage.GetPrimAtPath(self.CAP)
        if not prim.IsValid():
            return None
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return list(op.Get())
        return None

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
        """把帽写到给定帽中心世界坐标（帽 = 灯子 prim，只写帽 translate op，保留
        rotateXYZ+scale 形状）。换算（pxr 实测，灯 R180Z，照 B2/C4）：
        cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+CAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−CAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world(self.LAMP_PATH)
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
        self._set_flame_visible(False)
        print(f"[c3] flame extinguished @ frame {self.frame_idx}")

    def _on_cap_settled(self, center):
        """帽盖严实：熄灭火焰 + 帽锁在盖灭位（写回持握位，不再跟随夹爪）。"""
        self._extinguish_flame()
        self._set_cap_world(center)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    # ------------------------------------------------------------------
    # 倒粉下落动画（仿 d2s PowderDrop）
    # ------------------------------------------------------------------
    def _start_powder_fall(self, tip):
        """⑭ 进行到一半：勺尖生成一串粉粒（PowderDrop 父 Drop_0..N 球），delay 错帧起落
        成连续细粉流，从勺尖对角坠入燃烧匙碗内落定（落点 = 碗心，仿 d2s、落点换 C3 碗）。"""
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        start = tip + np.array([0.0, 0.0, -0.004])
        for i in range(self.POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * self.POWDER_STAGGER,
                "t": 0,
                "start": start.copy(),
                "target": np.array([self.SPOON_BOWL_XY[0], self.SPOON_BOWL_XY[1], self.POWDER_LAND_Z]),
                "hang": self.POWDER_HANG, "fall": self.POWDER_FALL,
            })
        self._set_visibility(self.POWDER_DROP, True)
        print(f"[c3] powder fall start @ tip {np.round(start, 3)} -> bowl {self.SPOON_BOWL_XY}")

    def _step_powder_anim(self):
        """推进下落串：每粒 delay 错帧起落，悬停→加速坠落→落定（隐藏该粒 + 勺上粉堆缩小），
        全部落定 → 勺上粉消失、碗内粉显示（powder poured into bowl）。"""
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
                # 勺上粉全倒出：powder_on_spoon 归 False（防 ReturnSpatula 高位段 y 过
                # POUR_TRIGGER_Y 再触发倒粉/点火判定）；碗内样品显示、随勺走（_SpoonLifecycle）
                self.powder_on_spoon = False
                self._set_visibility(self.POWDER_EFFECT, False)
                self._set_visibility(self.POWDER_BOWL, True)
                print("[c3] powder poured into combustion spoon bowl")

    def _shrink_powder_blob(self, landed_frac):
        """勺上粉堆随下落进度缩小（粉粒落定越多、勺上剩得越少，仿 d2s）。"""
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005 * remain)
            cyl.GetHeightAttr().Set(0.005 * remain)

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _scoop_starting(self, tip, flange_rotating):
        """⑨ 法兰 -45°→-90° 开始旋转（挖粉起）判定：法兰正在旋转 且 勺尖在粉丘附近（松带）。

        排除误触发：⑤ 法兰也旋转但勺尖在架位高位（x=0.6993 不在粉堆 x 带、z≈1.03 高于高度带）；
        ⑥⑦⑧ 法兰保持朝向恒定不旋转。只在 ⑨ 旋转首帧触发（勺尖在粉丘带内）→ 粉末随旋转从
        粉丘带起（d2s 同款判定，仅坐标换 C3）。"""
        near = (abs(tip[0] - POWDER_X) < 0.04
                and abs(tip[1] - DISH_XY[1]) < 0.08
                and tip[2] < POWDER_TOP_Z + 0.02)
        return flange_rotating and near

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


def _rest_matrix():
    """药匙架内竖插位姿（与场景 /World/Spatula 世界矩阵一致）：translate (0.6993,0.3608,0.828)，
    rotateXYZ(0,0,-180) 烘平后即下行序。Gf.Matrix4d 行主序、平移在最后一行（row-vector），
    写错到第 4 列会把药匙 reset 到世界原点（桌面下不可见）。"""
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


def _lerp_color(a, b, t):
    """两个 RGB 颜色线性插值（燃烧粉末白→黑残渣/深灰碳化）。"""
    return tuple(float(a[i]) + (float(b[i]) - float(a[i])) * t for i in range(3))
