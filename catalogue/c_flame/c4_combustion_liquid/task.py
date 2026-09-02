"""C4 燃烧试验（液体样品）任务：滴管「吸药品瓶液 → 滴入燃烧匙碗」生命周期 + 液体效果。

持握照 flametest/d3l（v24-v46 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——滴管
全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），效果 prim（DropperFill）
只需 position 跟随尖嘴即可。

生命周期（gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；药品瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（液柱被吸进尖嘴）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；燃烧匙碗口区挤胶头（<GRIP_SQUEEZED）
             → dropped → DropperFill 隐藏 + 液滴串坠落 + 碗液逐滴升高
  - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（一次
             持握内循环吸液-滴液，不松开）；末遍滴完回架松开（>gripper_open）
             → released（写回架内竖插位姿）并复位 rest

液体颜色：cfg.liquid_color enum [clear,red,blue,green,purple]。gen 预烘焙候选色
瓶液 SampleLiquid_<色> 与碗液 SpoonLiquid_<色> 变体，task 按 liquid_color 显一根
（headless 下运行时改材质不渲染，故烘焙多组、运行时只 show 一组）：
  clear → 瓶液 SampleLiquid / 碗液 SpoonLiquid；其余 → 对应 <色> 变体。
滴入燃烧匙碗的液体与瓶液同色，首滴落定后碗液 SpoonLiquid（<色>变体）显示并逐滴长高。

controller 顺序执行 ①DripSpoonPass（抓滴管**一次持握内循环吸液-滴液 cfg.drip_cycles
遍**滴入燃烧匙碗）→ ②LightFlamePass（取火柴→触灯芯点燃酒精灯→放回火柴）→
③SpoonToFlamePass（水平横夹燃烧匙杆身→碗口入外焰燃烧 4s→+y 5cm 观察 10s→提出放回原位）→
⑤CapLampPass（取灯帽→下扣盖灭火焰）。点火检测 = 火柴头近灯芯连续
MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit → 火焰 reveal（照 B2）；盖帽熄灭照 B2
_CapLifecycle（帽下降罩过火焰顶熄火、盖到位锁灯口）。
"""
import math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_REST, DROP_GRASP,
    SAMPLE_BOTTLE_XY, SPOON_XY,
    SPOON_BOWL_BOTTOM,
    EFFECT_SPOON_LIQUID,
    EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP,
    MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    SPOON_GRASP, SPOON_REST,
    SPOON_HELD_OFFSET, SPOON_LIQUID_OFFSET,
    FLAME_HOLD_TCP,
    CAP_GRASP, CAP_BURNER, CAP_CLOSED_THRESHOLD,
    CAP_COVER_NEAR, CAP_EXTINGUISH_XY, CAP_EXTINGUISH_Z,
    CAP_HELD_OFFSET, CAP_CENTER_DZ,
    IGNITION_DELAY, BURN_STEP, BURN_OUT_AT,
    BOIL_DELAY, EVAP_STEP, BOIL_CYCLE, SPOON_BUBBLE_RISE, SPOON_BUBBLE_N,
    SPOON_FLAME_NEAR,
    EFFECT_SPOON_FLAME, EFFECT_SPOON_FLAME_CONE, EFFECT_SPOON_FLAME_SPHERE,
    FLAME_COLOR_OPTIONS,
    EFFECT_SPOON_BUBBLE, EFFECT_LAMP_FLAME_GRPS,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    药品瓶口 xy（排空气/浸液区）
      spoon_xy     燃烧匙碗 xy（滴液区）
    """

    def __init__(self, task, name, path, orig, grasp, bottle_xy, spoon_xy,
                 fill_path=None):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.bottle_xy = np.array(bottle_xy)
        self.spoon_xy = np.array(spoon_xy)
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
                print(f"[c4] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[c4] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[c4] {self.name} filled (aspirated)")
        elif self.state == "filled":
            # 液柱跟随尖嘴
            if self.fill_path:
                self.task._set_fill_follow(self)
            # 碗口区挤胶头滴液
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.spoon_xy, gripper_pos)):
                self.state = "dropped"
                self.dropped = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, False)
                self.task._on_drop(self)
                print(f"[c4] {self.name} dropped into spoon")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[c4] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[c4] {self.name} released to rack -> rest")


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段 ②点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （与滴管的矩阵持握不同——火柴杆横躺，夹爪手指朝下竖直夹其杆身）。
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
                print(f"[c4] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位 MATCH_LIFT_Z）：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[c4] match released to rest")


class _SpoonLifecycle:
    """燃烧匙状态机（rest → attached → rest，阶段 ③ 水平横夹入外焰燃烧 4s → 观察 10s → 放回原位）。

    纯平移持握（同火柴）：勺原点（碗口平面）= 夹爪 + SPOON_HELD_OFFSET，姿态不变。
    碗液（/World/SpoonLiquid 或 <色> 变体）随碗平移（勺原点下 6.2mm = SPOON_LIQUID_OFFSET，
    即碗底上方 2mm 液面基准；高度逐滴长高的 HeightAttr 不动，只改 translate）。
    2026-09-02 用户「燃烧 4s → +y 5cm 观察 10s → 放回」：碗在外焰停留后夹爪回杆身抓点松爪 → 勺写回
    台面静止位并复位 rest（不再跟随）。
    """

    def __init__(self, task, name, path, rest, grasp, liquid_path):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(rest)
        self.grasp = np.array(grasp)
        self.liquid_path = liquid_path
        self.state = "rest"
        self._near_frames = 0
        self.attached = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.task._set_obj_world(self.path, self.orig)
        self.task._set_obj_world(self.liquid_path, self.task._liquid_center_at(self.orig))

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
                print(f"[c4] spoon attached (grip={opening:.4f})")
            return
        # 吸附期：勺 + 碗液跟随夹爪（纯平移保姿态）
        spoon_pos = np.asarray(gripper_pos) + np.array(SPOON_HELD_OFFSET)
        self.task._set_obj_world(self.path, spoon_pos)
        self.task._set_obj_world(self.liquid_path, spoon_pos + np.array(SPOON_LIQUID_OFFSET))
        # 燃烧完放回：夹爪回杆身抓点（勺原点=静止位）开爪 → 勺写回台面静止位、复位 rest。
        # 液面随当前高度（燃烧烧尽=0 或蒸发残留）写中心，回勺位后保持该高度。
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.grasp, gripper_pos)):
            self.state = "rest"
            self.attached = False
            self.task._set_obj_world(self.path, self.orig)
            self.task._set_obj_world(self.liquid_path,
                                     self.task._liquid_center_at(self.orig))
            print(f"[c4] spoon released to rest")


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
                print(f"[c4] cap attached (grip={opening:.4f})")
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
                    print(f"[c4] cap settled, flame extinguished")
            else:
                self.extinguish_counter = 0
            return

        # settled：帽锁灯口（不再跟随夹爪，帽已停在盖灭位），火焰已熄（_on_cap_settled 处理）


class C4CombustionLiquidTask(BaseTask):
    """C4 燃烧试验（液体样品）任务：滴管吸药品瓶液 → 滴入燃烧匙碗 + 液体效果 prim。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 碗液逐滴生长（底面贴 gen 烘焙的初始液碟底面 0.7986 = 碗底 0.7966 + 2mm，
    # 留 2mm 避碗底内球面 z-fighting；半径 Ø8mm 贴碗内缘）
    SPOON_LIQ_BOTTOM = SPOON_BOWL_BOTTOM + 0.002   # 0.7986
    DROP_LEVEL_STEP = 0.002   # 每滴落定后液面升高 2mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.008    # 上限 8mm（4 滴 = 8mm，顶 0.8066 贴碗口 0.8068 不满溢）

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内液柱 60mm 很满，一挤该是一串滴不是一滴——d3l 用户 2026-08-14）。每滴
    # delay 错帧起落 → 悬停成形 → 加速坠落，落定才长液面。
    # DROPS_PER_SQUEEZE 必须与 gen DROPS_PER_GROUP 一致(4)，gen.verify() 会断言。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数（成串时挂短点整体才连贯）
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.03m 落碗，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    DROPPER = "/World/Dropper"
    MATCH = "/World/Match"
    SPOON = "/World/CombustionSpoon"
    LAMP = "/World/AlcoholLamp"
    CAP = "/World/AlcoholLamp/cap"

    MATCH_IGNITE_NEAR_FRAMES = 15   # 火柴头近灯芯连续帧数阈值（仿 flametest）
    MATCH_IGNITE_DIST = 0.035       # 火柴头距灯芯 < 3.5cm 判定点火接近

    SPOON_LIQUID = EFFECT_SPOON_LIQUID
    SPOON_LIQUID_COLOR = "/World/SpoonLiquid_{name}"   # 候选色碗液变体
    BOTTLE_LIQUID = "/World/SampleLiquid"              # 瓶内液（半瓶，场景默认可见）
    BOTTLE_LIQUID_COLOR = "/World/SampleLiquid_{name}"  # 候选色瓶液变体
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    # 液体燃烧/沸腾现象效果 prim（阶段 ③ dwell，config combustion 双现象）
    SPOON_FLAME = EFFECT_SPOON_FLAME                 # 液面火焰组（pivot=火焰底=液面）
    SPOON_FLAME_PRIMS = (EFFECT_SPOON_FLAME_CONE, EFFECT_SPOON_FLAME_SPHERE)
    SPOON_BUBBLE = EFFECT_SPOON_BUBBLE               # 沸腾气泡父（Bubble_0..N）
    SPOON_BUBBLE_N = SPOON_BUBBLE_N                  # 气泡数（reset/step 遍历用，gen 同值）
    LAMP_FLAME_GRPS = EFFECT_LAMP_FLAME_GRPS         # 酒精灯火焰组（pivot=火焰底）
    # 碗液持握时中心基准：勺原点下 0.0102 = 碗底（SPOON_LIQUID_OFFSET[2] - DROP_LEVEL_MAX/2，
    # 满高 8mm 中心 = 勺原点 −0.0062 = SPOON_LIQUID_OFFSET，与 _SpoonLifecycle 一致）
    SPOON_BOWL_BOTTOM_DZ = SPOON_LIQUID_OFFSET[2] - DROP_LEVEL_MAX / 2.0

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        # 液体颜色：--result liquid_color=<色>（clear=原本色瓶液/碗液不变色）
        self.liquid_color = str(getattr(cfg, "liquid_color", "clear")).strip().lower()
        # 碗液路径：clear → SpoonLiquid（无色），其余 → SpoonLiquid_<色> 变体
        self._spoon_liquid_path = (self.SPOON_LIQUID_COLOR.format(name=self.liquid_color)
                                   if self.liquid_color != "clear" else self.SPOON_LIQUID)
        # 瓶液路径：clear → SampleLiquid（原本半瓶液），其余 → SampleLiquid_<色> 变体
        self._bottle_liquid_path = (self.BOTTLE_LIQUID_COLOR.format(name=self.liquid_color)
                                    if self.liquid_color != "clear" else self.BOTTLE_LIQUID)

        # 滴管生命周期句柄（参考点已 gen verify 实测）
        self.dropper = _DropperLifecycle(
            self, "dropper", self.DROPPER, DROP_REST, DROP_GRASP,
            SAMPLE_BOTTLE_XY, SPOON_XY, fill_path=self.DROPPER_FILL)
        self._drop_count = 0       # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []      # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）

        # 火柴生命周期句柄 + 点火状态（阶段 ② 点燃酒精灯）
        self._disable_collision(self.MATCH)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH, match_rest, MATCH_GRASP)
        self.flame_lit = False         # 火柴触灯芯点燃（火焰 reveal）
        self.match_ignite_counter = 0  # 火柴头近灯芯连续帧计数
        self.flame_prims = self._flame_paths()   # 4 个顶层火焰 prim（外/内焰各 球+锥）

        # 燃烧匙生命周期句柄（阶段 ③ 水平横夹入外焰）：碗液随勺平移
        self._disable_collision(self.SPOON)
        self.spoon = _SpoonLifecycle(self, "spoon", self.SPOON, SPOON_REST,
                                     SPOON_GRASP, self._spoon_liquid_path)

        # 灯帽生命周期句柄（阶段 ⑤ 燃烧放回后盖帽灭火）：帽纯平移持握，盖到位熄火锁灯口
        self._disable_collision(self.CAP)
        self.cap_closed_threshold = CAP_CLOSED_THRESHOLD      # 帽 attach 阈值（帽 Ø37mm）
        self.cap_cover_near = CAP_COVER_NEAR                  # 夹爪距 CAP_BURNER 盖到位近窗
        self.cap_dwell_frames = int(getattr(cfg, "cap_dwell_frames", 15))  # 盖到位连续帧
        self.cap = _CapLifecycle(self, "cap", self.CAP, CAP_GRASP, CAP_BURNER)
        self.cap_rest_translate = self._read_cap_translate()  # 帽静止位 local translate
        self.flame_extinguished = False

        # 液体燃烧/沸腾现象（阶段 ③ dwell，config combustion 双现象）：combustible=
        # 液面点燃淡蓝火焰+液面渐降烧尽；non_combustible=沸腾冒泡蒸发（无火焰）。
        self.combustion = str(getattr(cfg, "combustion", "combustible")).strip().lower()
        # 液面燃烧火焰色：--result flame_color=<色>（默认 blue 淡蓝），用户 09-02「火焰颜色通过输入决定」
        self.flame_color = str(getattr(cfg, "flame_color", "blue")).strip().lower()
        self._spoon_flame_grp = self.SPOON_FLAME.format(color=self.flame_color)
        self._spoon_flame_prims = tuple(p.format(color=self.flame_color) for p in self.SPOON_FLAME_PRIMS)
        self._burn_phase = "idle"        # idle/heating/ignited/burned_out 或 boiling
        self._burn_frames = 0
        self._burn_height = 0.0

    # ------------------------------------------------------------------
    def reset(self):
        print("[c4] task.reset: world.reset()...")
        super().reset()
        print("[c4] task.reset: robot.initialize()...")
        self.robot.initialize()
        self._drop_count = 0
        self._drop_queue = []
        self.dropper.reset()
        # 火柴复位 + 火焰隐藏（阶段 ② 点火前，同 B2 reset）
        self.match.reset()
        # 燃烧匙复位（勺回台面、碗液回碗位；阶段 ③）
        self.spoon.reset()
        # 灯帽复位（帽回灯旁静止位；阶段 ⑤）
        self.cap.reset()
        self.flame_lit = False
        self.flame_extinguished = False
        self.match_ignite_counter = 0
        self._set_flame_visible(False)
        # 隐藏所有效果 prim + 候选色变体
        for p in (self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visibility(p, False)
        for name in ("red", "blue", "green", "purple"):
            self._set_visibility(self.BOTTLE_LIQUID_COLOR.format(name=name), False)
            self._set_visibility(self.SPOON_LIQUID_COLOR.format(name=name), False)
        # 碗液复位：隐藏、高度归零（位置回碗底基准）
        self._set_visibility(self.SPOON_LIQUID, False)
        p = self.stage.GetPrimAtPath(self.SPOON_LIQUID)
        if p.IsValid():
            UsdGeom.Cylinder(p).GetHeightAttr().Set(0.0)
        # 燃烧/沸腾现象复位（阶段 ③）：液面火焰/气泡隐藏、状态归零
        self._burn_phase = "idle"
        self._burn_frames = 0
        self._burn_height = 0.0
        self._set_spoon_flame_visible(False)
        # 隐藏所有火焰色变体（非所选色保持隐藏，防多次 reset 残留）
        for name in FLAME_COLOR_OPTIONS:
            self._set_visibility(EFFECT_SPOON_FLAME_CONE.format(color=name), False)
            self._set_visibility(EFFECT_SPOON_FLAME_SPHERE.format(color=name), False)
        self._set_visibility(self.SPOON_BUBBLE, False)
        for i in range(self.SPOON_BUBBLE_N):
            self._set_visibility(f"{self.SPOON_BUBBLE}/Bubble_{i}", False)
        # 瓶液：只显示所选色（clear → 原本 SampleLiquid 可见，色变体隐藏）
        if self.liquid_color != "clear":
            self._set_visibility(self.BOTTLE_LIQUID, False)
        self._set_visibility(self._bottle_liquid_path, True)

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
        self.match.step(gripper_pos, opening)
        self.spoon.step(gripper_pos, opening)
        self.cap.step(gripper_pos, opening)      # 盖帽阶段：帽跟随/下降熄火/盖到位锁灯口
        self._step_match_ignite(gripper_pos)   # 点火检测（火柴头触灯芯 → flame_lit → 火焰 reveal）
        self._step_flame_anim()                 # 酒精灯火焰每帧 flicker（点燃后到熄灭前）
        self._step_burn_phenomenon(gripper_pos) # 碗液燃烧/沸腾现象（阶段 ③ dwell 期间）
        return self.get_basic_state_info(additional_info={
            "dropper_attached": self.dropper.attached,
            "dropper_filled": self.dropper.filled,
            "dropper_dropped": self.dropper.dropped,
            "dropper_released": self.dropper.released,
            "match_attached": self.match.attached,
            "match_released": self.match.released,
            "spoon_attached": self.spoon.attached,
            "flame_lit": self.flame_lit,
            "cap_attached": self.cap.attached,
            "cap_settled": self.cap.settled,
            "flame_extinguished": self.flame_extinguished,
            "combustion": self.combustion,
            "burn_phase": self._burn_phase,
        })

    def on_task_complete(self, success):
        print(f"[c4] episode done success={success} "
              f"dropper_dropped={self.dropper.dropped} "
              f"dropper_released={self.dropper.released} "
              f"match_released={self.match.released} flame_lit={self.flame_lit}")
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
        """DropperFill 截锥液柱跟随滴管尖嘴：translate=尖嘴（柱底贴尖嘴，+Z 收窄→加宽贴合
        玻璃体）。尖嘴在夹爪下 0.13m（保竖立），液柱从尖嘴向上 60mm（几何见 gen 脚本），
        整体在玻璃体直管段内、不顶到胶头。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(self.DROPPER_FILL, tip)

    # ------------------------------------------------------------------
    # 火柴位姿 + 点火检测
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
        """点火检测（仿 flametest/B2 _step_match_ignite）：火柴 attached 期间头近灯芯
        连续 MATCH_IGNITE_NEAR_FRAMES 帧 → flame_lit=True → 火焰 reveal。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    self._set_flame_visible(True)
                    print(f"[c4] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    def _flame_paths(self):
        # 火焰迁到 /World 顶层分组（gen rebuild_flames 水滴形 = 底半球 Sphere + 上部 Cone，
        # 每焰两 prim 挂 /World/<名>_grp 组下，pivot=火焰底）：灯下引用子 prim 在 RTX 不
        # 渲染，顶层组才渲染（flametest 已验证）。gen 初始隐藏，task 点着翻 visible；
        # 组本身每帧 flicker（scale/rotate），见 _step_flame_anim。
        return ["/World/flame_outer_grp/flame_outer",
                "/World/flame_outer_grp/flame_outer_sphere",
                "/World/flame_inner_grp/flame_inner",
                "/World/flame_inner_grp/flame_inner_sphere"]

    def _set_flame_visible(self, visible):
        """点火 reveal / 熄火隐藏全部火焰 prim（外/内焰各 球+锥 共 4 个）。"""
        for p in self.flame_prims:
            self._set_visibility(p, visible)

    # ------------------------------------------------------------------
    # 火焰 flicker + 液体燃烧/沸腾现象（阶段 ③ dwell，config combustion 双现象）
    # ------------------------------------------------------------------
    def _smooth_noise(self, t, seed):
        """确定性平滑噪声（每 3 帧一个随机值，相邻 smoothstep 插值）。火焰 flicker 用，
        同输入同输出（无随机抖动，录像可复现）。

        seed 只偏移采样点（_rand(i+seed*131)），不参与 f 相位——旧实现把 seed 加进
        floor(t/span) 导致每 3 帧 f 跳回 0 且 v0/v1 换段，火焰尺寸/侧摆每 3 帧突跳一次
        = 20Hz 爆闪（用户 09-01「点燃酒精灯后三摄像头红黄蓝爆闪」），已修。"""
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
        不漂移）。base_pos 给则同时写 translate（液面火焰跟随液面），None 则不动 translate。"""
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
        """酒精灯火焰每帧 flicker（点火后到熄灭前，用户 09-01「火焰要动起来不要不动
        不然太假了」）。"""
        if not (self.flame_lit and not self.flame_extinguished):
            return
        self._apply_flame_flicker(self.LAMP_FLAME_GRPS[0], None, seed=1)
        self._apply_flame_flicker(self.LAMP_FLAME_GRPS[1], None, seed=2)

    def _liquid_center_at(self, orig):
        """碗液中心世界坐标（勺原点 orig + 碗底偏移 + 当前高度/2）：液面随高度升降
        （燃烧烧尽/蒸发残留都让中心贴当前液面）。"""
        h = 0.0
        prim = self.stage.GetPrimAtPath(self._spoon_liquid_path)
        if prim.IsValid():
            h = max(0.0, float(UsdGeom.Cylinder(prim).GetHeightAttr().Get()))
        return np.array(orig) + np.array([0.0, 0.0, self.SPOON_BOWL_BOTTOM_DZ + h / 2.0])

    def _step_burn_phenomenon(self, gripper_pos):
        """碗液燃烧/沸腾现象（阶段 ③ dwell 期间）：combustible=液面点燃淡蓝火焰+液面
        渐降烧尽（一旦点燃，移开酒精灯也持续燃烧到烧尽，用户 09-02「移开酒精灯后还
        应燃烧一段时间」）；non_combustible=沸腾冒泡蒸发（无火焰，离开外焰即停沸）。
        触发=勺 attached 且夹爪近 FLAME_HOLD_TCP（碗在外焰中停留）。"""
        in_flame = (self.spoon.attached and np.linalg.norm(
            np.asarray(gripper_pos) - np.array(FLAME_HOLD_TCP)) < SPOON_FLAME_NEAR)
        # 可燃液体一旦点燃（ignited）就持续燃烧到烧尽，不再依赖 in_flame：
        # 阶段 ⑨ 碗移开酒精灯（OBSERVE_TCP）后火焰继续，直到液面烧尽（用户 09-02）。
        if self.combustion == "combustible" and self._burn_phase == "ignited":
            self._step_burn_combustible()
            return
        if not in_flame:
            if self._burn_phase in ("heating", "ignited", "burned_out", "boiling"):
                # 离开火焰：隐藏液面火焰/气泡（未点燃或不可燃，液面保持当前高度）
                self._set_spoon_flame_visible(False)
                self._set_visibility(self.SPOON_BUBBLE, False)
            self._burn_phase = "idle"
            self._burn_frames = 0
            return
        if self._burn_phase == "idle":
            self._burn_phase = "heating"
            self._burn_frames = 0
            # 首次进入：记录当前液面高度（滴加后的满高）
            prim = self.stage.GetPrimAtPath(self._spoon_liquid_path)
            if prim.IsValid():
                self._burn_height = max(0.0, float(
                    UsdGeom.Cylinder(prim).GetHeightAttr().Get()))
            else:
                self._burn_height = 0.0
        if self.combustion == "combustible":
            self._step_burn_combustible()
        else:
            self._step_burn_non_combustible()

    def _step_burn_combustible(self):
        """可燃：heating（受热升温）→ ignited（液面点燃淡蓝火焰）→ 液面渐降 → 烧尽熄火。"""
        self._burn_frames += 1
        if self._burn_phase == "heating":
            if self._burn_frames >= IGNITION_DELAY:
                self._burn_phase = "ignited"
                self._set_spoon_flame_visible(True)
                print(f"[c4] liquid ignited (combustible) @ frame {self.frame_idx}")
            return
        if self._burn_phase == "ignited":
            # 液面火焰跟随液面 + flicker（火焰要动）
            self._set_spoon_flame_pos()
            # 液面渐降（燃烧消耗）
            self._burn_height = max(0.0, self._burn_height - BURN_STEP)
            self._set_burn_liquid(self._burn_height)
            if self._burn_height <= BURN_OUT_AT:
                self._burn_phase = "burned_out"
                self._set_spoon_flame_visible(False)
                self._set_burn_liquid(0.0)   # 烧尽：碗内液体清空（用户 09-01「燃烧完应该是空的」）
                # 烧尽即隐藏碗液 prim：绿色液体在燃烧烧尽那一刻就消失，不拖到放回桌子
                # （用户 09-02「消失不应该出现在燃烧之后吗」——不能靠高度归零间接隐掉，
                #   height=0 的退化圆柱仍可能渲染出残色，必须显式 MakeInvisible）。
                self._set_visibility(self._spoon_liquid_path, False)
                print(f"[c4] liquid burned out @ frame {self.frame_idx}")
            return
        # burned_out：空勺在火焰中，无效果

    def _step_burn_non_combustible(self):
        """不可燃：heating（受热升温）→ boiling（沸腾冒泡蒸发，无火焰）。"""
        self._burn_frames += 1
        if self._burn_phase == "heating":
            if self._burn_frames >= BOIL_DELAY:
                self._burn_phase = "boiling"
                self._set_visibility(self.SPOON_BUBBLE, True)
                print(f"[c4] liquid boiling (non_combustible) @ frame {self.frame_idx}")
            return
        if self._burn_phase == "boiling":
            self._step_bubbles()                 # 气泡上升（错帧循环）
            self._burn_height = max(0.0, self._burn_height - EVAP_STEP)  # 蒸发缓慢下降
            self._set_burn_liquid(self._burn_height)
            return

    def _step_bubbles(self):
        """沸腾气泡：从当前液面错帧上升（每泡一周期，顶部消失 = 泡破），持续循环。"""
        spoon_pos = np.asarray(self.robot.get_gripper_position()) + np.array(SPOON_HELD_OFFSET)
        surface_z = spoon_pos[2] + self.SPOON_BOWL_BOTTOM_DZ + self._burn_height + 0.001
        bx, by = spoon_pos[0], spoon_pos[1]
        for i in range(self.SPOON_BUBBLE_N):
            cycle = (self._burn_frames + i * (BOIL_CYCLE // self.SPOON_BUBBLE_N)) % BOIL_CYCLE
            frac = cycle / float(BOIL_CYCLE)
            if frac >= 0.85:                     # 顶部消失（泡破）
                self._set_visibility(f"{self.SPOON_BUBBLE}/Bubble_{i}", False)
                continue
            self._set_visibility(f"{self.SPOON_BUBBLE}/Bubble_{i}", True)
            z = surface_z + frac * SPOON_BUBBLE_RISE
            self.object_utils.set_object_position(
                f"{self.SPOON_BUBBLE}/Bubble_{i}", (bx, by, z))

    def _set_burn_liquid(self, h):
        """燃烧/蒸发：更新碗液高度+中心（中心=碗底 + 新h/2，贴碗底基准不漂移）。"""
        prim = self.stage.GetPrimAtPath(self._spoon_liquid_path)
        if not prim.IsValid():
            return
        spoon_pos = np.asarray(self.robot.get_gripper_position()) + np.array(SPOON_HELD_OFFSET)
        bowl_bottom_z = spoon_pos[2] + self.SPOON_BOWL_BOTTOM_DZ
        UsdGeom.Cylinder(prim).GetHeightAttr().Set(max(0.0, h))
        self.object_utils.set_object_position(self._spoon_liquid_path,
            (spoon_pos[0], spoon_pos[1], bowl_bottom_z + h / 2.0))

    def _set_spoon_flame_pos(self):
        """液面火焰写 translate 到当前液面 + flicker（可燃点燃期间每帧调用）。"""
        spoon_pos = np.asarray(self.robot.get_gripper_position()) + np.array(SPOON_HELD_OFFSET)
        surface_z = spoon_pos[2] + self.SPOON_BOWL_BOTTOM_DZ + self._burn_height + 0.0005
        self._apply_flame_flicker(self._spoon_flame_grp,
                                  base_pos=(spoon_pos[0], spoon_pos[1], surface_z), seed=3)

    def _set_spoon_flame_visible(self, visible):
        """液面火焰 reveal/隐藏（锥+球两 prim）。"""
        for p in self._spoon_flame_prims:
            self._set_visibility(p, visible)

    # ------------------------------------------------------------------
    # 灯帽位姿 + 熄火（阶段 ⑤ 盖帽灭火，照 B2）
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
        rotateXYZ+scale 形状）。换算（pxr 实测，灯 R180Z，照 B2）：
        cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+CAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−CAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world(self.LAMP)
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
        print(f"[c4] flame extinguished @ frame {self.frame_idx}")

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

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴（尖嘴下逐滴错落坠落）。

        挤胶头瞬间在尖嘴正下方生成一串亮蓝液滴（DropperDrop 父 Xform 的 Drop_0.._N 球，
        每滴一格），delay 错帧起落形成连续"滴-滴-滴"（液柱 60mm 很满，一挤该是一串滴
        不是一滴）。每滴落定后碗液长高 DROP_LEVEL_STEP。
        """
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（碗口上方，液滴可见坠落）
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            target = np.array([SPOON_XY[0], SPOON_XY[1],
                               self.SPOON_LIQ_BOTTOM + level - 0.002])  # 落定在碗内液面
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(), "target": target,
                "level": level,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._drop_count += self.DROPS_PER_SQUEEZE
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[c4] squeeze -> {self.DROPS_PER_SQUEEZE} drops spawned")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该球+长碗液）。"""
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
                # 落定：隐藏这颗、长碗液，移出队列
                self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_spoon_level(d["level"])
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    def _grow_spoon_level(self, h):
        """液滴落定：燃烧匙碗液长到高度 h（圆柱高+上移，底面贴碗底），与瓶液同色。"""
        prim = self.stage.GetPrimAtPath(self._spoon_liquid_path)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self._spoon_liquid_path,
                (SPOON_XY[0], SPOON_XY[1], self.SPOON_LIQ_BOTTOM + h / 2))
        self._set_visibility(self._spoon_liquid_path, True)
        print(f"[c4] spoon liquid level h={h:.3f}")

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
