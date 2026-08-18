"""D3-L 酸性试剂滴加反应任务：两支滴管各自的「吸液→滴入试管」生命周期 + 液体效果。

持握照 flametest（v24-v46 已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵、不清 xform op 表）——滴管
全程保持架内竖立姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），效果 prim（DropperFill）
只需 position 跟随尖嘴即可。

生命周期（每支滴管，gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插；夹爪接近抓点且合拢（<gripper_closed，连续 3 帧）→ attached
  - attached 跟随；瓶口区挤胶头（<GRIP_SQUEEZED）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（GRIP_SQUEEZED~gripper_closed）→ filled（吸液）
             → DropperFill 显示（液柱被吸进尖嘴，"像不像水"的动态可视）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；试管口区挤胶头（<GRIP_SQUEEZED）
             → dropped → DropperFill 隐藏 + TubeDrops 显示且液面逐滴升高
             （每滴 +DROP_LEVEL_STEP，上限 DROP_LEVEL_MAX）
  - 现象（Bubbles/Precipitate）只在**加酸滴管**滴加（样品+酸混合）时按 cfg 触发
  - dropped  跟随；cycle 未结束回到瓶口再挤（<GRIP_SQUEEZED）→ 回 squeezed（**一次
             持握内循环**吸液-滴液，不松开）；末遍滴完回架松开（>gripper_open）
             → released（写回架内竖插位姿）并复位 rest

两个滴管生命周期句柄（sample/acid）共用一套状态机，各自参考点不同；controller 顺序执行
①SAMPLE_PASS（取样滴管**一次持握内循环吸液-滴液 cfg.sample_cycles 遍**）→ ②ACID_PASS
（加酸滴管同构，cfg.acid_cycles 遍，滴入触发现象）→ ③TUBE_SHAKE_PASS（抓起试管震荡，
试管生命周期 rest→attached→released，管内液柱随管平移，步9）。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
    DROP_ACID_REST, DROP_ACID_GRASP,
    SAMPLE_BOTTLE_XY, ACID_BOTTLE_XY, TUBE_XY, TUBE_GRASP_TCP,
    SHAKE_TOP_Z, SHAKE_STOP_EPS, SHAKE_STILL_FRAMES, PHENOMENA_FRAMES,
    EFFECT_TUBE_DROPS, EFFECT_PRECIPITATE, EFFECT_BUBBLES,
    EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
)

# 滴管相对夹爪的持握偏移（flametest 同款：HELD = REST - GRASP，纯平移不写旋转）。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])

# 试管持握（步9 抓起试管震荡）：抓管身中段（管口下 14mm），管底 0.139m 吊在夹爪下方，
# 管内液柱随管平移（_follow_tube_liquid）。TUBE_ORIG = 架内竖插位姿（管底=原点 z=0.806）。
TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], 0.806])
TUBE_HELD_OFFSET = np.array([0.0, 0.0, TUBE_ORIG[2] - TUBE_GRASP_TCP[2]])   # ≈(0,0,-0.139)


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    所对瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
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
                print(f"[d3l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d3l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d3l] {self.name} filled (aspirated)")
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
                print(f"[d3l] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                print(f"[d3l] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d3l] {self.name} released to rack -> rest")


class _TubeLifecycle:
    """试管生命周期：rest → attached（跟随 + 管内液柱随管平移）→ released。

    参考点（TCP 世界坐标）：
      grasp    试管口下抓点（管身中段，夹爪 z = 管口下 14mm）
    持握 = TCP + TUBE_HELD_OFFSET(0,0,-0.139)（管底 0.139m 吊在夹爪下方，纯平移
    保竖立）。震荡由 ShakeAction 驱动（task 只逐帧跟随，无需额外状态）。
    """

    def __init__(self, task, path, orig, grasp):
        self.task = task
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.released = False
        self.task._set_obj_world(self.path, self.orig)

    def step(self, gripper_pos, opening):
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = gripper_pos + TUBE_HELD_OFFSET
            # 夹爪开始合拢且已进近窗：先把试管平滑拉向持握位（同滴管，消闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d3l] tube attached (grip={opening:.4f})")
            return

        # 吸附期：试管 + 管内液柱逐帧跟随夹爪（纯平移保竖立）
        held = gripper_pos + TUBE_HELD_OFFSET
        self.task._set_obj_world(self.path, held)
        self.task._follow_tube_liquid(held)
        # 回架内松开：写回竖插位姿并复位 rest（液柱随管回原位）
        if (opening > self.task.gripper_open_threshold
                and self.task._near(self.grasp, gripper_pos)):
            self.released = True
            self.task._set_obj_world(self.path, self.orig)
            self.task._follow_tube_liquid(self.orig)
            self.state = "rest"
            print(f"[d3l] tube released to rack -> rest")


class D3LAcidReagentTask(BaseTask):
    """D3-L 酸性试剂滴加任务：两支滴管吸液→滴入试管 + 液体效果 prim。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 管内液体逐滴生长（底面贴管底 0.806；半径贴 Ø19.2 管壁内缘 0.009）
    TUBE_BOTTOM_Z = 0.806
    DROP_LEVEL_STEP = 0.004   # 每滴落定后液面升高 4mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.060    # 上限 60mm（3 挤 ≈ 48mm 接近上限）

    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内液柱 60mm 很满，一挤该是一串滴不是一滴——用户 2026-08-14）。每滴
    # delay 错帧起落 → 悬停成形 → 加速坠落，落定才长液面（4 滴/挤=16mm/挤）。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数（成串时挂短点整体才连贯）
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.13m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    # 现象时序（用户 2026-08-14）：加酸滴入即出现（_grow_tube_level）→ 随管震荡持续 →
    # 震荡停止后 PHENOMENA_FRAMES（3s）消退。检测"震荡停"=试管已抓起 + 夹爪在震荡/停留
    # 高度区 + 连续 SHAKE_STILL_FRAMES 帧位移 < SHAKE_STOP_EPS（震荡峰附近单帧 0.0001-0.0003
    # 但不会持续 20 帧静止；升到高度的到位冻结+settle ≈8 帧也不够——只有震荡结束后的
    # 停留 300 帧才满足，排除震荡前误触发）。
    SHAKE_TOP_Z = SHAKE_TOP_Z
    SHAKE_STOP_EPS = SHAKE_STOP_EPS
    SHAKE_STILL_FRAMES = SHAKE_STILL_FRAMES
    PHENOMENA_FRAMES = PHENOMENA_FRAMES

    # 气泡动画（用户 2026-08-17 重构：从试管底部生成 → 穿过液柱上升 → 到当前液面消失，
    # 不再循环成"页面上方一根黄色柱子震荡"）。小球池：每颗生成时从底部冒出一颗、逐帧
    # 上升、到液面即隐藏（"破掉"），生成错帧错开 → 稀疏冒泡而非整列同步。x/y 用 gen
    # 烘焙基准，每帧只写子球局部 z。N_BUBBLES 必须与 gen 脚本 BUBBLES 数量一致（8）。
    N_BUBBLES = 8
    BUBBLE_SPAWN_Z = 0.812     # 生成高度（管底 0.806 之上、沉淀柱顶 0.8105 之上，在液柱内）
    BUBBLE_POP_MARGIN = 0.002  # 离当前液面下方一点即消失（"破在液面"）
    BUBBLE_RISE = 0.0004       # 每帧上升量（m，@60Hz ≈ 0.024m/s，56mm 全程 ≈2.3s；用户
                              #   "变化太快"→ 0.0006 再降，配合错帧生成更缓）
    BUBBLE_SPAWN_INTERVAL = 25  # 每隔 25 帧生成一颗（≈0.4s 一颗，同屏 4-5 颗稀疏冒泡）

    DROPPER_SAMPLE = "/World/DropperSample"
    DROPPER_ACID = "/World/DropperAcid"
    TUBE = "/World/TestTube"

    TUBE_DROPS = EFFECT_TUBE_DROPS
    PRECIPITATE = EFFECT_PRECIPITATE
    BUBBLES = EFFECT_BUBBLES
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管/试管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER_SAMPLE)
        self._disable_collision(self.DROPPER_ACID)
        self._disable_collision(self.TUBE)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)

        self.has_bubbles = bool(getattr(cfg, "has_bubbles", False))
        self.has_precipitate = bool(getattr(cfg, "has_precipitate", False))

        # 两支滴管各自的生命周期句柄（参考点已 pxr 实测）
        self.droppers = {
            "sample": _DropperLifecycle(
                self, "sample", self.DROPPER_SAMPLE, DROP_SAMPLE_REST, DROP_SAMPLE_GRASP,
                SAMPLE_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
            "acid": _DropperLifecycle(
                self, "acid", self.DROPPER_ACID, DROP_ACID_REST, DROP_ACID_GRASP,
                ACID_BOTTLE_XY, TUBE_XY, fill_path=self.DROPPER_FILL),
        }
        self._tube_drop_count = 0      # 已生成的液滴总数（每滴 +1）
        self._drop_queue = []          # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）
        # 试管生命周期（步9 抓起试管震荡，液柱随管平移）
        self.tube = _TubeLifecycle(self, self.TUBE, TUBE_ORIG, TUBE_GRASP_TCP)

        # 现象时序状态：震荡停检测 + 消退倒计时
        #   _precip_rack         沉淀柱在架内竖插位姿（世界 translate，随管平移回此基准）
        #   _prev_gripper_pos    上一帧夹爪 TCP（算位移判震荡停）
        #   _shake_stop_frames   连续"几乎不动"帧数（≥3 才认为震荡已停）
        #   _phenomena_fade_frame 消退执行帧（frame_idx 到则隐藏现象），None=未排定
        #   _phenomena_faded     已消退过（防止停留期内反复重排倒计时）
        self._precip_rack = None
        r = self._get_obj_world(self.PRECIPITATE)
        if r is not None:
            self._precip_rack = np.asarray(r, dtype=float)
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        self._phenomena_fade_frame = None
        self._phenomena_faded = False
        # 气泡动画状态：基准 x/y（gen 烘焙的局部 translate）保持，每帧只动子球局部 z；
        # _bubble_active[i] 标记第 i 颗是否在飞（池内小球复用，升起→液面消失）
        self._bubbles_visible = False
        self._bubble_bases = []     # [(x, y), ...] 相对试管原位的局部基准
        self._bubble_z = []         # 当前 z（上升中）
        self._bubble_active = []    # 每颗是否在飞（True 显示中，False 空闲待复用）
        self._spawn_timer = 0       # 距下次生成剩余帧数
        self._init_bubble_anim()

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._tube_drop_count = 0
        self._drop_queue = []
        for d in self.droppers.values():
            d.reset()
        self.tube.reset()
        for p in (self.TUBE_DROPS, self.PRECIPITATE, self.BUBBLES,
                  self.DROPPER_FILL, self.DROPPER_DROP):
            self._set_visibility(p, False)
        # 现象时序复位：气泡 Xform 回原点（=气泡在管内架位）、沉淀回架内竖插位姿
        self._prev_gripper_pos = None
        self._shake_stop_frames = 0
        self._phenomena_fade_frame = None
        self._phenomena_faded = False
        self._bubbles_visible = False
        self._reset_bubble_anim()
        self.object_utils.set_object_position(self.BUBBLES, (0.0, 0.0, 0.0))
        if self._precip_rack is not None:
            self.object_utils.set_object_position(self.PRECIPITATE, self._precip_rack)

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
        self._step_bubble_anim()            # 气泡上升循环（可见时推进）
        for d in self.droppers.values():
            d.step(gripper_pos, opening)
        self.tube.step(gripper_pos, opening)  # 试管跟随/震荡（步9）
        self._step_phenomena(gripper_pos)     # 现象时序：震荡停→3s 后消退
        return self.get_basic_state_info(additional_info={
            "sample_attached": self.droppers["sample"].attached,
            "sample_filled": self.droppers["sample"].filled,
            "sample_dropped": self.droppers["sample"].dropped,
            "acid_attached": self.droppers["acid"].attached,
            "acid_filled": self.droppers["acid"].filled,
            "acid_dropped": self.droppers["acid"].dropped,
            "tube_attached": self.tube.attached,
            "tube_released": self.tube.released,
        })

    def on_task_complete(self, success):
        print(f"[d3l] episode done success={success} "
              f"sample_dropped={self.droppers['sample'].dropped} "
              f"acid_dropped={self.droppers['acid'].dropped} "
              f"tube_released={self.tube.released}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 滴管位姿
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        """物体尖嘴（原点）世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        """把物体写到给定世界位置（只写现有 xformOp:translate，保竖立姿态）。

        flametest 同款：不 ClearXformOpOrder、不写 4x4 矩阵——烘平场景里滴管/试管只有
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

    def _follow_tube_liquid(self, tube_pos):
        """试管被拿起时管内效果随管平移：液柱（TubeDrops）底贴管底、高度不变；
        气泡（Bubbles 父 Xform）与沉淀（Precipitate）同步平移——现象在**管内**，
        不能留在空架孔处。tube_pos = 试管管底世界坐标（同 _set_obj_world 的 held）。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            h = float(UsdGeom.Cylinder(prim).GetHeightAttr().Get() or 0.0)
            self.object_utils.set_object_position(
                self.TUBE_DROPS, (tube_pos[0], tube_pos[1], tube_pos[2] + h / 2))
        delta = np.asarray(tube_pos, dtype=float) - TUBE_ORIG   # 随管位移（回架=0）
        if self.has_bubbles:
            # Bubbles Xform 原点即"气泡在架内管内"位（球有各自 translate），整体平移 delta
            self.object_utils.set_object_position(self.BUBBLES, delta)
        if self.has_precipitate and self._precip_rack is not None:
            self.object_utils.set_object_position(
                self.PRECIPITATE, self._precip_rack + delta)

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

        挤胶头瞬间在尖嘴正下方生成一串亮蓝液滴（DropperDrop 父 Xform 的 Drop_0.._N 球，
        每滴一格），delay 错帧起落形成连续"滴-滴-滴"（液柱 60mm 很满，一挤该是一串滴
        不是一滴——用户 2026-08-14）。每滴落定后管内液面长高 DROP_LEVEL_STEP。
        反应现象（气泡/沉淀）只在**加酸**（样品+酸混合）时出现（_grow_tube_level
        name=="acid" 按 cfg 显示）。
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
        print(f"[d3l] squeeze ({dropper.name}) -> {self.DROPS_PER_SQUEEZE} drops spawned")

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
        """液滴落定：管内液面长到高度 h（圆柱高+上移，底面贴管底），加酸时触发现象。"""
        prim = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if prim.IsValid():
            UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
            self.object_utils.set_object_position(
                self.TUBE_DROPS,
                (TUBE_XY[0], TUBE_XY[1], self.TUBE_BOTTOM_Z + h / 2))
        self._set_visibility(self.TUBE_DROPS, True)
        if name == "acid":
            if self.has_bubbles:
                self._set_visibility(self.BUBBLES, True)
                self._bubbles_visible = True
            if self.has_precipitate:
                self._set_visibility(self.PRECIPITATE, True)
        print(f"[d3l] tube liquid level h={h:.3f}")

    # ------------------------------------------------------------------
    # 气泡上升动画（用户 2026-08-16：气泡要有移动上升效果）
    # ------------------------------------------------------------------
    def _init_bubble_anim(self):
        """读 N_BUBBLES 颗气泡的烘焙局部 translate 作基准 x/y；初始全隐藏、待生成。"""
        self._bubble_bases = []
        self._bubble_z = []
        self._bubble_active = []
        for i in range(self.N_BUBBLES):
            prim = self.stage.GetPrimAtPath(f"{self.BUBBLES}/Bubble_{i}")
            if not prim.IsValid():
                continue
            t = prim.GetAttribute("xformOp:translate")
            if not t or not t.HasValue():
                continue
            v = t.Get()
            self._bubble_bases.append((float(v[0]), float(v[1])))
            self._bubble_z.append(self.BUBBLE_SPAWN_Z)
            self._bubble_active.append(False)
            self._set_visibility(f"{self.BUBBLES}/Bubble_{i}", False)
        self._spawn_timer = 0

    def _reset_bubble_anim(self):
        """复位小球池：全部隐藏、z 回生成位、计时清零（reset/重跑用，基准 x/y 不变）。"""
        for i in range(len(self._bubble_active)):
            self._bubble_active[i] = False
            self._bubble_z[i] = self.BUBBLE_SPAWN_Z
            self._set_visibility(f"{self.BUBBLES}/Bubble_{i}", False)
        self._spawn_timer = 0

    def _step_bubble_anim(self):
        """气泡动画：小球池按 BUBBLE_SPAWN_INTERVAL 从底部生成一颗、逐帧上升、到**当前
        液面**即隐藏（"破掉"）——不是循环归位，形成从管底冒出穿液柱的稀疏气泡。

        只写子球局部 translate 的 z（x/y 保持基准）；随管平移由 Bubbles 父 Xform 承担
        （父 translate = tube delta，局部 z 不受影响，气泡在管内上升）。仅在气泡可见时
        推进（加酸滴入出现 → 震荡停后消退）。"""
        if not (self.has_bubbles and self._bubbles_visible):
            return
        # 当前液面 = 破灭高度（液面随滴落长高，贴实时液面；留 margin 破在液面下方）
        level = min(self._tube_drop_count * self.DROP_LEVEL_STEP, self.DROP_LEVEL_MAX)
        pop_z = self.TUBE_BOTTOM_Z + level - self.BUBBLE_POP_MARGIN
        # 生成：计时到且有空闲球 → 从底部冒出一颗
        if self._spawn_timer <= 0:
            for i, active in enumerate(self._bubble_active):
                if not active:
                    self._bubble_active[i] = True
                    self._bubble_z[i] = self.BUBBLE_SPAWN_Z
                    self._set_visibility(f"{self.BUBBLES}/Bubble_{i}", True)
                    break
            self._spawn_timer = self.BUBBLE_SPAWN_INTERVAL
        else:
            self._spawn_timer -= 1
        # 推进在飞气泡：上升，到液面消失（隐藏，留作复用）
        for i, (bx, by) in enumerate(self._bubble_bases):
            if not self._bubble_active[i]:
                continue
            z = self._bubble_z[i] + self.BUBBLE_RISE
            if z >= pop_z:
                self._bubble_active[i] = False
                self._set_visibility(f"{self.BUBBLES}/Bubble_{i}", False)
                continue
            self._bubble_z[i] = z
            prim = self.stage.GetPrimAtPath(f"{self.BUBBLES}/Bubble_{i}")
            if prim.IsValid():
                t = prim.GetAttribute("xformOp:translate")
                if t:
                    t.Set(Gf.Vec3d(bx, by, z))

    def _step_phenomena(self, gripper_pos):
        """现象时序（用户 2026-08-14）：加酸滴入即出现 → 随管震荡持续 → **震荡停止后
        3 秒（PHENOMENA_FRAMES）消退**（停留 5s 里前 3s 有现象）。

        检测"震荡停"：试管已抓起（tube.attached）+ 夹爪在震荡/停留高度区（z ≥ SHAKE_TOP_Z）
        + 连续 SHAKE_STILL_FRAMES 帧夹爪位移 < SHAKE_STOP_EPS（震荡时每帧横移 ≈振幅·2π/period
        ≈0.002>eps；升高度的到位冻结+settle 静止只有 ≈8 帧，被帧数门槛排除）。
        一旦判停 → 排定消退帧（当前帧 + PHENOMENA_FRAMES），到帧隐藏 Bubbles/Precipitate。
        """
        if not (self.has_bubbles or self.has_precipitate):
            return
        if self._phenomena_faded:
            return                       # 已消退过（停留期内不反复重排）
        if self._phenomena_fade_frame is not None:
            if self.frame_idx >= self._phenomena_fade_frame:
                self._set_visibility(self.BUBBLES, False)
                self._bubbles_visible = False
                self._set_visibility(self.PRECIPITATE, False)
                self._phenomena_faded = True
                self._phenomena_fade_frame = None
                print(f"[d3l] phenomena faded (frame {self.frame_idx})")
            return                       # 倒计时进行中，等消退
        # —— 检测震荡是否已停（尚未排定消退）——
        if not self.tube.attached or gripper_pos[2] < self.SHAKE_TOP_Z:
            self._prev_gripper_pos = None
            self._shake_stop_frames = 0
            return
        move = 0.0
        if self._prev_gripper_pos is not None:
            move = float(np.linalg.norm(
                np.asarray(gripper_pos, dtype=float) - self._prev_gripper_pos))
        self._prev_gripper_pos = np.asarray(gripper_pos, dtype=float)
        if move < self.SHAKE_STOP_EPS:
            self._shake_stop_frames += 1
            if self._shake_stop_frames >= self.SHAKE_STILL_FRAMES:
                self._phenomena_fade_frame = self.frame_idx + self.PHENOMENA_FRAMES
                print(f"[d3l] shake stopped -> phenomena fade at "
                      f"frame {self._phenomena_fade_frame} "
                      f"(now {self.frame_idx}, +{self.PHENOMENA_FRAMES})")
        else:
            self._shake_stop_frames = 0

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
