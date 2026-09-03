"""D1 酸碱滴定 P1（加指示剂）：滴管「吸酚酞 → 滴入锥形瓶 W」生命周期 + 无色→粉。

持握照 d3l/flametest（已验证）：滴管是静态碰撞体，吸附期逐帧把**世界位置**写为
TCP + HELD_OFFSET（只写 xformOp:translate，不写旋转矩阵）——滴管全程保持架内竖立
姿态（胶头上、尖嘴 0.13m 吊在夹爪下方），DropperFill 只需 position 跟随尖嘴即可。

生命周期（gripper 开度 = joint[7]，判定纯关节+TCP，无碰撞依赖）：
  rest → attached → squeezed → filled → dropped → released
  - rest     架内竖插（尖嘴 z=0.80）；夹爪接近抓点且合拢（连续 GRASP_NEAR_FRAMES 帧
             < gripper_closed）→ attached
  - attached 跟随；指示剂瓶口区挤胶头（<gripper_squeezed）→ squeezed（排空气）
  - squeezed 跟随；瓶口区松胶头（squeezed~closed 之间）→ filled（吸液）
             → DropperFill 显示（酚酞液柱被吸进尖嘴）
  - filled   跟随（DropperFill 逐帧跟随尖嘴）；锥形瓶 W 口区挤胶头（<gripper_squeezed）
             → dropped → DropperFill 隐藏 + 3 颗粉球成串坠落入瓶
  - dropped  跟随；回架松开（>gripper_open 且近抓点）→ released（写回架内竖插位姿）
             并复位 rest

瓶内 NaOH 无色（FlaskNaOH 初始可见）；最后一颗坠滴落定 → FlaskNaOH 隐藏 +
FlaskNaOHPink 显示（样液无色→粉，几何同柱 r0.033/h0.008，见 gen）。液柱/坠滴都是
可动画的几何 prim（改 translate/高度即时渲染，坑21 规避材质运行时段），为后续
「搬瓶→管下→滴定→摇瓶」的液面移动/摇晃效果预留：换色只切可见性、液面高度可生长。

reset()：NaOH 回无色、粉隐藏、滴管回架、DropperFill/Drop 全隐藏。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET,
    DROP_XY, DROP_REST, DROP_GRASP,
    IND_BOTTLE_XY, FLASK_XY,
    FLASK_NAOH_TOP_Z,
    EFFECT_FLASK_NAOH, EFFECT_FLASK_PINK,
    EFFECT_DROPPER_FILL, EFFECT_DROPPER_DROP,
)

# 滴管相对夹爪的持握偏移（flametest 同款）：HELD = REST - GRASP，纯平移不写旋转。
# 抓点 = 立放位 + (0,0,0.13)，故偏移 = (0,0,-0.13)：滴管全程保竖立、尖嘴 0.13m 吊在
# 夹爪下方（尖嘴底=原点，TCP z = 尖嘴 z + 0.13）。
HELD_OFFSET = np.array([0.0, 0.0, -TIP_OFFSET])

# DropperFill 柱（h0.045，圆柱以 translate 为中心）→ 要让柱体从尖嘴**向上**长 45mm，
# 需把 translate 抬到 尖嘴 + h/2（d3l 的 fill 是底心约定，D1 gen 是中心约定，此处补偿）。
FILL_CYL_HALF = 0.045 / 2.0


class _DropperLifecycle:
    """单支滴管状态机（rest/attached/squeezed/filled/dropped/released）。

    参考点（均为 gripper/TCP 世界坐标）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    指示剂瓶口 xy（排空气/浸液区，z 不区分）
      tube_xy      锥形瓶 W 口 xy（滴液区）
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
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d1] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（纯平移保竖立）
        self.task._set_obj_world(self.path, gripper_pos + HELD_OFFSET)

        if self.state == "attached":
            # 指示剂瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[d1] {self.name} squeezed-air at indicator bottle")
        elif self.state == "squeezed":
            # 指示剂瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and self.task._near_xy(self.bottle_xy, gripper_pos)):
                self.state = "filled"
                self.filled = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, True)
                print(f"[d1] {self.name} filled (aspirated phenolphthalein)")
        elif self.state == "filled":
            # 液柱跟随尖嘴
            if self.fill_path:
                self.task._set_fill_follow(self)
            # 锥形瓶 W 口区挤胶头滴液
            if (opening < self.task.gripper_squeezed_threshold
                    and self.task._near_xy(self.tube_xy, gripper_pos)):
                self.state = "dropped"
                self.dropped = True
                if self.fill_path:
                    self.task._set_visibility(self.fill_path, False)
                self.task._on_drop(self)
                print(f"[d1] {self.name} dropped into conical flask W")
        elif self.state == "dropped":
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            if (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world(self.path, self.orig)
                self.state = "rest"
                print(f"[d1] {self.name} released to rack -> rest")


class D1AcidBaseTitrationTask(BaseTask):
    """D1 P1 加指示剂任务：滴管吸酚酞 → 滴入锥形瓶 W 无色→粉 + 液体效果 prim。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 锥形瓶内 NaOH 液面（世界 z，FlaskNaOH 柱顶；液滴落定贴此面）
    FLASK_NAOH_TOP_Z = FLASK_NAOH_TOP_Z
    DROP_LAND_MARGIN = 0.002   # 坠滴落定在液面下 2mm（贴液面，"滴进去了"）

    # 滴落动画（同 d3l）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴，每滴 delay 错帧起落
    # → 悬停成形 → 加速坠落，落定才换色。DROPS_PER_SQUEEZE 必须与 gen Drop_0..2 一致(3)，
    # gen.verify() 会断言。
    DROPS_PER_SQUEEZE = 3
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.18m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）

    DROPPER = "/World/Dropper"
    FLASK_NAOH = EFFECT_FLASK_NAOH
    FLASK_PINK = EFFECT_FLASK_PINK
    DROPPER_FILL = EFFECT_DROPPER_FILL
    DROPPER_DROP = EFFECT_DROPPER_DROP

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 滴管是静态碰撞体：吸附期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰）
        self._disable_collision(self.DROPPER)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.0035)

        # 单支滴管生命周期（指示剂瓶 = bottle，锥形瓶 W = tube）
        self.dropper = _DropperLifecycle(
            self, "indicator", self.DROPPER, DROP_REST, DROP_GRASP,
            IND_BOTTLE_XY, FLASK_XY, fill_path=self.DROPPER_FILL)
        self._landed = 0            # 已落定坠滴数（换色门控）
        self._drop_queue = []       # 滴落动画队列（当前在飞的滴）
        self._pinked = False        # 样液已变粉（FlaskNaOH → FlaskNaOHPink 已切）

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self._landed = 0
        self._drop_queue = []
        self._pinked = False
        self.dropper.reset()
        # NaOH 回无色、粉隐藏；DropperFill/Drop 全隐藏
        self._set_visibility(self.FLASK_NAOH, True)
        self._set_visibility(self.FLASK_PINK, False)
        self._set_visibility(self.DROPPER_FILL, False)
        for k in range(self.DROPS_PER_SQUEEZE):
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{k}", False)
        self._set_visibility(self.DROPPER_DROP, False)

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
        return self.get_basic_state_info(additional_info={
            "indicator_attached": self.dropper.attached,
            "indicator_filled": self.dropper.filled,
            "indicator_dropped": self.dropper.dropped,
            "sample_pinked": self._pinked,
            "dropper_released": self.dropper.released,
        })

    def on_task_complete(self, success):
        print(f"[d1] episode done success={success} "
              f"dropped={self.dropper.dropped} pinked={self._pinked} "
              f"released={self.dropper.released}")
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
        """把物体逐帧向 target 平滑移动（消除闪现吸附）。"""
        cur = self._get_obj_world(path)
        if cur is None:
            return
        nxt = cur + (target - cur) * k
        self._set_obj_world(path, nxt)

    def _set_fill_follow(self, dropper):
        """DropperFill 液柱跟随滴管尖嘴：圆柱中心 = 尖嘴 + 柱半高（柱从尖嘴向上 45mm，
        在滴管玻璃管段内，不顶到胶头）。尖嘴在夹爪下 0.13m（保竖立）。"""
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        self.object_utils.set_object_position(
            self.DROPPER_FILL, (tip[0], tip[1], tip[2] + FILL_CYL_HALF))

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def _on_drop(self, dropper):
        """挤胶头滴液：一次成串 DROPS_PER_SQUEEZE 颗粉球从尖嘴下坠落入锥形瓶 W。

        挤胶头瞬间在尖嘴正下方生成 3 颗粉色液滴（DropperDrop 父 Xform 的 Drop_0..2，
        每颗一格），delay 错帧起落形成连续"滴-滴-滴"；每颗落定贴 NaOH 液面（0.810），
        最后一颗落定触发无色→粉（_on_flask_drop_land）。
        """
        tip = np.asarray(self.robot.get_gripper_position(), dtype=float) + HELD_OFFSET
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（瓶口上方 20mm，坠落可见）
        land_z = self.FLASK_NAOH_TOP_Z - self.DROP_LAND_MARGIN   # 贴瓶内 NaOH 液面
        for i in range(self.DROPS_PER_SQUEEZE):
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(),
                "target": np.array([FLASK_XY[0], FLASK_XY[1], land_z]),
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._set_visibility(self.DROPPER_DROP, True)
        print(f"[d1] squeeze -> {self.DROPS_PER_SQUEEZE} drops spawned "
              f"(target z={land_z:.3f})")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该颗+换色推进）。"""
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
                # 落定：隐藏这颗、无色→粉推进，移出队列
                self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._on_flask_drop_land()
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visibility(self.DROPPER_DROP, False)

    def _on_flask_drop_land(self):
        """坠滴落定：计数；最后一颗落定 → FlaskNaOH 无色隐藏 + FlaskNaOHPink 显示。

        酚酞遇 NaOH 碱液即刻显粉：3 颗都滴入后整份样液无色→粉（几何同柱，只切可见性）。
        """
        self._landed += 1
        if self._pinked:
            return
        if self._landed < self.DROPS_PER_SQUEEZE:
            return
        self._set_visibility(self.FLASK_NAOH, False)
        self._set_visibility(self.FLASK_PINK, True)
        self._pinked = True
        print(f"[d1] sample colorless -> PINK (all {self.DROPS_PER_SQUEEZE} drops landed)")

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
