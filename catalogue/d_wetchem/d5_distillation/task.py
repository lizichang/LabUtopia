"""D5 蒸馏分离任务：预组装蒸馏装置，机械臂仅点燃酒精灯，蒸馏现象（加热→沸腾→冷凝→
馏出液收集）由 task 现象状态机驱动。

文档 D11 注（逐字）：「极低优先级（仅 1 次）。装置组装涉及多组件磨口玻璃精密对接，建议
初期阶段由人工完成装置组装，机械臂仅执行加热和收集。」

故本实验机械臂只做 1 元动作：
  ① LightFlamePass  拿火柴点燃酒精灯（火柴头触灯芯 → flame_lit）。

蒸馏现象（task 驱动，机械臂点燃后不再动作）：
  加热（气泡随 vigor 渐起）→ 沸腾（气泡全开）→ 蒸馏（冷凝管出口液滴逐滴坠入接液瓶、
  接液瓶内馏出液逐滴生长）→ 收集完成（火焰熄灭、phase="done"）。

本 task 做 3 类驱动（与 d9/b2 同构：纯平移持握 + visibility 切换 + 火焰/气泡/液滴逐帧）：
  1. 火柴生命周期：rest → attached → released（照 D9 _HeldLifecycle，纯平移持握），
     头近灯芯 → flame_lit（酒精灯火焰 reveal）。
  2. 现象状态机（相态 idle→ignited→heating→boiling→distilling→finishing→done）：
     火焰点亮后按帧计数推进；heating 起气泡组 reveal+上升+触液面消失；distilling 起
     馏出液滴串循环坠落 + 接液瓶内馏出液逐滴生长；finishing 火焰熄灭。
  3. 火焰逐帧抖动（模拟火焰跳动）。
"""
import math
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
)


class _HeldLifecycle:
    """纯平移持握状态机（火柴通用）：rest → attached → released → rest。

    持握 = 纯平移 offset（held_offset）：火柴全程水平横躺，不随夹爪旋转（杆横躺，夹爪
    手指朝下竖直夹杆身）。释放时写回台面静止位。
    """

    def __init__(self, task, name, path, rest, grasp, held_offset):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp = np.array(grasp)
        self.held_offset = np.array(held_offset)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.task._set_obj_world(self.path, self.rest)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos, dtype=float) + self.held_offset
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_obj_world(self.path, held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world(self.path, held)
                print(f"[d5] {self.name} attached (grip={opening:.4f})")
            return

        if self.state != "attached":
            return   # released：已放回，不再跟随

        # 吸附期：物体跟随夹爪（纯平移）
        held = np.asarray(gripper_pos, dtype=float) + self.held_offset
        self.task._set_obj_world(self.path, held)
        # 松爪：写回台面静止位，复位 rest
        if opening > self.task.gripper_open_threshold:
            self.released = True
            self.task._set_obj_world(self.path, self.rest)
            self.state = "rest"
            print(f"[d5] {self.name} released to rest")


class D5DistillationTask(BaseTask):
    """D5 蒸馏分离任务：预组装装置，机械臂仅点燃酒精灯，蒸馏现象由现象状态机驱动。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 火柴点火判定（照 B2/D9）
    MATCH_IGNITE_NEAR_FRAMES = 15
    MATCH_IGNITE_DIST = 0.035

    # 现象几何常量（照 gen_d5_scene.py）
    LAMP_X, LAMP_Y = 0.5286, 0.0029
    FLASK_BOTTOM_Z = 0.957
    SAMPLE_LIQ_TOP = 0.971               # 样品液面（0.965 + 0.012/2），气泡触此消失
    BUBBLE_RISE = 0.0004                 # 气泡每帧上升量（m）
    BUBBLE_WOBBLE = 0.0015               # 气泡水平抖动振幅
    BUBBLE_SPAWN_INTERVAL = 2.0          # vigor=1 时每 N 帧生成一个气泡
    N_BUBBLES = 30
    RECV_X, RECV_Y = 0.900, 0.0029
    RECV_BOTTOM_Z = 0.80
    DROP_HOME = (0.900, 0.0029, 0.907)   # 冷凝管出口下方 1cm（液滴生成点）
    N_DROPS = 8
    DRIP_INTERVAL = 30                   # 每 N 帧落一滴
    DROP_FALL = 30                       # 液滴坠落帧数（加速下落）
    RECV_LEVEL_STEP = 0.006              # 每滴馏出液高度增量
    RECV_TARGET = 0.042                  # 收集完成液面高（7 滴）

    LAMP = "/World/AlcoholLamp"
    MATCH = "/World/Match"
    FLASK_BUBBLES = "/World/FlaskBubbles"
    DISTILLATE_DROP = "/World/DistillateDrop"
    RECV_LIQUID = "/World/ReceivingLiquid"

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 火柴吸附期逐帧传送 + 手指闭合会被物理干扰 → 关碰撞（灯/装置静态不碰，无需关）
        self._disable_collision(self.MATCH)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 相态时序（可经 config 覆盖）
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.heat_dwell_frames = int(getattr(cfg, "heat_dwell_frames", 180))
        self.boil_dwell_frames = int(getattr(cfg, "boil_dwell_frames", 180))
        self.result_hold_frames = int(getattr(cfg, "result_hold_frames", 90))
        self.flame_fade_frames = int(getattr(cfg, "flame_fade_frames", 45))

        # 火焰 prim（/World 顶层，照 B2/D9 _flame_paths）
        self.flame_prims = self._flame_paths()
        self.flame_base = [np.array(self._read_translate(p)) for p in self.flame_prims]

        # 气泡 prim（/World/FlaskBubbles/bubble_0..29，Sphere，初始隐藏）
        self.bubble_prims = [f"{self.FLASK_BUBBLES}/bubble_{i}" for i in range(self.N_BUBBLES)]
        self.bubble_base = [np.array(self._read_translate(p)) for p in self.bubble_prims]

        # 液滴 prim（/World/DistillateDrop/Drop_0..7，Sphere，初始隐藏）
        self.drop_prims = [f"{self.DISTILLATE_DROP}/Drop_{i}" for i in range(self.N_DROPS)]

        # 火柴生命周期（纯平移持握）
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _HeldLifecycle(self, "match", self.MATCH, match_rest,
                                    MATCH_GRASP, MATCH_HELD_OFFSET)

        # 现象状态
        self.phase = "idle"              # idle→ignited→heating→boiling→distilling→finishing→done
        self._phase_frames = 0
        self.vigor = 0.0                 # 沸腾强度 0..1（气泡生成速率）
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._bub_timer = 0.0
        self._rising = {}                # bubble idx -> 当前 z
        self._drop_timer = 0
        self._drop_state = ["idle"] * self.N_DROPS   # idle / falling
        self._drop_t = [0] * self.N_DROPS
        self._drop_from = [None] * self.N_DROPS
        self._drop_to = [None] * self.N_DROPS
        self.recv_level = 0.0

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.phase = "idle"
        self._phase_frames = 0
        self.vigor = 0.0
        self.flame_lit = False
        self.match_ignite_counter = 0
        self._bub_timer = 0.0
        self._rising = {}
        self._drop_timer = 0
        self._drop_state = ["idle"] * self.N_DROPS
        self._drop_t = [0] * self.N_DROPS
        self.recv_level = 0.0
        # 火焰/气泡/液滴/馏出液全隐 + 基准位复位
        self._set_visible(self.flame_prims, False)
        for p, base in zip(self.flame_prims, self.flame_base):
            self._set_translate(p, base)
        self._set_visible(self.bubble_prims, False)
        for p, base in zip(self.bubble_prims, self.bubble_base):
            self._set_translate(p, base)
        self._set_visible(self.DISTILLATE_DROP, False)   # 液滴父组隐藏
        self._set_visible(self.drop_prims, False)
        for p in self.drop_prims:
            self._set_translate(p, self.DROP_HOME)
        self._update_recv_liquid()
        # 火柴归位
        self.match.reset()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self.match.step(gripper_pos, opening)
        self._step_match_ignite(gripper_pos)
        self._step_phenomenon()
        self._update_effects()
        return self.get_basic_state_info(additional_info={
            "phase": self.phase,
            "flame_lit": self.flame_lit,
            "match_attached": self.match.attached,
            "recv_level": self.recv_level,
        })

    def on_task_complete(self, success):
        print(f"[d5] episode done success={success} phase={self.phase} "
              f"flame_lit={self.flame_lit} recv_level={self.recv_level:.3f}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 现象状态机
    # ------------------------------------------------------------------
    def _step_match_ignite(self, gripper_pos):
        """点火检测（照 B2/D9）：火柴 attached 期间头近灯芯连续帧 → flame_lit。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    print(f"[d5] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    def _step_phenomenon(self):
        """蒸馏相态机：ignited→heating→boiling→distilling→finishing→done。

        火焰点亮后按帧计数推进；heating 气泡渐起、boiling 气泡全开、distilling 液滴收集、
        finishing 收集完成后火焰熄灭、done 上报成功。
        """
        if self.phase == "idle":
            if self.flame_lit:
                self.phase = "ignited"
                self._phase_frames = 0
        elif self.phase == "ignited":
            self._phase_frames += 1
            if self._phase_frames >= self.ignite_dwell_frames:
                self.phase = "heating"
                self._phase_frames = 0
        elif self.phase == "heating":
            self._phase_frames += 1
            self.vigor = min(1.0, self._phase_frames / max(1, self.heat_dwell_frames))
            if self._phase_frames >= self.heat_dwell_frames:
                self.phase = "boiling"
                self._phase_frames = 0
        elif self.phase == "boiling":
            self._phase_frames += 1
            self.vigor = 1.0
            if self._phase_frames >= self.boil_dwell_frames:
                self.phase = "distilling"
                self._phase_frames = 0
                self._set_visible(self.DISTILLATE_DROP, True)   # 液滴父组显示（子逐滴 toggle）
        elif self.phase == "distilling":
            self.vigor = 1.0
            self._step_drip()
            if self.recv_level >= self.RECV_TARGET:
                self.phase = "finishing"
                self._phase_frames = 0
        elif self.phase == "finishing":
            self._phase_frames += 1
            self.vigor = 0.0
            if self._phase_frames >= self.result_hold_frames:
                self.flame_lit = False   # 收集完成 → 熄灭酒精灯
            if self._phase_frames >= self.result_hold_frames + self.flame_fade_frames:
                self.phase = "done"
        elif self.phase == "done":
            self.vigor = 0.0

        # 气泡动画（沸腾强度 > 0 时逐帧推进；火焰熄灭后 vigor=0 停止）
        self._step_bubbles()

    def _step_bubbles(self):
        """沸腾气泡：按 vigor 速率生成 → 上升 + 水平抖动 → 触样品液面消失。"""
        if self.vigor <= 0:
            return
        t = float(self.frame_idx)
        self._bub_timer += self.vigor
        while self._bub_timer >= self.BUBBLE_SPAWN_INTERVAL:
            self._bub_timer -= self.BUBBLE_SPAWN_INTERVAL
            idle = [i for i in range(self.N_BUBBLES) if i not in self._rising]
            if idle:
                i = idle[0]
                self._rising[i] = float(self.bubble_base[i][2])
                self._set_visible(self.bubble_prims[i], True)
        for i in list(self._rising.keys()):
            z = self._rising[i] + self.BUBBLE_RISE
            base = self.bubble_base[i]
            if z >= self.SAMPLE_LIQ_TOP:
                self._set_visible(self.bubble_prims[i], False)
                del self._rising[i]
            else:
                self._rising[i] = z
                jx = self.BUBBLE_WOBBLE * math.sin(t * 0.5 + i * 1.7)
                jy = self.BUBBLE_WOBBLE * math.cos(t * 0.4 + i * 2.3)
                self._set_translate(self.bubble_prims[i],
                                    (base[0] + jx, base[1] + jy, z))

    def _step_drip(self):
        """馏出液滴：每 DRIP_INTERVAL 帧坠一滴，加速下坠入接液瓶 → 馏出液逐滴生长。"""
        self._drop_timer += 1
        if self._drop_timer >= self.DRIP_INTERVAL and self.recv_level < self.RECV_TARGET:
            self._drop_timer = 0
            idle = [i for i in range(self.N_DROPS) if self._drop_state[i] == "idle"]
            if idle:
                i = idle[0]
                self._drop_state[i] = "falling"
                self._drop_t[i] = 0
                self._drop_from[i] = np.array(self.DROP_HOME, dtype=float)
                self._drop_to[i] = np.array(
                    [self.RECV_X, self.RECV_Y, self.RECV_BOTTOM_Z + self.recv_level + 0.005],
                    dtype=float)
                self._set_visible(self.drop_prims[i], True)
        for i in range(self.N_DROPS):
            if self._drop_state[i] != "falling":
                continue
            self._drop_t[i] += 1
            if self._drop_t[i] >= self.DROP_FALL:
                self.recv_level = min(self.RECV_TARGET, self.recv_level + self.RECV_LEVEL_STEP)
                self._set_visible(self.drop_prims[i], False)
                self._drop_state[i] = "idle"
                self._update_recv_liquid()
            else:
                frac = self._drop_t[i] / self.DROP_FALL
                p = self._drop_from[i] + (self._drop_to[i] - self._drop_from[i]) * (frac * frac)
                self._set_translate(self.drop_prims[i], p)

    def _update_recv_liquid(self):
        """接液瓶内馏出液柱：按 recv_level 设高度 + 中心，h<=0 隐藏。"""
        h = self.recv_level
        prim = self.stage.GetPrimAtPath(self.RECV_LIQUID)
        if not prim.IsValid():
            return
        if h <= 0:
            self._set_visible(self.RECV_LIQUID, False)
            return
        self._set_visible(self.RECV_LIQUID, True)
        UsdGeom.Cylinder(prim).GetHeightAttr().Set(float(h))
        self._set_translate(self.RECV_LIQUID,
                            (self.RECV_X, self.RECV_Y, self.RECV_BOTTOM_Z + h / 2.0))

    def _update_effects(self):
        """酒精灯火焰：点火后逐帧抖动位置。"""
        self._set_visible(self.flame_prims, self.flame_lit)
        if self.flame_lit:
            t = float(self.frame_idx)
            for path, base in zip(self.flame_prims, self.flame_base):
                jx = 0.0012 * math.sin(t * 0.6 + base[0] * 7.0)
                jz = 0.0010 * math.sin(t * 0.8 + base[2] * 5.0)
                self._set_translate(path, np.array(base) + np.array([jx, 0.0, jz]))

    # ------------------------------------------------------------------
    # 持握 / 辅助
    # ------------------------------------------------------------------
    def _get_obj_world(self, path):
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            self.object_utils.set_object_position(path, np.asarray(position, dtype=float))

    def _ease_obj_world(self, path, target, k=0.18):
        cur = self._get_obj_world(path)
        if cur is None:
            return
        self._set_obj_world(path, cur + (np.asarray(target, dtype=float) - cur) * k)

    def _near(self, pos, gripper_pos, z_thresh=0.02):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _flame_paths(self):
        return ["/World/flame_outer", "/World/flame_outer_sphere",
                "/World/flame_inner", "/World/flame_inner_sphere"]

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
