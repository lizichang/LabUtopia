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
    FLAME_GRPS, FLAME_PRIMS,
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

    # 现象几何常量（2026-09-03 gen_d5_final.py 实测：忠实用户 tmp 组装件）
    LAMP_X, LAMP_Y = 0.6981, 0.4610
    FLASK_BOTTOM_Z = 0.9091
    SAMPLE_LIQ_TOP = 0.9231               # 样品液面（气泡触此消失）
    BUBBLE_RISE = 0.0004                 # 气泡每帧上升量（m）
    BUBBLE_WOBBLE = 0.0015               # 气泡水平抖动振幅
    BUBBLE_SPAWN_INTERVAL = 2.0          # vigor=1 时每 N 帧生成一个气泡
    N_BUBBLES = 30
    RECV_X, RECV_Y = 0.7051, 0.0787
    RECV_BOTTOM_Z = 0.80
    DROP_HOME = (0.6991, 0.0759, 0.8372)   # 牛角管下尖(0.8432)下方 6mm（液滴生成点）
    N_DROPS = 8
    DRIP_INTERVAL = 30                   # 每 N 帧落一滴
    DROP_FALL = 30                       # 液滴坠落帧数（加速下落）
    RECV_LEVEL_STEP = 0.006              # 每滴馏出液高度增量
    RECV_TARGET = 0.042                  # 收集完成液面高（7 滴，顶 0.842 < 牛角管尖 0.8432）

    # 温度计红柱（用户 2026-09-03：开机不该就 100°C，红柱应在点燃酒精灯后缓慢上升）。
    # 红毛细管 = 整高静态 mesh（局部 z[0.006,0.2533] 底→110°C），gen 已给 Xform 加
    # (translate,scale) 两 op 压到室温；task 每帧按温度重写两 op（scale.z=f、translate.z=
    # z0(1-f) 绕底钉 pivot）让红柱随温升自下而上顶起。刻度线性实测：0° 刻线在红底上
    # 52.4mm、每 °C 升 1.765mm（tmp 刻度 major_0..100 @z 1.0064..1.1833，per-20°≈35.3mm）。
    ROOM_TEMP = 25.0                     # 起始室温（°C）
    BOIL_TEMP = 100.0                    # 水沸点（°C，沸腾后恒温）
    RED_OFF_TO_0MARK = 0.0524            # 红底到 0° 刻线距离 (m)
    RED_M_PER_DEG = 0.001765             # 每 °C 红柱上升 (m)
    RED_XFORM = "/World/Thermometer_001/Thermometer/capillary_liquid_001"

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

        # 火焰 = B5 水滴形组（/World/flame_<outer|inner>_grp，gen 建，pivot=火焰底）。
        #   叶子 prim（球+锥，组内局部坐标）只做可见性 toggle；组本身每帧 flicker（scale/rotate），
        #   见 _step_flame_anim。组 translate 固定 = gen 实测基准（复位时写回，勿挪）。
        self.flame_prims = self._flame_paths()
        self.flame_grps = list(FLAME_GRPS)
        self.flame_grp_base = [np.array(self._read_translate(p)) for p in self.flame_grps]

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

        # 温度计：读红毛细管局部 extent（底 z0 / 全长 L），起始温度 = 室温
        self._red_ok = False
        self._read_red_geom()
        self.temp_c = self.ROOM_TEMP

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
        self.temp_c = self.ROOM_TEMP           # 温度计红柱回室温
        # 火焰组复位：隐叶子 + 组 translate/scale/rotate 归位（防上轮 flicker 残留）
        self._set_visible(self.flame_prims, False)
        for p, base in zip(self.flame_grps, self.flame_grp_base):
            self._reset_flame_grp(p, base)
        self._set_visible(self.bubble_prims, False)
        for p, base in zip(self.bubble_prims, self.bubble_base):
            self._set_translate(p, base)
        self._set_visible(self.DISTILLATE_DROP, False)   # 液滴父组隐藏
        self._set_visible(self.drop_prims, False)
        for p in self.drop_prims:
            self._set_translate(p, self.DROP_HOME)
        self._update_recv_liquid()
        self._apply_red_level()                # 红柱写回室温液位
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
        self._apply_red_level()          # 红柱随温度逐帧升（室温起步）
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

        # 温度模型（红柱跟随）：点燃后在 ignited/heating 线性升温，ramp = 点燃停留 + 加热帧
        # 数，到沸腾相恰达 100°C 后恒温（水沸点，液体/蒸气温度不再升）；收集完成火焰熄仍 100。
        if self.phase in ("boiling", "distilling", "finishing"):
            self.temp_c = self.BOIL_TEMP
        elif self.flame_lit and self.phase in ("ignited", "heating"):
            ramp = max(1, self.ignite_dwell_frames + self.heat_dwell_frames)
            self.temp_c = min(self.BOIL_TEMP,
                              self.temp_c + (self.BOIL_TEMP - self.ROOM_TEMP) / ramp)

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

    def _read_red_geom(self):
        """读温度计红毛细管 mesh 局部 extent：底 z0 / 全长 L（gen_d5_final 同源读法）。
        gen 已给 Xform 加 (translate,scale) op，绕 z0 缩放；z0/L 只作底 pivot 与比例标定。"""
        prim = self.stage.GetPrimAtPath(self.RED_XFORM)
        if not prim.IsValid():
            self._red_ok = False
            return
        self._red_ok = True
        self._red_z0 = 0.006        # 兜底：tmp 实测局部底（gen 同值）
        self._red_len = 0.2473      # 兜底：局部全长（底→110°C）
        for c in prim.GetChildren():
            if c.GetTypeName() == "Mesh":
                g = UsdGeom.Gprim(c)
                ext = g.GetExtentAttr().Get() if g.GetExtentAttr() else None
                if ext and len(ext) == 2:
                    self._red_z0 = float(ext[0][2])
                    self._red_len = float(ext[1][2] - ext[0][2])
                break

    def _red_frac(self):
        """温度 → 红柱高比例 f（红底上 0° 刻线 52.4mm、每 °C 升 1.765mm 线性实测）。
        室温 25°C≈0.39、沸点 100°C≈0.93、110°C 刻度=1.0（封顶）。"""
        off = self.RED_OFF_TO_0MARK + self.temp_c * self.RED_M_PER_DEG
        f = off / max(1e-9, self._red_len)
        return float(max(0.0, min(1.0, f)))

    def _apply_red_level(self):
        """把红柱 Xform 的 scale.z=f / translate.z=z0(1-f) 写成当前温度对应液位。
        op 序 (translate,scale)：矩阵=T·S → 底 z0 钉住不动、柱顶从下往上顶到温度刻线。"""
        if not getattr(self, "_red_ok", False):
            return
        prim = self.stage.GetPrimAtPath(self.RED_XFORM)
        if not prim.IsValid():
            return
        f = self._red_frac()
        z0 = self._red_z0
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            ot = op.GetOpType()
            if ot == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(1.0, 1.0, f))
            elif ot == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(0.0, 0.0, z0 * (1.0 - f)))

    def _update_effects(self):
        """酒精灯火焰：点火 reveal（叶子 visible）→ B5 水滴组每帧 flicker（scale 高/宽+侧摆）。

        用户「火焰应该像 B5 一样动起来」：只挪 translate 的散装火焰无法缩放焰体，须对组写
        scale/rotateXYZ。叶子 prim 只切可见性，形状动画全在组上（_step_flame_anim）。
        """
        self._set_visible(self.flame_prims, self.flame_lit)
        if self.flame_lit:
            self._step_flame_anim()

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
        """4 个火焰叶子 prim（外/内焰各 球+锥，组内局部坐标，仅可见性用）。"""
        return list(FLAME_PRIMS)

    def _reset_flame_grp(self, grp_path, base):
        """复位火焰组：translate 回 gen 基准，scale/rotate 归位（清上轮 flicker 残留）。"""
        prim = self.stage.GetPrimAtPath(grp_path)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            ot = op.GetOpType()
            if ot == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
            elif ot == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            elif ot == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*base))

    def _smooth_noise(self, t, seed):
        """确定性平滑噪声（每 3 帧一个随机值，相邻 smoothstep 插值）。火焰 flicker 用，
        同输入同输出（无随机抖动，录像可复现）。照 B5/C4（修过 20Hz 爆闪 bug）。"""
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
        """对火焰组每帧 flicker：scale(高/宽) + rotateXYZ(侧摆)，pivot=组原点=火焰底
        （gen 组 op 序 translate→rotate→scale：先 scale 后 rotate 再 translate = 绕底不漂移）。
        base_pos 给则同时写 translate（D5 酒精灯火焰固定，恒 None 不动 translate）。"""
        prim = self.stage.GetPrimAtPath(grp_path)
        if not prim.IsValid():
            return
        t = float(self.frame_idx)
        h = 1.0 + 0.13 * self._smooth_noise(t, seed) + 0.06 * math.sin(t * 0.35 + seed)
        w = 1.0 + 0.10 * self._smooth_noise(t, seed + 7) + 0.04 * math.sin(t * 0.53 + seed)
        lean = 7.0 * self._smooth_noise(t, seed + 13)      # 侧摆度数（±~7°）
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            ot = op.GetOpType()
            if ot == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(w, w, h))
            elif ot == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3f(lean, 0.0, 0.0))
            elif base_pos is not None and ot == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*base_pos))

    def _step_flame_anim(self):
        """酒精灯火焰每帧 flicker（点燃后，B5 同款：外焰 seed1 / 内焰 seed2）。"""
        if not self.flame_lit:
            return
        self._apply_flame_flicker(FLAME_GRPS[0], None, seed=1)
        self._apply_flame_flicker(FLAME_GRPS[1], None, seed=2)

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
