"""D9 氧气检验任务：带火星木条悬停氧气试管口上方，氧气溢出使余烬复燃。

原理：带余烬（火星）的木条悬在竖立氧气试管口上方（不伸进去），氧气扩散上溢接触余烬使
木条**复燃**（火星 → 明火），据此判定气体是氧气。

用户逐字（2026-09-01）动作链：「摘开酒精灯帽 → 拿火柴点燃酒精灯 → 拿木条点燃 → 快速摆动
机械臂让它熄灭（甩灭明火留余烬）→ 火星现象 → 悬停氧气试管口上方（不伸进去）→ 复燃 → 取出
归位 → 盖灯帽（熄灯）」。

本 task 做 4 类驱动（与 d7/b2 同构：纯平移持握 + visibility 切换 + 火焰/余烬逐帧钉木条端）：
  1. 灯帽生命周期：rest（在灯上）→ attached（夹帽）→ released（放桌面 CAP_REST）。帽是灯子
     prim，纯平移持握（帽中心 = 夹爪 + CAP_HELD_OFFSET），经 _set_cap_world 换算成帽相对灯的
     local translate（照 B2 盖帽同款，仅方向相反：摘帽而非盖帽）。
  2. 火柴生命周期：rest → attached → released（照 B2 _MatchLifecycle，纯平移持握），
     头近灯芯 → flame_lit（酒精灯火焰 reveal）。
  3. 木条生命周期：rest → attached → released（照 B2 _MatchLifecycle，纯平移持握），
     点燃端 = 夹爪 + SPLINT_TIP_OFFSET。效果 prim（SplintChar 炭黑区 + 10 余烬火星点 +
     SplintFlame）逐帧钉到点燃端，火焰逐帧抖动、火星逐点错相微抖模拟「动」感。
  4. 现象状态机（读 cfg.oxygen_result）：
      点火（火柴触灯芯）→ flame_lit（灯焰显）
      木条端入灯焰 → splint_lit（复燃焰显，钉木条端）
      摆动熄火（夹爪近 SHAKE_GRIP）→ splint_lit 隐、splint_ember 显（余烬）
      悬停管口（端在管口上方 15mm）→ oxygen_result=reignite 时余烬隐 + 复燃焰显（复燃）；
        =negative 时余烬渐熄（无复燃）
  reset()：隐藏所有火焰/余烬 + 木条/火柴/帽归位。
"""
import math
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    CAP_HELD_OFFSET, CAP_ON_GRASP, CAP_REST, CAP_REST_GRASP, CAP_CLOSED_THRESHOLD,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    SPLINT_XY, SPLINT_REST_Z, SPLINT_GRASP, SPLINT_HELD_OFFSET, SPLINT_TIP_OFFSET,
    SPLINT_TIP, EMBER_N, EMBER_PREFIX, SPLINT_CHAR,
    FLAME_CENTER, SHAKE_GRIP, OXY_TUBE_XY, HOVER_TIP_Z,
)


class _HeldLifecycle:
    """纯平移持握状态机（火柴/木条通用）：rest → attached → released → rest。

    持握 = 纯平移 offset（held_offset）：物体全程水平横躺，不随夹爪旋转（火柴/木条杆横躺，
    夹爪手指朝下竖直夹杆身）。释放时写回台面静止位。
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
                print(f"[d9] {self.name} attached (grip={opening:.4f})")
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
            print(f"[d9] {self.name} released to rest")


class _CapLifecycle:
    """灯帽摘/盖状态机：rest → attached（夹帽）→ released（放目标位）。

    持握 = 纯平移 offset（CAP_HELD_OFFSET）：帽全程竖直开口朝下，不随夹爪旋转（照 B2 盖帽
    同款纯平移持握）。帽是灯子 prim，吸附期逐帧把帽写到夹爪持握位（帽中心 = 夹爪 +
    CAP_HELD_OFFSET，经 _set_cap_world 换算成帽相对灯的 local translate）。松爪后锁到
    release_world（None = 回灯上，用 _set_cap_rest 写回场景初值）。
    """

    def __init__(self, task, name, path, grasp, release_grasp, release_world=None,
                 reset_to_rest=True):
        self.task = task
        self.name = name
        self.path = path
        self.grasp = np.array(grasp)
        self.release_grasp = np.array(release_grasp)
        self.release_world = None if release_world is None else np.array(release_world)
        self.reset_to_rest = reset_to_rest
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        if self.reset_to_rest:
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
                print(f"[d9] {self.name} attached (grip={opening:.4f})")
            return

        if self.state != "attached":
            return   # released：已放目标位，不再跟随

        # 吸附期：帽跟随夹爪（纯平移），帽中心 = 夹爪 + CAP_HELD_OFFSET
        held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
        self.task._set_cap_world(held)
        # 松爪（到达释放夹点）：帽锁 release_world（None = 回灯上）
        if opening > self.task.gripper_open_threshold and self.task._near(self.release_grasp, gripper_pos):
            self.state = "released"
            self.released = True
            if self.release_world is None:
                self.task._set_cap_rest()
            else:
                self.task._set_cap_world(self.release_world)
            print(f"[d9] {self.name} released (grip={opening:.4f})")


class D9OxygenSplintTask(BaseTask):
    """D9 氧气检验任务：带火星木条悬停氧气试管口上方复燃。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 火柴点火判定（照 B2）
    MATCH_IGNITE_NEAR_FRAMES = 15
    MATCH_IGNITE_DIST = 0.035
    # 木条点燃判定（端近灯焰中心）
    SPLINT_IGNITE_NEAR_FRAMES = 10
    SPLINT_IGNITE_DIST = 0.02
    # 摆动熄火判定（夹爪近摆动中心，窗口 0.05 覆盖 shake 振幅 0.03）
    BLOWOUT_NEAR_FRAMES = 8
    BLOWOUT_WINDOW = 0.05
    # 悬停复燃判定（木条端近管口上方）
    HOVER_NEAR_FRAMES = 10
    HOVER_XY_DIST = 0.03

    LAMP = "/World/AlcoholLamp"
    CAP = "/World/AlcoholLamp/cap"
    MATCH = "/World/Match"
    SPLINT = "/World/WoodSplint"
    OXY_TUBE = "/World/OxygenTube"
    SPLINT_CHAR_PATH = SPLINT_CHAR
    SPLINT_EMBER_PATHS = [f"{EMBER_PREFIX}{i}" for i in range(EMBER_N)]
    SPLINT_FLAME = "/World/SplintFlame"
    SPLINT_FLAME_SPHERE = "/World/SplintFlame_sphere"
    SPLINT_FLAME_PATHS = [SPLINT_FLAME, SPLINT_FLAME_SPHERE]

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 碰撞：木条/火柴吸附期逐帧传送 + 手指闭合会被物理干扰 → 关碰撞；帽是灯子 prim，
        # 随灯一起关（灯静态不动，但帽要拔起，逐帧传送帽也需关碰撞）。
        self._disable_collision(self.MATCH)
        self._disable_collision(self.SPLINT)
        self._disable_collision(self.LAMP)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.cap_closed_threshold = CAP_CLOSED_THRESHOLD

        # 实验结果（main.py 已按 experiment_result schema 把 CLI/交互结果写回 cfg.oxygen_result）
        self.oxygen_result = str(getattr(cfg, "oxygen_result", "reignite")).strip().lower()
        if self.oxygen_result not in ("reignite", "negative"):
            self.oxygen_result = "reignite"

        # 火焰 prim（灯焰，/World 顶层；照 B2 _flame_paths）
        self.flame_prims = self._flame_paths()
        self.flame_base = [self._read_translate(p) for p in self.flame_prims]

        # 木条端效果 prim 基准位 + 相对木条端（SPLINT_TIP 静止位）的偏移。钉端时 = tip + offset。
        tip_rest = np.array(SPLINT_TIP, dtype=float)
        self.ember_base = [np.array(self._read_translate(p)) for p in self.SPLINT_EMBER_PATHS]
        self.char_base = np.array(self._read_translate(self.SPLINT_CHAR_PATH))
        self.splint_flame_sphere_base = np.array(self._read_translate(self.SPLINT_FLAME_SPHERE))
        self.splint_flame_base = np.array(self._read_translate(self.SPLINT_FLAME))
        self._ember_offsets = [b - tip_rest for b in self.ember_base]
        self._char_offset = self.char_base - tip_rest
        self._sflame_sphere_offset = self.splint_flame_sphere_base - tip_rest
        self._sflame_offset = self.splint_flame_base - tip_rest

        # 生命周周期
        self.cap_rest_translate = self._read_translate(self.CAP)
        self.cap = _CapLifecycle(self, "cap_off", self.CAP, CAP_ON_GRASP, CAP_REST_GRASP,
                                 release_world=CAP_REST, reset_to_rest=True)
        self.cap_on = _CapLifecycle(self, "cap_on", self.CAP, CAP_REST_GRASP, CAP_ON_GRASP,
                                    release_world=None, reset_to_rest=False)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _HeldLifecycle(self, "match", self.MATCH, match_rest,
                                    MATCH_GRASP, MATCH_HELD_OFFSET)
        splint_rest = (SPLINT_XY[0], SPLINT_XY[1], SPLINT_REST_Z)
        self.splint = _HeldLifecycle(self, "splint", self.SPLINT, splint_rest,
                                     SPLINT_GRASP, SPLINT_HELD_OFFSET)

        # 现象状态
        self.flame_lit = False        # 酒精灯火焰（火柴点燃）
        self.splint_lit = False       # 木条端明火（点燃/复燃）
        self.splint_ember = False     # 木条端余烬火星（甩灭后）
        self.splint_charred = False   # 木条端已炭化（首次点燃即永久真，炭黑区甩灭/复燃熄后仍保留）
        self.reignited = False        # 复燃结果（oxygen_result=reignite 且悬停到位）
        self.result_settled = False   # 悬停判定已定（防重复触发）
        self.match_ignite_counter = 0
        self.splint_ignite_counter = 0
        self.blowout_counter = 0
        self.hover_counter = 0

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.flame_lit = False
        self.splint_lit = False
        self.splint_ember = False
        self.splint_charred = False
        self.reignited = False
        self.result_settled = False
        self.match_ignite_counter = 0
        self.splint_ignite_counter = 0
        self.blowout_counter = 0
        self.hover_counter = 0
        # 火焰/余烬/炭黑全隐 + 基准位复位
        self._set_visible(self._flame_paths(), False)
        self._set_visible(self.SPLINT_FLAME_PATHS, False)
        self._set_visible(self.SPLINT_EMBER_PATHS, False)
        self._set_visible(self.SPLINT_CHAR_PATH, False)
        for p, base in zip(self.flame_prims, self.flame_base):
            self._set_translate(p, base)
        for p, base in zip(self.SPLINT_EMBER_PATHS, self.ember_base):
            self._set_translate(p, base)
        self._set_translate(self.SPLINT_CHAR_PATH, self.char_base)
        self._set_translate(self.SPLINT_FLAME_SPHERE, self.splint_flame_sphere_base)
        self._set_translate(self.SPLINT_FLAME, self.splint_flame_base)
        # 生命周期复位
        self.cap.reset()
        self.cap_on.reset()
        self.match.reset()
        self.splint.reset()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self.cap.step(gripper_pos, opening)
        # cap_on（盖帽）只在 cap_off 已放帽到桌面后才推进——否则摘帽下探桌面时（夹爪闭合
        # 0.0185<0.022、恰在 CAP_REST_GRASP）cap_on 会提前 attach 并逐帧把帽钉回夹爪，
        # 覆盖 cap_off 的 release（"帽一直跟爪子"根因）。
        if self.cap.released:
            self.cap_on.step(gripper_pos, opening)
        self.match.step(gripper_pos, opening)
        self.splint.step(gripper_pos, opening)
        if self.cap_on.released and self.flame_lit:
            self.flame_lit = False
        self._step_match_ignite(gripper_pos)
        self._step_splint_ignite(gripper_pos)
        self._step_blowout(gripper_pos)
        self._step_reignite(gripper_pos)
        self._update_effects(gripper_pos)
        return self.get_basic_state_info(additional_info={
            "flame_lit": self.flame_lit,
            "cap_removed": self.cap.released,
            "cap_on_released": self.cap_on.released,
            "match_attached": self.match.attached,
            "splint_attached": self.splint.attached,
            "splint_lit": self.splint_lit,
            "splint_ember": self.splint_ember,
            "splint_charred": self.splint_charred,
            "reignited": self.reignited,
            "splint_released": self.splint.released,
        })

    def on_task_complete(self, success):
        print(f"[d9] episode done success={success} "
              f"flame_lit={self.flame_lit} cap_removed={self.cap.released} "
              f"splint_lit={self.splint_lit} splint_ember={self.splint_ember} "
              f"reignited={self.reignited} splint_released={self.splint.released} "
              f"oxygen_result={self.oxygen_result}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 现象状态机
    # ------------------------------------------------------------------
    def _step_match_ignite(self, gripper_pos):
        """点火检测（照 B2）：火柴 attached 期间头近灯芯连续帧 → flame_lit。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    print(f"[d9] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    def _step_splint_ignite(self, gripper_pos):
        """木条端伸入灯焰 → splint_lit（明火显，钉木条端）。"""
        if self.splint_lit or self.splint_ember:
            return
        if self.splint.attached and self.flame_lit:
            tip = self._splint_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(FLAME_CENTER)) < self.SPLINT_IGNITE_DIST:
                self.splint_ignite_counter += 1
                if self.splint_ignite_counter >= self.SPLINT_IGNITE_NEAR_FRAMES:
                    self.splint_lit = True
                    self.splint_charred = True   # 首次点燃即炭化，此后炭黑区永久保留（不复原）
                    print(f"[d9] splint lit @ frame {self.frame_idx}")
            else:
                self.splint_ignite_counter = 0
        else:
            self.splint_ignite_counter = 0

    def _step_blowout(self, gripper_pos):
        """摆动熄火：明火灭（夹爪近摆动中心连续帧）。

        首次甩灭（点燃后）留余烬供悬停复燃；复燃后放回前二次甩灭则彻底熄灭（无余烬）。
        炭黑区由 splint_charred（首次点燃即永久真）驱动，与明火/余烬无关，甩灭后仍保留。
        """
        if not self.splint_lit:
            return
        if self.splint.attached:
            gp = np.asarray(gripper_pos, dtype=float)
            near = (np.linalg.norm(gp[:2] - np.array(SHAKE_GRIP[:2])) < self.BLOWOUT_WINDOW
                    and abs(gp[2] - SHAKE_GRIP[2]) < self.BLOWOUT_WINDOW)
            if near:
                self.blowout_counter += 1
                if self.blowout_counter >= self.BLOWOUT_NEAR_FRAMES:
                    self.splint_lit = False
                    if self.reignited:
                        self.splint_ember = False
                        print(f"[d9] splint extinguished after reignite @ frame {self.frame_idx}")
                    else:
                        self.splint_ember = True
                        print(f"[d9] splint blown out -> ember @ frame {self.frame_idx}")
            else:
                self.blowout_counter = 0
        else:
            self.blowout_counter = 0

    def _step_reignite(self, gripper_pos):
        """悬停管口：氧气结果判定（复燃 / 不复燃）。"""
        if not self.splint_ember or self.result_settled:
            return
        if self.splint.attached:
            tip = self._splint_tip(gripper_pos)
            hovered = (np.linalg.norm(tip[:2] - np.array(OXY_TUBE_XY)) < self.HOVER_XY_DIST
                       and abs(tip[2] - HOVER_TIP_Z) < 0.02)
            if hovered:
                self.hover_counter += 1
                if self.hover_counter >= self.HOVER_NEAR_FRAMES:
                    self.result_settled = True
                    if self.oxygen_result == "reignite":
                        self.splint_ember = False
                        self.splint_lit = True
                        self.reignited = True
                        print(f"[d9] splint REIGNITED by oxygen @ frame {self.frame_idx}")
                    else:
                        self.splint_ember = False
                        print(f"[d9] splint ember faded (no reignition) @ frame {self.frame_idx}")
            else:
                self.hover_counter = 0
        else:
            self.hover_counter = 0

    def _update_effects(self, gripper_pos):
        """每帧钉效果 prim 到木条端 + 切换 visibility + 火焰/火星「动」感抖动。

        木条端世界坐标 = 夹爪 + SPLINT_TIP_OFFSET（attached 时夹爪已把木条钉持握位）。
        splint_lit/splint_ember 仅在 attached 后为真，故 rest 期（未持握）effect 全隐、
        钉位无视觉影响。火焰逐帧加小幅正弦抖动（模拟火焰跳动），余烬火星点逐点错相微抖
        （模拟火星闪烁）。
        """
        t = float(self.frame_idx)
        tip = np.asarray(gripper_pos, dtype=float) + np.array(SPLINT_TIP_OFFSET)
        # 木条是否还在夹爪手上：放回（released）后余烬/明火全隐（它们到 release 时必已熄灭）；
        # 炭黑区例外——永久炭化，放回后仍钉在放回的木条端（用 _splint_tip_actual 读木条自身位，
        # 而非夹爪位，避免"放回后炭黑悬空跟机械臂走"）。
        held = self.splint.state == "attached"

        # 酒精灯火焰：点火后逐帧抖动位置
        self._set_visible(self._flame_paths(), self.flame_lit)
        if self.flame_lit:
            for path, base in zip(self.flame_prims, self.flame_base):
                jx = 0.0012 * math.sin(t * 0.6 + base[0] * 7.0)
                jz = 0.0010 * math.sin(t * 0.8 + base[2] * 5.0)
                self._set_translate(path, np.array(base) + np.array([jx, 0.0, jz]))

        # 炭黑区：首次点燃即永久炭化，放回后仍钉在放回的木条端（不悬空跟臂）。真实端坐标
        # 用 _splint_tip_actual()（木条自身世界位），故放回桌面后炭黑停在静止端。
        show_char = self.splint_charred
        self._set_visible(self.SPLINT_CHAR_PATH, show_char)
        if show_char:
            atip = self._splint_tip_actual()
            if atip is not None:
                self._set_translate(self.SPLINT_CHAR_PATH, atip + self._char_offset)

        # 余烬火星点：仅持握时显示 + 逐点错相微抖（火星闪烁感）
        show_ember = held and self.splint_ember
        self._set_visible(self.SPLINT_EMBER_PATHS, show_ember)
        if show_ember:
            for i, (path, off) in enumerate(zip(self.SPLINT_EMBER_PATHS, self._ember_offsets)):
                jx = 0.0008 * math.sin(t * 0.9 + i * 2.1)
                jz = 0.0008 * math.sin(t * 1.1 + i * 1.7)
                self._set_visible(path, True)
                self._set_translate(path, tip + off + np.array([jx, 0.0, jz]))

        # 木条端明火（点燃/复燃）：仅持握时显示 + 抖动位置
        show_flame = held and self.splint_lit
        self._set_visible(self.SPLINT_FLAME_PATHS, show_flame)
        if show_flame:
            jx = 0.0012 * math.sin(t * 0.7)
            jz = 0.0012 * math.sin(t * 0.9 + 1.3)
            self._set_translate(self.SPLINT_FLAME_SPHERE,
                                tip + self._sflame_sphere_offset + np.array([jx, 0.0, jz]))
            self._set_translate(self.SPLINT_FLAME,
                                tip + self._sflame_offset + np.array([jx, 0.0, jz]))

    def _splint_tip(self, gripper_pos):
        """木条点燃端中心世界坐标 = 夹爪 + SPLINT_TIP_OFFSET（端在夹爪 +X 0.11，水平朝前）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(SPLINT_TIP_OFFSET)

    def _splint_tip_actual(self):
        """木条点燃端**真实**世界坐标 = 木条自身 world translate + 局部 tip 偏移（+X 0.15）。

        与 _splint_tip(gripper_pos) 不同：这里读木条**自身**世界位，故放回桌面后（木条写回
        静止位）炭黑区仍钉在放回的木条端，不悬空跟机械臂走（旧 bug 根因 = 放回后仍用夹爪位钉）。
        """
        origin = self._get_obj_world(self.SPLINT)
        if origin is None:
            return None
        tip_local = (SPLINT_TIP[0] - SPLINT_XY[0], SPLINT_TIP[1] - SPLINT_XY[1],
                     SPLINT_TIP[2] - SPLINT_REST_Z)
        return np.asarray(origin, dtype=float) + np.array(tip_local)

    # ------------------------------------------------------------------
    # 持握 / 判定
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

    # ------------------------------------------------------------------
    # 灯帽持握（照 B2：纯平移持握，帽中心 = 夹爪 + CAP_HELD_OFFSET，帽是灯子 prim）
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
        """把帽写到给定帽中心世界坐标（帽 = 灯子 prim，只写帽 translate op，保留
        rotateXYZ+scale 形状）。换算（照 B2，灯 R180Z）：
        cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+CAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−CAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world(self.LAMP)
        if lamp_pos is None:
            return
        cx, cy, cz = center
        tx = lamp_pos[0] - cx
        ty = lamp_pos[1] - cy
        tz = cz - lamp_pos[2] - 0.0915  # CAP_CENTER_DZ（帽中心到灯底座 z 偏移）
        self._set_cap_translate((tx, ty, tz))

    def _set_cap_rest(self):
        """帽回灯上（写回读自场景的帽 local translate；D9 帽起始在灯上 = (0,0,0)）。"""
        if self.cap_rest_translate is not None:
            self._set_cap_translate(self.cap_rest_translate)

    def _ease_cap_world(self, target, k=0.18):
        """夹爪合拢期间帽逐帧平滑移向持握位（消除闪现吸附）。"""
        cur_origin = self._get_obj_world(self.CAP)
        if cur_origin is None:
            return
        cur_center = np.asarray(cur_origin, dtype=float) + np.array([0.0, 0.0, 0.0915])
        nxt = cur_center + (np.asarray(target, dtype=float) - cur_center) * k
        self._set_cap_world(nxt)

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _flame_paths(self):
        # 火焰迁到 /World 顶层（gen rebuild_flames 水滴形 = 底半球 Sphere + 上部 Cone，
        # 每焰两 prim）。默认可见，任务 reset 熄、点着翻 visible。
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
