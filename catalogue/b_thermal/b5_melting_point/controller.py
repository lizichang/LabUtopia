# -*- coding: utf-8 -*-
"""B5 熔点测定（提勒管法）控制器：装样段 = 十四个元动作 + 加热/移灯/盖帽段（段2 相态驱动）。

用户 2026-09-03 逐字：「加热的时候要停留15s，这个期间机械臂应该夹住酒精灯在x方向上来回
移动，来控制升温速度，最后盖酒精灯帽前，应该先把酒精灯往-x移动5cm移出再盖酒精灯。夹酒精灯
参考B2、B3。」

两段式（段1 装样+挂温度计+点火 → 段2 加热/移灯/盖帽相态驱动，照 B2 控制器）：
  段1  十四个元动作：①夹封口端拎起 ②插粉丘蘸粉 ③放回 ④夹开口端拎起 ⑤震实 ⑥放回 ⑦中部
       夹起 ⑧蘸油 ⑧'放回 ⑨'再夹封口端 ⑩竖贴泡 ⑪抓温度计旋转竖直 ⑫插提勒管 ⑬火柴点燃
       酒精灯。LightFlamePass 完成 → task flame_lit → phase idle→ignited。
  段2  段1 完成后机械臂回显保持（夹爪停 GRIP_OPEN），等 task 相态机自行推 ignited（dwell
       ignite_dwell_frames 帧）→ heating（夹灯 X 摆动 15s 控温 + 移灯 -X 5cm + 油浴对流 +
       熔点相变）→ cap_lamp（盖帽熄火）→ done。phase=="heating" 时跑 LampHeatMovePass
       （水平横夹灯体宽处 X 摆动 15s → -X 移 5cm → 松爪）；phase=="cap_lamp" 时跑
       CapLampPass（取帽 → 灯口上方 → 下扣盖灭 → 松爪）；读到 phase=="done"（盖帽熄火 +
       结果停留帧）→ 上报成功并请求 reset。

与 b1/b2/d2s 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，段2 按 phase 惰性实例化加热/盖帽元动作。
"""
import os
import numpy as np
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.extensions import get_extension_path_from_name

from controllers.base_controller import BaseController as TaskBaseController
from controllers.atomic_actions.flametest import IkMotionEngine
from .meta_actions import (PickCapillarySealedEnd, DipCapillaryIntoPowder, ReturnCapillaryToTable,
                           PickCapillaryOpenEnd, TampCapillary, ReturnCapillaryAfterTamp,
                           PickCapillaryMiddle, DipCapillaryInOil, ReturnCapillaryAfterOil,
                           StickCapillaryToBulb, PickThermometer, InsertThermometerIntoTube,
                           LightFlamePass, LampHeatMovePass, CapLampPass)
from .meta_actions.constants import GRIP_OPEN


class B5MeltingPointTaskController(TaskBaseController):
    """Composite controller: 段 1 跑装样+点火元动作，段 2 等 task 相态推完（加热/移灯/盖帽）。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b5] controller VERSION v13 (段1 meta-actions: PickSealedEnd + Dip + ReturnToTable "
              "+ PickOpenEnd + Tamp + ReturnAfterTamp + PickMiddle + DipOil + ReturnAfterOil "
              "+ PickSealedEnd(2) + StickVertical + PickThermo + InsertThermo + LightFlame; "
              "段2 phase watch: heating -> LampHeatMovePass, cap_lamp -> CapLampPass)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3s/b1）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 段 1：装样五段 + 挂温度计 + 点火（十四个元动作，顺序同 v12）
        self.meta_classes = [PickCapillarySealedEnd, DipCapillaryIntoPowder, ReturnCapillaryToTable,
                             PickCapillaryOpenEnd, TampCapillary, ReturnCapillaryAfterTamp,
                             PickCapillaryMiddle, DipCapillaryInOil, ReturnCapillaryAfterOil,
                             PickCapillarySealedEnd, StickCapillaryToBulb,
                             PickThermometer, InsertThermometerIntoTube, LightFlamePass]
        self.meta_names = [
            "grab the sealed end of the capillary and lift it so it hangs vertical, open end down",
            "move above the powder mound and dip the open end vertically into the powder",
            "return the capillary tube to the table, laying it horizontal again",
            "grab the open end and lift it so it hangs vertical, sealed end down",
            "rapidly move the capillary up and down vertically 10 times to pack the powder to the sealed end",
            "return the tamped capillary to the table, laying it horizontal again",
            "grab the middle of the capillary, keeping it horizontal",
            "move above the oil dish and dip the sealed end into the paraffin oil",
            "return the oiled capillary to the table, laying it horizontal again",
            "grab the sealed end of the capillary and lift it so it hangs vertical, sealed end up",
            "move to the thermometer bulb and press the vertical sealed end against it so it sticks",
            "grab the horizontal thermometer stem, lift it, and rotate it vertical so the bulb points down",
            "raise the thermometer high, align it over the thiele tube mouth, and release it so it drops in and the stopper seals the mouth",
            "pick up the match, carry it to the lamp, touch the wick to light the alcohol lamp, then return the match",
        ]
        self.meta_actions = [PickCapillarySealedEnd(self.engine, from_home=True),
                             DipCapillaryIntoPowder(self.engine),
                             ReturnCapillaryToTable(self.engine),
                             PickCapillaryOpenEnd(self.engine),
                             TampCapillary(self.engine),
                             ReturnCapillaryAfterTamp(self.engine),
                             PickCapillaryMiddle(self.engine),
                             DipCapillaryInOil(self.engine),
                             ReturnCapillaryAfterOil(self.engine),
                             PickCapillarySealedEnd(self.engine),
                             StickCapillaryToBulb(self.engine),
                             PickThermometer(self.engine),
                             InsertThermometerIntoTube(self.engine),
                             LightFlamePass(self.engine)]
        self._meta_idx = 0
        self.debug_heat_move = bool(getattr(cfg, "debug_heat_move", False))
        self.debug_cap_lamp = bool(getattr(cfg, "debug_cap_lamp", False))
        if self.debug_cap_lamp:
            # 调试（盖帽）：跳过段1全部元动作，直进段2等 cap_lamp 相（只跑 CapLampPass 盖帽；
            # task 端把灯预摆移灯位+火焰点亮）
            self._meta_idx = len(self.meta_actions)
            print("[b5] debug_cap_lamp: skip segment1 -> wait cap_lamp phase")
        elif self.debug_heat_move:
            # 调试（加热/移灯）：跳过段1全部元动作，直进段2等 heating 相
            # （只跑 LampHeatMovePass 夹灯摆动 15s + 移灯 -X 5cm）
            self._meta_idx = len(self.meta_actions)
            print("[b5] debug_heat_move: skip segment1 -> wait heating phase")
        self._lamp_pass = None     # 段2 加热/移灯元动作（phase=="heating" 时按需实例化）
        self._cap_pass = None      # 段2 盖帽元动作（phase=="cap_lamp" 时按需实例化）
        self._h5_sample = 0
        self._start = True
        self._done = False

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)
        self._done = False

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self._meta_idx = 0
        # 调试跳过段1：reset() 会在 _init_collect_mode 之后调用，须重应用（B2 缺此步，B5 补上）
        if self.debug_cap_lamp:
            self._meta_idx = len(self.meta_actions)
        elif self.debug_heat_move:
            self._meta_idx = len(self.meta_actions)
        self._lamp_pass = None
        self._cap_pass = None
        self._start = True
        self._done = False
        self._h5_sample = 0
        if self.mode == "collect":
            for m in self.meta_actions:
                m.reset()
            self.rmp_controller.reset()
        else:
            self.inference_engine.reset()

    def step(self, state):
        self.state = state
        if self.mode == "collect":
            return self._step_collect(state)
        return self._step_infer(state)

    def _cache_h5(self, state):
        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            # 只缓存 obs_names 里的观测相机（策略/数据集需要的）；跳过仅显示相机（camera_3 手部
            # 相机）。否则每 4 帧把全分辨率图攒进内存 list 到 episode 结束才写盘 → 一帧 ~2.3MB×3
            # 台、整集几十 GB → OOM SIGKILL（视频流流式无碍，元凶是这缓存）。
            obs_names = getattr(getattr(self.cfg, "infer", None), "obs_names", None) or {}
            camera_images = (
                {k: v for k, v in state["camera_data"].items() if k in obs_names}
                if obs_names else state["camera_data"]
            )
            self.data_collector.cache_step(
                camera_images=camera_images,
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )

    def _step_collect(self, state):
        if self._done:
            print("[b5] all phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        jp = state.get("joint_positions")

        # 段2：装样+点火完成，机械臂回显保持，等 task 相态机推 ignited → heating（夹灯摆动 15s
        # 控温 + 移灯 -X 5cm）→ cap_lamp（盖帽熄火）。phase=="heating" 跑 LampHeatMovePass、
        # phase=="cap_lamp" 跑 CapLampPass，两 pass 都以 task 信号门控创建（lamp_released /
        # cap_settled）：一完成即不重建，否则 phase 停留期内每帧 `_pass is None` 会无限重跑。
        # phase=="done"（盖帽熄火 + 结果停留帧）→ 上报成功。
        if self._meta_idx >= len(self.meta_actions):
            if (state.get("phase") == "heating" and self._lamp_pass is None
                    and not state.get("lamp_released")):
                self._lamp_pass = LampHeatMovePass(self.engine)
                self._lamp_pass.reset()
                print("[b5] 段2: phase heating -> run LampHeatMovePass (grab lamp, sway 15s, move -X 5cm)")
            if (state.get("phase") == "cap_lamp" and self._cap_pass is None
                    and not state.get("cap_settled")):
                self._cap_pass = CapLampPass(self.engine)
                self._cap_pass.reset()
                print("[b5] 段2: phase cap_lamp -> run CapLampPass (grab cap, cover lamp)")
            if self._lamp_pass is not None:
                action = self._lamp_pass.forward(state)
                if self._lamp_pass.is_done():
                    print("[b5] LampHeatMovePass done")
                    self._lamp_pass = None
            elif self._cap_pass is not None:
                action = self._cap_pass.forward(state)
                if self._cap_pass.is_done():
                    print("[b5] CapLampPass done")
                    self._cap_pass = None
            else:
                action = (ArticulationAction(joint_positions=np.array(jp, dtype=float))
                          if jp is not None else None)
            if state.get("phase") == "done":
                self._done = True
            self._cache_h5(state)
            return action, False, False

        if self._start:
            # 首帧：只发夹爪打开（稳定握姿再开始），臂不动
            self._start = False
            target = np.full(jp.shape[0], np.nan)
            target[7] = GRIP_OPEN / get_stage_units()
            target[8] = GRIP_OPEN / get_stage_units()
            action = ArticulationAction(joint_positions=target)
        else:
            meta = self.meta_actions[self._meta_idx]
            action = meta.forward(state)
            if meta.is_done():
                print(f"[b5] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1
                if self._meta_idx < len(self.meta_actions):
                    self.meta_actions[self._meta_idx].grip_target = meta.grip_target

        self._cache_h5(state)
        return action, False, False

    def _step_infer(self, state):
        if self._done:
            self.reset_needed = True
            return None, True, self._last_success
        if state.get("phase") == "done":
            self._done = True
            self._last_success = True
        state["language_instruction"] = self.get_language_instruction()
        action = self.inference_engine.step_inference(state)
        return action, False, self.is_success()

    def is_success(self):
        return self._done

    def get_language_instruction(self):
        return ("Grab the sealed end of the capillary tube and lift it so it hangs vertical, "
                "dip its open end into the powder mound, lay it back on the table, then grab "
                "the open end and lift it, tamping the powder to the sealed end by moving up "
                "and down rapidly 10 times, lay it back on the table, then grab the middle of "
                "the capillary keeping it horizontal, dip its sealed end into the paraffin oil, "
                "lay it back on the table, then grab the sealed end and lift it vertical again, "
                "press the sealed end against the thermometer bulb so it sticks, then grab the "
                "horizontal thermometer stem, lift it, and rotate it vertical so the bulb "
                "points down, raise it high, align it over the thiele tube mouth, and release "
                "it so it drops in and the stopper seals the mouth, then pick up the match, "
                "carry it to the alcohol lamp, touch the wick to light the flame, and return "
                "the match; then grab the alcohol lamp body, oscillate it in the x direction "
                "for 15 seconds to control the heating rate, move it 5 cm in the -x direction, "
                "then pick up the lamp cap and cover the lamp to extinguish the flame")
