# -*- coding: utf-8 -*-
"""B2 沸点测定控制器：阶段 B = 先跑滴加元动作，再等 task 相态推完（加热/沸腾）。

两段式（v1 自动观测 → v2 机械臂操作，逐步接入 V7 10 步）：
  段 1  DropperDripPass：抓滴管吸样品瓶液 → 滴入试管出液柱（cycles 遍）→ 放回。
        液滴落定后 task._grow_tube_level 逐滴长高 TestTubeLiquid，task 内部置
        _liquid_added → idle 门控解除，才允许 ignite。
  段 2  滴加完成后机械臂回显保持（夹爪停在 GRIP_OPEN，滴管已放回），等 task 相态机
        自行推 ignited → heating（8s，气泡逐个 reveal）→ boiling（5s，气泡不熄）→
        move_lamp（v1 加热/沸腾/毛细柱动画保留）。phase=="move_lamp" 时跑 LampMovePass
        （水平横夹灯体宽处 -y 移 10cm；移灯期间气泡继续沸腾）；读到 phase=="done"
        （灯移到位且气泡渐熄完成）→ 上报成功并请求 reset。

与 d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用
IkMotionEngine）。动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作
grip_target 传播）沿用 flametest/d2s。
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
from .meta_actions import (DropperDripPass, AddZeolitePass, HangThermometer,
                           LightFlamePass, LampMovePass, CapLampPass)
from .meta_actions.constants import GRIP_OPEN


class B2AlcoholHeatLiquidTaskController(TaskBaseController):
    """Composite controller: 段 1 跑滴加元动作，段 2 等 task 相态推完再报成功。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b2] controller VERSION v3.6 (DropperDripPass + AddZeolitePass x2 + HangThermometer + LightFlamePass + LampMovePass + CapLampPass + v1 phase watch)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 段 1：滴加元动作（一次持握内循环吸液-滴液 cfg.sample_cycles 遍）→ 放两颗沸石 → 挂温度计 → 点燃酒精灯
        sample_cycles = max(1, int(getattr(cfg, "sample_cycles", 3)))
        self.meta_classes = [DropperDripPass, AddZeolitePass, HangThermometer, LightFlamePass]
        self.meta_names = [f"dropper aspirate+drip into tube x{sample_cycles}",
                           "grab 2 zeolites + rotate + drop into tube",
                           "grab thermometer + insert into tube + hang on hook (step 3 of 4)",
                           "grab match + ignite alcohol lamp + return match (step 4 of 4)"]
        self.meta_actions = [DropperDripPass(self.engine, cycles=sample_cycles),
                             AddZeolitePass(self.engine),
                             HangThermometer(self.engine),
                             LightFlamePass(self.engine)]
        self._meta_idx = 0
        self.debug_lamp_move = bool(getattr(cfg, "debug_lamp_move", False))
        self.debug_cap_lamp = bool(getattr(cfg, "debug_cap_lamp", False))
        if self.debug_cap_lamp:
            # 调试（2026-08-28 盖帽）：跳过段 1 全部元动作，直接进段 2 等 cap_lamp 相
            # （只跑 CapLampPass 盖帽；task 端把灯预摆移灯位+火焰点亮）
            self._meta_idx = len(self.meta_actions)
            print("[b2] debug_cap_lamp: skip segment1 -> wait cap_lamp phase")
        elif self.debug_lamp_move:
            # 调试（用户 2026-08-27「先把前面的动作都隐藏就先做这一个动作」）：
            # 跳过段 1 全部元动作，直接进段 2 等 move_lamp 相（只跑 LampMovePass 抓灯移灯）
            self._meta_idx = len(self.meta_actions)
            print("[b2] debug_lamp_move: skip segment1 -> wait move_lamp phase")
        self._lamp_pass = None     # 段 2 移灯元动作（phase=="move_lamp" 时按需实例化）
        self._cap_pass = None      # 段 2 盖帽元动作（phase=="cap_lamp" 时按需实例化）
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
            # 只缓存 obs_names 里的观测相机（策略/数据集需要的）；跳过仅显示相机（如 camera_3
            # 手部相机）。否则每 4 帧把全分辨率图攒进内存 list 到 episode 结束才写盘——1920×3
            # 台 = 一帧 ~33MB，整集几十 GB → OOM SIGKILL（视频流本身流式无碍，元凶是这缓存）。
            # 过滤后 h5 只含观测相机；camera_display/视频/snapshot 仍用全部相机。
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
            print("[b2] all phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        jp = state.get("joint_positions")

        # 段 2：滴加完成，机械臂回显保持，等 task 相态（加热/沸腾）推到 move_lamp →
        # 跑 LampMovePass 移灯（一次），气泡渐熄 240 帧期间 phase 仍停 move_lamp；
        # 之后 cap_lamp → 跑 CapLampPass 盖帽（一次），熄火停 120 帧期间 phase 仍停
        # cap_lamp。两个 pass 都以 task 的完成信号门控创建（lamp_moved / cap_settled）：
        # 一完成即不重建，否则 phase 停留期内每帧 `_pass is None` 会无限重跑该动作
        # （2026-08-28 串联修复：完整流程下 arm 会在渐熄/结果停留期反复重做动作）。
        # phase=="done"（盖帽完成）→ 上报成功。
        if self._meta_idx >= len(self.meta_actions):
            if (state.get("phase") == "move_lamp" and self._lamp_pass is None
                    and not state.get("lamp_moved")):
                self._lamp_pass = LampMovePass(self.engine)
                self._lamp_pass.reset()
                print("[b2] 段2: phase move_lamp -> run LampMovePass (grab lamp -y 10cm)")
            if (state.get("phase") == "cap_lamp" and self._cap_pass is None
                    and not state.get("cap_settled")):
                self._cap_pass = CapLampPass(self.engine)
                self._cap_pass.reset()
                print("[b2] 段2: phase cap_lamp -> run CapLampPass (grab cap, cover lamp)")
            if self._lamp_pass is not None:
                action = self._lamp_pass.forward(state)
                if self._lamp_pass.is_done():
                    print("[b2] LampMovePass done")
                    self._lamp_pass = None
            elif self._cap_pass is not None:
                action = self._cap_pass.forward(state)
                if self._cap_pass.is_done():
                    print("[b2] CapLampPass done")
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
                print(f"[b2] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1

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
        return ("Use the dropper to aspirate liquid from the sample bottle and drip "
                "it into the test tube on the stand, then drop two boiling chips into the "
                "tube, hang the thermometer, and light the alcohol lamp with a match; "
                "heat the tube over the lamp until it boils, hold boiling for 5 seconds, "
                "then move the alcohol lamp away by 10 cm in the -y direction, then cover "
                "the lamp with its cap to extinguish the flame")
