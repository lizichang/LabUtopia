# -*- coding: utf-8 -*-
"""B3 水浴加热控制器：段 1 放固体+点火，段 2 等 task 相态机推到 cap_lamp 再盖帽。

两段式（照 B2 同构分层）：
  段 1  SolidTransferPass（两颗固体依次夹入预夹试管）→ LightFlamePass（火柴点燃酒精灯）。
        固体落定后 task 置 solid_added、火柴触灯芯 task 置 flame_lit → idle 门控解除，
        才允许 task 相态机推 ignited（火焰 reveal）。
  段 2  放固体+点火完成后机械臂回显保持（夹爪停在 GRIP_OPEN），等 task 相态机自行推
        ignited → heating（气泡逐个 reveal）→ boiling（5s，固体熔化/不熔化）→ cap_lamp。
        phase=="cap_lamp" 时跑 CapLampPass（原位盖帽，不移灯；盖帽期间气泡继续沸腾）；
        读到 phase=="done"（帽盖到位火焰熄灭 + 气泡渐熄完成）→ 上报成功并请求 reset。

与 B2 唯一结构差：段 1 无滴管/沸石/温度计（B3 固体用 SolidTransferPass 替代 AddZeolitePass，
顺序也是先固体后点火），段 2 无移灯（cap_lamp 直接原位盖帽）。动作级契约（grip 每帧发送、
到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest/d2s/b2。
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
from .meta_actions import SolidTransferPass, LightFlamePass, CapLampPass
from .meta_actions.constants import GRIP_OPEN


class B3WaterBathTaskController(TaskBaseController):
    """Composite controller: 段 1 放固体+点火，段 2 等 task 相态推完再盖帽报成功。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b3] controller VERSION v1.0 (SolidTransferPass + LightFlamePass + phase watch -> CapLampPass)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l/b2）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 段 1：放两颗固体入试管 → 点燃酒精灯
        self.meta_classes = [SolidTransferPass, LightFlamePass]
        self.meta_names = ["grab 2 solids + rotate + drop into tube",
                           "grab match + ignite alcohol lamp + return match"]
        self.meta_actions = [SolidTransferPass(self.engine),
                             LightFlamePass(self.engine)]
        self._meta_idx = 0
        self.debug_cap_lamp = bool(getattr(cfg, "debug_cap_lamp", False))
        if self.debug_cap_lamp:
            # 调试（2026-08-29 盖帽）：跳过段 1 全部元动作，直接进段 2 等 cap_lamp 相
            # （只跑 CapLampPass 盖帽；task 端把固体预置管底+火焰点亮）
            self._meta_idx = len(self.meta_actions)
            print("[b3] debug_cap_lamp: skip segment1 -> wait cap_lamp phase")
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
            # 只缓存 obs_names 里的观测相机（同 B2 防 OOM 过滤）
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
            print("[b3] all phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        jp = state.get("joint_positions")

        # 段 2：放固体+点火完成，机械臂回显保持，等 task 相态（加热/沸腾）推到 cap_lamp →
        # 跑 CapLampPass 原位盖帽（一次），熄火 + 气泡渐熄期间 phase 仍停 cap_lamp。
        # pass 以 task 的完成信号门控创建（cap_settled）：一完成即不重建，否则 phase
        # 停留期内每帧 `_pass is None` 会无限重跑盖帽（同 B2 串联修复）。
        # phase=="done"（盖帽完成）→ 上报成功。
        if self._meta_idx >= len(self.meta_actions):
            if (state.get("phase") == "cap_lamp" and self._cap_pass is None
                    and not state.get("cap_settled")):
                self._cap_pass = CapLampPass(self.engine)
                self._cap_pass.reset()
                print("[b3] 段2: phase cap_lamp -> run CapLampPass (grab cap, cover lamp in place)")
            if self._cap_pass is not None:
                action = self._cap_pass.forward(state)
                if self._cap_pass.is_done():
                    print("[b3] CapLampPass done")
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
                print(f"[b3] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Use the gripper to pick up each solid sample from the surface dish and "
                "drop it into the test tube clamped in the water bath, then light the "
                "alcohol lamp with a match; heat the water bath until it boils to melt the "
                "solid in the tube, hold boiling for 5 seconds, then cover the lamp with "
                "its cap in place to extinguish the flame")
