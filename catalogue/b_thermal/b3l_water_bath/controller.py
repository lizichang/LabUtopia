# -*- coding: utf-8 -*-
"""B3L 水浴加热控制器：段 1 滴管滴加溶液入试管 + 点燃酒精灯 + 拿试管入水浴，段 2 等 task 相态机
推到 tube_return → move_lamp → cap_lamp 逐个跑对应元动作。

两段式（照 B2/B3S 同构分层，2026-08-30 B3L 液体实验复刻 d3l 滴加链）：
  段 1  DropperDripPass（①-⑱ 取样滴管→瓶口挤气→浸液吸液→管口挤胶头滴液入试管，一次持握循环
        cfg.sample_cycles 遍→放回滴管架，**完全复刻 d3l sample_pass**，用户逐字「不是挖粉末
        而是往试管里面滴加溶液（动作参考d3l）」）→ LightFlamePass（**先点燃酒精灯**，火柴点火，
        用户逐字「先点燃酒精灯，然后再拿起试管过去加热」）→ PickTubePass（**后拿试管**，
        照 B1 _T_HELD_TUBE 矩阵持握，提出架孔 → 纯平移分段转移（不反转不洒液）→ 竖直浸入
        烧杯水浴 → **不松爪**，机械臂保持夹持直到加热结束，用户「拿试管加热的时候机械臂不能
        松手」）。
        滴液落定 task 置 solution_added、火柴触灯芯 task 置 flame_lit（火焰立即 reveal）、试管
        浸入 task 置 tube_immersed → idle 门控（三者齐备）解除，才允许 task 相态机推 ignited →
        加热。
  段 2  滴加+点火+试管入水浴完成后机械臂回显保持（夹爪停在 GRIP_OPEN），等 task 相态机自行推
        ignited → heating（气泡逐个 reveal）→ boiling（5s，液体渐变变色）→ tube_return。
        按 phase 逐个跑对应元动作（每个只跑一次，以 task 完成信号门控创建，同 B2 串联修复）：
          tube_return → ReturnTubePass（试管放回架孔，用户「直到加热结束才放回去」）
          move_lamp   → LampMovePass（酒精灯 +Y 移 20cm，用户「先把酒精灯往+y方向移动20cm
                        (参考b2)，然后再盖上灯冒」）
          cap_lamp    → CapLampPass（盖帽，帽盖到位火焰熄灭）
        读到 phase=="done"（帽盖到位 + 气泡渐熄完成）→ 上报成功并请求 reset。

段 1 与 B3S/D2-S 差异：无药匙/表面皿/粉丘挖粉链（液体样品用滴管滴加溶液，非挖粉）；无洗瓶。
结果现象 = 双颜色输入 before_color/liquid_color + 渐变变色（task._step_color_transition，
照 B2，只变色不沸腾）。动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target
传播）沿用 flametest/d2s/b2/b3s。
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
from .meta_actions import (DropperDripPass, PickTubePass, LightFlamePass,
                           ReturnTubePass, LampMovePass, CapLampPass)
from .meta_actions.constants import GRIP_OPEN, GRIP_TUBE


class B3LWaterBathTaskController(TaskBaseController):
    """Composite controller: 段 1 滴加溶液+点火，段 2 等 task 相态推完再盖帽报成功。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b3l] controller VERSION v1.0 (DropperDripPass drip solution + LightFlamePass first + PickTubePass hold-no-release + phase watch -> ReturnTubePass -> LampMovePass -> CapLampPass)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l/b2/b3s）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 段 1：滴管滴加溶液入试管（d3l sample_pass 复刻）→ **先点燃酒精灯**（用户逐字「先点燃
        # 酒精灯，然后再拿起试管过去加热」）→ **后拿试管**转水浴（照 B1 水平横夹 + 纯平移分段
        # 转移 + 浸入不松爪，用户「拿试管加热的时候机械臂不能松手」）
        self.meta_classes = [DropperDripPass, LightFlamePass, PickTubePass]
        self.meta_names = ["S1 pick dropper + aspirate solution from bottle + drip into rack tube",
                           "S2 grab match + ignite alcohol lamp + return match",
                           "S3 pick tube horizontally (B1 grip) + transfer pure-translation into beaker + immerse hold (no release)"]
        self.meta_actions = [DropperDripPass(self.engine,
                                             cycles=int(getattr(cfg, "sample_cycles", 3))),
                             LightFlamePass(self.engine),
                             PickTubePass(self.engine)]
        self._meta_idx = 0
        self.debug_cap_lamp = bool(getattr(cfg, "debug_cap_lamp", False))
        if self.debug_cap_lamp:
            # 调试（2026-08-30 盖帽）：跳过段 1 全部元动作，直接进段 2 等 cap_lamp 相
            # （只跑 CapLampPass 盖帽；task 端把溶液预置管底+火焰点亮）
            self._meta_idx = len(self.meta_actions)
            print("[b3l] debug_cap_lamp: skip segment1 -> wait cap_lamp phase")
        # 2026-08-31 调试（用户「把现在卡住后面的先隐藏，先看前面的对不对」）：true = 段 1
        # （S1 滴加 → S2 点火 → S3 试管浸入水浴）一完成即收尾成功，不推段 2（加热/沸腾/
        # 试管放回/移灯/盖帽）。前端 S1-S3 已验证可用，卡点在段 2，先隔离验证前端。
        self.debug_s3_only = bool(getattr(cfg, "debug_s3_only", False))
        if self.debug_s3_only:
            print("[b3l] debug_s3_only: segment1 done -> immediate success (skip segment 2)")
        self._s2_pass = None      # 段 2 元动作（tube_return/move_lamp/cap_lamp 相按需实例化）
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
        self._s2_pass = None
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
            print("[b3l] all phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        jp = state.get("joint_positions")

        # 段 2：滴加+点火+试管入水浴完成，机械臂回显保持，等 task 相态推 tube_return →
        # move_lamp → cap_lamp，按 phase 逐个跑对应元动作（每个只跑一次）。
        # pass 以 task 的完成信号门控创建（tube_returned/lamp_released/cap_settled）：
        # 一完成即 _s2_pass=None 不重建，否则 phase 停留期内每帧 `_s2_pass is None` 会
        # 无限重跑该元动作（同 B2 串联修复）。phase=="done"（盖帽完成）→ 上报成功。
        if self._meta_idx >= len(self.meta_actions):
            phase = state.get("phase")
            if self.debug_s3_only:
                # 2026-08-31 用户「先看前面的对不对」：S3 一完成即收尾，段 2（加热/沸腾/试管
                # 放回/移灯/盖帽）整段隐藏不跑，上报成功请求 reset，视频停在试管浸入水浴。
                print("[b3l] debug_s3_only: segment1 done -> success (skip segment 2)")
                self.data_collector.write_cached_data(state["joint_positions"][:-1])
                self._last_success = True
                self.reset_needed = True
                return None, True, True
            if self._s2_pass is None:
                is_tube_return = False
                if phase == "tube_return" and not state.get("tube_returned"):
                    self._s2_pass = ReturnTubePass(self.engine)
                    is_tube_return = True
                elif phase == "move_lamp" and not state.get("lamp_released"):
                    self._s2_pass = LampMovePass(self.engine)
                elif phase == "cap_lamp" and not state.get("cap_settled"):
                    self._s2_pass = CapLampPass(self.engine)
                if self._s2_pass is not None:
                    self._s2_pass.reset()
                    # 2026-08-30 修（根因）：旧代码在 reset() 前设 grip_target=GRIP_TUBE，但
                    # BaseMetaAction.reset() 会把 grip_target 重置回 GRIP_OPEN(0.04)——于是
                    # ReturnTubePass 第①步 mv 每帧发 0.04 → 爪子张开 → 试管悬浮爪间。
                    # 加热全程不松爪（用户「直到加热结束才放回去」），ReturnTubePass ①②③
                    # 须夹持试管（GRIP_TUBE），第④步 grip(GRIP_OPEN) 才真松爪。故 reset 后再设。
                    if is_tube_return:
                        self._s2_pass.grip_target = GRIP_TUBE
                    print(f"[b3l] 段2: phase {phase} -> run {type(self._s2_pass).__name__}")
            if self._s2_pass is not None:
                action = self._s2_pass.forward(state)
                if self._s2_pass.is_done():
                    print(f"[b3l] {type(self._s2_pass).__name__} done")
                    self._s2_pass = None
            else:
                action = (ArticulationAction(joint_positions=np.array(jp, dtype=float))
                          if jp is not None else None)
            if phase == "done":
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
                print(f"[b3l] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pick up the dropper from the rack, aspirate solution from the sample "
                "bottle and drip it into the test tube standing in the rack, return the "
                "dropper, then light the alcohol lamp with a match; pick up the test "
                "tube, transfer it into the beaker water bath and hold it while heating "
                "until the liquid changes color, then return the tube to the rack, "
                "move the alcohol lamp 20 cm away and cover it with its cap to "
                "extinguish the flame")
