"""A2 旋光仪测量控制器：顺序执行 10 个元动作，按完测量键后保持等读数屏定格。

与 a1 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - meta_actions/（一个流程步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → 等按钮读数走完 → success。

动作序列（10 元动作）：
  ① PrePoseWash 预摆 d2s 洗瓶入口姿势 → ② PickWashBottle 拿洗瓶 → ③ SqueezeWater 挤水入试管
  → ④ ReturnWashBottle 放回洗瓶 → ⑤ TubeShakePass 拿起试管震荡溶解 → ⑥ DropperTransferPass
  胶头滴管转移（吸试管内液 3 次、每次挤进旋光管加液口；2026-08-27 替代倒液）
  → ⑦ PickPolarimeterTube 拿旋光管 → ⑧ PlaceOnRails 放导轨 → ⑨ CloseLidPass 拨回翻盖
  （2026-08-27 用户「按按钮前伸到翻盖之后拨回盖上」）→ ⑩ PressStartPass 按启动键。

① PrePoseWash（2026-08-27 乱动修复）：d2s 的 PickWashBottle 是第 3 个动作（从药匙架高位
进入），A2 的是第 1 个（冷启动）→ IK 落不同分支、送红嘴乱动。先预摆到 d2s 同款入臂姿势，
分支连续照搬 d2s 行为。

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用
flametest/d2s/d3s/a1。最后动作（按启动键）完成后不立即结束 episode：仪器测量显示
（_ButtonLifecycle 红进度条 ~4s → 绿结果屏 → 按钮缓慢弹回）由 task 逐帧推进，可能晚于
最后动作结束。保持 episode 存活、臂停住，等 button_state 回到 released（结果屏定格）再
额外保持 FINISH_HOLD_FRAMES 帧让读数清晰可见后 success（a1 同款）。
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
from .meta_actions import (
    PrePoseWash, PickWashBottle, SqueezeWater, ReturnWashBottle, TubeShakePass,
    DropperTransferPass,
    PickPolarimeterTube, PlaceOnRails, CloseLidPass, PressStartPass,
)
from .meta_actions.constants import GRIP_OPEN


class A2PolarimeterTaskController(TaskBaseController):
    """Composite controller: A2 = 洗瓶注水 + 试管震荡溶解 + 倒液 + 放导轨 + 按测量键读数。"""

    FINISH_HOLD_FRAMES = 120  # 结果屏定格后额外保持帧数（2s @60fps，用户「结果显示完 2s 才结束」）

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[a2] controller VERSION v4 (10 meta-actions, dropper-transfer + close-lid + IK-driven)")
        # 引擎默认朝向 = 手指朝下（euler(0,π,0)）：正向持握。
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3s/a1）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 9 元动作（一个流程步骤 = 一个），按序执行；TubeShakePass/DropperTransferPass 带 cycles。
        shake_cycles = max(1, int(getattr(cfg, "shake_cycles", 3)))
        drop_cycles = max(1, int(getattr(cfg, "drop_cycles", 3)))
        self.meta_classes = [PrePoseWash, PickWashBottle, SqueezeWater, ReturnWashBottle,
                             TubeShakePass, DropperTransferPass,
                             PickPolarimeterTube, PlaceOnRails, CloseLidPass, PressStartPass]
        self.meta_names = [
            "W0 pre-pose to d2s wash-entry branch (fix erratic delivery)",
            "W1 pick wash bottle belly (x-offset descent, horizontal grip, lift)",
            "W2 squeeze wash bottle (water stream into test tube)",
            "W3 return wash bottle",
            "T1 pick test tube + shake to dissolve powder (TubeShakePass)",
            "D1 dropper transfer: suck tube liquid 3x, drip into fill port (no pour)",
            "P1 pick polarimeter tube (horizontal grip, lift clear)",
            "P2 place polarimeter tube on rails",
            "L close the lid (reach behind open lid, flip it back shut)",
            "B press the start button (trigger measurement reading)",
        ]
        self.meta_actions = [
            PrePoseWash(self.engine),
            PickWashBottle(self.engine),
            SqueezeWater(self.engine),
            ReturnWashBottle(self.engine),
            TubeShakePass(self.engine, cycles=shake_cycles),
            DropperTransferPass(self.engine, cycles=drop_cycles),
            PickPolarimeterTube(self.engine),
            PlaceOnRails(self.engine),
            CloseLidPass(self.engine),
            PressStartPass(self.engine),
        ]
        self._meta_idx = 0
        self._h5_sample = 0
        self._start = True
        self._finish_hold = 0   # 结果屏定格后的额外保持帧计数（FINISH_HOLD_FRAMES）

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        if self.mode == "collect":
            self._meta_idx = 0
            self._start = True
            self._finish_hold = 0
            for m in self.meta_actions:
                m.reset()
            self.rmp_controller.reset()
        else:
            self.inference_engine.reset()

    def step(self, state):
        self.state = state
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state):
        if self._meta_idx >= len(self.meta_actions):
            # 动作序列全部完成，但仪器测量显示可能还没走完：task 的 _update_button 在按下
            # 按钮后需 ~4s 进度条 → 结果屏 → 按钮缓慢弹回才到 released（结果屏定格）。期间
            # 臂停住、只推屏幕/按钮动画（done=False → 视频继续录），到 released 再额外保持
            # FINISH_HOLD_FRAMES 帧让读数清晰可见后结束（a1「要仪器显示完」）。
            bs = state.get("button_state")
            if bs not in ("released", "idle"):
                return self._hold_action(state)
            if self._finish_hold < self.FINISH_HOLD_FRAMES:
                self._finish_hold += 1
                return self._hold_action(state)
            print("[a2] all meta-actions done + measurement shown. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        if self._start:
            # 首帧：只发夹爪打开（稳定握姿再开始），臂不动
            self._start = False
            target = np.full(state["joint_positions"].shape[0], np.nan)
            target[7] = GRIP_OPEN / get_stage_units()
            target[8] = GRIP_OPEN / get_stage_units()
            action = ArticulationAction(joint_positions=target)
        else:
            meta = self.meta_actions[self._meta_idx]
            action = meta.forward(state)
            if meta.is_done():
                print(f"[a2] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
                self._meta_idx += 1
                if self._meta_idx < len(self.meta_actions):
                    self.meta_actions[self._meta_idx].grip_target = meta.grip_target

        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            self.data_collector.cache_step(
                camera_images=state["camera_data"],
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )
        return action, False, False

    def _hold_action(self, state):
        """臂停住当前位置（等待仪器测量动画走完 / 结果屏定格时用）。

        直接以当前关节角作为目标发回，等价于「保持不动」，让 task 逐帧推进按钮/屏幕
        动画；done=False 使主循环继续写视频帧。"""
        target = np.array(state["joint_positions"], dtype=float).copy()
        return ArticulationAction(joint_positions=target), False, False

    def _step_infer(self, state):
        if self._meta_idx >= len(self.meta_actions):
            self.reset_needed = True
            return None, True, self._last_success

        state["language_instruction"] = self.get_language_instruction()
        action = self.inference_engine.step_inference(state)
        return action, False, self.is_success()

    def is_success(self):
        return self._meta_idx >= len(self.meta_actions)

    def get_language_instruction(self):
        return ("Pick up the wash bottle, squeeze distilled water into the test tube, "
                "shake the test tube to dissolve the powder, transfer the solution "
                "into the polarimeter tube with a dropper, place the tube on the "
                "polarimeter rails, and press the start button to take the rotation "
                "reading")
