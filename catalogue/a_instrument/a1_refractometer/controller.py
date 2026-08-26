"""A1 折光率测量控制器：顺序执行元动作（取瓶塞 → 取滴管吸样滴样）。

与 d4l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册 ①PickCapPass（取瓶塞：直拔，不取瓶不旋转）→ ②SamplePass（取滴管吸样→滴样到
棱镜）顺序执行。动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target
传播）沿用 flametest/d2s/d3l/d4l。
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
from .meta_actions import PickCapPass, SamplePass, CloseCoverPass, PressStartPass
from .meta_actions.constants import GRIP_OPEN


class A1RefractometerTaskController(TaskBaseController):
    """Composite controller: A1 = 取瓶塞 + 取滴管吸样滴样 + 合盖 + 按测量键。

    动作序列全部完成后不立即结束 episode：仪器测量显示（_ButtonLifecycle 红进度条 ~4s →
    绿结果屏 → 按钮缓慢弹回）由 task 逐帧推进，可能晚于最后动作结束。保持 episode 存活、
    臂停住，等 button_state 回到 released（结果屏定格）再额外保持 FINISH_HOLD_FRAMES 帧
    让读数清晰可见后 success（用户 2026-08-26「要仪器显示完」）。
    """

    FINISH_HOLD_FRAMES = 60   # 结果屏定格后额外保持帧数（~1s，让 nD 读数清晰可见）

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[a1] controller VERSION v1 (pick cap + dropper sample, IK-driven)")
        # 引擎默认朝向 = 手指朝下（euler(0,π,0)）：正向持握。
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3l/d4l）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作：①PICK_CAP_PASS（取瓶塞直拔倒放桌面）→ ②SAMPLE_PASS（取滴管吸样→
        # 滴样到棱镜），顺序执行。
        self.meta_classes = [PickCapPass, SamplePass, CloseCoverPass, PressStartPass]
        self.meta_names = [
            "P pick cap off bottle (straight pull-off)",
            "S dropper aspirate sample + drip onto prism",
            "C close the prism cover (push -y over front edge)",
            "B press the start button (trigger measurement reading)",
        ]
        self.meta_actions = [
            PickCapPass(self.engine),
            SamplePass(self.engine),
            CloseCoverPass(self.engine),
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
            # 动作序列全部完成，但仪器测量显示可能还没走完：task 的 _ButtonLifecycle 在
            # 按下按钮后需 ~4s 进度条 → 结果屏 → 按钮缓慢弹回才到 released（结果屏定格）。
            # 期间臂停住、只推屏幕/按钮动画（done=False → 视频继续录），到 released 再额外
            # 保持 FINISH_HOLD_FRAMES 帧让读数清晰可见后结束（用户「要仪器显示完」）。
            bs = state.get("button_state")
            if bs not in ("released", "idle"):
                return self._hold_action(state)
            if self._finish_hold < self.FINISH_HOLD_FRAMES:
                self._finish_hold += 1
                return self._hold_action(state)
            print("[a1] all meta-actions done + measurement shown. success.")
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
                print(f"[a1] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Pull the stopper straight off the sample bottle, then pick up the "
                "dropper, aspirate some sample liquid from the bottle, drip it "
                "onto the prism of the refractometer, push the cover closed "
                "over the prism, and press the start button to take the reading")
