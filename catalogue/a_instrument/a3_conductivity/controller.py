"""A3 电导率测量控制器（v11：夹皿提起 → 移烧杯上方 → 倾斜倒粉 → 放回空皿 → 夹洗瓶 → 挤水
→ 放回洗瓶 → 夹玻璃棒移烧杯上方 → 下降插入烧杯搅拌 → 放回玻璃棒 → 竖直提起电极 →
移电极到烧杯上方下降深入 → 松爪放电极 → 按下机顶「开始」键）。

与 a2 同构分层（Lula IK + 元动作组合）：meta_actions/（一个流程步骤 = 一个元动作），
本控制器实例化元动作按序 forward()，全部完成 → 额外保持 FINISH_HOLD_FRAMES 帧让
按下「确认」键后的场景清晰可见 → success。

动作序列（v11 = 14 元动作）：
  ① PickSurfaceDish    竖直夹住玻璃皿提起来（接近 → 下探 → 夹紧 → 提出）
  ② MoveDishAboveBeaker 夹着皿水平移动到烧杯口正上方
  ③ PourDishIntoBeaker  倾斜玻璃皿把粉末倒入烧杯（下降 → 原地倾斜 → 保持）
  ④ ReturnSurfaceDish   把空皿放回天平秤盘（转竖直横移回 → 竖直下探 → 开爪松放 → 抬回）
  ⑤ PickWashBottle    水平横夹洗瓶肚子 + 抬起 + 把红嘴移到烧杯上方（仅移动，不含挤水）
  ⑥ SqueezeWashBottle 挤压洗瓶身出水（水从红嘴弧线落入烧杯，烧杯内液面上涨）
  ⑦ ReturnWashBottle  把挤完水的洗瓶放回原位（水平移回 → 下探 → 开爪松放 → 抬回）
  ⑧ PickGlassRod      到试管架水平横夹取出玻璃棒并移到烧杯口正上方（下探横夹 → 提出 → 移）
  ⑨ StirInBeaker      下降玻璃棒插入烧杯并圆周搅拌（下探 → 画圆搅拌 → 提出）
  ⑩ ReturnGlassRod    把搅拌完的玻璃棒放回试管架（水平移回 → 下探 → 开爪松放 → 抬回）
  ⑪ LiftElectrode     到导电率仪前竖直抓住电极探头并竖直提起（-X 接近 → 下探 → +X 移入 → 竖直夹 cap → 提起）
  ⑫ DipElectrodeIntoBeaker 把电极移到烧杯口上方 + 竖直下降深入（水平横移 → 下降浸入液面）
  ⑬ ReleaseElectrode  松爪把电极留在烧杯内（开爪松放 → 抬空爪清电极撤离）
  ⑭ PressConfirmButton 按下导电率仪机顶「开始」键（高位接近 → 下探 → 完全闭合爪子 → 下压按钮顶 → 松爪 → 抬回）

⑭ 按下后 task._ButtonLifecycle 驱动测量动画（按钮下沉 + 进度条 ~4s → 结果屏显示 cfg.conductivity
导电率读数 → 按钮弹回），controller 等 button_state 回到 released 再额外保持 FINISH_HOLD_FRAMES
帧让读数清晰可见后 success（同 A1 折光仪）。
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
from .meta_actions import (PickSurfaceDish, MoveDishAboveBeaker, PourDishIntoBeaker,
                           ReturnSurfaceDish, PickWashBottle, SqueezeWashBottle,
                           ReturnWashBottle, PickGlassRod, StirInBeaker,
                           ReturnGlassRod, LiftElectrode, DipElectrodeIntoBeaker,
                           ReleaseElectrode, PressConfirmButton)
from .meta_actions.constants import GRIP_OPEN


class A3ConductivityTaskController(TaskBaseController):
    """Composite controller: A3 v11 = 夹皿提起 → 移烧杯上方 → 倾斜倒粉 → 放回空皿 → 夹洗瓶 → 挤水 → 放回洗瓶 → 夹玻璃棒移烧杯上方 → 下降插入烧杯搅拌 → 放回玻璃棒 → 竖直提起电极 → 移电极到烧杯上方下降深入 → 松爪放电极 → 按下机顶「开始」键。"""

    FINISH_HOLD_FRAMES = 60   # 结果屏定格后额外保持帧数（~1s，让导电率读数清晰可见）

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[a3] controller VERSION v11 (PickSurfaceDish + MoveDishAboveBeaker + PourDishIntoBeaker + ReturnSurfaceDish + PickWashBottle + SqueezeWashBottle + ReturnWashBottle + PickGlassRod + StirInBeaker + ReturnGlassRod + LiftElectrode + DipElectrodeIntoBeaker + ReleaseElectrode + PressConfirmButton, IK-driven)")
        # 引擎默认朝向 = 手指朝下（euler(0,π,0)）：竖直夹皿。
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3s/a1/a2）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 14 元动作（一个流程步骤 = 一个），按序执行
        self.meta_classes = [PickSurfaceDish, MoveDishAboveBeaker, PourDishIntoBeaker,
                             ReturnSurfaceDish, PickWashBottle, SqueezeWashBottle,
                             ReturnWashBottle, PickGlassRod, StirInBeaker,
                             ReturnGlassRod, LiftElectrode, DipElectrodeIntoBeaker,
                             ReleaseElectrode, PressConfirmButton]
        self.meta_names = [
            "P1 pick surface dish (vertical grip, lift)",
            "P2 move dish above sample beaker",
            "P3 tilt dish, pour powder into beaker",
            "P4 return empty dish to balance",
            "P5 pick wash bottle, move red spout above beaker",
            "P6 squeeze wash bottle, pour water into beaker",
            "P7 return wash bottle to its home",
            "P8 pick glass rod from rack, move above beaker",
            "P9 lower glass rod into beaker and stir",
            "P10 return glass rod to rack after stirring",
            "P11 lift conductivity meter electrode vertically",
            "P12 move electrode above beaker and lower into liquid",
            "P13 release electrode to leave it standing in the beaker",
            "P14 press the meter start button on top",
        ]
        self.meta_actions = [
            PickSurfaceDish(self.engine),
            MoveDishAboveBeaker(self.engine),
            PourDishIntoBeaker(self.engine),
            ReturnSurfaceDish(self.engine),
            PickWashBottle(self.engine),
            SqueezeWashBottle(self.engine),
            ReturnWashBottle(self.engine),
            PickGlassRod(self.engine),
            StirInBeaker(self.engine),
            ReturnGlassRod(self.engine),
            LiftElectrode(self.engine),
            DipElectrodeIntoBeaker(self.engine),
            ReleaseElectrode(self.engine),
            PressConfirmButton(self.engine),
        ]
        self._meta_idx = 0
        self._h5_sample = 0
        self._start = True
        self._finish_hold = 0   # 全部动作完成后的额外保持帧计数（FINISH_HOLD_FRAMES）

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
            # 动作序列全部完成，但仪器测量显示可能还没走完：task 的 _ButtonLifecycle 在按下
            # 按钮后需 ~4s 进度条 → 结果屏（导电率读数）→ 按钮缓慢弹回才到 released（结果屏定格）。
            # 期间臂停住、只推屏幕/按钮动画（done=False → 视频继续录），到 released 再额外保持
            # FINISH_HOLD_FRAMES 帧让读数清晰可见后结束（用户「要仪器显示完」）。
            bs = state.get("button_state")
            if bs not in ("released", "idle"):
                return self._hold_action(state)
            if self._finish_hold < self.FINISH_HOLD_FRAMES:
                self._finish_hold += 1
                return self._hold_action(state)
            print("[a3] all meta-actions done + measurement shown. success.")
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
                print(f"[a3] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        """臂停住当前位置（夹起皿悬空时 / 等待仪器测量动画走完时用）。

        直接以当前关节角作为目标发回，等价于「保持不动」，让 task 逐帧推进按钮/屏幕动画；
        done=False 使主循环继续写视频帧。"""
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
        return ("Pick up the glass dish with sample powder, move it above the beaker, "
                "tilt the dish to pour the powder into the beaker, "
                "then return the empty dish to the balance, grip the wash bottle, "
                "move its red spout above the beaker, squeeze it to pour water into the beaker, "
                "return the wash bottle to its home, pick up the glass rod from the rack "
                "and move it above the beaker, lower the rod into the beaker and stir, "
                "return the glass rod to the rack, then lift the conductivity meter electrode "
                "vertically, move the electrode above the beaker and lower it into the liquid, "
                "release the electrode to leave it standing in the beaker, "
                "then press the meter start button on top")
