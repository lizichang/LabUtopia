"""D3-S 固体样品 + 酸性试剂滴加反应控制器：整个实验 = 顺序执行元动作。

D3-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加酸性试剂」。挖粉动作（PickSpatula →
ReturnSpatula）与试管震荡（TubeShakePass）复用 d2s（坐标逐字一致，用户要求机械臂/
粉末/试管相对位置都不变，挖粉轨迹才不跑偏）；酸滴管（AcidPass）照 d3l acid_pass 单段
滴加 + B2 水平横夹（ORIENT_FWD）。

与 d2s/d3l 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用
IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册顺序：①PickSpatula（挖粉倒粉）→ ②ReturnSpatula（药匙放回）→ ③AcidPass（吸酸滴酸）
→ ④TubeShakePass（拿起试管震荡）。加酸滴入后固体样品+酸混合触发现象（气泡/沉淀/变色）。
动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest/d2s。
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
from .meta_actions import PickSpatula, ReturnSpatula, AcidPass, TubeShakePass
from .meta_actions.constants import GRIP_OPEN, SHAKE_CENTER_TCP, SPAT_XY, SCOOP_ANCHOR_Y


class D3SAcidReagentTaskController(TaskBaseController):
    """Composite controller: 整个 D3-S 实验 = 元动作的顺序执行。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[d3s] controller VERSION v1 (meta-actions: PickSpatula->ReturnSpatula->AcidPass->TubeShakePass, IK-driven)")
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

        # 元动作：①PickSpatula（挖粉倒粉）→ ②ReturnSpatula（药匙放回）→ ③AcidPass（吸酸滴酸）
        # → ④TubeShakePass（拿起试管震荡混合），顺序执行。酸滴管一次持握内循环
        # 「吸酸-滴酸」cfg.acid_cycles 遍（抓一次→多遍滴→放回一次，中途不松开；管内多积
        # 几滴酸液，液面逐滴升高；加酸滴入后固体样品+酸混合触发现象）。挖粉/震荡复用 d2s。
        acid_cycles = max(1, int(getattr(cfg, "acid_cycles", 1)))
        shake_cycles = max(1, int(getattr(cfg, "shake_cycles", 3)))
        self.meta_classes = [PickSpatula, ReturnSpatula, AcidPass, TubeShakePass]
        self.meta_names = [
            "S1 pick spatula + scoop powder + pour into tube (d2s, coords unchanged)",
            "S2 return spatula to rack (d2s)",
            f"A acid aspirate+drip into tube x{acid_cycles} (horizontal grip, ORIENT_FWD)",
            f"T shake tube x{shake_cycles} (d2s)",
        ]
        # AcidPass/TubeShakePass 带 cycles 参数，不能再用 [C(self.engine) for C in ...] 统一构造
        self.meta_actions = [
            # 药匙家用移到第一列第3排 (0.659,0.3209)；挖粉 y 基准仍锚定 d2s 原家用 0.3608
            # （否则 ⑧ ShiftYNeg 从 0.3209 起勺尖错过粉丘）。滴管第一列第5排、第二列清空。
            PickSpatula(self.engine, home=SPAT_XY, scoop_anchor_y=SCOOP_ANCHOR_Y),
            # lift_first=True：D3-S 家用离⑬倒粉位仅 2.3cm，回程水平段太短调不直（残留倾斜带进
            # 下探+扫过试管口穿模）→ 先原位提到安全高位调直、再高位横移、最后竖直下探（2026-08-26）。
            ReturnSpatula(self.engine, home=SPAT_XY, lift_first=True),
            AcidPass(self.engine, cycles=acid_cycles),
            # 震荡抬高到 1.18（d3s 自用，用户"震荡抬高一点再震荡"）：管底清架顶 12cm、
            # X 扫掠不碰试管旁的滴管（滴管已在远端孔）。
            TubeShakePass(self.engine, cycles=shake_cycles, shake_center=SHAKE_CENTER_TCP),
        ]
        self._meta_idx = 0
        self._h5_sample = 0
        self._start = True

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        if self.mode == "collect":
            self._meta_idx = 0
            self._start = True
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
            print("[d3s] all meta-actions done. success.")
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
                print(f"[d3s] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Scoop the solid powder sample with the spatula and pour it into the "
                "test tube in the rack, return the spatula, aspirate the acid reagent "
                "with the dropper and drip it into the same tube, then pick up the tube "
                "and shake it to mix")
