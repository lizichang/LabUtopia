"""B1 酒精灯加热（固体样品）控制器：本批次 = 八个元动作顺序执行。

用户 2026-08-27 逐字：「先写咬粉末咬进试管里面，然后拿起酒精灯盖儿放到一边儿，再拿起火柴点燃
酒精灯，这几个过程先只写这些我来验收。」→ 本控制器编排：挖粉倒粉 → 放回药匙 → 开灯帽 →
点火。2026-08-27 追加第⑤步（用户逐字：「先夹住试管，水平横着夹住（跟夹药匙的方法一样）」）
→ 点完火后水平横夹试管提出架顶；用户再批「最后动作就拿起试管，不要再做其他移动动作了，
就先夹起试管」，又批「试管提出之后法兰旋转-100度」→ 改 -95°（「法兰改为旋转-95度吧」）
→ ⑥ 法兰只动 joint7 转 -95° 试管竖直→近水平，再批「现在加动作水平往负x方向移动（yz还有朝向
都不变）让爪子x坐标对齐火焰的x坐标」→ ⑦ 水平 -X 移到火焰上方；2026-08-28 再批「再加动作，
竖直z方向降低，让爪子在z坐标比火焰小2cm（过程中yx还有朝向不变，只有z变）」→ ⑧ 竖直下降
（爪子 z=火焰 z 0.9182−2cm=0.8982，x/y/朝向不变）；再批「然后加动作，只水平-y移动15cm，xz朝向
不要变」→ ⑨ 水平 -Y 15cm（后改「最后一步水平移动改为11cm」→ y 0.241→0.131，x/z/朝向不变）。
2026-08-28 追加预热/持续加热/放回（用户逐字：「现在需要加的动作是来回预热，现在加动作，在y的
方向上来回移动2cm，来回移动5次速度不要太快，最后持续加热持续8s，最后放回试管，先写完这些动作。」）
→ ⑥ PreheatTubePass（y 向 ±2cm 往复 5 次慢速预热，保持 -95° 倾斜）→ ⑦ HeatHoldPass（外焰
持续加热 8s = hold 480 帧）→ ⑧ ReturnTubePass（+Y 退开 → 升 z → +X 回架 → 法兰转回竖直 →
下降放回 → 松爪 → 抬走）。熄灭留待验收后接续。

与 d2s/d3s 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。

注册顺序：
  ①PickSpatula（挖粉倒粉，home=None → d2s SPAT_XY，B1 场景复刻 D2S 坐标逐字）
  ②ReturnSpatula（药匙放回，home=None → d2s SPAT_XY）
  ③OpenCapPass（拿起酒精灯盖放到一边：取帽 → 提起 → 横移 → 落台面 → 松爪归位）
  ④LightFlamePass（取火柴点燃酒精灯：抓杆 → 触灯芯 → 点燃 → 放回火柴；B1 无温度模型，
    flame_lit 置位即 reveal 火焰）
  ⑤PickTubePass（水平横夹试管 ORIENT_FWD → 提出架顶 → 法兰转 -95° 倾斜近水平 → 水平 -X
    移到火焰上方，爪子 x 对齐火焰 x → 竖直下降火焰下 2cm → 水平 -Y 15cm；用户「就先夹起试管」
    「试管提出之后法兰旋转-100度」改 -95°「法兰改为旋转-95度吧」「现在加动作水平往负x方向移动
    让爪子x坐标对齐火焰的x坐标」「让爪子在z坐标比火焰小2cm（过程中yx还有朝向不变，只有z变）」
    「只水平-y移动15cm，xz朝向不要变」）
  ⑥PreheatTubePass（预热：HeatSweepAction 在 TUBE_AT_FLAME_2 附近 y 向 ±2cm 正弦往复 5 次，
    period 150=2.5s/来回慢速，首帧采样实际朝向保持 -95° 倾斜；用户「在y的方向上来回移动2cm，
    来回移动5次速度不要太快」）
  ⑦HeatHoldPass（持续加热 8s：纯 hold HEAT_HOLD_FRAMES=480 帧，试管停外焰集中加热；用户「最后
    持续加热持续8s」；B1 无温度模型 → 无加热现象）
  ⑧ReturnTubePass（放回试管：PickTubePass 逆过程 = +Y 退开火焰 → 升 z → +X 回架上方 → 法兰转回
    竖直（FlangeRollTubeAction(angle=+95°)）→ 下降放回抓点 → 松爪（task 近抓点+开爪 → released
    → 写回静置矩阵）→ 抬走；用户「最后放回试管」）

动作级契约（grip 每帧发送、到达冻结、dwell、跨元动作 grip_target 传播）沿用 flametest/d2s。
跨元动作 grip_target 传播：①② 之间（药匙放回后爪子张开）与 ③④ 之间（灯帽释放后爪子张开）
由各元动作末尾 GripAction 置 GRIP_OPEN；④→⑤ 之间 LightFlamePass 末尾 GripAction 置
GRIP_OPEN，⑤ PickTubePass 首帧 grip_target=GRIP_OPEN（grip_target 在 _step_collect 元动作
切换处逐项传播，见 _step_collect）。
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
from .meta_actions import (PickSpatula, ReturnSpatula, OpenCapPass, LightFlamePass, PickTubePass,
                           PreheatTubePass, HeatHoldPass, ReturnTubePass)
from .meta_actions.constants import GRIP_OPEN


class B1AlcoholHeatSolidTaskController(TaskBaseController):
    """Composite controller: B1 本批次 = 挖粉倒粉 → 放回药匙 → 开灯帽 → 点火 → 拿试管提出架顶
    → y 向 ±2cm 往复预热 ×5 → 外焰持续加热 8s → 放回试管。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b1] controller VERSION v3 (meta-actions: PickSpatula->ReturnSpatula->OpenCapPass->LightFlamePass->PickTubePass->PreheatTubePass->HeatHoldPass->ReturnTubePass, IK-driven)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器（同 flametest/d2s/d3s）：精确关节控制替代 RMP
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self.engine = IkMotionEngine(solver, self.orient, ik_home)

        # 元动作：
        # ①PickSpatula（挖粉倒粉）→ ②ReturnSpatula（药匙放回）→ ③OpenCapPass（开灯帽放一边）
        # → ④LightFlamePass（取火柴点燃）→ ⑤PickTubePass（水平横夹试管移到火焰上方）。
        # 挖粉用 d2s 默认 home（None → d2s SPAT_XY (0.6993,0.3608)）：用户先决条件「表面皿、粉末
        # 和机械臂坐标一定要复刻 D2S，粉末才挖得准」，B1 场景已逐字复刻 D2S，故不传 home/参数。
        self.meta_classes = [PickSpatula, ReturnSpatula, OpenCapPass, LightFlamePass, PickTubePass,
                             PreheatTubePass, HeatHoldPass, ReturnTubePass]
        self.meta_names = [
            "P1 pick spatula + scoop powder + pour into tube (d2s, coords unchanged)",
            "P2 return spatula to rack (d2s)",
            "C open alcohol lamp cap and set it aside (pure-translation cap hold)",
            "L pick up match and light the alcohol lamp (match tip to wick, direct flame reveal)",
            "T pick test tube (horizontal ORIENT_FWD, same as spatula), lift it out of the rack, flange roll -95° to tilt it horizontal, then move -X so the gripper x aligns with the flame x, then lower -Y to the outer flame",
            "H1 preheat: y-axis ±2cm reciprocation ×5 (slow), keeping the -95° tilt",
            "H2 sustain heat 8s at the outer flame (hold 480 frames)",
            "R return test tube: +Y off the flame, lift, +X over the rack, flange roll back to vertical, lower into rack hole, release, lift away",
        ]
        self.meta_actions = [
            PickSpatula(self.engine),     # home=None → d2s SPAT_XY（B1 复刻 D2S 坐标）
            ReturnSpatula(self.engine),   # home=None → d2s SPAT_XY
            OpenCapPass(self.engine),
            LightFlamePass(self.engine),
            PickTubePass(self.engine),
            PreheatTubePass(self.engine),
            HeatHoldPass(self.engine),
            ReturnTubePass(self.engine),
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
            print("[b1] all meta-actions done. success.")
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
                print(f"[b1] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
                "test tube in the rack, return the spatula, open the alcohol lamp cap "
                "and set it aside, then pick up the match and light the alcohol lamp, "
                "then grasp the test tube horizontally, lift it out of the test tube "
                "rack, tilt it horizontal via a -95° flange roll, and move it above "
                "the alcohol lamp flame so the gripper x aligns with the flame x, "
                "then preheat by reciprocating ±2cm in the y direction 5 times, "
                "then hold at the outer flame for 8 seconds, "
                "then return the test tube to the rack")
