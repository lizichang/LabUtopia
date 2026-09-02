"""B5 熔点测定（提勒管法）控制器：装样段 = 五个元动作「夹封口端拎起→插粉丘→放回→夹开口端拎起→震实」。

用户 2026-08-31 逐字：「算了还是换一种方法吧，同时把表面皿的位置放回去；重新写：首先你夹
这个毛细管还是以同样的方式夹但是夹的位置变了……夹起来的时候因为重力水平细管就自动变成竖直
了就像把它拎起来了……然后再毛细管放回桌面（有倒在桌面上变成水平的）……第二次夹毛细管的 -x
端，还是拎起来，这样子自动数值就可以上下快速移动把粉末从一端搞到另一端」→ 弃程序化旋转，
改夹端部拎起自动竖直，编排五个元动作：
  ① PickCapillarySealedEnd  夹封口端(-X)拎起 → 开口端朝下（蘸粉准备，绕 Y +90°）
  ② DipCapillaryIntoPowder  水平移到粉丘上方→竖直下探开口端沉入粉丘 5mm 蘸粉
  ③ ReturnCapillaryToTable  蘸粉后放回桌面，毛细管倒成水平（松爪）
  ④ PickCapillaryOpenEnd    夹开口端(+X)拎起 → 封口端朝下（抖粉准备，绕 Y -90°）
  ⑤ TampCapillary           保持封口端朝下，竖直方向上下快速来回 10 次震实
挂温度计/入提勒管加热/观察熔点留待验收后接续。

与 b1/b2/d2s 同构分层（Lula IK + 元动作组合，RMP 对低 z 下探发散，弃用 RMP 用 IkMotionEngine）：
  - atomic_actions/flametest/（IK 原子动作）
  - meta_actions/（一个 v11 步骤 = 一个元动作，一类一文件）
  - 本控制器：实例化元动作按序 forward()，全部完成 → success。
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
                           StickCapillaryToBulb, PickThermometer, InsertThermometerIntoTube)
from .meta_actions.constants import GRIP_OPEN


class B5MeltingPointTaskController(TaskBaseController):
    """Composite controller: B5 本批次 = 拿起毛细管（熔点管，照 b2 火柴同款）。"""

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    # ------------------------------------------------------------------
    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[b5] controller VERSION v11 (meta-actions: PickSealedEnd + Dip + ReturnToTable "
              "+ PickOpenEnd + Tamp + ReturnAfterTamp + PickMiddle + DipOil + ReturnAfterOil "
              "+ PickSealedEnd(2) + StickVertical + PickThermo + InsertThermo, capillary pivot "
              "swing x3 then middle matrix hold then thermometer matrix hold)")
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

        # 元动作：①夹封口端拎起 → ②插粉丘蘸粉 → ③放回桌面 → ④夹开口端拎起 → ⑤竖直震实
        # → ⑥震实后放回 → ⑦毛细管中部水平夹起 → ⑧封口端蘸石蜡油 → ⑧'放回桌面 →
        # ⑨'再夹封口端拎起竖直 → ⑩封口端竖贴温度计泡 → ⑪抓温度计旋转竖直 → ⑫抬高对齐竖直插提勒管。
        # （grip 逐动作传播；三次夹取/中部水平夹取 task 侧按 phase 切夹点，温度计矩阵持握泡朝下）
        self.meta_classes = [PickCapillarySealedEnd, DipCapillaryIntoPowder, ReturnCapillaryToTable,
                             PickCapillaryOpenEnd, TampCapillary, ReturnCapillaryAfterTamp,
                             PickCapillaryMiddle, DipCapillaryInOil, ReturnCapillaryAfterOil,
                             PickCapillarySealedEnd, StickCapillaryToBulb,
                             PickThermometer, InsertThermometerIntoTube]
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
            "raise the thermometer high, align it over the thiele tube mouth, and descend vertically until the stopper seals the mouth",
        ]
        self.meta_actions = [PickCapillarySealedEnd(self.engine),
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
                             InsertThermometerIntoTube(self.engine)]
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
            print("[b5] all meta-actions done. success.")
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
                print(f"[b5] meta {self._meta_idx} done: {self.meta_names[self._meta_idx]}")
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
        return ("Grab the sealed end of the capillary tube and lift it so it hangs vertical, "
                "dip its open end into the powder mound, lay it back on the table, then grab "
                "the open end and lift it, tamping the powder to the sealed end by moving up "
                "and down rapidly 10 times, lay it back on the table, then grab the middle of "
                "the capillary keeping it horizontal, dip its sealed end into the paraffin oil, "
                "lay it back on the table, then grab the sealed end and lift it vertical again, "
                "press the sealed end against the thermometer bulb so it sticks, then grab the "
                "horizontal thermometer stem, lift it, and rotate it vertical so the bulb "
                "points down, raise it high, align it over the thiele tube mouth, and descend "
                "vertically until the stopper seals the mouth")
