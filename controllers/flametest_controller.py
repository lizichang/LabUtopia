"""焰色反应控制器：按 V7 文档 C1 的 13 步驱动机械臂，一步一 phase。

v21 修正（修复多物体同时附着）：
  - task 层新增 _any_obj_attached()：一次只抓一个物体
  - task 层 _near_grasp z_thresh 从 0.03 收紧至 0.015
  - task 层 GRIP_CLOSED_THRESH 裕量从 2mm 收紧至 1mm
  - yaml max_steps 从 15000 增至 30000

v20 修正（试管架移入工作空间 + 夹爪开合 = 物体直径）：
  - joint7 = 物体直径 / 2（总宽 = 2×joint7 = 物体直径），不再夹到比物体更窄
  - 从 USD mesh extent 提取精确直径：
    * 表面皿边缘 4.2mm  瓶塞 25.2mm  滴管玻璃管 8mm
    * 火柴杆 3mm       铂丝手柄 8mm   灯帽 34mm
  - 铂丝放在试管架上（rotateY=120°，手柄穿过孔板，丝/环挂在下方）
  - 滴管竖直放在试管架孔中
  - 试管架从 x=0.50 移至 x=0.38（Franka 工作空间内）
  - WIRE_TIP_OFFSET=(0.095,0,-0.055)，FLAME_HOLD gripper=(0.265,0.18,1.040)
  - DROPPER_NOZZLE_OFFSET=(0,0,-0.06)，HCL_DIP gripper z=0.89
  - 安全过渡高度 H=1.15（夹爪距焰顶 >=14cm，臂绝不穿入火焰）
  - 火焰绕行：经过灯口上方区域时走 y>0.28 或 y<0.05 的绕行航路点
  - 所有落位符合实验室规则：瓶塞倒放桌面、滴管归架、铂丝归架、灯帽盖灯
"""
import numpy as np
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat

from .base_controller import BaseController as TaskBaseController


class FlameTestTaskController(TaskBaseController):
    """Composite controller: 13 phases, one V7 step per phase."""

    GRIP_OPEN = 0.04

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[flametest] controller VERSION v21 (one-obj-at-a-time, tight z_thresh, max_steps=30000)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        self._build_phases()
        self.phase_idx = 0
        self.seg_idx = 0
        self.seg_frame = 0
        self.arrived_frames = 0
        self._h5_sample = 0
        self._start = True

    def _init_infer_mode(self, cfg, robot):
        super()._init_infer_mode(cfg, robot)

    # ------------------------------------------------------------------
    # 13 phase 段表
    # ------------------------------------------------------------------
    def _build_phases(self):
        def seg(pos, gripper="hold", dwell=0, grip=None):
            """pos: 目标位置；gripper: open/close/hold；dwell: 停留帧数；grip: 闭合宽度(m)。"""
            return {
                "pos": None if pos is None else np.array(pos, dtype=float),
                "gripper": gripper,
                "dwell": int(dwell),
                "grip": grip,
            }

        # ---- per-object 夹爪闭合宽度（joint7 = 单指位移，米）----
        # 总宽 = 2 * joint7 = 物体直径（从 USD mesh extent 精确提取）
        # 物体直径（USD 实测）：
        #   表面皿边缘 4.2mm  瓶塞 25.2mm  滴管玻璃管 8mm
        #   火柴杆 3mm       铂丝手柄 8mm   灯帽 34mm
        GRIP_DISH    = 0.0021  # total 4.2mm, 表面皿边缘厚度 4.2mm
        GRIP_STOPPER = 0.0126  # total 25.2mm, 瓶塞直径 25.2mm
        GRIP_DROPPER = 0.004   # total 8mm, 滴管玻璃管直径 8mm
        GRIP_MATCH   = 0.0015  # total 3mm, 火柴杆直径 3mm
        GRIP_WIRE    = 0.004   # total 8mm, 铂丝手柄直径 8mm (handle radius=0.004 at grasp z)
        GRIP_CAP     = 0.017   # total 34mm, 灯帽直径 34mm
        GRIP_OPEN    = self.GRIP_OPEN

        # ---- 抓取点（与 task.GRASP_POINTS 精确对齐）----
        DISH_GRASP    = (0.32,   -0.22, 0.803)
        DISH_CENTER   = (0.20,    0.02, 0.803)
        STO_GRASP     = (0.12,    0.02, 0.877)
        STO_SIDE      = (0.16,    0.06, 0.877)
        DROP_GRASP    = (0.416,  -0.14, 0.872)  # 滴管在试管架上
        SSTO_GRASP    = (0.20,    0.12, 0.877)
        SSTO_SIDE     = (0.24,    0.08, 0.877)
        MATCH_GRASP   = (0.4585,  0.24, 0.803)
        WIRE_GRASP    = (0.417,  -0.14, 0.867)  # 铂丝在试管架上
        CAP_GRASP     = (0.46,    0.28, 0.815)

        # 安全过渡高度（夹爪 z，焰顶 z=1.004，安全裕量 14.6cm）
        H = 1.15

        # ---- 功能位置 ----
        # 滴管：DROPPER_NOZZLE_OFFSET=(0,0,-0.06)，管口 = gripper_z - 0.06
        # 液面 z=0.8415，管口需到 z=0.83（深入 1.15cm）→ gripper z=0.89
        HCL_DIP = (0.12, 0.02, 0.89)
        DISH_DRIP = (0.20, 0.02, 0.91)       # 管口 z=0.85，皿面上方 5cm 滴液

        # 火柴：MATCH_TIP_OFFSET=(-0.048,0,0)，头在 gripper -x 4.8cm
        # 灯口 (0.36,0.18,0.96) → gripper = (0.408,0.18,0.96)
        IGNITE = (0.408, 0.18, 0.96)

        # 铂丝：WIRE_TIP_OFFSET=(0.095,0,-0.055)，环中心 = gripper + (0.095,0,-0.055)
        # 酸液面 z≈0.806 → gripper z = 0.806+0.055 = 0.861
        ACID_DIP = (0.105, 0.02, 0.861)
        # 样品瓶内粉末 z≈0.845（瓶口 z=0.87 下 2.5cm）→ gripper z = 0.845+0.055 = 0.900
        POWDER_DIP = (0.105, 0.12, 0.900)
        # 外焰中部 (0.36,0.18,0.985) → gripper = (0.265,0.18,1.040)
        FLAME_HOLD = (0.265, 0.18, 1.040)
        # 火焰左侧高位接近：环中心在(0.33,0.16,1.01) → gripper=(0.235,0.16,1.065)
        FLAME_APPROACH = (0.235, 0.16, 1.065)
        # 冷却位：远离火焰
        COOL_POS = (0.40, 0.18, 1.18)
        # 火焰绕行航路点（南侧绕行，y=-0.05 避开灯口 y=0.18）
        # 当从 COOL_POS 返回酸液区时，先南下绕过火焰再西行
        FLAME_DETOUR_S = (0.36, -0.05, H)
        # 水柱冲洗：环中心在(0.40,-0.10,0.84) → gripper=(0.305,-0.10,0.895)
        WASH_POS = (0.305, -0.10, 0.895)
        DISH_WASH = (0.40, -0.10, 0.86)

        # 灯帽盖灭：CAP_HELD_OFFSET 后帽底到灯口
        # cap 高 4cm，中心 z=0.81；夹在 z=0.815，HELD_OFFSET z=-0.005
        # 帽底 = gripper_z - 0.005 - 0.02 = gripper_z - 0.025
        # 灯口 z=0.958 → gripper z = 0.983
        CAP_BURNER = (0.36, 0.18, 0.983)

        # 抓取前 settling 帧数（确保臂完全到位再合爪）
        SETTLE = 5

        self.phases = [
            # ================================================================
            # P1 取表面皿置实验台中央
            # ================================================================
            [
                seg((0.32, -0.22, H)),
                seg(DISH_GRASP),
                seg(DISH_GRASP, "hold", SETTLE),
                seg(DISH_GRASP, "close", 25, grip=GRIP_DISH),
                seg((0.20, 0.02, H)),
                seg(DISH_CENTER),
                seg(DISH_CENTER, "open", 25),
                seg((0.20, 0.02, H)),
            ],
            # ================================================================
            # P2 旋开稀盐酸瓶磨口塞，倒放桌面
            # ================================================================
            [
                seg((0.12, 0.02, H)),
                seg(STO_GRASP),
                seg(STO_GRASP, "hold", SETTLE),
                seg(STO_GRASP, "close", 25, grip=GRIP_STOPPER),
                seg((0.12, 0.02, 0.93)),
                seg((0.16, 0.06, 0.93)),
                seg(STO_SIDE),
                seg(STO_SIDE, "open", 25),
                seg((0.16, 0.06, H)),
            ],
            # ================================================================
            # P3 滴管吸盐酸 -> 滴 3 滴到表面皿 -> 滴管归架 -> 盖紧瓶塞
            # ================================================================
            [
                # --- 取滴管（从试管架）---
                seg((0.416, -0.14, H)),
                seg(DROP_GRASP),
                seg(DROP_GRASP, "hold", SETTLE),
                seg(DROP_GRASP, "close", 25, grip=GRIP_DROPPER),
                seg((0.416, -0.14, H)),
                # --- 伸入盐酸瓶吸液 ---
                seg((0.12, 0.02, H)),
                seg((0.12, 0.02, 0.93)),       # 管口在瓶口高度(z=0.87)
                seg(HCL_DIP, "hold", 25),       # 管口 z=0.83，深入液面 1.15cm
                seg((0.12, 0.02, 0.93)),       # 提回瓶口高度
                # --- 移到表面皿上方滴 3 滴 ---
                seg((0.20, 0.02, H)),
                seg(DISH_DRIP, "hold", 200),    # 管口 z=0.85，停留滴 3 滴
                # --- 滴管归架（放回试管架）---
                seg((0.416, -0.14, H)),
                seg(DROP_GRASP),
                seg(DROP_GRASP, "open", 25),
                seg((0.416, -0.14, H)),
                # --- 盖紧盐酸瓶塞 ---
                seg((0.16, 0.06, H)),
                seg(STO_SIDE),
                seg(STO_SIDE, "hold", SETTLE),
                seg(STO_SIDE, "close", 25, grip=GRIP_STOPPER),
                seg((0.16, 0.06, 0.93)),
                seg((0.12, 0.02, 0.93)),
                seg(STO_GRASP),
                seg(STO_GRASP, "open", 25),
                seg((0.12, 0.02, H)),
            ],
            # ================================================================
            # P4 火柴点燃本生灯（蓝焰）
            # ================================================================
            [
                seg((0.4585, 0.24, H)),
                seg(MATCH_GRASP),
                seg(MATCH_GRASP, "hold", SETTLE),
                seg(MATCH_GRASP, "close", 25, grip=GRIP_MATCH),
                seg((0.4585, 0.24, 0.90)),
                seg(IGNITE, "hold", 20),         # 火柴头到达灯口点燃
                seg((0.4585, 0.24, 0.90)),
                seg(MATCH_GRASP),
                seg(MATCH_GRASP, "open", 25),
                seg((0.4585, 0.24, H)),
            ],
            # ================================================================
            # P5 取铂丝 -> 尖端浸入稀盐酸（表面皿中）
            # ================================================================
            [
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((0.417, -0.14, H)),
                seg((0.105, 0.02, H)),
                seg(ACID_DIP, "hold", 25),       # 尖端接触皿内酸液
                seg((0.105, 0.02, H)),
            ],
            # ================================================================
            # P6 外焰灼烧（60 帧，清洗铂丝，无特征色）
            # ================================================================
            [
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
            ],
            # ================================================================
            # P7 反复蘸酸+灼烧 3 次
            # ================================================================
            [
                seg(FLAME_DETOUR_S),               # 绕行：从 P6 COOL_POS 南下避开火焰
                seg((0.105, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.105, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg(FLAME_DETOUR_S),               # 绕行：南下避开火焰
                seg((0.105, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.105, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg(FLAME_DETOUR_S),               # 绕行：南下避开火焰
                seg((0.105, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.105, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
            ],
            # ================================================================
            # P8 冷却 5s
            # ================================================================
            [
                seg(COOL_POS, "hold", 300),
            ],
            # ================================================================
            # P9 打开样品瓶 -> 铂丝蘸粉末
            # ================================================================
            [
                # 放回铂丝（到试管架）
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((0.417, -0.14, H)),
                # 开样品瓶塞
                seg((0.20, 0.12, H)),
                seg(SSTO_GRASP),
                seg(SSTO_GRASP, "hold", SETTLE),
                seg(SSTO_GRASP, "close", 25, grip=GRIP_STOPPER),
                seg((0.20, 0.12, 0.93)),
                seg((0.24, 0.08, 0.93)),
                seg(SSTO_SIDE),
                seg(SSTO_SIDE, "open", 25),
                seg((0.24, 0.08, H)),
                # 再取铂丝（从试管架）
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((0.417, -0.14, H)),
                # 蘸粉末
                seg((0.105, 0.12, H)),
                seg((0.105, 0.12, 0.96)),    # 尖端在瓶口上方
                seg(POWDER_DIP, "hold", 20),  # 尖端进入瓶内粉末
                seg((0.105, 0.12, 0.96)),    # 提回瓶口上方
                seg((0.105, 0.12, H)),
            ],
            # ================================================================
            # P10 灼烧 2-5s（受染，黄色光晕出现在铂丝尖端）
            # ================================================================
            [
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 300),
                seg(COOL_POS),
            ],
            # ================================================================
            # P11 灯帽盖灭
            # ================================================================
            [
                # 放回铂丝（到试管架）
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((0.417, -0.14, H)),
                # 取灯帽
                seg((0.46, 0.28, H)),
                seg(CAP_GRASP),
                seg(CAP_GRASP, "hold", SETTLE),
                seg(CAP_GRASP, "close", 25, grip=GRIP_CAP),
                seg((0.46, 0.28, 1.05)),
                seg(CAP_BURNER, "hold", 20),   # 帽底罩住灯口
                seg((0.36, 0.18, 1.05), "open"),
            ],
            # ================================================================
            # P12 冲洗铂丝 -> 归位
            # ================================================================
            [
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((0.417, -0.14, H)),
                seg((0.305, -0.10, H)),
                seg(WASH_POS, "hold", 80),     # 尖端在水柱下冲洗
                seg((0.305, -0.10, H)),
                seg((0.417, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((0.417, -0.14, H)),
            ],
            # ================================================================
            # P13 表面皿冲洗归位
            # ================================================================
            [
                seg((0.20, 0.02, H)),
                seg(DISH_CENTER),
                seg(DISH_CENTER, "hold", SETTLE),
                seg(DISH_CENTER, "close", 25, grip=GRIP_DISH),
                seg((0.20, 0.02, H)),
                seg(DISH_WASH, "hold", 80),    # 表面皿在水柱下冲洗
                seg((0.32, -0.22, H)),
                seg(DISH_GRASP),
                seg(DISH_GRASP, "open", 25),
                seg((0.32, -0.22, H)),
            ],
        ]
        self.phase_names = [
            "P1 dish to center", "P2 open hcl stopper", "P3 drip 3 drops",
            "P4 ignite burner", "P5 dip wire in acid", "P6 burn (no color)",
            "P7 repeat dip+burn x3", "P8 cool 5s", "P9 dip powder",
            "P10 burn 2-5s (stain)", "P11 extinguish", "P12 rinse & return wire",
            "P13 rinse & return dish",
        ]

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.phase_idx = 0
        self.seg_idx = 0
        self.seg_frame = 0
        self.arrived_frames = 0
        self.hold_frames = 0
        self._h5_sample = 0
        self._start = True
        if self.mode == "collect":
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
        if self.phase_idx >= len(self.phases):
            print("[flametest] all 13 phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        seg = self.phases[self.phase_idx][self.seg_idx]
        action = self._execute_seg(seg, state)

        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            self.data_collector.cache_step(
                camera_images=state["camera_data"],
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )

        return action, False, False

    def _execute_seg(self, seg, state):
        joints = state["joint_positions"]
        gripper_pos = state["gripper_position"]

        if self._start:
            self._start = False
            target = [None] * joints.shape[0]
            target[7] = self.GRIP_OPEN / get_stage_units()
            target[8] = self.GRIP_OPEN / get_stage_units()
            return ArticulationAction(joint_positions=target)

        # Determine gripper value
        gripper_val = None
        if seg["gripper"] == "open":
            gripper_val = self.GRIP_OPEN
        elif seg["gripper"] == "close":
            gripper_val = seg["grip"] if seg["grip"] is not None else 0.02

        if seg["pos"] is not None:
            rmp_action = self.rmp_controller.forward(
                target_end_effector_position=seg["pos"],
                target_end_effector_orientation=self.orient,
            )
            target_jp = np.concatenate([
                np.asarray(rmp_action.joint_positions, dtype=float),
                [np.nan, np.nan],
            ])
            if gripper_val is not None:
                target_jp[7] = gripper_val / get_stage_units()
                target_jp[8] = gripper_val / get_stage_units()
            target = ArticulationAction(joint_positions=target_jp)
        else:
            target = [None] * joints.shape[0]
            if gripper_val is not None:
                target[7] = gripper_val / get_stage_units()
                target[8] = gripper_val / get_stage_units()
            target = ArticulationAction(joint_positions=target)

        # Segment completion
        self.seg_frame += 1
        seg_done = False
        if seg["pos"] is None:
            seg_done = self.seg_frame >= max(seg["dwell"], 25)
        else:
            dist = np.linalg.norm(gripper_pos - seg["pos"])
            if dist < 0.010:
                self.arrived_frames += 1
            else:
                self.arrived_frames = 0
            if self.arrived_frames >= 2:
                self.hold_frames += 1
                if self.hold_frames >= seg["dwell"]:
                    seg_done = True
            else:
                self.hold_frames = 0
            if not seg_done and self.seg_frame >= seg["dwell"] + 400:
                seg_done = True
                print(f"[flametest] seg force-done t={self.seg_frame} (dwell={seg['dwell']})")

        if seg_done:
            self.seg_frame = 0
            self.arrived_frames = 0
            self.hold_frames = 0
            self.seg_idx += 1
            if self.seg_idx >= len(self.phases[self.phase_idx]):
                self.seg_idx = 0
                print(f"[flametest] phase {self.phase_idx} done: {self.phase_names[self.phase_idx]}")
                self.phase_idx += 1

        if isinstance(target, ArticulationAction):
            return target
        return ArticulationAction(joint_positions=target)

    def _step_infer(self, state):
        if self.phase_idx >= len(self.phases):
            self.reset_needed = True
            return None, True, self._last_success

        language_instruction = self.get_language_instruction()
        state["language_instruction"] = language_instruction
        action = self.inference_engine.step_inference(state)

        return action, False, self.is_success()

    def is_success(self):
        return self.phase_idx >= len(self.phases)

    def get_language_instruction(self):
        return ("Place the watch glass at the center, open the dilute hydrochloric "
                "acid bottle, drip 2-3 drops with the dropper, ignite the bunsen "
                "burner, dip the platinum wire in the acid, burn it in the outer "
                "flame 3-4 times until no characteristic color, cool for 5 s, dip "
                "the solid sample powder, burn for 2-5 s to observe the flame "
                "color, extinguish the flame, rinse the wire and the watch glass")
