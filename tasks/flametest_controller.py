"""焰色反应控制器：按 V7 文档 C1 的 13 步驱动机械臂，一步一 phase。

每个 phase 是一段运动序列（段 = 目标位置 + 夹爪指令 + 停留帧数）：
  - pos 段：RMP 移动到目标（到位 <0.012 后稳定 2 帧即推进）
  - dwell 段：到位后停留 N 帧（任务在此期间完成事件检测：蘸酸/灼烧/滴液/点火/冷却/冲洗）
  - 夹爪段：闭爪（attached 检测）/ 开爪（释放检测），持续 25 帧

13 phase（与 V7 文档 C1 一一对应）：
  P1  取表面皿置中央（(0.32,-0.22) -> 中央 (0.20,0.02)）
  P2  旋开稀盐酸瓶磨口塞（抓塞 -> 放瓶旁）
  P3  滴管吸盐酸 -> 表面皿上方滴 3 滴 -> 放回 -> 盖紧瓶塞归位
  P4  点燃本生灯（火柴头在灯口停留 15 帧，任务显示蓝色火焰）
  P5  抓铂丝 -> 尖端浸入表面皿稀盐酸
  P6  尖端置于外焰灼烧 60 帧（无特征色）
  P7  反复蘸酸+灼烧 3 次（移动时尖端朝下、保持台面 10cm 以上：gripper z >= 1.00）
  P8  冷却约 5s（300 帧，远离火焰的安全位）
  P9  打开样品瓶 -> 尖端蘸取粉末
  P10 蘸样尖端外焰灼烧 150 帧（任务显示受染色 flame_stain_{color}）
  P11 取灯帽盖到灯口（任务熄灭火焰）
  P12 尖端在洗瓶下冲洗 60 帧 -> 铂丝归位
  P13 表面皿冲洗 -> 归位

铂丝几何（rotateY150° 斜置）：尖端相对夹爪 (0.05475, 0, -0.09303)，
因此夹爪全程在火焰侧上方安全区（z >= 1.00 时尖端 z >= 0.907 > 台面 10cm），
只有尖端伸入火焰，夹爪不与火焰重叠。
"""
import numpy as np
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat

from .base_controller import BaseController as TaskBaseController


class FlameTestTaskController(TaskBaseController):
    """Composite controller: 13 phases, one V7 step per phase."""

    def __init__(self, cfg, robot):
        """Initialize the flame test task controller.

        Args:
            cfg: Configuration object containing controller settings.
            robot: Robot instance to control.
        """
        super().__init__(cfg, robot)

    def _init_collect_mode(self, cfg, robot):
        """Initialize controller for data collection mode."""
        super()._init_collect_mode(cfg, robot)
        print("[flametest] controller VERSION v12.1 (13-phase, hold-dwell, h5 15Hz)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        self._build_phases()
        self.phase_idx = 0
        self.seg_idx = 0
        self.seg_frame = 0
        self.arrived_frames = 0
        self._h5_sample = 0
        self._start = True

    def _init_infer_mode(self, cfg, robot):
        """Initialize controller for inference mode."""
        super()._init_infer_mode(cfg, robot)

    # ------------------------------------------------------------------
    # 13 phase 段表
    # ------------------------------------------------------------------
    def _build_phases(self):
        def seg(pos, gripper="hold", dwell=0):
            """pos: 目标位置（None=纯夹爪段）；gripper: open/close/hold；dwell: 到位后停留帧数"""
            return {
                "pos": None if pos is None else np.array(pos, dtype=float),
                "gripper": gripper,
                "dwell": int(dwell),
            }

        # 关键坐标（与 lab_flametest.usd / FlameTestTask 一致）
        DISH_GRASP = (0.32, -0.22, 0.807)
        DISH_CENTER = (0.20, 0.02, 0.807)
        STO_GRASP = (0.12, 0.02, 0.875)
        STO_SIDE = (0.16, 0.06, 0.875)
        DROP_GRASP = (0.12, -0.10, 0.90)
        HCL_DIP = (0.12, 0.02, 0.94)        # 管口入瓶吸酸（管口=夹爪-0.10z）
        DISH_DRIP = (0.20, 0.02, 0.93)      # 管口在皿上方滴液
        MATCH_GRASP = (0.455, 0.24, 0.806)
        IGNITE = (0.413, 0.18, 0.96)        # 火柴头到灯口
        WIRE_GRASP = (0.228, -0.14, 0.898)
        ACID_DIP = (0.14525, 0.02, 0.90503)  # 尖端入液面
        FLAME_APPROACH = (0.22, 0.16, 1.03)  # 火焰斜下方接近点（tip z=0.937 未入焰，分段逼近防 RMP 过冲振荡）
        FLAME_HOLD = (0.30525, 0.18, 1.09303)  # 尖端在外焰中部（tip z=1.00，检测区 0.955-1.045 中央，±4.5cm 裕量）
        COOL_POS = (0.45, 0.18, 1.05)        # 冷却/撤离安全位
        SSTO_GRASP = (0.20, 0.12, 0.875)
        SSTO_SIDE = (0.24, 0.08, 0.875)
        POWDER_DIP = (0.14525, 0.12, 0.94803)  # 尖端入瓶蘸粉
        CAP_GRASP = (0.46, 0.28, 0.83)
        CAP_BURNER = (0.36, 0.18, 0.96)      # 灯帽盖灯口
        WASH_POS = (0.34525, -0.10, 0.94303)  # 尖端在喷嘴下冲洗
        DISH_WASH = (0.40, -0.10, 0.86)      # 皿在喷嘴下
        DISH_BACK = (0.32, -0.22, 0.807)     # 皿归位
        H = 1.00                             # 安全高度（尖端 z>=0.907 台面10cm+）

        self.phases = [
            # P1 取表面皿置中央
            [
                seg((0.32, -0.22, 0.86)),
                seg(DISH_GRASP),
                seg(DISH_GRASP, "close", 25),
                seg((0.20, 0.02, 0.86)),
                seg(DISH_CENTER),
                seg(DISH_CENTER, "open", 25),
                seg((0.20, 0.02, 0.85)),
            ],
            # P2 旋开稀盐酸瓶磨口塞（抓塞放瓶旁）
            [
                seg((0.12, 0.02, 0.89)),
                seg(STO_GRASP),
                seg(STO_GRASP, "close", 25),
                seg((0.12, 0.02, 0.90)),
                seg((0.16, 0.06, 0.90)),
                seg(STO_SIDE),
                seg(STO_SIDE, "open", 25),
                seg((0.16, 0.06, 0.85)),
            ],
            # P3 吸盐酸 -> 滴 3 滴 -> 放回滴管 -> 盖紧瓶塞归位
            [
                seg((0.12, -0.10, 0.95)),
                seg(DROP_GRASP),
                seg(DROP_GRASP, "close", 25),
                seg((0.12, 0.02, 0.95)),
                seg(HCL_DIP, "hold", 20),      # 管口入液吸酸
                seg((0.12, 0.02, H)),
                seg(DISH_DRIP, "hold", 200),   # 管口在皿上方滴 3 滴（检测窗口 20+3×30=110 帧）
                seg(DROP_GRASP),
                seg(DROP_GRASP, "open", 25),   # 放回滴管
                seg((0.16, 0.06, 0.89)),
                seg(STO_SIDE),
                seg(STO_SIDE, "close", 25),    # 抓瓶塞
                seg((0.12, 0.02, 0.89)),
                seg(STO_GRASP),
                seg(STO_GRASP, "open", 25),    # 盖回瓶口
                seg((0.12, 0.02, 0.90)),
            ],
            # P4 点燃本生灯（火柴头在灯口停留 15 帧 -> 蓝焰）
            [
                seg((0.455, 0.24, 0.86)),
                seg(MATCH_GRASP),
                seg(MATCH_GRASP, "close", 25),
                seg((0.455, 0.24, 0.90)),
                seg(IGNITE, "hold", 15),
                seg((0.455, 0.24, 0.90)),
                seg(MATCH_GRASP),
                seg(MATCH_GRASP, "open", 25),
                seg((0.455, 0.24, 0.85)),
            ],
            # P5 抓铂丝 -> 尖端浸入表面皿稀盐酸
            [
                seg((0.228, -0.14, 0.95)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "close", 25),
                seg((0.228, -0.14, H)),
                seg((0.14525, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.14525, 0.02, H)),
            ],
            # P6 尖端外焰灼烧（60 帧，无特征色）
            [
                seg(FLAME_APPROACH, "hold", 30),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
            ],
            # P7 反复蘸酸+灼烧 3 次（移动高度 >= H，尖端保持台面 10cm 以上）
            [
                seg((0.14525, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.14525, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 30),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg((0.14525, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.14525, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 30),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg((0.14525, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.14525, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 30),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
            ],
            # P8 冷却约 5s（300 帧）
            [
                seg(COOL_POS, "hold", 300),
            ],
            # P9 打开样品瓶 -> 尖端蘸取粉末
            # （单臂约束：先放下已冷却的铂丝再开瓶，开完瓶重新抓取蘸粉；
            #   避免抓瓶塞时 wire 尖端穿桌）
            [
                seg((0.228, -0.14, H)),        # 带 wire 回 rest 上方
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),   # 放下 wire（回到抓取位张开 -> 释放）
                seg((0.228, -0.14, 0.85)),
                seg((0.20, 0.12, 0.89)),
                seg(SSTO_GRASP),
                seg(SSTO_GRASP, "close", 25),
                seg((0.20, 0.12, 0.90)),
                seg((0.24, 0.08, 0.90)),
                seg(SSTO_SIDE),
                seg(SSTO_SIDE, "open", 25),
                seg((0.228, -0.14, 0.90)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "close", 25),  # 重新抓 wire
                seg((0.228, -0.14, H)),
                seg((0.14525, 0.12, H)),
                seg(POWDER_DIP, "hold", 15),
                seg((0.14525, 0.12, H)),
            ],
            # P10 蘸样尖端外焰灼烧（受染检测 150 帧 + 移动期，段总帧 300）
            [
                seg(FLAME_APPROACH, "hold", 30),
                seg(FLAME_HOLD, "hold", 300),
                seg(COOL_POS),
            ],
            # P11 取灯帽盖灭（先放下 wire 再抓帽，避免抓帽时 wire 尖端穿桌）
            [
                seg((0.228, -0.14, H)),        # 带 wire 回 rest 上方
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),   # 放下 wire（回到抓取位张开 -> 释放归位）
                seg((0.228, -0.14, 0.85)),
                seg((0.46, 0.28, 0.88)),
                seg(CAP_GRASP),
                seg(CAP_GRASP, "close", 25),
                seg((0.46, 0.28, 0.92)),
                seg(CAP_BURNER, "hold", 15),
                seg((0.36, 0.18, 0.90), "open"),
            ],
            # P12 重抓 wire -> 尖端冲洗 -> 铂丝归位
            [
                seg((0.228, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "close", 25),  # 重抓 wire
                seg((0.228, -0.14, H)),
                seg((0.34525, -0.10, H)),
                seg(WASH_POS, "hold", 60),     # 尖端在喷嘴下冲洗（检测 5 帧 -> 水柱）
                seg((0.34525, -0.10, H)),
                seg((0.228, -0.14, H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),   # 放下 wire 归位
                seg((0.228, -0.14, 0.85)),
            ],
            # P13 表面皿冲洗 -> 归位
            [
                seg((0.20, 0.02, 0.86)),
                seg(DISH_CENTER),
                seg(DISH_CENTER, "close", 25),
                seg(DISH_WASH, "hold", 60),
                seg((0.32, -0.22, 0.86)),
                seg(DISH_BACK),
                seg(DISH_BACK, "open", 25),
                seg((0.32, -0.22, 0.85)),
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
        """Reset controller state and phases."""
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
        """Execute one step of control.

        Args:
            state: Current state dictionary containing sensor data and robot state.

        Returns:
            Tuple containing action, done flag, and success flag.
        """
        self.state = state
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state):
        """Execute collection mode step."""
        if self.phase_idx >= len(self.phases):
            # 全部 13 步完成
            print("[flametest] all 13 phases done. success.")
            self.data_collector.write_cached_data(state["joint_positions"][:-1])
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        seg = self.phases[self.phase_idx][self.seg_idx]
        action = self._execute_seg(seg, state)

        # h5 缓存降采样：每 4 帧缓存 1 帧（15Hz，视频仍 60fps）。
        # 全帧缓存（3×512px×约5min≈30GB）在 write_cached_data 的 np.array 转换与
        # 进程池 pickle 传输时内存翻倍，62GB 服务器实测被 OOM SIGKILL（无 traceback）。
        self._h5_sample = (self._h5_sample + 1) % 4
        if self._h5_sample == 0 and "camera_data" in state:
            self.data_collector.cache_step(
                camera_images=state["camera_data"],
                joint_angles=state["joint_positions"][:-1],
                language_instruction=self.get_language_instruction(),
            )

        return action, False, False

    def _execute_seg(self, seg, state):
        """Execute one segment; advance to the next when done."""
        joints = state["joint_positions"]
        gripper_pos = state["gripper_position"]

        if self._start:
            self._start = False
            target = [None] * joints.shape[0]
            target[7] = 0.04 / get_stage_units()
            target[8] = 0.04 / get_stage_units()
            return ArticulationAction(joint_positions=target)

        # 目标 = RMP 位置 + 夹爪指令。
        # rmp_controller.forward 返回的 ArticulationAction 只含 7 个手臂关节（RMP 不控制
        # 夹爪），而 Franka 共 9 DOF（7 手臂 + 2 夹爪）。统一扩成 9 维 joint_positions：
        # 手臂 0-6 用 RMP 结果，夹爪 7/8 按指令写入；nan 在 apply_action 中保持当前值。
        if seg["pos"] is not None:
            rmp_action = self.rmp_controller.forward(
                target_end_effector_position=seg["pos"],
                target_end_effector_orientation=self.orient,
            )
            target_jp = np.concatenate([
                np.asarray(rmp_action.joint_positions, dtype=float),
                [np.nan, np.nan],
            ])
            if seg["gripper"] in ("open", "close"):
                gripper_val = 0.04 if seg["gripper"] == "open" else 0.02
                # 闭合目标 0.02 < 任务检测阈值 0.025（否则稳态开合停在 0.03 永不触发夹紧）
                target_jp[7] = gripper_val / get_stage_units()
                target_jp[8] = gripper_val / get_stage_units()
            target = ArticulationAction(joint_positions=target_jp)
        else:
            target = [None] * joints.shape[0]
            if seg["gripper"] in ("open", "close"):
                gripper_val = 0.04 if seg["gripper"] == "open" else 0.02
                target[7] = gripper_val / get_stage_units()
                target[8] = gripper_val / get_stage_units()
            target = ArticulationAction(joint_positions=target)

        # 段完成判定（"到位后 dwell"语义：移动期不消耗停留帧数）
        self.seg_frame += 1
        seg_done = False
        if seg["pos"] is None:
            # 纯夹爪段：固定时长
            seg_done = self.seg_frame >= max(seg["dwell"], 25)
        else:
            dist = np.linalg.norm(gripper_pos - seg["pos"])
            if dist < 0.012:
                self.arrived_frames += 1
            else:
                self.arrived_frames = 0
            if self.arrived_frames >= 2:
                # 到位后开始停留计数；离开目标则清零重计
                self.hold_frames += 1
                if self.hold_frames >= seg["dwell"]:
                    seg_done = True
            else:
                self.hold_frames = 0
            # 防卡死：总帧远超 dwell 仍未稳定到位则强制切段
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

        # pos 段：rmp forward 返回的已是 ArticulationAction，直接透传（避免双重包装）
        if isinstance(target, ArticulationAction):
            return target
        return ArticulationAction(joint_positions=target)

    def _step_infer(self, state):
        """Execute inference mode step."""
        if self.phase_idx >= len(self.phases):
            self.reset_needed = True
            return None, True, self._last_success

        language_instruction = self.get_language_instruction()
        state["language_instruction"] = language_instruction
        action = self.inference_engine.step_inference(state)

        return action, False, self.is_success()

    def is_success(self):
        """Task succeeds when all 13 phases have been executed."""
        return self.phase_idx >= len(self.phases)

    def get_language_instruction(self):
        return ("Place the watch glass at the center, open the dilute hydrochloric "
                "acid bottle, drip 2-3 drops with the dropper, ignite the bunsen "
                "burner, dip the platinum wire in the acid, burn it in the outer "
                "flame 3-4 times until no characteristic color, cool for 5 s, dip "
                "the solid sample powder, burn for 2-5 s to observe the flame "
                "color, extinguish the flame, rinse the wire and the watch glass")
