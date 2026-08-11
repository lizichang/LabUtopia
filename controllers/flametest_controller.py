"""焰色反应控制器：按 V7 文档 C1 的 13 步驱动机械臂，一步一 phase。

v21 修正（修复多物体同时附着）：
  - task 层新增 _any_obj_attached()：一次只抓一个物体
  - task 层 _near_grasp z_thresh 从 0.03 收紧至 0.015
  - task 层 GRIP_CLOSED_THRESH 裕量从 2mm 收紧至 1mm
  - yaml max_steps 从 15000 增至 30000

v22 修正（对齐修复后的 v17 USD，见 scripts/fix_flametest_v17.py）：
  - 表面皿移回可及位置 (0.32,-0.22)（v17 的 0.6682 超出 Franka 工作半径，
    RMP 卡在 x≈0.48 导致 P1 抓不到盘子、夹爪闭着扫过滴管误吸附）
  - 夹爪宽度按 mesh extent 实测：dish grip 0.0035、wire grip 0.0055

v24 修正（酒精灯替换本生灯 + 抓取/焰色物理修正，见 scripts/fix_flametest_v17.py）：
  - 本生灯 → 酒精灯（/World/AlcoholLamp，火焰 z 0.900-0.936，火柴点燃灯芯）
  - 表面皿固定 (0.20,0.02,0.80)，删除 P1 搬盘 / P13 洗盘
  - 铂丝/滴管抓"最上端"（0.977 / 0.931），抓取后先垂直提出试管架
    （铂丝 1.12 / 滴管 1.07，底端 > 架顶 0.917）再平移，杜绝穿模
  - WIRE_TIP_OFFSET 改物理环 (0,0,-0.170)，FLAME_HOLD 让环真正进入火焰
  - 焰色只局部变黄（染色锥放大到 1.5），不再整焰变色
  - 灯帽盖灭：抓帽顶 → 提到灯口正上方 → 下压盖住灯芯

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
import os
import numpy as np
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.extensions import get_extension_path_from_name

from .base_controller import BaseController as TaskBaseController


class FlameTestTaskController(TaskBaseController):
    """Composite controller: 11 phases (v24：酒精灯替换本生灯，去掉搬盘/洗盘)。"""

    GRIP_OPEN = 0.04

    def __init__(self, cfg, robot):
        super().__init__(cfg, robot)

    def _init_collect_mode(self, cfg, robot):
        super()._init_collect_mode(cfg, robot)
        print("[flametest] controller VERSION v31 (IK-based precise control, reachable match/cap)")
        self.orient = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # Lula IK 求解器：精确关节控制替代 RMP（RMP 对远距离低 z 目标发散 → 机械臂乱动）
        mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
        self._ik_solver = mg.LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
        rp, rq = robot.get_world_pose()
        self._ik_solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
        self._seg_ik_target = None
        self._seg_hold_joints = None   # 段内冻结关节（近奇异区抓点防漂移）
        self._prev_joints = None       # 上一帧关节（用于速度/稳定判定）
        # v34：IK 统一用固定 home 作 warm start。诊断证明 match/cap/ignite 从 home 均
        # 0.0mm 可达且无近限位解；而用"当前关节"作 warm start 会随段起点配置漂移，
        # 在近奇异区选出需要大摆动/近限位的坏解（arm 卡在 0.26m 外），甚至分支翻转。
        self._ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        self._build_phases()
        self.phase_idx = 0
        self.seg_idx = 0
        self.seg_frame = 0
        self.arrived_frames = 0
        self.hold_frames = 0
        self._h5_sample = 0
        self._start = True
        # v41：夹爪显式持位。诊断证明 ArticulationController 的 NaN→上一帧 applied
        # 替换在"hold 段"里不可靠（match 合爪后 lift/ignite 的 NaN 被物理解成 0.0000
        # 与 0.04 交替，爪口张开 → 火柴提前释放 → 永远点不着灯）。因此每帧都显式
        # 发送夹爪目标（open/close/hold 统一走 _grip_target），不再依赖 NaN。
        self._grip_target = self.GRIP_OPEN

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
        # 物体直径（USD 实测，v22/v24 修正）：
        #   瓶塞 25.2mm  滴管玻璃管 8mm  火柴杆 3mm
        #   铂丝手柄 11mm  酒精灯帽 37mm
        GRIP_STOPPER = 0.0126  # total 25.2mm, 瓶塞直径 25.2mm
        GRIP_DROPPER = 0.004   # total 8mm, 滴管玻璃管直径 8mm
        GRIP_MATCH   = 0.0015  # total 3mm, 火柴杆直径 3mm
        GRIP_WIRE    = 0.0055  # total 11mm, 铂丝手柄直径 11mm (mesh extent 实测)
        GRIP_CAP     = 0.0185  # total 37mm, 酒精灯帽直径 37.2mm
        GRIP_OPEN    = self.GRIP_OPEN

        # ---- 抓取点（与 task.GRASP_POINTS 精确对齐）----
        STO_GRASP     = (0.1200,  0.0200, 0.8770)
        STO_SIDE      = (0.1600,  0.0600, 0.8770)
        # v24：滴管抓玻璃管最上端（胶头下方），先在架内提出再平移
        DROP_GRASP    = (0.3591, -0.0205, 0.9310)
        DROP_XY       = (0.3591, -0.0205)
        SSTO_GRASP    = (0.2000,  0.1200, 0.8770)
        SSTO_SIDE     = (0.2400,  0.0800, 0.8770)
        # v31：火柴移到可及区 (0.42,0.26)（原 0.50,0.24 超出 Franka 工作半径 → RMP 乱动；
        # y=0.26 避开酒精灯底座 footprint y<=0.224，火柴杆朝 -x 指向灯芯）
        MATCH_GRASP   = (0.4200,  0.2600, 0.8150)  # v38：原 0.803 让手指 collider 底部(z=0.799)扎进桌面(z=0.80)卡死合爪；抬高 12mm
        # v24：铂丝抓手柄最上端（origin 附近）；抓取后先垂直提出试管架再平移
        WIRE_GRASP    = (0.3977, -0.0201, 0.9770)
        WIRE_XY       = (0.3977, -0.0201)
        # v31：灯帽移到可及区 (0.46,0.20)（原 0.46,0.28 超出工作半径）
        CAP_GRASP     = (0.4600,  0.2000, 0.9000)
        CAP_XY        = (0.4600,  0.2000)

        # 安全过渡高度（夹爪 z；持物时物品底端也高于所有障碍）
        H = 1.15

        # ---- 功能位置 ----
        # v24：滴管管口 = gripper_z - 0.119（抓管顶 0.931 后）
        DROP_LIFT = 1.07                     # 提出试管架：滴管底 0.812 > 架顶 0.917
        HCL_DIP = (0.12, 0.02, 0.95)         # 管口 z=0.831，深入液面
        DISH_DRIP = (0.20, 0.02, 0.97)       # 管口 z=0.851，正对表面皿中心滴液

        # 火柴：MATCH_TIP_OFFSET=(-0.0894,0,0)，头在 gripper -x 8.94cm（rotY=180 杆端为 origin）
        # v31：酒精灯芯顶 (0.36,0.18,0.9005) → gripper = (0.4494,0.18,0.9005)，火柴头视觉触达灯芯
        IGNITE = (0.4494, 0.18, 0.9005)

        # v24：铂丝环 = gripper - 0.170（物理位置）；表面皿固定在 (0.20,0.02)
        WIRE_LIFT = 1.12                     # 提出试管架：环底 0.8066 > 架顶 0.917
        ACID_DIP = (0.20, 0.02, 0.972)       # 环 z=0.802 浸入皿内酸液
        POWDER_DIP = (0.20, 0.12, 1.015)     # 环 z=0.845 进入瓶内粉末
        # 酒精灯火焰中心 (0.36,0.18,0.918) → gripper z = 0.918+0.170 = 1.088
        FLAME_HOLD = (0.36, 0.18, 1.088)     # 环真正进入火焰内部
        FLAME_APPROACH = (0.30, 0.16, 1.12)  # 高位接近，环 z=0.95 高于焰顶 0.936
        # 冷却位：远离火焰（v31：原 0.40,0.18,1.18 超 Franka 工作半径 → IK FAIL；
        # 移到 x=0.28（灯底座左侧，灯 x>=0.316）z=1.15，环底 0.98 高于焰顶 0.936）
        COOL_POS = (0.28, 0.18, 1.15)
        # 水柱冲洗：环 z=0.86 在水柱下 → gripper z = 0.86+0.170 = 1.03
        WASH_POS = (0.40, -0.10, 1.03)

        # 灯帽盖灭：帽中心 = gripper - 0.0085；帽底盖住灯芯（z≈0.905）
        # 盖灭时 gripper z = 0.905+0.0085+0.006 ≈ 0.92
        CAP_BURNER = (0.36, 0.18, 0.92)

        # 抓取前 settling 帧数（确保臂完全到位再合爪）
        SETTLE = 5

        self.phases = [
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
                seg((DROP_XY[0], DROP_XY[1], H)),
                seg(DROP_GRASP),
                seg(DROP_GRASP, "hold", SETTLE),
                seg(DROP_GRASP, "close", 25, grip=GRIP_DROPPER),
                seg((DROP_XY[0], DROP_XY[1], DROP_LIFT)),   # 先垂直提出试管架
                # --- 伸入盐酸瓶吸液 ---
                seg((0.12, 0.02, H)),
                seg((0.12, 0.02, 0.93)),       # 管口在瓶口高度(z=0.87)
                seg(HCL_DIP, "hold", 25),       # 管口 z=0.83，深入液面 1.15cm
                seg((0.12, 0.02, 0.93)),       # 提回瓶口高度
                # --- 移到表面皿上方滴 3 滴 ---
                seg((0.20, 0.02, H)),
                seg(DISH_DRIP, "hold", 200),    # 管口 z=0.85，停留滴 3 滴
                # --- 滴管归架（放回试管架）---
                seg((DROP_XY[0], DROP_XY[1], DROP_LIFT)),   # 高位回到架上方
                seg(DROP_GRASP),
                seg(DROP_GRASP, "open", 25),
                seg((DROP_XY[0], DROP_XY[1], H)),
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
            # P4 火柴点燃酒精灯
            # ================================================================
            [
                seg((0.42, 0.26, 1.05)),   # v31：H=1.15 超工作半径，用 1.05
                seg(MATCH_GRASP),
                seg(MATCH_GRASP, "hold", SETTLE),
                # v38：MATCH_GRASP 抬高到 z=0.815——原 z=0.803 时手指 collider 底部
                # (z≈0.799) 会扎进桌面(z=0.80)，接触力把夹爪卡在 ~0.018 且让近奇异区
                # 的臂猛甩。抬高后手指离桌面 ≥4mm，合爪干净触发 attach。
                # v37：dwell 25→60——pos=None 分支改 np.nan 数组后夹爪 ~27 帧
                # 才能从 0.04 到 <0.0025（火柴阈值最紧），25 帧不够。
                seg(None, "close", 60, grip=GRIP_MATCH),
                seg((0.42, 0.26, 0.90)),
                seg(IGNITE, "hold", 20),         # 火柴头触达灯芯点燃
                seg((0.42, 0.26, 0.90)),
                # v36：释放不回 MATCH_GRASP（近奇异再驱会甩）；在安全高度 z=0.90
                # 直接开爪，火柴 settle 回桌面。
                seg((0.42, 0.26, 0.90), "open", 25),
                seg((0.42, 0.26, 1.05)),   # v31：同上
            ],
            # ================================================================
            # P5 取铂丝（抓最上端，先提出试管架）-> 尖端浸入稀盐酸（表面皿中）
            # ================================================================
            [
                seg((WIRE_XY[0], WIRE_XY[1], H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),   # 先垂直提出试管架
                seg((0.20, 0.02, H)),
                seg(ACID_DIP, "hold", 25),       # 环浸入皿内酸液
                seg((0.20, 0.02, H)),
            ],
            # ================================================================
            # P6 酒精灯火焰灼烧（60 帧，清洗铂丝，无特征色）
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
                seg((0.20, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.20, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg((0.20, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.20, 0.02, H)),
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 60),
                seg(COOL_POS),
                seg((0.20, 0.02, H)),
                seg(ACID_DIP, "hold", 20),
                seg((0.20, 0.02, H)),
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
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((WIRE_XY[0], WIRE_XY[1], H)),
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
                seg((WIRE_XY[0], WIRE_XY[1], H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),
                # 蘸粉末
                seg((0.20, 0.12, H)),
                seg((0.20, 0.12, 1.03)),     # 环在瓶口上方 (z=0.86)
                seg(POWDER_DIP, "hold", 20),  # 环 z=0.845 进入瓶内粉末
                seg((0.20, 0.12, 1.03)),     # 提回瓶口上方
                seg((0.20, 0.12, H)),
            ],
            # ================================================================
            # P10 灼烧 2-5s（受染，黄色光晕出现在铂丝尖端）
            # ================================================================
            [
                seg(FLAME_APPROACH, "hold", 20),
                seg(FLAME_HOLD, "hold", 400),   # v23：加长灼烧，确保黄色焰色在相机里停留足够长
                seg(COOL_POS),
            ],
            # ================================================================
            # P11 灯帽盖灭（酒精灯）
            # ================================================================
            [
                # 放回铂丝（到试管架）
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((WIRE_XY[0], WIRE_XY[1], H)),
                # 取灯帽
                seg((0.46, 0.20, 1.00)),   # v31：H=1.15 超工作半径，用 1.00
                seg(CAP_GRASP),
                seg(CAP_GRASP, "hold", SETTLE),
                seg(CAP_GRASP, "close", 25, grip=GRIP_CAP),
                seg((0.46, 0.20, 1.00)),   # v31：原 1.05 超工作半径
                seg(CAP_BURNER, "hold", 25),   # 帽底盖住灯芯，任务检测熄灭
                seg((0.36, 0.18, 1.05), "open"),
                seg((0.36, 0.18, H)),
            ],
            # ================================================================
            # P12 冲洗铂丝 -> 归位
            # ================================================================
            [
                seg((WIRE_XY[0], WIRE_XY[1], H)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "hold", SETTLE),
                seg(WIRE_GRASP, "close", 25, grip=GRIP_WIRE),
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),
                seg((0.40, -0.10, H)),
                seg(WASH_POS, "hold", 80),     # 环在水柱下冲洗
                seg((0.40, -0.10, H)),
                seg((WIRE_XY[0], WIRE_XY[1], WIRE_LIFT)),
                seg(WIRE_GRASP),
                seg(WIRE_GRASP, "open", 25),
                seg((WIRE_XY[0], WIRE_XY[1], H)),
            ],
        ]
        self.phase_names = [
            "P1 open hcl stopper", "P2 drip 3 drops", "P3 ignite alcohol lamp",
            "P4 dip wire in acid", "P5 burn (no color)", "P6 repeat dip+burn x3",
            "P7 cool 5s", "P8 dip powder", "P9 burn 2-5s (stain)",
            "P10 extinguish", "P11 rinse & return wire",
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
        self._seg_ik_target = None
        self._seg_hold_joints = None
        self._prev_joints = None
        # v41：夹爪显式持位，见 _init_collect_mode 注释
        self._grip_target = self.GRIP_OPEN
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
            print("[flametest] all 11 phases done. success.")
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

    # 每帧关节最大变化量（rad），~1.8 rad/s @60Hz，保证平滑且不触发 PD 振荡
    MAX_JOINT_DELTA = 0.03

    def _lula_fk(self, joints7):
        """Lula FK of right_gripper for a 7-joint config (diagnostic)."""
        try:
            p, _ = self._ik_solver.compute_forward_kinematics("right_gripper", joints7)
            return np.asarray(p)
        except Exception:
            return np.full(3, np.nan)

    def _execute_seg(self, seg, state):
        joints = state["joint_positions"]
        gripper_pos = state["gripper_position"]

        if self._start:
            self._start = False
            # v37：与 pos 分支一致用 np.nan 数组（None 列表在 ArticulationAction 里
            # 驱动夹爪极慢，0.04→0.0189 用了近百帧，导致火柴闭合永远到不了阈值）。
            target = np.full(joints.shape[0], np.nan)
            target[7] = self.GRIP_OPEN / get_stage_units()
            target[8] = self.GRIP_OPEN / get_stage_units()
            return ArticulationAction(joint_positions=target)

        # Determine gripper value
        gripper_val = None
        if seg["gripper"] == "open":
            gripper_val = self.GRIP_OPEN
        elif seg["gripper"] == "close":
            gripper_val = seg["grip"] if seg["grip"] is not None else 0.02
        # v41：显式持位——open/close 更新 _grip_target，hold 沿用上次目标。
        # 每帧都发送夹爪目标，杜绝 NaN→applied 替换不可靠导致的爪口乱张。
        if gripper_val is not None:
            self._grip_target = gripper_val
        grip_out = self._grip_target if self._grip_target is not None else self.GRIP_OPEN

        if seg["pos"] is not None:
            cur = np.asarray(joints[:7], dtype=float)
            # v34：段入口一次 IK，warm start 固定 home（见 _init_collect_mode 注释）。
            # 避免"当前关节作 warm start"在近奇异区选出坏分支（卡死/大摆动）。
            if self._seg_ik_target is None:
                ik, ok = self._ik_solver.compute_inverse_kinematics(
                    frame_name="right_gripper",
                    target_position=seg["pos"],
                    target_orientation=self.orient,
                    warm_start=self._ik_home,
                )
                self._seg_ik_target = np.asarray(ik, dtype=float) if ok else None
                if not ok:
                    print(f"[flametest] IK FAIL target={np.round(seg['pos'],3)} — hold position, will force-done")
            if self._seg_hold_joints is not None:
                # 已冻结：保持当前关节，不再追 IK（近奇异区抓点处防止臂摆过头/漂移）
                cmd = self._seg_hold_joints
            elif self._seg_ik_target is not None:
                delta = np.clip(self._seg_ik_target - cur, -self.MAX_JOINT_DELTA, self.MAX_JOINT_DELTA)
                cmd = cur + delta
            else:
                cmd = cur
            target_jp = np.concatenate([cmd, [np.nan, np.nan]])
            # v41：夹爪每帧显式发送（不再依赖 NaN→applied 替换）
            target_jp[7] = grip_out / get_stage_units()
            target_jp[8] = grip_out / get_stage_units()
            target = ArticulationAction(joint_positions=target_jp)
        else:
            # v37：pos=None 段（原地 hold+合爪）用 np.nan 数组而非 None 列表。
            # None 列表经 ArticulationAction 驱动夹爪极慢（实测 0.04→0.0189 近百帧），
            # 而 np.nan 数组与 pos 分支同构，夹爪 ~25 帧即闭合到目标值。
            target = np.full(joints.shape[0], np.nan)
            # v41：夹爪每帧显式发送
            target[7] = grip_out / get_stage_units()
            target[8] = grip_out / get_stage_units()
            target = ArticulationAction(joint_positions=target)

        # Segment completion
        self.seg_frame += 1
        seg_done = False
        if seg["pos"] is None:
            seg_done = self.seg_frame >= max(seg["dwell"], 25)
        else:
            dist = np.linalg.norm(gripper_pos - seg["pos"])
            joint_err = 1e9
            if self._seg_ik_target is not None:
                joint_err = float(np.max(np.abs(
                    np.asarray(joints[:7], dtype=float) - self._seg_ik_target)))
            # v33 完成判定：TCP 进入抓取范围(1.5cm)且臂接近静止(关节变化<5mrad/frame)
            # 连续 3 帧 → 冻结关节保持位置，等 dwell 结束。match/cap/stopper 处于近奇异区，
            # 同一 TCP 可由多个 IK 分支到达，关节永远收敛不到单一解（joint_err≈1.1）；
            # 冻结即可稳住抓点，让任务侧 attach 判定通过。此前"dist 瞬时掠过即完成"
            # 导致段提前结束、臂还在摆动，attach 永远拿不到连续 3 帧 near。
            if self._seg_hold_joints is not None:
                # 已冻结：只累计 hold 帧（不因微小漂移重置）
                self.hold_frames += 1
                if self.hold_frames >= seg["dwell"]:
                    seg_done = True
            else:
                # v34b：冻结条件改为"TCP 进入抓取范围"即可，不再要求静止。
                # 诊断发现 match 抓取下探时 TCP 会穿过抓取区（attach 曾瞬时触发）然后
                # 摆过头卡在 17cm 外；只有"进圈即冻"才能抓住穿越瞬间的抓点。
                # 用与 task 一致的圆柱判定（z<1.5cm 且 xy<3cm），保证冻结点满足 attach。
                z_near = abs(float(gripper_pos[2]) - float(seg["pos"][2])) < 0.015
                xy_near = (float(np.linalg.norm(np.asarray(gripper_pos[:2], dtype=float)
                                                - np.asarray(seg["pos"][:2], dtype=float)))
                           < 0.03)
                if z_near and xy_near:
                    self.arrived_frames += 1
                else:
                    self.arrived_frames = 0
                if self.arrived_frames >= 2:
                    self._seg_hold_joints = np.asarray(joints[:7], dtype=float).copy()
                    print(f"[flametest] freeze at tgt={np.round(seg['pos'],3)} "
                          f"z={float(gripper_pos[2]):.3f} "
                          f"cur={np.round(self._seg_hold_joints,2)}")
                    self.hold_frames = 0
            # ---- 诊断：卡顿段每 60 帧 + force-done 时打印 Lula FK vs task FK ----
            if (seg["pos"] is not None and not seg_done
                    and self.seg_frame >= max(seg["dwell"], 60)
                    and self.seg_frame % 60 == 0):
                lf = self._lula_fk(np.asarray(joints[:7], dtype=float))
                print(f"[diag] t={self.seg_frame} tgt={np.round(seg['pos'],3)} "
                      f"cur={np.round(joints[:7],2)} "
                      f"taskFK={np.round(gripper_pos,3)} lulaFK={np.round(lf,3)} "
                      f"ik={np.round(self._seg_ik_target,2) if self._seg_ik_target is not None else None} "
                      f"je={joint_err:.3f} dist={dist:.3f} "
                      f"hold={'Y' if self._seg_hold_joints is not None else '-'}")

            if not seg_done and self.seg_frame >= seg["dwell"] + 600:
                seg_done = True
                lf = self._lula_fk(np.asarray(joints[:7], dtype=float))
                print(f"[flametest] seg force-done t={self.seg_frame} (dwell={seg['dwell']}) "
                      f"target={seg['pos']} gripper={np.round(gripper_pos, 3)} "
                      f"lulaFK={np.round(lf, 3)} dist={dist:.3f}")

        if seg_done:
            self.seg_frame = 0
            self.arrived_frames = 0
            self.hold_frames = 0
            self._seg_ik_target = None   # 下段重新计算 IK
            self._seg_hold_joints = None # 下段解锁重新逼近
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
        return ("Open the dilute hydrochloric acid bottle, drip 2-3 drops with the "
                "dropper onto the watch glass, ignite the alcohol lamp with a match, "
                "dip the platinum wire in the acid, burn it in the lamp "
                "flame 3-4 times until no characteristic color, cool for 5 s, dip "
                "the solid sample powder, burn for 2-5 s to observe the flame "
                "color, extinguish the flame with the cap, rinse the wire and return it")
