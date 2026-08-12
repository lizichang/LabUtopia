"""焰色反应任务（V7 文档 C1，13 步，一步一事件）。

13 步（严格按 LabUtopia_Action_Catalogue_v7 文档 C1）：
  P1 取表面皿置中央      P2 旋开稀盐酸瓶磨口塞   P3 滴管吸盐酸滴 2-3 滴（滴完盖塞归位）
  P4 点燃本生灯（蓝焰）  P5 铂丝蘸酸            P6 外焰灼烧
  P7 反复蘸酸+灼烧 3 次  P8 冷却 5s             P9 开样品瓶蘸粉末
  P10 灼烧 2-5s（受染）  P11 灯帽盖灭           P12 冲洗擦干归位
  P13 表面皿清洗归位

v21 修正（修复多物体同时附着 + 一次只抓一个 + 收紧 z 阈值）：
  - _near_grasp z_thresh 从 0.03 收紧至 0.015（1.5cm）
  - 新增 _any_obj_attached()：任何 kin_obj 或 wire 处于 attached 时禁止新附着
  - GRIP_CLOSED_THRESH 收紧裕量（从 grip+2mm 改为 grip+1mm）
  - 修复：抓铂丝时滴管同时附着（两者在试管架上仅相距 5mm）
  - 修复：伸入盐酸瓶时瓶塞误附着（瓶塞阈值 0.015 过宽松）
  - max_steps 从 15000 增至 30000（确保 13 phase 全部完成）

v22 修正（场景结构 + 可及性 + 夹爪宽度，配套 scripts/fix_flametest_v17.py）：
  - v17 USD 修复为 defaultPrim token "World"（否则 kit 的 add_reference 解析失败）
  - 子 prim 路径对齐 v17 实际命名：stopper_020 / stopper_021 / cap_004_011 /
    powder_002_002
  - 表面皿从不可及 x=0.6682 移回 (0.32,-0.22)（超出 Franka 桌面高度工作半径）
  - 夹爪宽度按 mesh extent 实测修正：dish 6.5mm、wire 11mm（原误记 8mm）

v23 修正（焰色可见性）：
  - 受染时火焰本体变成本色（yellow），不再只显示尖端染色锥——原染色锥
    r=6mm 被不透明白/蓝火焰（该高度 r≈8-10mm）完全包住，相机里看不到黄色
  - （v24 已改为"只局部变黄"并恢复蓝焰，见下）

v24 修正（酒精灯替换本生灯 + 抓取/焰色物理修正）：
  - 本生灯换成酒精灯（/World/AlcoholLamp，火焰 z 0.900-0.936）
  - 表面皿固定在 (0.20,0.02,0.80)，删除 P1 搬盘 / P13 洗盘
  - 铂丝/滴管改为抓"最上端"（手柄顶 0.977 / 玻璃管顶 0.931），抓取后先垂直提出
    试管架（铂丝 1.12 / 滴管 1.07，保证物品底端高于架顶 0.917）再平移
  - WIRE_TIP_OFFSET 修正为物理环位置 (0,0,-0.170)（旧 0.095,0,-0.055 是抽象点，
    实际环在夹爪正下方 17cm，导致尖端从未进入火焰而是在旁边绕）
  - DROPPER_NOZZLE_OFFSET 修正为 (0,0,-0.119)（管口在夹爪下方 11.9cm）
  - 焰色恢复"只有铂丝周围局部变黄"：染色锥迁到酒精灯下并放大到 1.5
    （r=18mm 探出酒精灯火焰 r=9mm），不再整焰变色

v20 修正（试管架移入工作空间 + 夹爪开合 = 物体直径）：
  - controller joint7 = 物体直径 / 2（从 USD mesh extent 精确提取）
  - 场景用 lab_flametest_v17.usd（含 TestTubeRack at (0.38,-0.14,0.80)）
  - 铂丝 rotateY(120°) 斜置：origin 在手柄底部 (0.368,-0.14,0.895)
    手柄中心 world=(0.417,-0.14,0.867)，环中心 world=(0.511,-0.14,0.812)
  - 滴管竖直放在试管架孔中：origin 在管口 (0.416,-0.14,0.812)
  - WIRE_TIP_OFFSET=(0.095,0,-0.055)：环中心 = gripper + offset
  - DROPPER_NOZZLE_OFFSET=(0,0,-0.06)：管口 = gripper + offset
  - stain 锥由 controller 定位到铂丝尖端，仅尖端周围 1.2cm 黄色光晕
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask


class FlameTestTask(BaseTask):
    """Task definition for the 13-step flame test (焰色反应) on a bunsen burner."""

    FLAME_COLORS = {
        "yellow": (1.00, 0.72, 0.12),   # v23：钠焰饱和黄（原 0.85,0.30 渲染后偏白）
        "purple": (0.80, 0.45, 1.00),
        "green": (0.35, 0.95, 0.40),
        "red": (1.00, 0.35, 0.25),
        "orange": (1.00, 0.60, 0.15),
        "blue": (0.30, 0.60, 1.00),
    }

    TABLE_Z = 0.80

    # ---- 铂丝（v17 USD：rotY=180，origin=手柄顶 (0.3977,-0.0201,0.9756)）----
    # 手柄 mesh local z[0,0.1195] → 世界 z[0.856,0.976]（从 origin 向下伸入试管架）
    # loop local z[0.162,0.169] → 世界 z[0.807,0.814]（环挂在最底端）
    # v24：抓手柄最上端（origin 附近），提出时环先离开架顶(0.917)再平移
    WIRE_REST  = np.array([0.5456, -0.0417, 0.9756])
    WIRE_GRASP = np.array([0.5456, -0.0417, 0.9770])
    WIRE_HELD_OFFSET = WIRE_REST - WIRE_GRASP  # (0, 0, -0.0014)
    # v24：环中心相对夹爪 = (0, 0, -(0.169+0.001))，物理位置而非抽象点
    WIRE_TIP_OFFSET = np.array([0.0, 0.0, -0.170])

    # ---- 各物体抓取点（夹爪 TCP 位置，世界坐标）----
    # v24：滴管改抓玻璃管最上端 z=0.931（胶头 z 0.927-0.962 上方）
    # 瓶塞：世界中心 z=0.8735，夹在近顶部 z=0.877
    # 火柴(rotY=180)：杆端(origin)世界 (0.42, 0.26, 0.803)，头朝 -x（v31：原 0.50,0.24 超工作半径；
    # y=0.26 避开酒精灯底座 y<=0.224，火柴杆朝 -x 指向灯芯）
    # 酒精灯帽：桌面旁 rest 中心 (0.6132,0.5456,0.8155)（帽底 0.80 贴桌面，mesh 高 3.1cm），
    # 夹在近顶部 z=0.824（v43：原 rest 0.8915/grasp 0.9000 用了"盖灯上"的高度，帽悬空 7.6cm）
    GRASP_POINTS = {
        "hcl_stopper":    np.array([0.1200, -0.2800, 0.8770]),
        "dropper":        np.array([0.5070, -0.0420, 0.9310]),
        "sample_stopper": np.array([-0.05,   0.30,   0.8770]),
        "match":          np.array([0.8868,  0.5939, 0.8150]),  # 抬高 12mm 让手指离桌，避免 collider 扎进桌面卡爪
        "cap":            np.array([0.6132,  0.5456, 0.8240]),
    }

    # ---- 物体静止位置（世界坐标，对于子物体指几何中心）----
    REST_POS = {
        "hcl_stopper":    np.array([0.1200, -0.2800, 0.8735]),
        "dropper":        np.array([0.5070, -0.0420, 0.8120]),
        "sample_stopper": np.array([-0.05,   0.30,   0.8735]),
        "match":          np.array([0.8868,  0.5939, 0.8133]),  # 随 GRASP 抬 12mm，保持 HELD_OFFSET z=-0.0017
        "cap":            np.array([0.6132,  0.5456, 0.8155]),
    }

    # ---- 子物体局部几何中心偏移（raw mesh center relative to prim origin）----
    # stoppers 的 mesh 顶点在局部 z[0.068,0.079]，中心 0.0735；cap 的圆柱以原点为中心
    LOCAL_GEOM_OFFSET = {
        "hcl_stopper":    np.array([0.0, 0.0, 0.0735]),
        "dropper":        np.array([0.0, 0.0, 0.0]),
        "sample_stopper": np.array([0.0, 0.0, 0.0735]),
        "match":          np.array([0.0, 0.0, 0.0]),
        "cap":            np.array([0.0, 0.0, 0.0915]),
    }

    # ---- 夹持偏移：物体 prim 原点相对夹爪的位置 = REST_POS - GRASP_POINT ----
    HELD_OFFSETS = {
        "hcl_stopper":    REST_POS["hcl_stopper"]    - GRASP_POINTS["hcl_stopper"],
        "dropper":        REST_POS["dropper"]        - GRASP_POINTS["dropper"],
        "sample_stopper": REST_POS["sample_stopper"] - GRASP_POINTS["sample_stopper"],
        "match":          REST_POS["match"]          - GRASP_POINTS["match"],
        "cap":            REST_POS["cap"]            - GRASP_POINTS["cap"],
    }

    # ---- 每物体夹爪闭合阈值（joint7 单指位移 < 此值才算夹紧）----
    # controller 设置 joint7 = grip_val（= 物体直径/2）；总宽 = 2*joint7
    # v45（本会话）：实测两指压住物体时 joint7 停在 grip_val+~1-2.3mm（PD 稳态
    # 误差+接触反力）。塞体最粗（~30mm），v45 完整运行实测停位 0.0149——旧阈值
    # grip+1mm(0.0136) 比停位还低 1.3mm，P1 瓶塞合爪永不触发 attach。改 grip+2~
    # 3.5mm：attach 在"已夹住但未卡死"的稳态区触发；物理上物体先经 _ease_obj_world
    # 平滑拉向夹爪（v28），无瞬移。v21 收紧到 +1mm 是为防试管架多物体误触发——
    # v44 已有最近物检查 + 紧密近窗(z<0.015)，且合爪只在原地 GripAction
    # （MoveAction 时手指全开 0.04），放宽安全。
    # grip 值：stopper 0.0126, dropper 0.004, match 0.0015, cap 0.0185, wire 0.0055
    GRIP_CLOSED_THRESH = {
        "hcl_stopper":    0.0160,   # 塞体 ~30mm，v45 实测停位 0.0149（+1.1mm 裕量）
        "dropper":        0.006,    # grip 0.004 + 2mm
        "sample_stopper": 0.0160,
        "match":          0.0035,   # grip 0.0015 + 2mm
        "cap":            0.022,    # 灯帽 v45 实测 attach 0.0203（grip 0.0185 + 3.5mm）
        "wire":           0.0075,   # grip 0.0055 + 2mm
    }

    # ---- 每物体释放阈值（joint7 单指位移 > 此值才判定松手）----
    # 通用阈值 gripper_open_threshold=0.03。灯帽在 P10 起升瞬间被确定性推到
    # 0.03381（夹爪与 kinematic 帽的碰撞解算），若用 0.03 会误判松手导致
    # 灭焰条件永不成立。帽阈值取 0.038：高于实测峰值 0.03381（含裕量），
    # 低于 controller 主动张开的 GRIP_OPEN=0.04，故主动松帽仍正常触发。
    RELEASE_THRESH = {
        "cap": 0.038,
    }

    # ---- 关键点 ----
    # v24：酒精灯位置与火焰区域（火焰 z 0.9005-0.9355，放宽一点容忍抖动）
    LAMP_POS = np.array([0.5132, 0.5256, 0.80])
    FLAME_Z = (0.898, 0.940)
    IGNITE_POS = np.array([0.5132, 0.5256, 0.9005])   # 灯芯顶端（火柴头触达点）
    # 火柴 rotY=180：杆端为 origin，头在 origin -x 方向 0.0894；HELD_OFFSET x=0
    # 头相对夹爪 = 0 - 0.0894 = -0.0894（v31：修正原 -0.048 与几何不符，头视觉触达灯芯）
    MATCH_TIP_OFFSET = np.array([-0.0894, 0.0, 0.0])
    # v24：滴管抓在 z=0.931（管顶），管口(z=0.812) 相对夹爪 z=-0.119
    DROPPER_NOZZLE_OFFSET = np.array([0.0, 0.0, -0.119])
    DISH_CENTER = np.array([0.5174, 0.2407])
    # v30：液滴可见性——flash 帧数（8->22）与下坠距离，滴酸肉眼可辨
    DROPLET_FLASH_FRAMES = 22
    DROPLET_FALL_DIST = 0.045   # 从滴管口到皿面的下坠距离
    DISH_TOP_Z = 0.805          # 液滴落到底的 z（皿内粉末面），防止穿过皿
    HCL_MOUTH = np.array([0.12, -0.28])
    SAMPLE_MOUTH = np.array([-0.05, 0.30])
    # v46：盖灭后灯帽落在酒精灯口。实测 v17：帽 mesh 半高 0.0155，灯体顶 0.8895；
    # 原 0.8915 → 帽底 0.876，下沉 13.5mm 嵌进灯体 = "没盖上去"。帽底贴灯口顶
    # 0.8895 → 帽中心 = 0.8895+0.0155 = 0.905，帽体盖住灯芯（芯顶 0.9005）。
    # 与控制器 CAP_BURNER(0.9135) 对齐：held 帽中心 = 0.9135-0.0085 = 0.905，零瞬移。
    CAP_SETTLED_POS = np.array([0.5132, 0.5256, 0.905])

    # ---- 真刚体（防穿模第 2 步）：瓶塞×2 + 灯帽 ----
    # RigidBodyAPI+CollisionAPI 在父 xform 下的第一个 Mesh prim 上（fix 脚本
    # make_stopper_cap_rigid 落盘）。流程期保持 kinematic（每帧 teleport 生效，
    # 变换不被物理覆盖）；落座用"盖到位后 kinematic 锁住"折中（瓶口凸包不建
    # 口洞，动态落座会把瓶塞顶飞，见 diag_rigid.py）。火柴/滴管/铂丝不在此列。
    RIGID_KIN_NAMES = ("hcl_stopper", "sample_stopper", "cap")
    # 落座成功判定（LiquidMixing 式读物理位姿）：目标几何中心 + 允许误差
    RIGID_SETTLE_TARGET = {
        "hcl_stopper":    (np.array([0.1200, -0.2800, 0.8735]), 0.025),
        "sample_stopper": (np.array([-0.05, 0.30, 0.8735]), 0.025),
        "cap":            (np.array([0.5132, 0.5256, 0.905]), 0.025),
    }

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

        self.dish_path = cfg.dish_path
        self.hcl_path = cfg.hcl_path
        self.hcl_stopper_path = self.hcl_path + "/stopper_020"
        self.dropper_path = cfg.dropper_path
        self.wire_path = cfg.wire_path
        self.sample_path = cfg.sample_path
        self.sample_stopper_path = self.sample_path + "/stopper_021"
        self.burner_path = cfg.burner_path   # v24：仍叫 burner_path，但指向 /World/AlcoholLamp
        # v26：染色锥迁到 /World 顶层（RTX 对引用灯下 over 子 prim 不渲染），
        # STAIN_ROOT 指向顶层作用域，染色锥路径 = STAIN_ROOT + "/flame_stain_{color}"
        self.STAIN_ROOT = "/World"
        self.cap_path = self.burner_path + "/cap"
        self.match_path = cfg.match_path
        self.droplet_path = cfg.droplet_path
        self.dish_acid_path = cfg.dish_acid_path

        self.flame_color = getattr(cfg, "flame_color", "yellow")

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 15))
        self.stain_dwell_frames = int(getattr(cfg, "stain_dwell_frames", 150))
        self.extinguish_dwell_frames = int(getattr(cfg, "extinguish_dwell_frames", 15))
        self.drop_dwell_frames = int(getattr(cfg, "drop_dwell_frames", 20))
        self.drop_interval_frames = int(getattr(cfg, "drop_interval_frames", 30))
        self.n_drops = int(getattr(cfg, "n_drops", 3))

        self.kin_objs = {
            "hcl_stopper":    {"path": self.hcl_stopper_path,    "parent": self.hcl_path},
            "dropper":        {"path": self.dropper_path,        "parent": None},
            "sample_stopper": {"path": self.sample_stopper_path, "parent": self.sample_path},
            "match":          {"path": self.match_path,          "parent": None},
            "cap":            {"path": self.cap_path,            "parent": self.burner_path},
        }
        self.wire_state = "rest"
        self._reset_kin_states()
        # v28（任务 #4）：夹爪连续处于抓取点近窗的帧数。>=GRASP_NEAR_FRAMES 才允许
        # 附着——防止臂从目标旁"路过"时（RMP 未收敛、close 段提前触发）误抓路过物体。
        self.GRASP_NEAR_FRAMES = 3
        self._grasp_near_frames = {name: 0 for name in self.kin_objs}

    def _reset_kin_states(self):
        for name in self.kin_objs:
            self.kin_objs[name]["state"] = "rest"

    def reset(self):
        super().reset()
        self.robot.initialize()
        self._min_gripper_z = 1e9   # 临时诊断：整集夹爪最低 z
        self._grasp_near_frames = {name: 0 for name in self.kin_objs}

        self.object_utils.set_object_position(
            object_path=self.wire_path, position=self.WIRE_REST.copy()
        )
        for name in self.kin_objs:
            self._set_obj_world(name, self.REST_POS[name])
            if name in self.RIGID_KIN_NAMES:
                self._set_kinematic(name, True)   # 真刚体保持 kinematic，静止位不被物理覆盖
        self._set_flame_visible(False)
        self._set_stain(False)
        self._set_visibility(self.droplet_path, False)
        self._set_visibility(self.dish_acid_path, False)
        # v24：表面皿是固定静态器材，始终保持可见（之前 reset 时隐藏导致
        # 快照/视频里"玻璃皿看不见"，用户多次反馈）
        self._set_visibility(self.dish_path + "/powder_002_002", True)

        self.wire_state = "rest"
        self.flame_on = False
        self.stain_on = False
        self.powder_dipped = False   # v24：蘸过待测粉末后才允许显色
        # 里程碑完成标记：仅当点燃/显色/灭焰全部真实发生才判 success（任务 #14）
        self._ignite_fired = False
        self._stain_fired = False
        self._extinguish_fired = False
        self.ignite_counter = 0
        self.stain_counter = 0
        self.extinguish_counter = 0
        self.drop_counter = 0
        self.drop_interval = 0
        self.n_dropped = 0
        self.droplet_flash = 0
        self.droplet_start = np.zeros(3)   # v30：液滴下落起点
        self._reset_kin_states()

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_objects_and_events()
        return self.get_basic_state_info(
            object_path=self.wire_path,
            additional_info={
                "flame_on": self.flame_on,
                "stain_on": self.stain_on,
                "flame_color": self.flame_color,
                "n_dropped": self.n_dropped,
                "wire_state": self.wire_state,
            },
        )

    # ------------------------------------------------------------------
    # 每帧更新
    # ------------------------------------------------------------------
    def _update_objects_and_events(self):
        gripper_pos = self.robot.get_gripper_position()
        if gripper_pos is not None:
            self._min_gripper_z = min(self._min_gripper_z, float(gripper_pos[2]))
        joint_positions = self.robot.get_joint_positions()
        if gripper_pos is None or joint_positions is None:
            return
        gripper_opening = joint_positions[7]

        self._update_kin_objects(gripper_pos, gripper_opening)
        self._update_wire(gripper_pos, gripper_opening)
        self._update_effects(gripper_pos)

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _grasp_point(self, name):
        """当前最合适的抓取点（TCP 世界坐标）。物体 rest 时由当前位置推导：
        grasp_z = cur_z - HELD_OFFSETS.z（夹近顶）。瓶口 rest → 0.877（同 GRASP_POINTS），
        桌面放置 → 0.810（v44，修 bug5：瓶塞从桌面再抓起时近窗认桌面位置，不再
        只认瓶口 → 机械臂空抓桌面、举空爪回瓶口瞬间塞子瞬移）。对所有 kin_obj
        统一，无需为每种物体配第二近窗。"""
        cur = self._get_obj_world(name)
        return np.array([cur[0], cur[1], cur[2] - self.HELD_OFFSETS[name][2]])

    def _any_obj_attached(self):
        """检查是否已有任何 kin_obj 或 wire 处于 attached 状态（一次只抓一个）。"""
        if self.wire_state == "attached":
            return True
        for name, obj in self.kin_objs.items():
            if obj["state"] == "attached":
                return True
        return False

    def _find_closest_graspable(self, gripper_pos):
        """v21: 找到离夹爪最近的可抓取物体（rest 状态的 kin_obj 或 wire）。
        返回 (type, name) 或 (None, None)。type 为 "kin" 或 "wire"。
        当滴管和铂丝在试管架上仅相距 5mm 时，确保只抓最近的那个。
        """
        candidates = []
        for name, obj in self.kin_objs.items():
            if obj["state"] == "rest":
                grasp = self._grasp_point(name)
                dist = np.linalg.norm(gripper_pos - grasp)
                candidates.append((dist, "kin", name))
        if self.wire_state == "rest":
            dist = np.linalg.norm(gripper_pos - self.WIRE_GRASP)
            candidates.append((dist, "wire", None))
        if not candidates:
            return None, None
        candidates.sort(key=lambda c: c[0])
        return candidates[0][1], candidates[0][2]

    def _update_kin_objects(self, gripper_pos, gripper_opening):
        for name, obj in self.kin_objs.items():
            if obj["state"] == "settled":
                continue

            if obj["state"] == "rest":
                # v21: 一次只抓一个物体——已有物体附着时不抓新的
                if self._any_obj_attached():
                    # review: 附着期间臂被占用，其它物体的近窗计数必须清零，
                    # 否则会被"跨附着急冻"绕过连续帧门禁（MEDIUM #3）。
                    self._grasp_near_frames[name] = 0
                    continue
                # v21: 只抓最近的物体（防止滴管/铂丝误抓）
                closest_type, closest_name = self._find_closest_graspable(gripper_pos)
                if closest_type != "kin" or closest_name != name:
                    # review: 非最近即清零，保证 GRASP_NEAR_FRAMES 真正连续（MEDIUM #3）
                    self._grasp_near_frames[name] = 0
                    continue
                grasp = self._grasp_point(name)
                closed_thresh = self.GRIP_CLOSED_THRESH[name]
                near = self._near_grasp(gripper_pos, grasp)
                # 连续近窗计数：非近窗即清零。>=GRASP_NEAR_FRAMES 才允许附着，
                # 防止臂从物体旁"路过"时误抓（v28，任务 #4）。
                self._grasp_near_frames[name] = self._grasp_near_frames[name] + 1 if near else 0
                # v28（任务 #4）：夹爪开始合拢且已进入近窗时先把物体平滑拉向夹爪，
                # 消除"悬空合爪（闭合瞬间物体纹丝不动）+闪现吸附（瞬间 teleport）"。
                # 只在 near（附着可达）时 ease：避免合爪未遂时把物体拖离原位
                # 残留为"悬空物体"（review MEDIUM #2）。
                held = gripper_pos + self.HELD_OFFSETS[name]
                if near and gripper_opening < self.gripper_open_threshold:
                    self._ease_obj_world(name, held)
                if (near and self._grasp_near_frames[name] >= self.GRASP_NEAR_FRAMES
                        and gripper_opening < closed_thresh):
                    obj["state"] = "attached"
                    self._set_obj_world(name, held)
                    print(f"[flametest] attached {name} (grip={gripper_opening:.4f} < {closed_thresh})")

            elif obj["state"] == "attached":
                self._set_obj_world(name, gripper_pos + self.HELD_OFFSETS[name])
                rel_thresh = self.RELEASE_THRESH.get(name, self.gripper_open_threshold)
                if gripper_opening > rel_thresh:
                    obj["state"] = "released"
                    self._settle_object(name)
                    print(f"[flametest] released {name} (grip={gripper_opening:.5f} > {rel_thresh})")

    def _update_wire(self, gripper_pos, gripper_opening):
        if self.wire_state == "rest":
            # v21: 一次只抓一个——已有 kin_obj 附着时不抓 wire
            if self._any_obj_attached():
                return
            # v21: 只抓最近的物体（防止抓铂丝时误抓滴管）
            closest_type, _ = self._find_closest_graspable(gripper_pos)
            if closest_type != "wire":
                return
            if (self._near_grasp(gripper_pos, self.WIRE_GRASP)
                    and gripper_opening < self.GRIP_CLOSED_THRESH["wire"]):
                self.wire_state = "attached"
                self.object_utils.set_object_position(
                    object_path=self.wire_path,
                    position=gripper_pos + self.WIRE_HELD_OFFSET,
                )
                print("[flametest] wire attached (snap-to-grip)")

        elif self.wire_state == "attached":
            self.object_utils.set_object_position(
                object_path=self.wire_path,
                position=gripper_pos + self.WIRE_HELD_OFFSET,
            )
            if (gripper_opening > self.gripper_open_threshold
                    and np.linalg.norm(gripper_pos - self.WIRE_GRASP) < 0.05):
                self.object_utils.set_object_position(
                    object_path=self.wire_path, position=self.WIRE_REST.copy()
                )
                self.wire_state = "rest"
                print("[flametest] wire returned to rest")

    def _update_effects(self, gripper_pos):
        # ---- 1. 点燃 ----
        if not self.flame_on:
            if self.kin_objs["match"]["state"] == "attached":
                tip = self._match_tip(gripper_pos)
                if np.linalg.norm(tip - self.IGNITE_POS) < 0.035:
                    self.ignite_counter += 1
                    if self.ignite_counter >= self.ignite_dwell_frames:
                        self.flame_on = True
                        self._ignite_fired = True
                        self._set_flame_visible(True)
                        print("[flametest] flame ignited (alcohol lamp)")
                else:
                    self.ignite_counter = 0
            else:
                self.ignite_counter = 0

        # ---- 2. 滴液 ----
        if self.kin_objs["dropper"]["state"] == "attached":
            nozzle = self._nozzle(gripper_pos)
            above_dish = (np.linalg.norm(nozzle[:2] - self.DISH_CENTER) < 0.035
                          and abs(nozzle[2] - 0.85) < 0.04)
            if above_dish:
                self.drop_counter += 1
                if self.drop_counter >= self.drop_dwell_frames and self.n_dropped < self.n_drops:
                    self.drop_interval += 1
                    if self.drop_interval >= self.drop_interval_frames:
                        self.n_dropped += 1
                        self.drop_interval = 0
                        # v30：flash 8->22 帧 + 记录下落起点，液滴肉眼可辨
                        self.droplet_flash = self.DROPLET_FLASH_FRAMES
                        self.droplet_start = nozzle + np.array([0.0, 0.0, -0.02])
                        self._set_obj_world_plain(self.droplet_path, self.droplet_start)
                        if self.n_dropped >= self.n_drops:
                            self._set_visibility(self.dish_acid_path, True)
                        print(f"[flametest] drop {self.n_dropped}/{self.n_drops}")
            else:
                self.drop_counter = 0
                self.drop_interval = 0
        else:
            self.drop_counter = 0
            self.drop_interval = 0
        if self.droplet_flash > 0:
            # v30：液滴下落动画——从滴管口坠向皿面，clamp 在皿内粉末面之上
            self.droplet_flash -= 1
            frac = 1.0 - self.droplet_flash / float(self.DROPLET_FLASH_FRAMES)
            z = self.droplet_start[2] - frac * self.DROPLET_FALL_DIST
            z = max(z, self.DISH_TOP_Z)
            self._set_obj_world_plain(
                self.droplet_path, np.array([self.droplet_start[0],
                                             self.droplet_start[1], z]))
        self._set_visibility(self.droplet_path, self.droplet_flash > 0)

        # ---- 3. 受染：铂丝尖端在外焰 -> 局部黄色光晕跟随尖端 ----
        if self.flame_on and self.wire_state == "attached":
            tip = gripper_pos + self.WIRE_TIP_OFFSET
            # v24：检测蘸粉末（环进入样品瓶内）→ 解锁显色
            if (np.linalg.norm(tip[:2] - self.SAMPLE_MOUTH) < 0.05
                    and 0.80 < tip[2] < 0.86):
                if not self.powder_dipped:
                    self.powder_dipped = True
                    print("[flametest] powder dipped (stain unlocked)")
            in_flame = (np.linalg.norm(tip[:2] - self.LAMP_POS[:2]) < 0.05
                        and self.FLAME_Z[0] < tip[2] < self.FLAME_Z[1])
            if in_flame and self.powder_dipped:
                self.stain_counter += 1
                if self.stain_counter >= self.stain_dwell_frames:
                    if not self.stain_on:
                        self.stain_on = True
                        self._stain_fired = True
                        self._set_stain(True)
                        print(f"[flametest] stain {self.flame_color} revealed")
                    self._position_stain_at_tip(tip)
            else:
                if self.stain_counter > 0 or self.stain_on:
                    self.stain_counter = 0
                    if self.stain_on:
                        self.stain_on = False
                        self._set_stain(False)
        elif self.stain_on:
            self.stain_on = False
            self._set_stain(False)

        # ---- 4. 灭焰 ----
        if self.flame_on and self.kin_objs["cap"]["state"] == "attached":
            # v24：帽底盖住灯芯（灯口 z≈0.905）
            if np.linalg.norm(gripper_pos - np.array([0.5132, 0.5256, 0.905])) < 0.04:
                self.extinguish_counter += 1
                if self.extinguish_counter >= self.extinguish_dwell_frames:
                    self.flame_on = False
                    self.stain_on = False
                    self._extinguish_fired = True
                    self._set_flame_visible(False)
                    self._set_stain(False)
                    self._set_obj_world("cap", self.CAP_SETTLED_POS)
                    self._set_kinematic("cap", True)   # 灯帽盖到位后 kinematic 锁住（折中）
                    self.kin_objs["cap"]["state"] = "settled"
                    self._verify_settle("cap", self.CAP_SETTLED_POS)
                    print("[flametest] flame extinguished, cap on burner")
            else:
                self.extinguish_counter = 0

        # ---- 5.（v42：WaterJet 已删，P12 冲洗取消，水柱逻辑移除）----

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _match_tip(self, gripper_pos):
        return gripper_pos + self.MATCH_TIP_OFFSET

    def _nozzle(self, gripper_pos):
        return gripper_pos + self.DROPPER_NOZZLE_OFFSET

    def _get_obj_world(self, name):
        """获取物体几何中心世界坐标（对子物体补偿 local_geom_offset）。"""
        obj = self.kin_objs[name]
        world_origin = self.object_utils.get_object_xform_position(obj["path"])
        if world_origin is None:
            return self.REST_POS[name].copy()
        return world_origin + self.LOCAL_GEOM_OFFSET[name]

    def _set_obj_world(self, name, world):
        """设置物体几何中心世界坐标（对子物体减去 local_geom_offset）。"""
        obj = self.kin_objs[name]
        if obj["parent"] is None:
            self.object_utils.set_object_position(object_path=obj["path"], position=world)
        else:
            parent_t = self.object_utils.get_object_xform_position(obj["parent"])
            local_t = world - parent_t - self.LOCAL_GEOM_OFFSET[name]
            self.object_utils.set_object_position(
                object_path=obj["path"], position=local_t)

    def _set_obj_world_plain(self, path, world):
        self.object_utils.set_object_position(object_path=path, position=world)

    def _ease_obj_world(self, name, target, k=0.18):
        """把物体几何中心逐帧向 target 平滑移动（k = 每帧进度）。

        v28（任务 #4）：抓取时物体不再静止到闭合瞬间再 teleport（闪现吸附），
        而是夹爪开始合拢且够近时逐帧逼近夹爪持物位，视觉上是平滑提起。
        """
        cur = self._get_obj_world(name)
        nxt = cur + (target - cur) * k
        self._set_obj_world(name, nxt)

    # ------------------------------------------------------------------
    # 真刚体辅助（防穿模第 2 步）：瓶塞/灯帽在 fix 脚本 make_stopper_cap_rigid
    # 里已转成真刚体（RigidBodyAPI+CollisionAPI+凸包，默认 kinematic）。
    # 流程期保持 kinematic，让每帧 teleport 不被物理覆盖；落座（盖回瓶口/灯口）
    # 用"盖到位后 kinematic 锁住"折中：瓶口的凸包碰撞不建模口洞，真动态落座
    # 会把瓶塞顶飞（diag_rigid.py 实测 err=0.099，z 顶到 0.951），所以落座
    # 一律 kinematic 锁位，成功判定读物理位姿（LiquidMixing 式）。
    # ------------------------------------------------------------------
    def _find_rigid_body_prim(self, parent_path):
        """父 xform 下第一个 Mesh prim（RigidBodyAPI 所在）。"""
        parent = self.stage.GetPrimAtPath(parent_path)
        if parent.IsValid():
            for child in parent.GetChildren():
                if child.GetTypeName() == "Mesh":
                    return child
        fallback = self.stage.GetPrimAtPath(
            parent_path + "/" + parent_path.rsplit("/", 1)[-1])
        return fallback if fallback.IsValid() else None

    def _set_kinematic(self, name, enabled):
        """开/关真刚体的 physics:kinematicEnabled（在 mesh prim 上）。"""
        if name not in self.RIGID_KIN_NAMES:
            return
        prim = self._find_rigid_body_prim(self.kin_objs[name]["path"])
        if prim is None or not prim.IsValid():
            return
        attr = prim.GetAttribute("physics:kinematicEnabled")
        if attr is None or not attr.IsValid():
            return
        attr.Set(bool(enabled))

    def _get_rigid_world(self, name):
        """读真刚体 mesh 的物理世界位姿（几何中心，带 LOCAL_GEOM_OFFSET 补偿）。
        读 mesh 自身 xform（body 所在），比 _get_obj_world 读父 xform 更贴近
        物理实际摆放。"""
        prim = self._find_rigid_body_prim(self.kin_objs[name]["path"])
        if prim is None or not prim.IsValid():
            return self._get_obj_world(name)
        wm = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        return np.array(wm.ExtractTranslation()) + self.LOCAL_GEOM_OFFSET[name]

    def _verify_settle(self, name, expected):
        """LiquidMixing 式成功判定：读真刚体物理位姿与期望位比对。
        落座是 kinematic 锁位（见上），读的是锁定位姿；若 teleport 未生效或
        物体被撞偏，err 会超阈值并被 WARN 出来。"""
        pos = self._get_rigid_world(name)
        tol = self.RIGID_SETTLE_TARGET[name][1]
        err = float(np.linalg.norm(pos - expected))
        ok = err < tol
        print(f"[flametest] settle {name}: physical={tuple(np.round(pos, 4))} "
              f"target={tuple(np.round(expected, 4))} err={err:.4f} "
              f"{'OK' if ok else 'WARN'}")
        return ok

    def _settle_object(self, name):
        if name == "hcl_stopper":
            cur = self._get_obj_world(name)
            if np.linalg.norm(cur[:2] - self.HCL_MOUTH) < 0.03:
                # 盖回瓶口：kinematic 锁位 + 物理位姿判定（凸包不建口洞，勿动态落座）
                self._set_obj_world(name, self.REST_POS[name])
                self._set_kinematic(name, True)
                self._verify_settle(name, self.REST_POS[name])
            else:
                # 倒放桌面：保持 kinematic 锁定（P3 还要再抓，位置必须与抓取点一致）
                cur[2] = self.TABLE_Z + 0.006
                self._set_obj_world(name, cur)
                self._set_kinematic(name, True)
        elif name == "sample_stopper":
            cur = self._get_obj_world(name)
            if np.linalg.norm(cur[:2] - self.SAMPLE_MOUTH) < 0.03:
                self._set_obj_world(name, self.REST_POS[name])
                self._set_kinematic(name, True)
                self._verify_settle(name, self.REST_POS[name])
            else:
                # 倒放桌面：避免 6mm 落下翻倒，保持 kinematic 锁定
                cur[2] = self.TABLE_Z + 0.006
                self._set_obj_world(name, cur)
                self._set_kinematic(name, True)
        elif name in ("dropper", "match"):
            self._set_obj_world(name, self.REST_POS[name])
        self.kin_objs[name]["state"] = "rest"

    def _set_flame_visible(self, visible: bool) -> None:
        # v30.2：火焰锥迁到 /World 顶层（引用灯下 over 子 prim 在 RTX 不渲染，
        # 见 fix 脚本 rebuild_flame_cones），路径从 burner_path 下改到顶层。
        for prim_name in ("flame_outer", "flame_inner"):
            prim = self.stage.GetPrimAtPath(f"/World/{prim_name}")
            if prim.IsValid():
                set_prim_visibility(prim, visible)

    def _position_stain_at_tip(self, tip_world):
        """将受染锥中心定位到铂丝尖端（世界坐标）。
        v26：染色锥迁到 /World 顶层（RTX 对引用灯下 over 子 prim 不渲染），
        因此 translate 直接用世界坐标，不再减 LAMP_POS。"""
        prim = self.stage.GetPrimAtPath(
            f"{self.STAIN_ROOT}/flame_stain_{self.flame_color}")
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(float(tip_world[0]), float(tip_world[1]),
                                    float(tip_world[2])))
                    break

    def _set_stain(self, visible: bool) -> None:
        for color in self.FLAME_COLORS:
            prim = self.stage.GetPrimAtPath(f"{self.STAIN_ROOT}/flame_stain_{color}")
            if prim.IsValid():
                set_prim_visibility(prim, False)
        if visible:
            prim = self.stage.GetPrimAtPath(
                f"{self.STAIN_ROOT}/flame_stain_{self.flame_color}")
            if prim.IsValid():
                set_prim_visibility(prim, True)
        # v26：焰色只做局部——染色锥 r=26mm 探出酒精灯火焰（r=9mm），
        # 只有铂丝周围那一圈火焰变黄，不再整焰变色。

    def _set_visibility(self, path, visible):
        try:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass

    def on_task_complete(self, success: bool) -> None:
        """任务 #14：成功必须以真实里程碑为准（点燃+显色+灭焰都发生），
        不能由 controller 走完所有分段就判成功。另保留夹爪最低 z 诊断。"""
        real = (getattr(self, '_ignite_fired', False)
                and getattr(self, '_stain_fired', False)
                and getattr(self, '_extinguish_fired', False))
        print(f"[flametest] episode done success={real} (controller_said={success}) "
              f"ignite={getattr(self, '_ignite_fired', False)} "
              f"stain={getattr(self, '_stain_fired', False)} "
              f"extinguish={getattr(self, '_extinguish_fired', False)} "
              f"min_gripper_z={getattr(self, '_min_gripper_z', float('nan')):.4f}")
        super().on_task_complete(real)
