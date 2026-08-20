"""headless 终端键盘遥操作 + waypoint 录制（无显示器场景专用）。

用法（在 labutopia 环境、有键盘的终端里跑；脚本内部强制 headless，无需加参数）：
    python -u scripts/teleop_record_d2s.py

按下的每个键立即生效（终端原始模式，无需回车）。场景 = D2-S 水溶性测试台。
起始状态 = 「法兰旋转完」：药匙已夹起放平（TCP=提起位 (0.6993,0.3608,1.15)、joint7
转 -90°），从舀粉这步开始操作。g 开爪会把药匙松回架，近架合爪可重新夹持。

移动 TCP（默认步长 5mm）：
    w / s 或 ↑ / ↓     z 轴（上 / 下）
    a / d 或 ← / →     x 轴（左 / 右）
    q / e               y 轴（前 / 后）
旋转夹爪（默认 10°，绕世界轴）：
    r / f      绕 X 轴
    t / v      绕 Y 轴
    y / n      绕 Z 轴
夹爪：
    g               开/关夹爪（开 0.04 ↔ 合 0.008）
记录：
    空格            渲染一张截图 PNG 到 outputs/teleop_shot/（你看图确认位置）
    回车            把当前 TCP 位姿存为一个 waypoint（同时截图存档）
    u               撤销最后一个 waypoint
其他：
    + / -           切换移动步长 1 / 2 / 5 / 10 mm
    ?               帮助
    Esc / q / Ctrl-C 退出并导出

退出后：
  - 写 outputs/teleop_waypoints_<ts>.json（waypoint 列表）
  - 终端打印对应的 mv() 序列（可直接粘进元动作 _build_actions）

相机：3 台——TeleopCamA/B（固定，复用 d2s camera_1/camera_2 位姿）+ TeleopCamHand
（机械臂手部随动，= 实验 camera_3 同一 prim /World/Franka/panda_hand/arm_camera）。

说明：四元数统一为 [x,y,z,w]（scipy 约定，与 IkMotionEngine.solve_verified 一致，
ORIENT_FWD=(0,0.7071,0,0.7071)=绕 Y 90° 已验证）。每个 waypoint 的 orient 直接喂 mv()。
"""
import os
import sys
import time
import json
import select
import termios
import tty

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import hydra
import omni
import omni.usd
import cv2
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils import extensions
from isaacsim.sensors.camera import Camera
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

import isaacsim.robot_motion.motion_generation as mg
from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from controllers.atomic_actions.flametest.ik_engine import IkMotionEngine

# —— d2s 常量（与 catalogue/d_wetchem/d2s_water_solubility/meta_actions/constants.py 一致）——
GRIP_OPEN = 0.04
GRIP_CLOSE = 0.008
SPATULA_PATH = "/World/Spatula"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "outputs")
SHOT_DIR = os.path.join(OUT_DIR, "teleop_shot")

# 相机位姿（复用 d2s 配置 camera_1 / camera_2，已对准工作台）
CAM_CONFIGS = [
    ("TeleopCamA", (1.00, 0.40, 1.15), (0.38519, 0.13095, 0.29403, 0.86489)),
    ("TeleopCamB", (0.80, 0.10, 1.44), (0.9425, 0.10815, 0.03605, 0.31417)),
]
CAM_RES = (1024, 1024)

MOVE_STEPS = [0.001, 0.002, 0.005, 0.010]   # 可选步长 1/2/5/10 mm
ROT_STEP = 10.0                              # 旋转步长（度）
ARRIVE_EPS = 0.010                           # 到位判定（截图前等待到达）
SETTLE_MAX = 150                             # 等待到位最大帧数

# 按键 → 世界轴方向增量（a/d 为 x，w/s 为 z，q/e 为 y）
MOVE_MAP = {
    "w": (0, 0, +1), "s": (0, 0, -1),
    "a": (-1, 0, 0), "d": (1, 0, 0),
    "q": (0, +1, 0), "e": (0, -1, 0),
    "\x1b[A": (0, 0, +1), "\x1b[B": (0, 0, -1),
    "\x1b[D": (-1, 0, 0), "\x1b[C": (1, 0, 0),
}

# —— 药匙持握（与 d2s task.py 同：药匙世界 = _T_HELD · tool_center 世界，逐帧覆写）——
SPATULA_PATH = "/World/Spatula"
GRIP_OPEN_THRESH = 0.03              # grip > 此值 → 松开药匙回架（同 flametest）
SPAT_GRIP_CLOSED = GRIP_CLOSE + 0.004  # 0.012：夹紧判定（grip 0.008 + 4mm 裕量）
SPAT_GRASP = (0.6993, 0.3608, 0.94)  # 药匙抓点（架内柄杆，z 0.94）
START_TCP = (0.6993, 0.3608, 1.15)   # pick④ 提起位：法兰转后 TCP 不变
START_ORIENT = (0.0, 0.7071068, 0.0, 0.7071068)  # ORIENT_FWD scipy [x,y,z,w]

# 药匙相对夹爪：平移(0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）。
# 平移必须在最后一行（USD 行向量约定）。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)


def _rest_matrix():
    """药匙架内竖插位姿（= 场景 /World/Spatula 世界矩阵，平移最后一行，同 task.py）。"""
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.6993, 0.3608, 0.828, 1.0)


def say(msg):
    """raw 模式下打印整行（\r\n 结尾，避免只换行不回车）。"""
    sys.stdout.write("\r\x1b[K" + msg + "\r\n")
    sys.stdout.flush()


def read_key(block_timeout=0.0):
    """非阻塞读一个按键；方向键/Esc 是 \x1b[...] 转义序列，在此合并读取。"""
    r, _, _ = select.select([sys.stdin], [], [], block_timeout)
    if not r:
        return ""
    b = os.read(0, 1)
    if not b:
        return ""
    ch = b.decode("latin1")
    if ch == "\x1b":
        seq = ch
        for _ in range(6):
            r, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not r:
                break
            b = os.read(0, 1)
            if not b:
                break
            seq += b.decode("latin1")
        return seq
    return ch


class TeleopRecorder:
    """headless 键盘遥操作 + waypoint 录制（D2-S 台面）。"""

    def __init__(self, robot, engine, cameras, world, stage):
        self.robot = robot
        self.engine = engine
        self.cameras = cameras
        self.world = world
        self.stage = stage
        self.spatula_path = SPATULA_PATH
        # 起始：直接「法兰旋转完」状态 —— 先解 pick④ 提起位（ORIENT_FWD），再只把
        # 最后一个关节（joint7，索引6）转 -90°（同 FlangeRollAction）。药匙已夹起放平，
        # 从这一步开始操作（用户 2026-08-19 要求）。
        self.target7 = self._flange_rolled_start()
        pos, rot = engine.fk_pose(self.target7)
        self.target_pos = np.asarray(pos, dtype=float)
        self.orient_q = Rotation.from_matrix(np.asarray(rot)).as_quat()   # [x,y,z,w]
        self.grip = GRIP_CLOSE          # 已夹紧药匙（0.008）
        self.spatula_state = "attached"  # 药匙随夹爪 6-DOF 持握
        self.waypoints = []
        self.step = 0.005
        self._running = True

    # ------------------------------------------------------------- 循环
    def run(self):
        self._help()
        while self._running and simulation_app.is_running():
            key = read_key(0.0)
            if key:
                self._handle(key)
            self._command_arm()
            self._update_spatula()
            self.world.step(render=True)
            time.sleep(1.0 / 120.0)

    def _command_arm(self):
        j = self.robot.get_joint_positions()
        target = np.full(j.shape[0], np.nan)
        target[:7] = self.target7
        target[7] = self.grip / get_stage_units()
        target[8] = self.grip / get_stage_units()
        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=target))

    # ------------------------------------------------------------- 药匙持握
    def _flange_rolled_start(self):
        """法兰旋转完的关节配置：解 pick④ 提起位 IK + 只转最后一个关节 -90°。"""
        home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
        ik = self.engine.solve_verified(np.array(START_TCP, dtype=float), home, START_ORIENT)
        if ik is None:
            say("[起始] IK 解 pick④ 提起位失败，退回 home")
            ik = home
        ik = np.asarray(ik, dtype=float)
        ik[6] -= np.pi / 2   # flange roll：只动最后一个关节 -90°（同 FlangeRollAction）
        return ik

    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_spatula_world(self, world_matrix):
        """把药匙写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.spatula_path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _set_spatula_from_gripper(self):
        # 行向量约定：先 _T_HELD（药匙局部→夹爪局部）再 tool_world（局部→世界）。
        self._set_spatula_world(_T_HELD * self._tool_world())

    def _update_spatula(self):
        """每帧药匙持握：attached → 跟随夹爪；grip 开 → 松回架；近架合爪 → 重新夹持。"""
        joints = self.robot.get_joint_positions()
        if joints is None:
            return
        opening = joints[7]
        if self.spatula_state == "attached":
            self._set_spatula_from_gripper()
            if opening > GRIP_OPEN_THRESH:
                self.spatula_state = "released"
                self._set_spatula_world(_rest_matrix())
                say("[药匙] 松开 → 回到架内竖插位")
        elif self.spatula_state == "released":
            gripper_pos = self.robot.get_gripper_position()
            if (gripper_pos is not None and opening <= GRIP_OPEN_THRESH
                    and self._near_spatula(gripper_pos)):
                self.spatula_state = "attached"
                self._set_spatula_from_gripper()
                say("[药匙] 重新夹持")

    def _near_spatula(self, gripper_pos):
        return (np.linalg.norm(gripper_pos[:2] - np.array(SPAT_GRASP)[:2]) < 0.05
                and abs(gripper_pos[2] - SPAT_GRASP[2]) < 0.02)

    # ------------------------------------------------------------- 按键
    def _handle(self, key):
        if key in MOVE_MAP:
            dx, dy, dz = MOVE_MAP[key]
            self.target_pos = self.target_pos + np.array([dx, dy, dz]) * self.step
            self._solve("移动")
        elif key in ("r", "f"):
            self._rotate("x", +1 if key == "r" else -1)
        elif key in ("t", "v"):
            self._rotate("y", +1 if key == "t" else -1)
        elif key in ("y", "n"):
            self._rotate("z", +1 if key == "y" else -1)
        elif key == "g":
            self.grip = GRIP_OPEN if self.grip <= GRIP_CLOSE + 0.001 else GRIP_CLOSE
            say(f"[爪] {'开' if self.grip > 0.03 else '合'} 宽度={self.grip}")
        elif key == " ":
            self._shot("check")
        elif key in ("\r", "\n"):
            self._save_wp()
        elif key == "u":
            if self.waypoints:
                self.waypoints.pop()
                say(f"[撤销] 剩 {len(self.waypoints)} 个 waypoint")
            else:
                say("[撤销] 没有 waypoint 可撤销")
        elif key == "+" or key == "=":
            self._cycle_step(+1)
        elif key == "-":
            self._cycle_step(-1)
        elif key == "?":
            self._help()
        elif key in ("\x1b", "q", "\x03"):
            self._export()
            self._running = False
        else:
            say(f"[按键] 未定义: {key!r}（按 ? 看帮助）")

    def _cycle_step(self, dir_):
        idx = MOVE_STEPS.index(self.step) if self.step in MOVE_STEPS else 2
        self.step = MOVE_STEPS[(idx + dir_) % len(MOVE_STEPS)]
        say(f"[步长] {self.step * 1000:.0f} mm")

    # ------------------------------------------------------------- 运动
    def _solve(self, action):
        cur = np.asarray(self.robot.get_joint_positions()[:7], dtype=float)
        ik = self.engine.solve_verified(self.target_pos, cur, self.orient_q)
        if ik is None:
            say(f"[{action}] IK 失败，目标不可达: {self.target_pos.round(4)}"
                f" —— 位置未更新")
            return
        self.target7 = np.asarray(ik, dtype=float)
        self._status(action)

    def _rotate(self, axis, sign):
        delta = Rotation.from_euler(axis, sign * ROT_STEP, degrees=True).as_matrix()
        new_rot = delta @ Rotation.from_quat(self.orient_q).as_matrix()
        self.orient_q = Rotation.from_matrix(new_rot).as_quat()
        self._solve("旋转")

    # ------------------------------------------------------------- 状态/截图
    def _status(self, action):
        cur = np.asarray(self.robot.get_joint_positions()[:7], dtype=float)
        pos, _ = self.engine.fk_pose(cur)
        dist_mm = float(np.linalg.norm(pos - self.target_pos)) * 1000.0
        grip_s = "开" if self.grip > 0.03 else "合"
        say(f"[{action}] 实际TCP={np.round(pos, 4)} 目标={np.round(self.target_pos, 4)}"
            f" 距={dist_mm:.1f}mm 步={self.step * 1000:.0f}mm"
            f" 爪={grip_s} wp={len(self.waypoints)}")

    def _settle(self):
        """等机械臂走到目标（截图/存点前用），超时 SETTLE_MAX 帧。"""
        for _ in range(SETTLE_MAX):
            cur = np.asarray(self.robot.get_joint_positions()[:7], dtype=float)
            pos, _ = self.engine.fk_pose(cur)
            if np.linalg.norm(pos - self.target_pos) < ARRIVE_EPS:
                return
            self._command_arm()
            self.world.step(render=True)

    def _shot(self, tag):
        self._settle()
        os.makedirs(SHOT_DIR, exist_ok=True)
        ts = time.strftime("%H%M%S")
        for name, cam in self.cameras.items():
            rgb = cam.get_rgb()
            if rgb is None:
                say(f"[截图] {name} 无图像（headless 首帧？再按一次空格）")
                continue
            bgr = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
            p = os.path.join(SHOT_DIR, f"{tag}_{ts}_{name}.png")
            cv2.imwrite(p, bgr)
            say(f"[截图] 已存 {p}")

    def _save_wp(self):
        self._settle()
        cur = np.asarray(self.robot.get_joint_positions()[:7], dtype=float)
        pos, rot = self.engine.fk_pose(cur)
        q = Rotation.from_matrix(np.asarray(rot)).as_quat()   # [x,y,z,w]
        self.waypoints.append({
            "pos": [float(v) for v in pos],
            "orient": [float(v) for v in q],
            "grip": float(self.grip),
        })
        say(f"[存点] waypoint {len(self.waypoints)}: TCP={np.round(pos, 4)}"
            f" orient[x,y,z,w]={np.round(q, 4)} 爪={'开' if self.grip > 0.03 else '合'}")
        self._shot("wp")

    # ------------------------------------------------------------- 导出
    def _export(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(OUT_DIR, exist_ok=True)
        p = os.path.join(OUT_DIR, f"teleop_waypoints_{ts}.json")
        with open(p, "w", encoding="utf-8") as f:
            json.dump({"n": len(self.waypoints), "waypoints": self.waypoints}, f, indent=2)
        say(f"\n=== 已写 {p}（{len(self.waypoints)} 个 waypoint）===")
        say("=== mv() 序列（粘进元动作 _build_actions）===")
        for i, wp in enumerate(self.waypoints):
            px, py, pz = [round(v, 4) for v in wp["pos"]]
            ox, oy, oz, ow = [round(v, 4) for v in wp["orient"]]
            say(f"  mv(e, ({px}, {py}, {pz}), orient=({ox}, {oy}, {oz}, {ow}), dwell=0),"
                f"  # {i + 1:02d} 爪={'开' if wp['grip'] > 0.03 else '合'}")

    @staticmethod
    def _help():
        say("=" * 72)
        say("D2-S 键盘遥操作录制（起始=法兰旋转完：药匙已夹起放平）")
        say("  移动(步长可调): w/s=z上下  a/d=x左右  q/e=y前后  方向键同 w/s/a/d")
        say("  旋转(10°):     r/f绕X  t/v绕Y  y/n绕Z      夹爪: g 开爪松药匙/合爪重夹")
        say("  空格=截图查看(固定A/B+手部随动)  回车=存waypoint   u=撤销  +/-=改步长  ?=帮助")
        say("  Esc或q=退出并导出 mv() 序列")
        say("=" * 72)


def disable_collision(root_path, stage):
    prim = stage.GetPrimAtPath(root_path)
    if not prim.IsValid():
        return
    stack = [prim]
    while stack:
        p = stack.pop()
        if UsdPhysics.CollisionAPI(p):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
        for c in p.GetChildren():
            stack.append(c)


def main():
    if not sys.stdin.isatty():
        print("需要真实终端（raw 模式读按键）。请在 SSH/本机终端里直接运行，"
              "不要在后台/管道里跑。")
        simulation_app.close()
        return

    hydra.initialize(config_path="../config", job_name="teleop_d2s")
    cfg = hydra.compose(config_name="level2_D2SWaterSolubility")

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)
    # 药匙碰撞关闭（与 task 一致），夹取才顺畅；台面/Cube 碰撞保留，防穿模
    disable_collision(SPATULA_PATH, stage)

    # 相机（构造先于 world.reset，initialize 后于 reset，同 base_task 顺序）
    cameras = {}
    for name, trans, orient in CAM_CONFIGS:
        cam = Camera(prim_path=f"/World/{name}", translation=np.array(trans),
                     name=name, frequency=60, resolution=CAM_RES)
        cam.set_local_pose(orientation=np.array(orient), camera_axes="usd")
        cam.prim.GetAttribute("focalLength").Set(10.0)
        cam.set_clipping_range(near_distance=0.05, far_distance=10.0)
        cameras[name] = cam
    # 第三相机：机械臂手部随动相机 = 实验 camera_3 同一 prim
    # （工厂已在 /World/Franka/panda_hand/arm_camera 创建，随臂移动）。
    # 视角必须和实验完全一致：姿态/焦距保持工厂默认（焦距=1.0 超广角，
    # 之前误设 10.0 造成 10 倍变焦、视角全错，2026-08-19 用户反馈）。
    # 只提分辨率便于看清；分辨率要等 initialize() 之后设（render product
    # 那时才创建，提前调 set_resolution 会崩）。
    arm_cam = robot.camera
    arm_cam.set_clipping_range(near_distance=0.05, far_distance=10.0)
    cameras["TeleopCamHand"] = arm_cam

    world.reset()
    # 必须先 robot.initialize() 关节才可用（get_joint_positions 才有值）；
    # 与 task.reset()（base_task.reset→world.reset 后 robot.initialize）同序
    robot.initialize()
    for cam in cameras.values():
        cam.initialize()
    # 手部相机 render product 此时已创建，才可把分辨率提到查看分辨率
    # （initialize 前 set_resolution 会因 _render_product_path=None 崩，
    #   2026-08-19 首跑实测报错）
    arm_cam.set_resolution(CAM_RES)

    # Lula IK 引擎（同 controller）
    mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
    solver = mg.LulaKinematicsSolver(
        robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
        urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
    rp, rq = robot.get_world_pose()
    solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
    engine = IkMotionEngine(solver, euler_angles_to_quat(np.array([0, np.pi, 0])),
                            np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741]))

    rec = TeleopRecorder(robot, engine, cameras, world, stage)
    # 直接置臂到「法兰旋转完」起始位姿（药匙已夹起），再进入遥操作。
    # 失败就退回驱动式就位（首帧命令 + 物理步进自然跟上）。
    try:
        robot.set_joint_positions(np.concatenate([rec.target7, [rec.grip, rec.grip]]))
    except Exception as e:
        say(f"[起始] set_joint_positions 失败（{e}），用驱动方式就位")
    for _ in range(10):
        rec._command_arm()
        world.step(render=True)
    rec._set_spatula_from_gripper()   # 药匙入位到手（法兰旋转完状态）

    say("已就位：法兰旋转完（药匙夹起放平），从舀粉开始操作。按 ? 看帮助（Esc/q 退出导出）")
    # 只在遥操作循环前才开终端原始模式（加载场景的几十秒终端保持正常）；
    # 必须在 simulation_app.close() 之前恢复——close() 有时 segfault，先恢复免留脏终端
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rec.run()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    simulation_app.close()


if __name__ == "__main__":
    main()
