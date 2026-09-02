"""B5 probe v6: 温度计竖直位姿可达性矩阵 + 顺序驱动验证（决定性）。

probe5 结论：旋转点 (0.59,0.22,1.05) 对所有 roll φ 均 IK FAIL（竖直朝向绝对不可达），
但 probe5 只报了「旋转点可达」的 φ，没单独报插管五段/携带位。本探针：
  1. 冷可达矩阵：pose × roll φ 全扫，报告每个位姿对哪些 φ 可达（竖直朝向）。
  2. 选一个「插管段可达」的 φ（优先）或「携带位可达」的 φ（次优）。
  3. 顺序驱动（真实路径）：抓取 → 携带位(orient=None) → 原地旋转(orient=φ) 验证竖直
     → 沿插管五段 linewalk 验证每段竖直。
  4. 若矩阵无任何可达 → 报告需换抓法/位姿。

用法：python scripts/verify_b5_probe6.py
结果写 /tmp/b5_probe6_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe6_out.txt", "w", encoding="utf-8")


def log(*a):
    _OUT.write(" ".join(str(x) for x in a) + "\n")
    _OUT.flush()


simulation_app = SimulationApp({"headless": True})

import hydra
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
import omni.usd
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller
from catalogue.factory import register_catalogue_actions

C = np.diag([-1.0, -1.0, 1.0])
ROD_EPS = 0.06
ORIENT_EPS_PROBE = 0.05


def rot_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w = s / 4.0
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    else:
        i = int(np.argmax([R[0][0], R[1][1], R[2][2]]))
        if i == 0:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + R[0][0] - R[1][1] - R[2][2]))
            w = (R[2][1] - R[1][2]) / s
            x = s / 4.0
            y = (R[0][1] + R[1][0]) / s
            z = (R[0][2] + R[2][0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + R[1][1] - R[0][0] - R[2][2]))
            w = (R[0][2] - R[2][0]) / s
            x = (R[0][1] + R[1][0]) / s
            y = s / 4.0
            z = (R[1][2] + R[2][1]) / s
        else:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + R[2][2] - R[0][0] - R[1][1]))
            w = (R[1][0] - R[0][1]) / s
            x = (R[0][2] + R[2][0]) / s
            y = (R[1][2] + R[2][1]) / s
            z = s / 4.0
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return q


def rot_from_z(z):
    z = np.asarray(z, dtype=float) / np.linalg.norm(z)
    x0 = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(x0, z)) > 0.9:
        x0 = np.array([0.0, 1.0, 0.0])
    x = np.cross(x0, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rot_z(phi_deg):
    phi = np.deg2rad(phi_deg)
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def usd_rot_std(m):
    return np.array([[m[0][0], m[1][0], m[2][0]],
                     [m[0][1], m[1][1], m[2][1]],
                     [m[0][2], m[1][2], m[2][2]]], dtype=float)


def tool_center_matrix(stage):
    tc = stage.GetPrimAtPath("/World/Franka/panda_hand/tool_center")
    return UsdGeom.Xformable(tc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def prim_matrix(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        log(f"[probe] prim not found: {path}")
        return None
    return UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def tool_xyz(m):
    return tuple(round(float(m[3][i]), 4) for i in range(3))


def rod_dir(held, tw):
    tw_w = held * tw
    return np.array([tw_w[2][0], tw_w[2][1], tw_w[2][2]], dtype=float)


def drive_to_target(engine, robot, world, stage, target, joints_cur, orient=None,
                    orient_eps=None, max_frames=6000, pos_tol=0.012, label=""):
    target = np.asarray(target, dtype=float)
    cur = np.asarray(joints_cur, dtype=float)
    n_fail = 0
    for frame in range(max_frames):
        ik = engine.solve_verified(target, cur, orient, orient_eps)
        if ik is None:
            n_fail += 1
            if n_fail > 30:
                log(f"[probe] IK FAIL {label} after {frame} frames target={np.round(target,3)} "
                    f"orient={None if orient is None else np.round(orient,4)}")
                break
            world.step(render=True)
            continue
        ik = np.asarray(ik, dtype=float)
        n_fail = 0
        delta = np.clip(ik - cur, -engine.MAX_JOINT_DELTA, engine.MAX_JOINT_DELTA)
        cur = cur + delta
        cmd = np.full(9, np.nan)
        cmd[:7] = cur
        robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=cmd))
        world.step(render=True)
        tw = tool_center_matrix(stage)
        got = np.array([tw[3][0], tw[3][1], tw[3][2]], dtype=float)
        if float(np.linalg.norm(got - target)) < pos_tol:
            break
    return cur


def main():
    log("PROBE6 START")
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    hydra.initialize(config_path="../config", job_name="verify_b5")
    cfg = hydra.compose(config_name="level2_B5MeltingPoint")

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)
    register_catalogue_actions()

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    controller = create_controller(cfg.controller_type, cfg=cfg, robot=robot)
    task.reset()
    engine = controller.engine
    log("controller engine ready")

    for _ in range(3000):
        joints = robot.get_joint_positions()
        if joints is not None and float(np.linalg.norm(joints[:7] - engine.ik_home)) < 0.05:
            break
        world.step(render=True)

    joints_cur = np.asarray(robot.get_joint_positions()[:7], dtype=float)
    log("joints home:", np.round(joints_cur, 3))

    # ============ 抓取温度计（orient=None，同真码）→ 反解 held ============
    THERMO_GRASP = (0.59, 0.22, 0.813)
    THERMO_HIGH = 1.05
    joints_cur = drive_to_target(engine, robot, world, stage, THERMO_GRASP, joints_cur,
                                 orient=None, label="T-grab")
    tw_grab = tool_center_matrix(stage)
    log(f"\n[T] 温度计抓取 tool pos={tool_xyz(tw_grab)}")
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    held = thermo_rest * tw_grab.GetInverse()
    log(f"[T] rod at grab = {np.round(rod_dir(held, tw_grab),3)}  (期望 [1 0 0])")
    R_h = usd_rot_std(held)
    z_des = np.linalg.inv(R_h) @ np.array([0.0, 0.0, 1.0])
    R_t0 = rot_from_z(z_des)

    # ============ 位姿 × roll 冷可达矩阵 ============
    TUBE_XY = (0.46, 0.0029)
    STAGE_XY = (0.40, 0.22)
    insert_z = 0.939 + 0.242
    poses = [
        ("插管⑤下探", (TUBE_XY[0], TUBE_XY[1], insert_z)),
        ("插管④对齐x", (TUBE_XY[0], TUBE_XY[1], 1.32)),
        ("插管③对齐y", (STAGE_XY[0], TUBE_XY[1], 1.32)),
        ("插管②清高", (STAGE_XY[0], STAGE_XY[1], 1.32)),
        ("插管①中转", (STAGE_XY[0], STAGE_XY[1], THERMO_HIGH)),
        ("旋转点", (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH)),
        ("带a(0.45,0.30,1.10)", (0.45, 0.30, 1.10)),
        ("带b(0.42,0.30,1.20)", (0.42, 0.30, 1.20)),
        ("带c(0.40,0.32,1.15)", (0.40, 0.32, 1.15)),
        ("带d(0.44,0.28,1.25)", (0.44, 0.28, 1.25)),
        ("带e(0.40,0.30,1.30)", (0.40, 0.30, 1.30)),
        ("带f(0.44,0.25,1.35)", (0.44, 0.25, 1.35)),
        ("带g(0.42,0.30,1.35)", (0.42, 0.30, 1.35)),
        ("带h(0.40,0.20,1.40)", (0.40, 0.20, 1.40)),
        ("带i(0.46,0.10,1.40)", (0.46, 0.10, 1.40)),
        ("带j(0.48,0.15,1.40)", (0.48, 0.15, 1.40)),
        ("带k(0.46,0.0029,1.45)", (0.46, 0.0029, 1.45)),
        ("带l(0.59,0.22,1.30)", (0.59, 0.22, 1.30)),
        ("带m(0.50,0.22,1.10)", (0.50, 0.22, 1.10)),
        ("带n(0.55,0.22,1.20)", (0.55, 0.22, 1.20)),
    ]
    log(f"\n[T] 冷可达矩阵（{len(poses)} 位姿 × 24 roll φ）：")
    reach_by_pose = {}   # pose_idx -> [(φ, orient)]
    reach_by_phi = {}    # φ -> [pose_idx]
    for pi, (lbl, pose) in enumerate(poses):
        row = []
        for phi in range(0, 360, 15):
            R_t = R_t0 @ rot_z(phi)
            q_xyzw = rot_to_quat(C.T @ R_t)
            orient = np.round(np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]), 6)
            ik = engine.solve_verified(np.asarray(pose, dtype=float), joints_cur,
                                       orient, ORIENT_EPS_PROBE)
            if ik is not None:
                row.append(phi)
                reach_by_pose.setdefault(pi, []).append((phi, orient))
                reach_by_phi.setdefault(phi, []).append(pi)
        log(f"[T] {lbl:24s} 可达 φ: {row if row else '—'}")
    best_phi = None
    best_orient = None
    insert_idxs = set(range(0, 5))
    for phi, pis in reach_by_phi.items():
        if insert_idxs.issubset(set(pis)):
            best_phi = phi
            break
    if best_phi is None and reach_by_phi:
        best_phi = max(reach_by_phi, key=lambda p: len(set(reach_by_phi[p]) & insert_idxs))
    if best_phi is None:
        log("[T] 冷矩阵无任何 (位姿,φ) 可达 —— 需换抓法/位姿")
    else:
        for pi, lst in reach_by_pose.items():
            for f, o in lst:
                if f == best_phi:
                    best_orient = o
                    break
        log(f"[T] >>> 选用 φ={best_phi} 竖直 orient (w,x,y,z) = {best_orient}")
        rot_pose = None
        rot_lbl = None
        for pi in reach_by_phi.get(best_phi, []):
            if pi == 1:
                rot_pose = poses[pi][1]
                rot_lbl = poses[pi][0]
                break
        if rot_pose is None and reach_by_phi.get(best_phi):
            pi = reach_by_phi[best_phi][-1]
            rot_pose = poses[pi][1]
            rot_lbl = poses[pi][0]
        joints_cur = drive_to_target(engine, robot, world, stage,
                                     (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                     joints_cur, orient=None, label="T-lift")
        if rot_pose is None:
            rot_pose = (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH)
            rot_lbl = "旋转点"
        log(f"[T] 顺序驱动：到 {rot_lbl} {rot_pose} 旋转")
        joints_cur = drive_to_target(engine, robot, world, stage, rot_pose, joints_cur,
                                     orient=None, label="T-to-rot")
        joints_cur = drive_to_target(engine, robot, world, stage, rot_pose, joints_cur,
                                     orient=best_orient, orient_eps=ORIENT_EPS_PROBE, label="T-rot")
        tw_v = tool_center_matrix(stage)
        rod = rod_dir(held, tw_v)
        ok = np.linalg.norm(rod - np.array([0, 0, 1])) < ROD_EPS
        log(f"[T] 旋转后 rod->{np.round(rod,3)}  {'VERTICAL' if ok else 'FAIL'}  tool={tool_xyz(tw_v)}")
        for lbl2, pose in poses[0:5]:
            joints_cur = drive_to_target(engine, robot, world, stage, pose, joints_cur,
                                         orient=best_orient, orient_eps=ORIENT_EPS_PROBE, label=f"T-{lbl2}")
            tw = tool_center_matrix(stage)
            rod_i = rod_dir(held, tw)
            ok = np.linalg.norm(rod_i - np.array([0, 0, 1])) < ROD_EPS
            log(f"[T] insert {lbl2}: rod->{np.round(rod_i,3)}  {'vertical' if ok else 'TILT'}  tool={tool_xyz(tw)}")

    log("PROBE6 DONE")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log("EXCEPTION:")
        log(traceback.format_exc())
        _OUT.flush()
        try:
            simulation_app.close()
        except Exception:
            pass
