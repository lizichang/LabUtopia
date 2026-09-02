"""B5 orientation probe v2: 测量 Lula FK 旋转 → USD tool_center 旋转 的恒定偏移 C，
并验证 (a) 用 C 修正后的「竖直温度计」朝向在旋转位/插管各段可达；(b) 毛细管强制统一
朝向（orient=[0,0,1,0]）后蘸油/贴泡封口端落点。

背景（2026-09-02）：
  - solve_verified 的 FK 朝向验证是对 Lula `right_gripper` 帧做的，但任务/探针拿
    tool_center 世界矩阵（USD）驱动持握对象。若两帧差一个恒定旋转 C，则 solve_verified
    通过 ≠ tool_center 真达到想要的绝对朝向 —— 这正是候选 C 解析竖直四元数却实测杆
    21° 歪的根因（B5 温度计「旋转好几圈、穿模、不竖直」）。
  - 毛细管 bug 与绝对朝向无关，是 orient=None 逐帧不验证朝向 → 抓/蘸/贴三处 tool 朝向
    漂移 → 封口端偏移 (−0.05,0) 被旋转 → 蘸油偏、贴泡错过 0.012m 判定窗（毛细管不粘、
    一直跟着臂）。修=三处强制同一 orient。

用法：python scripts/verify_b5_orient2.py
结果写 /tmp/b5_probe2_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe2_out.txt", "w", encoding="utf-8")


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

# 手指朝下目标（engine.orient，作为 (x,y,z,w)）
ORIENT_DOWN = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)


def quat_std(q):
    """(x,y,z,w) -> 标准旋转矩阵（列=基像）。"""
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def rot_to_quat(R):
    """标准旋转矩阵 -> (x,y,z,w)。"""
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


def drive_to_target(engine, robot, world, target, joints_cur, orient=None,
                    orient_eps=None, max_frames=3000, tol=0.02):
    target = np.asarray(target, dtype=float)
    ik = engine.solve_verified(target, joints_cur, orient, orient_eps)
    if ik is None:
        log(f"[probe] IK FAIL target={np.round(target,3)} orient={None if orient is None else np.round(orient,4)}")
        return joints_cur
    ik = np.asarray(ik, dtype=float)
    cur = np.asarray(joints_cur, dtype=float)
    for _ in range(max_frames):
        delta = np.clip(ik - cur, -engine.MAX_JOINT_DELTA, engine.MAX_JOINT_DELTA)
        cmd = cur + delta
        act = np.full(9, np.nan)
        act[:7] = cmd
        robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=act))
        world.step(render=True)
        cur = np.asarray(robot.get_joint_positions()[:7], dtype=float)
        if float(np.linalg.norm(cur - ik)) < tol:
            break
    return cur


def tool_center_matrix(stage):
    tc = stage.GetPrimAtPath("/World/Franka/panda_hand/tool_center")
    return UsdGeom.Xformable(tc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def usd_rot_std(m):
    """Gf 4x4（行=基像）-> 标准 3x3（列=基像）。"""
    return np.array([[m[0][0], m[1][0], m[2][0]],
                     [m[0][1], m[1][1], m[2][1]],
                     [m[0][2], m[1][2], m[2][2]]], dtype=float)


def prim_matrix(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        log(f"[probe] prim not found: {path}")
        return None
    return UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def check(label, got, expect, tol=0.012):
    got = [round(float(g), 4) for g in got[:3]]
    expect = [round(float(e), 4) for e in expect[:3]]
    err = float(np.linalg.norm(np.array(got) - np.array(expect)))
    log(f"  {'PASS' if err < tol else 'FAIL'} {label}: got={got} expect={expect} err={err:.4f} (tol={tol})")
    return err < tol


def main():
    log("PROBE2 START")
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

    # ============ 0) 测量 Lula FK → USD tool_center 恒定旋转偏移 C ============
    poses = [
        (0.59, 0.22, 0.813),   # 温度计抓点（手指朝下）
        (0.45, 0.15, 0.813),   # 毛细管中部抓点
        (0.40, 0.22, 1.05),    # 高位中转
    ]
    Cs = []
    for pose in poses:
        joints_cur = drive_to_target(engine, robot, world, pose, joints_cur, orient=ORIENT_DOWN,
                                     orient_eps=0.10, tol=0.01)
        fk_pos, fk_rot = engine.solver.compute_forward_kinematics("right_gripper", joints_cur)
        usd_m = tool_center_matrix(stage)
        usd_rot = usd_rot_std(usd_m)
        C = usd_rot @ np.asarray(fk_rot, dtype=float).T
        Cs.append(C)
        log(f"[C] pose={pose}\n   fk_rot=\n{np.round(np.asarray(fk_rot),4)}\n   usd_rot=\n{np.round(usd_rot,4)}\n   C=\n{np.round(C,4)}")
    C0 = np.mean(Cs, axis=0)
    U, _, Vt = np.linalg.svd(C0)
    C0 = U @ Vt
    if np.linalg.det(C0) < 0:
        U[:, -1] *= -1
        C0 = U @ Vt
    log(f"\n[C] mean C (Lula FK -> USD tool_center):\n{np.round(C0,4)}")
    spread = max(float(np.linalg.norm(C - C0)) for C in Cs)
    log(f"[C] max deviation from mean C = {spread:.4f}  (小=恒定偏移成立)")

    # ============ A) 温度计竖直：用 C 修正解析竖直四元数 ============
    # 2026-09-02 修正：quats_to_rot_matrices 对 orient 按 (w,x,y,z) scalar-first 解析
    # （源码 q[[1,2,3,0]] → scipy (x,y,z,w)）。此前的探针按 (x,y,z,w) 传 q_fk → Lula 解了
    # 错误的旋转 → 各段 IK FAIL（臂没动、杆仍水平）。现在先算 (x,y,z,w) 再换序 (w,x,y,z)。
    q_vert = np.array([0.67404, 0.21305, 0.67448, 0.21298], dtype=float)  # (x,y,z,w)
    R_des = quat_std(q_vert)
    R_fk_needed = C0.T @ R_des          # usd = C·fk → fk = Cᵀ·usd
    q_xyzw = rot_to_quat(R_fk_needed)   # (x,y,z,w)
    q_fk = np.round(np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]), 6)  # (w,x,y,z)
    log(f"\n[A] q_vert(x,y,z,w)={np.round(q_vert,4)}  R_des rod check: {np.round(R_des @ np.array([0.8185,0.5745,0.0]),4)}")
    log(f"[A] corrected q_fk (w,x,y,z)={q_fk}")

    THERMO_GRASP = (0.59, 0.22, 0.813)
    THERMO_HIGH = 1.05
    joints_cur = drive_to_target(engine, robot, world, THERMO_GRASP, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.10, tol=0.01)
    tw_grab = tool_center_matrix(stage)
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    held = thermo_rest * tw_grab.GetInverse()
    joints_cur = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=ORIENT_DOWN, orient_eps=0.10, tol=0.01)
    joints_cur = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=q_fk, orient_eps=0.05, max_frames=2500, tol=0.01)
    tw_v = tool_center_matrix(stage)
    thermo_world = held * tw_v
    rod = [thermo_world[i][2] for i in range(3)]
    log(f"[A] corrected orient: rod->{np.round(rod,3)}  (期望 (0,0,1))")
    check("thermometer vertical (rod->+Z)", rod, (0.0, 0.0, 1.0), tol=0.08)

    TUBE_XY = (0.46, 0.0029)
    STAGE = (0.40, 0.22)
    insert_z = 0.939 + 0.242
    stages = [
        ("① 中转(0.40,0.22,1.05)", (STAGE[0], STAGE[1], THERMO_HIGH)),
        ("② 清高(0.40,0.22,1.32)", (STAGE[0], STAGE[1], 1.32)),
        ("③ 对齐y(0.40,0.0029,1.32)", (STAGE[0], TUBE_XY[1], 1.32)),
        ("④ 对齐x(0.46,0.0029,1.32)", (TUBE_XY[0], TUBE_XY[1], 1.32)),
        ("⑤ 下探(0.46,0.0029,1.181)", (TUBE_XY[0], TUBE_XY[1], insert_z)),
    ]
    for lbl, pose in stages:
        joints_cur = drive_to_target(engine, robot, world, pose, joints_cur,
                                     orient=q_fk, orient_eps=0.05, max_frames=3000, tol=0.01)
        tw = tool_center_matrix(stage)
        thermo_w = held * tw
        rod_i = [thermo_w[i][2] for i in range(3)]
        ok = np.linalg.norm(np.array(rod_i) - np.array([0, 0, 1])) < 0.06
        log(f"[A] insert {lbl}: rod->{np.round(rod_i,3)}  {'vertical' if ok else 'TILT'}")

    # ============ B) 毛细管：三处强制同一朝向 ORIENT_DOWN ============
    CAP_MID = (0.45, 0.15, 0.813)
    OIL_DIP_GRIP = (0.30, 0.15, 0.804)
    STICK_SEALED = (0.356, 0.214, 0.813)
    STICK_GRIP = (0.406, 0.214, 0.813)

    # 贴合真实 MoveAction：freeze_dist≈0.01、orient_eps 0.05（≈2.9°）、
    # 收敛 tol=0.02（同 freeze 距离量级）。
    joints_cur = drive_to_target(engine, robot, world, CAP_MID, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05, tol=0.02)
    tw2 = tool_center_matrix(stage)
    cap_rest = prim_matrix(stage, "/World/CapillaryTube")
    cap_held = cap_rest * tw2.GetInverse()

    def sealed_at(held_m, tool_m):
        return held_m * tool_m

    def sealed_xyz(s):
        return (s[3][0], s[3][1], s[3][2])

    def tool_xyz(m):
        return tuple(round(float(m[3][i]), 4) for i in range(3))

    def rot_row(m):
        return (np.round([m[0][0], m[0][1], m[0][2]], 4),
                np.round([m[1][0], m[1][1], m[1][2]], 4),
                np.round([m[2][0], m[2][1], m[2][2]], 4))

    log(f"\n[B] CAP_MID tool pos={tool_xyz(tw2)} rot={rot_row(tw2)}")

    joints_cur = drive_to_target(engine, robot, world, OIL_DIP_GRIP, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05, max_frames=3000, tol=0.02)
    tw3 = tool_center_matrix(stage)
    s_oil = sealed_at(cap_held, tw3)
    log(f"[B] OIL_DIP tool pos={tool_xyz(tw3)} rot={rot_row(tw3)}")
    log("\n[B] 强制 ORIENT_DOWN 后蘸油落点：")
    check("oil sealed end @ dish center", sealed_xyz(s_oil), (0.25, 0.15, OIL_DIP_GRIP[2]), tol=0.012)

    joints_cur = drive_to_target(engine, robot, world, STICK_GRIP, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05, max_frames=3000, tol=0.02)
    tw4 = tool_center_matrix(stage)
    s_stick = sealed_at(cap_held, tw4)
    log(f"[B] STICK   tool pos={tool_xyz(tw4)} rot={rot_row(tw4)}")
    log("[B] 强制 ORIENT_DOWN 后贴泡落点：")
    check("stick sealed end @ bulb", sealed_xyz(s_stick), STICK_SEALED, tol=0.012)

    log("PROBE2 DONE")
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
