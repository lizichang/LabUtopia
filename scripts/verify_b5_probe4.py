"""B5 probe v4: 决定性全真值探针（MoveAction 忠实）。

背景（2026-09-02）：
  - orient 元组 = (w,x,y,z) scalar-first（quats_to_rot_matrices 源码 q[[1,2,3,0]]→scipy）。
  - usd = C·fk，C=diag(−1,−1,1)（v2b 实测，max dev 0.0135）。
  - 抓取用 orient=None（手指朝下目标但 check_orient=False → 朝向漂移，底座带 ~35° 偏航）。
    探针须与真码一致：抓取全用 orient=None。
  - capture-at-attach 后 held = rest·tool_grab⁻¹（零跳变，任务已做）。
  - 封口端世界 = (held·tw).translation = t_h + R_h·t_t，只依赖 tool 位置 → 与朝向漂移无关。
    故 OIL_DIP_GRIP/STICK_GRIP 须按运行时 held 反解（旧常量给旧硬编码 held 调）。
  - 温度计竖直：rod = (held·tw).col2 = R_h·(R_t·e_z)。要 rod=+Z → R_t col2 = R_h⁻¹·e_z。
    orient = wxyz(quat(Cᵀ·rot_from_z(R_h⁻¹·e_z)))。与抓取朝向无关（通用）。

本探针输出可直接硬编码的常量：
  [T] 温度计旋转竖直 orient (w,x,y,z)
  [B] 蘸油 TCP OIL_DIP_GRIP / 贴泡 TCP STICK_GRIP（由 held 数值反解）

用法：python scripts/verify_b5_probe4.py
结果写 /tmp/b5_probe4_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe4_out.txt", "w", encoding="utf-8")


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


def quat_std(q):
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


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
    """以 +Z 方向 z 构建标准旋转阵（右旋，det=+1）。"""
    z = np.asarray(z, dtype=float) / np.linalg.norm(z)
    x0 = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(x0, z)) > 0.9:
        x0 = np.array([0.0, 1.0, 0.0])
    x = np.cross(x0, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def usd_rot_std(m):
    """Gf 4x4（行=基像）-> 标准 3x3（列=基像）。"""
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


def drive_to_target(engine, robot, world, target, joints_cur, orient=None,
                    orient_eps=None, max_frames=6000, tol=0.002):
    """MoveAction 忠实：linewalk 每帧重解 IK。orient=None 同真码（手指朝下目标不验证）。"""
    target = np.asarray(target, dtype=float)
    cur = np.asarray(joints_cur, dtype=float)
    n_fail = 0
    for frame in range(max_frames):
        ik = engine.solve_verified(target, cur, orient, orient_eps)
        if ik is None:
            n_fail += 1
            if n_fail > 30:
                log(f"[probe] IK FAIL after {frame} frames target={np.round(target,3)} "
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
        if float(np.linalg.norm(cur - ik)) < tol:
            break
    return cur


def tool_xyz(m):
    return tuple(round(float(m[3][i]), 4) for i in range(3))


def rod_dir(held, tw):
    """对象局部+Z 世界方向 = (held·tw) 第3列（读 Gf 列2）。"""
    tw_w = held * tw
    return np.array([tw_w[i][2] for i in range(3)], dtype=float)


def sealed_xyz(w):
    return (w[3][0], w[3][1], w[3][2])


def solve_tool_for_origin(held, p_desired):
    """数值反解：对象局部原点要落 p_desired，tool 平移应到哪。
    f(t) = (held·T(t))[3][:3] 是仿射的：f(t)=a+B·t。用有限差分求 B 再解。"""
    def f(t):
        T = Gf.Matrix4d(1.0)
        T.SetTranslateOnly(Gf.Vec3d(*t))
        M = held * T
        return np.array([M[3][0], M[3][1], M[3][2]], dtype=float)

    t0 = np.array([0.0, 0.0, 0.0])
    f0 = f(t0)
    B = np.zeros((3, 3))
    eps = 1e-4
    for i in range(3):
        e = np.zeros(3)
        e[i] = eps
        B[:, i] = (f(t0 + e) - f0) / eps
    return np.linalg.solve(B, np.asarray(p_desired, dtype=float) - f0)


def check(label, got, expect, tol=0.012):
    got = [round(float(g), 4) for g in got[:3]]
    expect = [round(float(e), 4) for e in expect[:3]]
    err = float(np.linalg.norm(np.array(got) - np.array(expect)))
    log(f"  {'PASS' if err < tol else 'FAIL'} {label}: got={got} expect={expect} err={err:.4f} (tol={tol})")
    return err < tol


def main():
    log("PROBE4 START")
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

    # ============ T) 温度计：orient=None 抓取（同真码）→ 反解 held → 数值推导竖直 orient ============
    THERMO_GRASP = (0.59, 0.22, 0.813)
    THERMO_HIGH = 1.05
    joints_cur = drive_to_target(engine, robot, world, THERMO_GRASP, joints_cur, orient=None)
    tw_grab = tool_center_matrix(stage)
    log(f"\n[T] 温度计抓取 tool pos={tool_xyz(tw_grab)}")
    log(f"[T] 抓取 usd_rot=\n{np.round(usd_rot_std(tw_grab),4)}")
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    held = thermo_rest * tw_grab.GetInverse()
    log("[T] held = rest · grab⁻¹ :")
    for i in range(4):
        log("   ", [round(float(held[i][j]), 4) for j in range(4)])
    log(f"[T] rod at grab = {np.round(rod_dir(held, tw_grab),3)}  (期望 [1 0 0] 平躺)")

    # 竖直 orient：R_t col2 = R_h⁻¹·e_z → orient = wxyz(quat(Cᵀ·R_t))
    R_h = usd_rot_std(held)
    z_des = np.linalg.inv(R_h) @ np.array([0.0, 0.0, 1.0])
    R_t = rot_from_z(z_des)
    q_xyzw = rot_to_quat(C.T @ R_t)
    orient_vert = np.round(np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]), 6)
    log(f"[T] z_des(R_h⁻¹·e_z)={np.round(z_des,4)}  R_t col2 check={np.round(R_t[:,2],4)}")
    log(f"[T] >>> 竖直 orient (w,x,y,z) = {orient_vert}")

    # 抬起后原地旋转，验证竖直
    joints_cur = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=None)
    joints_cur = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=orient_vert, orient_eps=0.08, max_frames=4000)
    tw_v = tool_center_matrix(stage)
    rod = rod_dir(held, tw_v)
    bulb = -rod
    ok = np.linalg.norm(rod - np.array([0, 0, 1])) < 0.06
    log(f"[T] 旋转后 rod->{np.round(rod,3)} bulb_dir->{np.round(bulb,3)}  {'VERTICAL' if ok else 'FAIL'}")

    # 插管五段全程验证竖直
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
    for lbl2, pose in stages:
        joints_cur = drive_to_target(engine, robot, world, pose, joints_cur,
                                     orient=orient_vert, orient_eps=0.05, max_frames=4000)
        tw = tool_center_matrix(stage)
        rod_i = rod_dir(held, tw)
        ok = np.linalg.norm(rod_i - np.array([0, 0, 1])) < 0.06
        log(f"[T] insert {lbl2}: rod->{np.round(rod_i,3)}  {'vertical' if ok else 'TILT'}")

    # ============ B) 毛细管：orient=None 抓中部（同真码）→ 数值反解蘸油/贴泡 TCP ============
    CAP_MID = (0.45, 0.15, 0.813)
    CAP_HIGH = 1.05
    joints_cur = drive_to_target(engine, robot, world, (CAP_MID[0], CAP_MID[1], CAP_HIGH), joints_cur, orient=None)
    joints_cur = drive_to_target(engine, robot, world, CAP_MID, joints_cur, orient=None)
    tw2 = tool_center_matrix(stage)
    log(f"\n[B] 毛细管抓取 tool pos={tool_xyz(tw2)}")
    log(f"[B] 抓取 usd_rot=\n{np.round(usd_rot_std(tw2),4)}")
    cap_rest = prim_matrix(stage, "/World/CapillaryTube")
    cap_held = cap_rest * tw2.GetInverse()
    log("[B] cap_held = rest · grab⁻¹ :")
    for i in range(4):
        log("   ", [round(float(cap_held[i][j]), 4) for j in range(4)])
    log(f"[B] 封口端 at grab = {np.round(np.array(sealed_xyz(cap_held * tw2)),3)}  (期望 (0.40,0.15,0.813))")

    # 蘸油：封口端落油皿中心 (0.25,0.15) 沉油 z=0.804
    oil_desired = (0.25, 0.15, 0.804)
    oil_tool = solve_tool_for_origin(cap_held, oil_desired)
    log(f"[B] >>> 蘸油 TCP OIL_DIP_GRIP = {np.round(oil_tool,4)}")
    joints_cur = drive_to_target(engine, robot, world, oil_tool, joints_cur, orient=None, max_frames=4000)
    tw3 = tool_center_matrix(stage)
    s_oil = cap_held * tw3
    log(f"[B] 蘸油 tool pos={tool_xyz(tw3)}")
    check("oil sealed end @ dish center", sealed_xyz(s_oil), oil_desired, tol=0.012)

    # 贴泡：封口端落 STICK_SEALED=(0.356,0.214,0.813)
    STICK_SEALED = (0.356, 0.214, 0.813)
    stick_tool = solve_tool_for_origin(cap_held, STICK_SEALED)
    log(f"[B] >>> 贴泡 TCP STICK_GRIP = {np.round(stick_tool,4)}")
    joints_cur = drive_to_target(engine, robot, world, stick_tool, joints_cur, orient=None, max_frames=4000)
    tw4 = tool_center_matrix(stage)
    s_stick = cap_held * tw4
    log(f"[B] 贴泡 tool pos={tool_xyz(tw4)}")
    check("stick sealed end @ bulb", sealed_xyz(s_stick), STICK_SEALED, tol=0.012)

    log("PROBE4 DONE")
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
