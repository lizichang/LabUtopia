"""B5 orientation probe v3: 决定性扫参。

2026-09-02 结论前置：
  - orient 元组 = (w,x,y,z) scalar-first（quats_to_rot_matrices 源码 q[[1,2,3,0]]→scipy）。
  - usd = C·fk，C=diag(−1,−1,1)（Lula right_gripper X/Y 相对 USD tool_center 翻转 180°）。
  - 温度计竖直化目标 = R_v（标准阵），列2 = (−0.8185,−0.5745,0)（由抓点朝向 R_grab 推出），
    旧 ORIENT_VERT=(0.7071,0,0.7071,0) 与 q_vert=(0.674,0.213,0.674,0.213) 均给错目标。
  - 若抓取用 ORIENT_DOWN（R_grab=diag(1,−1,−1)），竖直化目标解析 = ORIENT_FWD=(0,0.7071,0,0.7071)。

本探针：抓取用 ORIENT_DOWN 定死朝向 → 反解 held → 在高位扫参各候选朝向 → 读真值 rod，
看哪个让温度计竖直（rod→(0,0,1)）且泡朝下。毛细管段同 v2（强制 ORIENT_DOWN 三处一致）。

用法：python scripts/verify_b5_orient3.py
结果写 /tmp/b5_probe3_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe3_out.txt", "w", encoding="utf-8")


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

ORIENT_DOWN = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)      # (w,x,y,z) → RotY180 → 手指朝下
ORIENT_FWD = np.array([0.0, 0.7071, 0.0, 0.7071], dtype=float)  # (w,x,y,z) → 绕(1,0,1)/√2 180°
ORIENT_VERT_OLD = np.array([0.7071068, 0.0, 0.7071068, 0.0], dtype=float)  # (w,x,y,z) → RotY90


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


def drive_to_target(engine, robot, world, target, joints_cur, orient=None,
                    orient_eps=None, max_frames=6000, tol=0.002):
    """MoveAction-faithful：linewalk，每帧重解 IK（同 move_action 的 linewalk=True 路径）。"""
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


def tool_center_matrix(stage):
    tc = stage.GetPrimAtPath("/World/Franka/panda_hand/tool_center")
    return UsdGeom.Xformable(tc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


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


def rod_dir(held, tw):
    """温度计杆（局部+Z）世界方向 = (held·tw) 第3列。"""
    tw_w = held * tw
    return np.array([tw_w[i][2] for i in range(3)], dtype=float)


def tool_xyz(m):
    return tuple(round(float(m[3][i]), 4) for i in range(3))


def rot_row(m):
    return (np.round([m[0][0], m[0][1], m[0][2]], 4),
            np.round([m[1][0], m[1][1], m[1][2]], 4),
            np.round([m[2][0], m[2][1], m[2][2]], 4))


def main():
    log("PROBE3 START")
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

    # ============ A) 温度计：抓取定死 ORIENT_DOWN → 反解 held → 高位扫参竖直 ============
    THERMO_GRASP = (0.59, 0.22, 0.813)
    THERMO_HIGH = 1.05

    joints_cur = drive_to_target(engine, robot, world, THERMO_GRASP, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.10)
    tw_grab = tool_center_matrix(stage)
    log("\n[A] grab tool_center pos=%s rot=%s" % (tool_xyz(tw_grab), rot_row(tw_grab)))
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    held = thermo_rest * tw_grab.GetInverse()
    log("[A] held = rest · grab⁻¹ :")
    for i in range(4):
        log("   ", [round(float(held[i][j]), 4) for j in range(4)])

    joints_cur = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=ORIENT_DOWN, orient_eps=0.10)

    # 真值目标：想让 rod→(0,0,1)，则 tool+Z（世界）= held⁻¹·e_z
    Hinv = held.GetInverse()
    z_des = Hinv.TransformDirection(Gf.Vec3d(0.0, 0.0, 1.0))
    z_des = np.array([z_des[0], z_des[1], z_des[2]], dtype=float)
    R_des_usd = rot_from_z(z_des)
    C = np.diag([-1.0, -1.0, 1.0])
    fk_needed = C.T @ R_des_usd
    q_xyzw = rot_to_quat(fk_needed)
    q_dyn = np.round(np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]), 6)  # (w,x,y,z)
    log(f"\n[A] 真值 derived: z_des(需要 tool+Z 世界方向)={np.round(z_des,4)}")
    log(f"[A] R_des_usd=\n{np.round(R_des_usd,4)}")
    log(f"[A] dynamic orient (w,x,y,z)={q_dyn}")

    cands = {
        "ORIENT_FWD     (0,0.7071,0,0.7071)": ORIENT_FWD,
        "ORIENT_VERT_OLD(0.7071,0,0.7071,0)": ORIENT_VERT_OLD,
        "dynamic z_des": q_dyn,
    }
    best = None
    for lbl, q in cands.items():
        jc = drive_to_target(engine, robot, world, (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                             joints_cur, orient=np.asarray(q, dtype=float), orient_eps=0.08,
                             max_frames=4000)
        tw_v = tool_center_matrix(stage)
        rod = rod_dir(held, tw_v)
        bulb = -rod
        err = float(np.linalg.norm(rod - np.array([0, 0, 1])))
        ok = err < 0.06
        log(f"[A] {lbl}: rod->{np.round(rod,3)} bulb_dir->{np.round(bulb,3)} err={err:.4f}  {'VERTICAL' if ok else 'FAIL'}")
        if ok and (best is None or err < best[1]):
            best = (lbl, err, np.asarray(q, dtype=float))
        joints_cur = jc

    # 用最优朝向跑完整插管五段，确认全程竖直
    if best is not None:
        lbl, _, q_best = best
        log(f"\n[A] 最优朝向 = {lbl} q={np.round(q_best,4)}，跑插管五段确认全程竖直：")
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
                                         orient=q_best, orient_eps=0.05, max_frames=4000)
            tw = tool_center_matrix(stage)
            rod_i = rod_dir(held, tw)
            ok = np.linalg.norm(rod_i - np.array([0, 0, 1])) < 0.06
            log(f"[A] insert {lbl2}: rod->{np.round(rod_i,3)}  {'vertical' if ok else 'TILT'}")
    else:
        log("\n[A] 无候选竖直——插管段跳过")

    # ============ B) 毛细管：抓/蘸/贴 三处强制 ORIENT_DOWN ============
    CAP_MID = (0.45, 0.15, 0.813)
    OIL_DIP_GRIP = (0.30, 0.15, 0.804)
    STICK_SEALED = (0.356, 0.214, 0.813)
    STICK_GRIP = (0.406, 0.214, 0.813)

    joints_cur = drive_to_target(engine, robot, world, CAP_MID, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05)
    tw2 = tool_center_matrix(stage)
    cap_rest = prim_matrix(stage, "/World/CapillaryTube")
    cap_held = cap_rest * tw2.GetInverse()

    def sealed_at(held_m, tool_m):
        return held_m * tool_m

    def sealed_xyz(s):
        return (s[3][0], s[3][1], s[3][2])

    def tool_for_sealed(held_m, p_sealed):
        """由运行时 held 反解：封口端要落在 p_sealed，tool 平移应到哪。sealed = t_h + R_h·t。"""
        R = usd_rot_std(held_m)
        t_h = np.array([held_m[3][0], held_m[3][1], held_m[3][2]], dtype=float)
        return np.linalg.inv(R) @ (np.asarray(p_sealed, dtype=float) - t_h)

    log(f"\n[B] CAP_MID tool pos={tool_xyz(tw2)} rot={rot_row(tw2)}")

    # 老常量是给旧硬编码 held 调的；capture-at-attach 后须用运行时 held 反解正确 TCP。
    oil_desired = (0.25, 0.15, OIL_DIP_GRIP[2])
    oil_tool = tool_for_sealed(cap_held, oil_desired)
    stick_tool = tool_for_sealed(cap_held, STICK_SEALED)
    log(f"[B] 反解：封口端落油皿{tuple(np.round(oil_desired,3))} → tool 应到 {np.round(oil_tool,3)}")
    log(f"[B] 反解：封口端落泡 {tuple(np.round(STICK_SEALED,3))} → tool 应到 {np.round(stick_tool,3)}")

    joints_cur = drive_to_target(engine, robot, world, oil_tool, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05, max_frames=4000)
    tw3 = tool_center_matrix(stage)
    s_oil = sealed_at(cap_held, tw3)
    log(f"[B] OIL_DIP tool pos={tool_xyz(tw3)} rot={rot_row(tw3)}")
    log("[B] 反解 TCP 后蘸油落点：")
    check("oil sealed end @ dish center", sealed_xyz(s_oil), oil_desired, tol=0.012)

    joints_cur = drive_to_target(engine, robot, world, stick_tool, joints_cur,
                                 orient=ORIENT_DOWN, orient_eps=0.05, max_frames=4000)
    tw4 = tool_center_matrix(stage)
    s_stick = sealed_at(cap_held, tw4)
    log(f"[B] STICK   tool pos={tool_xyz(tw4)} rot={rot_row(tw4)}")
    log("[B] 反解 TCP 后贴泡落点：")
    check("stick sealed end @ bulb", sealed_xyz(s_stick), STICK_SEALED, tol=0.012)

    log("PROBE3 DONE")
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
