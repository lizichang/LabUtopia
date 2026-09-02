"""B5 ground-truth probe: 驱动臂到「手指朝下」抓取位，读 tool_center 世界矩阵，
反解温度计/毛细管矩阵持握的正确 _HELD 矩阵（零跳变要求 world = HELD·tool_world）。

2026-09-01 扩展（capture-at-attach 修复后）验证四点：
  ① 毛细管 CAP_MID 夹持反解 held = rest·tool⁻¹ 后，横移到 OIL_DIP_GRIP → 封口端落 (0.25,0.15)=油皿中心
  ② 再横移到 STICK_GRIP → 封口端落 (0.356,0.214,0.813)=泡中心-Y 侧
  ③ 温度计 THERMO_GRASP 反解 held 后，原地旋转 ORIENT_VERT → 杆竖立（局部+Z→世界+Z）、泡朝下
  ④ tool_center 在 CAP_MID/OIL_DIP_GRIP/STICK_GRIP 三处朝向一致（手指朝下，封口端偏移恒定）

用法：python scripts/verify_b5_tool_orient.py
结果写 /tmp/b5_probe_out.txt（避开 Isaac stderr 噪音）。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe_out.txt", "w", encoding="utf-8")


def log(*a):
    msg = " ".join(str(x) for x in a)
    _OUT.write(msg + "\n")
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
from catalogue.b_thermal.b5_melting_point.meta_actions.constants import ORIENT_VERT


def print_mat(label, m):
    log(f"--- {label} (Gf 行向量: row0=X row1=Y row2=Z, row3=trans) ---")
    for i in range(4):
        log("   ", [round(float(m[i][j]), 4) for j in range(4)])


def drive_to_target(engine, robot, world, target, joints_cur, orient=None, max_frames=3000):
    """单次 IK + 关节钳制逼近（同 MoveAction 单次 IK 模式），返回到达时关节。"""
    target = np.asarray(target, dtype=float)
    ik = engine.solve_verified(target, joints_cur, orient)
    if ik is None:
        log(f"[probe] IK FAIL target={np.round(target,3)} orient={orient}")
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
        if float(np.linalg.norm(cur - ik)) < 0.02:
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
    ok = err < tol
    log(f"  {'PASS' if ok else 'FAIL'} {label}: got={got} expect={expect} err={err:.4f}")
    return ok


def main():
    log("PROBE START")
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
    log("joints:", np.round(joints_cur, 3))

    # ============ 温度计：attach 反解 + ORIENT_VERT 立起 ============
    THERMO_GRASP = (0.59, 0.22, 0.813)
    joints_cur = drive_to_target(engine, robot, world, THERMO_GRASP, joints_cur)
    tw = tool_center_matrix(stage)
    print_mat("tool_center @ THERMO_GRASP (fingers-down)", tw)
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    print_mat("MainThermometer rest", thermo_rest)
    correct_held = thermo_rest * tw.GetInverse()
    log("\n=== 反解正确 _THERMO_HELD = rest · tool_world⁻¹ ===")
    print_mat("correct _THERMO_HELD", correct_held)

    # 原地旋转竖立：测试候选目标朝向（solve_verified orient 参数），看哪个让温度计杆竖直。
    log(f"\nengine.orient (self.orient) = {np.round(np.asarray(engine.orient, dtype=float),5)}")

    def ham(a, b):
        x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
        return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
                w1*w2 - x1*x2 - y1*y2 - z1*z2)

    e_orient = np.asarray(engine.orient, dtype=float)
    q_ry_neg90 = np.array([0.0, -0.7071068, 0.0, 0.7071068])
    cands = {
        "A ham(ry-90, e.orient)": np.array(ham(q_ry_neg90, e_orient)),
        "B ham(e.orient, ry-90)": np.array(ham(e_orient, q_ry_neg90)),
        "OLD ORIENT_VERT": np.asarray(ORIENT_VERT, dtype=float),
        # 解析正解：世界朝向使杆竖直（probe 反解 rest·tool_grab⁻¹ 后推导）
        "C analytic [0.674,0.213,0.674,0.213]": np.array([0.67404, 0.21305, 0.67448, 0.21298]),
    }
    for lbl, q in cands.items():
        joints_cur = drive_to_target(engine, robot, world,
                                     (THERMO_GRASP[0], THERMO_GRASP[1], 1.05),
                                     joints_cur, orient=q, max_frames=1800)
        tw_v = tool_center_matrix(stage)
        thermo_world = correct_held * tw_v
        rod = [thermo_world[i][2] for i in range(3)]
        ok = np.linalg.norm(np.array(rod) - np.array([0, 0, 1])) < 0.08
        log(f"[vert] {lbl}: q={np.round(q,4)} rod->{np.round(rod,3)}  {'PASS vertical' if ok else 'FAIL'}")

    # ============ 毛细管：attach 反解 + 蘸油落点 + 贴泡落点 ============
    CAP_MID = (0.45, 0.15, 0.813)
    joints_cur = drive_to_target(engine, robot, world, CAP_MID, joints_cur)
    tw2 = tool_center_matrix(stage)
    print_mat("tool_center @ CAP_MID (fingers-down)", tw2)
    cap_rest = prim_matrix(stage, "/World/CapillaryTube")
    print_mat("CapillaryTube rest", cap_rest)
    cap_held = cap_rest * tw2.GetInverse()
    log("\n=== 反解正确 _CAP_HELD = rest · tool_world⁻¹ ===")
    print_mat("correct _CAP_HELD", cap_held)

    def sealed_at(held, tool_m):
        """attach 反解 held 下，tool 到 tool_m 时封口端（局部原点）世界坐标。"""
        return held * tool_m  # Transform((0,0,0)) = 第4行平移

    # ① 蘸油：tool 到 OIL_DIP_GRIP=(0.30,0.15)
    OIL_DIP_GRIP = (0.30, 0.15, 0.804)
    joints_cur = drive_to_target(engine, robot, world, OIL_DIP_GRIP, joints_cur)
    tw3 = tool_center_matrix(stage)
    s_oil = sealed_at(cap_held, tw3)
    log("\n=== 蘸油落点（OIL_DIP_GRIP 处封口端）===")
    check("oil sealed end @ dish center (0.25,0.15)",
          (s_oil[3][0], s_oil[3][1], s_oil[3][2]), (0.25, 0.15, OIL_DIP_GRIP[2]))

    # ② 贴泡：tool 到 STICK_GRIP=(0.406,0.214,0.813)
    STICK_GRIP = (0.406, 0.214, 0.813)
    joints_cur = drive_to_target(engine, robot, world, STICK_GRIP, joints_cur)
    tw4 = tool_center_matrix(stage)
    s_stick = sealed_at(cap_held, tw4)
    log("=== 贴泡落点（STICK_GRIP 处封口端）===")
    check("stick sealed end @ bulb (0.356,0.214,0.813)",
          (s_stick[3][0], s_stick[3][1], s_stick[3][2]), STICK_GRIP)

    # ④ 朝向一致性：三处 tool 旋转行应相等（手指朝下恒定）
    def rot_row(m):
        return (np.round([m[0][0], m[0][1], m[0][2]], 4),
                np.round([m[1][0], m[1][1], m[1][2]], 4),
                np.round([m[2][0], m[2][1], m[2][2]], 4))
    r2, r3, r4 = rot_row(tw2), rot_row(tw3), rot_row(tw4)
    log("=== 朝向一致性（CAP_MID/OIL_DIP/STICK 手指朝下相同）===")
    log(f"CAP_MID   : {r2}")
    log(f"OIL_DIP   : {r3}")
    log(f"STICK     : {r4}")
    same23 = rot_row(tw2 * tw3.GetInverse())
    log(f"CAP·OIL⁻¹ rot rows: {same23}  (≈identity 则一致)")

    log("PROBE DONE")
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
