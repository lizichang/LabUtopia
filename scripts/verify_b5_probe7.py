"""B5 probe v7: 换抓取朝向使竖直=ORIENT_FWD（决定性验证）。

probe6 定论：当前抓法（orient=None，杆 ∥ tool+X）下，竖直朝向在 20 位姿×24 roll 全不可达。
本探针验证修复假设：抓取时把 tool+X 摆到世界 −Y（杆落在 tool+Y 方向），则 ORIENT_FWD
（b2 已在更远位姿验证可达）可把杆竖起来（ORIENT_FWD col1=(0,0,1)），泡朝下。

  1. G1 抓取朝向 (w,x,y,z)=(0.3536,0.7071,0,0)（tool+X=-Y,tool+Y=+X,tool+Z=-Z）在抓点可达？
  2. 抓后 held：杆是否仍世界 +X 平躺（rod_world_dir 验）。
  3. ORIENT_FWD 在抓提升位 + 插管五段全部可达？（solve 冷测）
  4. 顺序驱动：抓(G1)→提→旋转(ORIENT_FWD)→沿插管五段 linewalk，验每段杆→+Z。

用法：python scripts/verify_b5_probe7.py
结果写 /tmp/b5_probe7_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/b5_probe7_out.txt", "w", encoding="utf-8")


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
ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 引擎 (w,x,y,z)
G1 = (0.3536, 0.7071, 0.0, 0.0)          # tool+X=-Y,tool+Y=+X,tool+Z=-Z (w,x,y,z)


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


def rod_world_dir(held, tw):
    """对象局部+Z（杆轴）世界方向 = (held·tw) 行2（Gf 行=基像）。"""
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
    log("PROBE7 START")
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

    THERMO_GRASP = (0.59, 0.22, 0.813)
    THERMO_HIGH = 1.05

    # ============ 1) G1 抓取朝向冷可达测试（抓点 + 提升位）============
    log(f"\n[1] G1 抓取朝向 ORIENT={G1}")
    for lbl2, pose in [("抓点", THERMO_GRASP), ("提升位", (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH))]:
        ik = engine.solve_verified(np.asarray(pose, dtype=float), joints_cur, np.asarray(G1), 0.08)
        log(f"[1] {lbl2} {pose}: {'可达' if ik is not None else 'IK FAIL'}")

    # ============ 2) 顺序驱动：G1 抓取 ============
    joints_cur = drive_to_target(engine, robot, world, stage,
                                 (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=G1, orient_eps=0.08, label="T-approach")
    joints_cur = drive_to_target(engine, robot, world, stage, THERMO_GRASP, joints_cur,
                                 orient=G1, orient_eps=0.08, label="T-grab")
    tw_grab = tool_center_matrix(stage)
    log(f"\n[2] G1 抓取 tool pos={tool_xyz(tw_grab)}")
    log(f"[2] 抓取 usd_rot=\n{np.round(usd_rot_std(tw_grab),4)}")
    thermo_rest = prim_matrix(stage, "/World/MainThermometer")
    held = thermo_rest * tw_grab.GetInverse()
    rod = rod_world_dir(held, tw_grab)
    log(f"[2] rod world at grab = {np.round(rod,3)}  (期望 [1 0 0] 平躺不变)")

    # ============ 3) ORIENT_FWD 冷可达：提升位 + 插管五段 ============
    TUBE_XY = (0.46, 0.0029)
    STAGE_XY = (0.40, 0.22)
    insert_z = 0.939 + 0.242
    stages = [
        ("提升位", (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH)),
        ("插管①中转", (STAGE_XY[0], STAGE_XY[1], THERMO_HIGH)),
        ("插管②清高", (STAGE_XY[0], STAGE_XY[1], 1.32)),
        ("插管③对齐y", (STAGE_XY[0], TUBE_XY[1], 1.32)),
        ("插管④对齐x", (TUBE_XY[0], TUBE_XY[1], 1.32)),
        ("插管⑤下探", (TUBE_XY[0], TUBE_XY[1], insert_z)),
    ]
    log("\n[3] ORIENT_FWD 冷可达（solve_verified）:")
    for lbl2, pose in stages:
        ik = engine.solve_verified(np.asarray(pose, dtype=float), joints_cur,
                                   np.asarray(ORIENT_FWD), 0.05)
        log(f"[3] {lbl2} {pose}: {'可达' if ik is not None else 'IK FAIL'}")

    # ============ 4) 顺序驱动全链验证 ============
    joints_cur = drive_to_target(engine, robot, world, stage,
                                 (THERMO_GRASP[0], THERMO_GRASP[1], THERMO_HIGH),
                                 joints_cur, orient=ORIENT_FWD, orient_eps=0.08, label="T-lift")
    tw_v = tool_center_matrix(stage)
    rod_v = rod_world_dir(held, tw_v)
    ok = np.linalg.norm(rod_v - np.array([0, 0, 1])) < ROD_EPS
    log(f"\n[4] 旋转后 rod->{np.round(rod_v,3)}  {'VERTICAL' if ok else 'FAIL'}  tool={tool_xyz(tw_v)}")
    for lbl2, pose in stages[1:6]:
        joints_cur = drive_to_target(engine, robot, world, stage, pose, joints_cur,
                                     orient=ORIENT_FWD, orient_eps=0.05, label=f"T-{lbl2}")
        tw = tool_center_matrix(stage)
        rod_i = rod_world_dir(held, tw)
        ok = np.linalg.norm(rod_i - np.array([0, 0, 1])) < ROD_EPS
        log(f"[4] insert {lbl2}: rod->{np.round(rod_i,3)}  {'vertical' if ok else 'TILT'}  tool={tool_xyz(tw)}")

    log("PROBE7 DONE")
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
