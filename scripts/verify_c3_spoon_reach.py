"""C3 燃烧匙把手朝向可达性探针（headless）。

问题：S4 两段朝向（远端 ORIENT_FWD 横夹 + 灯区手指朝下）切换 90° 致手腕旋转、
燃烧匙视觉跟着转。用户建议全程手指朝下（竖直夹，像 C4）。本探针实测：

  A) 手指朝下（orient=None，默认朝向）在燃烧匙把手不同高度抓点的 IK 可达性
  B) 对比 ORIENT_FWD 的可达性
  C) 灯区（FLAME_HOLD_TCP / 灯上方高位）手指朝下的可达性

把手中心线 x(z)=0.398+0.26z（z=0.90→0.632、z=1.00→0.658，斜度 0.26）。
底座 [-0.15,0.05,0.71]；灯已移 (0.30,0.38) → BOWL_AT_FLAME (0.30,0.38,0.912)。

用法：conda run -n labutopia python scripts/verify_c3_spoon_reach.py
结果写 /tmp/c3_spoon_reach_out.txt。
"""
import os
import sys
import traceback
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_OUT = open("/tmp/c3_spoon_reach_out.txt", "w", encoding="utf-8")


def log(*a):
    _OUT.write(" ".join(str(x) for x in a) + "\n")
    _OUT.flush()


simulation_app = SimulationApp({"headless": True})

import hydra
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import Usd

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller
from catalogue.factory import register_catalogue_actions

ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 引擎 (w,x,y,z)，手指朝前


def main():
    log("C3 SPOON REACH START")
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    hydra.initialize(config_path="../config", job_name="verify_c3_spoon")
    cfg = hydra.compose(config_name="level2_C3CombustionSolid")

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
    log("robot base:", cfg.robot.position, "engine ready")

    for _ in range(3000):
        joints = robot.get_joint_positions()
        if joints is not None and float(np.linalg.norm(joints[:7] - engine.ik_home)) < 0.05:
            break
        world.step(render=True)
    joints_home = np.asarray(robot.get_joint_positions()[:7], dtype=float)
    log("joints home:", np.round(joints_home, 3))

    # 把手中心线 x(z) = 0.398 + 0.26*z
    def rod_xy(z):
        return (0.398 + 0.26 * z, 0.250)

    # 各高度抓点（手指朝下 vs ORIENT_FWD 对比）
    grab_zs = [0.84, 0.86, 0.88, 0.90, 0.93, 0.95, 0.98, 1.00]
    lamp_z = 1.08
    flame_hold = (0.344, 0.38, 1.0352)   # 当前 SPOON_GRASP (0.640,0.25,0.93) 的派生值

    def test(name, target, orient, eps=0.08):
        ik = engine.solve_verified(np.asarray(target, float), joints_home,
                                   orient=orient, orient_eps=eps)
        if ik is None:
            log(f"  [{name}] orient={'DOWN' if orient is None else 'FWD'} "
                f"target={np.round(target,3)} -> IK FAIL")
            return False
        p, r = engine.fk_pose(np.asarray(ik, float))
        dist = float(np.linalg.norm(np.asarray(p) - np.asarray(target)))
        log(f"  [{name}] orient={'DOWN' if orient is None else 'FWD'} "
            f"target={np.round(target,3)} -> OK (fk dist {dist:.4f}m)")
        return True

    log("\n=== A/B) 燃烧匙把手各高度抓点可达性 ===")
    for z in grab_zs:
        x, y = rod_xy(z)
        target = (x, y, z)
        dist = float(np.linalg.norm(np.array(target) - np.array(cfg.robot.position)))
        log(f"[grab z={z}] rod({x:.3f},{y},{z}) 3D={dist:.3f}m")
        test("grab", target, None)        # 手指朝下
        test("grab", target, ORIENT_FWD)  # 手指朝前

    log("\n=== C) 灯区手指朝下可达性 ===")
    # 勺位上方提出高位（手指朝下，抓点 z=0.93 的 x）
    gx, gy = rod_xy(0.93)
    log(f"[lift above spoon] ({gx:.3f},{gy},{lamp_z})")
    test("lift_spoon", (gx, gy, lamp_z), None)
    # 灯上方高位（手指朝下）
    log(f"[lift above lamp] ({flame_hold[0]:.3f},{flame_hold[1]},{lamp_z})")
    test("lift_lamp", (flame_hold[0], flame_hold[1], lamp_z), None)
    # 灯区下探外焰（手指朝下）
    log(f"[flame hold] {flame_hold}")
    test("flame_hold", flame_hold, None)

    log("\n=== D) 灯区 ORIENT_FWD 可达性（全程横夹方案）===")
    log(f"[lift above spoon FWD] ({gx:.3f},{gy},{lamp_z})")
    test("lift_spoon", (gx, gy, lamp_z), ORIENT_FWD)
    log(f"[lift above lamp FWD] ({flame_hold[0]:.3f},{flame_hold[1]},{lamp_z})")
    test("lift_lamp", (flame_hold[0], flame_hold[1], lamp_z), ORIENT_FWD)
    log(f"[flame hold FWD] {flame_hold}")
    test("flame_hold", flame_hold, ORIENT_FWD)

    log("C3 SPOON REACH DONE")
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
