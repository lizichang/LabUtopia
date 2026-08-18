"""验证 d2s 截断动作（①夹取→②提起→③法兰-90°）的终态药匙姿态。

复用 main.py 的 setup（sim + 场景 + franka + task + controller），loop 到
controller 报 done，然后用引擎 ground truth 读 /World/Spatula 的世界变换，
推算药匙朝向（长轴 / 勺面法线是否水平、方向），并打印 tool_center 姿态。

用法：python scripts/verify_d2s_pose.py
"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

# scripts/ 下运行：把仓库根目录加进 sys.path 以便 import factories/ utils/ catalogue/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

simulation_app = SimulationApp({"headless": True})

import hydra
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import Usd, UsdGeom
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller
from catalogue.factory import register_catalogue_actions


def main():
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # hydra.initialize 的 config_path 必须是相对路径，且相对调用文件所在目录（scripts/）解析
    hydra.initialize(config_path="../config", job_name="verify_d2s")
    cfg = hydra.compose(config_name="level2_D2SWaterSolubility")

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)
    register_catalogue_actions()

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    controller = create_controller(cfg.controller_type, cfg=cfg, robot=robot)
    task.reset()

    max_steps = 30000
    step = 0
    while simulation_app.is_running() and step < max_steps:
        world.step(render=True)
        if not world.is_playing():
            continue
        state = task.step()
        if state is None:
            continue
        action, done, is_success = controller.step(state)
        if action is not None:
            robot.get_articulation_controller().apply_action(action)
        step += 1
        if done:
            print(f"\n[d2s] episode done at frame {step}, success={is_success}")
            print(f"[d2s] state: {state.get('additional_info', {})}")
            break

    # ground truth: spatula world transform (task 每帧写 _T_HELD * tool_world)
    spat = stage.GetPrimAtPath("/World/Spatula")
    if spat.IsValid():
        xf = UsdGeom.Xformable(spat).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        print("\n=== /World/Spatula world matrix (Gf row-vector) ===")
        for i in range(4):
            print("  ", [round(float(xf[i][j]), 4) for j in range(4)])
        # 行向量：行0=局部X、行1=局部Y、行2=局部Z（药匙在 _T_HELD 里局部X=长轴、局部Z=勺面法线）
        for name, row in (("长轴(局部X)", xf[0]), ("局部Y", xf[1]), ("勺面法线(局部Z)", xf[2])):
            v = np.array([row[0], row[1], row[2]])
            print(f"  {name}: {np.round(v,4)}  |z|={abs(v[2]):.3f} {'水平' if abs(v[2])<0.05 else '非水平'}")
    else:
        print("/World/Spatula not found")

    # 同时打印 tool_center 姿态，便于交叉核对
    tc = stage.GetPrimAtPath("/World/Franka/panda_hand/tool_center")
    if tc.IsValid():
        xf = UsdGeom.Xformable(tc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        print("\n=== tool_center world matrix ===")
        for i in range(3):
            print("  ", [round(float(xf[i][j]), 4) for j in range(4)])
        print("  trans:", np.round([xf[3][0], xf[3][1], xf[3][2]], 4))
    else:
        print("tool_center not found")

    simulation_app.close()


if __name__ == "__main__":
    main()
