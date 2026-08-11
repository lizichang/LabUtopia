"""诊断脚本：隔离 RMP 到位问题（完整运行大量 seg force-done、gripper 卡在 z~1.25）。

假设 A：kinematic 真刚体（瓶塞/灯帽的 CollisionAPI+凸包，任务 #2）干扰 RMP 规划。
假设 B：RMP 随机初始化差（长期问题），与刚体无关。

本脚本在当前场景（带真刚体 USD）下，用 controller.rmp_controller 命令夹爪依次
到达若干目标点（远离刚体的目标为主），每目标最多跑 RMP_FRAMES 帧，记录最小距离。

判读：
  - 若远目标（match/acid dip/flame，均不在瓶塞/灯帽附近）也收敛不了
    → 刚体碰撞不是主因（假设 B 占优）。
  - 若远目标能收敛、只有瓶塞/灯帽附近的目标不行 → 刚体碰撞是主因（假设 A）。

用法：conda activate labutopia; python scripts/diag_rmp.py
"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
os.chdir(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

simulation_config = {
    "headless": True,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true"],
}
simulation_app = SimulationApp(simulation_config)

import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from isaacsim.core.utils import extensions
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller

RMP_FRAMES = 300          # 每目标最多尝试帧数
CONVERGE = 0.012          # 到位判定（<1.2cm）
# 目标点：match 远离瓶塞/灯帽；acid dip / flame 也离刚体较远
TARGETS = {
    "match":       (0.5000, 0.2400, 0.8030),
    "acid_dip":    (0.2000, 0.0200, 0.9720),
    "flame":       (0.3600, 0.1800, 1.0880),
    "hcl_mouth":   (0.1200, 0.0200, 0.9300),   # 靠近 HCl 瓶口刚体
    "cap_grasp":   (0.4600, 0.2800, 0.9000),   # 靠近灯帽刚体
}


def main():
    ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    os.chdir(ROOT)
    from omegaconf import OmegaConf
    from datetime import datetime
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.now().strftime(fmt))
    cfg = OmegaConf.load(os.path.join(ROOT, "config", "level2_FlameTest.yaml"))

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    task.reset()
    controller = create_controller(cfg.controller_type, cfg=cfg, robot=robot)
    controller.reset()
    orient = euler_angles_to_quat(np.array([0, np.pi, 0]))

    for name, target in TARGETS.items():
        target = np.array(target, dtype=float)
        best = 1e9
        for i in range(RMP_FRAMES):
            rmp_action = controller.rmp_controller.forward(
                target_end_effector_position=target,
                target_end_effector_orientation=orient,
            )
            tp = np.concatenate([
                np.asarray(rmp_action.joint_positions, dtype=float), [np.nan, np.nan],
            ])
            robot.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=tp))
            world.step(render=True)
            gp = robot.get_gripper_position()
            if gp is None:
                continue
            dist = float(np.linalg.norm(gp - target))
            best = min(best, dist)
            if i % 50 == 0:
                print(f"[diag] {name} f={i} gripper={np.round(gp, 3)} dist={dist:.3f}", flush=True)
            if best < CONVERGE:
                break
        ok = best < CONVERGE
        print(f"[diag] RMP {name}: best={best:.4f} {'CONVERGED' if ok else 'FAIL'}", flush=True)

    print("[diag] done.", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
