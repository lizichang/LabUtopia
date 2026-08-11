"""诊断脚本：验证任务 #4 —— 抓取时"平滑提起"替代"悬空合爪+闪现吸附"。

模拟夹爪停在抓取点附近（带 ~3cm 偏移模拟 RMP 到位误差），逐帧递减 joint7
开度（开度 = joint_positions[7]），直接调 task._update_kin_objects(...)，
记录物体几何中心每帧位移：

  - max_jump：抓取期间单帧最大位移。
    旧行为：物体静止到闭合瞬间再 teleport 到夹爪 → max_jump ≈ 偏移(~0.03)。
    新行为（easing，k=0.18）：首帧只走 0.18×偏移(~0.005)，逐帧收敛 → max_jump 小。
  - attached：最终应进入 attached，持物位 = gripper + HELD_OFFSET。

判定：attached 且 max_jump < 0.010。

用法：conda activate labutopia; python scripts/diag_grasp.py
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

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task

MAX_JUMP = 0.010          # 单帧位移上限（平滑 vs 闪现的判据）
CLOSE_FRAMES = 25         # 与 controller 的 close seg 一致


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

    def _sim_grasp(name, offset):
        """模拟一次抓取：gripper 停在 grasp+offset，25 帧内开度 0.04→grip。"""
        task.reset()   # 清状态，物体回 rest
        grasp = task.GRASP_POINTS[name]
        grip = {  # controller 里各物体闭合宽度（joint7 目标值）
            "hcl_stopper": 0.0126, "sample_stopper": 0.0126,
            "dropper": 0.004, "match": 0.0015, "cap": 0.0185,
        }[name]
        gripper_pos = grasp + np.array(offset)
        opening0 = 0.04
        openings = np.linspace(opening0, grip, CLOSE_FRAMES)
        prev = task._get_obj_world(name)
        max_jump = 0.0
        attached = False
        for op in openings:
            task._update_kin_objects(gripper_pos, float(op))
            cur = task._get_obj_world(name)
            jump = float(np.linalg.norm(cur - prev))
            max_jump = max(max_jump, jump)
            prev = cur
            world.step(render=True)
            if task.kin_objs[name]["state"] == "attached":
                attached = True
                break
        held = gripper_pos + task.HELD_OFFSETS[name]
        final = task._get_obj_world(name)
        ok = attached and max_jump < MAX_JUMP
        print(f"[diag] grasp {name}: attached={attached} max_jump={max_jump:.4f} "
              f"final={tuple(np.round(final, 4))} held={tuple(np.round(held, 4))} "
              f"{'OK' if ok else 'WARN'}", flush=True)
        return ok

    # RMP 到位误差模拟：x/y/z 各偏移 1-3cm（旧代码这里会出现 ~3cm 闪现）
    off = (0.025, 0.015, 0.008)
    ok1 = _sim_grasp("hcl_stopper", off)
    ok2 = _sim_grasp("cap", off)
    ok3 = _sim_grasp("sample_stopper", off)
    print(f"[diag] done. hcl_stopper={ok1} cap={ok2} sample_stopper={ok3}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
