"""诊断脚本：验证"瓶塞+灯帽转真刚体"（防穿模第 2 步）。

真刚体（RigidBodyAPI+CollisionAPI+凸包）已在 fix 脚本落盘到 v17.usd。实测
（本脚本早期版本）发现：瓶口凸包碰撞不建模口洞，真动态落座会把瓶塞顶飞
（err=0.099, z 顶到 0.951）。因此落座采用"盖到位后 kinematic 锁住"折中，
成功判定 = LiquidMixing 式读物理位姿（_verify_settle）。

本脚本验证：
  1. HCl 瓶塞盖回瓶口：kinematic 锁位 + 物理位姿读 → 应 OK（err≈0）。
  2. 灯帽盖灭：kinematic 锁位 + 物理位姿读 → 应 OK（err≈0）。
  3. 样例瓶塞桌面放置读位正常。

用法：conda activate labutopia; python scripts/diag_rigid.py
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

    def _step(n=8):
        for _ in range(n):
            world.step(render=True)
            task.step()

    # ---- 1. HCl 瓶塞盖回瓶口：kinematic 锁位 + 物理位姿读 ----
    task._set_obj_world("hcl_stopper", task.REST_POS["hcl_stopper"])
    task._set_kinematic("hcl_stopper", True)
    _step()
    ok1 = task._verify_settle("hcl_stopper", task.REST_POS["hcl_stopper"])
    print(f"[diag] hcl_stopper mouth lock: {'OK' if ok1 else 'WARN'}", flush=True)

    # ---- 2. 灯帽盖灭：kinematic 锁位 + 物理位姿读 ----
    task._set_obj_world("cap", task.CAP_SETTLED_POS)
    task._set_kinematic("cap", True)
    _step()
    ok2 = task._verify_settle("cap", task.CAP_SETTLED_POS)
    print(f"[diag] cap burner lock: {'OK' if ok2 else 'WARN'}", flush=True)

    # ---- 3. 样例瓶塞桌面放置读位 ----
    task._set_obj_world("sample_stopper", np.array([0.24, 0.08, task.TABLE_Z + 0.006]))
    task._set_kinematic("sample_stopper", True)
    _step()
    ss_pos = task._get_rigid_world("sample_stopper")
    print(f"[diag] sample_stopper table rest: physical={tuple(np.round(ss_pos, 4))}", flush=True)

    print(f"[diag] done. mouth={ok1} cap={ok2}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
