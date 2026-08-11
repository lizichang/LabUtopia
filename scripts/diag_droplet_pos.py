"""诊断脚本：读取液滴 set_object_position 后的实际世界坐标，并渲染 camera_2。

背景：red diag 显示液滴渲染在皿中心(255,252)，但 vis diag（先隐藏再显示）的
diff 中心为 0、左上角却有 50x110 斑。本脚本不玩 baseline 隐藏，直接：
  1. reset 后把液滴定位到 DROP_POS 并置可见
  2. 用 UsdGeom.Xformable.ComputeWorldTransform 读回液滴实际世界坐标
  3. 渲染 camera_2，ASCII 显示中心 212x212 区域，肉眼确认深玻璃液滴斑
用法：conda activate labutopia; python scripts/diag_droplet_pos.py
"""
import os
import sys
import cv2
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
from pxr import Usd, UsdGeom

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task

DROPLET = "/World/Droplet"
DROP_POS = np.array([0.20, 0.02, 0.83])


def _ascii(sub, name, wcells=60):
    h, w = sub.shape[:2]
    rows = int(h / w * wcells * 0.5) + 1
    print(f"--- {name} ({h}x{w}) ---")
    for r in range(rows):
        line = []
        for c in range(wcells):
            px0 = int(c / wcells * w)
            px1 = int((c + 1) / wcells * w)
            py0 = int(r / rows * h)
            py1 = int((r + 1) / rows * h)
            v = sub[py0:py1, px0:px1].mean()
            line.append(" .:-=+*#%@"[min(int(v / 255 * 9), 9)])
        print("".join(line))


def main():
    from omegaconf import OmegaConf
    from datetime import datetime
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.now().strftime(fmt))
    cfg = OmegaConf.load("config/level2_FlameTest.yaml")

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    task.reset()
    for _ in range(8):
        world.step(render=True)

    # 直接定位 + 可见（不先隐藏）
    task._set_obj_world_plain(DROPLET, DROP_POS)
    task._set_visibility(DROPLET, True)
    for _ in range(6):
        world.step(render=True)

    # 读回液滴世界坐标
    prim = stage.GetPrimAtPath(DROPLET)
    wt = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = wt.ExtractTranslation()
    print(f"[diag] droplet world pos after set = ({t[0]:.4f},{t[1]:.4f},{t[2]:.4f}) "
          f"(want {DROP_POS})", flush=True)
    try:
        p = task._get_obj_world(DROPLET)
        print(f"[diag] task._get_obj_world(Droplet) = ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})", flush=True)
    except Exception as e:
        print(f"[diag] _get_obj_world failed: {e}", flush=True)

    state = task.get_basic_state_info(object_path=task.wire_path)
    img = np.asarray(state["camera_display"]["camera_2"].transpose(1, 2, 0),
                     dtype=np.float32)
    cv2.imwrite("/tmp/droplet_pos.png",
                cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR))
    _ascii(img[120:332, 140:372], "camera_2 center 212x232")
    simulation_app.close()


if __name__ == "__main__":
    main()
