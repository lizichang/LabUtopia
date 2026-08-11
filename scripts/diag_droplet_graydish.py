"""诊断脚本：灰皿 + 亮液滴组合的可见性测试。

结论链（diag_droplet_pos / mats / red）：RTX 不渲染液滴 diffuse（只认 emissive），
白皿 ~230 亮度使任何亮球对比度封顶 ~11。破局：把皿内粉末盘改灰（~130），
让强 emissive 液滴（~237）真正弹出。本脚本运行时绑"灰皿 + 多种亮液滴"测对比。

判据：blob vs ring 对比度 delta 越大越好（目标 >40），且液滴色不刺眼、非蓝。
用法：conda activate labutopia; python scripts/diag_droplet_graydish.py
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
from pxr import Sdf, Usd, UsdGeom, UsdShade, Gf

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task

DROPLET = "/World/Droplet"
DISH_DISK = "/World/SampleDish/powder_002_002/dish_visible_disk"
DROP_POS = np.array([0.20, 0.02, 0.83])

DISH_GRAY = (0.42, 0.42, 0.44)
DROP_CANDIDATES = {
    "neutral_22": ((0.01, 0.01, 0.01), (2.2, 2.2, 2.3), 0.95, 0.3),
    "yellow_recipe": ((0.01, 0.01, 0.01), (1.6, 0.55, 0.15), 0.95, 0.3),
    "teal_water": ((0.01, 0.01, 0.01), (0.7, 1.0, 1.4), 0.95, 0.3),
    "warm_amber": ((0.01, 0.01, 0.01), (1.6, 1.2, 0.6), 0.95, 0.3),
}


def _bind(stage, prim_path, mat_path, diffuse, emissive, opacity, rough, metal=0.0):
    if not stage.GetPrimAtPath(mat_path).IsValid():
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Principled_BSDF")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(prim_path)).Bind(
        UsdShade.Material(stage.GetPrimAtPath(mat_path)))


def _grab(world, task):
    state = task.get_basic_state_info(object_path=task.wire_path)
    return np.asarray(state["camera_display"]["camera_2"].transpose(1, 2, 0),
                      dtype=np.float32)


def _contrast(img, cx=255, cy=252, r_in=15, r_out=30):
    yy, xx = np.mgrid[0:512, 0:512]
    d = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    blob = d < r_in
    ring = (d > r_in + 5) & (d < r_out)
    bm = img[blob].mean(axis=0)
    rm = img[ring].mean(axis=0)
    return bm, rm, np.abs(bm - rm).mean()


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

    # 皿盘先改灰
    _bind(stage, DISH_DISK, "/World/DiagDishGray",
          DISH_GRAY, (0.08, 0.08, 0.09), 1.0, 0.7)
    for _ in range(6):
        world.step(render=True)

    task._set_obj_world_plain(DROPLET, DROP_POS)
    task._set_visibility(DROPLET, True)

    results = {}
    for name, (diffuse, emissive, opacity, rough) in DROP_CANDIDATES.items():
        _bind(stage, DROPLET, f"/World/DiagDrop_{name}",
              diffuse, emissive, opacity, rough)
        for _ in range(6):
            world.step(render=True)
        img = _grab(world, task)
        bm, rm, delta = _contrast(img)
        results[name] = delta
        print(f"[diag] {name}: droplet RGB=({bm[0]:.0f},{bm[1]:.0f},{bm[2]:.0f}) "
              f"dish RGB=({rm[0]:.0f},{rm[1]:.0f},{rm[2]:.0f}) delta={delta:.0f}", flush=True)
        cv2.imwrite(f"/tmp/graydish_{name}.png",
                    cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR))

    best = max(results.items(), key=lambda kv: kv[1])
    print(f"[diag] WINNER: {best[0]} delta={best[1]:.0f}", flush=True)
    print("[diag] done. images at /tmp/graydish_*.png", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
