"""诊断脚本：对比 3 种液滴材质在 camera_2 奶油白皿背景上的可视对比度。

背景：v30 液滴已放大到 10mm（渲染 30x29px 居中），但淡水色材质在白色皿上
不可见（上一轮 red diag 证明几何 OK，材质被背景吃掉）。本脚本逐个绑定候选
材质渲染，测量"液滴斑均值 vs 周边背景均值"的对比度 delta，选最大者。

候选：
  1. 深玻璃（dark glass）：深灰体 + 低粗糙高光
  2. 中调清液（mid clear）：偏灰清液
  3. 亮镜面（bright specular）：高反光白

判据：delta = |blob_mean - bg_mean| 最大者胜出（且 blob 不为蓝、不刺眼）。
用法：conda activate labutopia; python scripts/diag_droplet_mats.py
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
DROP_POS = np.array([0.20, 0.02, 0.83])

# 候选材质（diffuse, emissive, opacity, roughness, metallic）
# 结论（diag_droplet_pos.py + USD 读取）：RTX 把 opacity<1 的球当"薄玻璃壳"，
# 只认 emissive 自发光，低 emissive 渲染成透明（背景透出）→ 不可见；高 emissive
# 可见（red 5.0 验证）。所以液滴必须强 emissive。这里扫 emissive 强度，测对比度：
CANDIDATES = {
    "dgray_neutral": ((0.40, 0.40, 0.42), (0.08, 0.08, 0.09), 0.95, 0.7, 0.0),
    "dgray_yellow": ((0.40, 0.40, 0.42), (0.10, 0.09, 0.08), 0.95, 0.7, 0.0),
    "dgray_nearblk": ((0.18, 0.18, 0.20), (0.06, 0.06, 0.07), 0.95, 0.7, 0.0),
    "dgray_pale": ((0.55, 0.55, 0.57), (0.10, 0.10, 0.11), 0.95, 0.7, 0.0),
}


def _bind_material(stage, name, diffuse, emissive, opacity, roughness, metallic):
    mat_path = f"/World/DiagMat_{name}"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Principled_BSDF")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(DROPLET)).Bind(
        UsdShade.Material(stage.GetPrimAtPath(mat_path)))


def _grab(world, task):
    state = task.get_basic_state_info(object_path=task.wire_path)
    return np.asarray(state["camera_display"]["camera_2"].transpose(1, 2, 0),
                      dtype=np.float32)


def _contrast(img, cx=255, cy=252, r_in=15, r_out=28):
    """blob(半径 r_in 圆) vs 背景环(annulus) 的对比度。"""
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    d = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    blob = d < r_in
    ring = (d > r_in + 4) & (d < r_out)
    bm = img[blob].mean(axis=0)
    rm = img[ring].mean(axis=0)
    delta = np.abs(bm - rm).mean()
    return bm, rm, delta


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

    task._set_obj_world_plain(DROPLET, DROP_POS)
    task._set_visibility(DROPLET, True)

    results = {}
    for name, (diffuse, emissive, opacity, rough, metal) in CANDIDATES.items():
        _bind_material(stage, name, diffuse, emissive, opacity, rough, metal)
        for _ in range(6):
            world.step(render=True)
        img = _grab(world, task)
        bm, rm, delta = _contrast(img)
        results[name] = (bm, rm, delta)
        print(f"[diag] {name}: blob RGB=({bm[0]:.0f},{bm[1]:.0f},{bm[2]:.0f}) "
              f"bg RGB=({rm[0]:.0f},{rm[1]:.0f},{rm[2]:.0f}) delta={delta:.0f}", flush=True)
        cv2.imwrite(f"/tmp/droplet_mat_{name}.png",
                    cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR))

    best = max(results.items(), key=lambda kv: kv[1][2])
    print(f"[diag] WINNER: {best[0]} delta={best[1][2]:.0f}", flush=True)
    print("[diag] done. images at /tmp/droplet_mat_*.png", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
