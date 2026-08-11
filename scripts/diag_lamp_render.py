"""诊断脚本：受控渲染实验，判断酒精灯是否真的渲染、渲染在哪里。

背景：用户反馈"全部相机都看不到酒精灯"。文件层验证（Sdf.Layer + Usd.Stage）
确认 /World/AlcoholLamp 存在于 v17.usd，几何完整、位置 (0.36,0.18,0.8)。
但快照渲染里灯可能"太小/太暗/材质不显"。

本脚本复用 main.py 启动流程：
  1. 基线渲染三相机 → /tmp/lamp_render/orig_*.png
  2. 把灯所有 mesh（body/holder/wick/cap/...）绑上亮红 emissive 材质 → 渲染
  3. 两张图做差：红色区域 = 灯在每台相机里的确切渲染像素（bbox）

判定：
  - 有差（红块出现在某相机）→ 灯在渲染；问题只是"不显眼"→ 修材质/光照。
  - 三相机都无差 → 灯根本没渲染，查 mesh/材质/可见性。

用法：conda activate labutopia; python scripts/diag_lamp_render.py
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
from pxr import Usd, Sdf, UsdGeom, UsdShade, Gf

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task

OUT = "/tmp/lamp_render"
LAMP_MESHES = [
    "/World/AlcoholLamp/body/body",
    "/World/AlcoholLamp/holder/holder",
    "/World/AlcoholLamp/holder_top/holder_top",
    "/World/AlcoholLamp/holder_bot/holder_bot",
    "/World/AlcoholLamp/cap/cap",
    "/World/AlcoholLamp/wick/wick",
    "/World/AlcoholLamp/wick_tip/wick_tip",
    "/World/AlcoholLamp/wick_top/wick_top",
    "/World/AlcoholLamp/liquid/liquid",
    "/World/AlcoholLamp/liquid_top/liquid_top",
]


def _grab(world, task):
    """从相机抓图，返回 {cam_name: HxWx3 RGB numpy}。"""
    state = task.get_basic_state_info(object_path=task.wire_path)
    out = {}
    for cam_name, image_data in state["camera_display"].items():
        arr = np.asarray(image_data.transpose(1, 2, 0), dtype=np.float32)
        out[cam_name] = arr
    return out


def _diff_bbox(a, b, thresh=40):
    d = np.abs(a - b).mean(axis=2)
    m = d > thresh
    if not m.any():
        return None, 0
    ys, xs = np.nonzero(m)
    return (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())), int(m.sum())


def main():
    ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    os.chdir(ROOT)
    from omegaconf import OmegaConf
    from datetime import datetime
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.now().strftime(fmt))
    cfg = OmegaConf.load(os.path.join(ROOT, "config", "level2_FlameTest.yaml"))
    os.makedirs(OUT, exist_ok=True)

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

    # 基线渲染
    base = _grab(world, task)
    for cam, arr in base.items():
        cv2.imwrite(os.path.join(OUT, f"orig_{cam}.png"),
                    cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2BGR))
    print(f"[diag] baseline saved to {OUT}", flush=True)

    # 先确认 runtime stage 里灯 prim 的状态
    lamp = stage.GetPrimAtPath("/World/AlcoholLamp")
    print(f"[diag] lamp valid={lamp.IsValid()} active={lamp.IsActive()}", flush=True)

    # 亮红 emissive 材质，绑到灯所有 mesh
    mat_path = "/World/LampDiagRed"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Principled_BSDF")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.9, 0.05, 0.05))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(3.0, 0.0, 0.0))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    nmats = 0
    for mesh_path in LAMP_MESHES:
        prim = stage.GetPrimAtPath(mesh_path)
        if prim.IsValid():
            UsdShade.MaterialBindingAPI(prim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path)))
            nmats += 1
    print(f"[diag] bound red mat to {nmats} lamp meshes", flush=True)

    # 多走几帧让 RTX 拾取材质变化，再渲染
    for _ in range(8):
        world.step(render=True)
    red = _grab(world, task)
    for cam, arr in red.items():
        cv2.imwrite(os.path.join(OUT, f"red_{cam}.png"),
                    cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2BGR))

    # 对比
    for cam in base.keys():
        bbox, cnt = _diff_bbox(base[cam], red[cam])
        h, w = red[cam].shape[:2]
        if bbox is None:
            print(f"[diag] {cam}: NO DIFF — 灯没渲染 (diff_px=0)", flush=True)
        else:
            x0, y0, x1, y1 = bbox
            print(f"[diag] {cam}: DIFF bbox=({x0},{y0})-({x1},{y1}) "
                  f"size=({x1-x0}x{y1-y0})px diff_px={cnt} img=({w}x{h})", flush=True)
            # 红块均值（证明真的是红，而不是偶然差异）
            r_region = red[cam][y0:y1+1, x0:x1+1]
            print(f"    red region mean RGB=({r_region[...,0].mean():.0f},"
                  f"{r_region[...,1].mean():.0f},{r_region[...,2].mean():.0f})", flush=True)

    print("[diag] done. files in /tmp/lamp_render/", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
