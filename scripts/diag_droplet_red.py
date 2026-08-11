"""诊断脚本：给液滴绑亮红 emissive，定位它在 camera_2 里的确切渲染位置/大小。

用户反馈"机加盐酸的时候都看不到有东西滴加了上去"。上一轮 diff 发现液滴在皿中心
没有渲染出可见斑（中心 240x240 区仅 22px 零散噪声）。本脚本：
  1. 把液滴放到皿上方坠落点并置可见
  2. 给 /World/Droplet 绑亮红 emissive 材质（绕开"透明小球被背景吃掉"问题）
  3. 渲染 camera_2，统计红像素 bbox / 面积 / 质心
判定：
  - 红斑居中且 ~16px 级 → 几何/位置 OK，之前只是材质不可见 → 需要强化液滴材质
  - 红斑巨大/偏 → 液滴 scale 或位置错
  - 无红斑 → 液滴几何不渲染，查其它
用法：conda activate labutopia; python scripts/diag_droplet_red.py
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

    # 绑亮红 emissive
    mat_path = "/World/DropletDiagRed"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Principled_BSDF")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.02, 0.02))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(5.0, 0.0, 0.0))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    dp = stage.GetPrimAtPath(DROPLET)
    UsdShade.MaterialBindingAPI(dp).Bind(UsdShade.Material(stage.GetPrimAtPath(mat_path)))
    print(f"[diag] bound red mat to {DROPLET}", flush=True)

    for _ in range(8):
        world.step(render=True)
    state = task.get_basic_state_info(object_path=task.wire_path)
    img = np.asarray(state["camera_display"]["camera_2"].transpose(1, 2, 0),
                     dtype=np.float32)
    h, w = img.shape[:2]
    cv2.imwrite("/tmp/droplet_red.png",
                cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR))
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    red = (r > 120) & (r > g + 50) & (r > b + 50)
    ys, xs = np.nonzero(red)
    print(f"[diag] red px: {red.sum()}", flush=True)
    if red.sum() > 0:
        print(f"[diag] red bbox=({xs.min()},{ys.min()})-({xs.max()},{ys.max()}) "
              f"size=({xs.max()-xs.min()}x{ys.max()-ys.min()}) "
              f"centroid=({xs.mean():.0f},{ys.mean():.0f})", flush=True)
        print(f"[diag] red region mean RGB=({r[red].mean():.0f},{g[red].mean():.0f},"
              f"{b[red].mean():.0f})", flush=True)
    else:
        print("[diag] NO RED — 液滴几何/材质根本没渲染出来", flush=True)
    print("[diag] done. image at /tmp/droplet_red.png", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
