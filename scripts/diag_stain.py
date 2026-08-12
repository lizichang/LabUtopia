"""诊断脚本：强制显示 P9 受染状态（蓝焰 + 铂丝尖端黄色染色锥），验证渲染可见性。

背景：main.py --snapshot 只抓帧 0..N（episode 起始），P9 受染在任务后期无法用
快照验证。本脚本复用 main.py 的启动流程，但在 reset 后直接强制：
  1. 火焰 visible（蓝焰）
  2. 染色锥 visible + 定位到铂丝尖端（火焰内 0.918）
然后抓几帧相机图到 run_dir/stain_diag/，用于像素分析。

用法：conda activate labutopia; python scripts/diag_stain.py
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

import cv2
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
    os.makedirs(cfg.multi_run.run_dir, exist_ok=True)

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    task.reset()

    # ---- 强制 P9 受染状态 ----
    task.flame_on = True
    task.powder_dipped = True
    task.stain_on = True
    from pxr import Usd, Sdf, UsdGeom, UsdShade, Gf

    snapshot_dir = os.path.join(cfg.multi_run.run_dir, "stain_diag")
    os.makedirs(snapshot_dir, exist_ok=True)

    # === 第一帧：材质矩阵测试（关火）===
    # 已确证：dish_visible_disk_mat（emissive 0.15,0.55,1.6，R 低 B 高）在 runtime Cone
    # 上渲染饱和蓝；而黄色 emissive（1.8,1.296,0.216）怎么都渲染成奶油白。
    # 本帧放一排 runtime Cone 做对照，找出能渲染的黄色配方：
    #   A: 文件锥 /World/flame_stain_yellow（当前黄材，参考）
    #   T1: dish 蓝材（对照，已知能渲染）
    #   T2: 黄 R 主导 (1.6,0.55,0.15)  —— 结构同 dish（单通道高）
    #   T3: 黄 全通道<=1 (0.9,0.65,0.1)
    task._set_flame_visible(False)
    cone_path = f"{task.STAIN_ROOT}/flame_stain_{task.flame_color}"
    cone_prim = task.stage.GetPrimAtPath(cone_path)
    task._set_stain(True)
    if cone_prim.IsValid():
        UsdGeom.Cone(cone_prim).GetHeightAttr().Set(0.30)
        UsdGeom.Cone(cone_prim).GetRadiusAttr().Set(0.15)
    task._position_stain_at_tip(np.array([0.35, 0.02, 0.9]))

    def _make_test_cone(path, x, emissive):
        cone = task.stage.DefinePrim(path, "Cone")
        xf = UsdGeom.Xformable(cone)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x, 0.02, 0.9))
        UsdGeom.Cone(cone).GetHeightAttr().Set(0.30)
        UsdGeom.Cone(cone).GetRadiusAttr().Set(0.15)
        mat_path = path + "_mat"
        if not task.stage.GetPrimAtPath(mat_path).IsValid():
            mat = UsdShade.Material.Define(task.stage, mat_path)
            sh = UsdShade.Shader.Define(task.stage, mat_path + "/Principled_BSDF")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.01, 0.01, 0.01))
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cone).Bind(
            UsdShade.Material(task.stage.GetPrimAtPath(mat_path)))

    # 验证文件锥 A（v27 新黄 1.6,0.55,0.15）单独在桌面角落渲染是否饱和黄。
    cone_prim = task.stage.GetPrimAtPath(cone_path)
    task._set_stain(True)
    if cone_prim.IsValid():
        UsdGeom.Imageable(cone_prim).MakeVisible()
        UsdGeom.Cone(cone_prim).GetHeightAttr().Set(0.30)
        UsdGeom.Cone(cone_prim).GetRadiusAttr().Set(0.15)
    task._position_stain_at_tip(np.array([0.05, 0.02, 0.9]))
    print("[diag] frame1: flame OFF; file cone A (v27 yellow 1.6,0.55,0.15) enlarged @[0.05,0.02,0.9]", flush=True)

    # 空跑几帧让相机/物理初始化，再抓图
    for _ in range(8):
        world.step(render=True)

    state = task.get_basic_state_info(object_path=task.wire_path)
    for cam_name, image_data in state['camera_display'].items():
        img = cv2.cvtColor(image_data.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
        safe = cam_name.replace('/', '_')
        cv2.imwrite(os.path.join(snapshot_dir, f"stain_{safe}.png"), img)
        print(f"[diag] saved stain_{safe}.png")

    # === 第二帧：P9 真实尺寸测试 ===
    # 火重新打开，锥体恢复真实尺寸，放到铂丝尖端（火焰内），验证 P9 局部焰色。
    task._set_flame_visible(True)
    if cone_prim.IsValid():
        UsdGeom.Cone(cone_prim).GetHeightAttr().Set(0.066)
        UsdGeom.Cone(cone_prim).GetRadiusAttr().Set(0.0264)
    task._position_stain_at_tip(np.array([0.5132, 0.5256, 0.918]))
    print("[diag] frame2: flame ON, cone real size, at wire tip in-flame [0.5132,0.5256,0.918]", flush=True)

    # 打印染色锥实际状态（每一步独立 try，避免一处崩掉整段）
    sys.stdout.flush()
    prim = task.stage.GetPrimAtPath(cone_path)
    if prim.IsValid():
        try:
            vis = UsdGeom.Imageable(prim).ComputeVisibility(Usd.TimeCode.Default())
            print(f"[diag] stain prim {prim.GetPath()} visibility={vis}", flush=True)
        except Exception as e:
            print(f"[diag] visibility err: {e}", flush=True)
        try:
            xf = UsdGeom.Xformable(prim)
            wm = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            print(f"[diag] stain world T={tuple(wm.ExtractTranslation())}", flush=True)
        except Exception as e:
            print(f"[diag] xform err: {e}", flush=True)
        try:
            mat = task.stage.GetPrimAtPath("/World/flame_stain_yellow_mat")
            if mat.IsValid():
                for sh in mat.GetChildren():
                    for attr in sh.GetAttributes():
                        a = attr.GetName()
                        if a.startswith("inputs:"):
                            try:
                                print(f"[diag]     {a} = {attr.Get()}", flush=True)
                            except Exception as e:
                                print(f"[diag]     {a} = ERR {e}", flush=True)
            else:
                print("[diag]   mat MISSING", flush=True)
        except Exception as e:
            print(f"[diag] mat err: {e}", flush=True)
    else:
        print(f"[diag] stain prim {cone_path} MISSING", flush=True)

    for _ in range(4):
        world.step(render=True)
    state = task.get_basic_state_info(object_path=task.wire_path)
    for cam_name, image_data in state['camera_display'].items():
        img = cv2.cvtColor(image_data.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
        safe = cam_name.replace('/', '_')
        cv2.imwrite(os.path.join(snapshot_dir, f"stain_{safe}_2.png"), img)
        print(f"[diag] saved stain_{safe}_2.png")

    print(f"[diag] done, frames in {snapshot_dir}")
    simulation_app.close()


if __name__ == "__main__":
    main()
