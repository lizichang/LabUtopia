# -*- coding: utf-8 -*-
"""生成 lab_005.usd（胶头滴管滴加测试场景）。

基于 lab_001.usd 副本：
- 删除焰色反应器材（AlcoholLamp/BunsenBurner/CobaltGlass/PlatinumWire/SampleDish）
- 保留 HClBottle（盐酸瓶 = 吸液源，瓶口 z≈0.879）
- 引用 D3 资产：Dropper（立放）/TestTubeRack/TestTube（滴加目标）
- 内建效果 prim：BottleLiquid（瓶内液面，可见）/TubeDrops（试管内液滴，初始隐藏，
  task 检测到 dropped 后显示）

布局（桌面 z=0.80，工作区 x[0.2,0.37] y[-0.1,0.2]）：
  Dropper      (0.36, 0.16) 立放（尖嘴底=资产原点 z=0，管身 0-0.12，胶头 0.115-0.15）
  HClBottle    (0.22, -0.10) lab_001 原有，瓶口 z≈0.879
  TestTubeRack (0.30, 0.08)，TestTube 插在孔中（管底 z=0.809，管口 z=0.929）

引用路径用相对路径（../xxx.usd），按 USD 规范相对场景文件目录解析，跨机器可用。
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

SRC = r"E:/浙江大学/星辰计划/LabVLA_第一期轮转/LabUtopia/assets/chemistry_lab/lab_001/lab_001.usd"
OUT_DIR = r"E:/浙江大学/星辰计划/LabVLA_第一期轮转/LabUtopia/assets/chemistry_lab/lab_005"

REMOVE = [
    "/World/AlcoholLamp",
    "/World/BunsenBurner",
    "/World/CobaltGlass",
    "/World/PlatinumWire",
    "/World/SampleDish",
]

# (name, asset_file(相对 lab_005/ 目录), translate)
REFS = [
    ("Dropper", "../dropper.usd", (0.36, 0.16, 0.80)),
    ("TestTubeRack", "../test_tube_rack.usd", (0.30, 0.08, 0.80)),
    ("TestTube", "../test_tube.usd", (0.30, 0.08, 0.809)),
]

# 内建效果 prim: (name, kind, radius, height, translate, color, opacity, visible)
BUILTIN = [
    # 盐酸瓶内液面（瓶顶 z=0.879，液面在其下 7mm）
    ("BottleLiquid", "cylinder", 0.012, 0.012, (0.22, -0.10, 0.872), (0.55, 0.75, 0.95), 0.6, True),
    # 试管内液滴（管底 z=0.809，管口 z=0.929），task 检测到 dropped 后显示
    ("TubeDrops", "cylinder", 0.005, 0.015, (0.30, 0.08, 0.825), (0.55, 0.75, 0.95), 0.6, False),
]


def add_material(stage, prim, diffuse, opacity):
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    stage = Usd.Stage.Open(SRC)
    root = stage.GetPrimAtPath("/World")
    assert root.IsValid(), "/World not found in lab_001"

    # 1. 删除焰色反应器材
    for path in REMOVE:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(Sdf.Path(path))
            print(f"removed {path}")
        else:
            print(f"skip (absent) {path}")

    # 2. 引用 D3 资产 + 设 translate（相对路径，跨机器可解析）
    for name, asset_file, t in REFS:
        prim_path = f"/World/{name}"
        prim = UsdGeom.Xform.Define(stage, prim_path)
        prim.GetPrim().GetReferences().AddReference(asset_file)
        ops = prim.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(*t))
        else:
            prim.AddTranslateOp().Set(Gf.Vec3d(*t))
        print(f"referenced {name} -> {asset_file} at {t}")

    # 3. 内建效果 prim（BottleLiquid 可见，TubeDrops 初始隐藏）
    for name, kind, r, h, t, color, opacity, visible in BUILTIN:
        prim_path = f"/World/{name}"
        geom = UsdGeom.Cylinder.Define(stage, prim_path)
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        if not visible:
            UsdGeom.Imageable(geom).MakeInvisible()
        print(f"builtin {name} at {t} visible={visible}")

    out_file = os.path.join(OUT_DIR, "lab_005.usd")
    stage.Export(out_file)
    print("SAVED", out_file)


if __name__ == "__main__":
    main()
