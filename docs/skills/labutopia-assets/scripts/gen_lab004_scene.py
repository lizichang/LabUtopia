# -*- coding: utf-8 -*-
"""生成 lab_004.usd（D2 蒸馏水溶解性测试场景）。

基于 lab_001.usd 副本：
- 删除焰色反应器材（AlcoholLamp/BunsenBurner/CobaltGlass/HClBottle/PlatinumWire/SampleDish）
- 引用 D2 资产：TestTubeRack/TestTube/Spoon/WashBottle/SamplePowder
- 内建效果 prim：PowderOnSpoon（勺上粉末）/TubeSample（管内粉末）/TubeWater（管内液面）

布局（桌面 z=0.80，工作区 x[0.2,0.37] y[-0.1,0.2]）：
  Spoon        (0.22, 0.13) 平放，手柄沿 x，勺头 +x
  SamplePowder (0.22, 0.02)
  TestTubeRack (0.30, 0.08)，TestTube 插在孔中（管底 z=0.809，管口 z=0.929）
  WashBottle   (0.25, -0.05)
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

SRC = r"E:/浙江大学/星辰计划/LabVLA_第一期轮转/LabUtopia/assets/chemistry_lab/lab_001/lab_001.usd"
OUT_DIR = r"E:/浙江大学/星辰计划/LabVLA_第一期轮转/LabUtopia/assets/chemistry_lab/lab_004"
ASSET = r"E:/浙江大学/星辰计划/LabVLA_第一期轮转/LabUtopia/assets/chemistry_lab"

REMOVE = [
    "/World/AlcoholLamp",
    "/World/BunsenBurner",
    "/World/CobaltGlass",
    "/World/HClBottle",
    "/World/PlatinumWire",
    "/World/SampleDish",
]

# (name, asset_file, translate)
REFS = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.08, 0.80)),
    ("TestTube", "test_tube.usd", (0.30, 0.08, 0.809)),
    ("Spoon", "spoon.usd", (0.22, 0.13, 0.8025)),
    ("WashBottle", "wash_bottle.usd", (0.25, -0.05, 0.80)),
    ("SamplePowder", "sample_powder.usd", (0.26, 0.02, 0.80)),
]

# 内建效果 prim: (name, kind, radius, height, translate, color, opacity)
BUILTIN = [
    ("PowderOnSpoon", "cylinder", 0.004, 0.004, (0.305, 0.13, 0.80), (0.93, 0.93, 0.94), 1.0),
    ("TubeSample", "cylinder", 0.004, 0.008, (0.30, 0.08, 0.817), (0.93, 0.93, 0.94), 1.0),
    ("TubeWater", "cylinder", 0.006, 0.035, (0.30, 0.08, 0.831), (0.55, 0.75, 0.95), 0.6),
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

    # 2. 引用 D2 资产 + 设 translate
    for name, asset_file, t in REFS:
        prim_path = f"/World/{name}"
        prim = UsdGeom.Xform.Define(stage, prim_path)
        prim.GetPrim().GetReferences().AddReference(
            os.path.join(ASSET, asset_file)
        )
        ops = prim.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(*t))
        else:
            prim.AddTranslateOp().Set(Gf.Vec3d(*t))
        print(f"referenced {name} at {t}")

    # 3. 内建效果 prim（初始隐藏）
    for name, kind, r, h, t, color, opacity in BUILTIN:
        prim_path = f"/World/{name}"
        geom = UsdGeom.Cylinder.Define(stage, prim_path)
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"builtin {name} hidden at {t}")

    out_file = os.path.join(OUT_DIR, "lab_004.usd")
    stage.Export(out_file)
    print("SAVED", out_file)


if __name__ == "__main__":
    main()
