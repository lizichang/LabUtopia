# -*- coding: utf-8 -*-
"""生成 d2s_water_solubility.usd（D2-S 固体样品水溶性测试场景，IK/Lula 重写版，原 lab_004）。

基于 lab_001.usd 副本：
- 白名单删除：lab_001 自带器材/家具（AlcoholLamp/烧杯/锥形瓶/量筒/DryingBox/
  马弗炉/柜子/靶台等）全删，只留结构件 table/Cube/GroundPlane + 灯光 + 材质
- 抬高工作台面（/World/table + /World/Cube）使台面顶面 = z 0.80（flametest 约定，
  与 lab_flametest_v17.usd 一致；lab_001 原顶面 0.78，器材放 0.80 会悬空 2cm）
- 引用 assets/equipment/ 下的 3 个 simple 资产（TestTube/Spoon/SamplePowder），
  坐标按 simple 资产原点设计
  （test_tube_rack / wash_bottle 形状错误，已按用户要求移除——场景与资产都删）
- 内建效果 prim（PowderOnSpoon/TubeSample/TubeWater，初始隐藏；位置按 simple
  资产几何修正，运行时由 task 的 kin-object 动画驱动/覆写）

布局（桌面 z=0.80，工作区 x[0.2,0.37] y[-0.1,0.2]）：
  Spoon        (0.22, 0.13) 平放，手柄沿 x，勺头 +x（头中心 world 0.265）
  SamplePowder (0.26, 0.02)
  TestTube     (0.30, 0.08)（管底 z=0.809，管口 z=0.929；原插在试管架孔中，
               架已移除）

导出用 stage.Export()：把 lab_001 + 引用资产 + 编辑全部烘成单层（自包含，
加载时不依赖 assets/equipment/；equipment/ 的 *_simple.usd 仅作再生成源）。
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "assets", "scenes", "base", "lab_001", "lab_001.usd")
OUT_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility")
ASSET = os.path.join(REPO, "assets", "equipment")
OUT_FILE = os.path.join(OUT_DIR, "d2s_water_solubility.usd")

# 桌面目标顶面 z（flametest 约定，与 lab_flametest_v17.usd 一致）
TABLE_TOP = 0.80

# 白名单：只保留结构件/灯光/材质/物理场景。lab_001 自带的器材与家具
# （AlcoholLamp/烧杯/锥形瓶/量筒/DryingBox/马弗炉/柜子/靶台等）全部删除。
KEEP = {
    "/World/table",
    "/World/Cube",
    "/World/GroundPlane",
    "/World/CylinderLight",
    "/World/PhysicsScene",
    "/World/Looks",
    "/World/Render",
}

# (name, asset_file, translate)  asset_file 取 assets/equipment/ 下的 *_simple.usd
# 注意：test_tube_rack / wash_bottle 形状错误，已按用户要求从场景和资产中移除。
REFS = [
    ("TestTube", "test_tube_simple.usd", (0.30, 0.08, 0.809)),
    ("Spoon", "spoon_simple.usd", (0.22, 0.13, 0.8025)),
    ("SamplePowder", "sample_powder_simple.usd", (0.26, 0.02, 0.80)),
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
# PowderOnSpoon 对齐勺头中心 (0.22+0.045, 0.13, 0.8025+0.004=0.8065)，置于头顶上方
BUILTIN = [
    ("PowderOnSpoon", 0.004, 0.004, (0.265, 0.13, 0.8125), (0.93, 0.93, 0.94), 1.0),
    ("TubeSample", 0.004, 0.008, (0.30, 0.08, 0.817), (0.93, 0.93, 0.94), 1.0),
    ("TubeWater", 0.006, 0.035, (0.30, 0.08, 0.831), (0.55, 0.75, 0.95), 0.6),
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


def remove_lab001_equipment(stage):
    """删除 /World 下所有不在白名单里的 prim（lab_001 自带器材/家具全删）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        if str(child.GetPath()) in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(str(child.GetPath()))
    print(f"[remove] deleted {len(removed)} lab_001 prims: {removed}")


def raise_worktop(stage, target_top=TABLE_TOP):
    """把 /World/Cube（2x2m 台面 slab）顶面抬到 target_top，/World/table 同步抬高。

    lab_001 的台面 = Cube 顶面（0.78），table 是底下木台；统一 +delta 保持相对关系。
    """
    cube = stage.GetPrimAtPath("/World/Cube")
    if not cube.IsValid():
        print("[worktop] /World/Cube not found, skip")
        return
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    top = bc.ComputeWorldBound(cube).ComputeAlignedRange().GetMax()[2]
    delta = target_top - top
    for path in ("/World/Cube", "/World/table"):
        p = stage.GetPrimAtPath(path)
        if not p.IsValid():
            continue
        ops = UsdGeom.Xformable(p).GetOrderedXformOps()
        tr = ops[0].Get()
        ops[0].Set(Gf.Vec3d(tr[0], tr[1], tr[2] + delta))
    print(f"[worktop] surface top {top:.4f} -> {target_top:.2f} (delta {delta:+.4f})")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(ASSET, exist_ok=True)
    stage = Usd.Stage.Open(SRC)
    root = stage.GetPrimAtPath("/World")
    assert root.IsValid(), "/World not found in lab_001"

    raise_worktop(stage)

    # 1. 删除 lab_001 自带器材/家具（白名单外全删）
    remove_lab001_equipment(stage)

    # 2. 引用 D2 资产 + 设 translate（绝对路径先组合，Export 烘进单层）
    for name, asset_file, t in REFS:
        prim_path = f"/World/{name}"
        prim = UsdGeom.Xform.Define(stage, prim_path)
        prim.GetPrim().GetReferences().AddReference(
            os.path.abspath(os.path.join(ASSET, asset_file))
        )
        prim.AddTranslateOp().Set(Gf.Vec3d(*t))
        print(f"referenced {name} at {t}")

    # 3. 内建效果 prim（初始隐藏）
    for name, r, h, t, color, opacity in BUILTIN:
        prim_path = f"/World/{name}"
        geom = UsdGeom.Cylinder.Define(stage, prim_path)
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"builtin {name} hidden at {t}")

    stage.Export(OUT_FILE)
    print("SAVED", OUT_FILE)


if __name__ == "__main__":
    main()
