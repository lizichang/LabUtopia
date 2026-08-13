# -*- coding: utf-8 -*-
"""生成 d2s_water_solubility.usd —— D2-S 固体样品水溶性测试场景（烘平自包含，真实器材）。

基于 lab_001.usd 副本（Usd.Stage.Open 编辑 + stage.Export 烘平，不写回 lab_001）：
- 白名单删除 lab_001 自带器材/家具（含 Cabinet 离线 http payload，一并消除加载警告）
- 抬高工作台面 table+Cube，顶面 -> 0.80（flametest 约定）
- 引用 assets/equipment/ 真实器材 + 设 translate（架高按资产 bbox 动态贴台面）
- 内建效果 prim（PowderOnSpoon/TubeSample/TubeWater，初始隐藏，task 动画驱动）

导出 stage.Export()：单层自包含，无引用弧，加载不依赖 assets/equipment/，
默认根带 lab_001 的 defaultPrim=/World（Isaac Sim 空视口问题用引用式+无 default
prim 会出现，烘平后不存在）。

布局（v11：试管架中央/样品瓶后方/洗瓶右侧/药匙前方；台面顶 z=0.80）：
  TestTubeRack  (0.30,  0.00)  中央，底座落台面
  TestTube      (0.30,  0.06)  插在架上（真实 10mL）
  Spatula       (0.30, -0.06)  竖插在架上（用户要求）
  SampleBottle  (0.30, -0.32)  架正后方 ~18cm，初始开盖，粉末在瓶口
  WashBottle    (0.52,  0.06)  操作台右侧
  SamplePowder  (0.30, -0.32)  瓶口处（scale 0.5）

烘平后清理（单层里已是真实 prim，直接 RemovePrim）：
  - 试管架自带 4 根挤在中心原点的细杆（模型缺陷），换成真实试管/药匙站位
  - 样品瓶 stopper（v11 先舀取、步骤4才归位盖紧 -> 初始开盖）

用法：python scripts/gen_d2s_scene.py   （运行环境：本地 conda env 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility")
OUT = os.path.join(SCENE_DIR, "d2s_water_solubility.usd")
LAB001 = os.path.join(REPO, "assets", "scenes", "base", "lab_001", "lab_001.usd")
EQ = os.path.join(REPO, "assets", "equipment")

TABLE_TOP = 0.80
# lab_001 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (prim, asset_file, translate, scale)   tz=None 表示动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None),
    ("TestTube", "test_tube.usd", (0.30, 0.06, 0.806), None),
    ("Spatula", "spatula.usd", (0.30, -0.06, 0.828), None),
    ("SampleBottle", "sample_bottle.usd", (0.30, -0.32, 0.80), None),
    ("WashBottle", "wash_bottle.usd", (0.52, 0.06, 0.80), None),
    ("SamplePowder", "sample_powder.usd", (0.30, -0.32, 0.86), 0.5),
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
BUILTIN = [
    ("PowderOnSpoon", 0.004, 0.004, (0.30, -0.06, 0.965), (0.93, 0.93, 0.94), 1.0),
    ("TubeSample", 0.005, 0.012, (0.30, 0.06, 0.84), (0.93, 0.93, 0.94), 1.0),
    ("TubeWater", 0.006, 0.030, (0.30, 0.06, 0.855), (0.55, 0.75, 0.95), 0.6),
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
    """真正删除 lab_001 自带器材/家具（白名单外全删，含 Cabinet 离线 payload）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        name = child.GetName()
        if name in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(name)
    print(f"[remove] deleted {len(removed)} lab_001 prims: {removed}")


def raise_worktop(stage, target_top=TABLE_TOP):
    """Cube/table 顶面抬到 target_top（编辑 lab_001 内存副本，Export 后不写回）。"""
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


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (f" scale {scale}" if scale else ""))


def add_effects(stage):
    for name, r, h, t, color, opacity in BUILTIN:
        geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} hidden at {t}")


def cleanup_flattened(stage):
    """烘平后单层里已是真实 prim：删架子细杆 + 样品瓶瓶塞。

    先遍历收集路径（期间不删，避免 prim 失效），再统一 RemovePrim。
    """
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    to_remove = []

    def collect(prim):
        for child in prim.GetChildren():
            if child.GetTypeName() == "Mesh":
                try:
                    r = bc.ComputeWorldBound(child).ComputeAlignedRange()
                    sz = r.GetMax() - r.GetMin()
                except Exception:
                    continue
                if sz[2] > 3.0 * max(sz[0], sz[1]):  # 细杆
                    parent = child.GetParent()
                    if parent.IsValid() and parent.IsActive():
                        to_remove.append(str(parent.GetPath()))
            collect(child)

    rack = stage.GetPrimAtPath("/World/TestTubeRack")
    if rack.IsValid():
        collect(rack)
    stopper = stage.GetPrimAtPath("/World/SampleBottle/stopper")
    if stopper.IsValid() and stopper.IsActive():
        to_remove.append(str(stopper.GetPath()))

    for path in sorted(set(to_remove)):
        stage.RemovePrim(path)
    print(f"[cleanup] removed {len(set(to_remove))}: {sorted(set(to_remove))}")


def main():
    os.makedirs(SCENE_DIR, exist_ok=True)
    stage = Usd.Stage.Open(LAB001)
    raise_worktop(stage)
    remove_lab001_equipment(stage)
    for name, asset, t, scale in EQUIP:
        add_equip(stage, name, asset, t, scale)
    add_effects(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_001 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    cleanup_flattened(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
