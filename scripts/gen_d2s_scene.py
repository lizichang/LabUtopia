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
  TestTube      (0.2787, 0.1193)  前排左孔（Ø19.2×153mm，尺寸固化在 equipment）
  Spatula       (0.3174, 0.1193)  前排右孔，竖插
  SurfaceDish   (0.30, -0.32)  架正后方，表面皿（粉末在皿上，舀取时药匙水平插入）
  SamplePowder  (0.30, -0.32)  表面皿上（powder.usd scale 0.4，离群废料/env_light 由 cleanup 删）
  WashBottle    (0.52,  0.06)  操作台右侧

烘平后处理（单层里已是真实 prim）：
  - 删试管架自带 4 根挤在中心原点的细杆（模型缺陷，孔位由真实试管/药匙占用）
  - 表面皿去 env_light（flametest 残留光）+ 粉末子集重绑皿材质
  - 粉末（powder.usd）删离群废料 Object_0/Object_2 + env_light，纹理重定位到 equipment/textures

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
# 试管/药匙 xy 对齐试管架真实孔位（顶层板 14 孔，2列x7行；最前排 y≈0.119）：
#   左孔 (0.2790,0.1161)  <- 试管，右孔 (0.3170,0.1166) <- 药匙
# 试管 Ø19.2×153mm 已固化进 equipment/test_tube.usd（原 Ø15 放大 1.2779），勿再场景放大
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None),
    ("TestTube", "test_tube.usd", (0.2787, 0.1193, 0.806), None),
    ("Spatula", "spatula.usd", (0.3174, 0.1193, 0.828), None),
    ("SurfaceDish", "sample_dish.usd", (0.30, -0.32, 0.80), None),
    ("SamplePowder", "powder.usd", (0.3018, -0.3258, 0.7988), 0.4),
    ("WashBottle", "wash_bottle.usd", (0.52, 0.06, 0.80), None),
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
# PowderOnSpoon 在药匙尖端（spatula tip world z=0.828+0.135=0.963）
# TubeSample/TubeWater 在放大后试管内（xy=试管孔位）
BUILTIN = [
    ("PowderOnSpoon", 0.005, 0.005, (0.3174, 0.1193, 0.965), (0.93, 0.93, 0.94), 1.0),
    ("TubeSample", 0.006, 0.012, (0.2787, 0.1193, 0.84), (0.93, 0.93, 0.94), 1.0),
    ("TubeWater", 0.007, 0.035, (0.2787, 0.1193, 0.855), (0.55, 0.75, 0.95), 0.6),
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

    for path in sorted(set(to_remove)):
        stage.RemovePrim(path)
    print(f"[cleanup] removed {len(set(to_remove))}: {sorted(set(to_remove))}")


def cleanup_dish(stage):
    """表面皿：去 flametest 残留 env_light，粉末子集重绑皿材质（先不放粉末）。

    sample_dish.usd 是双材质皿：单 mesh 带 powder_mat / dish_mat 两个 GeomSubset
    （粉丘是 mesh 的一部分，flametest 残留）。真实粉末由 SamplePowder（powder.usd）
    摆在皿上，故把资产自带 powder 子集重绑到 dish 材质避免双份粉丘。
    """
    dish = stage.GetPrimAtPath("/World/SurfaceDish")
    if not dish.IsValid():
        print("[dish] not found, skip")
        return
    for child in list(dish.GetChildren()):
        if child.GetTypeName() == "DomeLight" or "env_light" in child.GetName():
            stage.RemovePrim(child.GetPath())
            print(f"[dish] removed {child.GetPath()}")
    dish_mat = stage.GetPrimAtPath("/World/SurfaceDish/_materials/dish_mat_002_002")
    if not dish_mat.IsValid():
        print("[dish] dish material not found, skip rebind")
        return

    def walk(prim):
        for c in prim.GetChildren():
            if c.GetTypeName() == "GeomSubset" and c.GetName().startswith("powder"):
                UsdShade.MaterialBindingAPI.Apply(c).Bind(UsdShade.Material(dish_mat))
                print(f"[dish] rebound {c.GetPath()} -> dish material")
            walk(c)

    walk(dish)


def powder(stage):
    """粉末收尾：离群废料/ env_light 已从 powder.usd 资产本体删掉（Object_1 即粉堆），
    这里仅保留防御性清理 + 纹理路径重定位（烘平若产出相对 ./textures 会失效）。"""
    pw = stage.GetPrimAtPath("/World/SamplePowder")
    if not pw.IsValid():
        print("[powder] not found, skip")
        return
    to_rm = []

    def collect(p):
        for c in p.GetChildren():
            if c.GetName() in ("Object_0", "Object_2", "env_light"):
                to_rm.append(str(c.GetPath()))
            collect(c)

    collect(pw)
    for path in sorted(set(to_rm)):
        stage.RemovePrim(path)
        print(f"[powder] removed {path}")

    # 纹理重定位：./textures/x -> <scene> 相对 equipment/textures 的路径
    scene_dir = os.path.dirname(OUT)
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Shader":
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path and v.path.replace("\\", "/").startswith("./textures/"):
                base = os.path.basename(v.path.replace("\\", "/"))
                newp = os.path.relpath(os.path.join(EQ, "textures", base), scene_dir).replace("\\", "/")
                inp.Set(Sdf.AssetPath(newp))
                print(f"[powder] texture {base} -> {newp}")


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
    cleanup_flattened(st2)   # 删架子自带细杆
    cleanup_dish(st2)        # 表面皿去 env_light + 粉末子集重绑皿材质
    powder(st2)              # 粉末：防御性清理 + 纹理重定位（资产本体已删废料/env_light）
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
