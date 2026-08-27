# -*- coding: utf-8 -*-
"""生成 a3_conductivity.usd —— A3 电导率测量场景（烘平自包含，真实器材）。

基于 lab_clean.usd（lab_001 副本，台面 Cube 顶 0.80，x/y ∈ [−1,1]）：
- 白名单删除 lab_001 自带器材/家具
- 引用 assets/equipment/ 真实器材 + 设 translate/rot（架高按资产 bbox 动态贴台面，
  或按需显式 tz：天平盘顶的称量纸/粉末、平躺的玻璃棒）
- 无内建效果 prim（样品粉是 assets 资产，直接引用；溶解液/读数等由 task 运行时加）

布局（2026-08-27，用户要求器材展开分开放、间隔大；机器人底座写 config = 台面中心
(0,0) 以便器材环形散布、全在 Franka 0.855m 臂展内）：
  机器人底座      (0.00, 0.00)   config robot.position [0.0,0.0,0.71]
  ── 测量区（电导率仪电极线缆锚在机身插座，烧杯须在插座 0.8m 内）──
  Meter          (0.05, 0.35)   不旋转：屏幕朝 +y；电极（+x 侧）朝 +x 面向样品烧杯
  SampleBeaker   (0.72, 0.24)   烧杯111 样品杯；距 meter 插座 ~0.73m（线缆够）
  RinseBeaker    (-0.52, 0.28)  烧杯111 洗杯（电极冲洗）
  ── 称量区（簇，内部间距豁免）──
  Balance        (-0.60,-0.55)  分析天平
  WeighingPaper  称量纸 12cm 放天平称盘顶（盘顶 z=0.8475）
  Powder         样品粉堆 放称量纸上（纸顶 z=0.8485）
  Spatula        (-0.75,-0.40)  药匙放天平左侧
  ── 水/工具区 ──
  WashBottle     (0.50,-0.60)   rotZ −90°（红嘴朝 +Y，对样品杯方向）
  GlassRod       (-0.05,-0.50)  rotX 90° 平躺桌面（沿 y，中心 -0.50）

主要间隔（器材 bbox 净距）：meter–sample 0.35 / meter–rinse 0.37 /
sample–rinse >1.2 / balance–meter >0.8 / balance–wash >0.7 / wash–sample >0.75
全部 ≥0.35（玻璃棒 0.15）；电极插座到烧杯 ≤0.73m（动态线缆 0.8m 内安全）。

用法：python scripts/gen_a3_scene.py   （conda env labutopia 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "a_instrument", "a3_conductivity")
OUT = os.path.join(SCENE_DIR, "a3_conductivity.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")

TABLE_TOP = 0.80
BALANCE_PAN_TOP = 0.8475     # 天平称盘顶 z（资产 bbox 顶 0.047 + 台面 0.80）
# 环境贴图源（d2s 同款；DomeLight 贴图断链会整场发黑）
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")
# lab_clean 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (name, asset, translate, scale, rot_x, rot_z)
#   translate tz=None → 动态贴台面（资产底座 min z -> 0.80）
#   称量纸/粉末显式 tz（叠在天平盘上）；玻璃棒显式 tz（平躺贴台面）
EQUIP = [
    ("Meter",         "conductivity_meter.usd",   (0.05, 0.35, None), None, None, None),
    ("SampleBeaker",  "beaker_111x75x116.usd",    (0.72, 0.24, None), None, None, None),
    ("RinseBeaker",   "beaker_111x75x116.usd",    (-0.52, 0.28, None), None, None, None),
    ("Balance",       "analytical_balance.usd",   (-0.60, -0.55, None), None, None, None),
    ("WeighingPaper", "weighing_paper.usd",       (-0.60, -0.55, BALANCE_PAN_TOP + 0.0005), None, None, None),
    # 粉堆本地底 z=0.0083（堆在原点之上）→ tz 让世界底落在纸顶 0.8485：0.8485 - 0.0083 = 0.8402
    ("Powder",        "powder.usd",               (-0.60, -0.55, 0.8485 - 0.0083), None, None, None),
    ("Spatula",       "spatula.usd",              (-0.72, -0.35, None), None, None, None),
    ("WashBottle",    "wash_bottle.usd",          (0.50, -0.60, None), None, None, -90.0),
    ("GlassRod",      "glass_rod_6x6x261.usd",    (-0.05, -0.3695, TABLE_TOP + 0.003), None, 90.0, None),
]


def remove_lab001_equipment(stage):
    """真正删除 lab_001 自带器材/家具（白名单外全删）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        name = child.GetName()
        if name in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(name)
    print(f"[remove] deleted {len(removed)} lab_clean prims: {removed}")


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot_x=None, rot_z=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot_x is not None or rot_z is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(rot_x or 0, 0, rot_z or 0))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" rotX{rot_x}" if rot_x else "")
          + (f" rotZ{rot_z}" if rot_z is not None else "")
          + (f" scale {scale}" if scale else ""))


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属/不锈钢无环境反射会反黑（d2s 同款）。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


# ---- post-export 修复（d2s/d3l/a2 同款） ----------------------------------------
def strip_dome_lights(st2):
    removed = []
    for p in Usd.PrimRange(st2.GetPseudoRoot()):
        if str(p.GetPath()) != "/World/env_light" and p.IsA(UsdLux.DomeLight):
            removed.append(str(p.GetPath()))
    for path in removed:
        st2.RemovePrim(path)
    if removed:
        print(f"[dome] removed leftover DomeLight: {removed}")
    else:
        print("[dome] no leftover DomeLight")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def brighten_lights(st2):
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    for op in UsdGeom.Xformable(cyl).GetOrderedXformOps():
        if op.GetOpName() != "xformOp:translate":
            continue
        v = op.Get()
        op.Set(Gf.Vec3d(x, v[1], v[2]))
        print(f"[light] CylinderLight translate {tuple(round(c, 3) for c in v)} "
              f"-> {tuple(round(c, 3) for c in (x, v[1], v[2]))}")
        return


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（d2s/d3l 同款）。"""
    rel = prim.GetRelationship("material:binding")
    if not rel:
        return False
    targets = rel.GetTargets()
    if not targets:
        return False
    mat = st2.GetPrimAtPath(targets[0])
    if not mat.IsValid():
        return False
    for c in mat.GetChildren():
        if c.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(c)
        for name, val in recipe.items():
            inp = sh.GetInput(name)
            vt = Sdf.ValueTypeNames.Color3f if name == "diffuseColor" else Sdf.ValueTypeNames.Float
            if not inp:
                inp = sh.CreateInput(name, vt)
            inp.Set(val)
        print(f"[mat] {prim.GetPath()} -> {c.GetPath()} {recipe}")
        return True
    return False


def fix_beaker_glass(st2, beaker_path):
    """烧杯玻璃透明化（d3l/d2s 试管同款）：资产玻璃材质 opacity 1.0 实心，不透可见杯内
    粉末/溶液 → 压到 opacity 0.18、ior 1.5、doubleSided（去曲面强反光带）。"""
    p = st2.GetPrimAtPath(beaker_path)
    if not p.IsValid():
        print(f"[mat] {beaker_path} not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.18, "ior": 1.5, "roughness": 0.05}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] beaker glass {c.GetPath()} -> op 0.18 / ior 1.5 / rough 0.05 / doubleSided")


def post_fix(st2):
    strip_dome_lights(st2)
    fix_env_light(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, -10.0)
    for name in ("SampleBeaker", "RinseBeaker"):
        fix_beaker_glass(st2, f"/World/{name}")


def _wbb(st, bc, path):
    p = st.GetPrimAtPath(path)
    assert p.IsValid(), f"{path} missing"
    r = bc.ComputeWorldBound(p).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def clear_gap(lo1, hi1, lo2, hi2):
    """两个 AABB 的最小净距：重叠轴贡献 0，取各轴分离量的欧氏范数。"""
    s = []
    for i in range(3):
        if hi2[i] < lo1[i]:
            s.append(lo1[i] - hi2[i])
        elif hi1[i] < lo2[i]:
            s.append(lo2[i] - hi1[i])
        else:
            s.append(0.0)
    return (sum(c * c for c in s)) ** 0.5


def verify():
    """场景世界 bbox 校验：器材就位/贴台/关键高度 + 站间净距 ≥0.35（用户要求大间隔）。"""
    st = Usd.Stage.Open(OUT)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    checks = []

    def check(name, cond):
        checks.append((name, cond))
        print(f"[verify] {name}: {'OK' if cond else 'FAIL'}")

    # ---- 就位 / 贴台 / 关键高度（世界 bbox，以左下角为准）----
    lo, hi = _wbb(st, bc, "/World/Meter")
    check("Meter 贴台面 z0.80", abs(lo[2] - 0.80) < 1e-3)
    check("Meter 左下 (-0.098,0.231)", abs(lo[0] + 0.098) < 0.01 and abs(lo[1] - 0.231) < 0.01)
    check("Meter 高 0.232", abs(hi[2] - lo[2] - 0.232) < 0.01)

    lo, hi = _wbb(st, bc, "/World/SampleBeaker")
    check("SampleBeaker 贴台面", abs(lo[2] - 0.80) < 1e-3)
    check("SampleBeaker 左下 (0.664,0.202)", abs(lo[0] - 0.664) < 0.01 and abs(lo[1] - 0.202) < 0.01)
    check("SampleBeaker 顶 0.916", abs(hi[2] - 0.916) < 1e-3)

    lo, hi = _wbb(st, bc, "/World/RinseBeaker")
    check("RinseBeaker 左下 (-0.576,0.242)", abs(lo[0] + 0.576) < 0.01 and abs(lo[1] - 0.242) < 0.01)
    check("RinseBeaker 顶 0.916", abs(hi[2] - 0.916) < 1e-3)

    lo, hi = _wbb(st, bc, "/World/Balance")
    check("Balance 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("Balance 左下 (-0.70,-0.655)", abs(lo[0] + 0.70) < 0.01 and abs(lo[1] + 0.655) < 0.01)

    lo, hi = _wbb(st, bc, "/World/WeighingPaper")
    check("WeighingPaper 贴天平盘顶 0.8475", abs(lo[2] - 0.8475) < 1e-3)
    check("WeighingPaper 左下 (-0.66,-0.61)", abs(lo[0] + 0.66) < 0.01 and abs(lo[1] + 0.61) < 0.01)

    lo, hi = _wbb(st, bc, "/World/Powder")
    check("Powder 底在纸顶 0.8485", abs(lo[2] - 0.8485) < 1e-3)
    check("Powder 左下 (-0.649,-0.595)", abs(lo[0] + 0.6487) < 0.01 and abs(lo[1] + 0.5946) < 0.01)

    lo, hi = _wbb(st, bc, "/World/Spatula")
    check("Spatula 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("Spatula 左下 (-0.731,-0.355)", abs(lo[0] + 0.731) < 0.01 and abs(lo[1] + 0.3548) < 0.01)

    lo, hi = _wbb(st, bc, "/World/WashBottle")
    check("WashBottle 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("WashBottle 左下 (0.468,-0.632)", abs(lo[0] - 0.468) < 0.01 and abs(lo[1] + 0.632) < 0.01)

    lo, hi = _wbb(st, bc, "/World/GlassRod")
    check("GlassRod 平躺贴台面", abs(lo[2] - 0.80) < 0.01)
    check("GlassRod 沿 y 长 0.261", abs(hi[1] - lo[1] - 0.261) < 0.01)

    # ---- 站间净距 ≥0.35（称量簇=天平+称量纸+粉堆+药匙 合并为一个站）----
    station_bbox = {
        "Meter": _wbb(st, bc, "/World/Meter"),
        "SampleBeaker": _wbb(st, bc, "/World/SampleBeaker"),
        "RinseBeaker": _wbb(st, bc, "/World/RinseBeaker"),
        "WashBottle": _wbb(st, bc, "/World/WashBottle"),
        "GlassRod": _wbb(st, bc, "/World/GlassRod"),
    }
    bbs = [_wbb(st, bc, p) for p in ("/World/Balance", "/World/WeighingPaper",
                                     "/World/Powder", "/World/Spatula")]
    cluster = (tuple(min(bb[0][i] for bb in bbs) for i in range(3)),
               tuple(max(bb[1][i] for bb in bbs) for i in range(3)))
    station_bbox["WeighStation"] = cluster

    names = list(station_bbox)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            lo1, hi1 = station_bbox[names[i]]
            lo2, hi2 = station_bbox[names[j]]
            g = clear_gap(lo1, hi1, lo2, hi2)
            check(f"净距 {names[i]}~{names[j]} ≥0.35 ({g:.3f})", g >= 0.35)

    ok = all(passed for _, passed in checks)
    assert os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")), \
        "env_bright.png missing (DomeLight would be black)"
    assert ok, "A3 scene verify FAIL"
    print("[verify] all OK ->", OUT)


def main():
    os.makedirs(SCENE_DIR, exist_ok=True)
    # 环境贴图（DomeLight 断链 = 整场发黑，d2s/a2 同款）
    import shutil
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    remove_lab001_equipment(stage)
    for name, asset, t, scale, rx, rz in EQUIP:
        add_equip(stage, name, asset, t, scale, rx, rz)
    add_env_light(stage)
    stage.Export(OUT)
    print(f"[export] {OUT}")

    st2 = Usd.Stage.Open(OUT)
    post_fix(st2)
    st2.GetRootLayer().Save()
    print("[save] post-fix")

    verify()
    print("DONE")


if __name__ == "__main__":
    main()
