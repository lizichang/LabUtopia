# -*- coding: utf-8 -*-
"""生成 a3_conductivity.usd —— A3 电导率测量场景（烘平自包含，真实器材）。

基于 lab_clean.usd（lab_001 副本，台面 Cube 顶 0.80，x/y ∈ [−1,1]）：
- 白名单删除 lab_001 自带器材/家具
- 引用 assets/equipment/ 真实器材 + 设 translate/rot（架高按资产 bbox 动态贴台面，
  或按需显式 tz：天平盘顶的称量纸/粉末、平躺的玻璃棒）
- 无内建效果 prim（样品粉是 assets 资产，直接引用；溶解液/读数等由 task 运行时加）

布局（2026-08-27 二改 = 用户 Isaac 重摆后 scene-realign，tmp=a3_conductivity_tmp.usd 为真相，
全部器材围绕机械臂底座 (0.37,0.16) 环形摆放；config robot.position = [0.37,0.16,0.71]）：
  机器人底座      (0.37, 0.16)    config robot.position [0.37,0.16,0.71]（用户指定）
  ── 测量区 ──
  Meter          (0.3982,-0.1054) 不旋转：屏幕朝 +y；电极（+x 侧）朝 +x
  SampleBeaker   (0.6469,0.0880,0.8857) rotX(-135) 烧杯111 样品杯——**完全复刻 tmp**：
                                 T(0.6469,0.0880,0.8857)+rotateXYZ(-135,0,0)，bbox
                                 0.609..0.685 / 0.049..0.209 / 0.765..0.925（与 tmp 逐位一致）
  ── 称量区（简化：无药匙/称量纸；表面皿+粉直接叠天平盘）──
  Balance        (0.3442, 0.5550) 分析天平
  SurfaceDish    表面皿 Ø60 放天平盘顶（盘顶 z=0.8475；皿底 0.8474 顶 0.8540）
  SamplePowder   粉堆 scale 0.25 落皿内（0.022×0.030×0.0075 半大小，贴皿内不溢出）
  ── 水/工具区 ──
  WashBottle     (0.6400, 0.3600) 洗瓶 rotZ180（tmp 里被翻转：红嘴朝 +X，对试管架方向）
  TestTubeRack   (0.8537, 0.1763) 试管架（玻璃棒插在其中）
  GlassRod       (0.8341, 0.1769) 玻璃棒 Ø6×261 立架内（底贴台面 0.80，顶 1.061
                                 高出架顶 0.917 上 0.144 供抓取）

站间 bbox 净距（用户重摆后紧凑）：最紧 Meter~SampleBeaker ~0.03m、WashBottle~Rack ~0.06m；
最远抓点 玻璃棒顶 (0.834,0.177,1.061) 距底座 (0.37,0.16) 3D 0.58m ≤ 0.855m 臂展（用户
中央底座布局，全器材围绕，均达记）。电极插座到烧杯 ~0.32m（动态线缆 0.8m 内安全）。

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
# 表面皿(Ø60)叠天平盘顶：皿本地底 z=0.0001 → 皿底 0.8474、皿顶 0.8540
DISH_TZ = BALANCE_PAN_TOP - 0.0001
DISH_TOP = DISH_TZ + 0.0066
# 粉堆 scale 0.25 落皿内（2026-08-28 用户：粉末缩小一半）：本地底 z=0.0083*0.25=0.002075 → 粉底贴皿顶
POWDER_SCALE = 0.25
POWDER_TZ = DISH_TOP - 0.0083 * POWDER_SCALE
# 环境贴图源（d2s 同款；DomeLight 贴图断链会整场发黑）
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")
# lab_clean 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (name, asset, translate, scale, rot_x, rot_z)
#   translate tz=None → 动态贴台面（资产底座 min z -> 0.80）
#   表面皿/粉堆显式 tz（叠天平盘）；玻璃棒显式 tz（立架内，底贴台面）
EQUIP = [
    ("Meter",         "conductivity_meter.usd",   (0.3982, -0.1054, None), None, None, None),
    ("SampleBeaker",  "beaker_111x75x116.usd",    (0.6469,  0.0880, 0.8857), None, -135, None),
    ("Balance",       "analytical_balance.usd",   (0.3442,  0.5550, None), None, None, None),
    ("SurfaceDish",   "sample_dish.usd",          (0.3442,  0.5550, DISH_TZ), None, None, None),
    ("SamplePowder",  "powder.usd",               (0.3442,  0.5550, POWDER_TZ), POWDER_SCALE, None, None),
    ("WashBottle",    "wash_bottle.usd",          (0.6400,  0.3600, None), None, None, 180),
    ("TestTubeRack",  "test_tube_rack.usd",       (0.8537,  0.1763, None), None, None, None),
    ("GlassRod",      "glass_rod_6x6x261.usd",    (0.8341,  0.1769, TABLE_TOP), None, None, None),
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
    for name in ("SampleBeaker",):
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
    check("Meter 左下 (0.250,-0.224)", abs(lo[0] - 0.250) < 0.01 and abs(lo[1] + 0.224) < 0.01)
    check("Meter 高 0.232", abs(hi[2] - lo[2] - 0.232) < 0.01)

    lo, hi = _wbb(st, bc, "/World/SampleBeaker")
    # tmp 原样：T(0.6469,0.0880,0.8857)+rotateXYZ(-135,0,0)，bbox 与 tmp 逐位一致
    check("SampleBeaker 左下 (0.609,0.049,0.765)", abs(lo[0] - 0.609) < 0.01
          and abs(lo[1] - 0.049) < 0.01 and abs(lo[2] - 0.765) < 0.01)
    check("SampleBeaker 右上 (0.685,0.209,0.925)", abs(hi[0] - 0.685) < 0.01
          and abs(hi[1] - 0.209) < 0.01 and abs(hi[2] - 0.925) < 0.01)

    lo, hi = _wbb(st, bc, "/World/Balance")
    check("Balance 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("Balance 左下 (0.244,0.450)", abs(lo[0] - 0.244) < 0.01 and abs(lo[1] - 0.450) < 0.01)

    lo, hi = _wbb(st, bc, "/World/SurfaceDish")
    check("SurfaceDish 贴天平盘顶 0.8475", abs(lo[2] - 0.8475) < 1e-3)
    check("SurfaceDish 左下 (0.314,0.525)", abs(lo[0] - 0.314) < 0.01 and abs(lo[1] - 0.525) < 0.01)

    lo, hi = _wbb(st, bc, "/World/SamplePowder")
    check("SamplePowder 底贴皿顶 0.854", abs(lo[2] - 0.854) < 1e-3)
    check("SamplePowder 左下 (0.332,0.544)", abs(lo[0] - 0.332) < 0.01 and abs(lo[1] - 0.544) < 0.01)
    check("SamplePowder 顶 0.8615 (高 0.0075 半大小)", abs(hi[2] - 0.8615) < 1e-3)

    lo, hi = _wbb(st, bc, "/World/WashBottle")
    check("WashBottle 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("WashBottle 左下 (0.608,0.328)", abs(lo[0] - 0.608) < 0.01 and abs(lo[1] - 0.328) < 0.01)

    lo, hi = _wbb(st, bc, "/World/TestTubeRack")
    check("TestTubeRack 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("TestTubeRack 左下 (0.811,0.034)", abs(lo[0] - 0.811) < 0.01 and abs(lo[1] - 0.034) < 0.01)

    lo, hi = _wbb(st, bc, "/World/GlassRod")
    check("GlassRod 立架内 底贴台面", abs(lo[2] - 0.80) < 0.01)
    check("GlassRod 顶 1.061 (高 0.261)", abs(hi[2] - 1.061) < 0.01)

    # ---- 站间净距 ≥0.02（2026-08-27 用户中央底座重摆后紧凑：Meter~Beaker ~0.03、
    #      WashBottle~Rack ~0.06 为最紧对；阈值只拦真实重叠，不拦用户有意紧凑摆放）----
    station_bbox = {
        "Meter": _wbb(st, bc, "/World/Meter"),
        "SampleBeaker": _wbb(st, bc, "/World/SampleBeaker"),
        "WashBottle": _wbb(st, bc, "/World/WashBottle"),
    }
    bbs = [_wbb(st, bc, p) for p in ("/World/Balance", "/World/SurfaceDish", "/World/SamplePowder")]
    station_bbox["WeighStation"] = (tuple(min(bb[0][i] for bb in bbs) for i in range(3)),
                                    tuple(max(bb[1][i] for bb in bbs) for i in range(3)))
    bbs = [_wbb(st, bc, p) for p in ("/World/TestTubeRack", "/World/GlassRod")]
    station_bbox["RackStation"] = (tuple(min(bb[0][i] for bb in bbs) for i in range(3)),
                                   tuple(max(bb[1][i] for bb in bbs) for i in range(3)))

    names = list(station_bbox)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            lo1, hi1 = station_bbox[names[i]]
            lo2, hi2 = station_bbox[names[j]]
            g = clear_gap(lo1, hi1, lo2, hi2)
            check(f"净距 {names[i]}~{names[j]} ≥0.02 ({g:.3f})", g >= 0.02)

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
