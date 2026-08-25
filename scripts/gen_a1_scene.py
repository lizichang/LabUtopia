# -*- coding: utf-8 -*-
"""生成 a1_refractometer.usd —— A1 折光率测量（折光仪）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材：阿贝折光仪（abbemat_advanced，**棱镜盖保留**——资产已内置
  -50° 掀开态，直接滴样不用掀盖、末步「合盖」由 task 驱动；屏幕朝前）、待测液体样品瓶
  （sample_bottle，**保留瓶塞**——取瓶/归瓶步需机械臂拔/盖）、试管架 + 胶头滴管（滴管插架左孔）。
- 布局（2026-08-25 重排：A1 简化去掉「掀盖」+「移到操作位」两步，各器件拉开间距、离机械臂
  底座 ≥0.27m；机械臂底座 (0.25,0.57,0.71) 在后 +Y、前 = -Y、臂展 0.855m、底座 y 失效区 <0.15m）：
      折光仪 (0.30,0.00) 中央（棱镜朝顶、屏幕朝 -y=前方）
      样品瓶 (0.10,0.34) 折光仪左侧（瓶身 Ø36、口 rim 0.870、白塞 0.868..0.879，距底座 0.27m）
      试管架 (0.55,0.20) 右侧；滴管插左孔 (0.531,0.1996,0.806)（距底座 0.47m）
- 去资产自带 env_light 残留（重复 DomeLight）；CylinderLight 2000→12000；样品瓶玻璃透明化
  （bottle 真玻璃，**stopper 保持白盖不透明**）；滴管玻璃透明化（管内液柱透出）。
- 内建效果 prim（task 动画驱动）：瓶内液面 SampleLiquid（可见）、棱镜液滴 PrismDrop（隐藏）、
  滴管尖液柱 DropperFill（隐藏）、挤胶头滴落球 DropperDrop（隐藏）、屏幕读数发光 ScreenGlow（隐藏）。

用法：python scripts/gen_a1_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "a_instrument", "a1_refractometer")
OUT = os.path.join(SCENE_DIR, "a1_refractometer.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 布局锚点（世界坐标，米，Z-up）——
# 机械臂底座 (0.25,0.57,0.71) 在后 +Y，向前 -Y 够工作区；臂展 0.855m，底座 y 失效区 <0.15m。
# 折光仪：asset min z=0（机身底/脚贴原点）→ tz=None 贴台面 0.80。棱镜朝顶、盖已内置 -50° 掀开态、屏幕朝 -y。
#   机身 body  x ±0.1125 y -0.165..+0.165 z 0..0.115
#   棱镜 prism 局部 (0,0.110,0.1165) 顶 0.1175 → 世界 (0.30,0.11,0.9175)
REFRACT_X, REFRACT_Y = 0.30, 0.00
PRISM_CY = REFRACT_Y + 0.110          # 棱镜世界 y 中心（滴样落点，距底座 0.46m）

# 样品瓶：asset min z=0 → 贴台面。口 rim 0.870，白塞 0.868..0.879（保留，机械臂拔/盖）。
# 距底座 0.27m（>0.15 失效区），折光仪左侧留 7cm 间隙。
BOTTLE_X, BOTTLE_Y = 0.10, 0.34

# 试管架（放滴管）：asset min z=-0.0965 → 架原点 z=0.8965。孔心列 x=±0.019 中排 y=-0.0004，
# 底层板顶(孔底) z=-0.0905 → 世界 0.806。滴管插左孔。距底座 0.47m，折光仪右侧留 9.5cm 间隙。
RACK_X, RACK_Y = 0.55, 0.20
RACK_Z = TABLE_TOP + 0.0965          # 0.8965 架原点
HOLE_Z = RACK_Z - 0.0905             # 0.806 孔底
DROPPER_XY = (RACK_X - 0.019, RACK_Y - 0.0004)   # (0.531, 0.1996)

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("Refractometer", "abbemat_advanced.usd", (REFRACT_X, REFRACT_Y, None), None, False),
    ("SampleBottle", "sample_bottle.usd", (BOTTLE_X, BOTTLE_Y, None), None, False),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, False),
    ("Dropper", "dropper.usd", (DROPPER_XY[0], DROPPER_XY[1], HOLE_Z), None, False),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot180=False):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot180:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (" rot180" if rot180 else "") + (f" scale {scale}" if scale else ""))


# ---- A1 效果 prim 材质配方（待测有机液，淡琥珀，nD≈1.40）----
SAMPLE = dict(color=(0.95, 0.90, 0.72), opacity=0.75, roughness=0.08, ior=1.40)  # 瓶内液面 / 棱镜液滴
FILL = dict(color=(0.92, 0.80, 0.50), opacity=0.90, roughness=0.05, ior=1.40)    # 滴管尖内吸起液柱
DROP = dict(color=(0.92, 0.80, 0.50), opacity=0.90, roughness=0.05, ior=1.40)    # 挤胶头滴落液滴
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)

# 瓶内液面（(BOTTLE_X,BOTTLE_Y) 台面 0.80..液面 0.840 = 半瓶；Ø36 内 Ø~34）
BOTTLE_LIQ_R = 0.014
BOTTLE_LIQ_H = 0.040            # 0.80..0.84
BOTTLE_LIQ_CZ = TABLE_TOP + BOTTLE_LIQ_H / 2      # 0.820

# 棱镜液滴（落在棱镜面，初始隐藏；棱镜世界中心 (0.30,0.11) 顶 z 0.9175）
PRISM_TOP_Z = TABLE_TOP + 0.1175                  # 0.9175
PRISM_DROP_R = 0.009
PRISM_DROP_H = 0.0012
PRISM_DROP_CZ = PRISM_TOP_Z + PRISM_DROP_H / 2    # 0.9181

# 滴管尖内吸起液柱（截锥，translate=尖嘴 0.806；同 d3l/d4l 约定）
FILL_R = (0.001, 0.0035)
FILL_H = 0.060

# 挤胶头滴落串：一次挤 DROPS_PER_GROUP 滴连续坠落
DROPS_PER_GROUP = 4
DROP_BALL_R = 0.003
DROP_HOME = (REFRACT_X, PRISM_CY, PRISM_TOP_Z + 0.02)  # 棱镜上方，task 动画才写实际坐标

# 屏幕读数发光（屏幕局部中心 (0,-0.149,0.0537)，后倾 ~19.5°；读数时显示）
SCREEN_C = (REFRACT_X, REFRACT_Y - 0.156, TABLE_TOP + 0.0537)   # (0.30,-0.156,0.8537)
SCREEN_UP = (0.0, 0.3335, 0.9427)   # 屏幕"向上"单位向量（底前 0.0081→顶后 0.0993）


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided。emissive：自发光。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if ior is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
    if double_sided and prim.IsA(UsdGeom.Gprim):
        UsdGeom.Gprim(prim).CreateDoubleSidedAttr().Set(True)


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。
    16 段圆周 + 底/顶 cap，subdivisionScheme=none。"""
    n = 16
    pts, counts, indices = [], [], []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_bottom * math.cos(a), r_bottom * math.sin(a), 0.0))
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_top * math.cos(a), r_top * math.sin(a), h))
    pts += [Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, h)]      # 底心 idx 2n、顶心 idx 2n+1
    for i in range(n):
        i0, i1 = i, (i + 1) % n
        counts.append(4)                                # 侧壁四边形（法向朝外）
        indices += [i0, i1, i1 + n, i0 + n]
    counts.append(n)                                    # 底 cap（法向朝下 -Z）
    indices += [2 * n] + list(range(n - 1, -1, -1))
    counts.append(n)                                    # 顶 cap（法向朝上 +Z）
    indices += [2 * n + 1] + list(range(n, 2 * n))
    mesh = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    return mesh


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 Xform + Drop_0.._N 琥珀小球。整体初始隐藏，
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。"""
    g = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    for i in range(DROPS_PER_GROUP):
        s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/Drop_{i}")
        s.CreateRadiusAttr(DROP_BALL_R)
        s.AddTranslateOp().Set(Gf.Vec3d(*DROP_HOME))
        add_material(stage, s.GetPrim(), DROP["color"], DROP["opacity"],
                     roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DropperDrop hidden ({DROPS_PER_GROUP} drop spheres)")


def add_a1_effects(stage):
    """内建效果 prim：瓶内液面（可见）+ 棱镜液滴（隐藏）+ 滴管尖液柱（隐藏）
    + 屏幕读数发光（隐藏）。"""
    # 样品瓶内液面：淡琥珀半透明柱（瓶底 0.80..液面 0.840 = 半瓶），可见（吸液源）
    sl = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    sl.CreateRadiusAttr(BOTTLE_LIQ_R)
    sl.CreateHeightAttr(BOTTLE_LIQ_H)
    sl.CreateAxisAttr("Z")
    sl.AddTranslateOp().Set(Gf.Vec3d(BOTTLE_X, BOTTLE_Y, BOTTLE_LIQ_CZ))
    add_material(stage, sl.GetPrim(), SAMPLE["color"], SAMPLE["opacity"],
                 roughness=SAMPLE["roughness"], ior=SAMPLE["ior"], double_sided=True)
    print(f"[effect] SampleLiquid visible (bottle {BOTTLE_LIQ_H:.3f}m to top 0.840)")

    # 棱镜液滴：淡琥珀薄圆盘落在棱镜面（顶 0.9175），初始隐藏，滴样后显示
    pd = UsdGeom.Cylinder.Define(stage, "/World/PrismDrop")
    pd.CreateRadiusAttr(PRISM_DROP_R)
    pd.CreateHeightAttr(PRISM_DROP_H)
    pd.CreateAxisAttr("Z")
    pd.AddTranslateOp().Set(Gf.Vec3d(REFRACT_X, PRISM_CY, PRISM_DROP_CZ))
    add_material(stage, pd.GetPrim(), SAMPLE["color"], SAMPLE["opacity"],
                 roughness=SAMPLE["roughness"], ior=SAMPLE["ior"], double_sided=True)
    UsdGeom.Imageable(pd).MakeInvisible()
    print(f"[effect] PrismDrop hidden on prism top {PRISM_TOP_Z:.4f}")

    # 滴管尖内吸起液柱（截锥 mesh，底心贴尖嘴 0.806）：初始隐藏，吸液后 task 跟随尖嘴
    fill = add_frustum(stage, "DropperFill", FILL_R[0], FILL_R[1], FILL_H)
    fill.AddTranslateOp().Set(Gf.Vec3d(DROPPER_XY[0], DROPPER_XY[1], HOLE_Z))
    add_material(stage, fill.GetPrim(), FILL["color"], FILL["opacity"],
                 roughness=FILL["roughness"], ior=FILL["ior"], double_sided=True)
    UsdGeom.Imageable(fill).MakeInvisible()
    print(f"[effect] DropperFill frustum hidden at tip (r {FILL_R} h {FILL_H})")

    # 屏幕读数发光：倾斜矩形贴合屏幕前表面（后倾 ~19.5°），初始隐藏，读数时显示
    cx, cy, cz = SCREEN_C
    upx, upy, upz = SCREEN_UP
    hw, hh = 0.05, 0.02          # 半宽 5cm / 半高 2cm
    pts = [
        Gf.Vec3f(cx - hw, cy - hh * upy, cz - hh * upz),
        Gf.Vec3f(cx + hw, cy - hh * upy, cz - hh * upz),
        Gf.Vec3f(cx + hw, cy + hh * upy, cz + hh * upz),
        Gf.Vec3f(cx - hw, cy + hh * upy, cz + hh * upz),
    ]
    gl = UsdGeom.Mesh.Define(stage, "/World/ScreenGlow")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    gl.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gl.CreateSubdivisionSchemeAttr("none")
    add_material(stage, gl.GetPrim(), (0.9, 0.95, 1.0), 1.0,
                 emissive=(0.15, 0.85, 0.45), double_sided=True)
    UsdGeom.Imageable(gl).MakeInvisible()
    print(f"[effect] ScreenGlow hidden at screen front {SCREEN_C}")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效）。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去器材资产自带的 flametest 残留 DomeLight（/root/env_light）。"""
    for name, *_ in EQUIP:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[clean] /World/{name} not found, skip")
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")
        if not paths:
            print(f"[clean] no DomeLight in {name}")


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply，直接用 material:binding relationship 取材质路径）。"""
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


def fix_bottle_materials(st2):
    """样品瓶玻璃透明化（磨砂 op0.8 → 真玻璃 op0.25 + ior 1.5 + doubleSided），
    瓶内 SampleLiquid 液面才透得出来。**只改 bottle mesh，stopper 保持白盖不透明**（
    取瓶/归瓶步机械臂要拔/盖塞，塞是实体盖不是玻璃）。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print(f"[mat] /World/SampleBottle not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if c.GetName() == "bottle":
            if override_bound_shader(st2, c, GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
        else:
            print(f"[mat] {c.GetPath()} ({c.GetName()}) kept as-is (stopper stays opaque)")


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 是 opacity=1.0 不透明光面，
    改成真玻璃 op 0.25（同瓶玻璃配方），管内 DropperFill 液柱才透得出来；胶头保持不透明。"""
    mat = st2.GetPrimAtPath("/World/Dropper/_materials/glass_001")
    if not mat.IsValid():
        print(f"[mat] Dropper glass_001 not found, skip")
    else:
        for c in mat.GetChildren():
            if c.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(c)
            for n, val in GLASS.items():
                inp = sh.GetInput(n)
                vt = Sdf.ValueTypeNames.Color3f if n == "diffuseColor" else Sdf.ValueTypeNames.Float
                if not inp:
                    inp = sh.CreateInput(n, vt)
                inp.Set(val)
            print(f"[mat] Dropper glass_001 -> transparent {GLASS}")
    g = st2.GetPrimAtPath("/World/Dropper/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")
    else:
        print(f"[mat] Dropper glass mesh not found for doubleSided, skip")


def verify(st2):
    """自检：打印各器材世界 bbox，断言布局关系：
    折光仪贴台面（棱镜顶 0.9175、盖保留 -50° 掀开态）、样品瓶贴台面（塞保留未删）、
    架贴台面、滴管插孔（底落孔底 0.806）、瓶内液面可见、棱镜液滴/滴管液柱/滴落球/读数发光
    初始隐藏。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["Refractometer", "SampleBottle", "TestTubeRack", "Dropper",
             "SampleLiquid", "PrismDrop", "DropperFill", "DropperDrop", "ScreenGlow"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:13s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")

    # 折光仪：机身底贴台面，棱镜/盖在顶
    rmn, rmx = boxes["Refractometer"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"refractometer base z {rmn[2]} != table {TABLE_TOP}"
    assert rmx[2] > PRISM_TOP_Z, f"refractometer top {rmx[2]} below prism top {PRISM_TOP_Z}"
    # 盖保留（资产已内置 -50° 掀开态，A1 无掀盖动作、末步合盖由 task 驱动）；棱镜 prim 存在且坐标正确
    cover = st2.GetPrimAtPath("/World/Refractometer/cover")
    assert cover.IsValid(), "cover prim missing (A1 keeps it for the close step)"
    prism = st2.GetPrimAtPath("/World/Refractometer/prism")
    assert prism.IsValid(), "prism prim missing"
    cr = bc.ComputeWorldBound(cover).ComputeAlignedRange()
    pr = bc.ComputeWorldBound(prism).ComputeAlignedRange()
    print(f"[verify] cover  min({cr.GetMin()[0]:+.4f},{cr.GetMin()[1]:+.4f},{cr.GetMin()[2]:+.4f}) "
          f"max({cr.GetMax()[0]:+.4f},{cr.GetMax()[1]:+.4f},{cr.GetMax()[2]:+.4f})")
    print(f"[verify] prism  min({pr.GetMin()[0]:+.4f},{pr.GetMin()[1]:+.4f},{pr.GetMin()[2]:+.4f}) "
          f"max({pr.GetMax()[0]:+.4f},{pr.GetMax()[1]:+.4f},{pr.GetMax()[2]:+.4f})")
    assert abs(pr.GetMax()[2] - PRISM_TOP_Z) < 0.002, f"prism top {pr.GetMax()[2]} != {PRISM_TOP_Z}"
    assert abs((pr.GetMin()[0] + pr.GetMax()[0]) / 2 - REFRACT_X) < 0.003, "prism x center off"
    assert abs((pr.GetMin()[1] + pr.GetMax()[1]) / 2 - PRISM_CY) < 0.003, "prism y center off"

    # 样品瓶：贴台面，塞保留（未删），瓶口 rim 0.870
    bmn, bmx = boxes["SampleBottle"]
    assert abs(bmn[2] - TABLE_TOP) < 0.002, f"bottle bottom {bmn[2]} not on table"
    assert bmx[2] > TABLE_TOP + 0.078, f"bottle top {bmx[2]} below stopper top"
    sbp = st2.GetPrimAtPath("/World/SampleBottle")
    stoppers = [pp.GetName() for pp in Usd.PrimRange(sbp) if pp.GetName() == "stopper"]
    assert stoppers, f"stopper missing (A1 keeps it for 拔/盖): {stoppers}"
    print(f"[verify] bottle stopper kept: {stoppers}")

    # 架贴台面、滴管插孔（底落孔底 0.806）
    kmn, kmx = boxes["TestTubeRack"]
    assert abs(kmn[2] - TABLE_TOP) < 0.002, f"rack bottom {kmn[2]} not on table"
    dmn, dmx = boxes["Dropper"]
    assert abs(dmn[2] - HOLE_Z) < 0.002, f"dropper bottom {dmn[2]} != hole bottom {HOLE_Z}"
    assert abs((dmn[0] + dmx[0]) / 2 - DROPPER_XY[0]) < 0.003, f"dropper x center off left hole"

    # 效果 prim：瓶内液面可见、顶 ≤0.841；棱镜液滴隐藏且在棱镜顶；滴管液柱隐藏贴尖嘴；
    # 滴落球隐藏 4 球；读数发光隐藏
    sl = boxes.get("SampleLiquid")
    assert sl is not None, "SampleLiquid missing"
    assert sl[1][2] <= TABLE_TOP + 0.041, f"sample liquid top {sl[1][2]} above 0.841"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/SampleLiquid")).ComputeVisibility() != "invisible", \
        "SampleLiquid should be visible"
    pd = boxes["PrismDrop"]
    assert abs(pd[0][2] - PRISM_TOP_Z) < 0.002, f"prism drop bottom {pd[0][2]} not on prism top {PRISM_TOP_Z}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/PrismDrop")).ComputeVisibility() == "invisible", \
        "PrismDrop should be hidden"
    fl = boxes["DropperFill"]
    assert abs(fl[0][2] - HOLE_Z) < 0.002, f"DropperFill bottom {fl[0][2]} not at tip {HOLE_Z}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/DropperFill")).ComputeVisibility() == "invisible", \
        "DropperFill should be hidden"
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    assert dd.IsValid(), "DropperDrop missing"
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == DROPS_PER_GROUP, f"DropperDrop spheres {nd} != {DROPS_PER_GROUP}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible", "DropperDrop should be hidden"
    sg = st2.GetPrimAtPath("/World/ScreenGlow")
    assert sg.IsValid(), "ScreenGlow missing"
    assert UsdGeom.Imageable(sg).ComputeVisibility() == "invisible", "ScreenGlow should be hidden"
    print(f"[verify] OK: 折光仪贴台(棱镜顶0.9175/盖保留-50°掀开) / 瓶贴台(塞保留) / 架贴台 / 滴管插孔(0.806) "
          f"/ 瓶液面可见+棱镜滴隐藏+滴管液柱隐藏+滴球{nd}隐藏+读数发光隐藏")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot180 in EQUIP:
        add_equip(stage, name, asset, t, scale, rot180)
    add_a1_effects(stage)
    add_dropper_drops(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    fix_bottle_materials(st2)    # 瓶玻璃透明化（**塞保留不透明**）
    fix_dropper_materials(st2)   # 滴管玻璃透明化（管内 DropperFill 液柱透出）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
