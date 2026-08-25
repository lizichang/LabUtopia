# -*- coding: utf-8 -*-
"""生成 e1_ph_testpaper.usd —— E1「pH 试纸检测」场景（烘平自包含，真实器材）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材（lab_clean 干净，无需删 prim、无需抬台面）
- 试管 + 待测溶液插试管架（同 D4-L 孔位约定：底面 z=0.806 = 架z−0.0905）
- 试纸条（TestPaper）已默认铺在白瓷板中央（简化动作：不再夹取）；3 个 pH 色斑变体
  （spot_acidic/spot_neutral/spot_alkaline）为 TestPaper 子 prim，初始全隐藏
  （task 按 cfg.ph_result 显示其一 —— 预留的 pH 变色接口）
- 环境光 + 主光提亮（同 d4l）；玻璃棒玻璃化透明

布局（E1 目录「中央前方左→右排列」：白瓷板→比色卡；试管架居中，玻璃棒/待测溶液都插试管架
前排孔——玻璃棒竖直插孔可抓取，与 d2s 药匙同款；平放 Ø6mm 棒贴桌面时夹爪手指需下探到台面
下会穿模，故竖直插孔。试纸本（PhBook）已删：动作简化后试纸条直接预铺白瓷板、不再从试纸本
撕取，米色 45×70 试纸本反而被误认成"多出的一张 pH 纸"）：
  TestTubeRack (0.30,  0.00)  底座贴台面 z 自动
  TestTube     (0.2787, 0.1193, 0.806)  前排左孔（待测溶液）
  GlassRod     (0.319,  0.117, 0.806)  玻璃棒 Ø6×261mm 竖直插前排右孔（底 z=0.806 顶 z=1.067）
  WhitePlate   (0.46,  0.32)  白瓷板 80×80×6mm
  TestPaper    (0.46,  0.32, 0.80675) 试纸条 7×70×0.5mm，默认铺在白瓷板中央（不再夹取）
  ColorChart   (0.62,  0.32)  比色卡 90×40×1.2mm

用法：python scripts/gen_e1_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "e_physical", "e1_ph_testpaper")
OUT = os.path.join(SCENE_DIR, "e1_ph_testpaper.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
HOLE_BOTTOM = 0.806   # 插孔底面 z = 架 z − 0.0905（skill 坑 33）

# (prim, asset_file, translate, scale, rot)   tz=None → 动态贴台面；rot=(rx,ry,rz)°
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None, None),
    ("TestTube", "test_tube.usd", (0.2787, 0.1193, HOLE_BOTTOM), None, None),
    # 玻璃棒 Ø6×261 长轴沿 Z（资产），竖直插前排右孔（同 d2s 药匙），底 z=0.806。
    ("GlassRod", "glass_rod_6x6x261.usd", (0.319, 0.117, HOLE_BOTTOM), None, None),
    ("WhitePlate", "white_porcelain_plate.usd", (0.46, 0.32, TABLE_TOP), None, None),
    ("ColorChart", "ph_color_chart.usd", (0.62, 0.32, TABLE_TOP), None, None),
]

# 试纸条几何（场景级 prim）：7(X)×70(Y)×0.5(Z)mm，中心在 TestPaper 原点。
STRIP_W, STRIP_L, STRIP_T = 0.007, 0.070, 0.0005
# 试纸条已默认铺在白瓷板中央（简化：不再夹取）。板顶 0.806 + 0.5mm 抬高（避免条底面与
# 板顶共面 z-fighting）+ 条半厚 0.00025 → 条中心 z=0.80675、顶面 0.8070。
STRIP_POS = (0.46, 0.32, TABLE_TOP + 0.006 + 0.0005 + STRIP_T / 2)  # 中心 0.80675

# 待测溶液（约 1mL 已配制好）：试管内液柱，r 取管内径 ~7.5mm、h 6mm（1mL）
SOLUTION = dict(r=0.0075, h=0.006, t=(0.2787, 0.1193, 0.808), color=(0.75, 0.80, 0.85),
                opacity=0.55, roughness=0.05, ior=1.33)

# pH 色斑变体（预留的变色接口）：近黑 diffuse + 单通道主导 emissive（flametest 坑 28，
# 小体积显色物纯 diffuse 会被 CylinderLight 12000 洗白）。task 按 cfg.ph_result ∈
# {acidic, neutral, alkaline} 显示对应变体。色斑 = 试纸中央的液滴印记。
# 2026-08-25 半径 3→6mm（12mm 直径）：camera1/2 512px@~1m 下 1px≈2.5mm，6mm 色斑仅 ~3px
# 几乎看不见（用户报"试纸上颜色没看出变化"），放大到 12mm + emissive 泛光才可辨识。
SPOT_R, SPOT_H = 0.006, 0.0005
SPOT_COLORS = {
    "acidic":  dict(color=(0.05, 0.02, 0.02), emissive=(2.2, 0.12, 0.12)),   # 红
    "neutral": dict(color=(0.02, 0.08, 0.03), emissive=(0.20, 1.60, 0.15)),  # 黄绿
    "alkaline": dict(color=(0.02, 0.03, 0.10), emissive=(0.12, 0.30, 2.2)),  # 蓝
}

# 玻璃材质：玻璃棒透明化（玻璃棒资产 glass_rod 默认可能不透明，透明化让棒更逼真）
GLASS_ROD = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.30, roughness=0.10, ior=1.5)


def add_material(stage, prim, recipe, double_sided=False):
    """UsdPreviewSurface 材质。recipe 键：color/diffuseColor, opacity, roughness, ior,
    emissive。透材质（opacity<1）自动 doubleSided。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    diffuse = recipe.get("diffuseColor", recipe.get("color", (0.9, 0.9, 0.9)))
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(recipe.get("opacity", 1.0))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(recipe.get("roughness", 0.5))
    if recipe.get("ior") is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(recipe["ior"])
    if recipe.get("emissive") is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*recipe["emissive"]))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
    if double_sided and prim.IsA(UsdGeom.Gprim):
        UsdGeom.Gprim(prim).CreateDoubleSidedAttr().Set(True)


def asset_local_min_z(asset_file):
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(os.path.abspath(os.path.join(EQ, asset)))
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(rot[0], rot[1], rot[2]))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx},{ty},{tz})"
          + (f" rot{rot}" if rot else "") + (f" scale {scale}" if scale else ""))


def add_test_paper(stage):
    """试纸条 = /World/TestPaper Xform（静态，已默认铺在白瓷板中央）+ paper 网格 + 3 个
    pH 色斑变体子 prim（初始隐藏，task 按 cfg.ph_result 显示其一）。色斑在试纸中央
    （局部原点上方 STRIP_T/2 + SPOT_H/2）。"""
    paper_xf = UsdGeom.Xform.Define(stage, "/World/TestPaper")
    paper_xf.AddTranslateOp().Set(Gf.Vec3d(*STRIP_POS))

    # 纸片：UsdGeom.Cube（默认 size=2，即 extent −1..+1）scale 半尺寸 → 7×70×0.5mm
    paper = UsdGeom.Cube.Define(stage, "/World/TestPaper/paper")
    paper.AddScaleOp().Set(Gf.Vec3f(STRIP_W / 2, STRIP_L / 2, STRIP_T / 2))
    # pH 试纸黄（与白瓷板拉开对比：米色 0.5mm 条在白板上几乎不可见）
    add_material(stage, paper.GetPrim(),
                 dict(color=(0.92, 0.82, 0.28), opacity=1.0, roughness=0.85))
    print(f"[effect] TestPaper paper strip {STRIP_W*1000:.0f}x{STRIP_L*1000:.0f}mm at {STRIP_POS}")

    # 3 个色斑变体（隐藏）
    for key, recipe in SPOT_COLORS.items():
        spot = UsdGeom.Cylinder.Define(stage, f"/World/TestPaper/spot_{key}")
        spot.CreateRadiusAttr(SPOT_R)
        spot.CreateHeightAttr(SPOT_H)
        spot.CreateAxisAttr("Z")
        # 底面抬 0.1mm 离开纸顶（0.8070），避免色斑底与纸顶共面 z-fight
        spot.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, STRIP_T / 2 + SPOT_H / 2 + 0.0001))
        add_material(stage, spot.GetPrim(), dict(recipe, opacity=1.0, roughness=0.3))
        UsdGeom.Imageable(spot).MakeInvisible()
        print(f"[effect] TestPaper/spot_{key} hidden (r={SPOT_R}, emissive {recipe['emissive']})")


def add_solution(stage):
    cyl = UsdGeom.Cylinder.Define(stage, "/World/TestSolution")
    cyl.CreateRadiusAttr(SOLUTION["r"])
    cyl.CreateHeightAttr(SOLUTION["h"])
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(*SOLUTION["t"]))
    add_material(stage, cyl.GetPrim(), SOLUTION, double_sided=True)
    print(f"[effect] TestSolution visible at {SOLUTION['t']} (r {SOLUTION['r']} h {SOLUTION['h']})")


def add_env_light(stage):
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity -> 12000")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_stray_env_lights(st2):
    keep = {"/World/env_light"}
    root = st2.GetPrimAtPath("/World")
    paths = [p.GetPath() for p in Usd.PrimRange(root)
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString not in keep]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed stray DomeLight {path}")
    if not paths:
        print("[clean] no stray DomeLight under /World")


def override_bound_shader(st2, prim, recipe):
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
            vt = Sdf.ValueTypeNames.Color3f if name == "diffuseColor" else Sdf.ValueTypeNames.Float
            inp = sh.GetInput(name)
            if not inp:
                inp = sh.CreateInput(name, vt)
            inp.Set(val)
        print(f"[mat] {prim.GetPath()} -> {c.GetPath()} {recipe}")
        return True
    return False


def glassify_rod(st2):
    """玻璃棒透明化（资产 glass_rod 可能不透明，透明化更像真玻璃）。"""
    rod = st2.GetPrimAtPath("/World/GlassRod")
    if not rod.IsValid():
        print("[mat] /World/GlassRod not found, skip")
        return
    for c in rod.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, GLASS_ROD):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] glass rod {c.GetPath()} -> transparent {GLASS_ROD}")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for name in ["TestTubeRack", "TestTube", "TestPaper", "GlassRod",
                 "WhitePlate", "ColorChart", "TestSolution"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        cx = (mn[0] + mx[0]) / 2
        cy = (mn[1] + mx[1]) / 2
        cz = (mn[2] + mx[2]) / 2
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f}) "
              f"center({cx:+.4f},{cy:+.4f},{cz:+.4f})")
    # 色斑变体不变量：3 个存在、半径==SPOT_R、初始隐藏
    for key in SPOT_COLORS:
        p = st2.GetPrimAtPath(f"/World/TestPaper/spot_{key}")
        assert p.IsValid(), f"spot_{key} missing"
        r = UsdGeom.Cylinder(p).GetRadiusAttr().Get()
        assert abs(r - SPOT_R) < 1e-9, f"spot_{key} r={r} != SPOT_R={SPOT_R}"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
            f"spot_{key} should be hidden initially"
    print(f"[verify] pH spots OK: {len(SPOT_COLORS)} variants r={SPOT_R} all hidden")
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print("[verify] no stray DomeLight")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot in EQUIP:
        add_equip(stage, name, asset, t, scale, rot)
    add_test_paper(stage)
    add_solution(stage)
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    glassify_rod(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
