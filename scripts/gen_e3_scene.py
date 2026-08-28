# -*- coding: utf-8 -*-
"""生成 e3_density.usd —— E3「密度测定」场景（烘平自包含，真实器材）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 分析天平中央，量筒 10mL 预放在天平称盘上（用户拍板：位置不动，机械臂不再搬量筒）
- 移液管（5mL，压缩 ~185mm）+洗耳球插移液管架，架在天平左侧
- 样品瓶预开盖（用户拍板：省略瓶盖动作，瓶塞已摘除），在天平前方
- 待测液体：样品瓶内 + 转移后量筒内，各 6 色变体（预留 liquid_color 变色接口）
- 密度读数（天平+量筒法 ρ=Δm/5mL）：天平前面板屏贴图
    balance_m1.png        = 空量筒质量 35.00 g（开场可见）
    balance_result_<key>.png = 转移 5mL 后质量 m2 + 密度 ρ（放液后切显，按 density 档位）
- 环境光 + 主光提亮（同 e1/d6/d7）

布局（天平中央不动，量筒在称盘上，移液管左侧，样品瓶前方；整体相对 v2 -Y 移
0.15m，远离机械臂底座 (0.25,0.57)——近身短臂展会触发臂自碰撞规避改道导致穿模）：
  Balance          (0.40, 0.17)     分析天平（称盘顶 z=0.8475）
  GraduatedCylinder(0.40, 0.17)    量筒 10mL 在称盘上（底 z=0.8475，口 z=0.995）
  PipetteStand     (0.22, 0.17)     移液管架，底贴台面 z=0.80
  Pipette          (0.22, 0.17, 0.82) 移液管尖插架孔（顶 1.005）
  SampleBottle     (0.40, 0.00)     样品瓶（预开盖），底贴台面 z=0.80

用法：python scripts/gen_e3_scene.py   （运行环境：labutopia conda env 有 pxr/numpy/PIL）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "e_physical", "e3_density")
OUT = os.path.join(SCENE_DIR, "e3_density.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
BAL_XY = (0.40, 0.17)      # 分析天平（中央，不动）
CYL_XY = (0.40, 0.17)      # 量筒（天平称盘上，预置不动）
PIPE_XY = (0.22, 0.17)     # 移液管架 / 移液管（天平左侧）
BOTTLE_XY = (0.40, 0.00)   # 样品瓶（天平前方，预开盖）
PIPE_Z = 0.82              # 移液管尖 = 架底座顶（架底 0.80 + 底座厚 0.02）
CYL_BASE_Z = 0.8475        # 量筒底 = 天平称盘顶（analytical_balance.usd pan 顶 z=0.0475）

# (prim, asset_file, translate, scale, rot)   tz=None → 动态贴台面；rot=(rx,ry,rz)°
EQUIP = [
    ("Balance", "analytical_balance.usd", (BAL_XY[0], BAL_XY[1], None), None, None),
    ("GraduatedCylinder", "graduated_cylinder_10ml.usd",
     (CYL_XY[0], CYL_XY[1], CYL_BASE_Z), None, None),
    ("PipetteStand", "pipette_stand.usd", (PIPE_XY[0], PIPE_XY[1], None), None, None),
    ("Pipette", "pipette.usd", (PIPE_XY[0], PIPE_XY[1], PIPE_Z), None, None),
    ("SampleBottle", "sample_bottle.usd", (BOTTLE_XY[0], BOTTLE_XY[1], None), None, None),
]

# 待测液体 6 色变体（预留变色接口）：近黑 diffuse + 单通道主导 emissive（flametest 坑 28，
# 强 CylinderLight 下纯 diffuse 被洗白）。colorless = 清水（透明淡）。
LIQUID_COLORS = {
    "colorless": dict(color=(0.70, 0.78, 0.85), emissive=(0.0, 0.0, 0.0), opacity=0.30),
    "blue":   dict(color=(0.02, 0.03, 0.10), emissive=(0.12, 0.30, 0.85), opacity=0.95),
    "red":    dict(color=(0.08, 0.02, 0.02), emissive=(0.85, 0.12, 0.12), opacity=0.95),
    "green":  dict(color=(0.02, 0.08, 0.03), emissive=(0.15, 0.80, 0.18), opacity=0.95),
    "yellow": dict(color=(0.08, 0.07, 0.02), emissive=(0.80, 0.70, 0.10), opacity=0.95),
    "purple": dict(color=(0.06, 0.02, 0.08), emissive=(0.65, 0.15, 0.80), opacity=0.95),
}

# 量筒内 5mL 转移液柱：内径 Ø13（r 0.0065），底=筒内底（称盘顶+底座厚 0.005），
# 顶=5mL 刻度（称盘顶 + 0.0735，10mL 分度中点）。
CYL_LIQ_R, CYL_LIQ_BOT, CYL_LIQ_TOP = 0.0065, CYL_BASE_Z + 0.005, CYL_BASE_Z + 0.0735
# 样品瓶内约半瓶液柱：瓶内径 ~Ø32（r 0.014），底 0.803 顶 0.831（28mm 深）
BOT_LIQ_R, BOT_LIQ_BOT, BOT_LIQ_TOP = 0.014, 0.803, 0.831
# 移液管内吸液柱（随移液管平移，子 prim）：内径 Ø5.6（r 0.0028），充满直管段
# z=0.02→0.125（尖 0.02→5mL 刻度 0.05→0mL 刻度 0.125），局部中心 0.0725 高 0.105
PIPE_LIQ_R, PIPE_LIQ_BOT, PIPE_LIQ_TOP = 0.0028, 0.02, 0.125

# —— 密度读数（天平+量筒法）：天平前面板屏（analytical_balance.usd /root/screen）——
# 屏幕 x∈[-0.05,0.05] z∈[0.0165,0.0328] 前面 y=-0.1032（凹进机身）。贴图 quad 贴屏幕
# 前表面（y 偏 -0.104 防 z-fighting）。密度档位与 config experiment_result.density 同源。
SCREEN_TEX_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
BAL_SCREEN_C = (0.40, BAL_XY[1] - 0.104, 0.80 + (0.016509 + 0.032785) / 2)  # (0.40,0.216,0.8246)
BAL_SCREEN_UP = (0.0, 0.0, 1.0)
BAL_SCREEN_HW = 0.05                                   # 半宽 50mm（x）
BAL_SCREEN_HH = (0.032785 - 0.016509) / 2.0            # 半高 ~8.14mm（z）
M1_GRAMS = 35.00                                       # 空量筒质量（固定）
DEFAULT_DENSITY = 1.0                                  # 默认密度（仅用于预生成占位贴图）


def add_material(stage, prim, recipe, double_sided=False):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动 doubleSided。"""
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


def add_liquid_children(stage, parent_path, local_center, radius, height):
    """在已存在的 parent_path 下加 6 色液柱变体（局部中心 local_center），初始全隐藏。"""
    for key, recipe in LIQUID_COLORS.items():
        cyl = UsdGeom.Cylinder.Define(stage, f"{parent_path}/liq_{key}")
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(height)
        cyl.CreateAxisAttr("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(*local_center))
        add_material(stage, cyl.GetPrim(), recipe, double_sided=True)
        UsdGeom.Imageable(cyl).MakeInvisible()
        print(f"[effect] {parent_path}/liq_{key} hidden (r={radius} h={height})")


def add_liquid_variants(stage, name, center, radius, height):
    """在 /World/{name} 下加 6 色液柱变体（轴 Z，中心=center），初始全隐藏；
    task 按 cfg.liquid_color 显示其一。"""
    xf = UsdGeom.Xform.Define(stage, f"/World/{name}")
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    add_liquid_children(stage, f"/World/{name}", (0.0, 0.0, 0.0), radius, height)


def make_balance_textures(tex_dir):
    """用 PIL 生成天平屏贴图（labutopia conda env 有 PIL/numpy；base python 无）。
    balance_m1.png = 空量筒质量 35.00 g；balance_result.png = 转移 5mL 后质量 m2 + 密度
    ρ（m2 = 35.00 + ρ×5.00）占位版（ρ=默认值，运行时 task 按实际 density 覆写）。屏
    100×16mm → 800×128（6.25:1）。贴图接 emissiveColor → 亮字自发光、近黑屏底不发。"""
    from PIL import Image, ImageDraw, ImageFont

    def font(size):
        return ImageFont.truetype(SCREEN_TEX_FONT, size)

    W, H = 800, 128
    BG = (8, 12, 20)            # 近黑蓝屏底
    TXT = (170, 240, 200)       # 主读数绿白
    SUB = (150, 200, 185)       # 密度小字

    # m1：空量筒质量（单一档）
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    f = font(56)
    t = f"{M1_GRAMS:.2f} g"
    bb = d.textbbox((0, 0), t, font=f)
    d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 30), t, font=f, fill=TXT)
    img.save(os.path.join(tex_dir, "balance_m1.png"))

    # result：转移 5mL 后质量 m2（大字）+ 密度 ρ（小字）——单张占位（ρ=默认值），
    # 运行时 task 按实际 density 覆写同名文件 balance_result.png。
    dv = DEFAULT_DENSITY
    m2 = M1_GRAMS + dv * 5.0
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    f = font(44)
    t = f"{m2:.2f} g"
    bb = d.textbbox((0, 0), t, font=f)
    d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 10), t, font=f, fill=TXT)
    f = font(28)
    t = f"ρ = {dv:.3f} g/mL"
    bb = d.textbbox((0, 0), t, font=f)
    d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 82), t, font=f, fill=SUB)
    img.save(os.path.join(tex_dir, "balance_result.png"))
    print(f"[screen] textures -> {tex_dir} (m1 + balance_result.png placeholder)")


def add_screen_tex_quad(stage, name, tex_path, visible=False):
    """天平屏 quad（竖直 x-z 平面，前表面 y=0.216 贴屏幕）+ st UV + 贴图发光材质。
    贴图经 UsdUVTexture 接 emissiveColor：亮字自发光、近黑屏底不发。
    task 按测量阶段显隐 BalanceM1（初始质量）/BalanceResult（放液后 m2+ρ）。"""
    cx, cy, cz = BAL_SCREEN_C
    hw, hh = BAL_SCREEN_HW, BAL_SCREEN_HH
    pts = [
        Gf.Vec3f(cx - hw, cy, cz - hh),
        Gf.Vec3f(cx + hw, cy, cz - hh),
        Gf.Vec3f(cx + hw, cy, cz + hh),
        Gf.Vec3f(cx - hw, cy, cz + hh),
    ]
    gl = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    gl.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gl.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(gl).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array,
                                               UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mat = UsdShade.Material.Define(stage, f"/World/{name}_mat")
    sh = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03, 0.04, 0.06))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    reader = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Reader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_path))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(gl).Bind(mat)
    UsdGeom.Gprim(gl).CreateDoubleSidedAttr().Set(True)
    if not visible:
        UsdGeom.Imageable(gl).MakeInvisible()
    print(f"[screen] {name} {'visible' if visible else 'hidden'} (texture {tex_path})")


def add_balance_screen(stage):
    """天平屏读数 prim：BalanceM1（空量筒质量，开场可见）+ BalanceResult（放液后，
    单张 m2+ρ 贴图，运行时 task 按实际 density 覆写，初始隐藏）。"""
    add_screen_tex_quad(stage, "BalanceM1", "textures/balance_m1.png", visible=True)
    add_screen_tex_quad(stage, "BalanceResult", "textures/balance_result.png", visible=False)


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


def remove_bottle_stopper(st2):
    """样品瓶预开盖：停用瓶塞 prim（sample_bottle.usd 默认带 /root/stopper）。"""
    stopper = st2.GetPrimAtPath("/World/SampleBottle/stopper")
    if stopper.IsValid():
        stopper.SetActive(False)
        print("[clean] SampleBottle/stopper deactivated (pre-opened)")
    else:
        print("[clean] /World/SampleBottle/stopper not found, skip")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for name in ["Balance", "GraduatedCylinder", "PipetteStand", "Pipette", "SampleBottle"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:18s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 液体变体：每个容器 6 色、半径正确、初始隐藏
    for name, radius in [("CylinderLiquid", CYL_LIQ_R), ("BottleLiquid", BOT_LIQ_R)]:
        for key in LIQUID_COLORS:
            p = st2.GetPrimAtPath(f"/World/{name}/liq_{key}")
            assert p.IsValid(), f"{name}/liq_{key} missing"
            assert abs(UsdGeom.Cylinder(p).GetRadiusAttr().Get() - radius) < 1e-9
            assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
                f"{name}/liq_{key} should be hidden initially"
    print(f"[verify] liquid variants OK: 2 containers x {len(LIQUID_COLORS)} colors, all hidden")
    # 移液管内吸液柱变体（child of /World/Pipette）
    for key in LIQUID_COLORS:
        p = st2.GetPrimAtPath(f"/World/Pipette/liq_{key}")
        assert p.IsValid(), f"Pipette/liq_{key} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible"
    print(f"[verify] pipette liquid variants OK: {len(LIQUID_COLORS)} colors, all hidden")
    # 天平屏读数：BalanceM1 可见，BalanceResult 隐藏
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/BalanceM1")).ComputeVisibility() != "invisible", \
        "BalanceM1 should be visible initially"
    p = st2.GetPrimAtPath("/World/BalanceResult")
    assert p.IsValid(), "BalanceResult missing"
    assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible"
    print("[verify] balance screen: m1 visible, BalanceResult hidden")
    # 贴图存在
    texs = ["balance_m1.png", "balance_result.png"]
    for tex in texs:
        assert os.path.exists(os.path.join(SCENE_DIR, "textures", tex)), f"missing texture {tex}"
    print(f"[verify] balance textures OK: {len(texs)} files")
    # 瓶塞已停用
    assert not st2.GetPrimAtPath("/World/SampleBottle/stopper").IsActive(), \
        "bottle stopper should be deactivated"
    print("[verify] bottle stopper deactivated")
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print("[verify] no stray DomeLight")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    tex_dir = os.path.join(SCENE_DIR, "textures")
    make_balance_textures(tex_dir)
    # 清理旧的多密度档贴图（density 改为任意数值后不再需要 per-density 贴图）
    for fn in os.listdir(tex_dir):
        if fn.startswith("balance_result_") and fn.endswith(".png"):
            os.remove(os.path.join(tex_dir, fn))
            print(f"[clean] removed stale texture {fn}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot in EQUIP:
        add_equip(stage, name, asset, t, scale, rot)
    # 量筒内 5mL 转移液柱（中心 = 液柱中点）
    add_liquid_variants(stage, "CylinderLiquid",
                        (CYL_XY[0], CYL_XY[1], (CYL_LIQ_BOT + CYL_LIQ_TOP) / 2),
                        CYL_LIQ_R, CYL_LIQ_TOP - CYL_LIQ_BOT)
    # 样品瓶内液柱
    add_liquid_variants(stage, "BottleLiquid",
                        (BOTTLE_XY[0], BOTTLE_XY[1], (BOT_LIQ_BOT + BOT_LIQ_TOP) / 2),
                        BOT_LIQ_R, BOT_LIQ_TOP - BOT_LIQ_BOT)
    # 移液管内吸液柱（随移液管平移，child of /World/Pipette）
    add_liquid_children(stage, "/World/Pipette",
                        (0.0, 0.0, (PIPE_LIQ_BOT + PIPE_LIQ_TOP) / 2),
                        PIPE_LIQ_R, PIPE_LIQ_TOP - PIPE_LIQ_BOT)
    # 天平屏读数（m1 可见 + 每密度档 result 隐藏）
    add_balance_screen(stage)
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    remove_bottle_stopper(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
