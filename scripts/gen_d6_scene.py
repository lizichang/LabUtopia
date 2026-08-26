# -*- coding: utf-8 -*-
"""生成 d6_testpaper_gas.usd —— D6 试纸气体检测（通用）场景（烘平自包含，真实器材）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材（lab_clean 干净，无需删 prim、无需抬台面）
- 试纸夹（test_paper_holder.usd）预夹好试纸，机械臂不碰试纸
- 试管含预置反应混合物（TubeSolution 液体，颜色由 cfg.liquid_color 6 色变体）
- 效果 prim（初始按默认态显示，task 动画驱动 visibility 切换）：
    TestPaper        试纸 4 变体（oxidative/alkaline × blue/negative 湿润端变色）
    TubeSolution     试管内反应液体 6 色变体（cfg.liquid_color）
    DropperDrop      滴管滴落水滴（润湿试纸动画）

布局（D6 试纸气体检测，操作区前移远离底座 y=0.57）：
  TestTubeRack   (0.28,0.16)  试管架（含反应试管 + 蒸馏水滴管）
  TestTube       (0.260,0.276) 反应试管（前排左孔，口 0.959）
  Dropper        (0.300,0.041) 蒸馏水滴管（后排右孔，已吸好蒸馏水）
  TestPaperHolder(0.48,0.20)  试纸夹（立杆顶 0.99，夹试纸水平伸出，湿润端悬挑）

用法：python scripts/gen_d6_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d6_testpaper_gas")
OUT = os.path.join(SCENE_DIR, "d6_testpaper_gas.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 布局坐标（世界坐标，米，Z-up）——
# 2026-08-26：试管架离底座太近（试管 0.154m）碰撞卡住 → 操作区整体前移 -Y 远离底座
# (0.25,0.57)：试管架 0.30→0.16（试管 0.416→0.276 距底座 0.294m、滴管 0.181→0.041 距 0.529m）。
# 试纸夹 (0.48,0.20) 已 0.44m（同 E1/E2 安全距离），不动。
RACK_XY = (0.28, 0.16)                       # 试管架（rack translate z=0.8965，底 0.80 顶板 0.917）
HOLE_X_L, HOLE_X_R = -0.020, 0.020           # 2 列孔 x 偏移（左/右）
HOLE_Y_FRONT, HOLE_Y_BACK = 0.116, -0.119    # 7 行孔 y 偏移（前排/后排）
TUBE_BOTTOM_Z = 0.806                        # 试管/滴管在架孔内底面（架 translate 0.8965-0.0905）

TUBE_XY = (RACK_XY[0] + HOLE_X_L, RACK_XY[1] + HOLE_Y_FRONT)    # (0.260,0.276) 反应试管
DROPPER_XY = (RACK_XY[0] + HOLE_X_R, RACK_XY[1] + HOLE_Y_BACK)  # (0.300,0.041) 蒸馏水滴管
HOLDER_XY = (0.48, 0.20)                     # 试纸夹（立杆顶 0.99，夹试纸水平伸出 +X）

# —— 试纸（夹缝 z≈0.99；70×12×1mm，后端 15mm 夹在夹缝、前端悬挑，湿润端=最后 15mm）——
# 2026-08-26 用户「看不出试纸变色」：宽 7→12mm（仍容于 14mm 夹持片），湿润端变色块更大更可见。
PAPER_Z = 0.99
PAPER_NEAR_X = HOLDER_XY[0] - 0.005          # 0.475 夹持端（夹缝起点）
PAPER_LEN = 0.070                            # 试纸总长 70mm
PAPER_W = 0.012                              # 宽 12mm（原 7mm，太窄变色块在 1024px 下看不清）
PAPER_T = 0.001                              # 厚 1mm
WET_LEN = 0.015                              # 湿润端长 15mm
BODY_LEN = PAPER_LEN - WET_LEN               # 非湿润端 55mm
BODY_CX = PAPER_NEAR_X + BODY_LEN / 2        # 0.5025
WET_CX = PAPER_NEAR_X + BODY_LEN + WET_LEN / 2  # 0.5375 湿润端中心

# —— 试纸颜色（4 变体 = 试纸类型 × 是否变色）——
PAPER_WHITE = (0.92, 0.91, 0.86)   # 淀粉碘化钾试纸（米白，氧化性气体检测）
PAPER_RED = (0.80, 0.28, 0.30)     # 红色石蕊试纸（碱性气体检测）
PAPER_BLUE = (0.12, 0.30, 0.72)    # 检测蓝 diffuse（淀粉-碘蓝 / 石蕊变蓝）
# 检测蓝 emissive：CylinderLight 12000 + DomeLight 2000 强光会把纯 diffuse 洗淡，加 emissive
# （单通道蓝主导）让湿润端变蓝在明亮光照下仍饱和可见（同 d2l 液体/d2s 药匙配方思路）。
PAPER_BLUE_EMISSIVE = (0.18, 0.38, 0.90)
PAPER_ROUGH = 0.90

# —— 试管内反应液体（6 色变体，cfg.liquid_color；圆柱 r8mm h40mm 贴管底）——
LIQUID_R, LIQUID_H = 0.008, 0.040
LIQUID_ROUGH = 0.15
LIQUID_OPACITY = 0.92
LIQUID_COLORS = {
    "colorless": (0.88, 0.89, 0.91),
    "blue": (0.15, 0.35, 0.78),
    "red": (0.78, 0.20, 0.22),
    "green": (0.18, 0.58, 0.34),
    "yellow": (0.86, 0.76, 0.20),
    "purple": (0.55, 0.28, 0.68),
}

# —— 滴管滴落水滴（润湿试纸动画，2 滴）——
DROPLET_R = 0.002


def add_material(stage, prim, recipe, double_sided=False):
    """UsdPreviewSurface 材质（同 e2/e1）。recipe 键：diffuseColor/color, opacity, roughness,
    ior, emissive, metallic。透材质（opacity<1）自动 doubleSided。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    diffuse = recipe.get("diffuseColor", recipe.get("color", (0.9, 0.9, 0.9)))
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(recipe.get("opacity", 1.0))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(recipe.get("roughness", 0.5))
    if recipe.get("metallic") is not None:
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(recipe["metallic"])
    if recipe.get("ior") is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(recipe["ior"])
    if recipe.get("emissive") is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*recipe["emissive"]))
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


def _add_box(stage, path, size, center, recipe):
    """薄长方体（试纸/湿润端用）。xformOpOrder [translate, scale]（先 translate 再 scale，
    否则 scale 会把 translate 一起乘掉，见 gen_test_paper_holder.py 同款坑）。"""
    box = UsdGeom.Cube.Define(stage, path)
    box.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2]))
    box.AddScaleOp().Set(Gf.Vec3f(size[0] / 2, size[1] / 2, size[2] / 2))
    add_material(stage, box.GetPrim(), recipe)
    return box


def add_paper_variant(stage, key, body_color, wet_color, wet_emissive=None):
    """一个试纸变体 = 非湿润端(body 55mm, 试纸基色) + 湿润端(wetend 15mm, 检测色)。

    检出变蓝的湿润端带 emissive（wet_emissive 非 None），强光下仍饱和可见（用户「看不出变色」）。"""
    UsdGeom.Xform.Define(stage, f"/World/TestPaper/{key}")
    _add_box(stage, f"/World/TestPaper/{key}/body", (BODY_LEN, PAPER_W, PAPER_T),
             (BODY_CX, HOLDER_XY[1], PAPER_Z),
             dict(color=body_color, roughness=PAPER_ROUGH))
    _add_box(stage, f"/World/TestPaper/{key}/wetend", (WET_LEN, PAPER_W, PAPER_T),
             (WET_CX, HOLDER_XY[1], PAPER_Z),
             dict(color=wet_color, roughness=PAPER_ROUGH, emissive=wet_emissive))


def add_paper(stage):
    """试纸 4 变体：oxidative(米白)/alkaline(红) × blue(检出变蓝)/negative(未变)。
    默认 oxidative_negative 可见（预夹好、未检测）；task 按 cfg.gas_result 切 visibility。"""
    variants = {
        "oxidative_blue": (PAPER_WHITE, PAPER_BLUE, PAPER_BLUE_EMISSIVE),
        "oxidative_negative": (PAPER_WHITE, PAPER_WHITE, None),
        "alkaline_blue": (PAPER_RED, PAPER_BLUE, PAPER_BLUE_EMISSIVE),
        "alkaline_negative": (PAPER_RED, PAPER_RED, None),
    }
    for key, (body, wet, emissive) in variants.items():
        add_paper_variant(stage, key, body, wet, emissive)
        xf = stage.GetPrimAtPath(f"/World/TestPaper/{key}")
        visible = (key == "oxidative_negative")   # 默认态：预夹好未检测
        UsdGeom.Imageable(xf).CreateVisibilityAttr().Set("invisible" if not visible else "inherited")
    print("[effect] TestPaper 4 variants (default oxidative_negative visible)")


def add_liquid(stage):
    """试管内反应液体 6 色变体（父 Xform 贴管底，task 让父跟随试管平移）。默认 blue。"""
    parent = UsdGeom.Xform.Define(stage, "/World/TubeSolution")
    parent.AddTranslateOp().Set(Gf.Vec3d(TUBE_XY[0], TUBE_XY[1], TUBE_BOTTOM_Z))
    for key, color in LIQUID_COLORS.items():
        cyl = UsdGeom.Cylinder.Define(stage, f"/World/TubeSolution/liquid_{key}")
        cyl.CreateRadiusAttr(LIQUID_R)
        cyl.CreateHeightAttr(LIQUID_H)
        cyl.CreateAxisAttr("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, LIQUID_H / 2))
        add_material(stage, cyl.GetPrim(),
                     dict(color=color, opacity=LIQUID_OPACITY, roughness=LIQUID_ROUGH, ior=1.33))
        visible = (key == "blue")
        UsdGeom.Imageable(cyl.GetPrim()).CreateVisibilityAttr().Set(
            "invisible" if not visible else "inherited")
    print("[effect] TubeSolution 6 liquid variants (default blue visible)")


def add_droplet(stage):
    """滴管滴落水滴（润湿试纸，2 滴），初始隐藏，task 动画从滴管尖坠到试纸湿润端。"""
    parent = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    parent.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    for i in range(2):
        sph = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/drop_{i}")
        sph.CreateRadiusAttr(DROPLET_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(0, 0, DROPLET_R))
        add_material(stage, sph.GetPrim(),
                     dict(color=(0.85, 0.88, 0.92), opacity=0.60, roughness=0.10, ior=1.33))
    UsdGeom.Imageable(parent).CreateVisibilityAttr().Set("invisible")
    print("[effect] DropperDrop 2 water drops (hidden initially)")


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
    """扫除烘平后残留的嵌套 DomeLight，只留 /World/env_light（test_tube_rack 等 equipment
    自带 env_light 贴图是 flametest 的 1×1 近黑，烘平后压暗环境反黑，同 d2s/e2）。"""
    keep = {"/World/env_light"}
    root = st2.GetPrimAtPath("/World")
    paths = [p.GetPath() for p in Usd.PrimRange(root)
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString not in keep]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed stray DomeLight {path}")
    if not paths:
        print("[clean] no stray DomeLight under /World")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for name in ["TestPaperHolder", "TestTubeRack", "TestTube", "Dropper"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:16s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 不变量：试管/滴管底面贴架孔底 0.806，试纸夹底座贴台面 0.80，试纸湿润端 z=0.99
    tube = st2.GetPrimAtPath("/World/TestTube")
    tr = bc.ComputeWorldBound(tube).ComputeAlignedRange()
    assert abs(tr.GetMin()[2] - TUBE_BOTTOM_Z) < 1e-3, \
        f"TestTube bottom z={tr.GetMin()[2]:.4f} != {TUBE_BOTTOM_Z}"
    holder = st2.GetPrimAtPath("/World/TestPaperHolder")
    hr = bc.ComputeWorldBound(holder).ComputeAlignedRange()
    assert abs(hr.GetMin()[2] - TABLE_TOP) < 1e-3, \
        f"Holder bottom z={hr.GetMin()[2]:.4f} != {TABLE_TOP}"
    # 默认可见性：试纸 oxidative_negative 可见、TubeSolution blue 可见、DropperDrop 隐藏
    for key, want in [("oxidative_negative", "inherited"), ("oxidative_blue", "invisible")]:
        xf = st2.GetPrimAtPath(f"/World/TestPaper/{key}")
        assert UsdGeom.Imageable(xf).ComputeVisibility() == want, \
            f"TestPaper/{key} visibility != {want}"
    liquid_blue = st2.GetPrimAtPath("/World/TubeSolution/liquid_blue")
    assert UsdGeom.Imageable(liquid_blue).ComputeVisibility() == "inherited", \
        "TubeSolution/liquid_blue should be visible by default"
    droplet = st2.GetPrimAtPath("/World/DropperDrop")
    assert UsdGeom.Imageable(droplet).ComputeVisibility() == "invisible", \
        "DropperDrop should be hidden initially"
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print("[verify] tube bottom 0.806 / holder bottom 0.80 / paper z 0.99 / "
          "default visibility ok / no stray DomeLight")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    stage = Usd.Stage.Open(LAB_CLEAN)
    add_equip(stage, "TestPaperHolder", "test_paper_holder.usd", (HOLDER_XY[0], HOLDER_XY[1], None), None)
    add_equip(stage, "TestTubeRack", "test_tube_rack.usd", (RACK_XY[0], RACK_XY[1], None), None)
    add_equip(stage, "TestTube", "test_tube.usd", (TUBE_XY[0], TUBE_XY[1], TUBE_BOTTOM_Z), None)
    add_equip(stage, "Dropper", "dropper.usd", (DROPPER_XY[0], DROPPER_XY[1], TUBE_BOTTOM_Z), None)
    add_paper(stage)
    add_liquid(stage)
    add_droplet(stage)
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
