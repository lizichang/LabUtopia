# -*- coding: utf-8 -*-
"""生成 d7_gas_identification.usd —— D7 气体鉴定场景（导气管 + 单孔橡皮塞）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 试管架 (0.28,0.16) 2列7行14孔；检验试管（右列第3排）
- 产气试管（带导气管橡皮塞）：移出试管架，用铁架台 + 试管夹固定（用户：抓取穿模 → 移出架外）
- 产气试管：白粉（石灰石）+ 无色酸液（稀盐酸，反应产气 CO2）
- 检验试管：检测试剂液体（澄清石灰水，颜色由 cfg.liquid_color 6 色变体）
- 带导气管橡皮塞预装塞紧产气试管口（机械臂不夹取塞子，末端悬下浸点正前方）
- 效果 prim（task 动画驱动 visibility/位移）：
    TubeSolution   检验试管内检测液 6 色变体（父 Xform 跟随试管平移）
    GasBubbles     导气管末端气泡（HoldDetect 通气动画）

布局（D7 气体鉴定，操作区沿用 D6 远离底座 y=0.57 的架位；2026-08-27 产气试管抬高 10cm
      给检验试管「从导气管下方接近」留空间，避免从导气管上方直下穿模）：
  TestTubeRack (0.28,0.16)            试管架
  IronStand    (0.30,-0.08)           铁架台立柱（底座贴台面 0.80，环/挂钩已移除）
  TestTubeClamp(0.3505,-0.1009,0.98)  试管夹（夹环套立柱 + 夹口夹产气试管）
  GasTube      (0.40,-0.08)           产气试管（试管夹固定，底 0.90，口 1.053）
  TestTube     (0.300,0.160)          检验试管（右列第3排，口 0.959）
  Stopper      (0.40,-0.08,1.044)     带导气管橡皮塞（预装塞紧，末端悬 (0.44,0.079,1.024)）

用法：python scripts/gen_d7_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d7_gas_identification")
OUT = os.path.join(SCENE_DIR, "d7_gas_identification.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 布局坐标（世界坐标，米，Z-up）——
# 试管架 2 列（x=±0.020）× 7 行（y 实测孔心，架中心为原点，间距 ~0.0397）：
#   row0=-0.119 .. row6=+0.119（row3=0.0 居中）
RACK_XY = (0.28, 0.16)
HOLE_X_L, HOLE_X_R = -0.020, 0.020
ROW3 = 0.0                                     # 右列第 3 排（居中孔，检验试管）
TUBE_BOTTOM_Z = 0.806                          # 检验试管底面（架孔底）

# 产气试管（带导气管橡皮塞）移出试管架，改用铁架台 + 试管夹固定（用户：抓取穿模 → 移出架外；
# 2026-08-27 再抬高 10cm 给「从导气管下方接近」留空间，避免检验试管从导气管上方直下穿模）。
# 试管夹局部几何（test_tube_clamp.usd bbox，与 B2 实测一致）：夹环(套立柱)在 -X、夹口(夹管)在
# +X，管中心 ≈ 夹 origin +(0.0495,0.0209)、立柱 ≈ 夹 origin +(-0.0505,0.0209)。
GAS_TUBE_BOTTOM_Z = 0.90                   # 产气试管底（试管夹抬高，离桌面 10cm）
GAS_TUBE_XY = (0.40, -0.08)                # 产气试管（试管夹固定，口 1.053）
STAND_XY = (GAS_TUBE_XY[0] - 0.100, GAS_TUBE_XY[1])          # 铁架台立柱 (0.30,-0.08)
CLAMP_XY = (GAS_TUBE_XY[0] - 0.0495, GAS_TUBE_XY[1] - 0.0209)  # 试管夹 origin (0.3505,-0.1009)
CLAMP_Z = 0.98                             # 试管夹夹管身高（管底 0.90 上 8cm、塞底 1.044 下 6.4cm）

TEST_TUBE_XY = (RACK_XY[0] + HOLE_X_R, RACK_XY[1] + ROW3)   # (0.300,0.160) 检验试管（右3排）
DIP_XY = (GAS_TUBE_XY[0] + 0.040, GAS_TUBE_XY[1] + 0.159)   # (0.440,0.079) 导气管末端悬于其上
STOPPER_PLUG_XY = GAS_TUBE_XY             # (0.40,-0.08) 橡皮塞预装塞紧产气试管口
STOPPER_PLUG_BOTTOM_Z = 1.044             # 塞底 = 管口 1.053 下 ~9mm（预装，机械臂不夹取）

# —— 产气试管内容（白粉 + 无色酸，固定不变）——
GAS_SOLID_R, GAS_SOLID_H = 0.008, 0.012      # 白粉贴管底 h12mm
GAS_SOLID_Z = GAS_TUBE_BOTTOM_Z              # 0.90
GAS_LIQUID_R, GAS_LIQUID_H = 0.008, 0.028    # 酸液在粉上 h28mm
GAS_LIQUID_Z = GAS_SOLID_Z + GAS_SOLID_H     # 0.818

# —— 检验试管检测液（6 色变体，cfg.liquid_color；圆柱 r8mm h100mm 贴管底）——
LIQUID_R, LIQUID_H = 0.008, 0.100
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

# —— 气泡（导气管末端通气，HoldDetect 动画）——
BUBBLE_R = 0.003                                # 气泡半径 Ø6mm
BUBBLE_COUNT = 8                                # 气泡队列（错帧连续上升）
FREE_END_Z = 1.024                              # 塞紧后导气管末端世界 z（塞底 1.044 - 局部 0.020，下探 75mm）
BUBBLE_SURFACE_Z = 1.039                        # 下浸后液面 z（末端 1.024 下 15mm）


def add_material(stage, prim, recipe, double_sided=False):
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


def override_tube_glass(stage, name):
    """试管要看清管内液体颜色：test_tube.usd 玻璃 opacity 0.35 偏雾，前壁压暗管内液体。
    场景层覆盖为更透明的真玻璃（opacity 0.04 + 近白 diffuse + 低粗糙度），液体颜色清晰透出。
    2026-08-27 用户「试管不够透明看不清液体颜色」→ opacity 0.12 → 0.04。"""
    shader = UsdShade.Shader(stage.GetPrimAtPath(f"/World/{name}/tube_mat/Shader"))
    if not shader.GetPrim().IsValid():
        print(f"[tube] /World/{name}/tube_mat/Shader not found, skip")
        return
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.04)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.92, 0.95, 1.0))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.03)
    print(f"[tube] {name} override glass -> opacity 0.04")


def add_gas_contents(stage):
    """产气试管内白粉 + 无色酸液（固定，不参与变色）。"""
    solid = UsdGeom.Cylinder.Define(stage, "/World/GasSolid")
    solid.CreateRadiusAttr(GAS_SOLID_R)
    solid.CreateHeightAttr(GAS_SOLID_H)
    solid.CreateAxisAttr("Z")
    solid.AddTranslateOp().Set(Gf.Vec3d(GAS_TUBE_XY[0], GAS_TUBE_XY[1], GAS_SOLID_Z + GAS_SOLID_H / 2))
    add_material(stage, solid.GetPrim(),
                 dict(color=(0.90, 0.90, 0.88), roughness=0.90))

    liquid = UsdGeom.Cylinder.Define(stage, "/World/GasLiquid")
    liquid.CreateRadiusAttr(GAS_LIQUID_R)
    liquid.CreateHeightAttr(GAS_LIQUID_H)
    liquid.CreateAxisAttr("Z")
    liquid.AddTranslateOp().Set(Gf.Vec3d(GAS_TUBE_XY[0], GAS_TUBE_XY[1], GAS_LIQUID_Z + GAS_LIQUID_H / 2))
    add_material(stage, liquid.GetPrim(),
                 dict(color=(0.86, 0.88, 0.90), opacity=0.85, roughness=0.15, ior=1.33))
    print("[content] GasSolid + GasLiquid in gas tube")


def add_detection_liquid(stage):
    """检验试管检测液 6 色变体（父 Xform 贴管底，task 让父跟随试管平移）。默认 colorless。"""
    parent = UsdGeom.Xform.Define(stage, "/World/TubeSolution")
    parent.AddTranslateOp().Set(Gf.Vec3d(TEST_TUBE_XY[0], TEST_TUBE_XY[1], TUBE_BOTTOM_Z))
    for key, color in LIQUID_COLORS.items():
        cyl = UsdGeom.Cylinder.Define(stage, f"/World/TubeSolution/liquid_{key}")
        cyl.CreateRadiusAttr(LIQUID_R)
        cyl.CreateHeightAttr(LIQUID_H)
        cyl.CreateAxisAttr("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, LIQUID_H / 2))
        add_material(stage, cyl.GetPrim(),
                     dict(color=color, opacity=LIQUID_OPACITY, roughness=LIQUID_ROUGH, ior=1.33))
        visible = (key == "colorless")
        UsdGeom.Imageable(cyl.GetPrim()).CreateVisibilityAttr().Set(
            "invisible" if not visible else "inherited")
    print("[effect] TubeSolution 6 liquid variants (default colorless visible)")


def add_bubbles(stage):
    """导气管末端气泡（初始隐藏；task 在 HoldDetect 时按队列上升）。"""
    parent = UsdGeom.Xform.Define(stage, "/World/GasBubbles")
    parent.AddTranslateOp().Set(Gf.Vec3d(DIP_XY[0], DIP_XY[1], 0))
    for i in range(BUBBLE_COUNT):
        sph = UsdGeom.Sphere.Define(stage, f"/World/GasBubbles/bubble_{i}")
        sph.CreateRadiusAttr(BUBBLE_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(0, 0, FREE_END_Z))
        add_material(stage, sph.GetPrim(),
                     dict(color=(0.85, 0.90, 0.95), opacity=0.50, roughness=0.05, ior=1.33))
    UsdGeom.Imageable(parent).CreateVisibilityAttr().Set("invisible")
    print(f"[effect] GasBubbles {BUBBLE_COUNT} bubbles (hidden initially)")


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


def remove_stand_fittings(st2):
    """铁架台 iron_stand.usd 自带水平支撑环 ring 与温度计挂钩 hook（B2 用），D7 不需要：
    环在 z≈0.914 会切穿固定其旁的产气试管，挂钩在高处无碰撞但冗余。场景层按 prim 名移除，
    只留底座 + 立柱，供试管夹固定产气试管。"""
    root = st2.GetPrimAtPath("/World/IronStand")
    if not root.IsValid():
        print("[clean] /World/IronStand not found, skip ring/hook removal")
        return
    remove = [pp.GetPath() for pp in Usd.PrimRange(root)
              if pp.GetName() in ("ring", "hook")]
    for path in remove:
        st2.RemovePrim(path)
        print(f"[clean] removed {path}")
    if not remove:
        print("[clean] no ring/hook under /World/IronStand")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for name in ["TestTubeRack", "IronStand", "TestTubeClamp", "GasTube", "TestTube",
                 "Stopper"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 不变量：产气试管底 0.90（试管夹抬高固定）/ 检验试管底 0.806（架孔底）、橡皮塞导气管末端 1.024
    # （下探段加长后末端低于塞底，Stopper 整体最低 z = 末端而非塞底 1.044）、铁架台底座贴台面 0.80
    for name, want_z in [("GasTube", GAS_TUBE_BOTTOM_Z), ("TestTube", TUBE_BOTTOM_Z),
                         ("Stopper", FREE_END_Z),
                         ("IronStand", TABLE_TOP)]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        assert abs(r.GetMin()[2] - want_z) < 1e-3, \
            f"{name} bottom z={r.GetMin()[2]:.4f} != {want_z}"
    # 试管夹夹住产气试管 + 夹环套住铁架台立柱（夹口 +X 夹管、夹环 -X 套立柱）
    clamp = st2.GetPrimAtPath("/World/TestTubeClamp")
    cr = bc.ComputeWorldBound(clamp).ComputeAlignedRange()
    assert cr.GetMin()[0] < STAND_XY[0] + 0.02, \
        f"clamp collar should reach pole (min x={cr.GetMin()[0]:.4f} vs pole {STAND_XY[0]:.4f})"
    assert cr.GetMax()[0] > GAS_TUBE_XY[0] + 0.008, \
        f"clamp jaw should wrap gas tube (max x={cr.GetMax()[0]:.4f} vs tube {GAS_TUBE_XY[0]:.4f})"
    # 铁架台水平支撑环 ring / 温度计挂钩 hook 已移除（D7 不需要）
    for name in ("ring", "hook"):
        assert not any(pp.GetName() == name
                       for pp in Usd.PrimRange(st2.GetPrimAtPath("/World/IronStand"))), \
            f"IronStand/{name} should be removed"
    # 导气管末端世界 x/y 应落在下浸点 (0.440,0.079)（ΔX=40mm ΔY=159mm 从塞中心 (0.40,-0.08)）
    stopper = st2.GetPrimAtPath("/World/Stopper")
    sr = bc.ComputeWorldBound(stopper).ComputeAlignedRange()
    assert abs(sr.GetMax()[0] - (STOPPER_PLUG_XY[0] + 0.043)) < 3e-3, \
        f"Stopper delivery-tube free end x={sr.GetMax()[0]:.4f} != {STOPPER_PLUG_XY[0] + 0.043}"
    assert abs(sr.GetMax()[1] - (STOPPER_PLUG_XY[1] + 0.162)) < 3e-3, \
        f"Stopper delivery-tube free end y={sr.GetMax()[1]:.4f} != {STOPPER_PLUG_XY[1] + 0.162}"
    # 默认可见性：检测液 colorless 可见、气泡隐藏
    liquid = st2.GetPrimAtPath("/World/TubeSolution/liquid_colorless")
    assert UsdGeom.Imageable(liquid).ComputeVisibility() == "inherited", \
        "TubeSolution/liquid_colorless should be visible by default"
    bubbles = st2.GetPrimAtPath("/World/GasBubbles")
    assert UsdGeom.Imageable(bubbles).ComputeVisibility() == "invisible", \
        "GasBubbles should be hidden initially"
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    # 试管玻璃覆盖为透明
    for name in ["GasTube", "TestTube"]:
        sh = UsdShade.Shader(st2.GetPrimAtPath(f"/World/{name}/tube_mat/Shader"))
        assert sh.GetInput("opacity").Get() < 0.2, f"{name} glass opacity should be overridden"
    print("[verify] gas tube 0.90 / test tube 0.806 / stopper tip 1.024 (pre-plugged) / "
          "stand 0.80 / clamp wraps tube + reaches pole / ring+hook removed / "
          "free-end xy ok / default visibility ok / no stray DomeLight / tube glass transparent")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    stage = Usd.Stage.Open(LAB_CLEAN)
    add_equip(stage, "TestTubeRack", "test_tube_rack.usd", (RACK_XY[0], RACK_XY[1], None), None)
    add_equip(stage, "IronStand", "iron_stand.usd", (STAND_XY[0], STAND_XY[1], None), None)
    add_equip(stage, "TestTubeClamp", "test_tube_clamp.usd",
              (CLAMP_XY[0], CLAMP_XY[1], CLAMP_Z), None)
    add_equip(stage, "GasTube", "test_tube.usd", (GAS_TUBE_XY[0], GAS_TUBE_XY[1], GAS_TUBE_BOTTOM_Z), None)
    override_tube_glass(stage, "GasTube")
    add_equip(stage, "TestTube", "test_tube.usd", (TEST_TUBE_XY[0], TEST_TUBE_XY[1], TUBE_BOTTOM_Z), None)
    override_tube_glass(stage, "TestTube")
    add_equip(stage, "Stopper", "rubber_stopper_delivery.usd",
              (STOPPER_PLUG_XY[0], STOPPER_PLUG_XY[1], STOPPER_PLUG_BOTTOM_Z), None)
    add_gas_contents(stage)
    add_detection_liquid(stage)
    add_bubbles(stage)
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stand_fittings(st2)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
