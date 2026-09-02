# -*- coding: utf-8 -*-
"""生成 d9_oxygen_splint.usd —— D9 氧气检验场景（带火星木条复燃）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 酒精灯 alcohol_lamp.usd @ (0.5286,0.0029)（复用 B2 位 + rot180，**灯帽留在灯上**——摘帽
  由 CapOffPass 运行时做，非 gen 期挪到桌边）
- 火柴 match.usd @ (0.40,-0.06,0.813)（复用 B2，头 +X 朝灯芯）
- 木条 wood_splint.usd @ (0.27,0.25,0.813)（新资产 Ø6mm×150mm，握持端原点，点燃端 +X）
- 氧气试管 test_tube.usd 竖立插**试管架孔**（右列中排），口朝上；氧气预先收集、无色、无液柱。
  不摆铁架台/试管夹——铁架台立柱恒在试管 -X 侧 10cm，木条横夹（尖端 +X）悬停管口时
  必横穿立柱（穿模），试管架低矮无立柱，木条从 -X 侧横越架顶无阻挡。
- 火焰 /World 顶层（RTX 铁律：火焰必须顶层 prim，不能烘焙进被引用资产 Xform 下）：
    flame_outer / flame_inner    酒精灯水滴形火焰（外蓝/内黄，default-visible，task reset 熄）
    SplintFlame                  木条端复燃火焰（小水滴形黄焰，default-visible）
    SplintEmber                  木条端余烬火星（红点 Sphere r4mm，default-visible）
  —— 全部 default-visible（熄灭由 task reset() _set_visible(False) 负责；bake invisible
     后翻 visible 会渲染成灰，见 B2/flametest 铁律）。

布局（世界坐标，米，Z-up；机械臂底座 [-0.15,0.05,0.71] 复用 B2，故点火/摘帽坐标照搬 B2）：
  AlcoholLamp (0.5286,0.0029)         酒精灯（灯芯 0.9005，火焰底 0.891；帽在灯上）
  Match        (0.40,-0.06,0.813)     火柴（头 +X 朝灯芯）
  WoodSplint   (0.27,0.25,0.813)      木条（握持端原点，点燃端 +X 到 0.42）
  TestTubeRack (0.50,0.35)            试管架（B2 架位）
  OxygenTube   (0.519,0.389)          氧气试管（右列中排，竖立，底 0.806，口 0.959）

用法：python scripts/gen_d9_oxygen_splint_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d9_oxygen_splint")
OUT = os.path.join(SCENE_DIR, "d9_oxygen_splint.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 酒精灯（复用 B2 位 + rot180；灯帽留在灯上，摘帽由 CapOffPass 运行时做）——
LAMP_X, LAMP_Y = 0.5286, 0.0029
WICK_Z = 0.9005                        # 灯芯顶（点火触发点）
FLAME_BASE_Z = TABLE_TOP + 0.091       # 灯芯根部世界 z（火焰底 = holder 顶 0.891，同 B2）
FLAME_APEX_Z = 0.936                   # 自由火焰 apex（无石棉网，~4.5cm 火焰）
FLAME_OUTER_R = 0.009                  # 外焰肚半径
FLAME_INNER_R = 0.005                  # 内焰肚半径
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022

# —— 火柴（复用 B2：头 +X 朝灯芯，抬高 12mm 防手指 collider 扎桌）——
MATCH_X, MATCH_Y, MATCH_T = 0.40, -0.06, 0.813

# —— 木条（新资产 wood_splint.usd：Ø6mm×150mm，握持端原点、点燃端 +X，局部 bbox x[0,0.15]）——
# x=0.27（原 0.30）——放回竖直下降 +X 抖动致点燃端穿试管架（架 min x=0.4573），整体 -X 退 30mm。
SPLINT_X, SPLINT_Y, SPLINT_Z = 0.27, 0.25, 0.813
SPLINT_LEN = 0.150
SPLINT_TIP = (SPLINT_X + SPLINT_LEN, SPLINT_Y, SPLINT_Z)   # 点燃端世界坐标 (0.42,0.25,0.813)

# —— 余烬/复燃效果 prim ——
EMBER_N = 10                          # 余烬火星点数量（散布炭黑区）
SPLINT_FLAME_R = 0.004                # 木条端复燃火焰肚半径
SPLINT_FLAME_H = 0.025                # 复燃火焰高度（尖 0.838）

# —— 氧气试管（竖立插试管架孔，口朝上；氧气预收集、无色；无铁架台避木条横越立柱穿模）——
# 试管架 2列×7行 Ø22.4 孔（skill 坑33）：列 x=±0.019；行 y=+0.116(最远)/+0.079/+0.039/0.000/
# -0.040/-0.080/-0.119(最近)。右列（+0.019）中排（+0.039）插氧气试管，底 0.806，口 0.959。
RACK_XY = (0.50, 0.35)                # 试管架（B2 架位）
RACK_Z = TABLE_TOP + 0.0965           # 架原点 z 0.8965（asset 底座 min z=-0.0965 → tz 贴台）
HOLE_Z = RACK_Z - 0.0905              # 孔底 0.806（底层板顶）
OXY_TUBE_XY = (RACK_XY[0] + 0.019, RACK_XY[1] + 0.039)   # (0.519,0.389) 右列中排
OXY_TUBE_BOTTOM_Z = HOLE_Z            # 0.806
OXY_TUBE_MOUTH_Z = OXY_TUBE_BOTTOM_Z + 0.1533           # 0.959（test_tube 高 0.1533）

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面；rot180 → 绕 Z 旋 180°
EQUIP = [
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_X, LAMP_Y, None), None, True),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, MATCH_T), None, False),
    ("WoodSplint", "wood_splint.usd", (SPLINT_X, SPLINT_Y, SPLINT_Z), None, False),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_XY[0], RACK_XY[1], None), None, False),
    ("OxygenTube", "test_tube.usd", (OXY_TUBE_XY[0], OXY_TUBE_XY[1], OXY_TUBE_BOTTOM_Z), None, False),
]


def asset_local_min_z(asset_file):
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot=False):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(os.path.abspath(os.path.join(EQ, asset)))
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx},{ty},{tz})"
          + (" rot180" if rot else "") + (f" scale {scale}" if scale else ""))


def override_tube_glass(stage, name):
    """氧气试管玻璃透明化（test_tube.usd 默认 opacity 0.35 偏雾 → 0.04 真玻璃，火星透过看得清）。"""
    shader = UsdShade.Shader(stage.GetPrimAtPath(f"/World/{name}/tube_mat/Shader"))
    if not shader.GetPrim().IsValid():
        print(f"[tube] /World/{name}/tube_mat/Shader not found, skip")
        return
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.04)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.92, 0.95, 1.0))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.03)
    print(f"[tube] {name} override glass -> opacity 0.04")


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


def add_droplet_flame(st2, x, y, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere（圆底） + 上部 Cone（收尖），一组两 prim，绕 Z 轴。
    球心在 z_b+r、cone 底在球心处收尖到 z_a。材质近黑 diffuse + HDR 单通道 emissive，
    默认可见（熄灭由 task reset() _set_visible(False) 负责）。"""
    zc = z_b + r
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(x, y, zc))
    h = z_a - zc
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(x, y, zc + h / 2))
    for prim in (sph, cone):
        pname = prim.GetPath().name
        UsdGeom.Imageable(prim).GetVisibilityAttr().Clear()
        mat = UsdShade.Material.Define(st2, f"/World/{pname}_mat")
        sh = UsdShade.Shader.Define(st2, f"/World/{pname}_mat/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.01, 0.01, 0.01))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(prim).Bind(mat)
    print(f"[flame] droplet {name}: sphere r{r} c{zc:.4f} (bottom {z_b:.4f}) + cone apex {z_a:.4f}")


def rebuild_flames(st2):
    """酒精灯火焰迁 /World 顶层（灯下引用子 prim RTX 不渲染）。外焰偏蓝 / 内焰偏黄。"""
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, LAMP_X, LAMP_Y, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z,
                      FLAME_APEX_Z, (0.35, 0.55, 2.40))            # 外焰偏蓝（B 主导）
    add_droplet_flame(st2, LAMP_X, LAMP_Y, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z,
                      FLAME_INNER_APEX_Z, (2.80, 0.55, 0.20))      # 内焰偏黄（R 主导）
    print(f"[flame] lamp flames droplet: outer base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"inner apex {FLAME_INNER_APEX_Z:.4f}")


def _bind_glow_mat(st2, prim, emissive, diffuse=(0.01, 0.01, 0.01), roughness=0.3):
    """给 prim 绑定近黑 diffuse + emissive 发光材质（单通道主导，RTX 亮场景显色配方）。"""
    pname = prim.GetPath().name
    mat = UsdShade.Material.Define(st2, f"/World/{pname}_mat")
    sh = UsdShade.Shader.Define(st2, f"/World/{pname}_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_splint_effects(st2):
    """木条端效果 prim（/World 顶层，default-visible，task 每帧钉到木条端 + 闪烁）：
    - SplintChar      炭黑区：黑圆柱包住木条端 30mm（轴 +X，近黑 diffuse + 微暗红 emissive）
    - SplintEmber_0..EMBER_N-1  余烬火星：多个暗红发光小点散布炭黑区（亮度错落）
    - SplintFlame     复燃火焰（小水滴黄焰，task 逐帧抖位置模拟火焰跳动）
    全部 default-visible（熄灭由 task reset() _set_visible(False) 负责）。"""
    import math
    CHAR_LEN = 0.030
    # 炭黑区（木条末端已变细 r2.2mm → 炭黑 r2.4mm 略粗包住变细端、可见，且比主体 r3mm 细一圈；轴 X 圆柱）
    char = UsdGeom.Cylinder.Define(st2, "/World/SplintChar")
    char.CreateRadiusAttr(0.0024)
    char.GetHeightAttr().Set(CHAR_LEN)
    char.CreateAxisAttr("X")
    UsdGeom.Xformable(char).AddTranslateOp().Set(
        Gf.Vec3d(SPLINT_TIP[0] - CHAR_LEN / 2, SPLINT_TIP[1], SPLINT_TIP[2]))
    UsdGeom.Imageable(char).GetVisibilityAttr().Clear()
    _bind_glow_mat(st2, char, (0.15, 0.04, 0.03), diffuse=(0.02, 0.02, 0.02), roughness=0.9)
    # 余烬火星：多个暗红发光小点（近黑 diffuse + 暗红单通道主导 emissive，亮度错落）
    for i in range(EMBER_N):
        frac = (i + 0.5) / EMBER_N
        dx = -frac * 0.028                       # x 从木条端往回 -2.8cm
        ang = i * 2.39996                        # 黄金角散布（确定性，无随机）
        rad = 0.0020 + 0.0004 * ((i * 37) % 3)   # 偏心 2.0..2.8mm（贴炭壳表面 r2.4，冒出即火星）
        dot_r = 0.0006 + 0.0004 * ((i * 13) % 3)  # 点半径 0.6..1.4mm
        bright = 0.7 + 0.9 * ((i * 7) % 4) / 3.0  # 亮度 0.7..1.6
        px = SPLINT_TIP[0] + dx
        py = SPLINT_TIP[1] + rad * math.cos(ang)
        pz = SPLINT_TIP[2] + rad * math.sin(ang)
        sph = UsdGeom.Sphere.Define(st2, f"/World/SplintEmber_{i}")
        sph.CreateRadiusAttr(dot_r)
        UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
        UsdGeom.Imageable(sph).GetVisibilityAttr().Clear()
        _bind_glow_mat(st2, sph,
                       (bright, 0.10 + 0.18 * bright / 1.6, 0.06 + 0.10 * bright / 1.6))
    # 复燃火焰（小水滴黄焰）
    add_droplet_flame(st2, SPLINT_TIP[0], SPLINT_TIP[1], "SplintFlame", SPLINT_FLAME_R,
                      SPLINT_TIP[2], SPLINT_TIP[2] + SPLINT_FLAME_H, (2.80, 0.55, 0.20))
    print(f"[effect] SplintChar r2.4x{CHAR_LEN:.3f} + {EMBER_N} ember dots + "
          f"SplintFlame at tip {SPLINT_TIP}")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    boxes = {}
    for name in ["AlcoholLamp", "Match", "WoodSplint", "TestTubeRack", "OxygenTube"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        boxes[name] = (r.GetMin(), r.GetMax())
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 不变量：酒精灯贴台 0.80（帽在灯上）/ 火柴抬高 / 木条平放（x 跨度 0.15）/
    # 氧气试管底 0.806 口 0.959（架孔底）/ 试管架贴台
    lamp = boxes["AlcoholLamp"]
    assert abs(lamp[0][2] - TABLE_TOP) < 0.002, f"lamp bottom {lamp[0][2]} not on table"
    assert boxes["Match"][0][2] > TABLE_TOP + 0.010, "match should be raised above table"
    spl = boxes["WoodSplint"]
    assert abs(spl[1][0] - spl[0][0] - SPLINT_LEN) < 0.002, f"splint x span {spl[1][0]-spl[0][0]} != {SPLINT_LEN}"
    assert abs((spl[0][1] + spl[1][1]) / 2 - SPLINT_Y) < 0.003, "splint y center off"
    tube = boxes["OxygenTube"]
    assert abs(tube[0][2] - OXY_TUBE_BOTTOM_Z) < 0.002, f"oxygen tube bottom {tube[0][2]} != {OXY_TUBE_BOTTOM_Z}"
    assert abs(tube[1][2] - OXY_TUBE_MOUTH_Z) < 0.003, f"oxygen tube mouth {tube[1][2]} != {OXY_TUBE_MOUTH_Z}"
    assert abs(boxes["TestTubeRack"][0][2] - TABLE_TOP) < 0.002, "rack bottom not on table"
    # 氧气试管在架孔内（x/y 对齐右列中排孔心 ±5mm）
    assert abs((tube[0][0] + tube[1][0]) / 2 - OXY_TUBE_XY[0]) < 0.005, "oxygen tube x off hole"
    assert abs((tube[0][1] + tube[1][1]) / 2 - OXY_TUBE_XY[1]) < 0.005, "oxygen tube y off hole"
    # 火焰/余烬迁 /World 顶层、默认可见
    for name in ("flame_outer", "flame_inner"):
        f = st2.GetPrimAtPath(f"/World/{name}")
        assert f.IsValid() and f.GetTypeName() == "Cone", f"{name} top-level cone missing"
        assert UsdGeom.Imageable(f).ComputeVisibility() != "invisible", f"{name} should be default visible"
        sph = st2.GetPrimAtPath(f"/World/{name}_sphere")
        assert sph.IsValid() and sph.GetTypeName() == "Sphere", f"{name}_sphere missing"
        assert UsdGeom.Imageable(sph).ComputeVisibility() != "invisible", f"{name}_sphere default visible"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"
    # 灯帽留在灯上（gen 期不移到桌边，摘帽由 CapOffPass 运行时做）
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    assert cap.IsValid(), "lamp cap missing"
    capr = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    assert capr.GetMin()[2] > TABLE_TOP + 0.05, f"cap should be on lamp, bottom {capr.GetMin()[2]:.4f}"
    # 木条端效果 prim
    ch = st2.GetPrimAtPath("/World/SplintChar")
    assert ch.IsValid() and ch.GetTypeName() == "Cylinder", "SplintChar missing"
    assert UsdGeom.Imageable(ch).ComputeVisibility() != "invisible", "SplintChar default visible"
    for i in range(EMBER_N):
        em = st2.GetPrimAtPath(f"/World/SplintEmber_{i}")
        assert em.IsValid() and em.GetTypeName() == "Sphere", f"SplintEmber_{i} missing"
        assert UsdGeom.Imageable(em).ComputeVisibility() != "invisible", \
            f"SplintEmber_{i} default visible"
    sf = st2.GetPrimAtPath("/World/SplintFlame")
    assert sf.IsValid() and sf.GetTypeName() == "Cone", "SplintFlame missing"
    assert UsdGeom.Imageable(sf).ComputeVisibility() != "invisible", "SplintFlame default visible"
    # 无 stray DomeLight
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    # 氧气试管玻璃透明化
    sh = UsdShade.Shader(st2.GetPrimAtPath("/World/OxygenTube/tube_mat/Shader"))
    assert sh.GetInput("opacity").Get() < 0.2, "oxygen tube glass opacity should be overridden"
    print("[verify] lamp 0.80 (cap on) / match raised / splint 150mm flat / "
          "oxygen tube 0.806→0.959 in rack hole / rack 0.80 / flames+ember top-level "
          "default-visible / no stray DomeLight / tube glass transparent")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot in EQUIP:
        add_equip(stage, name, asset, t, scale, rot)
    override_tube_glass(stage, "OxygenTube")
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stray_env_lights(st2)
    rebuild_flames(st2)
    add_splint_effects(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
