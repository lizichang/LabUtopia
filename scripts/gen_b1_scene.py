# -*- coding: utf-8 -*-
"""生成 b1_alcohol_heat_solid.usd —— B1 酒精灯加热（固体样品）场景（烘平自包含）。

用户 2026-08-27 方案（去掉试管夹，机械臂夹爪直接握试管）：
  「场景复刻 D2S：表面皿、粉末、机械臂坐标一定要复刻 D2S，这样粉末才能挖得准」
  → 表面皿/粉末/试管架/试管/药匙坐标逐字复制 gen_d2s_scene.py；
  「去掉试管夹，用机械臂的夹爪直接代替」→ 不摆 TestTubeClamp，加热时手臂持试管；
  「机械臂打开酒精灯帽，拿起火柴点燃（模仿焰色反应）」→ 灯帽初始盖在灯上（留给
  打开动作）、火柴头朝灯芯、火焰 flame_outer/flame_inner 初始隐藏（点火后 task reveal）。

布局（D2S 五件 + 酒精灯 + 火柴；台面顶 z=0.80，base=lab_clean 同 B2）：
  TestTubeRack  (0.6803, 0.3607)  工作区右侧（D2S 逐字）
  TestTube      (0.659,  0.241,  0.806)  架近侧左孔（D2S 逐字，口 z 0.9593）
  Spatula       (0.6993, 0.3608, 0.828, rotZ -180°)  架中心孔竖插（D2S 逐字）
  SurfaceDish   (0.5365, 0.105)   架正后方表面皿（D2S 逐字，粉末在皿上）
  SamplePowder  (0.5383, 0.0992, 0.7988, scale 0.4)  皿上粉丘（D2S 逐字）
  AlcoholLamp   (0.50,  0.0029)   台面前区中央（仿 B2 灯位 (0.5286,0.0029) 左移 3cm，
                  避开 D2S 皿 (0.5365,0.105) 挖粉区；2026-08-27 用户移灯 x=0.659 后
                  「位置全部调整回去」→ 回退 0.50；灯帽留在灯上，火焰初始隐藏）
  Match         (0.40, -0.06, 0.813)  灯前下方，火柴头(+X 端)朝灯芯（同 B2）

效果 prim（初始隐藏，task 动画驱动）：
  PowderOnSpoon  药匙尖粉末（挖粉后随药匙，D2S 同款）
  TubePowder     试管内白色粉末柱（⑬ 倒粉后显示；D2S TubeSample_white 同款）
  PowderDrop     药粉下落父 + 14 粉粒（⑬ 倒粉时错帧坠落，D2S/D3-S 同款）
  火焰 flame_outer/flame_inner（alcohol_lamp.usd 自带，点火后 reveal）

用法：python scripts/gen_b1_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b1_alcohol_heat_solid")
OUT = os.path.join(SCENE_DIR, "b1_alcohol_heat_solid.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— D2S 五件（逐字复刻 gen_d2s_scene.py EQUIP，保证挖粉坐标一致）——
RACK_XY = (0.6803, 0.3607)
TUBE = (0.659, 0.241, 0.806)
SPATULA_T = (0.6993, 0.3608, 0.828)
DISH_XY = (0.5365, 0.105)
POWDER_T = (0.5383, 0.0992, 0.7988)
POWDER_SCALE = 0.4

# —— 酒精灯 / 火柴（B1 新增）——
LAMP_XY = (0.50, 0.0029)          # 台面前区中央（仿 B2 灯位 (0.5286,0.0029)，左移 3cm 避 D2S 皿挖粉区；
                                  # 2026-08-27 用户移灯 x=0.659 后「位置全部调整回去」→ 回退 0.50）
MATCH_XY = (0.40, -0.06)          # 灯前下方（同 B2）；火柴头 = asset +X 端 → 朝灯芯 (0.50,0.0029)
MATCH_T = 0.813                   # 火柴原点 z：抬高 12mm 让手指离桌（同 B2，避免夹爪 collider 扎桌面）

# (prim, asset_file, translate, scale, rot_z)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (RACK_XY[0], RACK_XY[1], None), None, None),
    ("TestTube", "test_tube.usd", TUBE, None, None),
    ("Spatula", "spatula.usd", SPATULA_T, None, -180.0),
    ("SurfaceDish", "sample_dish.usd", (DISH_XY[0], DISH_XY[1], TABLE_TOP), None, None),
    ("SamplePowder", "powder.usd", POWDER_T, POWDER_SCALE, None),
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_XY[0], LAMP_XY[1], None), None, None),
    ("Match", "match.usd", (MATCH_XY[0], MATCH_XY[1], MATCH_T), None, None),
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
# PowderOnSpoon 在药匙尖端（spatula tip world z=0.828+0.135=0.963，xy 随药匙新坐标，D2S 同款）
BUILTIN = [
    ("PowderOnSpoon", 0.005, 0.005, (SPATULA_T[0], SPATULA_T[1], SPATULA_T[2] + 0.135),
     (0.93, 0.93, 0.94), 1.0),
]

# 试管内粉末柱（⑬ 倒粉后显示）：D2S TubeSample_white 同款（白色粉末，透明真玻璃下可见）
TUBE_POWDER_R = 0.004             # 2026-08-27 用户「粉末只舀了一勺不可能那么多」→ 缩小（8mm 直径）
TUBE_POWDER_H = 0.006             # 6mm 高（一勺粉量，不再 12mm）
TUBE_POWDER_CZ = 0.809            # 粉末柱中心 = 管底 0.806 + 3mm（坐管底，不再悬 34mm 高）
TUBE_POWDER_COLOR = (0.93, 0.93, 0.94)

# 药粉下落（task._step_powder_anim 驱动，D2S/D3-S 同款）：父 PowderDrop + N 颗粉粒。
# ⑬ 药匙回卷倒粉时粉粒从勺尖错帧坠落进试管（r=0.003 小粒成细粉流），落定后 show TubePowder。
POWDER_DROPS = 14                 # 粉粒数（连续细粉流观感）
POWDER_DROP_R = 0.003             # 粉粒半径（同 D2L 滴球 r=0.003）
POWDER_DROP_COLOR = (0.93, 0.93, 0.94)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot_z=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot_z is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, rot_z))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" scale {scale}" if scale else "") + (f" rotZ {rot_z}" if rot_z is not None else ""))


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, emissive=None,
                 double_sided=False):
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


def add_effects(stage):
    """内建效果 prim（全部初始隐藏，task 动画驱动）：
      PowderOnSpoon  药匙尖粉末（挖粉后随药匙）
      TubePowder     试管内白色粉末柱（⑬ 倒粉后显示）
    """
    for name, r, h, t, color, opacity in BUILTIN:
        geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} hidden at {t}")
    # 试管内粉末柱（TubePowder）：白色粉末，初始隐藏，⑬ 倒粉后 task reveal
    pw = UsdGeom.Cylinder.Define(stage, "/World/TubePowder")
    pw.CreateRadiusAttr(TUBE_POWDER_R)
    pw.CreateHeightAttr(TUBE_POWDER_H)
    pw.CreateAxisAttr("Z")
    pw.AddTranslateOp().Set(Gf.Vec3d(TUBE[0], TUBE[1], TUBE_POWDER_CZ))
    add_material(stage, pw.GetPrim(), TUBE_POWDER_COLOR, 1.0)
    UsdGeom.Imageable(pw).MakeInvisible()
    print(f"[effect] TubePowder hidden at ({TUBE[0]}, {TUBE[1]}, {TUBE_POWDER_CZ})")
    # 药粉下落：父 PowderDrop + N 颗粉粒（父+单粒全隐藏，task 下落动画逐颗驱动，D2S/D3-S 同款）
    drop = UsdGeom.Xform.Define(stage, "/World/PowderDrop")
    for i in range(POWDER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/PowderDrop/Drop_{i}")
        sph.CreateRadiusAttr(POWDER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(TUBE[0], TUBE[1], TUBE[2] + 0.1533))   # 管口 0.9593
        add_material(stage, sph.GetPrim(), POWDER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(drop).MakeInvisible()
    print(f"[effect] PowderDrop hidden ({POWDER_DROPS} powder grains)")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属药匙/玻璃试管在无环境反射下反黑不可见。
    贴图路径先用相对 ./textures/，烘平后由 fix_env_light() 在场景层重新指向。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃/金属件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（d2s 同款）：去试管玻璃反光，现象看得清。"""
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
    print("[light] CylinderLight has no translate op, skip")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效），
    烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def strip_dome_lights(st2):
    """扫除 flametest 残留的嵌套 DomeLight，只保留 /World/env_light
    （试管架等资产自带近黑 env_light 会把金属药匙压暗）。"""
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


def brighten_spatula(stage):
    """药匙 = 普通不锈钢（银黑）：metallic 1.0 + low roughness + 深灰 diffuse（d2s 同款）。"""
    sh = stage.GetPrimAtPath("/World/Spatula/material/stainless_steel")
    if not sh.IsValid() or sh.GetTypeName() != "Shader":
        print("[spatula] material not found, skip")
        return
    ush = UsdShade.Shader(sh)
    ush.GetInput("metallic").Set(1.0)
    ush.GetInput("roughness").Set(0.45)
    ush.GetInput("diffuseColor").Set(Gf.Vec3f(0.24, 0.24, 0.27))
    ush.GetInput("emissiveColor").Set(Gf.Vec3f(0.0, 0.0, 0.0))
    print("[spatula] stainless: metallic 1.0, roughness 0.45, diffuse 0.24, emissive 0")


def cleanup_dish(stage):
    """表面皿：去 flametest 残留 env_light，粉末子集重绑皿材质（d2s 同款）。"""
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
    """粉末收尾：离群废料/ env_light 防御性清理 + 纹理路径重定位（d2s 同款）。"""
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


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（d2s/d3l 同款，material:binding 取材质再找 shader）。"""
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


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：opacity 0.35→0.12、ior 1.5、roughness 0.05→0.25、
    doubleSided（d2s/d3l 同款，管内粉末柱透出来）。"""
    p = st2.GetPrimAtPath("/World/TestTube")
    if not p.IsValid():
        print("[mat] /World/TestTube not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.5, "roughness": 0.25}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] tube glass {c.GetPath()} -> op 0.12 / ior 1.5 / rough 0.25 / doubleSided")


def hide_flames(st2):
    """火焰初始隐藏（实验未点火）：flame_outer/flame_inner 两个 Cone。"""
    for fl in ("flame_outer", "flame_inner"):
        p = st2.GetPrimAtPath(f"/World/AlcoholLamp/{fl}")
        if not p.IsValid():
            print(f"[clean] /World/AlcoholLamp/{fl} not found, skip")
            continue
        UsdGeom.Imageable(p).MakeInvisible()
        print(f"[clean] hidden {p.GetPath()}")


def verify(st2):
    """自检：D2S 五件坐标与 D2S 场景一致（挖粉准）+ 酒精灯贴台 + 灯帽在灯上 +
    火柴抬高 12mm + 火焰隐藏 + 无残留 DomeLight。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "Spatula", "SurfaceDish", "SamplePowder",
             "AlcoholLamp", "Match"]
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

    # 试管架贴台面（0.80）
    rmn, rmx = boxes["TestTubeRack"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"rack bottom {rmn[2]} not on table"
    # 试管：底在孔底 0.806、口 z≈0.9593（D2S 实测）
    tmn, tmx = boxes["TestTube"]
    assert abs(tmn[2] - 0.806) < 0.002, f"tube bottom {tmn[2]} != 0.806"
    assert abs(tmx[2] - 0.9593) < 0.005, f"tube mouth {tmx[2]} != 0.9593"
    # 药匙：尖在 0.963（D2S 实测），中心落在架中心孔
    spn, spx = boxes["Spatula"]
    assert abs(spx[2] - 0.963) < 0.005, f"spatula tip {spx[2]} != 0.963"
    assert abs((spn[0] + spx[0]) / 2 - SPATULA_T[0]) < 0.003, f"spatula x center off {SPATULA_T[0]}"
    # 表面皿贴台面、粉末在皿内（粉底 > 皿底、xy 落在皿口径内——浅碗粉坐碗底，rim 高于粉）
    dsn, dsx = boxes["SurfaceDish"]
    assert abs(dsn[2] - TABLE_TOP) < 0.002, f"dish bottom {dsn[2]} not on table"
    pwn, pwx = boxes["SamplePowder"]
    assert pwn[2] >= dsn[2] - 0.001, f"powder bottom {pwn[2]} below dish floor {dsn[2]}"
    assert pwn[0] >= dsn[0] - 0.005 and pwx[0] <= dsx[0] + 0.005, f"powder x outside dish"
    assert pwn[1] >= dsn[1] - 0.005 and pwx[1] <= dsx[1] + 0.005, f"powder y outside dish"
    # 酒精灯贴台面、灯帽在灯上（cap 存在且贴灯顶）
    lmn, lmx = boxes["AlcoholLamp"]
    assert abs(lmn[2] - TABLE_TOP) < 0.002, f"lamp bottom {lmn[2]} not on table"
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    assert cap.IsValid(), "lamp cap missing (should be on lamp for open-cap step)"
    cr = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = cr.GetMin(), cr.GetMax()
    print(f"[verify] cap     min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert cmx[2] >= lmx[2] - 0.03, f"cap bottom {cmn[2]} not on lamp top {lmx[2]}"
    # 火柴：抬高 12mm 以上（底 > 0.81）
    mn, mx = boxes["Match"]
    assert mn[2] > TABLE_TOP + 0.010, f"match bottom {mn[2]} not raised 12mm"
    # 火焰初始隐藏
    for fl in ("flame_outer", "flame_inner"):
        p = st2.GetPrimAtPath(f"/World/AlcoholLamp/{fl}")
        assert p.IsValid(), f"{fl} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", f"{fl} should be hidden"
    # 无残留 DomeLight（只留 /World/env_light）
    domes = [str(p.GetPath()) for p in Usd.PrimRange(st2.GetPseudoRoot())
             if p.IsA(UsdLux.DomeLight) and str(p.GetPath()) != "/World/env_light"]
    assert not domes, f"stray DomeLight: {domes}"
    # 效果 prim 存在且隐藏
    for fx in ("PowderOnSpoon", "TubePowder"):
        p = st2.GetPrimAtPath(f"/World/{fx}")
        assert p.IsValid(), f"{fx} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", f"{fx} should be hidden"
    # PowderDrop 父 + 14 粉粒全存在且隐藏
    dp = st2.GetPrimAtPath("/World/PowderDrop")
    assert dp.IsValid(), "PowderDrop missing"
    assert UsdGeom.Imageable(dp).ComputeVisibility() == "invisible", "PowderDrop should be hidden"
    for i in range(POWDER_DROPS):
        s = st2.GetPrimAtPath(f"/World/PowderDrop/Drop_{i}")
        assert s.IsValid(), f"PowderDrop/Drop_{i} missing"
        assert UsdGeom.Imageable(s).ComputeVisibility() == "invisible", f"Drop_{i} should be hidden"
    print("[verify] OK: D2S 五件坐标一致(架贴台/管底0.806/口0.9593/药匙尖0.963/皿贴台/粉在皿上) "
          f"+ 灯贴台/灯帽在灯上 + 火柴抬高12mm + 火焰隐藏 + 无残留DomeLight + 效果prim隐藏")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot_z in EQUIP:
        add_equip(stage, name, asset, t, scale, rot_z)
    add_effects(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    cleanup_dish(st2)        # 表面皿去 env_light + 粉末子集重绑皿材质
    powder(st2)              # 粉末：防御性清理 + 纹理重定位
    strip_dome_lights(st2)   # 扫除试管架等残留 DomeLight
    brighten_spatula(st2)    # 药匙 = 银黑不锈钢
    fix_tube_material(st2)   # 试管玻璃去反光（管内粉末看得清）
    hide_flames(st2)         # 火焰初始隐藏（灯帽保留在灯上，留给打开动作）
    brighten_lights(st2)     # 主光 2000→12000
    set_cylinder_light_x(st2, x=-10.0)   # 主光挪远侧（去试管反光）
    fix_env_light(st2)       # env 贴图路径断链 → 场景目录
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
