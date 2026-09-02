# -*- coding: utf-8 -*-
"""生成 c3_combustion_solid.usd —— C3 燃烧试验（固体样品）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）。

C3 燃烧试验（固体）：药匙挖固体粉末 → 倒入燃烧匙碗 → 点燃酒精灯 → 燃烧匙碗伸入火焰
→ 观察燃烧现象。器材（用户 2026-08-31 定稿）：
- 试管架（test_tube_rack.usd）：药匙的家（竖插中心孔，同 d2s）。
- 药匙（spatula.usd）：竖插试管架中心孔（rotZ -180°，勺头扁平面沿 X），挖粉倒粉。
- 燃烧匙（combustion_spoon.usd，碗朝上 + 长把手上竖）：碗贴台面、把手竖直立在
  试管架旁边（用户定稿位置，机械臂横夹把手薄边，碗伸入火焰）。
- 表面皿 + 粉堆（sample_dish.usd + powder.usd）：药匙挖粉源。
- 酒精灯（alcohol_lamp.usd）+ 火柴（match.usd）：点火加热；灯帽摘旁备灭火。
- 火焰：迁到 /World 顶层（灯下引用子 prim RTX 不渲染，同 b2/b5），初始隐藏
  （C3 酒精灯开始时未点燃，task 点火后显示 flame_outer/flame_inner）。

用法：python scripts/gen_c3_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil

from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "c_flame", "c3_combustion_solid")
OUT = os.path.join(SCENE_DIR, "c3_combustion_solid.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- 布局（机器人底座 [-0.15, 0.05, 0.71]，与 d2s 逐字一致；最远器材=试管架药匙 ~0.84m）----
# 试管架 + 药匙：后右，药匙的家（竖插中心孔，照 d2s 逐字：架 (0.6803,0.3607)、
# 药匙 (0.6993,0.3608,0.828) rotZ -180°）。用户 2026-09-01 要求与 d2s 保持一致（改前自定义坐标致臂卡死）。
RACK_X, RACK_Y = 0.6803, 0.3607
SPATULA_T = (0.6993, 0.3608, 0.828)
SPATULA_ROT = (0, 0, -180)               # d2s 同款：勺头扁平面沿 X，为后续机械臂旋转铺路

# 燃烧匙：竖立靠在试管架旁边（随架移到 d2s 位，保持相对架不变：架 -0.0397/-0.0593 同步）。
# 资产碗局部 z[-0.0102,0] 半球朝 +Z、把手 z[-0.0004,0.29] 竖直向上（长把手 29cm 上竖），
# 原厂即"碗朝上把手竖直"，无需旋转。碗心 (0.596,0.250) 落在试管架左前侧（架左缘 x≈0.656）。
SPOON_X, SPOON_Y = 0.596, 0.250         # 用户定稿（随架移）：碗心在架左前侧，把手竖直立起
SPOON_TZ = 0.8068                        # 用户定稿 T.z（碗缘 z=0.8068，碗底圆弧略入台面 3mm）
SPOON_ROT = None                         # 原厂朝向已竖直，不加旋转

# 酒精灯：台面中偏右、从底座 y=0.05 线挪离 +x/+y（2026-09-01 用户：灯/帽/火柴离机械臂太近，
# 挪离 +x/+y 且不碰现有器材）。灯体 bbox x[0.3564,0.4436] y[0.1364,0.2236]（verify 2026-09-01）
# 东缘 0.4436 距表面皿西缘 0.5065 留 6.3cm。火焰迁 /World 顶层（初始隐藏）。
LAMP_X, LAMP_Y = 0.30, 0.38
FLAME_BASE_Z = 0.900                     # 火焰底 = 灯芯顶（wick_top z=0.901），对齐 C1（0.900-0.936）
FLAME_APEX_Z = 0.936                     # 火焰尖（C1 同款）
FLAME_OUTER_R = 0.009
FLAME_INNER_R = 0.005
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022

# 样品燃烧火焰（combustible：碗内粉末点燃，焰色=flame_color 输入；pivot=火焰底=粉末顶，
# task 运行时写 translate 跟随粉末顶 + flicker；初始隐藏）。grp 包装（照 C4）。
SAMPLE_FLAME_R = 0.005
SAMPLE_FLAME_APEX_DZ = 0.028          # 火焰高（粉末顶上方 ~28mm，照 C4 SPOON_FLAME_APEX_DZ）
SAMPLE_FLAME_EMISSIVE = (1.00, 0.60, 0.15)   # 默认橙（task 运行时按 flame_color 覆盖 emissive）

# 表面皿 + 粉堆：d2s 逐字位置（挖粉源，横移路径不经过灯）。
DISH_T = (0.5365, 0.105, TABLE_TOP)
POWDER_T = (DISH_T[0] + 0.0018, DISH_T[1] - 0.0058, TABLE_TOP - 0.0012)  # → (0.5383,0.0992,0.7988) = d2s
POWDER_SCALE = 0.4

# 火柴：酒精灯 +X 侧（点火），抬高 13mm，头朝灯芯。随灯挪离底座线 +y（原 y=0.05 → 0.18）：
# 杆 x[0.5400,0.6378] 距灯体东缘 0.4436 留 9.6cm、距架 y≥0.2179 留 3.5cm、距表面皿 y≤0.135 留 4.2cm。
MATCH_T = (0.44, 0.38, TABLE_TOP + 0.013)

# 灯帽：摘下放灯正北 12cm 台面，帽底贴台面（原 +X 侧 0.47 会被火柴 0.54 顶到，改 +Y 侧；
# 新位置 (0.40,0.30)，远离所有器材）。rotZ180 把 cap 局部 y 取反：局部 -0.12 → 世界 +0.12。
CAP_DETACH = (0.0, -0.12, -0.0762)

# (prim, asset, translate, scale, rotxyz)  tz=None → 动态贴台面；rotxyz 角度
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, None),
    ("CombustionSpoon", "combustion_spoon.usd", (SPOON_X, SPOON_Y, SPOON_TZ), None, SPOON_ROT),
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_X, LAMP_Y, None), None, (0, 0, 180)),
    ("SurfaceDish", "sample_dish.usd", DISH_T, None, None),
    ("SamplePowder", "powder.usd", POWDER_T, POWDER_SCALE, None),
    ("Spatula", "spatula.usd", SPATULA_T, None, SPATULA_ROT),
    ("Match", "match.usd", MATCH_T, None, None),
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
# PowderOnSpoon 在药匙尖端（spatula tip world z=0.828+0.137=0.965，xy 随药匙新坐标 d2s），
# 初始隐藏，task 挖粉（⑨ 法兰 -45°→-90°）触发后显示、逐帧跟随勺尖（同 d2s）。
# PowderInBowl 在燃烧匙碗内（碗口 z=0.8068、碗底≈0.797，6mm 粉堆沉碗心 z≈0.802），初始
# 隐藏，⑭ 倒粉粉粒全部落定后显示（碗内样品）。
BUILTIN = [
    ("PowderOnSpoon", 0.005, 0.005, (SPATULA_T[0], SPATULA_T[1], 0.965),
     (0.93, 0.93, 0.94), 1.0),
    ("PowderInBowl", 0.012, 0.006, (SPOON_X, SPOON_Y, SPOON_TZ - 0.005),
     (0.93, 0.93, 0.94), 1.0),
]

# 下落粉粒（仿 d2s PowderDrop）：父 PowderDrop + N 颗小粉粒，⑭ 倒粉触发后 task 逐颗驱动，
# 从勺尖错帧（delay stagger）坠入燃烧匙碗内落定。home 位放碗心，初始全隐藏。
POWDER_DROPS = 14            # 粉粒数（连续细粉流观感，同 d2s）
POWDER_DROP_R = 0.003        # 粉粒半径（同 d2s）
POWDER_DROP_COLOR = (0.93, 0.93, 0.94)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    # 坑：Blender 导出资产几何写在 frame0 timeSamples（无 default），Default() 读空；
    # 用 TimeCode(0.0) 同时兼容 default-only 与 frame0-only（本项目资产都有 0.0 样本）。
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale=None, rotxyz=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(os.path.abspath(os.path.join(EQ, asset)))
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rotxyz is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(*rotxyz))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" rot{rotxyz}" if rotxyz else "") + (f" scale {scale}" if scale else ""))


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
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
    """内建效果 prim（PowderOnSpoon / PowderInBowl / PowderDrop 粉粒）：初始隐藏，task 驱动。
    PowderOnSpoon 挖粉后跟随勺尖；⑭ 倒粉触发 PowderDrop 粉粒从勺尖错帧坠入燃烧匙碗，
    落定后 PowderOnSpoon 隐藏、PowderInBowl 显示（碗内样品）。"""
    for name, r, h, t, color, opacity in BUILTIN:
        geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} hidden at {t}")
    # 下落粉粒：父 PowderDrop + N 颗小粉粒（全隐藏，task 下落动画逐颗驱动、错帧起落）
    drop = UsdGeom.Xform.Define(stage, "/World/PowderDrop")
    for i in range(POWDER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/PowderDrop/Drop_{i}")
        sph.CreateRadiusAttr(POWDER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(SPOON_X, SPOON_Y, SPOON_TZ))
        add_material(stage, sph.GetPrim(), POWDER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    drop.MakeInvisible()
    print(f"[effect] PowderDrop hidden ({POWDER_DROPS} grains)")


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


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight translate.x 移到 -10（lab_clean 默认 x=2.1 的巨型灯压爆 RTX 曝光）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    for op in UsdGeom.Xformable(cyl).GetOrderedXformOps():
        if op.GetOpName() != "xformOp:translate":
            continue
        v = op.Get()
        op.Set(Gf.Vec3d(x, v[1], v[2]))
        print(f"[light] CylinderLight translate -> ({x}, {v[1]:.3f}, {v[2]:.3f})")
        return
    print("[light] CylinderLight has no translate op, skip")


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


def remove_asset_env_lights(st2):
    for name, *_ in EQUIP:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")


def detach_lamp_cap(st2):
    """灯帽从灯顶摘下，放灯正北 12cm 台面（同 b5 换算，R180 后 cap 局部 x/y 取反）。"""
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[cap] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(*CAP_DETACH)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[cap] translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[cap] (no translate op) add translate {tuple(tgt)}")


def add_droplet_flame_grp(st2, name, r, z_b, z_a, emissive, hidden=True, x=0.0, y=0.0):
    """水滴形火焰组 /World/<name>_grp：pivot=火焰底 (x,y,z_b)。

    组 op 序 translate→rotateXYZ→scale（点先 scale 后 rotate 再 translate = 绕组原点
    即火焰底缩放/侧摆，不漂移）。task 每帧动画组 scale(高/宽 flicker)+rotateXYZ(侧摆)；
    样品火焰另写 translate 跟随粉末顶。球心=底+r，锥从球顶到 apex（照 C4）。"""
    grp = UsdGeom.Xform.Define(st2, f"/World/{name}_grp")
    grp.AddTranslateOp().Set(Gf.Vec3d(x, y, z_b))
    grp.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    grp.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    zc = r                              # 球心 = 底 + r（局部，底为组原点）
    h = (z_a - z_b) - r                 # 锥高（底球顶 → apex）
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_grp/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, zc))
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}_grp/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, zc + h / 2.0))
    for prim in (sph, cone):
        pname = prim.GetPath().name
        mat = UsdShade.Material.Define(st2, f"/World/{name}_grp/{pname}_mat")
        sh = UsdShade.Shader.Define(st2, f"/World/{name}_grp/{pname}_mat/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.01, 0.01, 0.01))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(prim).Bind(mat)
        if hidden:
            UsdGeom.Imageable(prim).MakeInvisible()
    print(f"[flame] grp {name}: pivot({x},{y},{z_b:.4f}) r{r} apex {z_a:.4f}"
          + (" (hidden)" if hidden else ""))
    return grp


def add_sample_flame(st):
    """样品燃烧火焰（combustible：碗内粉末点燃，焰色=flame_color 输入）。pivot=火焰底=粉末顶，
    task 每帧写 translate 跟随粉末顶 + scale/rotate flicker。初始隐藏，dwell 点燃后 task reveal。"""
    add_droplet_flame_grp(st, "SampleFlame", SAMPLE_FLAME_R, 0.0, SAMPLE_FLAME_APEX_DZ,
                          SAMPLE_FLAME_EMISSIVE, hidden=True, x=SPOON_X, y=SPOON_Y)
    print("[effect] SampleFlame grp hidden (powder-surface flame, color from flame_color)")


def rebuild_flames(st2):
    """酒精灯火焰迁到 /World 顶层并 grp 包装（pivot=火焰底，task 每帧 flicker，仿 C4）。
    初始隐藏（C3 灯未点燃，task 点火后显示）。"""
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat",
                 "/World/flame_outer", "/World/flame_inner",
                 "/World/flame_outer_sphere", "/World/flame_inner_sphere",
                 "/World/flame_outer_grp", "/World/flame_inner_grp"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame_grp(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                          (0.35, 0.55, 2.40), hidden=True, x=LAMP_X, y=LAMP_Y)   # 外焰偏蓝
    add_droplet_flame_grp(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                          (2.80, 0.55, 0.20), hidden=True, x=LAMP_X, y=LAMP_Y)   # 内焰偏黄
    print(f"[lamp] flames grp: base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"at ({LAMP_X},{LAMP_Y}) (hidden, flicker-driven)")


def cleanup_dish(st2):
    """表面皿自带粉丘（flametest 残留 powder GeomSubset）重绑到皿材质，避免与 powder.usd
    真实粉丘双份。"""
    dish = st2.GetPrimAtPath("/World/SurfaceDish")
    if not dish.IsValid():
        print("[dish] /World/SurfaceDish not found, skip")
        return
    dish_mat = st2.GetPrimAtPath("/World/SurfaceDish/_materials/dish_mat_002_002")
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


def powder_textures(st2):
    """粉末纹理重定位：powder.usd 烘平后 ./textures/x 相对路径失效，重定位到 equipment/textures。"""
    scene_dir = os.path.dirname(OUT)
    for prim in Usd.PrimRange(st2.GetPseudoRoot()):
        if prim.GetTypeName() != "Shader":
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path and \
                    v.path.replace("\\", "/").startswith("./textures/"):
                base = os.path.basename(v.path.replace("\\", "/"))
                newp = os.path.relpath(os.path.join(EQ, "textures", base), scene_dir).replace("\\", "/")
                inp.Set(Sdf.AssetPath(newp))
                print(f"[powder] texture {base} -> {newp}")


def brighten_spatula(st2):
    """药匙 = 普通不锈钢（银黑）：metallic 1.0 + low roughness + 深灰 diffuse（同 d2s）。

    金属药匙在无环境反射下反黑不可见；灯光修好（CylinderLight 12000 + env 贴图）
    后改回标准不锈钢（去 emissive 防洗白）。值域与 spatula.usd 源材质一致（幂等）。
    """
    sh = st2.GetPrimAtPath("/World/Spatula/material/stainless_steel")
    if not sh.IsValid() or sh.GetTypeName() != "Shader":
        print("[spatula] material not found, skip")
        return
    ush = UsdShade.Shader(sh)
    ush.GetInput("metallic").Set(1.0)
    ush.GetInput("roughness").Set(0.45)
    ush.GetInput("diffuseColor").Set(Gf.Vec3f(0.24, 0.24, 0.27))
    ush.GetInput("emissiveColor").Set(Gf.Vec3f(0.0, 0.0, 0.0))
    print("[spatula] stainless: metallic 1.0, roughness 0.45, diffuse 0.24, emissive 0")


def brighten_spoon(st2):
    """燃烧匙铜碗/铬把手在 RTX 下金属反黑不可见：改非金属亮 diffuse 保证可见。

    2026-09-01 快照实证（三连测）：① 资产只写 diffuse/metallic/roughness，金属
    metallic 0.9/0.85 在 headless RTX 无环境反射 → 深灰近黑（铜色丢失、与背景
    同色不可见）；② 补 ior/specular/opacity 仍深灰；③ emissive 在 headless RTX
    不生效（emissive 红 0 像素）。唯一可靠配方 = metallic 0 + 亮 diffuse
    （magenta 测试 80px 证实几何可见）。故铜碗/铬把手改亮铜/亮银 diffuse + 低
    roughness，metallic 置 0，牺牲金属反射换可见性（同 d2s 药匙在灯修好前的做法）。
    """
    for mat_name, (diffuse, rough) in {
        "copper": ((0.82, 0.50, 0.26), 0.35),   # 亮铜碗
        "chrome_rod": ((0.70, 0.75, 0.80), 0.25),  # 亮银铬把手
    }.items():
        sh = st2.GetPrimAtPath(f"/World/CombustionSpoon/Looks/{mat_name}/shader")
        if not sh.IsValid() or sh.GetTypeName() != "Shader":
            print(f"[spoon] {mat_name} shader not found, skip")
            continue
        ush = UsdShade.Shader(sh)
        ush.GetInput("diffuseColor").Set(Gf.Vec3f(*diffuse))
        ush.GetInput("metallic").Set(0.0)
        ush.GetInput("roughness").Set(rough)
        ush.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        ush.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.5)
        ush.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        print(f"[spoon] {mat_name}: metallic 0, diffuse {diffuse}, rough {rough}, specular 0.5")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    names = ["TestTubeRack", "CombustionSpoon", "AlcoholLamp",
             "SurfaceDish", "SamplePowder", "Spatula", "Match"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")

    # 试管架底座贴台面
    rkn, rkx = boxes["TestTubeRack"]
    assert abs(rkn[2] - TABLE_TOP) < 0.002, f"rack base z {rkn[2]} != table {TABLE_TOP}"

    # 燃烧匙：碗缘贴台面（用户定稿 T.z=0.8068，碗底圆弧略入台面 3mm）、把手竖直立在架旁
    # （碗心 x≈SPOON_X、把手顶端 x≈SPOON_X+0.08、顶端 z 高 ~29cm）。
    pn, px = boxes["CombustionSpoon"]
    assert abs(pn[2] - TABLE_TOP) < 0.006, f"spoon bowl bottom {pn[2]:.4f} not near table"
    assert px[0] > SPOON_X + 0.02, f"spoon blade not extending +X: x max {px[0]}"
    assert px[2] - pn[2] > 0.25, f"spoon blade too short: {px[2]-pn[2]:.3f}"
    assert not st2.GetPrimAtPath("/World/IronStand").IsValid(), "IronStand should be removed"

    # 酒精灯身中心在 (LAMP_X, LAMP_Y)；灯帽摘旁（勿用整灯 bbox 含帽）
    body = st2.GetPrimAtPath("/World/AlcoholLamp/body")
    br = bc.ComputeWorldBound(body).ComputeAlignedRange()
    bmn, bmx = br.GetMin(), br.GetMax()
    assert abs((bmn[0] + bmx[0]) / 2 - LAMP_X) < 0.002, "lamp body center x off"
    assert abs((bmn[1] + bmx[1]) / 2 - LAMP_Y) < 0.002, "lamp body center y off"

    # 表面皿贴台面，粉丘在皿上
    dsn, dsx = boxes["SurfaceDish"]
    assert abs(dsn[2] - TABLE_TOP) < 0.002, f"dish bottom {dsn[2]} not on table"
    pwn, pwx = boxes["SamplePowder"]
    assert pwn[2] > TABLE_TOP - 0.001, f"powder below table: {pwn[2]}"
    assert abs((pwn[0] + pwx[0]) / 2 - DISH_T[0]) < 0.015, "powder x off dish center"
    assert abs((pwn[1] + pwx[1]) / 2 - DISH_T[1]) < 0.015, "powder y off dish center"

    # 药匙竖插试管架中心孔（同 d2s）：竖直（高 ~13.5cm）、底近台面、xy 落在架孔
    spn, spx = boxes["Spatula"]
    assert TABLE_TOP - 0.001 < spn[2] < TABLE_TOP + 0.015, \
        f"spatula bottom {spn[2]:.4f} not near table"
    assert spx[2] - spn[2] > 0.12, f"spatula not vertical: height {spx[2]-spn[2]:.3f}"
    assert abs((spn[0] + spx[0]) / 2 - SPATULA_T[0]) < 0.01, "spatula x off rack hole"
    assert abs((spn[1] + spx[1]) / 2 - SPATULA_T[1]) < 0.01, "spatula y off rack hole"

    # 火柴躺台面抬高
    mtn, mtx = boxes["Match"]
    assert mtn[2] > TABLE_TOP + 0.010, f"match not raised: {mtn[2]}"

    # 灯帽摘下放灯正北 12cm：帽底贴台面
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    print(f"[verify] cap        min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    assert abs((cmn[1] + cmx[1]) / 2 - (LAMP_Y + 0.12)) < 0.005, "cap center y off 12cm north of lamp"

    # 火焰迁到 /World 顶层 + grp 包装（pivot=火焰底，flicker），apex 0.936，初始隐藏
    f = st2.GetPrimAtPath("/World/flame_outer_grp/flame_outer")
    assert f.IsValid() and f.GetTypeName() == "Cone", "flame_outer cone missing"
    assert abs(UsdGeom.Cone(f).GetRadiusAttr().Get() - FLAME_OUTER_R) < 0.0005, "flame r wrong"
    assert abs(FLAME_APEX_Z - 0.936) < 0.0005, "flame apex not 0.936"
    assert UsdGeom.Imageable(f).ComputeVisibility() == "invisible", "flame_outer should be hidden"
    assert st2.GetPrimAtPath("/World/flame_outer_grp").IsValid(), "flame_outer_grp missing"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"

    # 样品燃烧火焰 grp（combustible 点燃 reveal），初始隐藏
    sf = st2.GetPrimAtPath("/World/SampleFlame_grp/SampleFlame")
    assert sf.IsValid() and sf.GetTypeName() == "Cone", "SampleFlame cone missing"
    assert UsdGeom.Imageable(sf).ComputeVisibility() == "invisible", "SampleFlame should be hidden"

    print("[verify] OK: 试管架贴台 | 燃烧匙碗贴台·把手竖直立架旁 | "
          "灯居中·帽摘正北 | 表面皿+粉丘 | 药匙竖插试管架 | 火柴抬高 | 火焰迁顶层·初始隐藏(尖0.936)")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rotxyz in EQUIP:
        add_equip(stage, name, asset, t, scale, rotxyz)
    add_effects(stage)  # PowderOnSpoon 效果 prim（初始隐藏，task 挖粉后显示）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    strip_dome_lights(st2)
    remove_asset_env_lights(st2)
    detach_lamp_cap(st2)        # 灯帽摘下放灯旁 12cm 台面
    rebuild_flames(st2)         # 火焰迁到 /World 顶层 + grp 包装（初始隐藏，flicker）
    add_sample_flame(st2)       # 样品燃烧火焰 grp（combustible 点燃 reveal，焰色=输入）
    cleanup_dish(st2)           # 表面皿自带粉丘重绑皿材质（去双份粉丘）
    powder_textures(st2)        # 粉末纹理重定位到 equipment/textures
    brighten_spatula(st2)       # 药匙 = 不锈钢（metallic 1.0 + 深灰 diffuse）
    brighten_spoon(st2)         # 燃烧匙铜碗/铬把手补 opacity=1.0（防金属反黑不可见）
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)   # 移巨型 CylinderLight 远离相机
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
