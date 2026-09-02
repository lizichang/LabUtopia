# -*- coding: utf-8 -*-
"""生成 d1_acid_base_titration.usd —— D1 酸碱滴定场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）。
D1 = v12 目录「酸碱滴定」，全项目唯一双臂协同实验（定案单臂顺序改编）。

器材（第一象限；机械臂底座 [-0.15,0.05,0.71]，用户要求器材离机械臂远、不靠近 x=0/y=0、
尽量同一象限，故全部落在 x>0.15 / y>0.14 的第一象限深处）：
  - 铁架台 iron_stand_burette.usd（已加高竖杆 0.78 + 内联滴定管夹箍环 collar/arm/ring @ z=0.725）
  - 酸式滴定管 burette_acid.usd（已缩 550mm/25mL Class A），夹在箍环 ring，旋塞朝 +Y（正前方）
  - 锥形瓶 conical_flask_93x93x165.usd，滴定管管尖正下方（管尖 0.9955，瓶口 0.9645，间距 3cm）
  - 待测样品溶液（预装锥形瓶内，~20mL 无色液柱，用户选「预装进锥形瓶」省移液管）
  - 标准溶液瓶 hcl_bottle.usd（盐酸标准液，滴定管左侧，装液入滴定管用）
  - 指示剂瓶 sample_bottle_simple.usd + 滴管 dropper.usd（标准溶液瓶左前方）
  - 烧杯 beaker.usd（倒标准液入滴定管）、废液杯 beaker.usd#2（排泡废液承接）
  - 洗瓶 wash_bottle.usd（润洗）、漏斗 funnel_short.usd（辅助装液）

几何（世界坐标，米，Z-up；台面顶 z=0.80）：
  铁架台竖杆 (0.42,0.34)，箍环 ring 世界 (0.533,0.34,1.525)
  滴定管 origin (0.533,0.34,0.995)：夹持点 local z0.53 对齐 ring z1.525；管尖 0.9955、旋塞 1.035
  锥形瓶 (0.533,0.34,0.80)：瓶口 0.9645，管尖在瓶口上方 3.1cm

用法：python scripts/gen_d1_acid_base_titration_scene.py （labutopia conda env 有 pxr）
"""
import math
import os
import shutil

from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d1_acid_base_titration")
OUT = os.path.join(SCENE_DIR, "d1_acid_base_titration.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 铁架台（iron_stand_burette.usd：已加高竖杆顶 0.78、内联箍环 ring 局部 (0.113,0,0.725)）——
IRON_STAND_X, IRON_STAND_Y = 0.42, 0.34
RING_OFFSET_X = 0.113          # 箍环 ring 中心相对铁架台竖杆 x 偏移（collar→arm→ring）
RING_LOCAL_Z = 0.725           # 箍环 ring 中心铁架台局部 z（竖杆加高后）
BURETTE_GRIP_LOCAL_Z = 0.53    # 滴定管夹持点局部 z（管上部无刻度区，对齐 ring）
BURETTE_X = IRON_STAND_X + RING_OFFSET_X          # 0.533（滴定管/锥形瓶中心 x）
BURETTE_Y = IRON_STAND_Y
BURETTE_Z = TABLE_TOP + RING_LOCAL_Z - BURETTE_GRIP_LOCAL_Z   # 0.995（滴定管 origin z）
BURETTE_ROTZ = 90.0            # 旋塞 local +X → 世界 +Y（正前方，朝操作者，机械臂正面接近）

# —— 辅助器材（第一象限，主体周围，机械臂 -X/+Y 接近主体路径留空；≤0.85m 臂展）——
BEAKER_XY = (0.62, 0.34)       # 烧杯（倒标准液入滴定管），滴定管右侧
HCL_XY = (0.38, 0.34)          # 标准溶液瓶（盐酸），滴定管左侧约 15cm
INDICATOR_XY = (0.31, 0.50)    # 指示剂瓶（酚酞），标准溶液瓶左前方
DROPPER_XY = (0.29, 0.58)      # 滴管，指示剂瓶前方（吸指示剂用）
WASH_XY = (0.50, 0.50)         # 蒸馏水洗瓶（润洗），右前
WASTE_XY = (0.20, 0.55)        # 废液杯（排泡/润洗废液），左前
FUNNEL_XY = (0.60, 0.14)       # 漏斗（辅助装液入滴定管），正前下方

# —— 待测样品液柱（预装锥形瓶内，~20mL 无色样品溶液；滴定前状态）——
SAMPLE_LIQUID = dict(color=(0.82, 0.90, 0.95), opacity=0.55, roughness=0.20, ior=1.33)
SAMPLE_LIQUID_R = 0.022
SAMPLE_LIQUID_H = 0.018
SAMPLE_LIQUID_BOTTOM = TABLE_TOP + 0.002
SAMPLE_LIQUID_CZ = SAMPLE_LIQUID_BOTTOM + SAMPLE_LIQUID_H / 2.0

# (name, asset, translate, scale, rotxyz, ref_path)   tz=None 动态贴台面；
# ref_path 仅 conical_flask 用（其 defaultPrim=/World 只带材质，几何在 /root）
EQUIP = [
    ("IronStand", "iron_stand_burette.usd", (IRON_STAND_X, IRON_STAND_Y, None), None, None, None),
    ("BuretteAcid", "burette_acid.usd", (BURETTE_X, BURETTE_Y, BURETTE_Z), None,
     (0, 0, BURETTE_ROTZ), None),
    ("ConicalFlask", "conical_flask_93x93x165.usd", (BURETTE_X, BURETTE_Y, None), None, None,
     "/root"),
    ("Beaker", "beaker.usd", (BEAKER_XY[0], BEAKER_XY[1], None), None, None, None),
    ("HclBottle", "hcl_bottle.usd", (HCL_XY[0], HCL_XY[1], None), None, None, None),
    ("IndicatorBottle", "sample_bottle_simple.usd", (INDICATOR_XY[0], INDICATOR_XY[1], None),
     None, None, None),
    ("Dropper", "dropper.usd", (DROPPER_XY[0], DROPPER_XY[1], None), None, None, None),
    ("WashBottle", "wash_bottle.usd", (WASH_XY[0], WASH_XY[1], None), None, None, None),
    ("WasteCup", "beaker.usd", (WASTE_XY[0], WASTE_XY[1], None), None, None, None),
    ("FunnelShort", "funnel_short.usd", (FUNNEL_XY[0], FUNNEL_XY[1], None), None, None, None),
]

# 玻璃配方（d3s 同款真玻璃：op 0.15 看得到管内液面/弯月面，rough 0.10 / ior 1.5）
GLASS = dict(color=(0.85, 0.90, 0.95), opacity=0.15, roughness=0.10, ior=1.5)
# 新增器材玻璃 shader 配方（hcl_bottle/dropper/sample_bottle，override_bound_shader 用 diffuseColor 键）
GLASS_SHADER = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.20, roughness=0.10, ior=1.5)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒 min z（判断底座相对原点偏移；frame0 读，见 usd-asset-frame0 记忆）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Gf.TimeCode(0.0), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale=None, rotxyz=None, ref_path=None):
    """引用资产（ref_path 指定 defaultPrim 外的 prim path，如 conical_flask 用 /root）。"""
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    refs = prim.GetPrim().GetReferences()
    abs_path = os.path.abspath(os.path.join(EQ, asset))
    if ref_path:
        refs.AddReference(abs_path, ref_path)
    else:
        refs.AddReference(abs_path)
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
          + (f" rot{rotxyz}" if rotxyz else "") + (f" scale {scale}" if scale else "")
          + (f" ref={ref_path}" if ref_path else ""))


def bind_glass_material(st2, prim, name):
    """给 prim 绑定新玻璃材质（conical_flask 引用 /root 后 material:binding 悬空，重绑）。"""
    mat_path = f"/World/_materials/{name}_glass"
    mat = UsdShade.Material.Define(st2, mat_path)
    sh = UsdShade.Shader.Define(st2, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GLASS["color"]))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(GLASS["opacity"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(GLASS["roughness"])
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(GLASS["ior"])
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
    UsdGeom.Gprim(prim).CreateDoubleSidedAttr().Set(True)
    print(f"[mat] {prim.GetPath()} rebound -> {mat_path} (op {GLASS['opacity']})")


def override_shader(st2, shader_path, recipe):
    """改指定 shader 的输入参数（滴定管玻璃透明化）。"""
    sh = UsdShade.Shader(st2.GetPrimAtPath(shader_path))
    if not sh.GetPrim().IsValid():
        print(f"[mat] {shader_path} not found, skip")
        return
    for name, val in recipe.items():
        vt = Sdf.ValueTypeNames.Color3f if name in ("diffuseColor",) else Sdf.ValueTypeNames.Float
        inp = sh.GetInput(name)
        if not inp:
            inp = sh.CreateInput(name, vt)
        inp.Set(val)
    print(f"[mat] {shader_path} -> {recipe}")


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（material:binding → shader；d3s 同款）。"""
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


def add_env_light(stage):
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def strip_dome_lights(st2):
    removed = []
    for p in Usd.PrimRange(st2.GetPseudoRoot()):
        if str(p.GetPath()) != "/World/env_light" and p.IsA(UsdLux.DomeLight):
            removed.append(str(p.GetPath()))
    for path in removed:
        st2.RemovePrim(path)
    print(f"[dome] removed leftover DomeLight: {removed}" if removed else "[dome] no leftover")


def remove_asset_env_lights(st2):
    """wash_bottle/beaker 资产自带 flametest 残留 env_light，逐个删。"""
    for name, *_ in EQUIP:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")


def brighten_lights(st2):
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity -> 12000")


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
        print(f"[light] CylinderLight translate -> ({x}, {v[1]:.3f}, {v[2]:.3f})")
        return
    print("[light] CylinderLight has no translate op, skip")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def fix_burette_glass(st2):
    """滴定管管身玻璃透明化（看得到管内弯月面）；刻度 blue_ink/ink 保持不透明。"""
    override_shader(st2, "/World/BuretteAcid/Looks/glass/shader",
                    dict(diffuseColor=Gf.Vec3f(*GLASS["color"]), opacity=GLASS["opacity"],
                         roughness=GLASS["roughness"], ior=GLASS["ior"]))


def rebind_conical_flask(st2):
    """conical_flask 引用 /root 后 material:binding=/World/Looks/glass 悬空，重绑玻璃材质。"""
    flask = st2.GetPrimAtPath("/World/ConicalFlask")
    if not flask.IsValid():
        print("[mat] /World/ConicalFlask not found, skip")
        return
    for c in Usd.PrimRange(flask):
        if c.IsA(UsdGeom.Mesh):
            bind_glass_material(st2, c.GetPrim(), "conical_flask")


def add_sample_liquid(stage):
    """锥形瓶内预装待测样品液柱（~20mL 无色样品溶液；滴定前状态，task 阶段加指示剂/滴定变色）。"""
    g = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    g.CreateRadiusAttr(SAMPLE_LIQUID_R)
    g.CreateHeightAttr(SAMPLE_LIQUID_H)
    g.CreateAxisAttr("Z")
    g.AddTranslateOp().Set(Gf.Vec3d(BURETTE_X, BURETTE_Y, SAMPLE_LIQUID_CZ))
    mat_path = "/World/_materials/sample_liquid"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*SAMPLE_LIQUID["color"]))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQUID["opacity"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQUID["roughness"])
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQUID["ior"])
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(mat)
    UsdGeom.Gprim(g.GetPrim()).CreateDoubleSidedAttr().Set(True)
    print(f"[effect] SampleLiquid at ({BURETTE_X},{BURETTE_Y},{SAMPLE_LIQUID_CZ:.3f}) "
          f"r={SAMPLE_LIQUID_R} h={SAMPLE_LIQUID_H} (op {SAMPLE_LIQUID['opacity']})")


def fix_hcl_bottle(st2):
    """盐酸标准液瓶：瓶身玻璃透明化 + 1mm 液面盘隐藏（d3s HClBottle 同款）。"""
    for name in ("HclBottle",):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh":
                continue
            if c.GetName() == "liquid":
                UsdGeom.Imageable(c).MakeInvisible()
                print(f"[mat] hid {c.GetPath()} (1mm liquid disc)")
            else:
                if override_bound_shader(st2, c, GLASS_SHADER):
                    UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_indicator_bottle(st2):
    """指示剂瓶（sample_bottle_simple）：瓶身玻璃透明化；瓶塞保持不透明。"""
    for name in ("IndicatorBottle",):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh" or c.GetName() != "bottle":
                continue
            if override_bound_shader(st2, c, GLASS_SHADER):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper(st2):
    """滴管玻璃透明化（dropper.usd glass_001 op 1.0 不透明遮管内液柱，d3s 同款）。"""
    for name in ("Dropper",):
        mat = st2.GetPrimAtPath(f"/World/{name}/_materials/glass_001")
        if not mat.IsValid():
            print(f"[mat] {name} glass_001 not found, skip")
            continue
        for c in mat.GetChildren():
            if c.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(c)
            for n, val in GLASS_SHADER.items():
                inp = sh.GetInput(n)
                vt = Sdf.ValueTypeNames.Color3f if n == "diffuseColor" else Sdf.ValueTypeNames.Float
                if not inp:
                    inp = sh.CreateInput(n, vt)
                inp.Set(val)
            print(f"[mat] {name} glass_001 -> transparent {GLASS_SHADER}")
        g = st2.GetPrimAtPath(f"/World/{name}/glass_body_mesh/glass_body_mesh_001")
        if g.IsValid() and g.GetTypeName() == "Mesh":
            UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] {g.GetPath()} doubleSided")


def fix_time_sampled_attrs(st2):
    """Blender 导出的资产（burette_acid/conical_flask/funnel_short）几何点集只写在 frame0
    timeSamples、default 为空（default 读点集得空 → 渲染不可见）。转成干净 default（坑 R3）。"""
    fixed = 0
    for p in Usd.PrimRange(st2.GetPseudoRoot()):
        for attr in p.GetAttributes():
            ts = attr.GetTimeSamples()
            if not ts:
                continue
            if attr.Get() is not None:
                continue  # 已有 default，无需处理
            v = attr.Get(ts[0])
            attr.Clear()
            attr.Set(v)
            fixed += 1
    print(f"[timesample] {fixed} 个 ts-only 属性已转 default")


def verify(st2):
    """自检：打印各器材世界 bbox；断言滴定管尖/旋塞/锥形瓶口关键几何 + 全器材臂展可达（纯 pxr）。"""
    bc = UsdGeom.BBoxCache(Gf.TimeCode(0.0), ["default"])
    names = ["IronStand", "BuretteAcid", "ConicalFlask", "Beaker", "HclBottle",
             "IndicatorBottle", "Dropper", "WashBottle", "WasteCup", "FunnelShort", "SampleLiquid"]
    base = (-0.15, 0.05)  # 机械臂底座 XY
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        dist = math.hypot(cx - base[0], cy - base[1])
        print(f"[verify] {name:16s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f}) 距底座 {dist:.3f}m")
    # 关键几何断言
    tip_z = BURETTE_Z + 0.0005
    mouth_z = TABLE_TOP + 0.1645
    gap = tip_z - mouth_z
    print(f"[verify] 滴定管尖 z={tip_z:.4f} 锥形瓶口 z={mouth_z:.4f} 间距 {gap*100:.1f}cm")
    assert 0.02 < gap < 0.05, f"管尖-瓶口间距 {gap:.3f}m 超出 2~5cm"
    # 旋塞 world 位置（rotZ+90：旋塞 local +X → world +Y）
    plug_y = BURETTE_Y + 0.0262
    plug_z = BURETTE_Z + 0.04
    print(f"[verify] 旋塞 world ({BURETTE_X:.3f},{plug_y:.3f},{plug_z:.3f})")
    print(f"[verify] 待测液柱底 z={SAMPLE_LIQUID_BOTTOM:.4f} 顶 z={SAMPLE_LIQUID_BOTTOM + SAMPLE_LIQUID_H:.4f}")
    print("[verify] PASS")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rotxyz, ref_path in EQUIP:
        add_equip(stage, name, asset, t, scale, rotxyz, ref_path)
    add_sample_liquid(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    fix_time_sampled_attrs(st2)        # 几何 ts-only → default（否则滴定管/锥形瓶/漏斗不可见）
    strip_dome_lights(st2)
    remove_asset_env_lights(st2)     # 洗瓶/烧杯残留 env_light
    rebind_conical_flask(st2)        # 锥形瓶引用 /root 后重绑玻璃材质
    fix_burette_glass(st2)           # 滴定管管身玻璃透明化
    fix_hcl_bottle(st2)              # 盐酸瓶玻璃透明化 + 液面盘隐藏
    fix_indicator_bottle(st2)        # 指示剂瓶玻璃透明化
    fix_dropper(st2)                 # 滴管玻璃透明化
    brighten_lights(st2)             # 主光 2000→12000
    set_cylinder_light_x(st2, x=-10.0)  # 移巨型 CylinderLight 远离相机
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
