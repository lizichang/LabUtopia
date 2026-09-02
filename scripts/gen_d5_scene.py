# -*- coding: utf-8 -*-
"""生成 d5_distillation.usd —— D5 蒸馏分离（预组装蒸馏装置 + 酒精灯加热收集）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材（三脚架 / 石棉网 / 蒸馏烧瓶 / 温度计 + 橡皮塞 / 冷凝管 /
  接液瓶 / 铁架台 + 滴定管夹 / 酒精灯 / 沸石 / 火柴）。
- 布局（预组装蒸馏装置，机械臂仅执行「点燃酒精灯 → 加热收集 → 盖灯帽熄火」，照文档
  D11 注「装置组装建议人工完成、机械臂仅加热收集」）：
      酒精灯(桌面 z0.80) → 三脚架(z0.955 环) → 石棉网(z0.955) → 蒸馏烧瓶(底 z0.957，内置
      沸石×3 + 20mL 样品液) → 温度计(泡尖在支管口 z1.099，杆身经瓶口向上)
      侧支管(+X) → 冷凝管(斜置 -58°，进口贴支管、出口罩接液瓶口) → 接液瓶(桌面)
  简化：不摆铁架台+滴定管夹（其底座 x[-0.152,+0.048] 反重块与三脚架腿/接液瓶必撞，且夹环
  local x+0.113 对不上冷凝管中段 0.752；文档已把「组装」外包人工，机械臂只加热收集）。冷凝管
  两端由烧瓶支管口+接液瓶口托住、中间悬空（学校简化演示常见）。橡皮塞无温度计孔→不摆。
  酒精灯复用 B2/D9 位 (0.5286,0.0029) + rot180（火柴/灯帽坐标全照 B2 已验证值）；
  灯帽从灯顶挪到桌面静止位 CAP_REST=(0.42,-0.01)（盖帽动作在桌面夹帽，B2 同款）；
  火焰迁到 /World 顶层（灯下引用子 prim 在 RTX 不渲染）。

用法：python scripts/gen_d5_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d5_distillation")
OUT = os.path.join(SCENE_DIR, "d5_distillation.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 锚：酒精灯世界位置（复用 B2/D9 (0.5286,0.0029)，火柴/灯帽坐标照抄已验证值）——
LAMP_X, LAMP_Y = 0.5286, 0.0029
# 三脚架环顶（asset ring z[0.151,0.155]，环顶 0.155）→ 世界 0.955；石棉网坐环上。
GAUZE_Z = TABLE_TOP + 0.155 + 0.001     # 0.956（石棉网 origin，min z=-0.001 → 底 0.955 坐环顶）
FLASK_BOTTOM_Z = GAUZE_Z + 0.001        # 0.957（蒸馏烧瓶底坐网上）
# 蒸馏烧瓶 asset（frame0 实测）：body Ø65(半径 0.0325) z[0,0.210]；侧支管 side_arm x[0.0036,0.0752]
# y±0.004 z[0.1284,0.1548]（中心 z=0.1416，支管口 +X 端 x=0.0752）。故支管口世界：
SIDE_ARM_TIP = (LAMP_X + 0.0752, LAMP_Y, FLASK_BOTTOM_Z + 0.1416)   # (0.6038,0.0029,1.0986)
FLASK_MOUTH_Z = FLASK_BOTTOM_Z + 0.210   # 瓶口 1.167（温度计+橡皮塞插此）
# 温度计泡尖在支管口高度（文档 D11「水银球位于支管口处」），杆身经橡皮塞穿瓶口向上。
THERMO_BULB_Z = FLASK_BOTTOM_Z + 0.1416  # 1.099

# 冷凝管：斜置 ~-58°（rotateY），进口（plastic_top 端，local +Z）贴支管口、出口（local z=0）
# 罩接液瓶口。冷凝管 asset body z[0,0.317] + plastic_top z[0.316,0.347]，总长 0.347，Ø36。
# 进口 = SIDE_ARM_TIP，出口 = 接液瓶口上方 2cm → 倾斜角 = atan(Δz/Δx)。
COND_OUTLET = (0.900, LAMP_Y, 0.917)     # 出口（接液瓶口 0.897 上方 2cm）
_dx = SIDE_ARM_TIP[0] - COND_OUTLET[0]          # -0.2962
_dz = SIDE_ARM_TIP[2] - COND_OUTLET[2]          # +0.1816
COND_LEN = math.hypot(_dx, _dz)                 # ~0.3474
COND_ROT_Y = math.degrees(math.atan2(_dx, _dz))  # -58.4°（local +Z 从出口指向进口）

# 接液瓶（锥形瓶 77×77×97，口 z0.0975 → 世界 0.897）
RECV_X, RECV_Y = COND_OUTLET[0], COND_OUTLET[1]

# 火柴 / 灯帽（复用 B2 已验证值；灯帽挪桌面静止位 CAP_REST）
MATCH_X, MATCH_Y = 0.40, -0.06
MATCH_T = 0.813
CAP_REST = (0.42, -0.01, 0.8155)   # 灯帽静止位世界中心（-X 侧桌面，B2 同款）

# (prim, asset_file, translate, scale, rot, ref_path)
#   tz=None → 动态贴台面；rot=(rx,ry,rz) 角度；ref_path=None → 用 defaultPrim。
#   锥形瓶 asset defaultPrim=/World 只带材质（几何在同层 /root 兄弟 prim），须显式 ref /root。
EQUIP = [
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_X, LAMP_Y, None), None, (0, 0, 180), None),
    ("Tripod", "tripod_stand.usd", (LAMP_X, LAMP_Y, None), None, (0, 0, 0), None),
    ("AsbestosGauze", "asbestos_gauze.usd", (LAMP_X, LAMP_Y, GAUZE_Z), None, (0, 0, 0), None),
    ("DistillationFlask", "distillation_flask.usd", (LAMP_X, LAMP_Y, FLASK_BOTTOM_Z), None, (0, 0, 0), None),
    ("Thermometer", "thermometer.usd", (LAMP_X, LAMP_Y, THERMO_BULB_Z), None, (0, 0, 0), None),
    ("Condenser", "condenser_reflux.usd", COND_OUTLET, None, (0, COND_ROT_Y, 0), None),
    ("ReceivingFlask", "conical_flask_77x77x97.usd", (RECV_X, RECV_Y, None), None, (0, 0, 0), "/root"),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, MATCH_T), None, (0, 0, 0), None),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移，frame0 读几何）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot=(0, 0, 0), ref_path=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    asset_path = os.path.abspath(os.path.join(EQ, asset))
    if ref_path:
        prim.GetPrim().GetReferences().AddReference(asset_path, Sdf.Path(ref_path))
    else:
        prim.GetPrim().GetReferences().AddReference(asset_path)
    tx, ty, tz = t
    if tz is None:
        min_z = asset_local_min_z(asset)
        tz = TABLE_TOP - min_z
        print(f"[equip] {name} base offset {min_z:+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot != (0, 0, 0):
        prim.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" rot{rot}" if rot != (0, 0, 0) else "") + (f" scale {scale}" if scale else "")
          + (f" ref={ref_path}" if ref_path else ""))


# ---- D5 效果 prim（内建，task 动画驱动）----
# 烧瓶内样品液（可见，蓝色半透明，20mL）、沸石×3（可见，沉底）、沸腾气泡组（隐藏）、
# 馏出液滴落串（隐藏，冷凝管出口坠入接液瓶）、接液瓶内馏出液（隐藏，逐滴生长）。
SAMPLE_LIQ_R = 0.028             # 液柱半径（烧瓶内径 ~0.030）
SAMPLE_LIQ_H = 0.012             # 液面高 12mm（~20mL，贴烧瓶圆底）
SAMPLE_LIQ_CZ = FLASK_BOTTOM_Z + SAMPLE_LIQ_H / 2 + 0.002   # 0.965
ZEO_Z = FLASK_BOTTOM_Z + 0.006   # 沸石底（烧瓶底上方 6mm 内壁）
BUBBLE_R = 0.002                 # 气泡半径
RECV_LIQ_R = 0.030               # 接液瓶内液柱半径（锥形瓶下部）
DROP_BALL_R = 0.003              # 馏出液滴半径
N_DROPS = 8                      # 液滴串数量（task 循环复用）
DROP_HOME = (COND_OUTLET[0], COND_OUTLET[1], COND_OUTLET[2] - 0.01)   # 出口下方 1cm


def _gen_bubbles(n=30, seed=7):
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        r = 0.022 * math.sqrt(rng.random())
        a = 2.0 * math.pi * rng.random()
        out.append((LAMP_X + r * math.cos(a), LAMP_Y + r * math.sin(a), FLASK_BOTTOM_Z + 0.010))
    return out

BUBBLE_BASE = _gen_bubbles()

# 样品液（同 d2l 单通道主导配方防洗白：近黑 diffuse + 单通道主导 emissive）
SAMPLE_LIQ = dict(color=(0.05, 0.06, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                  emissive=(0.15, 0.35, 2.0))


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


def add_shared_material(stage, mat_path, diffuse, opacity, prims, roughness=0.5,
                        emissive=None):
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    for p in prims:
        UsdShade.MaterialBindingAPI(p).Bind(mat)


def add_d5_effects(stage):
    """内建效果 prim：烧瓶内样品液（可见）+ 沸石×3（可见，equip 引用）+ 气泡组（隐藏）
    + 馏出液滴落串（隐藏）+ 接液瓶内馏出液（隐藏，task 逐滴生长）。"""
    # 样品液柱（烧瓶内，蓝色半透明，可见）
    liq = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    liq.CreateRadiusAttr(SAMPLE_LIQ_R)
    liq.CreateHeightAttr(SAMPLE_LIQ_H)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(LAMP_X, LAMP_Y, SAMPLE_LIQ_CZ))
    add_material(stage, liq.GetPrim(), SAMPLE_LIQ["color"], SAMPLE_LIQ["opacity"],
                 roughness=SAMPLE_LIQ["roughness"], ior=SAMPLE_LIQ["ior"],
                 emissive=SAMPLE_LIQ["emissive"], double_sided=True)
    print(f"[effect] SampleLiquid visible (flask {SAMPLE_LIQ_H:.3f}m, "
          f"top {SAMPLE_LIQ_CZ + SAMPLE_LIQ_H / 2:.3f})")

    # 沸石×3（沉底，assets zeolite.usd 引用；白色颗粒）
    for i, (zx, zy) in enumerate([(0.002, 0.002), (-0.005, 0.004), (0.004, -0.005)]):
        zprim = UsdGeom.Xform.Define(stage, f"/World/Zeolite{i}")
        zprim.GetPrim().GetReferences().AddReference(
            os.path.abspath(os.path.join(EQ, "zeolite.usd")))
        zprim.AddTranslateOp().Set(Gf.Vec3d(LAMP_X + zx, LAMP_Y + zy, ZEO_Z))
    print("[effect] Zeolite0..2 at flask bottom")

    # 气泡组（烧瓶内，隐藏，task 沸腾时 reveal + 上升）
    UsdGeom.Xform.Define(stage, "/World/FlaskBubbles")
    bub_prims = []
    for i, (x, y, z) in enumerate(BUBBLE_BASE):
        sp = UsdGeom.Sphere.Define(stage, f"/World/FlaskBubbles/bubble_{i}")
        sp.CreateRadiusAttr(BUBBLE_R)
        sp.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        UsdGeom.Imageable(sp).MakeInvisible()
        bub_prims.append(sp.GetPrim())
    add_shared_material(stage, "/World/FlaskBubbles/bubble_mat",
                        (0.72, 0.85, 1.0), 1.0, bub_prims, roughness=0.3,
                        emissive=(0.7, 1.0, 1.8))
    print(f"[effect] {len(BUBBLE_BASE)} bubbles hidden")

    # 馏出液滴落串（冷凝管出口 → 接液瓶，隐藏，task 逐滴坠落）
    g = UsdGeom.Xform.Define(stage, "/World/DistillateDrop")
    for i in range(N_DROPS):
        s = UsdGeom.Sphere.Define(stage, f"/World/DistillateDrop/Drop_{i}")
        s.CreateRadiusAttr(DROP_BALL_R)
        s.AddTranslateOp().Set(Gf.Vec3d(*DROP_HOME))
        add_material(stage, s.GetPrim(), (0.35, 0.75, 1.0), 0.90, roughness=0.05, ior=1.33,
                     double_sided=True)
        UsdGeom.Imageable(s).MakeInvisible()
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DistillateDrop hidden ({N_DROPS} drops)")

    # 接液瓶内馏出液（隐藏 h0，task 逐滴生长）
    rl = UsdGeom.Cylinder.Define(stage, "/World/ReceivingLiquid")
    rl.CreateRadiusAttr(RECV_LIQ_R)
    rl.CreateHeightAttr(0.0)
    rl.CreateAxisAttr("Z")
    rl.AddTranslateOp().Set(Gf.Vec3d(RECV_X, RECV_Y, TABLE_TOP))
    add_material(stage, rl.GetPrim(), (0.05, 0.06, 0.12), 0.95, roughness=0.05, ior=1.33,
                 emissive=(0.15, 0.35, 2.0), double_sided=True)
    UsdGeom.Imageable(rl).MakeInvisible()
    print("[effect] ReceivingLiquid hidden h0 (grow by distillate drip)")


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
    print("[light] CylinderLight intensity 2000 -> 12000")


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


def move_lamp_cap(st2):
    """灯帽从灯顶挪到桌面静止位 CAP_REST=(0.42,-0.01,0.8155)（B2 同款换算）。

    帽 = /World/AlcoholLamp/cap（灯子 prim，灯 R180 后帽局部 y 取反）。帽 Ø37mm×3.1cm
    竖直开口朝下倒扣桌面。换算（B2 pxr 实测）：cx=灯x−tx、cy=灯y−ty、cz=灯z+tz+CAP_CENTER_DZ
    → tx=0.5286−0.42=0.1086、ty=0.0029−(−0.01)=0.0129、tz=0.8155−0.8002−0.0915=−0.0762。
    """
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[clean] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(0.1086, 0.0129, -0.0762)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[clean] cap translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[clean] cap (no translate op) add translate {tuple(tgt)}")


def add_droplet_flame(st2, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere + 上部 Cone（B2 同款），默认可见（task reset 再熄）。"""
    zc = z_b + r
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(LAMP_X, LAMP_Y, zc))
    h = z_a - zc
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(LAMP_X, LAMP_Y, zc + h / 2))
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
    print(f"[lamp] droplet {name}: sphere r{r} c{zc:.4f} + cone apex {z_a:.4f}")


FLAME_BASE_Z = TABLE_TOP + 0.091        # 灯芯根部世界 z（0.891，火焰底）
FLAME_APEX_Z = GAUZE_Z - 0.001 - 0.001  # 石棉网底（0.954，火焰尖刚好碰到）
FLAME_OUTER_R = 0.009
FLAME_INNER_R = 0.005
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022


def rebuild_flames(st2):
    """酒精灯火焰：删灯下引用子 prim，在 /World 顶层重建（flametest 良方）。

    外焰偏蓝（B 主导淡蓝）底 0.891 apex 0.954（碰石棉网底）、内焰偏黄（R 主导）apex 0.913。
    task._flame_paths() 返回两组 4 路径统一熄/亮。
    """
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                      (0.35, 0.55, 2.40))
    add_droplet_flame(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                      (2.80, 0.55, 0.20))
    print(f"[lamp] flames: outer apex {FLAME_APEX_Z:.4f} touches gauze bottom, "
          f"inner apex {FLAME_INNER_APEX_Z:.4f}")


def _set_shader_glass(sh):
    """把 UsdPreviewSurface shader 设成透明玻璃（op0.15 透出内部液体；保留 transmission）。"""
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.85, 0.92, 0.98))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.15)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)


def bind_or_override_glass(st2, mesh_prim):
    """给玻璃 Mesh 透明化：已有可解析 material:binding → 重写其 shader；否则新建绑定。

    （锥形瓶 asset 几何在 /root、材质在 /World/Looks，ref /root 后 mesh 无有效材质→新建。）
    """
    UsdGeom.Gprim(mesh_prim).CreateDoubleSidedAttr().Set(True)
    rel = mesh_prim.GetRelationship("material:binding")
    if rel:
        for tgt in rel.GetTargets():
            mat = st2.GetPrimAtPath(tgt)
            if not mat.IsValid():
                continue
            for c in mat.GetChildren():
                if c.GetTypeName() == "Shader":
                    _set_shader_glass(UsdShade.Shader(c))
                    print(f"[mat] override {mesh_prim.GetPath()} -> {c.GetPath()}")
                    return
    mat = UsdShade.Material.Define(st2, str(mesh_prim.GetPath()) + "_glass")
    sh = UsdShade.Shader.Define(st2, str(mesh_prim.GetPath()) + "_glass/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    _set_shader_glass(sh)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh_prim).Bind(mat)
    print(f"[mat] bind new glass {mesh_prim.GetPath()}")


def fix_glass_materials(st2):
    """玻璃器材透明化：蒸馏烧瓶/接液瓶 → 真玻璃，透出内部样品液/沸石/气泡/馏出液。

    递归遍历各器材下 Mesh（mesh 嵌套深度 2-3 层），重写/新建绑定 shader。
    冷凝管已 op0.40 半透明，不动（plastic_top 保持不透明塑料）。
    """
    for name in ("DistillationFlask", "ReceivingFlask"):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        meshes = [c for c in Usd.PrimRange(p) if c.GetTypeName() == "Mesh"]
        for m in meshes:
            bind_or_override_glass(st2, m)
        print(f"[mat] {name}: {len(meshes)} mesh -> transparent glass")


def verify(st2):
    """自检：打印各器材世界 bbox，断言垂直堆叠关系。

    酒精灯/三脚架/石棉网/烧瓶贴台、石棉网坐三脚架环上、烧瓶坐网上、温度计泡尖在支管口、
    冷凝管进口贴支管口出口罩接液瓶、接液瓶贴台、铁架台贴台、火柴抬高、灯帽桌面静止位、
    火焰迁 /World 顶层默认可见、样品液可见、沸石/气泡/液滴组齐。
    """
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    names = ["AlcoholLamp", "Tripod", "AsbestosGauze", "DistillationFlask", "Thermometer",
             "Condenser", "ReceivingFlask", "Match", "SampleLiquid",
             "Zeolite0", "Zeolite1", "Zeolite2"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:16s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 酒精灯/三脚架/接液瓶贴台
    for nm in ("AlcoholLamp", "Tripod", "ReceivingFlask"):
        assert abs(boxes[nm][0][2] - TABLE_TOP) < 0.002, f"{nm} bottom not on table"
    # 石棉网坐三脚架环上（网底 ≈ 环顶 0.955）
    gmn, gmx = boxes["AsbestosGauze"]
    trip_top = TABLE_TOP + 0.155
    assert abs(gmn[2] - trip_top) < 0.003, f"gauze bottom {gmn[2]:.4f} != tripod ring top {trip_top}"
    # 烧瓶底坐石棉网上（±2mm 内，容许轻微陷入网面）
    fmn, fmx = boxes["DistillationFlask"]
    assert -0.002 <= fmn[2] - gmx[2] <= 0.003, \
        f"flask bottom {fmn[2]:.4f} not on gauze top {gmx[2]:.4f}"
    # 温度计泡尖（min z，origin=泡尖）在支管口高度（±2cm 内）
    thn, thx = boxes["Thermometer"]
    assert abs(thn[2] - (FLASK_BOTTOM_Z + 0.1416)) < 0.02, \
        f"thermo bulb {thn[2]:.4f} not near side arm {FLASK_BOTTOM_Z + 0.1416:.4f}"
    # 冷凝管：进口端（高、-X 端）贴支管口高度，出口端（低、+X 端）沉向接液瓶口上方（滴液入瓶）
    cmn, cmx = boxes["Condenser"]
    assert abs(cmx[2] - SIDE_ARM_TIP[2]) < 0.02, \
        f"condenser inlet z {cmx[2]:.4f} vs side arm {SIDE_ARM_TIP[2]:.4f}"
    rmn, rmx = boxes["ReceivingFlask"]
    assert cmn[2] > rmx[2] - 0.01, \
        f"condenser outlet bottom {cmn[2]:.4f} should be near receiving flask mouth {rmx[2]:.4f}"
    assert cmx[0] > rmn[0], "condenser should extend over receiving flask"
    # 火柴抬高 12mm
    assert boxes["Match"][0][2] > TABLE_TOP + 0.010, "match not raised"
    # 灯帽静止位 CAP_REST（帽底贴台）
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    cr = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmin, cmax = cr.GetMin(), cr.GetMax()
    print(f"[verify] cap     min({cmin[0]:+.4f},{cmin[1]:+.4f},{cmin[2]:+.4f}) "
          f"max({cmax[0]:+.4f},{cmax[1]:+.4f},{cmax[2]:+.4f})")
    assert abs(cmin[2] - TABLE_TOP) < 0.002, f"cap bottom {cmin[2]} not on table"
    assert abs((cmin[0] + cmax[0]) / 2 - 0.42) < 0.005, "cap center x != 0.42"
    # 样品液可见、顶在烧瓶内
    sld = boxes["SampleLiquid"]
    assert sld[1][2] < FLASK_BOTTOM_Z + 0.03, "sample liquid top too high in flask"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/SampleLiquid")).ComputeVisibility() != "invisible"
    # 气泡组 / 液滴串齐、隐藏
    bub = st2.GetPrimAtPath("/World/FlaskBubbles")
    nb = sum(1 for c in bub.GetChildren() if c.GetTypeName() == "Sphere")
    assert nb == len(BUBBLE_BASE), f"bubbles {nb} != {len(BUBBLE_BASE)}"
    dd = st2.GetPrimAtPath("/World/DistillateDrop")
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == N_DROPS, f"drops {nd} != {N_DROPS}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible"
    rl = st2.GetPrimAtPath("/World/ReceivingLiquid")
    assert UsdGeom.Cylinder(rl).GetHeightAttr().Get() == 0.0, "ReceivingLiquid should be h0"
    # 火焰迁 /World 顶层默认可见
    for name in ("flame_outer", "flame_inner"):
        f = st2.GetPrimAtPath(f"/World/{name}")
        assert f.IsValid() and f.GetTypeName() == "Cone", f"{name} top-level cone missing"
        assert UsdGeom.Imageable(f).ComputeVisibility() != "invisible", f"{name} should be visible"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), "old lamp flame present"
    print("[verify] OK: 台面贴底 / 网坐环 / 烧瓶坐网 / 温度计泡尖在支管口 / "
          "冷凝管进口贴支管出口罩接液瓶 / 火柴抬高 / 灯帽桌面静止 / "
          "样品液+沸石+气泡+液滴+馏出液齐 / 火焰迁/World顶层默认可见")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot, ref_path in EQUIP:
        add_equip(stage, name, asset, t, scale, rot, ref_path)
    add_d5_effects(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    move_lamp_cap(st2)
    rebuild_flames(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    fix_glass_materials(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
