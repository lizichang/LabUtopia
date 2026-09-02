# -*- coding: utf-8 -*-
"""生成 b5_melting_point.usd —— B5 熔点测定（提勒管法）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用器材：铁架台（删铁环/挂钩，只留铁柱+底座，真实熔点装置只有铁柱）+ 酒精灯 + 提勒管
  （竖直，侧管 b 形环鼓向远离铁柱）+ 试管夹（夹提勒管主管上部，代替真实烧瓶夹）+ 温度计
  （插入主管，水银球在侧管上下叉口之间）+ 毛细管（平放台面待装样）。
- 载热液 = 石蜡油（BUILTIN 浅黄透明液柱，液面高于上叉口，供热对流循环）。
- 酒精灯火焰尖对准侧管下弯处（b 形环最低点 = 加热点），外焰偏蓝内焰偏黄（b2 同款水滴形）。
- 布局锚：铁柱 (STAND_X, STAND_Y)，提勒管主管轴在铁柱 -X 侧 10cm（同 b2 堆叠中心线），
  侧管环鼓向更 -X（远离铁柱），酒精灯在侧管下弯处正下方。

用法：python scripts/gen_b5_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b5_melting_point")
OUT = os.path.join(SCENE_DIR, "b5_melting_point.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- 布局常量 ----
STAND_X, STAND_Y = 0.50, 0.0029      # 铁柱世界位置（从 b2 的 0.6286 挪近底座；2026-09-02 底座再 -X 15cm
                                      # → 提勒管口 x=0.40 距底座 0.844m < 0.885m 极限，保插管可达）
TUBE_X = STAND_X - 0.100             # 提勒管主管轴 x（铁柱 -X 侧 10cm）
TUBE_Y = STAND_Y

# 提勒管（rot180 侧管环鼓 -X）：主管圆底 z 悬空（离台面 12.8cm，给酒精灯留空间）
THIELE_BOTTOM_Z = 0.928
# 侧管 V 顶点（加热点）：资产局部 z=0.0515（上下叉口正中）、x=+0.045（鼓 +X），rot180 后 x=-0.045
HEAT_X = TUBE_X - 0.045
HEAT_Z = THIELE_BOTTOM_Z + 0.0515  # 0.9795 = 火焰尖 = V 顶点（对称 V 最鼓处）
LAMP_X = HEAT_X                       # 酒精灯中心 = 侧管下弯处正下方

# 夹子：夹主管上部（珠边口下方），夹口圆弧中心对准主管轴
CLAMP_Z = THIELE_BOTTOM_Z + 0.120     # 1.048
CLAMP_T = (TUBE_X + 0.050, STAND_Y + 0.0209, CLAMP_Z)

# 石蜡油载热液：主管内腔（内径 Ø21 r=0.0105，内壁底 z 局部 0.002）
OIL_R = 0.010
OIL_BOTTOM_Z = THIELE_BOTTOM_Z + 0.004   # 0.932
OIL_TOP_Z = THIELE_BOTTOM_Z + 0.138      # 1.066（液面高于上叉口 0.085）
OIL_H = OIL_TOP_Z - OIL_BOTTOM_Z
OIL_CZ = (OIL_BOTTOM_Z + OIL_TOP_Z) / 2

# 毛细管：台面左前区（待装样，长 100mm 沿 x；抬高 12mm 防夹爪 collider 扎桌面，同火柴）
# 2026-09-02 tmp 重摆：闭口端 (0.1710,0.2704)、开口端 (0.2710,0.2704)（原 (0.40,0.15)）
CAP_T = (0.1710, 0.2704, TABLE_TOP + 0.013)

# 表面皿 + 粉丘（台面右前区，挖粉取样起点；2026-09-02 tmp 重摆：皿 (0.4433,0.1488)，原 (0.33,0.30)）
DISH_T = (0.4433, 0.1488, TABLE_TOP)
POWDER_T = (DISH_T[0] + 0.0018, DISH_T[1] - 0.0058, TABLE_TOP - 0.0012)
POWDER_SCALE = 0.4

# 火柴（点火用）：躺灯 -x -y 侧，头朝 +X（match.usd +X 端=火柴头），抬高 12mm 防卡爪（b2 同款）
# 2026-09-02 tmp 重摆 (0.4817,-0.164)（灯 +X -Y 侧右下）
MATCH_T = (0.4817, -0.164, 0.813)

# 试管架 + 主温度计（2026-09-02 用户「温度计先插试管架里本来就是竖直的，不用再旋转」→ 倒插，
# 泡朝上露出镂空区贴毛细管，臂手指朝前 ORIENT_FWD 水平横夹竖直杆身提出，再法兰转 166° 泡朝下）。
# 试管架（test_tube_rack.usd）孔格 = 2 列（x=±0.021）×3 排（后 y=+0.118 / 中 -0.0004 / 前），
# 底板顶（孔底）world z = RACK_Z(0.8965) − 0.0905 = 0.806。温度计放**左前孔**（2026-09-02 用户
# 重摆 tmp：试管架从前区 (0.30,-0.15) 挪到 (0.3659,0.3884)，温度计改插左前孔，避开臂工作区）。
# 温度计倒插（rot(180,0,0)，局部 +Z→世界 −Z）：泡尖（原点 local z=0）朝上、挂环顶（local 0.2762）
# 落底板顶 0.806 → origin z = 0.806 + 0.2762 = 1.0822。泡 world z[1.068,1.084]（泡中心 1.076）、
# 杆 world z[0.814,1.074]、塞 world z[0.933,0.957]、挂环 world z[0.806,0.819]。
RACK_X, RACK_Y = 0.3659, 0.3884   # 试管架位置（tmp 2026-09-02 重摆：左前孔温度计 (0.3471,0.2696)）
RACK_Z = TABLE_TOP + 0.0965        # 0.8965 试管架原点 z（tz=None 自动贴台面）
RACK_BASE_TOP_Z = RACK_Z - 0.0905  # 0.806 底板顶（= 孔底，温度计挂环落点）
THERMO_HOLE_X = RACK_X - 0.0188    # 0.3471 左列孔心 x（tmp 实测）
THERMO_HOLE_Y = RACK_Y - 0.1188    # 0.2696 前排孔心 y（tmp 实测）
THERMO_ORIGIN_Z = RACK_BASE_TOP_Z + 0.2762  # 1.0822 泡尖 origin z（倒插后挂环顶落底板顶）
MAIN_THERMO_T = (THERMO_HOLE_X, THERMO_HOLE_Y, THERMO_ORIGIN_Z)  # 主温度计倒插（泡朝上）
STOPPER_DZ = 0.125          # 橡胶塞底 z（温度计局部；塞中心 0.137 = 温度计中高）

# 蘸油皿（培养皿 + 石蜡油薄层）：装样后臂抓温度计垂直下探蘸泡，再贴毛细管（油滴法吸附）
OIL_DISH_T = (0.25, 0.15, TABLE_TOP)   # 毛细管 -X 侧，与粉丘 (0.33,0.30) 错开
OIL_DISH_R = 0.024                     # 油液半径（皿 Ø60 内留边）
OIL_DISH_H = 0.004                     # 油液厚 4mm

# 灯帽摘下来放灯旁 12cm（-X 远离铁柱）台面：帽底贴台面，帽心 (LAMP_X-0.12, TUBE_Y, 0.8155)
CAP_DETACH = (0.12, 0.0, -0.0762)        # cap translate op（灯局部坐标，R180 换算见 detach_lamp_cap）

# (prim, asset, translate, scale, rotxyz)  tz=None → 动态贴台面；rotxyz=(rx,ry,rz) 角度
EQUIP = [
    ("IronStand", "iron_stand.usd", (STAND_X, STAND_Y, None), None, (0, 0, 180)),
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_X, TUBE_Y, None), None, (0, 0, 180)),
    ("ThieleTube", "thiele_tube.usd", (TUBE_X, TUBE_Y, THIELE_BOTTOM_Z), None, (0, 0, 180)),
    ("TestTubeClamp", "test_tube_clamp.usd", CLAMP_T, None, (0, 0, 180)),
    ("CapillaryTube", "capillary_tube.usd", CAP_T, None, (0, 90, 0)),
    ("SurfaceDish", "sample_dish.usd", DISH_T, None, None),
    ("SamplePowder", "powder.usd", POWDER_T, POWDER_SCALE, None),
    ("OilDish", "sample_dish.usd", OIL_DISH_T, None, None),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, None),
    ("MainThermometer", "thermometer.usd", MAIN_THERMO_T, None, (180, 0, 0)),
    ("Match", "match.usd", MATCH_T, None, None),
]

# 火焰几何（b2 同款水滴形：底半球 + 上部收尖锥）
FLAME_BASE_Z = TABLE_TOP + 0.091            # 灯芯根部世界 z（火焰底 = holder 顶/灯芯露出点）
FLAME_APEX_Z = HEAT_Z                       # 火焰尖刚好碰到侧管下弯处
FLAME_OUTER_R = 0.009                       # 外焰肚半径
FLAME_INNER_R = 0.005                       # 内焰（焰心）肚半径
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022   # 内焰焰心 apex

# 石蜡油材质（浅黄透明油状）
OIL = dict(color=(0.95, 0.85, 0.50), opacity=0.55, roughness=0.10, ior=1.45)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
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


def add_oil_liquid(stage):
    """石蜡油载热液柱：主管内腔（r<0.0105），液面高于上叉口。静态可见（本就装在管里）。"""
    liq = UsdGeom.Cylinder.Define(stage, "/World/ParaffinOil")
    liq.CreateRadiusAttr(OIL_R)
    liq.CreateHeightAttr(OIL_H)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, OIL_CZ))
    add_material(stage, liq.GetPrim(), OIL["color"], OIL["opacity"],
                 roughness=OIL["roughness"], ior=OIL["ior"], double_sided=True)
    print(f"[effect] ParaffinOil cylinder r{OIL_R} h{OIL_H:.3f} cz{OIL_CZ:.4f} "
          f"(top {OIL_TOP_Z:.4f} > upper port {THIELE_BOTTOM_Z+0.085:.4f})")


def add_oil_dish_liquid(stage):
    """蘸油皿里的石蜡油薄层：装样后臂把温度计泡垂直下探蘸油（油滴法吸附的油源）。

    sample_dish.usd 培养皿 Ø60mm、内凹高约 6.6mm，贴台面 z[0.80,0.8066]。油液 = 薄圆柱
    （Ø48mm×4mm）贴皿底，底面略高于台面（皿玻璃底厚 2mm → 油底 0.802、顶 0.806）。浅黄
    透明（OIL 材质，与提勒管载热液同配方）。温度计泡 Ø10mm 下探 5mm 即可沾油。
    """
    liq = UsdGeom.Cylinder.Define(stage, "/World/OilDishLiquid")
    liq.CreateRadiusAttr(OIL_DISH_R)
    liq.CreateHeightAttr(OIL_DISH_H)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(OIL_DISH_T[0], OIL_DISH_T[1], TABLE_TOP + 0.004))
    add_material(stage, liq.GetPrim(), OIL["color"], OIL["opacity"],
                 roughness=OIL["roughness"], ior=OIL["ior"], double_sided=True)
    print(f"[effect] OilDishLiquid cylinder r{OIL_DISH_R} h{OIL_DISH_H} "
          f"at ({OIL_DISH_T[0]},{OIL_DISH_T[1]},{TABLE_TOP+0.004:.4f})")


def add_sample_plug(st2):
    """管内样品柱（蘸粉后可见）：毛细管开口端内一段白色粉末柱，蘸粉前不可见。

    毛细管 asset 烘平后 tube mesh 直接挂在 /World/CapillaryTube 下（reference /root 是
    identity 被 flatten 优化掉），管轴沿局部 Z（闭口端 z=0、开口端 z=0.100）。样品柱 =
    Cylinder 轴 Z、半径 0.0005（Ø1mm，落在 Ø1.6mm 管内）、高 0.005（5mm 粉柱），中心
    z=0.094 → 柱 z[0.0915,0.0965] 卡在开口端（0.100）之内。挂在 CapillaryTube 下 → 随管
    task 逐帧 pivot 矩阵一起动，无需单独更新。visibility 默认 invisible，蘸粉后由 task 打开。
    """
    plug = UsdGeom.Cylinder.Define(st2, "/World/CapillaryTube/SamplePlug")
    plug.CreateRadiusAttr(0.0005)
    plug.CreateHeightAttr(0.005)
    plug.CreateAxisAttr("Z")
    UsdGeom.Xformable(plug).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.094))
    UsdGeom.Imageable(plug).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    mat = UsdShade.Material.Define(st2, "/World/CapillaryTube/SamplePlug_mat")
    sh = UsdShade.Shader.Define(st2, "/World/CapillaryTube/SamplePlug_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.90, 0.88, 0.84))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.20, 0.19, 0.18))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(plug).Bind(mat)
    print("[plug] SamplePlug cylinder r0.0005 h0.005 at local z0.094 (invisible until dip)")


def add_thermo_stopper(stage):
    """主温度计中间加橡胶塞：贯穿温度计杆、塞提勒管口（Ø20.5 ≈ 管口内径 Ø21）。

    rubber_stopper_3.usd：Ø20.5×24mm 圆柱，轴沿 Z，原点在底（z 0→0.024），mesh 几何写在
    frame0 timeSamples（Blender 导出无 default，见 burette/锥形瓶坑）。温度计资产烘平后
    /World/MainThermometer/Thermometer 恒等（无 xform op），泡底 z=-0.002、顶（挂环）
    z=0.2762，中高 = (−0.002+0.2762)/2 = 0.137。塞子中心对齐中高 → 塞底 translate z =
    0.137 − 0.012（塞高 0.024 半） = 0.125。塞子挂 MainThermometer 下（随温度计一起
    水平/竖直），温度计杆 Ø8mm 从塞子 Ø20.5mm 中心贯穿，杆中段被不透明塞子遮住 → 视觉
    = 温度计穿过塞子（用户 2026-09-01「塞子中心被温度计贯穿」）。
    """
    stopper = UsdGeom.Xform.Define(stage, "/World/MainThermometer/Stopper")
    stopper.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, "rubber_stopper_3.usd")))
    stopper.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, STOPPER_DZ))
    print(f"[stopper] MainThermometer/Stopper <- rubber_stopper_3.usd at local z={STOPPER_DZ}")


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
        if not paths:
            print(f"[clean] no DomeLight in {name}")


def remove_ring_hook(st2):
    """删铁架台的铁环/挂钩：熔点装置铁架台只有铁柱+底座（夹子直接连铁柱），
    铁环（托石棉网）和挂钩（挂温度计）都用不上，留着会挡提勒管/温度计。"""
    for name in ("ring", "hook"):
        path = f"/World/IronStand/root/{name}"
        p = st2.GetPrimAtPath(path)
        if p.IsValid():
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")
        else:
            print(f"[clean] {path} not found, skip")


def detach_lamp_cap(st2):
    """灯帽从灯顶摘下，放灯旁 12cm（-X 远离铁柱）台面，闭口朝下贴台面。

    资产 cap xform = [Translate(0,0,0) rotateX90 scale0.01]，mesh 局部 z[0.076,0.107]
    （帽底 0.076/帽顶 0.107），cap 中心 dz=0.0915。酒精灯整组已 R180，cap 局部 x/y
    被旋 180° 取反，z 不变。换算（pxr 实测，同 b2 move_lamp_cap）：
      帽心世界 = (灯x - tx, 灯y - ty, 灯z + 0.0915 + tz)
    目标帽心 (LAMP_X-0.12, TUBE_Y, 0.8155) → tx=0.12, ty=0.0, tz=-0.0762。
    """
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[cap] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(*CAP_DETACH)
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[cap] translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[cap] (no translate op) add translate {tuple(tgt)}")


def add_droplet_flame(st2, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere（底部圆） + 上部 Cone（收尖），绕 Z 轴。x 对齐酒精灯中心。"""
    zc = z_b + r
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(LAMP_X, TUBE_Y, zc))
    h = z_a - zc
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(LAMP_X, TUBE_Y, zc + h / 2))
    for prim in (sph, cone):
        pname = prim.GetPath().name
        # 初始熄灭：B5 装样阶段（拿毛细管/蘸粉/贴泡/插管）酒精灯尚未点燃，
        # 火焰留待后续加热观察阶段由 task 打开 visibility（用户 2026-09-02「最开始是没有点燃火焰的」）
        UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
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


def rebuild_flames(st2):
    """酒精灯火焰迁到 /World 顶层（灯下引用子 prim RTX 不渲染），apex 对准侧管下弯处。"""
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                      (0.35, 0.55, 2.40))   # 外焰偏蓝
    add_droplet_flame(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                      (2.80, 0.55, 0.20))   # 内焰偏黄
    print(f"[lamp] flames: base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"(touches thiele heating point x={LAMP_X:.4f})")


def fix_thermo_material(st2, name="Thermometer"):
    """温度计红液去反光（同 b2）：matte + emissive 红，透过提勒管+石蜡油看得清。"""
    mat = st2.GetPrimAtPath(f"/World/{name}/Looks/RedLiquid")
    if not mat.IsValid():
        print(f"[mat] {name} RedLiquid not found, skip")
        return
    specs = (
        ("diffuseColor", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(0.02, 0.005, 0.005)),
        ("emissiveColor", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1.6, 0.40, 0.40)),
        ("roughness", Sdf.ValueTypeNames.Float, 1.0),
        ("specularColor", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(0.0, 0.0, 0.0)),
        ("metallic", Sdf.ValueTypeNames.Float, 0.0),
    )
    for c in mat.GetChildren():
        if c.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(c)
        for n, vt, val in specs:
            inp = sh.GetInput(n)
            if not inp:
                inp = sh.CreateInput(n, vt)
            inp.Set(val)
        print(f"[mat] {name} RedLiquid -> matte+emissive red ({sh.GetPath()})")


def cleanup_dish(st2, name="SurfaceDish"):
    """皿自带粉丘（flametest 残留 powder GeomSubset）重绑到皿材质，避免与 powder.usd
    真实粉丘双份。sample_dish.usd 自带 env_light 已由 remove_asset_env_lights 删。
    表面皿 / 蘸油皿同款资产，按 name 复用。"""
    dish = st2.GetPrimAtPath(f"/World/{name}")
    if not dish.IsValid():
        print(f"[dish] /World/{name} not found, skip")
        return
    dish_mat = st2.GetPrimAtPath(f"/World/{name}/_materials/dish_mat_002_002")
    if not dish_mat.IsValid():
        print(f"[dish] {name} dish material not found, skip rebind")
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


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "ThieleTube", "TestTubeClamp",
             "CapillaryTube", "ParaffinOil",
             "SurfaceDish", "SamplePowder", "OilDish",
             "TestTubeRack", "MainThermometer", "Match"]
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

    # 铁架台底座贴台面
    smn, smx = boxes["IronStand"]
    assert abs(smn[2] - TABLE_TOP) < 0.002, f"IronStand base z {smn[2]} != table {TABLE_TOP}"
    # 铁环/挂钩已删
    assert not st2.GetPrimAtPath("/World/IronStand/root/ring").IsValid(), "ring not removed"
    assert not st2.GetPrimAtPath("/World/IronStand/root/hook").IsValid(), "hook not removed"

    # 提勒管：主管底悬空 THIELE_BOTTOM_Z，主管右壁（珠边 +X）= TUBE_X+0.013，侧管环鼓 -X
    tmn, tmx = boxes["ThieleTube"]
    assert abs(tmn[2] - THIELE_BOTTOM_Z) < 0.002, f"thiele bottom {tmn[2]} != {THIELE_BOTTOM_Z}"
    assert abs(tmx[0] - (TUBE_X + 0.013)) < 0.002, f"thiele right wall {tmx[0]} off {TUBE_X+0.013}"
    assert tmn[0] < TUBE_X - 0.026, f"side arm V vertex not bulging -X: x min {tmn[0]}"
    assert abs(tmx[2] - (THIELE_BOTTOM_Z + 0.150)) < 0.002, "thiele top wrong"

    # 酒精灯身中心 x = 侧管下弯处 x（火焰对准加热点）；灯帽已摘旁，勿用整灯 bbox（含帽）
    body = st2.GetPrimAtPath("/World/AlcoholLamp/body")
    br = bc.ComputeWorldBound(body).ComputeAlignedRange()
    bmn, bmx = br.GetMin(), br.GetMax()
    lamp_cx = (bmn[0] + bmx[0]) / 2
    assert abs(lamp_cx - HEAT_X) < 0.002, f"lamp body center x {lamp_cx} != heat x {HEAT_X}"

    # 夹子夹住主管上部：z 在主管上部，夹口 x 包住主管（x min < 主管右壁）
    cmn, cmx = boxes["TestTubeClamp"]
    assert cmx[2] > tmn[2] + 0.05, f"clamp too low: top {cmx[2]}"
    assert cmn[0] < tmx[0] - 0.005, f"clamp jaw not wrapping tube: x min {cmn[0]}"

    # 石蜡油：液柱在主管内，液面高于上叉口（0.085），底高于内壁底
    omn, omx = boxes["ParaffinOil"]
    assert omn[2] >= THIELE_BOTTOM_Z + 0.002 - 0.003, "oil bottom below inner wall"
    assert omx[2] >= THIELE_BOTTOM_Z + 0.085, f"oil top {omx[2]} below upper port"
    assert abs((omn[0] + omx[0]) / 2 - TUBE_X) < 0.004, "oil x off thiele axis"

    # 毛细管平放台面上方 12mm（抬高防卡爪，长 100mm 沿 x）
    cpn, cpx = boxes["CapillaryTube"]
    assert cpn[2] > TABLE_TOP + 0.010, f"capillary not raised above table: {cpn[2]}"
    assert abs(cpx[0] - cpn[0] - 0.100) < 0.002, "capillary length wrong (should be horizontal 100mm)"

    # 管内样品柱：存在、默认 invisible（蘸粉后由 task 打开）
    plug = st2.GetPrimAtPath("/World/CapillaryTube/SamplePlug")
    assert plug.IsValid(), "SamplePlug missing"
    assert UsdGeom.Imageable(plug).GetVisibilityAttr().Get() == UsdGeom.Tokens.invisible, \
        "SamplePlug should start invisible"

    # 试管架：贴台面（底板底 0.80），左前孔放倒插温度计
    rkn, rkx = boxes["TestTubeRack"]
    assert abs(rkn[2] - TABLE_TOP) < 0.002, f"rack bottom {rkn[2]} not on table"

    # 主温度计：倒插左前孔（rot(180,0,0)，竖直长 278mm 沿 Z，泡朝上、挂环底落底板顶）
    mthn, mthx = boxes["MainThermometer"]
    assert abs((mthx[2] - mthn[2]) - 0.278) < 0.006, \
        f"main thermo length {mthx[2]-mthn[2]:.4f} wrong (should be vertical 278mm)"
    assert abs((mthn[0] + mthx[0]) / 2 - THERMO_HOLE_X) < 0.004, "main thermo x off left hole"
    assert abs((mthn[1] + mthx[1]) / 2 - THERMO_HOLE_Y) < 0.004, "main thermo y off front hole"
    assert abs(mthn[2] - RACK_BASE_TOP_Z) < 0.004, \
        f"main thermo bottom (ring) {mthn[2]} not on rack base {RACK_BASE_TOP_Z}"
    # 橡胶塞：贯穿温度计杆、中心对齐温度计中高（局部 z0.137），挂 MainThermometer 下随倒插
    stopper = st2.GetPrimAtPath("/World/MainThermometer/Stopper")
    assert stopper.IsValid(), "Stopper missing"
    bc0 = UsdGeom.BBoxCache(Gf.TimeCode(0.0), ["default"])  # 塞子 mesh 在 frame0（Blender 导出无 default）
    sr = bc0.ComputeWorldBound(stopper).ComputeAlignedRange()
    smn, smx = sr.GetMin(), sr.GetMax()
    print(f"[verify] stopper    min({smn[0]:+.4f},{smn[1]:+.4f},{smn[2]:+.4f}) "
          f"max({smx[0]:+.4f},{smx[1]:+.4f},{smx[2]:+.4f})")
    # 塞子随温度计倒插（rot180）：世界 z=origin−(STOPPER_DZ+0.012)=1.0822−0.137=0.9452，x/y 对准杆轴
    assert abs((smn[0] + smx[0]) / 2 - THERMO_HOLE_X) < 0.004, "stopper x not coaxial with stem"
    assert abs((smn[1] + smx[1]) / 2 - THERMO_HOLE_Y) < 0.004, "stopper y not coaxial with stem"
    assert abs((smn[2] + smx[2]) / 2 - (THERMO_ORIGIN_Z - STOPPER_DZ - 0.012)) < 0.004, \
        "stopper center z not at thermo middle"

    # 表面皿贴台面，粉丘在皿上（粉丘 x/y 视觉中心≈皿中心，底部高于台面）
    dsn, dsx = boxes["SurfaceDish"]
    assert abs(dsn[2] - TABLE_TOP) < 0.002, f"dish bottom {dsn[2]} not on table"
    pwn, pwx = boxes["SamplePowder"]
    assert pwn[2] > TABLE_TOP - 0.001, f"powder below table: {pwn[2]}"
    assert abs((pwn[0] + pwx[0]) / 2 - DISH_T[0]) < 0.015, "powder x off dish center"
    assert abs((pwn[1] + pwx[1]) / 2 - DISH_T[1]) < 0.015, "powder y off dish center"

    # 蘸油皿贴台面 + 皿内石蜡油薄层（油底高于台面、油顶低于皿口）
    odn, odx = boxes["OilDish"]
    assert abs(odn[2] - TABLE_TOP) < 0.002, f"oil dish bottom {odn[2]} not on table"
    oil = st2.GetPrimAtPath("/World/OilDishLiquid")
    assert oil.IsValid(), "OilDishLiquid missing"
    orng = bc.ComputeWorldBound(oil).ComputeAlignedRange()
    oln, olx = orng.GetMin(), orng.GetMax()
    assert oln[2] > TABLE_TOP, f"oil below table: {oln[2]}"
    assert abs((oln[0] + olx[0]) / 2 - OIL_DISH_T[0]) < 0.004, "oil x off dish center"
    assert abs((oln[1] + olx[1]) / 2 - OIL_DISH_T[1]) < 0.004, "oil y off dish center"

    # 火柴躺台面抬高 12mm（防卡爪），头朝灯芯
    mtn, mtx = boxes["Match"]
    assert mtn[2] > TABLE_TOP + 0.010, f"match not raised above table: {mtn[2]}"

    # 灯帽摘下放灯旁 12cm（-X 远离铁柱）：帽底贴台面，帽心 (LAMP_X-0.12, TUBE_Y)
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    print(f"[verify] cap        min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    assert abs((cmn[0] + cmx[0]) / 2 - (LAMP_X - 0.12)) < 0.005, "cap center x off 12cm beside lamp"
    assert abs((cmn[1] + cmx[1]) / 2 - TUBE_Y) < 0.005, "cap center y off lamp axis"

    # 火焰迁到 /World 顶层，apex 对准侧管下弯处
    f = st2.GetPrimAtPath("/World/flame_outer")
    assert f.IsValid() and f.GetTypeName() == "Cone", "flame_outer cone missing"
    assert abs(UsdGeom.Cone(f).GetRadiusAttr().Get() - FLAME_OUTER_R) < 0.0005, "flame r wrong"
    assert abs(FLAME_APEX_Z - HEAT_Z) < 0.0005, "flame apex not at heat point"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"

    print("[verify] OK: 铁架台贴台/删环钩 | 提勒管悬空竖直+侧管鼓-X | 灯对准下弯处 | "
          "夹子夹主管上部 | 石蜡油液面过上叉口 | 毛细管平放 | "
          "试管架贴台+温度计倒插左前孔泡朝上 | 表面皿+粉丘 | 火柴 | 灯帽摘旁 | 火焰尖对准加热点")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rotxyz in EQUIP:
        add_equip(stage, name, asset, t, scale, rotxyz)
    add_oil_liquid(stage)
    add_oil_dish_liquid(stage)  # 蘸油皿石蜡油薄层（油滴法蘸泡用）
    add_thermo_stopper(stage)  # 主温度计中间加橡胶塞（贯穿+塞管口，随 Export 一起烘平）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    remove_ring_hook(st2)       # 熔点装置铁架台只留铁柱+底座
    detach_lamp_cap(st2)        # 灯帽摘下放灯旁 12cm 台面
    rebuild_flames(st2)         # 火焰迁到 /World 顶层，apex 对准侧管下弯处
    cleanup_dish(st2)           # 表面皿自带粉丘重绑皿材质（去双份粉丘）
    cleanup_dish(st2, "OilDish")  # 蘸油皿同理去自带粉丘
    powder_textures(st2)        # 粉末纹理重定位到 equipment/textures
    brighten_lights(st2)
    fix_env_light(st2)
    fix_thermo_material(st2, "MainThermometer")  # 主温度计：红液去反光
    add_sample_plug(st2)        # 管内样品柱（开口端白粉柱，蘸粉前不可见）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
