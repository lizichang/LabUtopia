# -*- coding: utf-8 -*-
"""生成 b3_water_bath.usd —— B3 水浴加热（酒精灯加热烧杯水 → 试管内固体样品熔化）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World），参考 gen_b2_scene.py 的
铁架台布局（铁柱 R180 后环/钩支臂指向 -X，垂直堆叠中心线 x=0.5286）：

   酒精灯(桌面 z0.80，灯芯顶 0.9005) → 铁环(z0.910-0.918 托石棉网) → 石棉网(z0.9194)
     → 烧杯(底 0.9205 坐网上、口 1.0109，内装水至 0.9805)

2026-08-29 用户修正（逐字）：
  1. 试管最开始先放在试管架里面（不预夹在烧杯 → TestTubeRack + TestTube 立架上孔）。
  2. 不需要沸石，表面皿上放粉丘（删 zeolite×2 → SamplePowder=powder.usd scale 0.4 贴皿顶）。
  3. 烧杯不是正的，绕 X 转 -135° 才正 → 烧杯资产改 beaker.usd（用户 Blender 转好，内建
     rotateXYZ(-135,0,0)，直立 90mm 高，底座 z=0），场景直接引用不额外旋转。
  4. 烧杯中的水是流动液体，参考 d3l → BeakerWater = 半透明 Cylinder（UsdPreviewSurface，
     opacity 0.5 / ior 1.33 / roughness 0.05），非静态建模 mesh。
  5. 不需要试管夹，铁架台挂钩隐藏（删 TestTubeClamp；iron_stand /root/root/hook MakeInvisible）。
  6. 布局复刻 D2-S：「整个位置需要重新调整，先把 d2s 所有物品包含机械臂的位置复刻过来，
     这样子挖粉末才不会出错。铁架台和烧杯、酒精灯还是这样相对位置，但随便放个位置后面我来调整」
     → 试管/药匙/皿/粉/洗瓶 全用 D2-S 同款坐标（挖粉动作坐标与 d2s 完全一致，见常量区）；
       机械臂底座 config robot.position=[-0.15,0.05,0.71]（同 d2s）；加热堆叠保持 B2 相对几何，
       整组 -Y 平移 25cm 到 y=-0.25（原 y=0.0029 被 D2-S 皿占位；火柴/灯帽随灯平移）。

其余沿用 B2：火柴躺灯旁、灯帽静止位、水滴形火焰迁 /World 顶层、效果 prim（BeakerWater 可见
+ BeakerBubbles 初始隐藏 + TubeMelt_<色> 初始隐藏，task 驱动）。

用法：python scripts/gen_b3_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b3_water_bath")
OUT = os.path.join(SCENE_DIR, "b3_water_bath.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- 加热堆叠（铁架台/酒精灯/石棉网/烧杯）：B2 相对几何，整组 -Y 平移 25cm 避开 D2-S 样品区 ----
# 用户 2026-08-29：「铁架台和烧杯、酒精灯还是这样相对位置，但随便放个位置后面我来调整」。
# 原 y=0.0029 被 D2-S 复刻的皿 (0.5365,0.105) 占位 → 下移到 y=-0.25（用户后面可再调）。
# 相对几何不变：堆叠中心线在铁柱 -X 侧 10cm → TUBE_X=STAND_X-0.10=0.5286。
STAND_X, STAND_Y = 0.6286, -0.25
TUBE_X = STAND_X - 0.100          # 0.5286（烧杯/酒精灯/石棉网同轴）
TUBE_Y = STAND_Y                  # -0.25

# B3 无 RING_RAISE（B2 是给温度计挂钩/外焰改动的 2cm 上移，B3 不需要）：铁环保持资产原始高度。
GAUZE_Z = TABLE_TOP + 0.1194      # 石棉网中心 0.9194（坐铁环顶 0.918 上；网 z[-0.001,0.0011]）
GAUZE_BOTTOM_Z = GAUZE_Z - 0.0010 # 0.9184（火焰 apex 目标）
GAUZE_TOP_Z = GAUZE_Z + 0.0011    # 0.9205（烧杯底贴这里）

# 250mL 烧杯（beaker.usd：Ø75 × 高 90mm，直立，底 z=0 顶 z=0.0904）坐石棉网上。
# beaker.usd 内建 rotateXYZ(-135,0,0)（用户 2026-08-29 Blender 转正），底座贴网顶。
# 注意：beaker.usd mesh extent 是旋转前局部值，BBoxCache 会误报 160mm → verify 用 _points_bbox。
BEAKER_BOTTOM_Z = GAUZE_TOP_Z     # 0.9205（底座贴网顶）
BEAKER_TOP_Z = BEAKER_BOTTOM_Z + 0.0904  # 1.0109

# 烧杯内水（水浴）：2/3 填充（90mm 杯装 60mm），内径 ~Ø69 → 水柱 r=0.031，水面 0.9805
WATER_H = 0.060
WATER_TOP_Z = BEAKER_BOTTOM_Z + WATER_H   # 0.9805
WATER_R = 0.031
WATER_CZ = BEAKER_BOTTOM_Z + WATER_H / 2  # 0.9505（水柱中心 z）

# 火柴 / 灯帽：B2 相对灯偏移随灯平移。帽局部 (0.1086,0.0129,-0.076) rot180 翻转 → 世界
# (lamp_x-0.1086, lamp_y-0.0129) = (0.42, -0.2629)；火柴 (lamp_x-0.1286, lamp_y-0.0629) =
# (0.40, -0.3129)，头 +X 端朝灯芯 (0.5286,-0.25)。
MATCH_X, MATCH_Y = TUBE_X - 0.1286, TUBE_Y - 0.0629   # 0.40, -0.3129
MATCH_T = 0.813                      # 火柴原点 z：抬高 12mm 让手指离桌

# ---- 阶段A 样品区：D2-S 布局复刻（挖粉动作坐标必须与 d2s 完全一致，否则勺子/IK 出错）----
# D2-S（gen_d2s_scene.py EQUIP，2026-08-29 实测 d2s_water_solubility.usd 世界 bbox 一致）：
#   试管架 (0.6803,0.3607)、试管 (0.659,0.241,0.806) 架近侧左孔、药匙 (0.6993,0.3608,0.828,
#   rotZ-180) 架中心孔竖插、皿 (0.5365,0.105,0.80)、粉 (0.5383,0.0992,0.7988, scale0.4)、
#   洗瓶 (0.370,0.525,0.80, rotZ-180)。
RACK_X, RACK_Y = 0.6803, 0.3607
RACK_TZ = TABLE_TOP + 0.0965       # 0.8965（架底贴台面：架资产 min z=-0.0965）
RACK_HOLE_BOTTOM = RACK_TZ - 0.0905  # 0.8060（架孔洞底，管/勺底贴此）
RACK_TUBE_X, RACK_TUBE_Y = 0.659, 0.241   # D2-S 试管位（架近侧左孔，勿用架中心推导）
TUBE_BOTTOM_Z = RACK_HOLE_BOTTOM   # 0.8060（管在架孔里，底贴洞底）
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.1533  # 0.9593
SPATULA_X, SPATULA_Y, SPATULA_Z = 0.6993, 0.3608, 0.828  # 药匙（架中心孔竖插）
DISH_X, DISH_Y, DISH_Z = 0.5365, 0.105, 0.80   # 玻璃皿（D2-S 位）
DISH_TOP_Z = DISH_Z + 0.0066       # 0.8066（皿顶，D2-S 实测）
POWDER_X, POWDER_Y, POWDER_Z = 0.5383, 0.0992, 0.7988  # 粉丘（D2-S 位，贴皿顶）
POWDER_SCALE = 0.4                 # 粉丘缩放（D2-S 同款）
WASHB_X, WASHB_Y, WASHB_Z = 0.370, 0.525, 0.80  # 洗瓶（D2-S 位；B3 不用，用户「d2s所有物品」）

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面；rot180 → 绕 Z 旋 180°
# 加热堆叠整组 rot180（铁柱环/钩支臂指向 -X）；烧杯轴对称 rot180=False（壶嘴朝 +Y）。
# D2-S 复刻品（药匙/洗瓶）rot180=True（= D2-S rotZ-180，等效）。
EQUIP = [
    ("IronStand", "iron_stand.usd", (STAND_X, STAND_Y, None), None, True),
    ("AlcoholLamp", "alcohol_lamp.usd", (TUBE_X, TUBE_Y, None), None, True),
    ("AsbestosGauze", "asbestos_gauze.usd", (TUBE_X, TUBE_Y, GAUZE_Z), None, True),
    ("Beaker", "beaker.usd", (TUBE_X, TUBE_Y, BEAKER_BOTTOM_Z), None, False),
    # 阶段A 样品区：D2-S 布局复刻（试管先立架孔里 + 药匙挖粉 + 皿上粉丘 + 洗瓶）
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, False),
    ("TestTube", "test_tube.usd", (RACK_TUBE_X, RACK_TUBE_Y, RACK_HOLE_BOTTOM), None, False),
    ("Spatula", "spatula.usd", (SPATULA_X, SPATULA_Y, SPATULA_Z), None, True),
    ("SurfaceDish", "sample_dish.usd", (DISH_X, DISH_Y, DISH_Z), None, False),
    ("SamplePowder", "powder.usd", (POWDER_X, POWDER_Y, POWDER_Z), POWDER_SCALE, False),
    ("WashBottle", "wash_bottle.usd", (WASHB_X, WASHB_Y, WASHB_Z), None, True),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, MATCH_T), None, False),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def _points_bbox(st2, prim_path):
    """世界坐标 points-based 包围盒（避开 BBoxCache 的 extent 陈旧/旋转失真）。

    beaker.usd 的 mesh extent 是旋转前的局部 bbox，BBoxCache 变换后把直立烧杯误报成
    160mm 高（真身 90mm）→ 对带旋转的器材用实际 points 求世界 bbox。返回 (min, max) 或 None。
    """
    p = st2.GetPrimAtPath(prim_path)
    pts = []
    for m in Usd.PrimRange(p):
        if m.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(m)
        M = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for q in mesh.GetPointsAttr().Get(Usd.TimeCode.Default()):
            w = M.Transform(Gf.Vec3d(*q))
            pts.append((w[0], w[1], w[2]))
    if not pts:
        return None
    xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
    return (Gf.Vec3d(min(xs), min(ys), min(zs)), Gf.Vec3d(max(xs), max(ys), max(zs)))


def add_equip(stage, name, asset, t, scale, rot180=False):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot180:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (" rot180" if rot180 else "") + (f" scale {scale}" if scale else ""))


# ---- B3 效果 prim（内建，task 动画驱动）----
# 烧杯内水柱（水浴，蓝半透明，可见）、烧杯内加热气泡（球组，初始隐藏）、
# 试管内熔化液柱（TubeMelt_<色>，初始隐藏 h0，task 按 sample_phase/melt_color 揭示）。
BUBBLE_R = 0.002                       # 烧杯水浴气泡半径（Ø4mm）
MELT_R = 0.008                         # 试管内熔化液柱半径（< 管内径 ~0.0085）
MELT_H = 0.020                         # 熔化液柱高（固体熔化后沉试管底）
# 熔化液柱放烧杯水中央（未来试管浸入水浴处，非试管架处）；初始隐藏，task 揭示。
MELT_CZ = WATER_CZ                     # 0.9505（烧杯水中心 z）

# 水浴气泡基准位：30 颗离散小泡，在烧杯水柱内「环带」内（避试管 + 避烧杯壁），固定种子可复现。
# 试管 Ø19.2mm 外径 r0.0096 浸在烧杯水中央，气泡若落在管足迹内会上升穿管（~17% 会穿）。
# → 环带 R_IN=0.013（管外径 0.0096 + 泡 0.002 + 余量）、R_OUT=0.028（水柱 0.031 − 泡 0.002 − 余量）。
# 上升动画由 task 动态池驱动（连续生成 + 速度差异 + 蛇形 + 破灭复用），z 写水面下。
def _gen_beaker_bubbles(n=30, seed=7):
    rng = random.Random(seed)
    out = []
    R_IN = 0.013
    R_OUT = 0.028
    for _ in range(n):
        r = math.sqrt(R_IN ** 2 + rng.random() * (R_OUT ** 2 - R_IN ** 2))  # 均匀环带
        a = 2.0 * math.pi * rng.random()
        z = BEAKER_BOTTOM_Z + 0.01 + rng.random() * (WATER_H - 0.02)
        out.append((TUBE_X + r * math.cos(a), TUBE_Y + r * math.sin(a), z))
    return out

BUBBLE_BASE = _gen_beaker_bubbles()

# 水浴/熔化液体材质配方（d2l-liquid-color-recipe：近黑 diffuse + 单通道主导 emissive，
# 防被 CylinderLight 12000 洗白；clear=无色透明水柱）
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)
MELT_COLORS = {
    "clear":  dict(color=(0.72, 0.85, 1.0), opacity=0.55, roughness=0.05, ior=1.33,
                   emissive=None),
    "red":    dict(color=(0.10, 0.03, 0.03), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.2, 0.12, 0.12)),
    "blue":   dict(color=(0.03, 0.05, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 0.30, 2.2)),
    "green":  dict(color=(0.03, 0.10, 0.04), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 2.0, 0.12)),
    "purple": dict(color=(0.12, 0.03, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.0, 0.15, 2.2)),
}
# 玻璃配方（assets 自带 op1.0 不透明，改真玻璃透出水柱/熔化液柱）
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.12, roughness=0.25, ior=1.5)


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided。
    emissive：自发光（高亮有色熔化液柱用）。"""
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
    """建一个材质绑定到多个 prim（气泡组共用）。"""
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


def add_beaker_water(stage):
    """烧杯内水柱（水浴）：蓝半透明柱，可见。底面贴烧杯底、顶到水面 WATER_TOP_Z。"""
    w = UsdGeom.Cylinder.Define(stage, "/World/BeakerWater")
    w.CreateRadiusAttr(WATER_R)
    w.CreateHeightAttr(WATER_H)
    w.CreateAxisAttr("Z")
    w.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, WATER_CZ))
    add_material(stage, w.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    print(f"[effect] BeakerWater visible (r{WATER_R} h{WATER_H} top {WATER_TOP_Z:.4f})")


def add_beaker_bubbles(stage):
    """烧杯水浴内气泡组：30 球在水柱内，初始隐藏，task 加热时 reveal + 上升。"""
    UsdGeom.Xform.Define(stage, "/World/BeakerBubbles")
    bub_prims = []
    for i, (x, y, z) in enumerate(BUBBLE_BASE):
        sp = UsdGeom.Sphere.Define(stage, f"/World/BeakerBubbles/bubble_{i}")
        sp.CreateRadiusAttr(BUBBLE_R)
        sp.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        UsdGeom.Imageable(sp).MakeInvisible()
        bub_prims.append(sp.GetPrim())
    add_shared_material(stage, "/World/BeakerBubbles/bubble_mat",
                        (0.72, 0.85, 1.0), 1.0, bub_prims, roughness=0.3,
                        emissive=(0.7, 1.0, 1.8))
    print(f"[effect] {len(BUBBLE_BASE)} beaker bubbles hidden")


def add_melt_liquid(stage):
    """试管内熔化液柱：/World/TubeMelt_<色>，初始全隐藏。task 按 cfg.melt_color 揭示一根
    （sample_phase=melted 时）。半径 MELT_R、高 MELT_H，底面贴试管底 TUBE_BOTTOM_Z。"""
    for name, m in MELT_COLORS.items():
        geom = UsdGeom.Cylinder.Define(stage, f"/World/TubeMelt_{name}")
        geom.CreateRadiusAttr(MELT_R)
        geom.CreateHeightAttr(MELT_H)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, MELT_CZ))
        translucent = m.get("opacity", 1.0) < 1.0
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] TubeMelt_{name} hidden (r{MELT_R} h{MELT_H})")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃烧杯/试管/水柱无环境反射照不亮。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 2000 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 后 ./textures/ 失效，烘平后相对 textures/ 重指）。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去器材资产自带的残留 DomeLight（/root/env_light），避免双灯压暗环境。"""
    for name, *_ in EQUIP:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[clean] /World/{name} not found, skip")
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")
        if not paths:
            print(f"[clean] no DomeLight in {name}")


def move_lamp_cap(st2):
    """灯帽从灯顶挪到桌边（闭口朝下贴台面）静止位：灯世界 (TUBE_X,TUBE_Y) + 局部
    (0.1086,0.0129,-0.076)，rot180 翻转 → 世界帽位 (lamp_x-0.1086, lamp_y-0.0129)
    = (0.42, STAND_Y-0.0129)。同 B2（2026-08-28 九改：往 -X 避石棉网左边缘 0.4726）。"""
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[clean] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(0.1086, 0.0129, -0.076)
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[clean] cap translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[clean] cap (no translate op) add translate {tuple(tgt)}")


# 酒精灯火焰（同 B2 水滴形，底半球 + 上部锥收尖，迁 /World 顶层）。
FLAME_BASE_Z = TABLE_TOP + 0.091            # 灯芯根部世界 z（火焰底）
FLAME_APEX_Z = GAUZE_Z - 0.00105            # 石棉网底 0.9184（火焰尖刚好碰到，加热烧杯底）
FLAME_OUTER_R = 0.009                       # 外焰肚半径
FLAME_INNER_R = 0.005                       # 内焰（焰心）肚半径
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022   # 内焰 apex（0.913，在外焰内）

def add_droplet_flame(st2, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere（底部圆） + 上部 Cone（收尖），一组两 prim，绕 Z 轴。"""
    zc = z_b + r                       # 球心 = 水滴最宽处
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, zc))
    h = z_a - zc
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, zc + h / 2))
    for prim in (sph, cone):
        pname = prim.GetPath().name
        UsdGeom.Imageable(prim).GetVisibilityAttr().Clear()   # 默认可见（勿 invisible）
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
    print(f"[lamp] droplet {name}: sphere r{r} c{zc:.4f} (bottom {z_b:.4f}) + cone apex {z_a:.4f}")


def rebuild_flames(st2):
    """酒精灯火焰：删灯下引用子 prim，在 /World 顶层重建（flametest 已验证良方）。"""
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                      (0.35, 0.55, 2.40))                          # 外焰偏蓝
    add_droplet_flame(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                      (2.80, 0.55, 0.20))                          # 内焰偏黄
    print(f"[lamp] flames droplet: outer base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"(touches gauze bottom {GAUZE_BOTTOM_Z:.4f})")


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（烘平后 MaterialBindingAPI 未 apply，用 material:binding
    取材质路径再找 shader，同 d3l gen）。"""
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


def fix_beaker_material(st2):
    """烧杯玻璃透明化 + 去反光：beaker.usd 自带 opacity 1.0 不透明（遮住水柱/水浴气泡）
    → op0.12 真玻璃 + ior1.5 + roughness0.25 + doubleSided（同 B2 试管配方）。

    新 beaker.usd 结构为 /World/Beaker/beaker_111x75x116/beaker_111x75x116_008（嵌套 Xform），
    用 PrimRange 递归找 Mesh。"""
    p = st2.GetPrimAtPath("/World/Beaker")
    if not p.IsValid():
        print("[mat] /World/Beaker not found, skip")
        return
    for c in Usd.PrimRange(p):
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.5, "roughness": 0.25}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] beaker glass {c.GetPath()} -> op 0.12 / ior 1.5 / rough 0.25")


def hide_iron_stand_hook(st2):
    """铁架台挂钩隐藏（用户「铁架台上面的挂钩也隐藏或者删除」）。hook = /root/root/hook，
    烘平后位于 /World/IronStand/root/hook（iron_stand.usd 内 /root/root/hook）。"""
    hook = st2.GetPrimAtPath("/World/IronStand/root/hook")
    if not hook.IsValid():
        print("[clean] /World/IronStand/root/hook not found, skip")
        return
    UsdGeom.Imageable(hook).MakeInvisible()
    print("[clean] iron stand hook hidden")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光（同 B2：op 0.35→0.12 + ior 1.5 + roughness 0.25）。"""
    p = st2.GetPrimAtPath("/World/TestTube")
    if not p.IsValid():
        print("[mat] /World/TestTube not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.5, "roughness": 0.25}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] tube glass {c.GetPath()} -> op 0.12 / ior 1.5 / rough 0.25")


def cleanup_dish(st2):
    """表面皿粉末子集重绑皿材质（照 D2-S cleanup_dish）：sample_dish.usd 是双材质皿，
    单 mesh 带 powder/dish 两个 GeomSubset（flametest 残留），真实粉丘由 SamplePowder 摆
    皿上，故把资产自带 powder 子集重绑到皿材质避免皿底显出双份粉。"""
    dish = st2.GetPrimAtPath("/World/SurfaceDish")
    if not dish.IsValid():
        print("[dish] not found, skip")
        return
    for child in list(dish.GetChildren()):
        if child.GetTypeName() == "DomeLight" or "env_light" in child.GetName():
            st2.RemovePrim(child.GetPath())
            print(f"[dish] removed {child.GetPath()}")
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


def brighten_spatula(st2):
    """药匙 = 普通不锈钢（银黑）：metallic 1.0 + low roughness + 深灰 diffuse（照 D2-S）。
    spatula.usd 源材质即此值，B3 场景烘焙后 idempotent 覆写，防黑杆/防发白。"""
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


def verify(st2):
    """自检：打印各器材世界 bbox，断言 2026-08-29 修正后的布局：
    烧杯直立坐网上（points 实测 90mm）、D2-S 样品区复刻（试管/药匙/皿/粉/洗瓶同 d2s 坐标）、
    无试管夹、铁架台挂钩隐藏、加热堆叠整组 -Y 平移。
    beaker.usd 有旋转且 extent 陈旧 → Beaker 用 _points_bbox，其余 axis-aligned 用 BBoxCache。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "AsbestosGauze", "Beaker", "TestTubeRack", "TestTube",
             "Spatula", "SurfaceDish", "SamplePowder", "WashBottle", "Match", "BeakerWater"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        if name == "Beaker":
            b = _points_bbox(st2, f"/World/{name}")   # 旋转+extent 陈旧 → points 实测
            if b is None:
                print(f"[verify] /World/{name} no mesh")
                continue
            mn, mx = b
        else:
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:13s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 铁架台：底座贴台面
    smn, smx = boxes["IronStand"]
    assert abs(smn[2] - TABLE_TOP) < 0.002, f"IronStand base z {smn[2]} != table {TABLE_TOP}"
    # 铁架台挂钩隐藏（用户「挂钩也隐藏或删除」）
    hook = st2.GetPrimAtPath("/World/IronStand/root/hook")
    assert hook.IsValid(), "iron stand hook missing"
    assert UsdGeom.Imageable(hook).ComputeVisibility() == "invisible", "hook should be hidden"
    # 石棉网坐铁环上：网底 >= 环顶(0.918)，网底 - 环顶 <= 2mm
    gmn, gmx = boxes["AsbestosGauze"]
    ring_top = TABLE_TOP + 0.118
    assert gmn[2] >= ring_top - 0.002, f"gauze bottom {gmn[2]} below ring top {ring_top}"
    # 烧杯直立坐石棉网上（points 实测）：底贴网顶、高 90.4mm、无试管夹（TEST: 无 TestTubeClamp）
    bmn, bmx = boxes["Beaker"]
    assert 0 <= bmn[2] - gmx[2] <= 0.0015, f"beaker bottom {bmn[2]} not on gauze top {gmx[2]}"
    assert abs((bmx[2] - bmn[2]) - 0.0904) < 0.003, f"beaker height {bmx[2]-bmn[2]:.4f} != 0.0904"
    assert abs(bmx[2] - BEAKER_TOP_Z) < 0.003, f"beaker top {bmx[2]:.4f} != {BEAKER_TOP_Z}"
    assert abs((bmn[1] + bmx[1]) / 2 - STAND_Y) < 0.003, \
        f"beaker y {(bmn[1]+bmx[1])/2:+.4f} != STAND_Y {STAND_Y} (stack shifted)"
    assert not st2.GetPrimAtPath("/World/TestTubeClamp").IsValid(), \
        "TestTubeClamp should be removed (user: 不需要试管夹)"
    # 烧杯水柱在烧杯内：水面 < 烧杯口、水底 > 烧杯底（d3l 流动液体 Cylinder）
    wmn, wmx = boxes["BeakerWater"]
    assert wmn[2] > bmn[2] - 0.002, f"beaker water bottom {wmn[2]} below beaker bottom {bmn[2]}"
    assert wmx[2] < bmx[2] - 0.005, f"beaker water top {wmx[2]} too close to beaker top {bmx[2]}"
    # 试管架贴台面（D2-S 位）、试管立在架孔里（底贴洞底 0.8060，无试管夹）
    rmn, rmx = boxes["TestTubeRack"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"rack bottom {rmn[2]} not on table"
    assert abs((rmn[0] + rmx[0]) / 2 - RACK_X) < 0.01 and abs((rmn[1] + rmx[1]) / 2 - RACK_Y) < 0.01, \
        f"rack center not at D2-S ({RACK_X},{RACK_Y})"
    tmn, tmx = boxes["TestTube"]
    assert abs(tmn[2] - RACK_HOLE_BOTTOM) < 0.003, f"tube bottom {tmn[2]:.4f} not in rack hole {RACK_HOLE_BOTTOM}"
    assert rmn[0] < tmn[0] < rmx[0], "tube x not within rack x"
    assert abs((tmn[1] + tmx[1]) / 2 - RACK_TUBE_Y) < 0.003, \
        f"tube y center {((tmn[1]+tmx[1])/2):.4f} != {RACK_TUBE_Y}"
    # 药匙立架中心孔竖插（D2-S 位）：勺底贴洞底、xy 在架范围
    smn, smx = boxes["Spatula"]
    assert abs(smn[2] - RACK_HOLE_BOTTOM) < 0.003, f"spatula bottom {smn[2]:.4f} not in rack hole"
    assert rmn[0] < smn[0] and smx[0] < rmx[0], "spatula x not within rack"
    assert rmn[1] < smn[1] and smx[1] < rmx[1], "spatula y not within rack"
    # 酒精灯灯芯顶低于石棉网底（火焰区留白 >= 1.5cm）
    lmn, lmx = boxes["AlcoholLamp"]
    wick_top = TABLE_TOP + 0.1005
    assert wick_top + 0.015 < gmn[2], f"flame gap {gmn[2] - wick_top:.3f} < 1.5cm"
    # 灯帽静止位：帽底贴台面、中心 (0.42, STAND_Y-0.0129)（随灯 -Y 平移）
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn2, cmx2 = r.GetMin(), r.GetMax()
    print(f"[verify] cap     min({cmn2[0]:+.4f},{cmn2[1]:+.4f},{cmn2[2]:+.4f}) "
          f"max({cmx2[0]:+.4f},{cmx2[1]:+.4f},{cmx2[2]:+.4f})")
    assert abs(cmn2[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn2[2]} not on table"
    cx, cy = 0.5 * (cmn2[0] + cmx2[0]), 0.5 * (cmn2[1] + cmx2[1])
    assert abs(cx - 0.42) < 0.005, f"cap center x {cx:+.4f} != 0.42"
    assert abs(cy - (STAND_Y - 0.0129)) < 0.005, \
        f"cap center y {cy:+.4f} != {STAND_Y - 0.0129:.4f}"
    # 玻璃皿贴台（D2-S 位）、粉丘贴皿顶（碗形皿凹坑内 → 底略低于皿顶缘）、洗瓶 D2-S 位、
    # 火柴抬高 12mm
    dsh = boxes["SurfaceDish"]
    assert abs(dsh[0][2] - TABLE_TOP) < 0.002, f"dish bottom {dsh[0][2]} not on table"
    assert abs((dsh[0][0] + dsh[1][0]) / 2 - DISH_X) < 0.01 and abs((dsh[0][1] + dsh[1][1]) / 2 - DISH_Y) < 0.01, \
        f"dish center not at D2-S ({DISH_X},{DISH_Y})"
    pw = boxes["SamplePowder"]
    assert pw[0][2] > dsh[0][2] - 0.003, f"powder bottom {pw[0][2]:.4f} below dish base {dsh[0][2]:.4f}"
    assert pw[1][2] > dsh[1][2] - 0.002, f"powder top {pw[1][2]:.4f} not above dish rim {dsh[1][2]:.4f}"
    pcx, pcy = 0.5 * (pw[0][0] + pw[1][0]), 0.5 * (pw[0][1] + pw[1][1])
    assert abs(pcx - DISH_X) < 0.01 and abs(pcy - DISH_Y) < 0.01, \
        f"powder center ({pcx:.3f},{pcy:.3f}) off dish ({DISH_X},{DISH_Y})"
    wb = boxes["WashBottle"]
    assert abs(wb[0][2] - TABLE_TOP) < 0.002, f"washbottle bottom {wb[0][2]} not on table"
    assert abs((wb[0][0] + wb[1][0]) / 2 - 0.4097) < 0.01, "washbottle x not at D2-S pos"
    assert abs((wb[0][1] + wb[1][1]) / 2 - WASHB_Y) < 0.01, "washbottle y not at D2-S pos"
    mt = boxes["Match"]
    assert mt[0][2] > TABLE_TOP + 0.010, f"match bottom {mt[0][2]} not raised 12mm above table"
    # 效果 prim：烧杯水柱可见、水浴气泡组齐且隐藏、熔化液柱 5 根隐藏
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/BeakerWater")).ComputeVisibility() != "invisible", \
        "BeakerWater should be visible"
    bub = st2.GetPrimAtPath("/World/BeakerBubbles")
    assert bub.IsValid(), "BeakerBubbles missing"
    nb = sum(1 for c in bub.GetChildren() if c.GetTypeName() == "Sphere")
    assert nb == len(BUBBLE_BASE), f"bubbles count {nb} != {len(BUBBLE_BASE)}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/BeakerBubbles/bubble_0")).ComputeVisibility() == "invisible", \
        "bubble_0 should be hidden"
    for name in MELT_COLORS:
        mp = st2.GetPrimAtPath(f"/World/TubeMelt_{name}")
        assert mp.IsValid(), f"TubeMelt_{name} missing"
        assert UsdGeom.Imageable(mp).ComputeVisibility() == "invisible", f"TubeMelt_{name} should be hidden"
        assert abs(UsdGeom.Cylinder(mp).GetRadiusAttr().Get() - MELT_R) < 1e-9, f"TubeMelt_{name} r != {MELT_R}"
    # 火焰迁 /World 顶层（外焰 apex 碰石棉网底、内焰在外焰内、外焰 B 主导、内焰 R 主导）
    for name, r, z_b, z_a in (("flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z),
                              ("flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z)):
        f = st2.GetPrimAtPath(f"/World/{name}")
        assert f.IsValid() and f.GetTypeName() == "Cone", f"{name} top-level cone missing"
        assert UsdGeom.Imageable(f).ComputeVisibility() != "invisible", f"{name} should be default visible"
        h = z_a - (z_b + r)
        assert abs(UsdGeom.Cone(f).GetHeightAttr().Get() - h) < 0.0005, f"{name} cone height wrong"
        sph = st2.GetPrimAtPath(f"/World/{name}_sphere")
        assert sph.IsValid() and sph.GetTypeName() == "Sphere", f"{name}_sphere missing"
        sz = UsdGeom.Xformable(sph).GetOrderedXformOps()[0].Get()[2]
        assert abs(sz - (z_b + r)) < 0.002, f"{name}_sphere center z {sz} != {z_b + r}"
        sh = UsdShade.Shader(st2.GetPrimAtPath(f"/World/{name}_mat/Shader"))
        c = sh.GetInput("emissiveColor").Get()
        if name == "flame_outer":
            assert c[2] > c[0] and c[2] > 1.0, f"outer flame should be blue-dominant, got {tuple(c)}"
            assert abs(z_a - gmn[2]) < 0.004, f"outer flame apex {z_a} not at gauze bottom {gmn[2]}"
        else:
            assert c[0] > c[2] and c[0] > 1.0, f"inner flame should be yellow-dominant, got {tuple(c)}"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"
    print(f"[verify] OK: 台面贴底 / 网坐环上 / 烧杯直立坐网上(90mm,堆叠移y={STAND_Y}) / 水柱在杯内 / "
          f"D2-S样品区复刻(架/管/勺/皿/粉/洗瓶) / 挂钩隐藏 / 无试管夹 / 灯在网下 / 帽随灯移 / "
          f"火柴抬高12mm / 水浴气泡组齐({nb}泡)隐藏 / 熔化液柱5根隐藏 / 火焰迁/World顶层")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot180 in EQUIP:
        add_equip(stage, name, asset, t, scale, rot180)
    add_beaker_water(stage)
    add_beaker_bubbles(stage)
    add_melt_liquid(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    move_lamp_cap(st2)
    rebuild_flames(st2)          # 火焰迁到 /World 顶层
    brighten_lights(st2)
    fix_env_light(st2)
    fix_beaker_material(st2)     # 烧杯玻璃透明化（水柱/气泡透出）
    fix_tube_material(st2)       # 试管玻璃透明化（熔化液柱透出）
    hide_iron_stand_hook(st2)    # 铁架台挂钩隐藏（用户第5条）
    cleanup_dish(st2)            # 皿粉末子集重绑皿材质（D2-S 同款，防皿底显双份粉）
    brighten_spatula(st2)        # 药匙不锈钢（D2-S 同款，防黑杆/防发白）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
