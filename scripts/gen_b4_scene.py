# -*- coding: utf-8 -*-
"""生成 b4_ice_bath.usd —— B4 冰浴/冷却场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：

B4 冰浴（2026-08-29 用户逐字定稿）：
  「不需要温度计不需要试管夹（机械臂来夹），然后烧杯用beaker.usd，整个场景器材要散，
    烧杯里面本来就装着6个冰块（如果冰块太大，你就修改冰块usd让它再小一点），然后洗瓶
    放在烧杯正-x方向，试管放在试管架里面，试管里面本来就有药品（药品模仿d3l）」

- 烧杯（beaker.usd，Ø75×高90mm 直立，底 z=0 顶 z=0.0904）放台面中央，内装 6 块冰块
  （ice_cube.usd，用户下载 Sketchfab 真实冰块，fix_ice_cube.py 已缩到 1.5cm 保留原贴图）。
- 洗瓶（wash_bottle.usd，rot180 红嘴朝 +X）放烧杯正 -X 方向（同 y，x 差 25cm）。
- 试管（test_tube.usd，Ø19.2×153mm）立试管架（test_tube_rack.usd）前排左孔
  （孔心=架+(-0.021,+0.116)，底面 z=0.806=架z−0.0905），管内预装药品液柱
  /World/TubeDrug（模仿 d3l：半透明光洁液柱，visible，r 略小于管内缘）。
- 无温度计、无试管夹、无加热堆叠（铁架台/酒精灯/石棉网/火柴/灯帽全不要）——器材散开。
- 玻璃透明化：beaker（自带 op1.0 不透明会遮住冰块）+ test_tube（op0.35 磨砂遮药品）
  → 真玻璃 op0.12 / ior1.5 / rough0.25 / doubleSided（同 B3/D3-L）。
- 器材资产自带残留 DomeLight（beaker/rack/wash_bottle 的 /root/env_light）→ remove 掉，
  再统一加场景 DomeLight（env_bright.png）照亮玻璃件。

用法：python scripts/gen_b4_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil

from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b4_ice_bath")
OUT = os.path.join(SCENE_DIR, "b4_ice_bath.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- 布局（2026-08-29 用户定稿；器材散开，无温度计/试管夹）----
# 烧杯放台面中央（机械臂可伸入冰浴插试管），洗瓶在烧杯正 -X（同 y 差 25cm），
# 试管架在烧杯左前斜角（散开不挤一起）。
BEAKER_X, BEAKER_Y = 0.45, 0.10
BEAKER_BOTTOM_Z = TABLE_TOP                 # 烧杯底贴台面（资产 min z=0 → translate z=0.80）
BEAKER_TOP_Z = BEAKER_BOTTOM_Z + 0.0904     # 0.8904（烧杯口）

# 洗瓶：烧杯正 -X 方向（同 y），rot180 红嘴朝 +X（正对烧杯，后续挤水可入杯）
WASHB_X, WASHB_Y = 0.20, 0.10

# 试管架 + 试管：架贴台面（资产 min z=-0.0965 → z=0.8965），试管插前排左孔
RACK_X, RACK_Y = 0.30, 0.36
RACK_TZ = TABLE_TOP + 0.0965                 # 0.8965
HOLE_BOTTOM = RACK_TZ - 0.0905               # 0.8060（孔洞底，管底贴此）
TUBE_X = RACK_X - 0.021                      # 0.279（近侧左孔孔心，d3l 校准）
TUBE_Y = RACK_Y - 0.119                      # 0.241（近侧/前排孔=最靠 -Y 机器人侧；
                                             #  2026-08-30 用户「够不到 → 放 -y 侧」，d2s/b3 同款近孔）
TUBE_BOTTOM_Z = HOLE_BOTTOM
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.1533        # 0.9593

# 冰块：烧杯内 6 块（fix_ice_cube 已缩到 ~1.5cm，底 z=0 顶 z=0.0121）。
# 烧杯内底 ≈ 杯外底 0.80 + 壁厚 ~2mm → 冰底 0.802；底层 4 块错开 + 顶层 2 块压上。
ICE_OFFSETS = [
    # (dx, dy, z底, yaw°)
    (-0.012, -0.010, 0.8020, 0),
    ( 0.012, -0.010, 0.8020, 90),
    ( 0.000,  0.012, 0.8020, 30),
    ( 0.000, -0.004, 0.8020, 150),
    (-0.008,  0.004, 0.8135, 200),   # 二层（底下 12mm 上叠）
    ( 0.008,  0.004, 0.8135, 0),     # 二层
]

# 药品液柱（模仿 d3l）：管内液体，r 略小于管内缘、h 40mm 贴管底可见。
# 预烘焙多色变体（2026-08-30 用户「写成终端接口像d3l一样还要有溶液颜色和晶体颜色」）：
#   澄清溶液 TubeDrug_<色>（冷却前）→ 浑浊溶液 TubeCloud_<色>（冷却析出后）+ 晶体
#   TubeCrystal_<色>（沉管底）。headless 下运行时改材质不渲染，故预烘焙，task 按
#   cfg.liquid_color / cfg.crystal_color 切换 visibility。配方同 d2s SOLUBILITY_COLORS
#   （近黑 diffuse + 单通道主导 emissive，CylinderLight 12000 下才不被洗白）。
DRUG_R = 0.0086                         # 液柱半径（< 管内缘 ~0.009）
DRUG_H = 0.040
DRUG_CZ = TUBE_BOTTOM_Z + DRUG_H / 2    # 0.826（澄清/浑浊液柱中心，随管平移）
CRYSTAL_R = 0.008                       # 晶体层半径（≈液柱内，沉管底）
CRYSTAL_H = 0.014
CRYSTAL_CZ = TUBE_BOTTOM_Z + CRYSTAL_H / 2   # 0.813（晶体沉管底，略抬高更醒目）
FOG_R = 0.0105                          # 外壁雾层半径（> 试管外径 0.0096，包住管身）
FOG_H = 0.12
FOG_CZ = TUBE_BOTTOM_Z + 0.06           # 0.866（雾层包管身中下段）
FOG_OPACITIES = (0.15, 0.35, 0.55)      # 起雾 3 档渐浓（visibility 切换模拟冷凝渐变）

# 溶液/晶体颜色配方（d2s SOLUBILITY_COLORS 同款：近黑 diffuse + 单通道主导 emissive）
LIQUID_COLORS = {
    "clear":  dict(diffuse=(0.90, 0.95, 1.0), opacity=0.10, roughness=0.1, ior=1.33),
    "white":  dict(diffuse=(0.93, 0.93, 0.94), opacity=1.0, roughness=0.5),
    "red":    dict(diffuse=(0.10, 0.03, 0.03), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.2, 0.12, 0.12)),
    "blue":   dict(diffuse=(0.03, 0.05, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 0.30, 2.2)),
    "green":  dict(diffuse=(0.03, 0.10, 0.04), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 2.0, 0.12)),
    "purple": dict(diffuse=(0.12, 0.03, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.0, 0.15, 2.2)),
}
# 浑浊 3 档（每色）：溶液色 → 乳白 渐变（0.35/0.65/0.90 浑浊度），同液柱几何。
# 冷却时档 1→2→3 渐显（悬浮细晶渐多 → 溶液变浑浊），升温时 3→2→1 渐隐（细晶溶解回澄清）。
# 用户「浑浊应该是渐变（特别慢）」——预烘焙多档，task 按冷却/升温进度切 visibility。
_CLOUD_MILK_D = (0.82, 0.80, 0.74)
_CLOUD_MILK_E = (1.0, 0.95, 0.80)
CLOUD_LEVELS = (0.35, 0.65, 0.90)


def _cloud_recipe(r, frac):
    """溶液色 r 混入 frac 比例乳白 → 浑浊档配方（opacity 1.0 保证 headless 可见）。"""
    return dict(
        diffuse=tuple((1 - frac) * r["diffuse"][i] + frac * _CLOUD_MILK_D[i] for i in range(3)),
        opacity=1.0,
        roughness=0.85,
        emissive=tuple((1 - frac) * (r.get("emissive") or (0.9, 0.9, 0.9))[i]
                       + frac * _CLOUD_MILK_E[i] for i in range(3)),
    )


# 晶体（沉管底）：比溶液更饱和（emissive ×1.8 更强穿透半透明浑浊、升温后清晰可见；
# 白晶体 = 亮白）。晶体颜色集不含 clear（晶体不可能是无色），crystal_color 选项只有
# none/white/red/blue/green/purple。
CRYSTAL_COLORS = {
    c: dict(r, emissive=(tuple(v * 1.8 for v in r["emissive"]) if r.get("emissive") else
                          (1.6, 1.6, 1.6)))
    for c, r in LIQUID_COLORS.items() if c != "clear"
}

# 水流/液面效果（挤水动画：水滴从红嘴尖坠入烧杯口 + 烧杯内液面上涨，2026-08-30 用户
# 「机械臂往里面挤入液体要真实…冰块浮起来」；值须与 meta_actions/constants.py 一致）
BEAKER_INNER_BOTTOM_Z = TABLE_TOP + 0.002   # 0.802（烧杯壁厚 2mm，液面/冰块坐此）
WATER_DROPS = 16
WATER_DROP_R = 0.004
WATER_DROP_COLOR = (0.85, 0.88, 0.92)
LIQUID_R = 0.030
LIQUID_H0 = 0.004
LIQUID_COLOR = (0.85, 0.88, 0.92)
LIQUID_OPACITY = 0.35

# (prim, asset_file, translate, rot180)   tz 显式给定（beaker/rack/tube 底贴面精确值）
EQUIP = [
    ("Beaker", "beaker.usd", (BEAKER_X, BEAKER_Y, BEAKER_BOTTOM_Z), False),
    ("WashBottle", "wash_bottle.usd", (WASHB_X, WASHB_Y, TABLE_TOP), True),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, RACK_TZ), False),
    ("TestTube", "test_tube.usd", (TUBE_X, TUBE_Y, TUBE_BOTTOM_Z), False),
]


def _points_bbox(st2, prim_path):
    """世界坐标 points-based 包围盒（避开 BBoxCache 的 extent 陈旧/旋转失真）。

    beaker.usd mesh extent 是旋转前局部 bbox，BBoxCache 会把直立烧杯误报 160mm（真身
    90mm）→ 带旋转的器材用实际 points 求世界 bbox。返回 (min, max) 或 None。"""
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


def add_equip(stage, name, asset, t, rot180=False):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(os.path.abspath(os.path.join(EQ, asset)))
    tx, ty, tz = t
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot180:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (" rot180" if rot180 else ""))


def add_ice_cubes(stage):
    """烧杯内 6 块冰块：/World/Ice_{0..5}，translate+绕 Z yaw，底层贴烧杯内底。"""
    for i, (dx, dy, z, yaw) in enumerate(ICE_OFFSETS):
        prim = UsdGeom.Xform.Define(stage, f"/World/Ice_{i}")
        prim.GetPrim().GetReferences().AddReference(
            os.path.abspath(os.path.join(EQ, "ice_cube.usd")))
        prim.AddTranslateOp().Set(Gf.Vec3d(BEAKER_X + dx, BEAKER_Y + dy, z))
        if yaw:
            prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, yaw))
        print(f"[ice] Ice_{i} at ({BEAKER_X + dx:.3f}, {BEAKER_Y + dy:.3f}, {z}) yaw {yaw}")


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided。"""
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


def add_tube_phenomenon_variants(stage):
    """试管内现象效果 prim（全部初始隐藏，task 按 cfg.liquid_color/crystal_color 驱动）：
      TubeDrug_<色>         澄清溶液（溶液本色，冷却前/升温后）
      TubeCloud_<色>_<1..3> 浑浊 3 档（冷却渐显/升温渐隐，溶液色→乳白渐变）
      TubeCrystal_<色>      晶体层（沉管底，晶体色，强 emissive 透出浑浊）
      TubeFog_<1..3>        外壁雾层（提出后起雾，3 档渐浓）
    几何中心：液柱/云 0.826、晶体 0.813、雾 0.866（均相对试管孔 (TUBE_X,TUBE_Y)，随管平移）。"""
    def make_cyl(name, r, h, t, recipe):
        geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), recipe["diffuse"], recipe.get("opacity", 1.0),
                     roughness=recipe.get("roughness", 0.5), ior=recipe.get("ior"),
                     emissive=recipe.get("emissive"))
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} hidden at {t}")

    for c, m in LIQUID_COLORS.items():
        make_cyl(f"TubeDrug_{c}", DRUG_R, DRUG_H, (TUBE_X, TUBE_Y, DRUG_CZ), m)
    for c, r in LIQUID_COLORS.items():
        for lv, frac in enumerate(CLOUD_LEVELS, start=1):
            make_cyl(f"TubeCloud_{c}_{lv}", DRUG_R, DRUG_H, (TUBE_X, TUBE_Y, DRUG_CZ),
                     _cloud_recipe(r, frac))
    for c, m in CRYSTAL_COLORS.items():
        make_cyl(f"TubeCrystal_{c}", CRYSTAL_R, CRYSTAL_H, (TUBE_X, TUBE_Y, CRYSTAL_CZ), m)
    for i, op in enumerate(FOG_OPACITIES, start=1):
        make_cyl(f"TubeFog_{i}", FOG_R, FOG_H, (TUBE_X, TUBE_Y, FOG_CZ),
                 dict(diffuse=(0.85, 0.88, 0.92), opacity=op, roughness=0.3, ior=1.31))
    print(f"[effect] tube phenomenon variants hidden "
          f"({len(LIQUID_COLORS)} drug / {len(LIQUID_COLORS)}x{len(CLOUD_LEVELS)} cloud / "
          f"{len(CRYSTAL_COLORS)} crystal / {len(FOG_OPACITIES)} fog)")


def add_effects(stage):
    """挤水/液面效果：/World/WaterStream 父节点 + 16 颗水滴（初始隐），/World/BeakerLiquid
    烧杯内液面圆柱（初始隐，半径略小于烧杯内径，底贴烧杯内底 0.802）。task 运行时驱动
    显隐与高度（挤水时水滴沿抛物线坠落、液面随落定水滴上涨、冰块浮起）。"""
    stream = UsdGeom.Xform.Define(stage, "/World/WaterStream")
    UsdGeom.Imageable(stream.GetPrim()).MakeInvisible()
    for i in range(WATER_DROPS):
        sp = UsdGeom.Sphere.Define(stage, f"/World/WaterStream/Drop_{i}")
        sp.CreateRadiusAttr(WATER_DROP_R)
        add_material(stage, sp.GetPrim(), WATER_DROP_COLOR, 0.9, roughness=0.1)
        UsdGeom.Imageable(sp.GetPrim()).MakeInvisible()
    liq = UsdGeom.Cylinder.Define(stage, "/World/BeakerLiquid")
    liq.CreateRadiusAttr(LIQUID_R)
    liq.CreateHeightAttr(LIQUID_H0)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(BEAKER_X, BEAKER_Y, BEAKER_INNER_BOTTOM_Z + LIQUID_H0 / 2.0))
    add_material(stage, liq.GetPrim(), LIQUID_COLOR, LIQUID_OPACITY,
                 roughness=0.05, ior=1.33, double_sided=True)
    UsdGeom.Imageable(liq.GetPrim()).MakeInvisible()
    print(f"[effect] WaterStream ({WATER_DROPS} drops) + BeakerLiquid "
          f"(r{LIQUID_R} h0{LIQUID_H0}) added (hidden)")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃烧杯/试管/冰块在无环境反射下照不亮。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 2000 照不亮细玻璃件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（d2s/d3l/b1 同款）：lab_clean 默认
    x=2.1 的 100m 巨型灯悬在工作区/相机视野边缘，RTX 自动曝光被压爆 → 全黑（或过曝
    洗白）；移到 x=-10 远离相机，场景才正常受光（b1 注释："去试管玻璃反光，现象看得清"）。"""
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


def strip_dome_lights(st2):
    """扫除残留的嵌套 DomeLight，只保留 /World/env_light（d2s/d3l/b1 同款）：
    器材资产自带的近黑 env_light 会把环境压暗，RTX 里多个 DomeLight 叠加取暗。"""
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
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效），烘平后
    场景文件在 SCENE_DIR，相对 textures/ 能正确指向场景目录下的贴图。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去器材资产自带的残留 DomeLight（beaker/rack/wash_bottle 的 /root/env_light），
    避免与场景 env_light 双灯压暗。"""
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


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（烘平后 material:binding relationship 取材质）。"""
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
    """烧杯玻璃透明化：beaker.usd 自带 opacity 1.0 不透明（遮住内装冰块）
    → op0.12 真玻璃 + ior1.5 + roughness0.25 + doubleSided（同 B3 配方）。"""
    p = st2.GetPrimAtPath("/World/Beaker")
    if not p.IsValid():
        print("[mat] /World/Beaker not found, skip")
        return
    for c in Usd.PrimRange(p):
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.0, "roughness": 0.5, "specular": 0.0}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] beaker glass {c.GetPath()} -> op 0.12 / ior 1.0 / rough 0.5")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：test_tube.usd 自带 opacity 0.35 → 0.12（更透明，
    内部药品液柱看得清）+ ior 1.5 + roughness 0.25 + doubleSided（同 D3-L）。"""
    p = st2.GetPrimAtPath("/World/TestTube")
    if not p.IsValid():
        print("[mat] /World/TestTube not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.5, "roughness": 0.25, "specular": 0.0}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] tube glass {c.GetPath()} -> op 0.12 / ior 1.5 / rough 0.25 / doubleSided")


def fix_ice_visible(st2, indices=(3, 4, 5)):
    """把原色冰块改「可见」（默认 Ice_3/4/5，非红色的 3 块）：冰块原本半透明 + 无色，
    被 DomeLight 2000 + CylinderLight 12000 洗到看不见（和透明水/玻璃烧杯一个色，无对比）。
    三管齐下保持「冰」的白/冷色调但明显可见（2026-08-30 用户「怎么让不是红色的变得
    明显…最后肯定要原本的颜色」）：
      ① opacity 0.9     → 从近透明提到接近实体，读作固体冰块
      ② roughness 0.35  → 磨砂散射光，呈现哑光白（比镜面透光更容易被相机捕获）
      ③ emissive 冷白 0.25 → 轻微自发光，不依赖场景打光，暗处/背光也醒目
    冷白 diffuse (0.80,0.86,0.95) + ior 1.31（冰折射率）保持冰的质感，非塑料非红。
    最终版：删掉 fix_ice_red，改对全部 6 块调用本函数（indices=range(6)）。"""
    for i in indices:
        p = st2.GetPrimAtPath(f"/World/Ice_{i}")
        if not p.IsValid():
            print(f"[mat] Ice_{i} not found, skip")
            continue
        for c in Usd.PrimRange(p):
            if c.GetTypeName() != "Mesh":
                continue
            add_material(st2, c, (0.80, 0.86, 0.95), 0.9, roughness=0.35, ior=1.31,
                         emissive=(0.20, 0.22, 0.28))
            print(f"[mat] Ice_{i} {c.GetPath()} -> frosted ice (visible, original tone)")


def verify(st2):
    """自检：打印各器材/冰块世界 bbox，断言 2026-08-29 布局：
    烧杯直立坐台面（points 实测 90mm）、6 冰块在烧杯内、洗瓶在烧杯正 -X（同 y）、
    试管立架前排左孔（底贴洞底 0.806）、药品液柱在管内、无温度计/试管夹。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["Beaker", "WashBottle", "TestTubeRack", "TestTube"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        if name == "Beaker":
            b = _points_bbox(st2, f"/World/{name}")
            if b is None:
                print(f"[verify] /World/{name} no mesh")
                continue
            mn, mx = b
        else:
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:12s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 烧杯直立坐台面：底贴 0.80、高 90.4mm、杯心在 (BEAKER_X, BEAKER_Y)
    bmn, bmx = boxes["Beaker"]
    assert abs(bmn[2] - TABLE_TOP) < 0.002, f"beaker bottom {bmn[2]} not on table"
    assert abs((bmx[2] - bmn[2]) - 0.0904) < 0.003, f"beaker height {bmx[2]-bmn[2]:.4f} != 0.0904"
    assert abs((bmn[0] + bmx[0]) / 2 - BEAKER_X) < 0.02 and \
        abs((bmn[1] + bmx[1]) / 2 - BEAKER_Y) < 0.02, "beaker center off (BEAKER_X,Y)"
    # 6 块冰块：都在烧杯 xy 范围（Ø75 → ±0.0377 加杯壁余量 ±0.012）内、底 >= 烧杯底
    for i in range(6):
        p = st2.GetPrimAtPath(f"/World/Ice_{i}")
        assert p.IsValid(), f"Ice_{i} missing"
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        imn, imx = r.GetMin(), r.GetMax()
        assert imn[0] > bmn[0] - 0.012 and imx[0] < bmx[0] + 0.012, \
            f"Ice_{i} x outside beaker: {imn[0]:.3f}..{imx[0]:.3f}"
        assert imn[1] > bmn[1] - 0.012 and imx[1] < bmx[1] + 0.012, \
            f"Ice_{i} y outside beaker: {imn[1]:.3f}..{imx[1]:.3f}"
        assert imn[2] >= bmn[2] - 0.003, f"Ice_{i} bottom {imn[2]:.4f} below beaker bottom"
        print(f"[verify] Ice_{i} inside beaker OK (z {imn[2]:.4f}..{imx[2]:.4f})")
    # 洗瓶在烧杯正 -X：瓶中心 x < 杯中心 x、y 同（±2cm 内）
    wb = boxes["WashBottle"]
    wbcx = (wb[0][0] + wb[1][0]) / 2
    wbcy = (wb[0][1] + wb[1][1]) / 2
    assert wbcx < BEAKER_X, f"wash bottle x {wbcx:.3f} not at -X of beaker {BEAKER_X}"
    assert abs(wbcy - BEAKER_Y) < 0.02, f"wash bottle y {wbcy:.3f} not same as beaker {BEAKER_Y}"
    # 试管架贴台面、试管立前排左孔（底贴洞底 0.806）
    rmn, rmx = boxes["TestTubeRack"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"rack bottom {rmn[2]} not on table"
    tmn, tmx = boxes["TestTube"]
    assert abs(tmn[2] - HOLE_BOTTOM) < 0.003, f"tube bottom {tmn[2]:.4f} not in rack hole"
    assert rmn[0] < tmn[0] and tmx[0] < rmx[0], "tube x not within rack x"
    assert rmn[1] < tmn[1] and tmx[1] < rmx[1], "tube y not within rack y"
    # 药品液柱变体：以 default 色（clear）验几何在试管内（变体全隐藏，bbox 仍可算）：
    # 底面贴管底、顶 < 管口、r < 管内缘
    dp = st2.GetPrimAtPath("/World/TubeDrug_clear")
    assert dp.IsValid(), "TubeDrug_clear missing"
    d = bc.ComputeWorldBound(dp).ComputeAlignedRange()
    dmn, dmx = d.GetMin(), d.GetMax()
    assert dmn[2] >= tmn[2] - 0.002, f"drug bottom {dmn[2]} below tube bottom {tmn[2]}"
    assert dmx[2] < tmx[2] - 0.002, f"drug top {dmx[2]} too close to tube mouth {tmx[2]}"
    assert dmx[0] - dmn[0] <= 0.019, "drug r > tube inner (Ø18/2=0.009)"
    # 无温度计 / 无试管夹
    assert not st2.GetPrimAtPath("/World/Thermometer").IsValid(), "Thermometer should be absent"
    assert not st2.GetPrimAtPath("/World/TestTubeClamp").IsValid(), \
        "TestTubeClamp should be absent"
    # 效果 prim：现象变体全部存在且初始隐藏（task reset 按 cfg 色显示对应变体）
    for c in LIQUID_COLORS:
        assert st2.GetPrimAtPath(f"/World/TubeDrug_{c}").IsValid(), f"TubeDrug_{c} missing"
        for lv in range(1, 4):
            assert st2.GetPrimAtPath(f"/World/TubeCloud_{c}_{lv}").IsValid(), \
                f"TubeCloud_{c}_{lv} missing"
        assert UsdGeom.Imageable(st2.GetPrimAtPath(f"/World/TubeDrug_{c}")).ComputeVisibility() == "invisible", \
            f"TubeDrug_{c} should be invisible initially"
    for c in CRYSTAL_COLORS:
        assert st2.GetPrimAtPath(f"/World/TubeCrystal_{c}").IsValid(), f"TubeCrystal_{c} missing"
    for i in range(1, 4):
        assert st2.GetPrimAtPath(f"/World/TubeFog_{i}").IsValid(), f"TubeFog_{i} missing"
    print(f"[verify] phenomenon variants OK "
          f"({len(LIQUID_COLORS)} drug / {len(LIQUID_COLORS)}x{len(CLOUD_LEVELS)} cloud / "
          f"{len(CRYSTAL_COLORS)} crystal / 3 fog, all hidden)")
    # 效果 prim：水流父节点 + 烧杯内液面圆柱存在，且液面落在烧杯 xy 范围内
    assert st2.GetPrimAtPath("/World/WaterStream").IsValid(), "WaterStream missing"
    assert st2.GetPrimAtPath("/World/BeakerLiquid").IsValid(), "BeakerLiquid missing"
    lr = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/BeakerLiquid")).ComputeAlignedRange()
    assert abs((lr.GetMin()[0] + lr.GetMax()[0]) / 2 - BEAKER_X) < 0.02, "liquid not under beaker x"
    assert abs((lr.GetMin()[1] + lr.GetMax()[1]) / 2 - BEAKER_Y) < 0.02, "liquid not under beaker y"
    print("[verify] WaterStream + BeakerLiquid OK")
    print("[verify] OK: 烧杯直立坐台面(90mm) / 6冰块在杯内 / 洗瓶在杯正-X(同y) / "
          "试管立架前排左孔(底0.806) / 药品液柱在管内 / 无温度计·试管夹 / 水流+液面已建")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, rot180 in EQUIP:
        add_equip(stage, name, asset, t, rot180)
    add_ice_cubes(stage)
    add_tube_phenomenon_variants(stage)
    add_effects(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    strip_dome_lights(st2)         # 全局扫除残留 DomeLight（d2s/d3l/b1 同款）
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)   # 移巨型 CylinderLight 远离相机视野（关键，缺了全黑）
    fix_env_light(st2)
    fix_beaker_material(st2)   # 烧杯玻璃透明化（冰块透出）
    fix_tube_material(st2)     # 试管玻璃透明化（药品液柱透出）
    fix_ice_visible(st2, indices=range(6))   # 6 块冰块全恢复原色（磨砂白 + 轻自发光，删 fix_ice_red）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
