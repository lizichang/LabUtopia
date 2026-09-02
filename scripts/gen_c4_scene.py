# -*- coding: utf-8 -*-
"""生成 c4_combustion_liquid.usd —— C4 燃烧试验（液体样品）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）。

C4 燃烧试验（液体）：滴管从药品瓶吸液 → 滴入燃烧匙碗 → 点燃酒精灯 → 燃烧匙碗
伸入火焰 → 观察液体燃烧现象（火焰变色）。= C3 固体版燃烧骨架（定稿：无铁架台、
燃烧匙靠试管架）+ d3l 药品瓶滴加：

骨架（照 C3 实际定稿 usd 的 xform，见 c3_combustion_solid.usd）：
- 试管架（test_tube_rack.usd）：后右，燃烧匙斜靠支点 + 滴管插后排孔。
- 燃烧匙（combustion_spoon.usd，碗朝上 + 把手斜向上）：碗贴台面、把手斜靠试管架
  左前侧（translate 0.636,0.3093,0.8068，无旋转，原厂即"碗朝上把手斜向上"）。
- 酒精灯（alcohol_lamp.usd，rot180）+ 火柴（match.usd）：点火；灯帽摘旁备灭火。
- 火焰：迁到 /World 顶层（初始隐藏，C4 灯未点燃，task 点火后显示）。

液体样品区（模仿 d3l：药品瓶 + 滴管，取代 C3 固体挖粉区）：
- 删 C3 的 IronStand / Spatula / SurfaceDish / SamplePowder / TestTube。
- SampleBottle（sample_bottle.usd，药品瓶装液体样品，auto 贴台 (0.50,0.55)；
  stopper 摘下倒放瓶旁桌面（密封面朝上），瓶玻璃透明化，瓶内 SampleLiquid 半瓶液可见）。
- Dropper（dropper.usd）立插试管架右列最后一排（第7排）孔（用户 09-01 两改定：
  第1排→第3排→最后一排，远离燃烧匙把手/药品瓶；右孔避开斜靠把手，横夹可达）。
- SampleLiquid_<色>（候选色瓶液变体，隐藏，task 按 cfg.liquid_color 显一根）。
- DropperFill（滴管尖内吸液截锥，隐藏，照 d3l 锥台贴合收窄尖端）。
- SpoonLiquid（燃烧匙碗内液体，隐藏，task 滴落后 reveal）。
- DropperDrop（挤胶头滴落串，隐藏，滴入燃烧匙碗）。

用法：python scripts/gen_c4_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import shutil

from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "c_flame", "c4_combustion_liquid")
OUT = os.path.join(SCENE_DIR, "c4_combustion_liquid.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- C3 骨架（照 c3_combustion_solid.usd 实际定稿 xform）----
# 试管架：后右，燃烧匙斜靠支点 + 滴管插后排孔。
RACK_X, RACK_Y = 0.72, 0.42
RACK_HOLE_BOTTOM = TABLE_TOP + 0.006   # 0.806（架孔洞底 = 架z0.8965 - 0.0905，照 d3l）

# 燃烧匙：碗贴台面、把手斜靠试管架左前侧（C3 定稿 translate，无旋转，原厂
# "碗朝上把手斜向上 +X"）。碗口在资产局部 z=0 → 世界 z=0.8068。
SPOON_X, SPOON_Y = 0.636, 0.3093
SPOON_TZ = 0.8068                       # 燃烧匙 translate z（C3 定稿）
SPOON_MOUTH_Z = SPOON_TZ                # 碗口世界 z = 0.8068（滴液/火焰目标）
SPOON_BOWL_BOTTOM = SPOON_TZ - 0.0102   # 碗底世界 z = 0.7966

# 酒精灯：火柴正对的 -Y 位置（用户 09-01「酒精灯和灯帽整体移动到火柴的正对 -y」）。
# 灯芯 (LAMP_X,LAMP_Y,0.9007) 在火柴头 (0.7094,0.05) 正下方 —— 火柴直推 -Y 点火、
# 直退 +Y 回程，永不横向扫过火焰柱。火焰迁 /World 顶层（初始隐藏）。
LAMP_X, LAMP_Y = 0.7094, -0.10
FLAME_BASE_Z = 0.900                     # 火焰底 = 灯芯顶（对齐 C3）
FLAME_APEX_Z = 0.936
FLAME_OUTER_R = 0.009
FLAME_INNER_R = 0.005
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022

# 火柴：酒精灯 +Y 侧 13mm 抬高，头朝 +X；头中心 x=0.7094 正对灯芯 (0.7094,-0.10)。
MATCH_T = (0.62, 0.05, TABLE_TOP + 0.013)

# 灯帽：摘下放灯旁 12cm（-X）台面（同 C3 换算），随灯整体平移 → (0.5894,-0.10)。
CAP_DETACH = (0.12, 0.0, -0.0762)

# ---- 液体样品区（模仿 d3l：药品瓶 + 滴管）----
# 药品瓶（sample_bottle.usd：auto 贴台 0.80；stopper 摘下倒放瓶旁，瓶口 rim 0.870、液面 0.840）。
# 用户 09-01：原 (0.40,0.35) 移 +X 10cm、+Y 20cm → (0.50,0.55)。
SAMPLE_BOTTLE_X, SAMPLE_BOTTLE_Y = 0.50, 0.55
# 瓶内半瓶液（0.80..0.84）
SAMPLE_LIQ_R = 0.014
SAMPLE_LIQ_H = 0.040
SAMPLE_LIQ_CZ = TABLE_TOP + SAMPLE_LIQ_H / 2      # 0.820
# 滴管（dropper.usd：尖嘴底=原点）立插试管架右列**最后一排（第7排）**孔（0.7385,0.5395）。
# 架孔 7 排对称分布（d3s 实测：row1 偏移 -0.1197、row3 -0.0398、row5 +0.0395、排距
# ~0.0399；第7排 = 对称 +0.1195）。用户 09-01 两改：原第1排不好→第3排→定稿最后一排
# （远离燃烧匙把手/药品瓶，工作区后缘）；右孔(0.7385)避开斜靠的燃烧匙把手
# （把手最远 x=0.725 < 0.7385），抓点 z 0.936 可达。
DROPPER_X, DROPPER_Y = RACK_X + 0.0185, RACK_Y + 0.1195   # 0.7385, 0.5395
# 滴管尖内吸液截锥（照 d3l DropperFill：下底 Ø2mm 贴尖嘴 → 上底 Ø7mm，覆盖收窄段）
DROPPER_FILL_R = (0.001, 0.0035)
DROPPER_FILL_H = 0.060

# 燃烧匙碗内液体（滴落后 reveal）：半球碗（r~0.0102），液盘贴碗底。
SPOON_LIQ_R = 0.008
SPOON_LIQ_H = 0.004
SPOON_LIQ_CZ = SPOON_BOWL_BOTTOM + 0.004            # 0.8006

# 挤胶头滴落串：一次挤 DROPS_PER_GROUP 滴连续坠落（照 d3l）。
DROPS_PER_GROUP = 4
DROP_BALL_R = 0.003
DROP_HOME = (SPOON_X, SPOON_Y, SPOON_MOUTH_Z + 0.02)   # 燃烧匙碗口上方（动画起点）

# 液面燃烧火焰（combustible：碗液面点燃，火焰色 cfg.flame_color 选一组，用户 09-02
# 「火焰颜色通过输入决定」；pivot=火焰底=液面，task 每帧动画组 translate 跟随液面 +
# scale/rotate flicker）。
SPOON_FLAME_R = 0.005
SPOON_FLAME_APEX_DZ = 0.028   # 液面火焰高（碗口上方可见 ~28mm）

# 液面燃烧火焰候选色（火焰=自发光，diffuse 近黑 0.01 + 单通道主导 emissive 出饱和焰色，
# 同 flametest 配方）。default=blue 淡蓝酒精标准燃烧色。
SPOON_FLAME_COLORS = {
    "blue":   (0.40, 0.70, 2.60),   # 淡蓝（酒精标准燃烧色，默认）
    "yellow": (2.40, 1.80, 0.15),   # 黄焰（钠）
    "orange": (2.60, 0.80, 0.15),   # 橙焰（钙）
    "red":    (2.40, 0.20, 0.15),   # 红焰（锶/锂）
    "green":  (0.25, 2.20, 0.30),   # 绿焰（铜/钡）
    "purple": (1.80, 0.30, 2.40),   # 紫焰（钾）
}

# 沸腾气泡（non_combustible：碗液面冒泡蒸发，不燃烧）
SPOON_BUBBLE_N = 6
SPOON_BUBBLE_R = 0.0025

# (prim, asset, translate, scale, rotxyz)  tz=None → 动态贴台面；rotxyz 角度
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, None),
    ("CombustionSpoon", "combustion_spoon.usd", (SPOON_X, SPOON_Y, SPOON_TZ), None, None),
    ("AlcoholLamp", "alcohol_lamp.usd", (LAMP_X, LAMP_Y, None), None, (0, 0, 180)),
    ("SampleBottle", "sample_bottle.usd", (SAMPLE_BOTTLE_X, SAMPLE_BOTTLE_Y, None), None, None),
    ("Dropper", "dropper.usd", (DROPPER_X, DROPPER_Y, RACK_HOLE_BOTTOM), None, None),
    ("Match", "match.usd", MATCH_T, None, None),
]

# 无色透明水（cfg.liquid_color=clear 时瓶液/碗液/吸液/滴落的 fallback）。
# 2026-09-01 用户「clear 应为无色透明」：不再照 d3l 的浅天蓝 (0.72,0.85,1.0)，
# 改 A3 已确认无色水配方 WATER_DROP_COLOR=(1.0,1.0,1.0)（淡蓝→白），
# 透明度压到 0.40（比 A3 烧杯水 0.35 稍实，半瓶液在瓶内仍可辨）。
CLEAR_WATER = dict(color=(1.0, 1.0, 1.0), opacity=0.40, roughness=0.05, ior=1.33)
WATER = CLEAR_WATER   # 瓶内液 fallback（SampleLiquid）
FILL = CLEAR_WATER    # 滴管吸液柱（DropperFill）
DROP = CLEAR_WATER    # 滴落液滴 + 碗液 fallback（DropperDrop / SpoonLiquid）
# 候选色瓶液配方（照 d3l LIQUID_COLORS：近黑 diffuse + 单通道主导 emissive 才出饱和色）
# 候选色瓶液配方（照 d3l LIQUID_COLORS）
LIQUID_COLORS = {
    "red":    dict(color=(0.10, 0.03, 0.03), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.2, 0.12, 0.12)),
    "blue":   dict(color=(0.03, 0.05, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 0.30, 2.2)),
    "green":  dict(color=(0.03, 0.10, 0.04), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(0.12, 2.0, 0.12)),
    "purple": dict(color=(0.12, 0.03, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                   emissive=(2.0, 0.15, 2.2)),
}
# 玻璃配方（照 d3l GLASS：滴管/瓶 op0.25 真玻璃透出液体）
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
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


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。
    （照 d3l：滴管收窄尖端内腔是锥形，直圆柱会悬空穿模，须锥台贴合。）"""
    n = 16
    pts, counts, indices = [], [], []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_bottom * math.cos(a), r_bottom * math.sin(a), 0.0))
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_top * math.cos(a), r_top * math.sin(a), h))
    pts += [Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, h)]
    for i in range(n):
        i0, i1 = i, (i + 1) % n
        counts.append(4)
        indices += [i0, i1, i1 + n, i0 + n]
    counts.append(n)
    indices += [2 * n] + list(range(n - 1, -1, -1))
    counts.append(n)
    indices += [2 * n + 1] + list(range(n, 2 * n))
    mesh = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    return mesh


def add_env_light(stage):
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


# ---- C4 效果 prim（内建，task 动画驱动）----
def add_sample_liquid(stage):
    """药品瓶内无色/浅蓝液柱 fallback（cfg.liquid_color=clear 时用它）：可见半瓶液。"""
    liq = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    liq.CreateRadiusAttr(SAMPLE_LIQ_R)
    liq.CreateHeightAttr(SAMPLE_LIQ_H)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(SAMPLE_BOTTLE_X, SAMPLE_BOTTLE_Y, SAMPLE_LIQ_CZ))
    add_material(stage, liq.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    print(f"[effect] SampleLiquid visible (r{SAMPLE_LIQ_R} h{SAMPLE_LIQ_H} "
          f"top {TABLE_TOP + SAMPLE_LIQ_H:.4f})")


def add_sample_liquid_colors(stage):
    """候选色瓶液变体（照 d3l add_color_liquid）：SampleLiquid_<色>，初始隐藏；
    task 按 cfg.liquid_color 显一根（clear 回退 SampleLiquid）。"""
    for name, m in LIQUID_COLORS.items():
        geom = UsdGeom.Cylinder.Define(stage, f"/World/SampleLiquid_{name}")
        geom.CreateRadiusAttr(SAMPLE_LIQ_R)
        geom.CreateHeightAttr(SAMPLE_LIQ_H)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(SAMPLE_BOTTLE_X, SAMPLE_BOTTLE_Y, SAMPLE_LIQ_CZ))
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=True)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] SampleLiquid_{name} hidden (r={SAMPLE_LIQ_R})")


def add_dropper_fill(stage):
    """滴管尖内吸液截锥：隐藏，task 吸液后 reveal 并逐帧跟随滴管尖。"""
    r_bottom, r_top = DROPPER_FILL_R
    mesh = add_frustum(stage, "DropperFill", r_bottom, r_top, DROPPER_FILL_H)
    mesh.AddTranslateOp().Set(Gf.Vec3d(DROPPER_X, DROPPER_Y, RACK_HOLE_BOTTOM))
    add_material(stage, mesh.GetPrim(), FILL["color"], FILL["opacity"],
                 roughness=FILL["roughness"], ior=FILL["ior"], double_sided=True)
    UsdGeom.Imageable(mesh).MakeInvisible()
    print(f"[effect] DropperFill hidden (frustum r{r_bottom}->{r_top} h{DROPPER_FILL_H})")


def add_spoon_liquid(stage):
    """燃烧匙碗内液体 fallback（无色/浅蓝）：隐藏，task 滴落后 reveal（液体滴入碗）。"""
    sl = UsdGeom.Cylinder.Define(stage, "/World/SpoonLiquid")
    sl.CreateRadiusAttr(SPOON_LIQ_R)
    sl.CreateHeightAttr(SPOON_LIQ_H)
    sl.CreateAxisAttr("Z")
    sl.AddTranslateOp().Set(Gf.Vec3d(SPOON_X, SPOON_Y, SPOON_LIQ_CZ))
    add_material(stage, sl.GetPrim(), DROP["color"], DROP["opacity"],
                 roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
    UsdGeom.Imageable(sl).MakeInvisible()
    print(f"[effect] SpoonLiquid hidden (r{SPOON_LIQ_R} in spoon bowl)")


def add_spoon_liquid_colors(stage):
    """碗内液候选色变体（照 d3l 预烘焙变色柱）：SpoonLiquid_<色>，初始隐藏；
    task 按 cfg.liquid_color 显一根（clear 回退 SpoonLiquid），滴入碗的液体与瓶液同色。"""
    for name, m in LIQUID_COLORS.items():
        geom = UsdGeom.Cylinder.Define(stage, f"/World/SpoonLiquid_{name}")
        geom.CreateRadiusAttr(SPOON_LIQ_R)
        geom.CreateHeightAttr(SPOON_LIQ_H)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(SPOON_X, SPOON_Y, SPOON_LIQ_CZ))
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=True)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] SpoonLiquid_{name} hidden (r={SPOON_LIQ_R})")


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 + Drop_0.._N 亮蓝小球。父+每球 MakeInvisible
    （防 delay 中的球停在 home 位闪现成拖影）。task._on_drop 每次挤生成一串、
    _step_drop_anim 逐滴错帧坠入燃烧匙碗。"""
    g = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    for i in range(DROPS_PER_GROUP):
        s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/Drop_{i}")
        s.CreateRadiusAttr(DROP_BALL_R)
        s.AddTranslateOp().Set(Gf.Vec3d(*DROP_HOME))
        add_material(stage, s.GetPrim(), DROP["color"], DROP["opacity"],
                     roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
        UsdGeom.Imageable(s).MakeInvisible()
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DropperDrop hidden ({DROPS_PER_GROUP} drop spheres)")


# ---- 后处理（照 C3/d3l）----
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
    液面火焰另写 translate 跟随液面。球心=底+r，锥从球顶到 apex。"""
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


def add_spoon_flame(st):
    """液面燃烧火焰（combustible：碗液面点燃，火焰色 cfg.flame_color 选一组，用户 09-02
    「火焰颜色通过输入决定」）。pivot=火焰底=液面，task 每帧写 translate 跟随液面 +
    scale/rotate flicker。多色变体 SpoonFlame_<色>_grp 初始全部隐藏，task 按 flame_color
    显一组（default=blue 淡蓝酒精标准燃烧色）。"""
    for name, emissive in SPOON_FLAME_COLORS.items():
        add_droplet_flame_grp(st, f"SpoonFlame_{name}", SPOON_FLAME_R, 0.0,
                              SPOON_FLAME_APEX_DZ, emissive, hidden=True,
                              x=SPOON_X, y=SPOON_Y)
    print(f"[effect] SpoonFlame_<色> grps hidden (x{len(SPOON_FLAME_COLORS)} colors)")


def add_spoon_bubbles(st):
    """沸腾气泡（non_combustible：碗液面冒泡蒸发，无火焰）。/World/SpoonBubble 父 +
    Bubble_0..N 半透明白球，task 沸腾期错帧上升循环。初始隐藏。"""
    g = UsdGeom.Xform.Define(st, "/World/SpoonBubble")
    for i in range(SPOON_BUBBLE_N):
        s = UsdGeom.Sphere.Define(st, f"/World/SpoonBubble/Bubble_{i}")
        s.CreateRadiusAttr(SPOON_BUBBLE_R)
        s.AddTranslateOp().Set(Gf.Vec3d(SPOON_X, SPOON_Y, SPOON_LIQ_CZ))
        add_material(st, s.GetPrim(), (1.0, 1.0, 1.0), 0.35,
                     roughness=0.2, ior=None, emissive=(0.6, 0.6, 0.6), double_sided=True)
        UsdGeom.Imageable(s).MakeInvisible()
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] SpoonBubble hidden ({SPOON_BUBBLE_N} bubbles)")


def rebuild_flames(st2):
    for path in ("/World/flame_outer", "/World/flame_inner",
                 "/World/flame_outer_sphere", "/World/flame_inner_sphere",
                 "/World/flame_outer_grp", "/World/flame_inner_grp",
                 "/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame_grp(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                          (0.35, 0.55, 2.40), hidden=True, x=LAMP_X, y=LAMP_Y)
    add_droplet_flame_grp(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                          (2.80, 0.55, 0.20), hidden=True, x=LAMP_X, y=LAMP_Y)
    print(f"[lamp] flames grp: base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"at ({LAMP_X},{LAMP_Y}) (hidden, flicker-driven)")


# ---- 玻璃透明化（照 d3l：引用资产材质 override）----
def override_bound_shader(st2, prim, recipe):
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


def flip_bottle_stopper(st2):
    """药品瓶盖倒放桌面（用户 09-01：瓶旁应有倒放的瓶盖；实验室标准=开瓶后盖子
    倒放、密封面朝上不触台面防污染，照 B3L flip_solution_stopper / d4s/d4l）。

    保留原 prim（复用 sample_bottle 的 stopper 几何/材质，原无 xform op 纯几何），给
    /World/SampleBottle/stopper 加 xform op：op 顺序 [translate, rotateXYZ]（translate
    最外层不被旋转，同 B3L/d4s 手法）。rotateY180° 让瓶口密封面（原下底面 z=0.068）
    朝上 → 倒放；translate 移到瓶 -X 侧 45mm 并抬 0.079（翻转后下底 z=-0.079 回到
    台面 0.80）→ 世界中心 (SAMPLE_BOTTLE_X-0.045, SAMPLE_BOTTLE_Y)、盖厚 11mm 竖放、
    密封面（洁净面）朝上不触台面。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print("[clean] /World/SampleBottle not found, skip")
        return
    stopper = p.GetChild("stopper")
    if not stopper:
        print("[clean] SampleBottle has no stopper, skip flip")
        return
    xf = UsdGeom.Xformable(stopper)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-0.045, 0.0, 0.079))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 180.0, 0.0))
    print(f"[clean] SampleBottle stopper flipped to desk "
          f"(inverted, sealing face up, center {SAMPLE_BOTTLE_X - 0.045:.4f},{SAMPLE_BOTTLE_Y:.4f} "
          "on table 0.80)")


def fix_bottle_materials(st2):
    """药品瓶玻璃透明化（照 d3l）：磨砂 op0.8 → 真玻璃 op0.25 + ior1.5 + doubleSided，
    瓶内 SampleLiquid 液面透出。若自带 1mm 液面薄盘（"liquid" mesh）隐藏。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print("[mat] /World/SampleBottle not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if c.GetName() == "liquid":
            UsdGeom.Imageable(c).MakeInvisible()
            print(f"[mat] hid {c.GetPath()} (1mm liquid disc, replaced by SampleLiquid)")
        elif c.GetName() == "stopper":
            print(f"[mat] keep {c.GetPath()} opaque (flipped cap on desk, not glass)")
        else:
            if override_bound_shader(st2, c, GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化（照 d3l）：glass_001 op1.0 磨砂 → 真玻璃 op0.25 透出吸液；胶头不透明。"""
    mat = st2.GetPrimAtPath("/World/Dropper/_materials/glass_001")
    if not mat.IsValid():
        print("[mat] Dropper glass_001 not found, skip")
    else:
        for c in mat.GetChildren():
            if c.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(c)
            for n, val in GLASS.items():
                inp = sh.GetInput(n)
                vt = Sdf.ValueTypeNames.Color3f if n == "diffuseColor" else Sdf.ValueTypeNames.Float
                if not inp:
                    inp = sh.CreateInput(n, vt)
                inp.Set(val)
            print(f"[mat] Dropper glass_001 -> transparent {GLASS}")
    g = st2.GetPrimAtPath("/World/Dropper/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")
    else:
        print("[mat] Dropper glass mesh not found for doubleSided, skip")


def relocate_absolute_textures(st2):
    """烘平后材质贴图若为仓库内绝对路径 → 改场景相对路径（照 d3l）。"""
    scene_dir = os.path.dirname(OUT)
    n = 0
    for prim in Usd.PrimRange(st2.GetPseudoRoot()):
        if prim.GetTypeName() != "Shader":
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path:
                p = v.path.replace("\\", "/")
                if os.path.isabs(p) and p.startswith(REPO):
                    rel = os.path.relpath(p, scene_dir).replace("\\", "/")
                    inp.Set(Sdf.AssetPath(rel))
                    print(f"[tex] absolute {os.path.basename(p)} -> {rel}")
                    n += 1
    if n:
        print(f"[tex] relocated {n} absolute texture path(s)")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    names = ["TestTubeRack", "CombustionSpoon", "AlcoholLamp",
             "SampleBottle", "Dropper", "Match"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        if name == "SampleBottle":
            # 只量瓶身（bottle 子 mesh），排除倒放桌面的瓶盖 stopper 子 prim（照 B3L）
            bottle_mesh = p.GetChild("bottle")
            if not bottle_mesh.IsValid():
                print(f"[verify] /World/{name}/bottle missing")
                continue
            r = bc.ComputeWorldBound(bottle_mesh).ComputeAlignedRange()
        else:
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")

    # 试管架底座贴台面
    rkn, rkx = boxes["TestTubeRack"]
    assert abs(rkn[2] - TABLE_TOP) < 0.002, f"rack base z {rkn[2]} != table {TABLE_TOP}"

    # 无铁架台（C3 定稿：燃烧匙靠试管架，不用铁架台）
    assert not st2.GetPrimAtPath("/World/IronStand").IsValid(), "IronStand should be removed"

    # 燃烧匙：碗贴台、把手斜靠试管架（bbox 与架左缘重叠 x、把手顶端高 ~30cm）
    pn, px = boxes["CombustionSpoon"]
    assert px[0] > rkn[0] - 0.005, f"spoon blade not leaning on rack: x max {px[0]} vs rack {rkn[0]}"
    assert px[2] - pn[2] > 0.25, f"spoon blade too short: {px[2]-pn[2]:.3f}"
    assert abs(pn[2] - 0.7966) < 0.005, f"spoon bowl bottom {pn[2]:.4f} != C3 定稿 0.7966"

    # 酒精灯身中心在 (LAMP_X, LAMP_Y)；灯帽摘旁（勿用整灯 bbox 含帽）
    body = st2.GetPrimAtPath("/World/AlcoholLamp/body")
    br = bc.ComputeWorldBound(body).ComputeAlignedRange()
    bmn, bmx = br.GetMin(), br.GetMax()
    assert abs((bmn[0] + bmx[0]) / 2 - LAMP_X) < 0.002, "lamp body center x off"
    assert abs((bmn[1] + bmx[1]) / 2 - LAMP_Y) < 0.002, "lamp body center y off"

    # 药品瓶贴台面（0.80）、中心在 (SAMPLE_BOTTLE_X, SAMPLE_BOTTLE_Y)
    btl = boxes["SampleBottle"]
    assert abs(btl[0][2] - TABLE_TOP) < 0.002, f"bottle bottom {btl[0][2]} not on table"
    bx, by = 0.5 * (btl[0][0] + btl[1][0]), 0.5 * (btl[0][1] + btl[1][1])
    assert abs(bx - SAMPLE_BOTTLE_X) < 0.01 and abs(by - SAMPLE_BOTTLE_Y) < 0.01, \
        f"bottle center ({bx:.3f},{by:.3f}) off ({SAMPLE_BOTTLE_X},{SAMPLE_BOTTLE_Y})"

    # 瓶盖倒放桌面（用户 09-01）：盖底贴台面 0.80、盖厚 11mm、中心在瓶 -X 侧 45mm、
    # rotY180（倒放密封面朝上，照 B3L）
    cap = st2.GetPrimAtPath("/World/SampleBottle/stopper")
    assert cap.IsValid(), "SampleBottle stopper missing"
    rc = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn3, cmx3 = rc.GetMin(), rc.GetMax()
    assert abs(cmn3[2] - TABLE_TOP) < 0.002, f"stopper bottom {cmn3[2]:.4f} not on table 0.80"
    assert abs(cmx3[2] - cmn3[2] - 0.011) < 0.002, f"stopper height {cmx3[2]-cmn3[2]:.4f} != 11mm"
    scx, scy = 0.5 * (cmn3[0] + cmx3[0]), 0.5 * (cmn3[1] + cmx3[1])
    assert abs(scx - (SAMPLE_BOTTLE_X - 0.045)) < 0.006, f"stopper center x {scx:.4f} != {SAMPLE_BOTTLE_X-0.045:.4f}"
    assert abs(scy - SAMPLE_BOTTLE_Y) < 0.006, f"stopper center y {scy:.4f} != {SAMPLE_BOTTLE_Y}"
    cap_rot = cap.GetAttribute("xformOp:rotateXYZ")
    assert cap_rot and abs(cap_rot.Get()[1] - 180.0) < 1.0, \
        f"stopper rotateXYZ.y != 180 (not inverted): {cap_rot.Get() if cap_rot else None}"

    # 滴管立试管架后排孔（尖嘴底贴洞底 0.8060、xy 在架范围）
    dmn, dmx = boxes["Dropper"]
    assert abs(dmn[2] - RACK_HOLE_BOTTOM) < 0.003, \
        f"dropper tip bottom {dmn[2]:.4f} not in rack hole"
    assert rkn[0] < dmn[0] and dmx[0] < rkx[0], "dropper x not within rack"
    assert rkn[1] < dmn[1] and dmx[1] < rkx[1], "dropper y not within rack"
    assert (dmn[1] + dmx[1]) / 2 > RACK_Y, "dropper should be last row (y > rack center)"
    assert abs((dmn[1] + dmx[1]) / 2 - DROPPER_Y) < 0.002, \
        f"dropper row center {(dmn[1]+dmx[1])/2:.4f} != last row {DROPPER_Y}"

    # 火柴躺台面抬高
    mtn, mtx = boxes["Match"]
    assert mtn[2] > TABLE_TOP + 0.010, f"match not raised: {mtn[2]}"

    # 灯帽摘下放灯旁 12cm（-X）：帽底贴台面
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    assert abs((cmn[0] + cmx[0]) / 2 - (LAMP_X - 0.12)) < 0.005, "cap center x off 12cm beside lamp"

    # 火焰迁到 /World 顶层分组（pivot=火焰底），apex 0.936，初始隐藏
    f = st2.GetPrimAtPath("/World/flame_outer_grp/flame_outer")
    assert f.IsValid() and f.GetTypeName() == "Cone", "flame_outer cone missing"
    assert abs(UsdGeom.Cone(f).GetRadiusAttr().Get() - FLAME_OUTER_R) < 0.0005, "flame r wrong"
    assert abs(FLAME_APEX_Z - 0.936) < 0.0005, "flame apex not 0.936"
    assert UsdGeom.Imageable(f).ComputeVisibility() == "invisible", "flame_outer should be hidden"
    assert st2.GetPrimAtPath("/World/flame_outer_grp").IsValid(), "flame_outer_grp missing"
    assert not st2.GetPrimAtPath("/World/flame_outer").IsValid(), "ungrouped flame_outer present"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"

    # 液面燃烧火焰组（combustible：dwell 点燃后 task reveal，多色变体全隐藏）+ 沸腾气泡组初始隐藏
    for name in SPOON_FLAME_COLORS:
        sf = st2.GetPrimAtPath(f"/World/SpoonFlame_{name}_grp/SpoonFlame_{name}")
        assert sf.IsValid() and sf.GetTypeName() == "Cone", f"SpoonFlame_{name} cone missing"
        assert UsdGeom.Imageable(sf).ComputeVisibility() == "invisible", \
            f"SpoonFlame_{name} should be hidden"
    sb = st2.GetPrimAtPath("/World/SpoonBubble")
    assert sb.IsValid(), "SpoonBubble missing"
    nbb = sum(1 for c in sb.GetChildren() if c.GetTypeName() == "Sphere")
    assert nbb == SPOON_BUBBLE_N, f"SpoonBubble spheres {nbb} != {SPOON_BUBBLE_N}"
    assert UsdGeom.Imageable(sb).ComputeVisibility() == "invisible", "SpoonBubble should be hidden"

    # 药品瓶液可见、候选色瓶液/吸液截锥/碗液/滴落串全隐藏
    sl = st2.GetPrimAtPath("/World/SampleLiquid")
    assert sl.IsValid(), "SampleLiquid missing"
    assert UsdGeom.Imageable(sl).ComputeVisibility() != "invisible", "SampleLiquid should be visible"
    assert abs(UsdGeom.Cylinder(sl).GetHeightAttr().Get() - SAMPLE_LIQ_H) < 1e-9, "SampleLiquid h wrong"
    for name in LIQUID_COLORS:
        p = st2.GetPrimAtPath(f"/World/SampleLiquid_{name}")
        assert p.IsValid(), f"SampleLiquid_{name} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
            f"SampleLiquid_{name} should be hidden"
    for hidden in ("DropperFill", "SpoonLiquid"):
        p = st2.GetPrimAtPath(f"/World/{hidden}")
        assert p.IsValid(), f"{hidden} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", f"{hidden} should be hidden"
    for name in LIQUID_COLORS:
        p = st2.GetPrimAtPath(f"/World/SpoonLiquid_{name}")
        assert p.IsValid(), f"SpoonLiquid_{name} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
            f"SpoonLiquid_{name} should be hidden"
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    assert dd.IsValid(), "DropperDrop missing"
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == DROPS_PER_GROUP, f"DropperDrop spheres {nd} != {DROPS_PER_GROUP}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible", "DropperDrop parent should be hidden"

    # C3 固体挖粉器皿 + 铁架台已删
    for gone in ("IronStand", "Spatula", "SurfaceDish", "SamplePowder", "TestTube"):
        assert not st2.GetPrimAtPath(f"/World/{gone}").IsValid(), f"{gone} should be removed"

    print("[verify] OK: 试管架贴台 | 燃烧匙碗贴台·把手斜靠试管架(无铁架台) | 灯居中·帽摘旁 | "
          "药品瓶贴台+瓶液可见 | 滴管后排立架孔 | 火柴抬高 | 火焰迁顶层·初始隐藏(尖0.936) | "
          "色变体/吸液截锥/碗液/滴落串隐藏 | 铁架台/药匙/皿粉/试管已删")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rotxyz in EQUIP:
        add_equip(stage, name, asset, t, scale, rotxyz)
    add_sample_liquid(stage)
    add_sample_liquid_colors(stage)
    add_dropper_fill(stage)
    add_spoon_liquid(stage)
    add_spoon_liquid_colors(stage)
    add_dropper_drops(stage)
    add_spoon_flame(stage)       # 液面燃烧火焰（combustible，dwell 点燃 reveal）
    add_spoon_bubbles(stage)     # 沸腾气泡（non_combustible，dwell 冒泡）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    strip_dome_lights(st2)
    remove_asset_env_lights(st2)
    detach_lamp_cap(st2)        # 灯帽摘下放灯旁 12cm 台面
    rebuild_flames(st2)         # 火焰迁到 /World 顶层（初始隐藏）
    flip_bottle_stopper(st2)    # 药品瓶盖摘下倒放瓶旁桌面（密封面朝上）
    fix_bottle_materials(st2)   # 药品瓶玻璃透明化（瓶液透出）
    fix_dropper_materials(st2)  # 滴管玻璃透明化（吸液/滴落透出）
    relocate_absolute_textures(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)   # 移巨型 CylinderLight 远离相机
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
