# -*- coding: utf-8 -*-
"""生成 d2l_water_solubility.usd —— D2-L 液体样品水溶性测试场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材
- 试管 + 一支胶头滴管插进试管架孔里（底面 z=0.806 = 架z−0.0905）
- 液体样品瓶（sample_bottle）+ 蒸馏水洗瓶（wash_bottle，操作台右侧）
- 去瓶塞（样品瓶已开瓶，瓶口 rim=0.070 → 世界 0.870）
- 内建效果 prim：SampleLiquid（样品瓶体积，可见）/TubeDrops（管内液体）
  /LayerColumn（样品色分层柱，震荡前分层态）/Cloud（浑浊云，cloudy 档）
  /WaterStream（洗瓶挤水水流），后四初始隐藏（task 动画驱动）

布局（2026-08-24 对齐 D2-S：底座 [-0.15,0.05,0.71]，工作区右移 x≈0.66；洗瓶/样品瓶/试管架
沿 y 共线（x≈0.68），样品瓶在洗瓶与架之间；滴管顶替 D2-S 药匙的架中心孔抓取位）：
  TestTubeRack  (0.6803, 0.3607)  底座贴台面 z=0.8965
  TestTube      (0.659, 0.241, 0.806)  架最近侧孔（D2-S 试管位，管口 z=0.9593）
  DropperSample (0.6993, 0.3608, 0.806)  架中心孔（D2-S 药匙位，0.904m 已验证可达）
  SampleBottle  (0.6809, -0.10)  液体样品瓶（与洗瓶/架 y 共线，两者之间）
  WashBottle    (0.370, 0.525)  洗瓶（2026-08-25 对齐 D2-S：绕 Z -180° → 红嘴朝 +X）

混合现象三档（2026-08-24 用户确认：震荡前先显示分层，震荡步骤才分化现象）：
  - 震荡前：管内顶部出现样品色分层柱 LayerColumn（顶贴液面、向下 LAYER_H）
  - 震荡时按 cfg.mixing 分化（task._step_mixing）：
      miscible → LayerColumn 长满整管（扩散均一）
      layered  → LayerColumn 保持（仍两层）
      cloudy   → Cloud 乳白柱盖满（白浊），停震后退
  几何实现——headless 下运行时改材质不渲染，现象全部走预烘焙几何 + visibility。

用法：python scripts/gen_d2l_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2l_water_solubility")
OUT = os.path.join(SCENE_DIR, "d2l_water_solubility.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
HOLE_BOTTOM = 0.806  # = 架 translate z(0.8965) − 0.0905

# (prim, asset_file, translate, scale, rotate)   tz=None → 动态贴台面；rotate=None 不加旋转
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.6803, 0.3607, None), None, None),
    ("TestTube", "test_tube.usd", (0.659, 0.241, HOLE_BOTTOM), None, None),
    # 滴管顶替 D2-S 药匙的架中心孔抓取位（距底座 [-0.15,0.05] 0.904m，D2-S 已验证可达）
    ("DropperSample", "dropper.usd", (0.6993, 0.3608, HOLE_BOTTOM), None, None),
    # 样品瓶：与洗瓶/试管架沿 y 共线（x≈0.68），位洗瓶与架之间；y=-0.10 低于底座 y=0.05
    # 避贴底座失效区（相对 y=-0.15，与洗瓶同侧）。
    ("SampleBottle", "sample_bottle.usd", (0.6809, -0.10, None), None, None),
    # 洗瓶：2026-08-25 对齐 D2-S 修改——translate (0.370,0.525,0.80)，绕 Z -180°（原资产瓶嘴
    # -X 转 180° 翻到 +X，红嘴朝 +X）。红嘴中心 (0.476,0.525,0.844)、嘴尖相对瓶心 +X 0.106。
    # 瓶身 Mesh_006=6.4×6.4×16.8cm 方柱，z 0.8001-0.9683。②注水动作（WashBottlePass）以
    # 嘴尖世界坐标（抬 15cm 后 (0.649,0.231,0.994)）为挤水基准。
    ("WashBottle", "wash_bottle.usd", (0.370, 0.525, 0.80), None, (0, 0, -180)),
]

# 液体材质配方（复用 d3l 最逼真配方）：
# WATER = 蒸馏水（近无色透明：浅白微蓝 + 低透 0.30，隔玻璃看是"清水"而非灰块；ior 1.33 折射
#   仍隐约可见液面）。旧值 (0.72,0.85,1.0) 透 0.50 太实，被 CylinderLight 12000 洗成灰。
# CLOUD_MILK = 浑浊云（乳白不透明）
WATER = dict(color=(0.90, 0.95, 1.0), opacity=0.30, roughness=0.05, ior=1.33)
CLOUD_MILK = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85, emissive=(1.0, 0.95, 0.80))
# 样品液颜色（2026-08-25 用户：样品色改为输入决定，同 d3l liquid_color 方法）。headless 运行时
# 改材质不渲染（记忆 headless-render-ignores-materials），故为每个候选色预烘焙一组样品色 prim
# （SampleLiquid/LayerColumn/DropperFill/DropperDrop 各 <色>），task 按 cfg.sample_color show
# 对应一组（其余隐藏）。clear=无色样品（淡灰白近透明，仍稍见瓶内液面）；默认 blue。
# 2026-08-25 修「气泡」：旧值照搬 flametest-yellow-recipe（近黑 diffuse + 强 emissive ~2.2）是
# 焰色反应用的霓虹发光配方，用在液体上让液柱自发光像霓虹灯/气泡（震荡时底部「很多动的」）。
# 改「近真实液体」：亮一点的 diffuse（显固有色）+ 极弱单通道 emissive（~0.5 只保一点点色，
# 不再霓虹）。纯 diffuse 会被 CylinderLight 12000 洗灰粉，故仍留弱 emissive 兜底。
SAMPLE_COLOR_NAMES = ("clear", "red", "blue", "green", "purple")
SAMPLE_COLORS = {
    "clear":  dict(color=(0.85, 0.90, 0.95), opacity=0.70, roughness=0.05, ior=1.33, emissive=None),
    "red":    dict(color=(0.40, 0.10, 0.10), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.5, 0.06, 0.06)),
    "blue":   dict(color=(0.08, 0.18, 0.40), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.06, 0.15, 0.5)),
    "green":  dict(color=(0.08, 0.40, 0.10), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.06, 0.5, 0.06)),
    "purple": dict(color=(0.35, 0.08, 0.40), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.4, 0.06, 0.5)),
}

# miscible 终点「均一混液」材质（稀释后**浅色但仍清晰显色**）。旧「浅色版」= 亮 diffuse
# (0.28,0.45,0.75) + 弱非主导 emissive + opacity 0.85，被 CylinderLight 12000 洗成无色透明
# （用户 2026-08-25 二反馈：蓝色溶解完还是无色透明）。根因与「气泡」修复同源：高亮度 diffuse
# 反光强被灯光洗白，emissive 又不主导压不住。改回**纯样品同款配方**（近黑 diffuse 防洗白 +
# 单通道主导 emissive），只把 emissive 抬到 ~0.8（比纯样品 0.5 更亮=稀释更浅），opacity 提到
# 0.95（与可见的纯样品柱一致，不再半透漏背景）。蓝色终点 = 浅天空蓝。
MIXED_COLORS = {
    "clear":  dict(color=(0.90, 0.93, 1.0), opacity=0.85, roughness=0.05, ior=1.33, emissive=None),
    "red":    dict(color=(0.38, 0.10, 0.10), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.80, 0.10, 0.10)),
    "blue":   dict(color=(0.08, 0.18, 0.40), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.12, 0.28, 0.80)),
    "green":  dict(color=(0.08, 0.40, 0.10), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.12, 0.80, 0.12)),
    "purple": dict(color=(0.35, 0.08, 0.40), opacity=0.95, roughness=0.05, ior=1.33, emissive=(0.64, 0.12, 0.80)),
}

# 内建效果 prim（不随样品色变化）: (name, type, radius, height, translate, 材质配方 dict, visible)
# TubeDrops = 管内**水**柱（0.009 贴管壁 Ø19.2 内缘；task 注水后显示/液面逐滴涨）
# Cloud     = 浑浊云（0.0089 罩在液柱内不穿模；cloudy 档震荡盖满、停震褪去）
# 样品色 prim（SampleLiquid/LayerColumn/DropperFill/DropperDrop）随 sample_color 预烘焙
# 多组变体，见 add_sample_color_variants()。
EFFECTS = [
    ("TubeDrops", "cylinder", 0.009, 0.030, (0.659, 0.241, 0.821), WATER, False),
    ("Cloud", "cylinder", 0.0089, 0.0, (0.659, 0.241, 0.806), CLOUD_MILK, False),
]
# 样品色 prim 几何（随 sample_color 预烘焙 <色> 变体，见 add_sample_color_variants）：
# SampleLiquid = 样品瓶内半瓶液体（cyl r=0.014 h=0.040，瓶底 0.80→液面 0.84）
# LayerColumn = 样品色分层柱（r=0.0086 略小于液柱防穿模；震荡前分层态，miscible 长满）
# DropperFill = 滴管尖内吸起的样品液柱（截锥，skill 坑 18：尖端内腔锥形须锥台；下底 r0.001
#   →上底 r0.0035 h=0.060，柱底贴尖嘴，task._set_fill_follow 逐帧跟随）
# DropperDrop = 挤胶头滴落串（父 Xform + 4 球 r=0.003）
LAYER_R = 0.0086   # 分层柱半径（略小于液柱 0.009 防穿模）
CLOUD_R = 0.0089   # 浑浊云半径（同 d3l，罩在液柱内）
SAMPLE_LIQ_R, SAMPLE_LIQ_H = 0.014, 0.040
SAMPLE_LIQ_T = (0.6809, -0.10, 0.820)
LAYER_T = (0.659, 0.241, 0.806)
FILL_R, FILL_H, FILL_T = (0.001, 0.0035), 0.060, (0.6993, 0.3608, 0.806)
DROP_R, DROP_T = 0.003, (0.659, 0.241, 0.820)

# 挤胶头滴落串（DropperDrop）：一次挤压 = DROPPER_DROPS 滴样品液连续坠落（成串滴）。
# DropperDrop 是父 Xform，task 动画驱动 Drop_0.._N 各球。
DROPPER_DROPS = 4

# 洗瓶挤水水流（WaterStream）：父 Xform + WATER_DROPS 颗小水滴球（r=0.0015 水蓝），沿抛物线
# 从红嘴尖 (0.649,0.231,0.994) 坠入试管口 (0.659,0.241,0.9593)。task._step_water_anim 挤水时
# 逐颗错帧发射、松爪收尾（仿 D2-S）。整体初始隐藏。
WATER_DROPS = 16
WATER_DROP_R = 0.0015
WATER_DROP_COLOR = (0.35, 0.65, 0.95)
WATER_START = (0.649, 0.231, 0.994)


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


def asset_local_min_z(asset_file):
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rotate=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rotate is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3d(*rotate))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    extra = (f" rotate {rotate}" if rotate else "") + (f" scale {scale}" if scale else "")
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + extra)


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。

    skill 坑 18：收口/尖端容器（滴管、锥形瓶…）内腔是锥形，直圆柱液体会悬空穿模
    （液面不贴壁、露出空隙），须用锥台贴合。下底 r=底部内半径、上底 r=液面高度处内半径。
    16 段圆周 + 底/顶 cap，subdivisionScheme=none（memory: 自建 mesh 必须设，否则不显色）。
    """
    n = 16
    pts, counts, indices = [], [], []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_bottom * math.cos(a), r_bottom * math.sin(a), 0.0))
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append(Gf.Vec3f(r_top * math.cos(a), r_top * math.sin(a), h))
    pts += [Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, h)]      # 底心 idx 2n、顶心 idx 2n+1
    for i in range(n):
        i0, i1 = i, (i + 1) % n
        counts.append(4)                                # 侧壁四边形（法向朝外）
        indices += [i0, i1, i1 + n, i0 + n]
    counts.append(n)                                    # 底 cap（法向朝下 -Z）
    indices += [2 * n] + list(range(n - 1, -1, -1))
    counts.append(n)                                    # 顶 cap（法向朝上 +Z）
    indices += [2 * n + 1] + list(range(n, 2 * n))
    mesh = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    return mesh


def add_effects(stage):
    for name, kind, r, h, t, m, visible in EFFECTS:
        if kind == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
            geom.CreateRadiusAttr(r)
            geom.CreateHeightAttr(h)
            geom.CreateAxisAttr("Z")
        elif kind == "frustum":
            r_bottom, r_top = r
            geom = add_frustum(stage, name, r_bottom, r_top, h)
        else:
            geom = UsdGeom.Sphere.Define(stage, f"/World/{name}")
            geom.CreateRadiusAttr(r)
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        translucent = m.get("opacity", 1.0) < 1.0
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        if not visible:
            UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} {'visible' if visible else 'hidden'} at {t}")


def add_water_stream(stage):
    """洗瓶挤水水流：/World/WaterStream 父 Xform + WATER_DROPS 颗水蓝小球（r=0.0015）。
    task._step_water_anim 挤水时逐颗错帧发射、沿抛物线坠落，松爪收尾。整体初始隐藏；
    home 位置 = 红嘴尖（task 动画才写实际坠落坐标）。"""
    g = UsdGeom.Xform.Define(stage, "/World/WaterStream")
    for i in range(WATER_DROPS):
        s = UsdGeom.Sphere.Define(stage, f"/World/WaterStream/Drop_{i}")
        s.CreateRadiusAttr(WATER_DROP_R)
        s.AddTranslateOp().Set(Gf.Vec3d(*WATER_START))
        add_material(stage, s.GetPrim(), WATER_DROP_COLOR, 0.90,
                     roughness=0.05, ior=1.33, double_sided=True)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] WaterStream hidden ({WATER_DROPS} water drop spheres)")


def add_sample_color_variants(stage):
    """样品色 prim 预烘焙（2026-08-25 用户：样品色输入决定，同 d3l 方法）。为每个候选色建
    一组样品色 prim（SampleLiquid_<色>/LayerColumn_<色>/DropperFill_<色>/DropperDrop_<色>），
    全隐藏；task 按 cfg.sample_color show 对应一组（其余隐藏），动画用同一组变体。headless
    运行时改材质不渲染，故预烘焙多组 + visibility 切换（见 SAMPLE_COLORS 注释）。"""
    for cname, m in SAMPLE_COLORS.items():
        translucent = m.get("opacity", 1.0) < 1.0
        # 样品瓶内液体（cyl，瓶底 0.80→液面 0.84）
        cy = UsdGeom.Cylinder.Define(stage, f"/World/SampleLiquid_{cname}")
        cy.CreateRadiusAttr(SAMPLE_LIQ_R); cy.CreateHeightAttr(SAMPLE_LIQ_H); cy.CreateAxisAttr("Z")
        cy.AddTranslateOp().Set(Gf.Vec3d(*SAMPLE_LIQ_T))
        add_material(stage, cy.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        # 分层柱（震荡前分层态，miscible 长满）
        lc = UsdGeom.Cylinder.Define(stage, f"/World/LayerColumn_{cname}")
        lc.CreateRadiusAttr(LAYER_R); lc.CreateHeightAttr(0.0); lc.CreateAxisAttr("Z")
        lc.AddTranslateOp().Set(Gf.Vec3d(*LAYER_T))
        add_material(stage, lc.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        # miscible 终点均一混液（稀释样品色；满管高度 task 运行时设）
        mx = UsdGeom.Cylinder.Define(stage, f"/World/MixedLiquid_{cname}")
        mx.CreateRadiusAttr(LAYER_R); mx.CreateHeightAttr(0.0); mx.CreateAxisAttr("Z")
        mx.AddTranslateOp().Set(Gf.Vec3d(*LAYER_T))
        mm = MIXED_COLORS[cname]
        add_material(stage, mx.GetPrim(), mm["color"], mm["opacity"],
                     roughness=mm.get("roughness", 0.5), ior=mm.get("ior"),
                     emissive=mm.get("emissive"), double_sided=translucent)
        # 滴管内液柱（截锥，柱底贴尖嘴）
        fr = add_frustum(stage, f"DropperFill_{cname}", FILL_R[0], FILL_R[1], FILL_H)
        fr.AddTranslateOp().Set(Gf.Vec3d(*FILL_T))
        add_material(stage, fr.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        # 滴落串
        g = UsdGeom.Xform.Define(stage, f"/World/DropperDrop_{cname}")
        for i in range(DROPPER_DROPS):
            s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop_{cname}/Drop_{i}")
            s.CreateRadiusAttr(DROP_R)
            s.AddTranslateOp().Set(Gf.Vec3d(*DROP_T))
            add_material(stage, s.GetPrim(), m["color"], m["opacity"],
                         roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                         emissive=m.get("emissive"), double_sided=True)
        # 全隐藏（task 按 sample_color show 选中组）
        for p in (cy, lc, mx, fr, g):
            UsdGeom.Imageable(p).MakeInvisible()
        print(f"[effect] sample '{cname}': SampleLiquid/LayerColumn/MixedLiquid/DropperFill/DropperDrop hidden")


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
        print(f"[light] CylinderLight translate {tuple(round(c, 3) for c in v)} "
              f"-> {tuple(round(c, 3) for c in (x, v[1], v[2]))}")
        return
    print("[light] CylinderLight has no translate op, skip")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def place_stopper_flat(st2):
    """瓶塞倒放桌面（真实开瓶操作）：取下样品瓶口上的磨砂塞，倒扣平放在瓶旁桌面，
    不再删除 stopper/stopper_mat（旧版 remove_stoppers 直接删 → 场景看不到盖子）。

    塞子资产局部 = 磨砂白扁塞 Ø25.2×11mm，插在瓶口 z 0.068..0.079（瓶口顶 0.070）；
    SampleBottle 世界 translate (0.6809,-0.10,0.80)。倒放 = 先绕 X 轴 180°（原朝上面
    翻到朝下倒扣）再平移：rotateX 把 z 翻到 [-0.079,-0.068]，translate dz=0.079 把倒扣
    塞底压到桌面世界 z=0.80；dx=-0.062 挪到瓶身 -X 旁 ~32mm（与瓶身同 y，不穿模）。
    ops 声明顺序 = [translate, rotateXYZ]（XformOp 组合 M=T·R → 点先旋转后平移）。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print("[stopper] /World/SampleBottle not found, skip")
        return
    stopper = next((pp for pp in Usd.PrimRange(p) if pp.GetName() == "stopper"), None)
    if stopper is None:
        print("[stopper] no stopper prim, skip")
        return
    xf = UsdGeom.Xformable(stopper)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)   # op0（先声明 → 后作用于点）
    ro = xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)   # op1（后声明 → 先作用于点）
    tr.Set(Gf.Vec3d(-0.062, 0.0, 0.079))
    ro.Set(Gf.Vec3d(180.0, 0.0, 0.0))
    print(f"[stopper] placed flat beside bottle at {stopper.GetPath()} "
          f"(T(-0.062,0,0.079) R(180,0,0))")


def remove_stray_env_lights(st2):
    """清理器材资产自带的残留 DomeLight（根因 2026-08-24：wash_bottle.usd 自带
    color_0C0C0C 纯黑环境贴图 DomeLight，残留会污染环境光 → 整场景变暗 + 黑反射块，
    同 v17 环境暗根因）。只保留 gen 自建的 /World/env_light（亮环境贴图）与
    /World/CylinderLight 主光；TestTubeRack 残留曾由旧版单独清理，现泛化到全部器材。"""
    keep = {"/World/env_light"}
    root = st2.GetPrimAtPath("/World")
    paths = [p.GetPath() for p in Usd.PrimRange(root)
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString not in keep]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed stray DomeLight {path}")
    if not paths:
        print("[clean] no stray DomeLight under /World")


def relocate_absolute_textures(st2):
    """坑 12：烘平后材质贴图 asset 属性若为仓库内绝对路径 → 改为场景相对路径。"""
    scene_dir = os.path.dirname(OUT)
    n = 0
    for prim in Usd.PrimRange(st2.GetPseudoRoot()):
        if prim.GetTypeName() != "Shader":
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path:
                p = v.path.replace("\\", "/")
                if os.path.isabs(p):
                    if p.startswith(REPO):
                        rel = os.path.relpath(p, scene_dir).replace("\\", "/")
                        inp.Set(Sdf.AssetPath(rel))
                        print(f"[tex] absolute {os.path.basename(p)} -> {rel}")
                        n += 1
                    else:
                        print(f"[tex] WARN absolute outside repo: {p}")
    print(f"[tex] relocated {n} absolute texture path(s)")


def verify(st2):
    """自检：打印各器材/效果世界 bbox，断言效果 prim 存在、初始可见性正确。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "DropperSample", "SampleBottle", "WashBottle",
             "TubeDrops", "Cloud", "WaterStream"]
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:15s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 默认样品色（blue）瓶内液体 bbox（代表）
    p_sl = st2.GetPrimAtPath("/World/SampleLiquid_blue")
    if p_sl.IsValid():
        r = bc.ComputeWorldBound(p_sl).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] SampleLiquid_blue min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 不随样品色变化的效果 prim 初始隐藏
    for name in ("TubeDrops", "Cloud", "WaterStream"):
        p = st2.GetPrimAtPath(f"/World/{name}")
        assert p.IsValid(), f"{name} missing"
        assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
            f"{name} should be hidden initially"
    # 样品色变体：每组（SampleLiquid/LayerColumn/MixedLiquid/DropperFill/DropperDrop）都存在且全隐藏
    for cname in SAMPLE_COLOR_NAMES:
        for base in ("SampleLiquid", "LayerColumn", "MixedLiquid", "DropperFill", "DropperDrop"):
            p = st2.GetPrimAtPath(f"/World/{base}_{cname}")
            assert p.IsValid(), f"{base}_{cname} missing"
            assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
                f"{base}_{cname} should be hidden initially"
        # 分层柱半径不变量
        assert abs(UsdGeom.Cylinder(st2.GetPrimAtPath(f"/World/LayerColumn_{cname}")).GetRadiusAttr().Get()
                   - LAYER_R) < 1e-9, f"LayerColumn_{cname} r != LAYER_R"
        # 滴落串球数量
        n_drop = len([c for c in st2.GetPrimAtPath(f"/World/DropperDrop_{cname}").GetChildren()
                      if c.GetTypeName() == "Sphere"])
        assert n_drop == DROPPER_DROPS, f"DropperDrop_{cname} children {n_drop} != {DROPPER_DROPS}"
    # 浑浊云半径不变量
    assert abs(UsdGeom.Cylinder(st2.GetPrimAtPath("/World/Cloud")).GetRadiusAttr().Get()
               - CLOUD_R) < 1e-9, "Cloud r != CLOUD_R"
    # 洗瓶水流球数量
    n_stream = len([c for c in st2.GetPrimAtPath("/World/WaterStream").GetChildren()
                    if c.GetTypeName() == "Sphere"])
    assert n_stream == WATER_DROPS, f"WaterStream children {n_stream} != WATER_DROPS={WATER_DROPS}"
    # 残留 DomeLight 必须清干净（纯黑环境贴图会污染整场景光照，2026-08-24 用户反馈场景偏暗）
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print(f"[verify] effects OK: sample colors {len(SAMPLE_COLOR_NAMES)}x5 variants hidden, "
          f"Layer r={LAYER_R} Cloud r={CLOUD_R}, WaterStream x{WATER_DROPS} / "
          f"DropperDrop x{DROPPER_DROPS}, no stray DomeLight")


GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)


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


def fix_bottle_materials(st2):
    """样品瓶玻璃透明化 + 自带 1mm 液面盘隐藏（改用 SampleLiquid 体积表现）。"""
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
            print(f"[mat] kept {c.GetPath()} (磨砂白塞, 保留资产材质, 已倒放桌面)")
        else:
            if override_bound_shader(st2, c, GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 不透明会遮住管内液柱。"""
    mat = st2.GetPrimAtPath("/World/DropperSample/_materials/glass_001")
    if not mat.IsValid():
        print("[mat] DropperSample glass_001 not found, skip")
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
            print(f"[mat] DropperSample glass_001 -> transparent {GLASS}")
    g = st2.GetPrimAtPath("/World/DropperSample/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：test_tube.usd 自带玻璃 opacity 0.35 → 0.12（更透明、反光
    更弱，内部液柱看得更清）+ 补 ior 1.5 + roughness 0.25 柔化反光 + doubleSided。"""
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


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rotate in EQUIP:
        add_equip(stage, name, asset, t, scale, rotate)
    add_effects(stage)
    add_sample_color_variants(stage)
    add_water_stream(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    place_stopper_flat(st2)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)
    fix_env_light(st2)
    relocate_absolute_textures(st2)
    fix_bottle_materials(st2)
    fix_dropper_materials(st2)
    fix_tube_material(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
