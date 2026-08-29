# -*- coding: utf-8 -*-
"""生成 d8l_complex_color.usd —— D8-L 络合/显色试剂滴加反应（液体样品，3 试剂）场景（烘平自包含）。

复刻 d3l 模板（用户 2026-08-28："3 个试剂瓶 + 3 个胶头滴管 + 多输入，直接复刻 d3l 模板"）。
D8-L 与 D3-L 同构，仅扩为 **3 支滴管 + 3 个试剂瓶**，液体变色扩为 **3 段**（样品初始色 →
加试剂 1 后色 → 加试剂 2 后色），并加 **分层（LayerBottom 底部有色液相）** 与 **气泡（Bubbles_<色> 小球簇，同 d3l 真实感改造）**。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 试管 + 3 支胶头滴管插进试管架孔里（底面 z=0.806 = 架z−0.0905）
- 去瓶塞（3 瓶已开瓶，瓶口 rim=0.070 → 世界 0.870）+ 去器材残留 env_light
- 内建效果 prim：SampleLiquid/Reagent1Liquid/Reagent2Liquid（3 瓶体积，可见）
  /TubeDrops（管内液滴）/Precipitate（沉淀）/PrecipitateCloud（浑浊云）
  /LayerBottom（分层底部有色液相）/Bubbles_<色>（气泡），后五初始隐藏（task 动画驱动）
- 3 段变色液柱：TubeDropsColor_<stage>_<色>（stage∈{sample,reagent1,reagent2}，
  色∈LIQUID_COLORS），半径逐段递增（sample 0.0084 < reagent1 0.0085 < reagent2 0.0086，
  嵌套：后段在外层盖前段 → 最终色=最后非 clear 段色；headless 下运行时改材质不渲染故预烘焙多组）
- 瓶玻璃透明化（assets 的 bottle_mat 是 op 0.8/rough 0.33 磨砂玻璃，隔它看不清液体）
  → op 0.25 / rough 0.1 / ior 1.5 真玻璃；hcl_bottle 自带 1mm 薄盘液体隐藏，改体积圆柱

布局（3 滴管占架 4 孔 2×2：试管前排左、滴管后排左/后排右/前排右；3 瓶一排）：
  TestTubeRack     (0.30,  0.00)  底座贴台面 z=0.8965
  TestTube         (0.2787, 0.1193, 0.806)  前排左孔（d2s 校准坐标）
  DropperSample    (0.2815, -0.1187, 0.806)  后排左孔 立放
  DropperReagent1  (0.3202, -0.1187, 0.806)  后排右孔 立放
  DropperReagent2  (0.3202,  0.1193, 0.806)  前排右孔 立放（试管同排对侧）
  SampleBottle     (0.4045, 0.3585)  样品瓶（用户调整：台面前方偏右），底座贴台面
  Reagent1Bottle   (0.1696, 0.361)  试剂 1 瓶（hcl_bottle.usd 玻璃瓶），底座贴台面
  Reagent2Bottle   (0.287,  0.361)  试剂 2 瓶（hcl_bottle.usd 玻璃瓶），底座贴台面

用法：python scripts/gen_d8l_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d8l_complex_color")
OUT = os.path.join(SCENE_DIR, "d8l_complex_color.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# 孔位（skill 坑 33 + 实测）：顶层板 14 孔 2 列×7 行，孔心列 x=架x±0.019；插孔底面 z=架z−0.0905
HOLE_BOTTOM = 0.806  # = 架 translate z(0.8965) − 0.0905

# 架孔 2 列 x = 0.30±0.019 ≈ 0.2815（左）/ 0.3202（右）；前排 y=+0.1193、后排 y=-0.1187
# (prim, asset_file, translate, scale)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None),
    ("TestTube", "test_tube.usd", (0.2787, 0.1193, HOLE_BOTTOM), None),
    # 3 支滴管：试管前排左(0.2815,0.1193)对侧放 3 支，尽量分散（用户 2026-08-28）
    ("DropperSample", "dropper.usd", (0.2815, -0.1187, HOLE_BOTTOM), None),
    ("DropperReagent1", "dropper.usd", (0.3202, -0.1187, HOLE_BOTTOM), None),
    ("DropperReagent2", "dropper.usd", (0.3202, 0.1193, HOLE_BOTTOM), None),
    ("SampleBottle", "sample_bottle.usd", (0.4045, 0.3585, None), None),
    ("Reagent1Bottle", "hcl_bottle.usd", (0.1696, 0.361, None), None),
    ("Reagent2Bottle", "hcl_bottle.usd", (0.287, 0.361, None), None),
]

# 翻放瓶盖（2026-08-17 用户："在试剂瓶旁边放上瓶盖（翻着放符合实验室标准）"）。
# 无现成瓶盖资产，自建薄壁杯形 mesh（闭口端朝下贴台面、开口朝上）。尺寸贴合资产 stopper
# （Ø25.2mm × H11mm，材质 white 0.9,0.9,0.92）：r_out=0.0135、r_in=0.0115、h=0.011。
# 放各瓶一侧（x ±65mm 偏移）；D8 三瓶各放一盖。
CAPS = [
    ("SampleBottleCap", 0.4045 + 0.065, 0.3585),
    ("Reagent1BottleCap", 0.1696 - 0.065, 0.361),
    ("Reagent2BottleCap", 0.287 + 0.065, 0.361),
]
CAP_R_OUT, CAP_R_IN, CAP_H = 0.0135, 0.0115, 0.011
CAP_RECIPE = dict(color=(0.9, 0.9, 0.92), opacity=1.0, roughness=0.4)  # 白塑料，同 stopper 配方

# 液体材质配方（最逼真）：roughness 0.05 光洁水面 + ior 1.33 水折射
# 样品=浅蓝水样；试剂 1=淡青蓝、试剂 2=淡琥珀，三瓶液色区分（headless 下无色透明看不见，
# 给淡色区分三瓶）
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)
REAGENT1 = dict(color=(0.68, 0.90, 0.90), opacity=0.70, roughness=0.05, ior=1.33)
REAGENT2 = dict(color=(0.90, 0.85, 0.60), opacity=0.70, roughness=0.05, ior=1.33)
# 沉淀：全哑光(rough 0.85)+ 乳白色（同 d3l 2026-08-23 用户要求"沉淀乳白"）；op 1.0 全不透明
OPAQUE_WHITE = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85, emissive=(1.0, 0.95, 0.80))
# 震荡时的"浑浊云"：灰色浑浊圆柱（2026-08-29 用户"加一层灰色"；几何实现，headless 下
# 运行时改 shader 材质不渲染）。近黑 diffuse + 灰 emissive 主导（灰=三通道均衡，不漂白）。
GRAY_TURBID = dict(color=(0.10, 0.10, 0.11), opacity=0.95, roughness=0.9, emissive=(1.2, 1.2, 1.3))
# 分层底部液相：重相（有机相/密度较大相）。2026-08-29 用户反馈原琥珀太暗发灰、误看成
# 沉淀 → 改亮黄琥珀（R/G 双通道 emissive 主导、B 压低，黄橙分明不与灰色沉淀混淆；
# 复用 flametest-yellow-recipe：近黑 diffuse + emissive 主导，headless 饱和不漂白），
# op 0.95 近不透（隔住下方变色液、界面清晰）。
LAYER = dict(color=(0.15, 0.12, 0.02), opacity=0.95, roughness=0.1, ior=1.33, emissive=(1.9, 1.5, 0.2))
# 滴管内液柱 / 滴落液滴：更亮更不透（op0.9 亮蓝）
FILL = dict(color=(0.35, 0.75, 1.0), opacity=0.90, roughness=0.05, ior=1.33)
DROP = dict(color=(0.35, 0.75, 1.0), opacity=0.90, roughness=0.05, ior=1.33)

# 内建效果 prim: (name, type, radius, height, translate, 材质配方 dict, visible)
# 三瓶内半瓶液体（cyl 从瓶底 0.80 到液面 0.84）；hcl 资产自带 1mm 薄盘液体由 fix 隐藏
# TubeDrops/Precipitate/PrecipitateCloud/LayerBottom = 管内现象；DropperFill = 滴管尖内液柱
# DropperFill frustum 的 r = (r_bottom, r_top)：下底 Ø2mm 贴尖嘴、上底 Ø7mm（内缩玻璃体 Ø8mm）
EFFECTS = [
    ("SampleLiquid", "cylinder", 0.014, 0.040, (0.4045, 0.3585, 0.820), WATER, True),
    ("Reagent1Liquid", "cylinder", 0.014, 0.040, (0.1696, 0.361, 0.820), REAGENT1, True),
    ("Reagent2Liquid", "cylinder", 0.014, 0.040, (0.287, 0.361, 0.820), REAGENT2, True),
    ("TubeDrops", "cylinder", 0.009, 0.030, (0.2787, 0.1193, 0.821), WATER, False),
    ("Precipitate", "cylinder", 0.0088, 0.003, (0.2787, 0.1193, 0.8075), OPAQUE_WHITE, False),
    ("PrecipitateCloud", "cylinder", 0.0089, 0.003, (0.2787, 0.1193, 0.8075), GRAY_TURBID, False),
    ("LayerBottom", "cylinder", 0.0087, 0.003, (0.2787, 0.1193, 0.8075), LAYER, False),
    ("DropperFill", "frustum", (0.001, 0.0035), 0.060, (0.2815, -0.1187, 0.806), FILL, False),
]

# ========== 3 段液体变色（2026-08-28）==========
# headless 渲染下运行时改材质不渲染（记忆 headless-render-ignores-materials），变色走几何：
# 为每段×候选色预烘焙一根"变色液柱"圆柱（TubeDropsColor_<stage>_<色>），task 按
# cfg.{initial_color,color_after_reagent1,color_after_reagent2} show 对应一根，逐滴把 height
# 从液面向下长。三段半径逐段递增（嵌套，后段在外层盖前段 → 最终色=最后非 clear 段色）。
# opacity 0.95 近不透；配方对齐气泡鲜艳色（flametest-yellow-recipe：单通道 emissive 主导 +
# 近黑 diffuse 才饱和，纯 diffuse 被 CylinderLight 12000 洗成灰粉）。
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
# 三段变色液柱半径（逐段递增，嵌套后段在外层盖前段；均略小于液柱 0.009 防穿模、
# 小于 Precipitate 0.0088 / Cloud 0.0089 → 震荡浑浊云罩变色液，静置褪后显色）
STAGE_RADII = {"sample": 0.0084, "reagent1": 0.0085, "reagent2": 0.0086}
STAGES = ["sample", "reagent1", "reagent2"]
LAYER_R = 0.0087   # 分层底部液柱半径（液柱 0.009 内、最外段变色 0.0086 外 → 盖住变色液）

# ========== 气泡方案（复刻 d3l 2026-08-19 真实感改造，中等档）==========
# 与真实反应差距修正：原来 8 颗 Ø14mm 慢速泡从管底一点直线上飘 → 像"烧开水"。改为
# Ø4.4mm 离散小泡 ×40 颗池、速度 ~0.06m/s、管底盘状散布区（中心 30 + 近壁环 10）、蛇形
# 上飘、每滴试剂触发爆发后渐衰（VIGOR_DECAY）。上升动画由 task._step_bubble_anim 驱动：
# 本列表 x/y 是基准、z 全 0.806（task 每帧覆盖 z、子球初始隐藏）。
# 不变量：len(BUBBLES) 必须 == task.N_BUBBLES(40)，verify() 会断言。
# 颜色跟随最终变色（2026-08-24 用户）——每组近黑 diffuse + 该色 emissive 单通道主导。
# clear = 原本液体浅天蓝（WATER 色），其余 = 变色后目标色。task 按最终色（最后非 clear 段）
# 选一组 show（headless 下运行时改材质不渲染，故预烘焙多组）。
BUBBLE_GROUPS = {
    "clear":  dict(color=(0.72, 0.85, 1.0), opacity=1.0, roughness=0.3, emissive=(0.7, 1.0, 1.8)),
    "red":    dict(color=(0.05, 0.02, 0.02), opacity=1.0, roughness=0.3, emissive=(2.6, 0.12, 0.12)),
    "blue":   dict(color=(0.02, 0.04, 0.10), opacity=1.0, roughness=0.3, emissive=(0.15, 0.45, 2.6)),
    "green":  dict(color=(0.02, 0.10, 0.04), opacity=1.0, roughness=0.3, emissive=(0.15, 2.4, 0.15)),
    "purple": dict(color=(0.10, 0.02, 0.12), opacity=1.0, roughness=0.3, emissive=(2.3, 0.18, 2.6)),
}
BUBBLE_R = 0.0022   # Ø4.4mm（管内缘 Ø18mm → 泡缘距壁 ≥ 4.6mm，离散小泡不贴壁）


def _gen_bubbles(n_center=30, n_wall=10, seed=42):
    """生成 40 个基准位（固定种子可复现）：中心盘状区 r≤0.0035 + 近管壁环 r 0.0055~0.0063
    （模拟壁面成核）。近壁上限 0.0063 + 泡半径 0.0022 = 0.0085 < 管内缘 0.009，不插壁。
    z 全写 0.806（管底圆底收敛点，task 每帧覆盖）。试管中心 (0.2787, 0.1193) 同 d3l。"""
    rng = random.Random(seed)
    out = []
    for _ in range(n_center):
        r = 0.0035 * math.sqrt(rng.random())          # 均匀圆盘（sqrt 面积均匀）
        a = 2.0 * math.pi * rng.random()
        out.append((0.2787 + r * math.cos(a), 0.1193 + r * math.sin(a), 0.806))
    for _ in range(n_wall):
        r = 0.0055 + 0.0008 * rng.random()            # 0.0055~0.0063 近壁一圈
        a = 2.0 * math.pi * rng.random()
        out.append((0.2787 + r * math.cos(a), 0.1193 + r * math.sin(a), 0.806))
    return out


BUBBLES = _gen_bubbles()

# 挤胶头滴落串：一次挤 = DROPS_PER_GROUP 滴连续坠落（液柱 60mm 很满，一挤该是一串滴）
DROPS_PER_GROUP = 4


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided。emissive：自发光。"""
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
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (f" scale {scale}" if scale else ""))


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。
    skill 坑 18：滴管尖端内腔锥形，直圆柱液体会悬空穿模，须锥台贴合。16 段 + 底/顶 cap。"""
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
                     emissive=m.get("emissive"),
                     double_sided=translucent)
        if not visible:
            UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} {'visible' if visible else 'hidden'} at {t} "
              f"(op {m['opacity']} rough {m.get('roughness', 0.5)} ior {m.get('ior')})")


def add_color_liquid(stage):
    """3 段候选色变色液柱（2026-08-28）：为 STAGES×LIQUID_COLORS 每个组合建一根同轴圆柱
    （/World/TubeDropsColor_<stage>_<色>），初始全隐藏、height 0；task 按三段输入 show 对应
    一根，逐滴改 height（顶贴液面向下扩散）。几何实现——headless 下运行时改材质不渲染。"""
    for stage_name in STAGES:
        r_stage = STAGE_RADII[stage_name]
        for name, m in LIQUID_COLORS.items():
            geom = UsdGeom.Cylinder.Define(stage, f"/World/TubeDropsColor_{stage_name}_{name}")
            geom.CreateRadiusAttr(r_stage)
            geom.CreateHeightAttr(0.0)
            geom.CreateAxisAttr("Z")
            geom.AddTranslateOp().Set(Gf.Vec3d(0.2787, 0.1193, 0.806))
            translucent = m.get("opacity", 1.0) < 1.0
            add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                         roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                         emissive=m.get("emissive"), double_sided=translucent)
            UsdGeom.Imageable(geom).MakeInvisible()
            print(f"[effect] TubeDropsColor_{stage_name}_{name} hidden (r={r_stage})")


def add_bubbles(stage):
    """气泡组 ×5（复刻 d3l 2026-08-24：气泡颜色跟随液体变色）：/World/Bubbles_<色> 每组
    40 颗球（基准同 BUBBLES），颜色 = 该色近黑 diffuse + 单通道 emissive。初始全隐藏；task
    按最终色（最后非 clear 段）show 对应一组（clear=原本液体浅天蓝，其余=变色后目标色）。"""
    for name, recipe in BUBBLE_GROUPS.items():
        g = UsdGeom.Xform.Define(stage, f"/World/Bubbles_{name}")
        for i, (x, y, z) in enumerate(BUBBLES):
            s = UsdGeom.Sphere.Define(stage, f"/World/Bubbles_{name}/Bubble_{i}")
            s.CreateRadiusAttr(BUBBLE_R)
            s.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            add_material(stage, s.GetPrim(), recipe["color"], recipe["opacity"],
                         roughness=recipe["roughness"], emissive=recipe["emissive"])
        UsdGeom.Imageable(g).MakeInvisible()
        print(f"[effect] Bubbles_{name} hidden ({len(BUBBLES)} spheres, r={BUBBLE_R})")


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 Xform + Drop_0.._N 亮蓝小球（r=0.003）。
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。整体初始隐藏。"""
    g = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    for i in range(DROPS_PER_GROUP):
        s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/Drop_{i}")
        s.CreateRadiusAttr(0.003)
        s.AddTranslateOp().Set(Gf.Vec3d(0.2787, 0.1193, 0.820))
        add_material(stage, s.GetPrim(), DROP["color"], DROP["opacity"],
                     roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DropperDrop hidden ({DROPS_PER_GROUP} drop spheres)")


def add_cap(stage, name, x, y):
    """翻放瓶盖（实验室标准：闭口端贴台面、开口朝上，内面不触桌防污染）。
    薄壁杯形 mesh，局部坐标底在 z=0（=台面顶），开口在 z=CAP_H。subdivisionScheme=none。"""
    n = 20
    pts, counts, indices = [], [], []

    def ring(z, r):
        for i in range(n):
            a = 2.0 * math.pi * i / n
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), z))

    ring(0.0, CAP_R_OUT)                    # 0..19   外壁底
    ring(CAP_H, CAP_R_OUT)                  # 20..39  外壁顶
    ring(0.0, CAP_R_IN)                     # 40..59  内壁底
    ring(CAP_H, CAP_R_IN)                   # 60..79  内壁顶
    pts.append(Gf.Vec3f(0, 0, 0))           # 80      底心
    O, OT, I, IT, C = 0, n, 2 * n, 3 * n, 4 * n
    for i in range(n):
        i0, i1 = i, (i + 1) % n
        counts.append(4); indices += [O + i0, O + i1, OT + i1, OT + i0]   # 外壁
        counts.append(4); indices += [I + i0, IT + i0, IT + i1, I + i1]   # 内壁（反向绕序朝内）
    for i in range(n):                                                    # 底环（R_in..R_out 实底）
        i0, i1 = i, (i + 1) % n
        counts.append(4); indices += [I + i0, O + i0, O + i1, I + i1]
    for i in range(n):                                                    # 腔底内圆（底心→内壁底）
        i0, i1 = i, (i + 1) % n
        counts.append(3); indices += [C, I + i0, I + i1]
    mesh = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.AddTranslateOp().Set(Gf.Vec3d(x, y, TABLE_TOP))
    add_material(stage, mesh.GetPrim(), CAP_RECIPE["color"], CAP_RECIPE["opacity"],
                 roughness=CAP_RECIPE["roughness"], double_sided=True)
    print(f"[equip] {name} flipped cap (r_out {CAP_R_OUT}, r_in {CAP_R_IN}, h {CAP_H}) "
          f"at ({x}, {y}, {TABLE_TOP})")
    return mesh


def add_caps(stage):
    for name, x, y in CAPS:
        add_cap(stage, name, x, y)


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃滴管/试管在无环境反射下照不亮。
    贴图路径用相对 ./textures/，烘平后由 fix_env_light() 在场景层重指向 textures/env_bright.png。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（用户 2026-08-19：x 调到 -10）。"""
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
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效）。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_stoppers(st2):
    """去瓶塞：实验已开瓶，删 3 瓶自带的 stopper + stopper_mat
    （覆盖在瓶口上 0.068..0.079，删后瓶口 rim=0.070 → 世界 0.870）。
    注意：stage.Export 烘平会把引用资产的 root 包装 Xform 合并进引用 prim，故用遍历匹配。"""
    for name in ("SampleBottle", "Reagent1Bottle", "Reagent2Bottle"):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[clean] /World/{name} not found, skip")
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetName() in ("stopper", "stopper_mat")]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")
        if not paths:
            print(f"[clean] /World/{name} has no stopper/stopper_mat")


def remove_stray_env_lights(st2):
    """清理器材资产自带的残留 DomeLight（纯黑环境贴图污染整场景光照，v17 环境暗根因）。
    只保留 gen 自建的 /World/env_light 与 /World/CylinderLight 主光。"""
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
    """自检：打印各器材/效果世界 bbox，确认孔位/瓶口高度符合设计；并断言 3 段变色液柱、
    分层液柱不变量（纯 pxr）。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "DropperSample", "DropperReagent1",
             "DropperReagent2", "SampleBottle", "Reagent1Bottle", "Reagent2Bottle",
             "SampleLiquid", "Reagent1Liquid", "Reagent2Liquid",
             "TubeDrops", "Precipitate", "PrecipitateCloud", "LayerBottom",
             "DropperFill", "DropperDrop",
             "SampleBottleCap", "Reagent1BottleCap", "Reagent2BottleCap"]
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:17s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 3 段候选色变色液柱不变量：每段每色存在、radius==STAGE_RADII[stage]、初始隐藏
    for stage_name in STAGES:
        for name in LIQUID_COLORS:
            p = st2.GetPrimAtPath(f"/World/TubeDropsColor_{stage_name}_{name}")
            assert p.IsValid(), f"TubeDropsColor_{stage_name}_{name} missing"
            r = UsdGeom.Cylinder(p).GetRadiusAttr().Get()
            assert abs(r - STAGE_RADII[stage_name]) < 1e-9, \
                f"TubeDropsColor_{stage_name}_{name} r={r} != {STAGE_RADII[stage_name]}"
            vis = UsdGeom.Imageable(p).ComputeVisibility() == "invisible"
            assert vis, f"TubeDropsColor_{stage_name}_{name} should be hidden initially"
    print(f"[verify] LiquidColor OK: {len(STAGES)} stages x {len(LIQUID_COLORS)} colors, "
          f"radii {STAGE_RADII}, all hidden")
    # 分层液柱不变量：存在、radius==LAYER_R、初始隐藏
    lp = st2.GetPrimAtPath("/World/LayerBottom")
    assert lp.IsValid(), "LayerBottom missing"
    assert abs(UsdGeom.Cylinder(lp).GetRadiusAttr().Get() - LAYER_R) < 1e-9, \
        f"LayerBottom r != LAYER_R"
    assert UsdGeom.Imageable(lp).ComputeVisibility() == "invisible", \
        "LayerBottom should be hidden initially"
    print(f"[verify] LayerBottom OK: r={LAYER_R}, hidden")
    # 气泡组不变量（复刻 d3l）：每组数量==len(BUBBLES)==task.N_BUBBLES、半径==BUBBLE_R、
    # 泡缘不插管壁、初始隐藏。
    TUBE_INNER_R = 0.009  # 管内缘 Ø18mm / 2
    for gname in BUBBLE_GROUPS:
        bubbles = st2.GetPrimAtPath(f"/World/Bubbles_{gname}")
        assert bubbles.IsValid(), f"Bubbles_{gname} missing"
        n = len([c for c in bubbles.GetChildren() if c.GetTypeName() == "Sphere"])
        assert n == len(BUBBLES), \
            f"Bubbles_{gname} children {n} != len(BUBBLES)={len(BUBBLES)}"
        for i, (bx, by, bz) in enumerate(BUBBLES):
            p = st2.GetPrimAtPath(f"/World/Bubbles_{gname}/Bubble_{i}")
            assert p.IsValid(), f"Bubbles_{gname}/Bubble_{i} missing"
            r = UsdGeom.Sphere(p).GetRadiusAttr().Get()
            assert abs(r - BUBBLE_R) < 1e-9, \
                f"{gname} Bubble_{i} r={r} != BUBBLE_R={BUBBLE_R}"
            dr = math.hypot(bx - 0.2787, by - 0.1193)
            assert dr + r <= TUBE_INNER_R, \
                f"{gname} Bubble_{i} clips wall: dr+r={dr + r:.4f} > inner {TUBE_INNER_R}"
        assert UsdGeom.Imageable(bubbles).ComputeVisibility() == "invisible", \
            f"Bubbles_{gname} should be hidden initially"
        print(f"[verify] Bubbles_{gname} OK: {n} spheres r={BUBBLE_R}, "
              f"all inside tube, hidden")
    # 残留 DomeLight 必须清干净（纯黑环境贴图会污染整场景光照）
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print("[verify] no stray DomeLight")


# 瓶玻璃配方：assets 自带 bottle_mat 是 op0.8/rough0.33 磨砂玻璃，隔它看不清液体
# → 真玻璃 op 0.25 / rough 0.1 / ior 1.5，液体才透得出来
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数。烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply（会告警），故直接用 material:binding relationship
    取材质路径再找 shader。"""
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
    """瓶玻璃透明化 + hcl 瓶 1mm 液面盘隐藏（改用 Reagent*Liquid 体积表现）。
    hcl_bottle 自带 liquid 实测 z 0.040..0.041 = 1mm 厚薄盘（非半瓶），隐藏掉。"""
    for name in ("SampleBottle", "Reagent1Bottle", "Reagent2Bottle"):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh":
                continue
            if c.GetName() == "liquid":
                UsdGeom.Imageable(c).MakeInvisible()
                print(f"[mat] hid {c.GetPath()} (1mm liquid disc, replaced by volume cyl)")
            else:
                if override_bound_shader(st2, c, GLASS):
                    UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 是 opacity=1.0 不透明光面（把管内液柱
    整个遮住），改成真玻璃 op 0.25，管内 DropperFill 液柱才透得出来；胶头保持不透明。"""
    for name in ("DropperSample", "DropperReagent1", "DropperReagent2"):
        mat = st2.GetPrimAtPath(f"/World/{name}/_materials/glass_001")
        if not mat.IsValid():
            print(f"[mat] {name} glass_001 not found, skip")
            continue
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
            print(f"[mat] {name} glass_001 -> transparent {GLASS}")
    # 玻璃 mesh 双面渲染（透过玻璃看后壁，不设会漏空）
    for name in ("DropperSample", "DropperReagent1", "DropperReagent2"):
        g = st2.GetPrimAtPath(f"/World/{name}/glass_body_mesh/glass_body_mesh_001")
        if g.IsValid() and g.GetTypeName() == "Mesh":
            UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] {g.GetPath()} doubleSided")
        else:
            print(f"[mat] {name} glass mesh not found for doubleSided, skip")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：test_tube.usd 自带玻璃 opacity 0.35 → 0.12 + ior 1.5 +
    roughness 0.05 → 0.25（柔化锐利高光，不盖管底沉淀）。"""
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
    for name, asset, t, scale in EQUIP:
        add_equip(stage, name, asset, t, scale)
    add_effects(stage)
    add_color_liquid(stage)   # 3 段候选色变色液柱（初始全隐藏）
    add_bubbles(stage)        # 气泡组 ×5（颜色跟随最终变色，初始全隐藏）
    add_dropper_drops(stage)
    add_caps(stage)          # 3 瓶翻放瓶盖（2026-08-17 用户要求，闭口朝下开口朝上）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_stoppers(st2)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)
    fix_env_light(st2)
    relocate_absolute_textures(st2)
    fix_bottle_materials(st2)   # 3 瓶玻璃透明化 + hcl 1mm 液面盘隐藏
    fix_dropper_materials(st2)  # 3 支滴管玻璃透明化（不透明会遮住管内液柱）
    fix_tube_material(st2)      # 试管玻璃透明化（op 0.35→0.12，内部现象看得更清）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
