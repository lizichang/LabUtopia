# -*- coding: utf-8 -*-
"""生成 d4l_alkali_reagent.usd —— D4-L 碱性试剂滴加反应（液体样品）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材（lab_clean 干净，无需删任何 prim、无需抬台面）
- 试管 + 两支胶头滴管插进试管架孔里（底面 z=0.806 = 架z−0.0905，孔心对齐顶层板孔位）
- 去样品瓶塞（样品已开瓶）+ 去试管架残留 env_light；**碱瓶（alkaline_bottle.usd）自带
  rubber_stopper 橡胶塞翻放到桌面倒放**（同 D3-L 瓶盖模式，gen 静态摆，无拔塞动作）
- 内建效果 prim：SampleLiquid（样品瓶体积）/AlkaliLiquid（碱瓶体积，可见）
  /TubeDrops（管内液滴）/Bubbles（气泡）/Precipitate（沉淀），后三初始隐藏（task 动画驱动）
- 最逼真液体配方：roughness 0.05 光洁水面 + ior 1.33 水折射 + opacity 0.45 + doubleSided
  （参考 lab_003 酒精灯独立 liquid mesh 方案，但比它的灰色不透明 op1.0 更像水）
- 样品瓶玻璃透明化（assets 的 bottle_mat 是 op 0.8/rough 0.33 磨砂玻璃，隔它看不清液体）
  → op 0.25 / rough 0.1 / ior 1.5 真玻璃；**碱瓶是塑料瓶（op1.0 不透明）不玻璃化**

布局（与 D3-L 相同，仅把 HCl 试剂瓶换成碱性试剂瓶；碱瓶塞子翻放桌面倒放）：
  TestTubeRack  (0.30,  0.00)  底座贴台面 z=0.8965
  TestTube      (0.2787, 0.1193, 0.806)  前排左孔（d2s 校准坐标）
  DropperSample (0.2815, -0.1187, 0.806)  后排左孔 立放（离试管最远，用户 2026-08-14 调整）
  DropperAlkali (0.3202, -0.1187, 0.806)  后排右孔 立放
  SampleBottle  (0.4045, 0.3585)  样品瓶（用户调整：台面前方偏右），底座贴台面
  AlkaliBottle  (0.1696, 0.361)   碱性试剂瓶（塑料瓶+橡胶塞，用户 2026-08-24 新建），
                                  底座贴台面、瓶口敞（rim=0.070 → 世界 0.870）；
                                  橡胶塞翻放桌面 (0.1046, 0.361) 大端朝下触台面 0.80

用法：python scripts/gen_d4l_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d4l_alkali_reagent")
OUT = os.path.join(SCENE_DIR, "d4l_alkali_reagent.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# 孔位（skill 坑 33 + 实测）：顶层板 14 孔 2 列×7 行，孔心列 x=架x±0.019；插孔底面 z=架z−0.0905
HOLE_BOTTOM = 0.806  # = 架 translate z(0.8965) − 0.0905

# (prim, asset_file, translate, scale)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None),
    ("TestTube", "test_tube.usd", (0.2787, 0.1193, HOLE_BOTTOM), None),
    # 滴管挪到离试管最远的后排孔（试管前排 y=+0.119 → 滴管后排 y=-0.1187，
    # 用户 2026-08-14 要求：两个滴管放到离试管最远的最靠边一列）
    ("DropperSample", "dropper.usd", (0.2815, -0.1187, HOLE_BOTTOM), None),
    ("DropperAlkali", "dropper.usd", (0.3202, -0.1187, HOLE_BOTTOM), None),
    ("SampleBottle", "sample_bottle.usd", (0.4045, 0.3585, None), None),
    ("AlkaliBottle", "alkaline_bottle.usd", (0.1696, 0.361, None), None),
]

# 翻放瓶盖（2026-08-17 用户："在两个试剂瓶旁边放上瓶盖（翻着放符合实验室标准）"）。
# 无现成瓶盖资产，自建薄壁杯形 mesh（闭口端朝下贴台面、开口朝上）。尺寸贴合资产 stopper
# （Ø25.2mm × H11mm，材质 white 0.9,0.9,0.92）：r_out=0.0135(Ø27 略大于瓶口读作瓶盖)、
# r_in=0.0115(壁厚 2mm)、h=0.011(同 stopper 高)。放各瓶一侧（x ±65mm 偏移）。
# D4-L：样品瓶放翻放瓶盖；碱瓶塞子 = 自带 rubber_stopper 静态翻放桌面（flip_alkali_stopper），
# 不放盖。两瓶都不做机械臂拔/盖塞动作（用户 2026-08-24："d3l 也没有拿瓶盖的动作，
# 我就让你摆放在桌面上"）。
CAPS = [
    ("SampleBottleCap", 0.4045 + 0.065, 0.3585),
]
CAP_R_OUT, CAP_R_IN, CAP_H = 0.0135, 0.0115, 0.011
CAP_RECIPE = dict(color=(0.9, 0.9, 0.92), opacity=1.0, roughness=0.4)  # 白塑料，同 stopper 配方

# 液体材质配方（最逼真）：roughness 0.05 光洁水面 + ior 1.33 水折射
# （2026-08-14 用户反馈"液体痕迹不明显"：0.45 太透、隔着玻璃看不清 → 提到 0.70+提亮；
#   2026-08-19 用户反馈"蓝色液体不够透明看不到沉淀"：0.70 遮管底沉淀 → 0.55；
#   再反馈"颜色还是太深"：浅蓝 0.58→0.72 + opacity 0.55→0.50，让白沉淀透过浅水显形）
# alkali 青蓝区分碱液/水样（碱液常用无色，但 headless 下无色透明看不见，故给淡青蓝）
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)
ALKALI = dict(color=(0.68, 0.90, 0.90), opacity=0.70, roughness=0.05, ior=1.33)
# 沉淀：全哑光(rough 0.85 无自身高光)+ 乳白色（2026-08-23 用户要求"沉淀由红色改回乳白色"）。
# 乳白=暖调微奶油白（R>G>B 略偏暖,不是灰白）；op 1.0 全不透明（半透明白嵌在半透明蓝液柱内
# 会被蓝液盖住看不清——08-20"白雾没显示"教训,故红色阶段也保持 op 1.0）。emissive 白色温和档
# 提亮可见性（红单通道 1.8 太强,白两通道过 1 会被光洗成纯白丢乳白调）。
OPAQUE_WHITE = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85, emissive=(1.0, 0.95, 0.80))
# 震荡时的"浑浊云"：乳白色不透明圆柱（几何实现——headless 下运行时改 shader 材质不渲染，
# 2026-08-20 用户反馈"浑浊和之前一模一样"根因）。高度由 task 动画：拎起→盖满整根液柱
# （整管液体变乳白）；归架→缩到 0（蓝液柱 + 管底乳白柱）。半径 0.0089 略小于液柱 0.009，
# 盖在蓝液柱内不穿模。2026-08-23 随沉淀改回乳白（08-22 曾改亮红）。
CLOUD_MILK = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85, emissive=(1.0, 0.95, 0.80))
# 滴管内液柱 / 滴落液滴：更亮更不透（op0.9 亮青蓝，碱液色），透过透明玻璃清晰可见
FILL = dict(color=(0.40, 0.85, 0.95), opacity=0.90, roughness=0.05, ior=1.33)
DROP = dict(color=(0.40, 0.85, 0.95), opacity=0.90, roughness=0.05, ior=1.33)

# 内建效果 prim: (name, type, radius, height, translate, 材质配方 dict, visible)
# SampleLiquid = 样品瓶内半瓶液体（cyl 从瓶底 0.80 到液面 0.84）
# TubeDrops/Precipitate = 管内液滴/沉淀；Bubbles 单独建（小球簇）
# （D4-L 无 AlkaliLiquid：碱瓶是不透明塑料瓶，内部液体看不见，不加死代码圆柱；
#   碱瓶自带 liquid 38mm 正常体积保留，fix_bottle_materials 不碰碱瓶）
# DropperFill = 滴管尖内吸起的液体柱（skill 坑 18：尖端容器内腔是锥形，直圆柱会悬空穿模）。
#   滴管玻璃体实测：尖嘴 Ø1.6mm(z=0) → 30mm 处 Ø8mm → 直管 Ø8mm 到胶头。吸液后液体填满
#   收窄尖端，故用截锥 mesh：下底 Ø2mm(尖) → 上底 Ø8mm(体)，高 40mm（覆盖收窄段 0..30mm
#   + 直管 10mm）。柱底贴尖嘴（task 逐帧跟随），初始隐藏。
# TubeDrops = 管内液体（0.008→0.009 贴管壁 Ø19.2 内缘、0.020→0.030 更高更显眼）
EFFECTS = [
    ("SampleLiquid", "cylinder", 0.014, 0.040, (0.4045, 0.3585, 0.820), WATER, True),
    ("TubeDrops", "cylinder", 0.009, 0.030, (0.2787, 0.1193, 0.821), WATER, False),
    ("Precipitate", "cylinder", 0.0088, 0.003, (0.2787, 0.1193, 0.8075), OPAQUE_WHITE, False),
    ("PrecipitateCloud", "cylinder", 0.0089, 0.003, (0.2787, 0.1193, 0.8075), CLOUD_MILK, False),
    # frustum 的 r = (r_bottom, r_top)：下底 Ø2mm 贴尖嘴、上底 Ø7mm（内缩玻璃体 Ø8mm 一个壁厚，
    # 透过透明玻璃可见独立液柱）。h=60mm（收窄段 0..30mm + 直管 30mm，明显可见）。
    # translate 是 mesh 底心（底在局部 z=0）→ 落在尖嘴 z=0.806，柱体 0.806..0.866 在玻璃体
    # 0..0.12 内、不露在尖嘴外（task._set_fill_follow 用同一约定：translate=尖嘴）。
    ("DropperFill", "frustum", (0.001, 0.0035), 0.060, (0.2815, -0.1187, 0.806), FILL, False),
]

# ========== 滴加酸后液体变色（2026-08-24）==========
# headless 渲染下运行时改材质不生效（记忆 headless-render-ignores-materials / 上方
# CLOUD_MILK 注释已证实），变色必须走几何：为每个候选色预烘焙一根"变色液柱"圆柱
# （TubeDropsColor_<色>），task 按 cfg.liquid_color show 对应一根，逐滴把 height 从
# 液面向下长（_color_frac，顶贴液面向下扩散）。
# 半径 0.0086 略小于液柱 0.009 防穿模；小于 Precipitate 0.0088 / Cloud 0.0089 →
# 震荡浑浊时云罩住变色柱（看不清颜色），静置云褪后变色液显现。初始全隐藏、height 0。
# opacity 0.95 近不透（0.85 半透时底下浅天蓝液柱透出、蓝+红混成灰粉——2026-08-24 用户
# "透过玻璃看偏灰粉不红"根因）。配方对齐气泡鲜艳色（flametest-yellow-recipe：单通道
# emissive 主导 + 近黑 diffuse 才饱和，纯 diffuse 红被 CylinderLight 12000 洗成灰粉）。
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
TUBE_COLOR_R = 0.0086   # 变色液柱半径（略小于液柱 0.009 防穿模）

# ========== 气泡方案（2026-08-19 真实感改造，中等档）==========
# 与真实反应差距修正（用户确认问题）：原来 8 颗 Ø14mm 慢速(0.024m/s)泡从管底一点直线
# 上飘 → 像"烧开水"。本次改为：Ø4.4mm 离散小泡 ×40 颗池、速度 ~0.06m/s、管底盘状散布区
# （中心 30 + 近壁环 10，模拟壁面成核）、蛇形上飘（task 每帧加摆动）、每滴酸触发爆发后
# 渐衰（VIGOR_DECAY，像反应物消耗）。上升动画仍由 task._step_bubble_anim 驱动：
# 本列表 x/y 是基准、z 全 0.806（task 每帧覆盖 z、子球初始隐藏）。
# 不变量：len(BUBBLES) 必须 == task.N_BUBBLES(40)，verify() 会断言。
# 颜色：跟随液体变色（2026-08-24 用户）——每组近黑 diffuse + 该色 emissive 单通道主导
# （emissive 主导才出饱和色，才不被 0.50 不透明蓝液柱遮住）。clear = 原本液体浅天蓝
# （WATER 色），其余 = 变色后目标色。task 按 cfg.liquid_color 选一组 show（headless 下
# 运行时改材质不渲染，故预烘焙多组）。
# —— 1d. 如何改回"无色真气泡" ——
# 把 BUBBLE_GROUPS 里某组 opacity 降到 0.30、emissive 归零 + 重跑 gen 即可；
# 风险：透明无色泡对蓝液柱可见性差——若看不清，先降 EFFECTS 里 TubeDrops 的
# WATER 液柱 opacity，再调高气泡 opacity。
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
    z 全写 0.806（管底圆底收敛点，task 每帧覆盖）。"""
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

# 挤胶头滴落串：一次挤 = DROPS_PER_GROUP 滴连续坠落（液柱 60mm 很满，一挤该是一串滴
# 不是一滴——用户 2026-08-14）。DropperDrop 是父 Xform，task 动画驱动 Drop_0.._N 各球。
DROPS_PER_GROUP = 4


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided，
    否则从外看透过液体时后壁不渲染会像空容器。emissive：自发光（高亮小件用，
    如气泡）。"""
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
                     emissive=m.get("emissive"),
                     double_sided=translucent)
        if not visible:
            UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} {'visible' if visible else 'hidden'} at {t} "
              f"(op {m['opacity']} rough {m.get('roughness', 0.5)} ior {m.get('ior')})")


def add_color_liquid(stage):
    """候选色变色液柱（滴加酸后液体变色，2026-08-24）：为 LIQUID_COLORS 每个候选色建
    一根同轴圆柱（/World/TubeDropsColor_<色>），初始全隐藏、height 0；task 按
    cfg.liquid_color show 对应一根，逐滴改 height（顶贴液面向下扩散，_color_frac）。
    几何实现——headless 下运行时改材质不渲染（见 LIQUID_COLORS 注释）。"""
    for name, m in LIQUID_COLORS.items():
        geom = UsdGeom.Cylinder.Define(stage, f"/World/TubeDropsColor_{name}")
        geom.CreateRadiusAttr(TUBE_COLOR_R)
        geom.CreateHeightAttr(0.0)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(0.2787, 0.1193, 0.806))
        translucent = m.get("opacity", 1.0) < 1.0
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     emissive=m.get("emissive"), double_sided=translucent)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] TubeDropsColor_{name} hidden (liquid color, r={TUBE_COLOR_R})")


def add_bubbles(stage):
    """气泡组 ×5（2026-08-24 用户：气泡颜色跟随液体变色）：/World/Bubbles_<色> 每组 40 颗
    球（基准同 BUBBLES），颜色 = 该色近黑 diffuse + 单通道 emissive。初始全隐藏；task 按
    cfg.liquid_color show 对应一组（clear=原本液体浅天蓝，其余=变色后目标色）。"""
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
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。整体初始隐藏；
    home 位置随意（试管口），task 动画才写实际坐标。"""
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

    薄壁杯形 mesh：外壁 + 内壁 + 底环(R_in..R_out 实底) + 腔底内圆。n=20 段、
    subdivisionScheme=none（memory: 自建 mesh 必须设，否则不显色）。局部坐标底在 z=0
    （= 台面顶），开口在 z=CAP_H。doubleSided 兜底绕序。"""
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
    贴图路径用相对 ./textures/ 会在 stage.Export 时按 lab_clean 层解析成不存在路径，
    烘平后由 fix_env_light() 在场景层重新指向 textures/env_bright.png。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件
    （d2s 药匙反黑经验：金属/玻璃细杆需 12000）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（用户 2026-08-19：x 调到 -10）。
    base lab_clean 的灯位 x=2.1 偏 +X 侧；只动 translate 的 x，y/z 保持。"""
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
    烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向场景目录下的贴图。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_stoppers(st2):
    """去样品瓶塞：样品已开瓶，删 sample_bottle 自带的 stopper + stopper_mat
    （覆盖在瓶口上 0.068..0.079，删后瓶口 rim=0.070 → 世界 0.870）。

    **碱瓶塞翻放**：alkaline_bottle 自带 rubber_stopper 橡胶塞，D4-L 任务无机械臂
    拔塞动作（同 D3-L 瓶盖模式），gen 静态翻放到桌面倒放——见 flip_alkali_stopper。

    注意：stage.Export 烘平会把引用资产的 root 包装 Xform 合并进引用 prim
    （不是 /World/<瓶>/root/stopper，而是 /World/<瓶>/stopper），故用遍历匹配。"""
    for name in ("SampleBottle",):
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
    flip_alkali_stopper(st2)


def flip_alkali_stopper(st2):
    """碱瓶橡胶塞：从瓶口摘出，静态倒放桌面（同 D3-L SampleBottleCap 模式，无机械臂
    动作）。保留原 prim（复用 alkaline_bottle 的 rubber_stopper 几何/材质），只改 local
    xform：translate 移到碱瓶左侧 65mm、rotateXYZ 绕 y 转 180° 让大端朝下。塞子局部
    原点在几何底（小端面），世界位姿 = AlkaliBottle_T(0.1696,0.361,0.79965)·S·R_y180·
    T(-0.065,0,+0.01362) → 世界原点 (0.1046,0.361,0.81327)、大端触台面 0.80（实验室
    标准倒放：大端朝下触台面、小端朝上不触台面防污染）。"""
    stopper = None
    for pp in Usd.PrimRange(st2.GetPrimAtPath("/World/AlkaliBottle")):
        if pp.GetName() == "rubber_stopper":
            stopper = pp
            break
    if stopper is None:
        print("[clean] AlkaliBottle rubber_stopper missing, skip flip")
        return
    t = stopper.GetAttribute("xformOp:translate")
    r = stopper.GetAttribute("xformOp:rotateXYZ")
    if t:
        t.Set(Gf.Vec3d(-0.065, 0.0, 0.01362))
    if r:
        r.Set(Gf.Vec3d(0.0, 180.0, 0.0))
    print("[clean] AlkaliBottle rubber_stopper flipped to desk "
          "(inverted, big end down on table 0.80)")


def remove_stray_env_lights(st2):
    """清理器材资产自带的残留 DomeLight（根因 2026-08-24：alkaline_bottle.usd /
    test_tube_rack.usd 等自带 color_0C0C0C 纯黑环境贴图 DomeLight，残留会污染环境光
    → 整场景变暗 + 黑反射块，同 v17 环境暗根因）。只保留 gen 自建的 /World/env_light
    （亮环境贴图）与 /World/CylinderLight 主光。TestTubeRack 残留曾由旧版单独清理，
    现泛化到全部器材（覆盖碱瓶 AlkaliBottle 等）。"""
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
    """坑 12：烘平后材质贴图 asset 属性若为仓库内绝对路径 → 改为场景相对路径。
    env_bright.png 已由 fix_env_light 指向场景相对，这里兜底器材自带纹理。"""
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
    """自检：打印各器材/效果世界 bbox，确认孔位/瓶口高度符合设计；并断言气泡不变量
    （数量 == len(BUBBLES) == task.N_BUBBLES、半径 == BUBBLE_R、泡缘不插管壁 Ø18 内缘）。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "DropperSample", "DropperAlkali",
             "SampleBottle", "AlkaliBottle", "SampleLiquid",
             "TubeDrops", "Precipitate", "PrecipitateCloud", "Bubbles",
             "DropperFill", "DropperDrop", "SampleBottleCap"]
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:15s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 气泡组不变量校验（纯 pxr，改 BUBBLE_GROUPS/BUBBLES/BUBBLE_R/task.N_BUBBLES 防回归）：
    # 每组数量==len(BUBBLES)==task.N_BUBBLES、半径==BUBBLE_R、泡缘不插管壁、初始隐藏。
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
    # 候选色变色液柱不变量：每根存在、radius==TUBE_COLOR_R、初始隐藏（task 运行时按
    # cfg.liquid_color show 对应一根 + 改 height）
    for name in LIQUID_COLORS:
        p = st2.GetPrimAtPath(f"/World/TubeDropsColor_{name}")
        assert p.IsValid(), f"TubeDropsColor_{name} missing"
        r = UsdGeom.Cylinder(p).GetRadiusAttr().Get()
        assert abs(r - TUBE_COLOR_R) < 1e-9, \
            f"TubeDropsColor_{name} r={r} != TUBE_COLOR_R={TUBE_COLOR_R}"
        vis = UsdGeom.Imageable(p).ComputeVisibility() == "invisible"
        assert vis, f"TubeDropsColor_{name} should be hidden initially"
    print(f"[verify] LiquidColor OK: {len(LIQUID_COLORS)} tubes "
          f"r={TUBE_COLOR_R} all hidden")
    # 碱瓶橡胶塞断言：静态倒放桌面（用户 2026-08-24：无拔塞动作，gen 摆桌面）。
    # 大端朝下触台面 0.80、小端朝上 0.81327（塞子几何高 13.27mm）→ 世界 bbox z 0.80..0.81327、
    # xy ≈ (0.1046, 0.361)。
    stopper = None
    for pp in Usd.PrimRange(st2.GetPrimAtPath("/World/AlkaliBottle")):
        if pp.GetName() == "rubber_stopper":
            stopper = pp
            break
    assert stopper is not None, "AlkaliBottle rubber_stopper missing"
    r = bc.ComputeWorldBound(stopper).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    print(f"[verify] AlkaliStopper    min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
          f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    assert abs(mn[2] - 0.800) < 1e-3, f"stopper bottom {mn[2]:.4f} not on table 0.80"
    assert abs(mx[2] - 0.81327) < 1e-3, f"stopper top {mx[2]:.4f} != 0.81327 (inverted)"
    # 中心 xy（bbox min/max 是塞子边缘；Ø18.3mm 半宽 0.00915，中心 = 均值）
    cx = (mn[0] + mx[0]) / 2.0
    cy = (mn[1] + mx[1]) / 2.0
    assert abs(cx - 0.1046) < 3e-3, f"stopper center x {cx:.4f} not at 0.1046"
    assert abs(cy - 0.361) < 3e-3, f"stopper center y {cy:.4f} not at 0.361"
    # 倒放姿态（大端朝下触台面）：rotateXYZ 必须绕 y 转 180°
    rot = stopper.GetAttribute("xformOp:rotateXYZ")
    if rot and rot.HasValue():
        v = rot.Get()
        assert abs(v[1] - 180.0) < 1.0, f"stopper rotateXYZ.y={v[1]:.1f} != 180 (not inverted)"
    # 残留 DomeLight 必须清干净（纯黑环境贴图会污染整场景光照，2026-08-24 用户反馈 D2-L 偏暗）
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
    """样品瓶玻璃透明化（真玻璃 op 0.25 让瓶内 SampleLiquid 透出）。

    **碱瓶完全不碰**：alkaline_bottle 是塑料瓶（diffuse op1.0 不透明，正常试剂瓶），
    不自带玻璃化；其 liquid 是 38mm 正常体积（非 hcl 的 1mm 薄盘），无需隐藏
    （被不透明瓶身挡住本就看不见）。"""
    for name in ("SampleBottle",):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh":
                continue
            if c.GetName() == "liquid":
                UsdGeom.Imageable(c).MakeInvisible()
                print(f"[mat] hid {c.GetPath()} (sample 1mm liquid disc)")
            else:
                if override_bound_shader(st2, c, GLASS):
                    UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
    print("[mat] AlkaliBottle untouched (opaque plastic bottle, liquid kept)")


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 是 opacity=1.0 不透明光面（把管内液柱
    整个遮住，用户反馈"液柱不明显"根因）。改成真玻璃 op 0.25（同瓶玻璃配方），
    管内 DropperFill 液柱才透得出来；胶头（rubber_001）保持不透明。"""
    for name in ("DropperSample", "DropperAlkali"):
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
    for name in ("DropperSample", "DropperAlkali"):
        g = st2.GetPrimAtPath(f"/World/{name}/glass_body_mesh/glass_body_mesh_001")
        if g.IsValid() and g.GetTypeName() == "Mesh":
            UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] {g.GetPath()} doubleSided")
        else:
            print(f"[mat] {name} glass mesh not found for doubleSided, skip")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：test_tube.usd 自带玻璃 opacity 0.35 → 0.12（更透明、反光
    更弱，内部气泡/液柱看得更清，用户 2026-08-16）+ 补 ior 1.5 真玻璃 + doubleSided。
    **roughness 0.05 → 0.25**（原极光滑：曲面上 CylinderLight 12000 + DomeLight 2000 的
    锐利竖向高光带正好盖住管底沉淀，用户 2026-08-19"反光太强看不清沉淀"→ 柔化反光；
    op 0.12 透明保持，仍看得清内部）。遍历 /World/TestTube 下 mesh，取 material:binding
    覆写 shader（同瓶玻璃修法）。"""
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
    add_color_liquid(stage)   # 候选色变色液柱（滴加酸后液体变色，初始全隐藏）
    add_bubbles(stage)
    add_dropper_drops(stage)
    add_caps(stage)          # 翻放瓶盖（2026-08-17 用户要求，闭口朝下开口朝上）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_stoppers(st2)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, x=-10.0)
    fix_env_light(st2)
    relocate_absolute_textures(st2)
    fix_bottle_materials(st2)   # 样品瓶玻璃透明化；碱瓶塑料不碰（不透明试剂瓶，liquid 保留）
    fix_dropper_materials(st2)  # 滴管玻璃透明化（不透明会遮住管内液柱）
    fix_tube_material(st2)      # 试管玻璃透明化（op 0.35→0.12，内部气泡/液柱看得更清）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
