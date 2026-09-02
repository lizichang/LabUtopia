# -*- coding: utf-8 -*-
"""生成 b2_alcohol_heat_liquid.usd —— B2 沸点测定（酒精灯加热试管液体）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材（铁架台新钩版 / 酒精灯 / 石棉网 / 试管 / 试管夹 / 试管架 / 滴管 / 温度计
  / 玻璃皿+沸石 / 待测液体样品瓶 / 火柴——阶段A 机械臂操作升级新增）
- 布局 = 用户 b2_tmp.usd 相对位置（2026-08-25 改版：整组绕 Z 旋 180° 并平移），锚：铁架台铁柱在 (STAND_X, STAND_Y)，
  垂直堆叠中心线 x=铁架台x−0.100（堆叠在铁柱 −X 侧，R180 后环/钩支臂指向 −X）：
      酒精灯(桌面 z0.80) → 铁环(z0.930-0.938 托石棉网) → 石棉网(z0.939)
      → 试管(底 z0.9406 坐网上) → 试管夹(夹管上部 z1.044-1.073) → 钩(z1.236-1.2473，挂温度计)
- 2026-08-27 用户：铁架台上整套（铁环+挂钩+石棉网+试管+试管夹）上移 2cm（RING_RAISE）——
  酒精灯当前是内焰加热不是外焰加热，先动几何看效果，动作/火焰后续再改。酒精灯+火焰不动。
- 2026-08-27 二改：温度计玻璃管（杆/泡/毛细柱/白底/刻度）绕竖轴旋 -90°（y→x，Rz(-90)，
  刻度面从 +X 转到 -Y 朝 camera_2 特写，白底转到 -X），挂环方向不变；火焰底座从灯芯顶
  0.9005 下移到灯芯根部 0.891（holder 顶/灯芯露出点，包裹外露灯芯）——机械臂动作未改。
- 台面前区 (0.50,0.35) 放试管架：滴管插左孔（中排，底面落孔底 z0.806）、温度计插右后排孔
  (0.521,0.468)（2026-08-25 用户：右中排温度计顶高1.084 挡机械臂下探滴管 → 移 y 值最大=最远孔）
- 去资产自带 env_light 残留（重复 DomeLight）；灯帽从灯顶挪到桌边(y-0.467)；
  火焰迁到 /World 顶层（灯下引用子 prim 在 RTX 不渲染，2026-08-27 用户「火柴点燃看不到火焰」）

用法：python scripts/gen_b2_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b2_alcohol_heat_liquid")
OUT = os.path.join(SCENE_DIR, "b2_alcohol_heat_liquid.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# 2026-08-27 用户：铁架台上整套上移 2cm（酒精灯内焰加热→想改外焰加热，先动几何看效果）。
# 作用于铁环/挂钩（子 prim translate）+ 石棉网/试管/试管夹（EQUIP z）。酒精灯+火焰不动。
RING_RAISE = 0.02
# 2026-08-27 用户：「铁架台上面的挂钩能不能再往上移1厘米…这样子温度计能挂的高一点」。
# 挂钩在 RING_RAISE 之上再 +1cm（铁环/网/管/夹不动）；钩卡箍顶 1.2473→1.2573 逼近柱顶
# 1.26 → 铁柱加高 POLE_RAISE（柱顶 1.27，钩下留 1.3cm 柱身）。
HOOK_RAISE_EXTRA = 0.01    # 挂钩额外 +1cm（仅挂钩）
POLE_RAISE = 0.01          # 铁柱 Mesh 加高（顶 0.460→0.470）
# 锚：铁架台铁柱世界位置。b2_tmp 2026-08-25 用户整组绕 Z 旋 180° 并平移：
# 铁柱到 (0.6286, 0.0029)，堆叠中心线在铁柱 −X 侧 10cm → x=0.5286（环心/试管/石棉网/酒精灯）。
STAND_X, STAND_Y = 0.6286, 0.0029
TUBE_X = STAND_X - 0.100
TUBE_Y = STAND_Y
GAUZE_Z = TABLE_TOP + 0.1194 + RING_RAISE        # 石棉网中心（坐铁环上，环顶 0.918+RING_RAISE）
TUBE_BOTTOM_Z = TABLE_TOP + 0.1206 + RING_RAISE  # 试管底（坐石棉网上；b2_tmp translate z +RING_RAISE）
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.1533  # 试管口（底 0.9406 + 高 0.1533 = 1.0939）
# 试管夹：b2_tmp 里相对铁柱偏移 (0.0505,-0.0209,0.2384)，整组 R180 后 → (−0.0505,+0.0209,0.2384)。
# asset test_tube_clamp.usd 已内置 cm→m 换算，故 scene 只需平移（R180 由场景 xform op [T,R] 提供）。
CLAMP_T = (STAND_X - 0.0505, STAND_Y + 0.0209, TABLE_TOP + 0.2384 + RING_RAISE)
# 试管架：asset 世界 bbox z[-0.0965,0.0205]，min z=-0.0965 → tz=None 贴台面 = 架原点 z=0.80+0.0965=0.8965。
# 孔心相对架原点（skill 坑33 校准 + 2026-08-25 pxr 顶板 mesh 圆拟合）：顶板 2列×7行 Ø22.4 孔格。
# 列 x=±0.019；行 y=+0.118(后排=最远)/+0.079/+0.039/0.000(中排)/-0.040/-0.080/-0.119(前排)。
# 底层板顶(孔底) z=-0.0905 → 世界 0.806。滴管(Ø8) 插左列中排，温度计(Ø8) 插右列后排（最远孔）。
RACK_X, RACK_Y = 0.50, 0.35
RACK_Z = TABLE_TOP + 0.0965          # 0.8965 架原点
HOLE_Z = RACK_Z - 0.0905             # 0.806 孔底（底层板顶）

# 阶段A 新器材（2026-08-25 用户：沸石放玻璃皿上；初始坐标供目检微调）
# 玻璃皿（表面皿 Ø60×6.6mm，asset min z=0.0001 → 贴台）放台面前区；沸石粒叠皿上（显式 tz）；
# 待测液体样品瓶（Ø36×79 含塞）在皿下方；火柴躺灯旁（火柴头 = asset +X 端，头朝灯芯）。
DISH_X, DISH_Y = 0.40, 0.28
DISH_TOP_Z = TABLE_TOP + 0.0066      # 皿顶（皿底贴台面 0.80）
ZEO_T = DISH_TOP_Z                   # 沸石底(新资产底 z=0)贴皿顶 → 世界 0.8066
ZEO1_X, ZEO2_X = 0.39, 0.41          # 两颗沸石并排（±1cm 沿 x，皿 Ø60 半径 0.03 内，2026-08-27 用户「放两个沸石」）
BOTTLE_X, BOTTLE_Y = 0.40, 0.15
MATCH_X, MATCH_Y = 0.40, -0.06       # 火柴头 +X 端朝灯芯方向 (0.5286,0.0029)
MATCH_T = 0.813                      # 火柴原点 z：抬高 12mm 让手指离桌（避免 collider 扎进桌面卡爪，参考 flametest）

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面（资产底座 min z -> 0.80）；
# rot180=True → 加热堆叠整组绕 Z 旋 180°（xform op 顺序 [T,R]，pxr 净效果=绕局部原点旋转再平移）
EQUIP = [
    ("IronStand", "iron_stand.usd", (STAND_X, STAND_Y, None), None, True),
    ("AlcoholLamp", "alcohol_lamp.usd", (TUBE_X, TUBE_Y, None), None, True),
    ("AsbestosGauze", "asbestos_gauze.usd", (TUBE_X, TUBE_Y, GAUZE_Z), None, True),
    ("TestTube", "test_tube.usd", (TUBE_X, TUBE_Y, TUBE_BOTTOM_Z), None, True),
    ("TestTubeClamp", "test_tube_clamp.usd", CLAMP_T, None, True),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, False),
    # 滴管插左列中排、温度计插右列后排（y 值最大=最远孔，2026-08-25 用户：右中排温度计
    # 离滴管仅3.8cm 顶高1.084 挡机械臂下探滴管 → 移后排 (0.521,0.468)）；温度计 min z=-0.002 → +0.002 补偿让泡底落 0.806
    ("Dropper", "dropper.usd", (RACK_X - 0.019, RACK_Y - 0.0004, HOLE_Z), None, False),
    ("Thermometer", "thermometer.usd", (RACK_X + 0.021, RACK_Y + 0.118, HOLE_Z + 0.002), None, False),
    # 阶段A 新增（2026-08-25）：玻璃皿 + 沸石（显式 tz 叠皿上）+ 待测液体样品瓶 + 火柴（头朝灯芯）
    ("SurfaceDish", "sample_dish.usd", (DISH_X, DISH_Y, None), None, False),
    ("Zeolite", "zeolite.usd", (ZEO1_X, DISH_Y, ZEO_T), None, False),
    ("Zeolite2", "zeolite.usd", (ZEO2_X, DISH_Y, ZEO_T), None, False),
    ("SampleBottle", "sample_bottle.usd", (BOTTLE_X, BOTTLE_Y, None), None, False),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, MATCH_T), None, False),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


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


# ---- B2 效果 prim（内建，task 动画驱动）----
# 试管内液体（蓝半透明水柱，可见）、沸腾气泡（球组，初始隐藏）。
# 几何按 TestTube 世界 bbox 推导：管 (TUBE_X, TUBE_Y) 底 z0.9206 顶 z1.0739，Ø19.2 内 Ø~17。
LIQUID_TOP = TUBE_BOTTOM_Z + 0.052     # 液面高 5.2cm（约 1/3 管）
LIQUID_R = 0.008                       # 水柱半径 < 管内径 ~0.0085
COLOR_R = LIQUID_R - 0.0004            # 变色柱半径（d3l 同款：略小于液柱防 z-fight，叠在原液上染下去）
BUBBLE_R = 0.0025                      # 气泡半径（Ø5mm；正常值，2026-08-29 从调试 Ø12mm 回退）
# 气泡基准位（2026-08-28 照搬 d3l 真实感改造「中等档」）：40 颗离散小泡，中心盘状区 30 +
# 近管壁环 10（模拟壁面成核），固定种子可复现。上升动画由 task 动态池驱动（连续生成 +
# 速度差异 + 蛇形摆动 + 到液面破灭复用），z 全写管底上方 0.012（task 每帧覆盖 z）。
# 不变量：len(BUBBLE_BASE) == task.N_BUBBLES(40)，verify() 断言。
def _gen_bubbles(n_center=30, n_wall=10, seed=42):
    rng = random.Random(seed)
    out = []
    for _ in range(n_center):
        r = 0.0030 * math.sqrt(rng.random())          # 均匀圆盘（sqrt 面积均匀）
        a = 2.0 * math.pi * rng.random()
        out.append((TUBE_X + r * math.cos(a), TUBE_Y + r * math.sin(a), TUBE_BOTTOM_Z + 0.012))
    for _ in range(n_wall):
        r = 0.0045 + 0.0010 * rng.random()            # 0.0045~0.0055 近壁一圈
        a = 2.0 * math.pi * rng.random()
        out.append((TUBE_X + r * math.cos(a), TUBE_Y + r * math.sin(a), TUBE_BOTTOM_Z + 0.012))
    return out

BUBBLE_BASE = _gen_bubbles()

# 2026-08-27 用户「整体去掉蒸汽的显示」→ 蒸汽两段式（steam_inner/steam_plume）已删，
# 沸腾只留气泡动画（task 驱动）。


# —— 阶段B 滴加：液体材质配方（同 d3l：水透明 + ior 水折射；滴落液滴更亮更不透）——
# DropperFill（滴管尖内固定液柱）已删：2026-08-25 用户「固定竖直液柱很奇怪 + 移动时
# 浅色轨迹」→ 参考 d2l 无液柱，滴管空管移动、只在挤胶头瞬间 DropperDrop 成串坠落。
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)   # 样品瓶内半瓶液 / 试管内水柱
DROP = dict(color=(0.35, 0.75, 1.0), opacity=0.90, roughness=0.05, ior=1.33)    # 挤胶头滴落的液滴

# 2026-08-28 用户「加两个参数一个加热前液体颜色，一个是加热后液体颜色。可以参考d3l」：
# 候选色液柱配方（d3l 同款：近黑 diffuse + 单通道主导 emissive，防被 CylinderLight 12000 洗白
# 成透明，见 d2l-liquid-color-recipe）。task 按 cfg.before_color 显主液柱一根、cfg.liquid_color
# 显变色柱一根（d3l 字段名同款）。
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
# 瓶/管/滴管玻璃配方：assets 自带 bottle_mat op0.8/rough0.33 磨砂玻璃，隔它看不清液体 → 真玻璃
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)

# 样品瓶内液体（(BOTTLE_X,BOTTLE_Y) 台面 0.80..液面 0.840 = 半瓶；Ø36 内 Ø~34）
BOTTLE_LIQ_R = 0.014
BOTTLE_LIQ_H = 0.040            # 0.80..0.84
BOTTLE_LIQ_CZ = TABLE_TOP + BOTTLE_LIQ_H / 2      # 0.820
# 挤胶头滴落串：一次挤 DROPS_PER_GROUP 滴连续坠落（滴管内吸满液，一挤该是一串滴不是
# 一滴——d3l 用户 2026-08-14）；DropperDrop 父 Xform + Drop_0.._N 球，task 动画驱动
DROPS_PER_GROUP = 4
DROP_BALL_R = 0.003
DROP_HOME = (TUBE_X, TUBE_Y, TUBE_BOTTOM_Z + 0.020)


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False,
                 emissive=None):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided，
    否则从外看透过液体时后壁不渲染会像空容器。emissive：自发光（高亮小件用）。"""
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
    """建一个材质绑定到多个 prim（气泡组共用）。emissive：自发光（高亮小件用）。"""
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


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 Xform + Drop_0.._N 亮蓝小球（r=0.003）。
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。父 + 每个球都初始
    MakeInvisible——球若只靠父隐藏，_on_drop reveal 父后 delay 中的球会继承可见、
    停在 home 位闪现成"拖影"（2026-08-25 用户反馈）。上场由 task 逐球 set visible。"""
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


def add_b2_effects(stage):
    """内建效果 prim：试管内水柱（初始隐藏 h0，滴加后逐滴生长）+ 样品瓶内液面（可见）
    + 气泡组（初始隐藏）。滴管尖内固定液柱 DropperFill 已删（2026-08-25 用户）。"""
    # 水柱：试管内蓝色半透明柱。阶段B 初始隐藏、height 0——task._grow_tube_level 滴加时
    # 逐滴长高（底面贴管底 TUBE_BOTTOM_Z，上限 DROP_LEVEL_MAX=0.060 远低于管口）
    liq = UsdGeom.Cylinder.Define(stage, "/World/TestTubeLiquid")
    liq.CreateRadiusAttr(LIQUID_R)
    liq.CreateHeightAttr(0.0)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, TUBE_BOTTOM_Z))
    add_material(stage, liq.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    UsdGeom.Imageable(liq).MakeInvisible()
    print(f"[effect] TestTubeLiquid hidden h0 (grow by dropper drip)")

    # 样品瓶内液面：蓝色半透明柱（瓶底 0.80..液面 0.840 = 半瓶），可见（吸液源）
    sl = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    sl.CreateRadiusAttr(BOTTLE_LIQ_R)
    sl.CreateHeightAttr(BOTTLE_LIQ_H)
    sl.CreateAxisAttr("Z")
    sl.AddTranslateOp().Set(Gf.Vec3d(BOTTLE_X, BOTTLE_Y, BOTTLE_LIQ_CZ))
    add_material(stage, sl.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    print(f"[effect] SampleLiquid visible (bottle {BOTTLE_LIQ_H:.3f}m to top 0.840)")

    # 气泡组：8 球在液体内，初始隐藏，task 沸腾时 reveal + 上升。
    # 2026-08-27 加大提亮：原 r1.5mm 近白 op0.7 在 512px 蓝液里几乎不可见（用户「液体里
    # 也看不到气泡」）→ r2.5mm + 亮白 diffuse + 弱自发光（对比蓝液，沸石是成核点）。
    UsdGeom.Xform.Define(stage, "/World/TestTubeBubbles")
    bub_prims = []
    for i, (x, y, z) in enumerate(BUBBLE_BASE):
        sp = UsdGeom.Sphere.Define(stage, f"/World/TestTubeBubbles/bubble_{i}")
        sp.CreateRadiusAttr(BUBBLE_R)
        sp.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        UsdGeom.Imageable(sp).MakeInvisible()
        bub_prims.append(sp.GetPrim())
    add_shared_material(stage, "/World/TestTubeBubbles/bubble_mat",
                        (0.72, 0.85, 1.0), 1.0, bub_prims, roughness=0.3,
                        emissive=(0.7, 1.0, 1.8))
    print(f"[effect] {len(BUBBLE_BASE)} bubbles hidden")


def add_color_liquid(stage):
    """加热前/后候选色液柱（2026-08-28 用户参考 d3l）：/World/TubeLiquidBefore_<色>（加热前色 =
    主液柱，滴加逐滴长高）+ /World/TubeLiquidColor_<色>（加热后色 = d3l 同款变色柱，顶贴液面
    向下扩散，细一圈 COLOR_R 叠在主液上）。初始全隐藏、height 0；task 按 cfg.before_color 显
    主液柱一根（"clear" 回退 TestTubeLiquid 水柱）、cfg.liquid_color 显变色柱一根（沸腾时
    _step_color_transition 顶贴液面向下扩散染过去）。"""
    for prefix, r in (("TubeLiquidBefore", LIQUID_R), ("TubeLiquidColor", COLOR_R)):
        for name, m in LIQUID_COLORS.items():
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{prefix}_{name}")
            geom.CreateRadiusAttr(r)
            geom.CreateHeightAttr(0.0)
            geom.CreateAxisAttr("Z")
            # 底面 = 管底 TUBE_BOTTOM_Z（task 逐滴/逐帧把 center 设到 TUBE_BOTTOM_Z + h/2）
            geom.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, TUBE_BOTTOM_Z))
            translucent = m.get("opacity", 1.0) < 1.0
            add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                         roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                         emissive=m.get("emissive"), double_sided=translucent)
            UsdGeom.Imageable(geom).MakeInvisible()
            print(f"[effect] {prefix}_{name} hidden (r={r})")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃试管/酒精灯在无环境反射下照不亮。
    贴图路径先用相对 ./textures/，烘平后由 fix_env_light() 在场景层重新指向。"""
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


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效），
    烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去 4 件器材资产自带的 flametest 残留 DomeLight（/root/env_light），会与场景
    env_light 双灯、且近黑贴图压暗环境。遍历每个器材 prim 删 DomeLight / env_light。"""
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
    """灯帽从灯顶挪到桌边（闭口朝下贴台面）静止位 CAP_REST=(0.42,-0.01,0.8155)。

    2026-08-28 二次可达性修复：帽是灯子 prim，随灯滑移。旧位 ty 0.103 移灯后帽中心
    (0.5286,-0.20) 在机械臂底座(y=-0.08)后方 12cm 低 z，Lula IK 无解 → 下去夹卡死。
    新位让帽静止在灯前 -X 侧桌面 (0.42,-0.01)（3D 距底座 0.44m、y 在底座前 3cm，同
    火柴夹点可达性，pxr 实测与灯体/火柴无碰撞）。移灯期间 task
    逐帧 _set_cap_world(CAP_REST) 把帽钉在静止位（不随灯滑），盖帽动作在此夹帽。
    盖灭时帽中心 0.8917 盖严实（同资产原始帽位，2026-08-28 六改；CAP_BURNER 见 constants）。
    2026-08-28 九改（机械臂夹帽擦石棉网）：石棉网 x[0.4726,0.5853] 左边缘 0.4726，旧
    CAP_REST x=0.45 → 帽右边缘 0.4686 离石棉网仅 4mm，机械臂手腕擦进石棉网。往 -X 移
    3cm 到 x=0.42（手腕右边缘 ~0.45 避开；往 -Y 移碰火柴 y=-0.06 不可行，见 constants）。

    资产 cap xform = [Translate(0,0,0) rotateX90 scale0.01]，mesh 在局部 z[0.076,0.107]，
    故 translate z=-0.076 让帽底(z=0.076)落回台面 0.80。
    酒精灯整组已 R180，cap 局部 y 会被旋 180° 取反。换算（pxr 实测）：cx=灯x−tx、
    cy=灯y−ty、cz=灯z+tz+CAP_CENTER_DZ(0.0915) → tx=0.5286−0.42=0.1086、
    ty=0.0029−(−0.01)=0.0129、tz=0.8155−0.80−0.0915=−0.076。"""
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


def raise_stand_ring_hook(st2):
    """铁环/挂钩上移 RING_RAISE；挂钩再额外 +HOOK_RAISE_EXTRA（2026-08-27 用户：整套上移
    2cm 改外焰加热 → 挂钩再往上移 1cm 让温度计挂更高。高于柱顶则柱子加高，见 raise_stand_pole）。

    iron_stand.usd 里 ring/hook 是无本地 transform 的子 prim（位置烘焙在 mesh 里），
    烘平后路径 /World/IronStand/root/{ring,hook}。给它们加 translate(0,0,+RING_RAISE)
    （hook 再 +HOOK_RAISE_EXTRA）：R180 只影响 x/y，局部 +Z 即世界 +Z。石棉网/试管/试管夹
    由 EQUIP 常量+RING_RAISE 上移（不随挂钩额外抬升）。
    """
    raises = {"ring": RING_RAISE, "hook": RING_RAISE + HOOK_RAISE_EXTRA}
    for name, dz in raises.items():
        p = st2.GetPrimAtPath(f"/World/IronStand/root/{name}")
        if not p.IsValid():
            print(f"[raise] /World/IronStand/root/{name} MISSING")
            continue
        UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(0, 0, dz))
        r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(p).ComputeAlignedRange()
        print(f"[raise] {name} +{dz:.3f} world z -> [{r.GetMin()[2]:+.4f},{r.GetMax()[2]:+.4f}]")


def raise_stand_pole(st2):
    """铁柱加高 POLE_RAISE（2026-08-27 用户：挂钩上移后若高于柱顶就把柱子调高）。

    iron_stand.usd 的柱是 Mesh（/root/root/pole/pole_002，points z[0.010,0.460]，父 Xform
    无 op），不是圆柱 prim → 不能改高度 attr。在父 Xform /World/IronStand/root/pole 加
    [T,S] op 绕柱底锚 0.010 缩放：底不动（贴底座顶 0.81）、顶 0.460→0.460+POLE_RAISE。
    半径不变（scale 只在 Z），铁环/挂钩卡箍仍贴合柱身。op 序 [T,S] → M=T·S，p'=S(p) 再 T：
    p' = s·p + t，取 t = 0.010·(1−s) 即底锚不动。
    """
    p = st2.GetPrimAtPath("/World/IronStand/root/pole")
    if not p.IsValid():
        print("[raise] /World/IronStand/root/pole MISSING")
        return
    bottom, top = 0.010, 0.460            # 柱 mesh 点 z 范围（asset 局部）
    s = (top + POLE_RAISE - bottom) / (top - bottom)
    t = bottom * (1 - s)
    xf = UsdGeom.Xformable(p)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, t))
    xf.AddScaleOp().Set(Gf.Vec3d(1, 1, s))
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(p).ComputeAlignedRange()
    print(f"[raise] pole scale {s:.5f} tz {t:+.6f} world z -> [{r.GetMin()[2]:+.4f},{r.GetMax()[2]:+.4f}]")


def rotate_thermo_stem(st2):
    """温度计玻璃管绕竖轴旋 -90°（2026-08-27 用户：上面挂环方向不变，下面整根玻璃管以及
    所有东西以 z 为轴 y 朝 x 方向转 90°）。camera_2 特写从 -Y 朝 +Y 看温度计，旋后刻度面
    从 +X 转到 -Y（朝向镜头）、白底从 -Y 转到 -X。

    asset 里 stem/bulb/毛细柱/白底/刻度各 prim 局部原点都在温度计轴线上（居中），给每个
    杆件 prim 加 RotateXYZ(0,0,-90) = 绕自身竖轴转（Rz(-90) 使 +Y→+X = 用户说的"y 朝 x"）。
    挂环 hanging_ring 不加 → 方向不变。杆/泡/液柱轴对（转了看不出来），实际只有白底
    (→-X) 和刻度(→-Y 朝镜头) 换边。机械臂抓杆/插管/挂环坐标不受影响（杆轴对仍居中，
    抓点是位置无关轴向）。
    """
    rot = Gf.Vec3f(0, 0, -90)
    for name in ("stem", "bulb", "bulb_liquid", "capillary_liquid", "white_backing", "scale"):
        p = st2.GetPrimAtPath(f"/World/Thermometer/Thermometer/{name}")
        if not p.IsValid():
            print(f"[thermo] {name} MISSING")
            continue
        UsdGeom.Xformable(p).AddRotateXYZOp().Set(rot)
        print(f"[thermo] {name} RotateXYZ(0,0,-90)")
    # 挂环必须没有 rotate op（方向不变）
    ring = st2.GetPrimAtPath("/World/Thermometer/Thermometer/hanging_ring")
    for op in UsdGeom.Xformable(ring).GetOrderedXformOps():
        assert op.GetOpType() != UsdGeom.XformOp.TypeRotateXYZ, \
            "hanging_ring got a rotation — ring direction must stay"
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(
        st2.GetPrimAtPath("/World/Thermometer")).ComputeAlignedRange()
    print(f"[thermo] thermometer bbox after stem rotation "
          f"x[{r.GetMin()[0]:+.4f},{r.GetMax()[0]:+.4f}] y[{r.GetMin()[1]:+.4f},{r.GetMax()[1]:+.4f}]")


# 酒精灯火焰迁到 /World 顶层（2026-08-27 用户「火柴点燃看不到火焰」）。
# 根因同 flametest（diag_flame 系列）：/World/AlcoholLamp 是引用型 over Xform，其下
# over 子 prim（flame_outer/flame_inner）在 RTX usdrt population 里不渲染——无论位置/
# 大小/材质都完全不可见；而顶层新增的 Cone 正常渲染。修复：删灯下旧火焰锥+材质，在
# /World 顶层用 Cone.Define 重建——几何烘焙进 height/radius、translate 用世界坐标
# （底=灯芯顶 0.9005，顶端 ~0.936）、近黑 diffuse + 强 HDR 单通道 emissive（flametest
# 已验证良方）、默认 visible（不设 invisible——prim 在 population 时 invisible 会阻止
# RTX 材质初始化，之后翻 visible 仍渲染默认灰；熄灭由任务 reset() 的 _set_visible(False)
# 负责，点着时再翻 visible）。
# 2026-08-27 用户一改：火焰底座从灯芯顶(0.9005)下移到灯芯根部 = holder 顶（世界 0.891，
# 灯芯从 holder 露出的位置），火焰底在根部包裹外露灯芯。
# 2026-08-27 二改（用户：「要加长追上面刚好能碰到石棉网，然后火焰不应该是一个锥形，应该
# 像一个水滴底部是圆的」）→ 火焰加长到 apex 刚好碰石棉网底，每焰改水滴形（底半球 Sphere
# 圆底 + 上部 Cone 收尖），不再是尖锥。
FLAME_BASE_Z = TABLE_TOP + 0.091            # 灯芯根部世界 z（火焰底 = holder 顶/灯芯露出点）
FLAME_APEX_Z = GAUZE_Z - 0.00105            # 石棉网底 0.9384（火焰尖刚好碰到；derive 自 GAUZE_Z）
FLAME_OUTER_R = 0.009                       # 外焰肚半径（水滴最宽处 = 底半球半径）
FLAME_INNER_R = 0.005                       # 内焰（焰心）肚半径
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022   # 内焰焰心 apex（0.913，在外焰内）

def add_droplet_flame(st2, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere（底部圆） + 上部 Cone（收尖），一组两 prim，绕 Z 轴。
    球心在 z_b+r（球底 z_b 贴根部）、cone 底在球心处（=水滴最宽肚）收尖到 z_a。
    材质近黑 diffuse + HDR 单通道 emissive（flametest 良方），默认可见（勿 invisible，
    熄灭由 task reset() _set_visible(False) 负责）。"""
    zc = z_b + r                       # 球心 = 水滴最宽处
    sph = UsdGeom.Sphere.Define(st2, f"/World/{name}_sphere")
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, zc))
    h = z_a - zc
    cone = UsdGeom.Cone.Define(st2, f"/World/{name}")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    cone.CreateAxisAttr("Z")          # 锥尖 +Z 朝上（焰舌向石棉网）
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, zc + h / 2))
    for prim in (sph, cone):
        pname = prim.GetPath().name   # 球 = flame_<x>_sphere、锥 = flame_<x>
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
    """酒精灯火焰：删灯下引用子 prim，在 /World 顶层重建（flametest 已验证良方）。

    2026-08-27 二改（用户：「要加长追上面刚好能碰到石棉网，然后火焰不应该是一个锥形，
    应该像一个水滴底部是圆的」）→ 每焰水滴形 = 底半球 Sphere(圆底) + 上部 Cone(收尖)：
    外焰 flame_outer 底 0.891(灯芯根部) apex 0.9384(石棉网底) 刚好碰，内焰 flame_inner
    焰心更小 apex 0.913。材质近黑 diffuse + HDR 单通道 emissive，默认可见。
    2026-08-27 焰色（用户：「外焰偏蓝色，内焰偏黄色」）= 外焰 B 主导淡蓝、内焰 R 主导黄。
    task._flame_paths() 返回两组的 4 个路径统一控制熄/亮。
    """
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                      (0.35, 0.55, 2.40))                          # 外焰偏蓝（B 主导淡蓝）
    add_droplet_flame(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                      (2.80, 0.55, 0.20))                          # 内焰偏黄（R 主导黄）
    print(f"[lamp] flames droplet: outer base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"(touches gauze bottom {GAUZE_Z - 0.00105:.4f}), inner apex {FLAME_INNER_APEX_Z:.4f}")


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数。烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply（会告警），故直接用 material:binding relationship
    取材质路径再找 shader（同 d3l gen）。"""
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


def remove_stoppers(st2):
    """去样品瓶塞：实验已开瓶，删 SampleBottle 自带 stopper + stopper_mat
    （覆盖在瓶口上 0.068..0.079，删后瓶口 rim=0.070 → 世界 0.870）。

    注意：stage.Export 烘平会把引用资产的 root 包装 Xform 合并进引用 prim
    （不是 /World/SampleBottle/root/stopper，而是 /World/SampleBottle/stopper），
    故用遍历匹配（同 d3l gen）。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print("[clean] /World/SampleBottle not found, skip")
        return
    paths = [pp.GetPath() for pp in Usd.PrimRange(p)
             if pp.GetName() in ("stopper", "stopper_mat")]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed {path}")
    if not paths:
        print("[clean] /World/SampleBottle has no stopper/stopper_mat")


def fix_bottle_materials(st2):
    """样品瓶玻璃透明化（磨砂 op0.8 → 真玻璃 op0.25 + ior 1.5 + doubleSided），
    瓶内 SampleLiquid 液面才透得出来。若自带 1mm 液面薄盘（"liquid" mesh）隐藏
    （改用内建 SampleLiquid 体积表现）。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print(f"[mat] /World/SampleBottle not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if c.GetName() == "liquid":
            UsdGeom.Imageable(c).MakeInvisible()
            print(f"[mat] hid {c.GetPath()} (1mm liquid disc, replaced by SampleLiquid)")
        else:
            if override_bound_shader(st2, c, GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 是 opacity=1.0 不透明光面（把吸起的液体
    遮住，用户反馈"液柱不明显"根因）。改成真玻璃 op 0.25（同瓶玻璃配方），滴管才能
    透出吸液/滴落效果；胶头（rubber_001）保持不透明。"""
    mat = st2.GetPrimAtPath("/World/Dropper/_materials/glass_001")
    if not mat.IsValid():
        print(f"[mat] Dropper glass_001 not found, skip")
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
    # 玻璃 mesh 双面渲染（透过玻璃看后壁，不设会漏空）
    g = st2.GetPrimAtPath("/World/Dropper/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")
    else:
        print(f"[mat] Dropper glass mesh not found for doubleSided, skip")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光：test_tube.usd 自带玻璃 opacity 0.35 → 0.12（更透明、反光
    更弱，内部滴加液柱看得更清，用户 2026-08-16）+ 补 ior 1.5 真玻璃 + doubleSided +
    **roughness 0.05 → 0.25**（柔化 CylinderLight 12000 的锐利竖向高光带，同 d3l）。"""
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


def fix_thermo_material(st2):
    """温度计红液去反光：RedLiquid 材质是亮红 diffuse(0.85) + 强镜面 specular(0.8) +
    roughness 0.25 光面 —— CylinderLight 12000 下反光成亮斑，camera_2 看不清红色读数
    （用户 2026-08-29）。照 d2l/flametest 液色配方改：近黑 diffuse + 红单通道主导
    emissive + 糙面（roughness 1.0）+ 去镜面 specular。红柱自发光透过玻璃管清晰可见、
    无高光斑。capillary_liquid/bulb_liquid 共用此材质，一改两柱同修。"""
    mat = st2.GetPrimAtPath("/World/Thermometer/Looks/RedLiquid")
    if not mat.IsValid():
        print("[mat] Thermometer RedLiquid not found, skip")
        return
    specs = (  # (输入名, 类型, 值)
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
        print(f"[mat] Thermometer RedLiquid -> matte+emissive red ({sh.GetPath()})")


def verify(st2):
    """自检：打印各器材世界 bbox，断言垂直堆叠关系：
    铁架台底座贴台面、石棉网坐铁环上（0.4mm 间隙）、试管底坐石棉网上（≤1mm）、
    试管夹夹住试管上部、灯芯顶低于石棉网底（火焰区留白）、钩低于铁柱顶、
    试管架贴台面、滴管/温度计插进架孔（底面落孔底、x 对准孔心）、
    液体在试管内（不超管口/不低于管底）、气泡组齐全。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "AsbestosGauze", "TestTube", "TestTubeClamp",
             "TestTubeRack", "Dropper", "Thermometer",
             "SurfaceDish", "Zeolite", "Zeolite2", "SampleBottle", "Match",
             "SampleLiquid", "DropperDrop"]
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
    # 铁架台：底座贴台面，钩顶 < 铁柱顶
    smn, smx = boxes["IronStand"]
    assert abs(smn[2] - TABLE_TOP) < 0.002, f"IronStand base z {smn[2]} != table {TABLE_TOP}"
    # 石棉网坐铁环上：网底 > 环顶(0.918+RING_RAISE)，网底 − 环顶 ≤ 2mm
    gmn, gmx = boxes["AsbestosGauze"]
    ring_top = TABLE_TOP + 0.118 + RING_RAISE
    assert gmn[2] >= ring_top - 0.002, f"gauze bottom {gmn[2]} below ring top {ring_top}"
    # 试管底坐石棉网上：管底 − 网顶 ≤ 1.5mm 且 ≥ 0
    tmn, tmx = boxes["TestTube"]
    assert 0 <= tmn[2] - gmx[2] <= 0.0015, f"tube bottom {tmn[2]} not on gauze top {gmx[2]}"
    # 试管夹夹住试管上部：夹子 z 落在管上部（>管底+5cm）、钳口抱管、支臂从管伸到铁柱（R180 后指向 +X）
    cmn, cmx = boxes["TestTubeClamp"]
    assert cmn[2] < tmx[2] - 0.01, f"clamp z top {cmx[2]} below tube upper {tmx[2]-0.01}"
    assert cmx[2] > tmn[2] + 0.05, f"clamp z bottom {cmn[2]} not on tube upper section"
    assert cmx[0] >= tmx[0] - 0.001, f"clamp x max {cmx[0]} not reaching tube right {tmx[0]}"
    assert cmx[0] >= STAND_X - 0.03, f"clamp arm not reaching pole: x max {cmx[0]}"
    assert cmn[0] < TUBE_X + 0.03, f"clamp jaw not wrapping tube: x min {cmn[0]}"
    # 酒精灯灯芯顶低于石棉网底（火焰区留白 ≥ 1.5cm）
    lmn, lmx = boxes["AlcoholLamp"]
    # 灯芯顶 = wick_tip 世界 z（资产内 0.1005 → 台面 0.80）
    wick_top = TABLE_TOP + 0.1005
    assert wick_top + 0.015 < gmn[2], f"flame gap {gmn[2] - wick_top:.3f} < 1.5cm"
    # 钩低于铁柱顶（铁柱顶 = 台面 0.80 + 0.46 + POLE_RAISE 0.01 = 1.27；柱 Mesh 加高后）
    assert smx[2] < TABLE_TOP + 0.470 + 0.002, f"IronStand top {smx[2]} exceeds pole top"
    # 挂钩上移 1cm 后：钩顶 = 台面 + 0.4273(钩 mesh 顶) + RING_RAISE + HOOK_RAISE_EXTRA = 1.2573，
    #   且低于柱顶 ≥1cm（柱子加高留白，卡箍不会悬在柱尖）
    hk = st2.GetPrimAtPath("/World/IronStand/root/hook")
    hr = bc.ComputeWorldBound(hk).ComputeAlignedRange()
    hook_top = TABLE_TOP + 0.4273 + RING_RAISE + HOOK_RAISE_EXTRA
    assert abs(hr.GetMax()[2] - hook_top) < 0.003, \
        f"hook top {hr.GetMax()[2]:+.4f} != {hook_top:+.4f}"
    assert hr.GetMax()[2] < smx[2] - 0.01, \
        f"hook top {hr.GetMax()[2]:+.4f} not below pole top {smx[2]:+.4f}"
    # 灯帽静止位 CAP_REST(0.42,-0.01)：灯前 -X 侧桌面，帽底贴台面、中心 0.8157
    #   （2026-08-28 九改：往 -X 移 3cm 避石棉网左边缘 0.4726，见 constants）
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    print(f"[verify] cap     min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    # 帽中心在 CAP_REST（±5mm 容差）；不在堆叠中心线下方
    cx, cy = 0.5 * (cmn[0] + cmx[0]), 0.5 * (cmn[1] + cmx[1])
    assert abs(cx - 0.42) < 0.005, f"cap center x {cx:+.4f} != 0.42"
    assert abs(cy + 0.01) < 0.005, f"cap center y {cy:+.4f} != -0.01"
    # 试管架：底座贴台面（0.80）
    rmn, rmx = boxes["TestTubeRack"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"rack bottom {rmn[2]} not on table"
    # 滴管：底面落孔底 HOLE_Z(0.806)，x 中心对准左孔心 (RACK_X-0.019)，y 中心对准中排 (RACK_Y-0.0004)
    dmn, dmx = boxes["Dropper"]
    assert abs(dmn[2] - HOLE_Z) < 0.002, f"dropper bottom {dmn[2]} != hole bottom {HOLE_Z}"
    assert abs((dmn[0] + dmx[0]) / 2 - (RACK_X - 0.019)) < 0.003, f"dropper x center {(dmn[0]+dmx[0])/2:.4f} not at left hole"
    assert abs((dmn[1] + dmx[1]) / 2 - (RACK_Y - 0.0004)) < 0.003, f"dropper y center {(dmn[1]+dmx[1])/2:.4f} not at center row"
    # 温度计：泡底落孔底（min z=-0.002 已 +0.002 补偿），x 中心对准右列孔心 (RACK_X+0.021)、
    # y 中心对准后排孔心 (RACK_Y+0.118)——2026-08-25 用户：右中排挡机械臂下探滴管，移 y 值最大=最远孔
    thn, thx = boxes["Thermometer"]
    assert abs(thn[2] - HOLE_Z) < 0.002, f"thermo bottom {thn[2]} != hole bottom {HOLE_Z}"
    assert abs((thn[0] + thx[0]) / 2 - (RACK_X + 0.021)) < 0.003, f"thermo x center {(thn[0]+thx[0])/2:.4f} not at right hole"
    assert abs((thn[1] + thx[1]) / 2 - (RACK_Y + 0.118)) < 0.003, f"thermo y center {(thn[1]+thx[1])/2:.4f} not at back row"
    # 2026-08-27 温度计玻璃管绕竖轴 -90°（刻度面转朝 -Y/camera_2，白底转 -X），挂环不转
    for nm in ("stem", "bulb", "bulb_liquid", "capillary_liquid", "white_backing", "scale"):
        sp = st2.GetPrimAtPath(f"/World/Thermometer/Thermometer/{nm}")
        assert sp.IsValid(), f"thermo {nm} missing"
        rots = [op.Get() for op in UsdGeom.Xformable(sp).GetOrderedXformOps()
                if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ]
        assert rots == [Gf.Vec3f(0, 0, -90)], f"thermo {nm} rotate {rots} != (0,0,-90)"
    ring_p = st2.GetPrimAtPath("/World/Thermometer/Thermometer/hanging_ring")
    assert ring_p.IsValid(), "thermo hanging_ring missing"
    ring_rots = [op for op in UsdGeom.Xformable(ring_p).GetOrderedXformOps()
                 if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ]
    assert not ring_rots, "thermo hanging_ring must have no rotation (ring direction unchanged)"
    # 滴管/温度计都在架顶板范围（x±0.043, y±0.143），确认孔位在板内
    assert dmx[0] < rmx[0] + 0.001 and thn[0] > rmn[0] - 0.001, "instruments outside rack plate x"
    # 效果 prim：试管内水柱初始隐藏 h0（阶段B 滴加后逐滴生长，上限远低于管口），气泡组存在
    lq = st2.GetPrimAtPath("/World/TestTubeLiquid")
    assert lq.IsValid(), "TestTubeLiquid missing"
    assert UsdGeom.Cylinder(lq).GetHeightAttr().Get() == 0.0, "TestTubeLiquid height should be 0"
    assert UsdGeom.Imageable(lq).ComputeVisibility() == "invisible", "TestTubeLiquid should be hidden"
    # 滴加上限 DROP_LEVEL_MAX=0.060 → 液顶 0.9806，远低于管口 1.0739（防溢出断言）
    assert TUBE_BOTTOM_Z + 0.060 < tmx[2] - 0.05, "dropped liquid cap too close to tube mouth"
    # 样品瓶内液面：可见、顶 ≤ 液面 0.840、x 对准瓶心
    sld = boxes.get("SampleLiquid")
    assert sld is not None, "SampleLiquid missing"
    assert sld[1][2] <= TABLE_TOP + 0.041, f"sample liquid top {sld[1][2]} above 0.840"
    assert abs((sld[0][0] + sld[1][0]) / 2 - BOTTLE_X) < 0.005, "SampleLiquid x off bottle"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/SampleLiquid")).ComputeVisibility() != "invisible", \
        "SampleLiquid should be visible"
    # 滴落球组：4 球、初始隐藏
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    assert dd.IsValid(), "DropperDrop missing"
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == DROPS_PER_GROUP, f"DropperDrop spheres {nd} != {DROPS_PER_GROUP}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible", "DropperDrop should be hidden"
    bub = st2.GetPrimAtPath("/World/TestTubeBubbles")
    assert bub.IsValid(), "bubbles group missing"
    nb = sum(1 for c in bub.GetChildren() if c.GetTypeName() == "Sphere")
    assert nb == len(BUBBLE_BASE), f"bubbles count {nb} != {len(BUBBLE_BASE)}"
    for i in range(nb):   # 2026-08-27 加大提亮：半径须为 BUBBLE_R
        bs = st2.GetPrimAtPath(f"/World/TestTubeBubbles/bubble_{i}")
        assert abs(UsdGeom.Sphere(bs).GetRadiusAttr().Get() - BUBBLE_R) < 0.0005, \
            f"bubble_{i} radius {UsdGeom.Sphere(bs).GetRadiusAttr().Get()} != {BUBBLE_R}"
    # 加热前/后候选色液柱：2×len(LIQUID_COLORS) 根，h0、隐藏、底面贴管底；
    # Before（主液柱）r=LIQUID_R、Color（变色柱）r=COLOR_R（d3l 同款细一圈）
    for prefix, r in (("TubeLiquidBefore", LIQUID_R), ("TubeLiquidColor", COLOR_R)):
        for name in LIQUID_COLORS:
            lp = st2.GetPrimAtPath(f"/World/{prefix}_{name}")
            assert lp.IsValid(), f"{prefix}_{name} missing"
            assert abs(UsdGeom.Cylinder(lp).GetRadiusAttr().Get() - r) < 1e-9, \
                f"{prefix}_{name} r != {r}"
            assert UsdGeom.Cylinder(lp).GetHeightAttr().Get() == 0.0, f"{prefix}_{name} height should be 0"
            assert UsdGeom.Imageable(lp).ComputeVisibility() == "invisible", \
                f"{prefix}_{name} should be hidden"
            tz = UsdGeom.Xformable(lp).GetOrderedXformOps()[0].Get()[2]
            assert abs(tz - TUBE_BOTTOM_Z) < 0.001, f"{prefix}_{name} base z {tz} != TUBE_BOTTOM_Z"
    print(f"[verify] LiquidColor OK: 2x{len(LIQUID_COLORS)} tubes "
          f"(Before r={LIQUID_R} + Color r={COLOR_R}) all hidden")
    # 火焰迁到 /World 顶层（灯下子 prim 已删；顶层 prim 默认可见，任务 reset 再熄）
    # 2026-08-27 二改：水滴形 = 每焰 /World/{name} Cone(收尖) + /World/{name}_sphere
    # Sphere(圆底)。底在灯芯根部 FLAME_BASE_Z(0.891)；外焰 apex FLAME_APEX_Z 须刚好碰
    # 石棉网底 gmn[2]（≤±0.004），内焰 apex 在外焰内。球心 z_b+r、cone 底同球心。
    for name, r, z_b, z_a in (("flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z),
                              ("flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z)):
        f = st2.GetPrimAtPath(f"/World/{name}")
        assert f.IsValid() and f.GetTypeName() == "Cone", f"{name} top-level cone missing"
        assert UsdGeom.Imageable(f).ComputeVisibility() != "invisible", f"{name} should be default visible"
        assert abs(UsdGeom.Cone(f).GetRadiusAttr().Get() - r) < 0.0005, f"{name} cone r wrong"
        h = z_a - (z_b + r)
        assert abs(UsdGeom.Cone(f).GetHeightAttr().Get() - h) < 0.0005, f"{name} cone height wrong"
        tz = UsdGeom.Xformable(f).GetOrderedXformOps()[0].Get()[2]
        assert abs(tz - ((z_b + r) + h / 2)) < 0.002, \
            f"{name} cone center z {tz} != {(z_b + r) + h / 2}"
        sph = st2.GetPrimAtPath(f"/World/{name}_sphere")
        assert sph.IsValid() and sph.GetTypeName() == "Sphere", f"{name}_sphere missing"
        assert UsdGeom.Imageable(sph).ComputeVisibility() != "invisible", f"{name}_sphere default visible"
        assert abs(UsdGeom.Sphere(sph).GetRadiusAttr().Get() - r) < 0.0005, f"{name}_sphere r wrong"
        sz = UsdGeom.Xformable(sph).GetOrderedXformOps()[0].Get()[2]
        assert abs(sz - (z_b + r)) < 0.002, \
            f"{name}_sphere center z {sz} != {z_b + r} (bottom should be {z_b})"
        # 焰色（2026-08-27 用户：外焰偏蓝、内焰偏黄）：外焰 B 主导、内焰 R 主导
        sh = UsdShade.Shader(st2.GetPrimAtPath(f"/World/{name}_mat/Shader"))
        c = sh.GetInput("emissiveColor").Get()
        assert c is not None, f"{name} emissive input missing"
        if name == "flame_outer":
            assert c[2] > c[0] and c[2] > 1.0, f"outer flame should be blue-dominant, got {tuple(c)}"
        else:
            assert c[0] > c[2] and c[0] > 1.0, f"inner flame should be yellow-dominant, got {tuple(c)}"
        if name == "flame_outer":
            assert abs(z_a - gmn[2]) < 0.004, \
                f"outer flame apex {z_a} not at gauze bottom {gmn[2]}"
    assert not st2.GetPrimAtPath("/World/AlcoholLamp/flame_outer").IsValid(), \
        "old lamp sub-prim flame still present"
    # 阶段A 新器材：玻璃皿贴台、两颗沸石底贴皿顶且并排（±1cm 沿 x）、样品瓶贴台、火柴抬高 12mm
    dsh = boxes["SurfaceDish"]
    assert abs(dsh[0][2] - TABLE_TOP) < 0.002, f"dish bottom {dsh[0][2]} not on table"
    for zn, zx in (("Zeolite", ZEO1_X), ("Zeolite2", ZEO2_X)):
        zeo = boxes[zn]
        assert abs(zeo[0][2] - dsh[1][2]) < 0.002, f"{zn} bottom {zeo[0][2]} not on dish top {dsh[1][2]}"
        zcx = (zeo[0][0] + zeo[1][0]) / 2
        assert abs(zcx - zx) < 0.003, f"{zn} x center {zcx:.4f} not at {zx}"
        assert abs((zeo[0][1] + zeo[1][1]) / 2 - DISH_Y) < 0.003, f"{zn} y center off dish {DISH_Y}"
    sbt = boxes["SampleBottle"]
    assert abs(sbt[0][2] - TABLE_TOP) < 0.002, f"bottle bottom {sbt[0][2]} not on table"
    # 瓶塞已删（开瓶）：/World/SampleBottle 下无 "stopper" prim
    sbp = st2.GetPrimAtPath("/World/SampleBottle")
    stoppers = [pp.GetName() for pp in Usd.PrimRange(sbp) if pp.GetName() == "stopper"]
    assert not stoppers, f"stopper still present: {stoppers}"
    mt = boxes["Match"]
    assert mt[0][2] > TABLE_TOP + 0.010, f"match bottom {mt[0][2]} not raised 12mm above table"
    print(f"[verify] OK: 台面贴底 / 网坐环上 / 管坐网上 / 灯在网下 / 钩在柱内 / 架贴台 / 滴管+温度计插孔 / 滴加效果(管柱隐藏h0+瓶液面可见+滴球) / 气泡组齐({nb}泡r{BUBBLE_R}) / 火焰迁/World顶层(默认可见) / 皿贴台+两颗沸石并排叠皿上+样品瓶(开瓶)+火柴抬高12mm")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot180 in EQUIP:
        add_equip(stage, name, asset, t, scale, rot180)
    add_b2_effects(stage)
    add_color_liquid(stage)      # 加热前/后候选色液柱（初始隐藏，task 按 cfg 各 show 一根）
    add_dropper_drops(stage)     # 挤胶头滴落串（初始隐藏，task 动画驱动）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    remove_stoppers(st2)         # 样品瓶开瓶（删 stopper，瓶口 0.870）
    move_lamp_cap(st2)
    rebuild_flames(st2)          # 火焰迁到 /World 顶层（灯下引用子 prim RTX 不渲染）
    brighten_lights(st2)
    fix_env_light(st2)
    fix_bottle_materials(st2)    # 瓶玻璃透明化（SampleLiquid 液面透出）
    fix_dropper_materials(st2)   # 滴管玻璃透明化（吸起的液体/滴落效果透出）
    fix_tube_material(st2)       # 试管玻璃透明化（滴加液柱/气泡看得清）
    fix_thermo_material(st2)     # 温度计红液去反光（matte+emissive 红，camera_2 看清读数）
    raise_stand_ring_hook(st2)   # 2026-08-27 铁环/挂钩上移 2cm + 挂钩额外 1cm（网/管/夹已在 EQUIP 常量+RING_RAISE）
    raise_stand_pole(st2)        # 柱加高 POLE_RAISE（挂钩上移后钩卡箍顶逼近柱顶）
    rotate_thermo_stem(st2)      # 2026-08-27 温度计玻璃管绕竖轴 -90°（刻度面转朝 camera_2，挂环不动）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
