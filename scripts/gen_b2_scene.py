# -*- coding: utf-8 -*-
"""生成 b2_alcohol_heat_liquid.usd —— B2 沸点测定（酒精灯加热试管液体）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材（铁架台新钩版 / 酒精灯 / 石棉网 / 试管 / 试管夹 / 试管架 / 滴管 / 温度计
  / 玻璃皿+沸石 / 待测液体样品瓶 / 火柴——阶段A 机械臂操作升级新增）
- 布局 = 用户 b2_tmp.usd 相对位置（2026-08-25 改版：整组绕 Z 旋 180° 并平移），锚：铁架台铁柱在 (STAND_X, STAND_Y)，
  垂直堆叠中心线 x=铁架台x−0.100（堆叠在铁柱 −X 侧，R180 后环/钩支臂指向 −X）：
      酒精灯(桌面 z0.80) → 铁环(z0.910-0.918 托石棉网) → 石棉网(z0.919)
      → 试管(底 z0.9206 坐网上) → 试管夹(夹管上部 z1.024-1.053) → 钩(z1.216-1.227，挂温度计)
- 台面前区 (0.50,0.35) 放试管架：滴管插左孔（中排，底面落孔底 z0.806）、温度计插右后排孔
  (0.521,0.468)（2026-08-25 用户：右中排温度计顶高1.084 挡机械臂下探滴管 → 移 y 值最大=最远孔）
- 去资产自带 env_light 残留（重复 DomeLight）；灯帽从灯顶挪到桌边(y-0.467)；
  火焰 flame_outer/flame_inner 初始隐藏（未点火，task 再动画）

用法：python scripts/gen_b2_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
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
# 锚：铁架台铁柱世界位置。b2_tmp 2026-08-25 用户整组绕 Z 旋 180° 并平移：
# 铁柱到 (0.6286, 0.0029)，堆叠中心线在铁柱 −X 侧 10cm → x=0.5286（环心/试管/石棉网/酒精灯）。
STAND_X, STAND_Y = 0.6286, 0.0029
TUBE_X = STAND_X - 0.100
TUBE_Y = STAND_Y
GAUZE_Z = TABLE_TOP + 0.1194        # 石棉网中心（坐铁环上，环顶 0.918）
TUBE_BOTTOM_Z = TABLE_TOP + 0.1206  # 试管底（坐石棉网上，b2_tmp 试管 translate z=0.1206）
# 试管夹：b2_tmp 里相对铁柱偏移 (0.0505,-0.0209,0.2384)，整组 R180 后 → (−0.0505,+0.0209,0.2384)。
# asset test_tube_clamp.usd 已内置 cm→m 换算，故 scene 只需平移（R180 由场景 xform op [T,R] 提供）。
CLAMP_T = (STAND_X - 0.0505, STAND_Y + 0.0209, TABLE_TOP + 0.2384)
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
ZEO_T = DISH_TOP_Z + 0.0021          # 沸石底(局部 min z=-0.0021)贴皿顶 → 世界 0.8087
BOTTLE_X, BOTTLE_Y = 0.40, 0.15
MATCH_X, MATCH_Y = 0.40, -0.06       # 火柴头 +X 端朝灯芯方向 (0.5286,0.0029)

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
    ("Zeolite", "zeolite.usd", (DISH_X, DISH_Y, ZEO_T), None, False),
    ("SampleBottle", "sample_bottle.usd", (BOTTLE_X, BOTTLE_Y, None), None, False),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, None), None, False),
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
# 气泡基础位（液体内，task 上升动画的复位点；以试管中心 (TUBE_X,TUBE_Y) 为基准）
BUBBLE_BASE = [
    (TUBE_X + 0.0000, TUBE_Y + 0.0000, TUBE_BOTTOM_Z + 0.010),
    (TUBE_X - 0.0035, TUBE_Y + 0.0025, TUBE_BOTTOM_Z + 0.022),
    (TUBE_X + 0.0035, TUBE_Y - 0.0025, TUBE_BOTTOM_Z + 0.034),
    (TUBE_X - 0.0030, TUBE_Y - 0.0035, TUBE_BOTTOM_Z + 0.016),
    (TUBE_X + 0.0030, TUBE_Y + 0.0035, TUBE_BOTTOM_Z + 0.028),
    (TUBE_X + 0.0000, TUBE_Y + 0.0010, TUBE_BOTTOM_Z + 0.040),
]


# —— 阶段B 滴加：液体材质配方（同 d3l：水透明 + ior 水折射；滴落液滴更亮更不透）——
# DropperFill（滴管尖内固定液柱）已删：2026-08-25 用户「固定竖直液柱很奇怪 + 移动时
# 浅色轨迹」→ 参考 d2l 无液柱，滴管空管移动、只在挤胶头瞬间 DropperDrop 成串坠落。
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)   # 样品瓶内半瓶液 / 试管内水柱
DROP = dict(color=(0.35, 0.75, 1.0), opacity=0.90, roughness=0.05, ior=1.33)    # 挤胶头滴落的液滴
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


def add_shared_material(stage, mat_path, diffuse, opacity, prims):
    """建一个材质绑定到多个 prim（气泡组共用）。"""
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
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

    # 气泡组：6 球在液体内，初始隐藏，task 沸腾时 reveal + 上升
    UsdGeom.Xform.Define(stage, "/World/TestTubeBubbles")
    bub_prims = []
    for i, (x, y, z) in enumerate(BUBBLE_BASE):
        sp = UsdGeom.Sphere.Define(stage, f"/World/TestTubeBubbles/bubble_{i}")
        sp.CreateRadiusAttr(0.0015)
        sp.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        UsdGeom.Imageable(sp).MakeInvisible()
        bub_prims.append(sp.GetPrim())
    add_shared_material(stage, "/World/TestTubeBubbles/bubble_mat", (0.9, 0.95, 1.0), 0.7, bub_prims)
    print(f"[effect] {len(BUBBLE_BASE)} bubbles hidden")


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
    """灯帽从灯顶挪到桌边（b2_tmp 用户布局：cap 放 y-0.467，闭口朝下贴台面）。
    资产 cap xform = [Translate(0,0,0) rotateX90 scale0.01]，mesh 在局部 z[0.076,0.107]，
    故 translate z=-0.076 让帽底(z=0.076)落回台面 0.80。
    酒精灯整组已 R180，cap 局部 y 会被旋 180° 取反，故 y 用 +0.467 才能落到桌边 −Y。"""
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[clean] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(0.0, 0.467, -0.076)
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[clean] cap translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[clean] cap (no translate op) add translate {tuple(tgt)}")


def hide_flames(st2):
    """火焰初始隐藏（实验未点火）：flame_outer/flame_inner 两个 Cone。"""
    for fl in ("flame_outer", "flame_inner"):
        p = st2.GetPrimAtPath(f"/World/AlcoholLamp/{fl}")
        if not p.IsValid():
            print(f"[clean] /World/AlcoholLamp/{fl} not found, skip")
            continue
        UsdGeom.Imageable(p).MakeInvisible()
        print(f"[clean] hidden {p.GetPath()}")


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


def verify(st2):
    """自检：打印各器材世界 bbox，断言垂直堆叠关系：
    铁架台底座贴台面、石棉网坐铁环上（0.4mm 间隙）、试管底坐石棉网上（≤1mm）、
    试管夹夹住试管上部、灯芯顶低于石棉网底（火焰区留白）、钩低于铁柱顶、
    试管架贴台面、滴管/温度计插进架孔（底面落孔底、x 对准孔心）、
    液体在试管内（不超管口/不低于管底）、气泡组齐全。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "AsbestosGauze", "TestTube", "TestTubeClamp",
             "TestTubeRack", "Dropper", "Thermometer",
             "SurfaceDish", "Zeolite", "SampleBottle", "Match",
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
    # 石棉网坐铁环上：网底 > 环顶(0.918)，网底 − 环顶 ≤ 2mm
    gmn, gmx = boxes["AsbestosGauze"]
    ring_top = TABLE_TOP + 0.118
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
    # 钩低于铁柱顶（铁柱顶 = 台面 0.80 + 0.46 = 1.26）
    assert smx[2] < TABLE_TOP + 0.461, f"IronStand top {smx[2]} exceeds pole top"
    # 灯帽在桌边（y 偏移 -0.467），帽底贴台面
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    print(f"[verify] cap     min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    assert cmx[1] < TUBE_Y, f"cap y {cmx[1]} not on -y side"
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
    nb = sum(1 for _ in bub.GetChildren())
    assert nb >= 4, f"bubbles count {nb}"
    # 阶段A 新器材：玻璃皿贴台、沸石底贴皿顶且中心对齐皿中心、样品瓶贴台、火柴贴台
    dsh = boxes["SurfaceDish"]
    assert abs(dsh[0][2] - TABLE_TOP) < 0.002, f"dish bottom {dsh[0][2]} not on table"
    zeo = boxes["Zeolite"]
    assert abs(zeo[0][2] - dsh[1][2]) < 0.002, f"zeolite bottom {zeo[0][2]} not on dish top {dsh[1][2]}"
    zc = ((zeo[0][0] + zeo[1][0]) / 2, (zeo[0][1] + zeo[1][1]) / 2)
    dc = ((dsh[0][0] + dsh[1][0]) / 2, (dsh[0][1] + dsh[1][1]) / 2)
    assert abs(zc[0] - dc[0]) < 0.02 and abs(zc[1] - dc[1]) < 0.02, f"zeolite center {zc} off dish center {dc}"
    sbt = boxes["SampleBottle"]
    assert abs(sbt[0][2] - TABLE_TOP) < 0.002, f"bottle bottom {sbt[0][2]} not on table"
    # 瓶塞已删（开瓶）：/World/SampleBottle 下无 "stopper" prim
    sbp = st2.GetPrimAtPath("/World/SampleBottle")
    stoppers = [pp.GetName() for pp in Usd.PrimRange(sbp) if pp.GetName() == "stopper"]
    assert not stoppers, f"stopper still present: {stoppers}"
    mt = boxes["Match"]
    assert abs(mt[0][2] - TABLE_TOP) < 0.002, f"match bottom {mt[0][2]} not on table"
    print(f"[verify] OK: 台面贴底 / 网坐环上 / 管坐网上 / 灯在网下 / 钩在柱内 / 架贴台 / 滴管+温度计插孔 / 滴加效果(管柱隐藏h0+瓶液面可见+滴球) / 气泡组齐({nb}泡) / 皿贴台+沸石叠皿上+样品瓶(开瓶)+火柴贴台")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot180 in EQUIP:
        add_equip(stage, name, asset, t, scale, rot180)
    add_b2_effects(stage)
    add_dropper_drops(stage)     # 挤胶头滴落串（初始隐藏，task 动画驱动）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    remove_stoppers(st2)         # 样品瓶开瓶（删 stopper，瓶口 0.870）
    move_lamp_cap(st2)
    hide_flames(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    fix_bottle_materials(st2)    # 瓶玻璃透明化（SampleLiquid 液面透出）
    fix_dropper_materials(st2)   # 滴管玻璃透明化（吸起的液体/滴落效果透出）
    fix_tube_material(st2)       # 试管玻璃透明化（滴加液柱/气泡看得清）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
