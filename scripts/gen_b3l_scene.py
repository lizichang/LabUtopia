# -*- coding: utf-8 -*-
"""生成 b3l_water_bath.usd —— B3L 水浴加热·液体实验（滴加溶液 → 酒精灯加热烧杯水浴
→ 试管内液体渐变变色）场景（烘平自包含）。

基于 b3s_water_bath.usd 骨架（加热堆叠/试管架/火柴/灯帽/火焰/烧杯水浴全保留），
仅替换「挖粉」为「滴加溶液」：
  删 Spatula/SurfaceDish/SamplePowder → 增 Dropper（dropper.usd，立插架近侧列第3排
  (0.659,0.3209)，尖嘴底贴洞底 0.806）+ SolutionBottle（sample_bottle.usd，替代表面皿位
  (0.5365,0.20)，auto 贴台面 0.80，瓶口 rim 0.870 / 液面 0.840；瓶盖倒放桌面
  (0.4915,0.20) 密封面朝上）。
  删挖粉效果（PowderOnSpoon/PowderDrop/TubeSample）→ 增滴加效果（照 d3l/B2）：
  SolutionLiquid（瓶内半瓶液，可见）+ DropperDrop（挤胶头滴落串，隐藏）
  + TestTubeLiquid（无色水柱 fallback，隐藏 h0）+ TubeLiquidBefore_<色>/TubeLiquidColor_<色>
  （加热前/后候选色液柱，隐藏 h0，task 驱动渐变变色）。
  删熔化液柱 TubeMelt_<色>。

烧杯水浴（BeakerWater 可见 + BeakerBubbles 30 泡初始隐藏）、酒精灯/石棉网/铁架台
（挂钩隐藏）、火柴/灯帽、水滴形火焰迁 /World 顶层——全部照搬 b3s 不动。

用法：python scripts/gen_b3l_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b3l_water_bath")
OUT = os.path.join(SCENE_DIR, "b3l_water_bath.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# ---- 加热堆叠（铁架台/酒精灯/石棉网/烧杯）：同 b3s，整组 -Y 平移 25cm ----
STAND_X, STAND_Y = 0.6286, -0.25
TUBE_X = STAND_X - 0.100          # 0.5286（烧杯/酒精灯/石棉网同轴）
TUBE_Y = STAND_Y                  # -0.25

GAUZE_Z = TABLE_TOP + 0.1194      # 石棉网中心 0.9194
GAUZE_BOTTOM_Z = GAUZE_Z - 0.0010 # 0.9184（火焰 apex 目标）
GAUZE_TOP_Z = GAUZE_Z + 0.0011    # 0.9205（烧杯底贴这里）

# 250mL 烧杯（beaker.usd：Ø75 × 高 90mm，内建 rotateXYZ(-135,0,0)，底 z=0 顶 z=0.0904）
BEAKER_BOTTOM_Z = GAUZE_TOP_Z     # 0.9205
BEAKER_TOP_Z = BEAKER_BOTTOM_Z + 0.0904  # 1.0109

# 烧杯内水（水浴）：2/3 填充，r=0.031，水面 0.9805
WATER_H = 0.060
WATER_TOP_Z = BEAKER_BOTTOM_Z + WATER_H   # 0.9805
WATER_R = 0.031
WATER_CZ = BEAKER_BOTTOM_Z + WATER_H / 2  # 0.9505

# 火柴：同 b3s（架后 -X/-Y 各 20cm，头 +X 端朝灯芯）
MATCH_X, MATCH_Y = 0.3314, 0.1607
MATCH_T = 0.813

# ---- 阶段A 样品区：b3s 布局，挖粉器皿换滴管+溶液瓶 ----
RACK_X, RACK_Y = 0.6803, 0.3607
RACK_TZ = TABLE_TOP + 0.0965       # 0.8965（架底贴台面）
RACK_HOLE_BOTTOM = RACK_TZ - 0.0905  # 0.8060（架孔洞底，管/滴管底贴此）
RACK_TUBE_X, RACK_TUBE_Y = 0.659, 0.241   # 试管（架近侧左孔，同 b3s/d2s）
TUBE_BOTTOM_Z = RACK_HOLE_BOTTOM   # 0.8060
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.1533  # 0.9593
# 滴管（dropper.usd：尖嘴底=原点）立插架近侧列第3排，尖嘴底贴洞底
# （原中心孔 0.6993,0.3608 距底座 0.904m 手指朝下 IK 不可达；2026-08-31 挪近侧列 + ORIENT_FWD，
#   用户再挪第2排(0.281)→第3排(0.3209)，0.853m 仍可达）
DROPPER_X, DROPPER_Y = 0.659, 0.3209
DROPPER_Z = RACK_HOLE_BOTTOM       # 0.8060
# 溶液瓶（sample_bottle.usd：auto 贴台面 0.80；瓶口 rim 0.870、液面 0.840），替代表面皿位
# y=0.20（2026-08-31 自 0.105 挪：rel y=0.15 达 D2S 判据，旧 rel y=0.055 下探瓶口 IK FAIL）
SOLUTION_BOTTLE_X, SOLUTION_BOTTLE_Y = 0.5365, 0.20

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面
EQUIP = [
    ("IronStand", "iron_stand.usd", (STAND_X, STAND_Y, None), None, True),
    ("AlcoholLamp", "alcohol_lamp.usd", (TUBE_X, TUBE_Y, None), None, True),
    ("AsbestosGauze", "asbestos_gauze.usd", (TUBE_X, TUBE_Y, GAUZE_Z), None, True),
    ("Beaker", "beaker.usd", (TUBE_X, TUBE_Y, BEAKER_BOTTOM_Z), None, False),
    # 阶段A 样品区：试管立架孔 + 滴管立插架中心孔 + 溶液瓶（删药匙/皿/粉）
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, False),
    ("TestTube", "test_tube.usd", (RACK_TUBE_X, RACK_TUBE_Y, RACK_HOLE_BOTTOM), None, False),
    ("Dropper", "dropper.usd", (DROPPER_X, DROPPER_Y, DROPPER_Z), None, False),
    ("SolutionBottle", "sample_bottle.usd", (SOLUTION_BOTTLE_X, SOLUTION_BOTTLE_Y, None), None, False),
    ("Match", "match.usd", (MATCH_X, MATCH_Y, MATCH_T), None, False),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def _points_bbox(st2, prim_path):
    """世界坐标 points-based 包围盒（避开 BBoxCache 的 extent 陈旧/旋转失真）。"""
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


# ---- B3L 效果 prim（内建，task 动画驱动）----
# 烧杯内水柱（水浴，蓝半透明，可见）、烧杯内加热气泡（球组，初始隐藏）。
# 溶液瓶内半瓶液（SolutionLiquid，可见）、试管内无色水柱 fallback（TestTubeLiquid，隐藏 h0）、
# 加热前/后候选色液柱（TubeLiquidBefore_<色>/TubeLiquidColor_<色>，隐藏 h0）、
# 挤胶头滴落串（DropperDrop，父 + 4 球，隐藏）。
BUBBLE_R = 0.002                       # 烧杯水浴气泡半径（同 b3s）

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

# 水浴/瓶液/滴落液材质配方（d2l-liquid-color-recipe：近黑 diffuse + 单通道主导 emissive）
WATER = dict(color=(0.72, 0.85, 1.0), opacity=0.50, roughness=0.05, ior=1.33)
DROP = dict(color=(0.35, 0.75, 1.0), opacity=0.90, roughness=0.05, ior=1.33)
# 候选色液柱配方（照 B2 LIQUID_COLORS）：加热前色 = 主液柱，加热后色 = 变色柱
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
# 瓶/滴管玻璃配方（assets 自带 op1.0 磨砂不透明，改真玻璃透出液体，照 b2 GLASS op0.25）
BOTTLE_GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)
# 试管内液柱半径（试管 Ø19.2mm 外 r0.0096 壁厚~1.1mm 内径~0.0085）：主液柱 LIQUID_R 贴内缘；
# 变色柱 COLOR_R 必须 **大于** LIQUID_R 盖在主液柱外层——否则细柱被不透明(op0.95)主液柱包住、
# 变色永远透不出来（2026-09-01 修「溶液根本没变蓝」，旧 COLOR_R=LIQUID_R-0.0004 细一圈是错的）。
LIQUID_R = 0.0070
COLOR_R = 0.0080                        # >LIQUID_R 盖红柱外，<内径 0.0085 不穿模
# 溶液瓶内半瓶液（(0.5365,0.20) 台面 0.80..液面 0.840 = 半瓶）
BOTTLE_LIQ_R = 0.014
BOTTLE_LIQ_H = 0.040                  # 0.80..0.84
BOTTLE_LIQ_CZ = TABLE_TOP + BOTTLE_LIQ_H / 2      # 0.820
# 挤胶头滴落串：一次挤 DROPS_PER_GROUP 滴连续坠落（同 B2/d3l）
DROPS_PER_GROUP = 4
DROP_BALL_R = 0.003
DROP_HOME = (RACK_TUBE_X, RACK_TUBE_Y, TUBE_BOTTOM_Z + 0.020)   # 架内试管底部（动画起点）


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
    """烧杯内水柱（水浴）：蓝半透明柱，可见。"""
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


def add_solution_liquid(stage):
    """溶液瓶内半瓶液：蓝色半透明柱（瓶底 0.80..液面 0.840），可见（吸液源）。"""
    sl = UsdGeom.Cylinder.Define(stage, "/World/SolutionLiquid")
    sl.CreateRadiusAttr(BOTTLE_LIQ_R)
    sl.CreateHeightAttr(BOTTLE_LIQ_H)
    sl.CreateAxisAttr("Z")
    sl.AddTranslateOp().Set(Gf.Vec3d(SOLUTION_BOTTLE_X, SOLUTION_BOTTLE_Y, BOTTLE_LIQ_CZ))
    add_material(stage, sl.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    print(f"[effect] SolutionLiquid visible (bottle {BOTTLE_LIQ_H:.3f}m to top 0.840)")


def add_testtube_liquid(stage):
    """试管内无色水柱 fallback（cfg.before_color=clear 时主液柱用它）：隐藏 h0，滴加逐滴长高。"""
    liq = UsdGeom.Cylinder.Define(stage, "/World/TestTubeLiquid")
    liq.CreateRadiusAttr(LIQUID_R)
    liq.CreateHeightAttr(0.0)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(RACK_TUBE_X, RACK_TUBE_Y, TUBE_BOTTOM_Z))
    add_material(stage, liq.GetPrim(), WATER["color"], WATER["opacity"],
                 roughness=WATER["roughness"], ior=WATER["ior"], double_sided=True)
    UsdGeom.Imageable(liq).MakeInvisible()
    print(f"[effect] TestTubeLiquid hidden h0 (grow by dropper drip)")


def add_color_liquid(stage):
    """加热前/后候选色液柱（照 B2 add_color_liquid）：TubeLiquidBefore_<色>（主液柱，滴加逐滴
    长高）+ TubeLiquidColor_<色>（变色柱，粗一圈 COLOR_R 盖外层，顶贴液面向下扩散）。初始全隐藏 h0；
    task 按 cfg.before_color 显主液柱一根（clear 回退 TestTubeLiquid）、cfg.liquid_color 显变色柱
    一根（加热时 _step_color_transition 顶贴液面向下渐变染过去）。试管会从架孔移到水浴，
    task 以试管当前管底为基准写柱位（_set_tube_world 跟踪 _tube_bottom），故此处初始锚定
    架内试管位即可。"""
    for prefix, r in (("TubeLiquidBefore", LIQUID_R), ("TubeLiquidColor", COLOR_R)):
        for name, m in LIQUID_COLORS.items():
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{prefix}_{name}")
            geom.CreateRadiusAttr(r)
            geom.CreateHeightAttr(0.0)
            geom.CreateAxisAttr("Z")
            geom.AddTranslateOp().Set(Gf.Vec3d(RACK_TUBE_X, RACK_TUBE_Y, TUBE_BOTTOM_Z))
            translucent = m.get("opacity", 1.0) < 1.0
            add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                         roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                         emissive=m.get("emissive"), double_sided=translucent)
            UsdGeom.Imageable(geom).MakeInvisible()
            print(f"[effect] {prefix}_{name} hidden (r={r})")


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 + Drop_0.._N 亮蓝小球（r=0.003）。父+每球都
    MakeInvisible（防 delay 中的球停在 home 位闪现成拖影）。task._on_drop 每次挤生成一串、
    _step_drop_anim 逐滴错帧坠落。"""
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


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）。"""
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
    """灯帽从灯顶挪到桌边（闭口朝下贴台面）静止位：世界帽位 (0.42, STAND_Y-0.0129)。"""
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


# 酒精灯火焰（同 b3s 水滴形，底半球 + 上部锥收尖，迁 /World 顶层）。
FLAME_BASE_Z = TABLE_TOP + 0.091            # 灯芯根部世界 z（火焰底）
FLAME_APEX_Z = GAUZE_Z - 0.00105            # 石棉网底 0.9184
FLAME_OUTER_R = 0.009
FLAME_INNER_R = 0.005
FLAME_INNER_APEX_Z = FLAME_BASE_Z + 0.022

def add_droplet_flame(st2, name, r, z_b, z_a, emissive):
    """水滴形火焰 = 底半球 Sphere + 上部 Cone 收尖，绕 Z 轴。"""
    zc = z_b + r
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
    print(f"[lamp] droplet {name}: sphere r{r} c{zc:.4f} (bottom {z_b:.4f}) + cone apex {z_a:.4f}")


def rebuild_flames(st2):
    """酒精灯火焰：删灯下引用子 prim，在 /World 顶层重建。"""
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if st2.GetPrimAtPath(path).IsValid():
            st2.RemovePrim(path)
    add_droplet_flame(st2, "flame_outer", FLAME_OUTER_R, FLAME_BASE_Z, FLAME_APEX_Z,
                      (0.35, 0.55, 2.40))
    add_droplet_flame(st2, "flame_inner", FLAME_INNER_R, FLAME_BASE_Z, FLAME_INNER_APEX_Z,
                      (2.80, 0.55, 0.20))
    print(f"[lamp] flames droplet: outer base {FLAME_BASE_Z:.4f} apex {FLAME_APEX_Z:.4f} "
          f"(touches gauze bottom {GAUZE_BOTTOM_Z:.4f})")


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（烘平后 MaterialBindingAPI 未 apply，用 material:binding
    取材质路径再找 shader，同 b3s/d3l gen）。"""
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
    """烧杯玻璃透明化 + 去反光（同 b3s：op 0.12 真玻璃 + ior1.5 + rough0.25）。"""
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
    """铁架台挂钩隐藏。"""
    hook = st2.GetPrimAtPath("/World/IronStand/root/hook")
    if not hook.IsValid():
        print("[clean] /World/IronStand/root/hook not found, skip")
        return
    UsdGeom.Imageable(hook).MakeInvisible()
    print("[clean] iron stand hook hidden")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光（同 b3s：op 0.12 + ior 1.5 + roughness 0.25）。"""
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


def flip_solution_stopper(st2):
    """溶液瓶盖倒放桌面（实验室标准：开瓶后盖子倒放、密封面朝上不触台面防污染，同 d4s/d4l）。

    保留原 prim（复用 sample_bottle 的 stopper 几何/材质，原无 xform op 纯几何），给
    /World/SolutionBottle/stopper 加 xform op：op 顺序 [translate, rotateXYZ]（translate
    最外层不被旋转，同 d4s 手法）。rotateY180° 让瓶口密封面（原下底面 z=0.068）朝上 →
    倒放；translate 移到瓶 -X 侧 45mm 并抬 0.079（翻转后下底 z=-0.079 回到台面 0.80）→
    世界中心 (0.4915,SOLUTION_BOTTLE_Y)、盖厚 11mm 竖放、密封面（洁净面）朝上不触台面。"""
    p = st2.GetPrimAtPath("/World/SolutionBottle")
    if not p.IsValid():
        print("[clean] /World/SolutionBottle not found, skip")
        return
    stopper = p.GetChild("stopper")
    if not stopper:
        print("[clean] SolutionBottle has no stopper, skip flip")
        return
    xf = UsdGeom.Xformable(stopper)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-0.045, 0.0, 0.079))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 180.0, 0.0))
    print("[clean] SolutionBottle stopper flipped to desk "
          f"(inverted, sealing face up, center {SOLUTION_BOTTLE_X - 0.045:.4f},{SOLUTION_BOTTLE_Y:.4f} "
          "on table 0.80)")


def fix_bottle_materials(st2):
    """溶液瓶玻璃透明化（磨砂 op1.0 → 真玻璃 op0.25 + ior1.5 + doubleSided），瓶内
    SolutionLiquid 液面才透得出来。若自带 1mm 液面薄盘（"liquid" mesh）隐藏。"""
    p = st2.GetPrimAtPath("/World/SolutionBottle")
    if not p.IsValid():
        print("[mat] /World/SolutionBottle not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if c.GetName() == "liquid":
            UsdGeom.Imageable(c).MakeInvisible()
            print(f"[mat] hid {c.GetPath()} (1mm liquid disc, replaced by SolutionLiquid)")
        else:
            if override_bound_shader(st2, c, BOTTLE_GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 opacity=1.0 不透明光面（遮住吸起的液体）
    → 真玻璃 op0.25 + ior1.5（照 b2），滴管才透出吸液/滴落效果；胶头保持不透明。"""
    mat = st2.GetPrimAtPath("/World/Dropper/_materials/glass_001")
    if not mat.IsValid():
        print("[mat] Dropper glass_001 not found, skip")
    else:
        for c in mat.GetChildren():
            if c.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(c)
            for n, val in BOTTLE_GLASS.items():
                inp = sh.GetInput(n)
                vt = Sdf.ValueTypeNames.Color3f if n == "diffuseColor" else Sdf.ValueTypeNames.Float
                if not inp:
                    inp = sh.CreateInput(n, vt)
                inp.Set(val)
            print(f"[mat] Dropper glass_001 -> transparent {BOTTLE_GLASS}")
    g = st2.GetPrimAtPath("/World/Dropper/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")
    else:
        print("[mat] Dropper glass mesh not found for doubleSided, skip")


def verify(st2):
    """自检：打印各器材世界 bbox，断言 B3L 布局：加热堆叠（网坐环上/烧杯直立/水在杯内）
    + 样品区（试管/滴管立架孔、溶液瓶贴台）+ 滴加效果（瓶液可见/变色液柱/滴落串全隐藏）
    + 火焰迁顶层 + 挂钩隐藏。beaker 用 _points_bbox（旋转+extent 陈旧），其余 BBoxCache。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "AsbestosGauze", "Beaker", "TestTubeRack", "TestTube",
             "Dropper", "SolutionBottle", "Match", "BeakerWater"]
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
        elif name == "SolutionBottle":
            # 只量瓶身（bottle 子 mesh），排除翻放桌面的瓶盖 stopper 子 prim
            bottle_mesh = p.GetChild("bottle")
            if not bottle_mesh.IsValid():
                print(f"[verify] /World/{name}/bottle missing")
                continue
            r = bc.ComputeWorldBound(bottle_mesh).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
        else:
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:13s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 铁架台底座贴台面 + 挂钩隐藏
    smn, smx = boxes["IronStand"]
    assert abs(smn[2] - TABLE_TOP) < 0.002, f"IronStand base z {smn[2]} != table {TABLE_TOP}"
    hook = st2.GetPrimAtPath("/World/IronStand/root/hook")
    assert hook.IsValid(), "iron stand hook missing"
    assert UsdGeom.Imageable(hook).ComputeVisibility() == "invisible", "hook should be hidden"
    # 石棉网坐铁环上
    gmn, gmx = boxes["AsbestosGauze"]
    ring_top = TABLE_TOP + 0.118
    assert gmn[2] >= ring_top - 0.002, f"gauze bottom {gmn[2]} below ring top {ring_top}"
    # 烧杯直立坐石棉网上（points 实测）：底贴网顶、高 90.4mm、堆叠 y 平移
    bmn, bmx = boxes["Beaker"]
    assert 0 <= bmn[2] - gmx[2] <= 0.0015, f"beaker bottom {bmn[2]} not on gauze top {gmx[2]}"
    assert abs((bmx[2] - bmn[2]) - 0.0904) < 0.003, f"beaker height {bmx[2]-bmn[2]:.4f} != 0.0904"
    assert abs(bmx[2] - BEAKER_TOP_Z) < 0.003, f"beaker top {bmx[2]:.4f} != {BEAKER_TOP_Z}"
    assert abs((bmn[1] + bmx[1]) / 2 - STAND_Y) < 0.003, \
        f"beaker y {(bmn[1]+bmx[1])/2:+.4f} != STAND_Y {STAND_Y} (stack shifted)"
    assert not st2.GetPrimAtPath("/World/TestTubeClamp").IsValid(), \
        "TestTubeClamp should be removed (user: 不需要试管夹)"
    # 烧杯水柱在烧杯内
    wmn, wmx = boxes["BeakerWater"]
    assert wmn[2] > bmn[2] - 0.002, f"beaker water bottom {wmn[2]} below beaker bottom {bmn[2]}"
    assert wmx[2] < bmx[2] - 0.005, f"beaker water top {wmx[2]} too close to beaker top {bmx[2]}"
    # 试管架贴台面、试管立架孔里（底贴洞底 0.8060）
    rmn, rmx = boxes["TestTubeRack"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"rack bottom {rmn[2]} not on table"
    assert abs((rmn[0] + rmx[0]) / 2 - RACK_X) < 0.01 and abs((rmn[1] + rmx[1]) / 2 - RACK_Y) < 0.01, \
        f"rack center not at b3s ({RACK_X},{RACK_Y})"
    tmn, tmx = boxes["TestTube"]
    assert abs(tmn[2] - RACK_HOLE_BOTTOM) < 0.003, f"tube bottom {tmn[2]:.4f} not in rack hole {RACK_HOLE_BOTTOM}"
    assert rmn[0] < tmn[0] < rmx[0], "tube x not within rack x"
    assert abs((tmn[1] + tmx[1]) / 2 - RACK_TUBE_Y) < 0.003, \
        f"tube y center {((tmn[1]+tmx[1])/2):.4f} != {RACK_TUBE_Y}"
    # 滴管立架中心孔（原药匙位）：尖嘴底贴洞底、xy 在架范围
    dmn, dmx = boxes["Dropper"]
    assert abs(dmn[2] - RACK_HOLE_BOTTOM) < 0.003, f"dropper tip bottom {dmn[2]:.4f} not in rack hole"
    assert rmn[0] < dmn[0] and dmx[0] < rmx[0], "dropper x not within rack"
    assert rmn[1] < dmn[1] and dmx[1] < rmx[1], "dropper y not within rack"
    # 溶液瓶贴台面（0.80）、中心在 (0.5365,0.20)
    btl = boxes["SolutionBottle"]
    assert abs(btl[0][2] - TABLE_TOP) < 0.002, f"bottle bottom {btl[0][2]} not on table"
    bx, by = 0.5 * (btl[0][0] + btl[1][0]), 0.5 * (btl[0][1] + btl[1][1])
    assert abs(bx - SOLUTION_BOTTLE_X) < 0.01 and abs(by - SOLUTION_BOTTLE_Y) < 0.01, \
        f"bottle center ({bx:.3f},{by:.3f}) off ({SOLUTION_BOTTLE_X},{SOLUTION_BOTTLE_Y})"
    # 瓶盖倒放桌面：盖底贴台面 0.80、盖厚 11mm、中心在瓶 -X 侧 45mm、rotY180（倒放密封面朝上）
    cap = st2.GetPrimAtPath("/World/SolutionBottle/stopper")
    assert cap.IsValid(), "SolutionBottle stopper missing"
    rc = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn3, cmx3 = rc.GetMin(), rc.GetMax()
    assert abs(cmn3[2] - TABLE_TOP) < 0.002, f"stopper bottom {cmn3[2]:.4f} not on table 0.80"
    assert abs(cmx3[2] - cmn3[2] - 0.011) < 0.002, f"stopper height {cmx3[2]-cmn3[2]:.4f} != 11mm"
    scx, scy = 0.5 * (cmn3[0] + cmx3[0]), 0.5 * (cmn3[1] + cmx3[1])
    assert abs(scx - (SOLUTION_BOTTLE_X - 0.045)) < 0.006, f"stopper center x {scx:.4f} != 0.4915"
    assert abs(scy - SOLUTION_BOTTLE_Y) < 0.006, f"stopper center y {scy:.4f} != {SOLUTION_BOTTLE_Y}"
    cap_rot = cap.GetAttribute("xformOp:rotateXYZ")
    assert cap_rot and abs(cap_rot.Get()[1] - 180.0) < 1.0, \
        f"stopper rotateXYZ.y != 180 (not inverted): {cap_rot.Get() if cap_rot else None}"
    # 酒精灯灯芯顶低于石棉网底（火焰区留白 >= 1.5cm）
    lmn, lmx = boxes["AlcoholLamp"]
    wick_top = TABLE_TOP + 0.1005
    assert wick_top + 0.015 < gmn[2], f"flame gap {gmn[2] - wick_top:.3f} < 1.5cm"
    # 灯帽静止位：帽底贴台面、中心 (0.42, STAND_Y-0.0129)
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn2, cmx2 = r.GetMin(), r.GetMax()
    assert abs(cmn2[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn2[2]} not on table"
    cx, cy = 0.5 * (cmn2[0] + cmx2[0]), 0.5 * (cmn2[1] + cmx2[1])
    assert abs(cx - 0.42) < 0.005, f"cap center x {cx:+.4f} != 0.42"
    assert abs(cy - (STAND_Y - 0.0129)) < 0.005, \
        f"cap center y {cy:+.4f} != {STAND_Y - 0.0129:.4f}"
    # 火柴抬高 12mm
    mt = boxes["Match"]
    assert mt[0][2] > TABLE_TOP + 0.010, f"match bottom {mt[0][2]} not raised 12mm above table"
    # 效果 prim：烧杯水柱可见、水浴气泡组齐且隐藏
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/BeakerWater")).ComputeVisibility() != "invisible", \
        "BeakerWater should be visible"
    bub = st2.GetPrimAtPath("/World/BeakerBubbles")
    assert bub.IsValid(), "BeakerBubbles missing"
    nb = sum(1 for c in bub.GetChildren() if c.GetTypeName() == "Sphere")
    assert nb == len(BUBBLE_BASE), f"bubbles count {nb} != {len(BUBBLE_BASE)}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/BeakerBubbles/bubble_0")).ComputeVisibility() == "invisible", \
        "bubble_0 should be hidden"
    # 溶液瓶液可见（吸液源）、试管内无色水柱 fallback 隐藏 h0
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/SolutionLiquid")).ComputeVisibility() != "invisible", \
        "SolutionLiquid should be visible"
    slp = st2.GetPrimAtPath("/World/SolutionLiquid")
    assert abs(UsdGeom.Cylinder(slp).GetRadiusAttr().Get() - BOTTLE_LIQ_R) < 1e-9, \
        "SolutionLiquid r != BOTTLE_LIQ_R"
    assert abs(UsdGeom.Cylinder(slp).GetHeightAttr().Get() - BOTTLE_LIQ_H) < 1e-9, \
        "SolutionLiquid h != BOTTLE_LIQ_H"
    tl = st2.GetPrimAtPath("/World/TestTubeLiquid")
    assert tl.IsValid(), "TestTubeLiquid missing"
    assert UsdGeom.Imageable(tl).ComputeVisibility() == "invisible", "TestTubeLiquid should be hidden"
    assert abs(UsdGeom.Cylinder(tl).GetHeightAttr().Get() or 0.0) < 1e-9, "TestTubeLiquid h should be 0"
    # 变色液柱：2×len(LIQUID_COLORS) 根，h0、隐藏；Before r=LIQUID_R、Color r=COLOR_R
    for prefix, r in (("TubeLiquidBefore", LIQUID_R), ("TubeLiquidColor", COLOR_R)):
        for name in LIQUID_COLORS:
            p = st2.GetPrimAtPath(f"/World/{prefix}_{name}")
            assert p.IsValid(), f"{prefix}_{name} missing"
            assert UsdGeom.Imageable(p).ComputeVisibility() == "invisible", \
                f"{prefix}_{name} should be hidden"
            assert abs(UsdGeom.Cylinder(p).GetHeightAttr().Get() or 0.0) < 1e-9, \
                f"{prefix}_{name} h should be 0"
            assert abs(UsdGeom.Cylinder(p).GetRadiusAttr().Get() - r) < 1e-9, \
                f"{prefix}_{name} r != {r}"
    # 滴落串齐且全隐藏
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    assert dd.IsValid(), "DropperDrop missing"
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == DROPS_PER_GROUP, f"DropperDrop spheres {nd} != {DROPS_PER_GROUP}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible", "DropperDrop parent should be hidden"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/DropperDrop/Drop_0")).ComputeVisibility() == "invisible", \
        "Drop_0 should be hidden"
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
    # B3L 特有：删了挖粉器皿/效果、熔化液柱
    for gone in ("Spatula", "SurfaceDish", "SamplePowder",
                 "PowderOnSpoon", "PowderDrop", "TubeSample", "TubeMelt_red"):
        assert not st2.GetPrimAtPath(f"/World/{gone}").IsValid(), f"{gone} should be removed"
    print(f"[verify] OK: 台面贴底 / 网坐环上 / 烧杯直立坐网上(90mm,堆叠移y={STAND_Y}) / 水柱在杯内 / "
          f"样品区(架/试管/滴管立孔/溶液瓶贴台) / 挂钩隐藏 / 无试管夹 / 灯在网下 / 帽随灯移 / "
          f"火柴抬高12mm / 水浴气泡组齐({nb}泡)隐藏 / 溶液瓶液可见 / 变色液柱2x{len(LIQUID_COLORS)}根"
          f"隐藏 / 滴落串({nd}球)隐藏 / 挖粉器皿与熔化液柱已删 / 火焰迁/World顶层")


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
    add_solution_liquid(stage)
    add_testtube_liquid(stage)
    add_color_liquid(stage)
    add_dropper_drops(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    move_lamp_cap(st2)
    rebuild_flames(st2)          # 火焰迁到 /World 顶层
    brighten_lights(st2)
    fix_env_light(st2)
    fix_beaker_material(st2)     # 烧杯玻璃透明化（水柱/气泡透出）
    fix_tube_material(st2)       # 试管玻璃透明化（变色液柱透出）
    hide_iron_stand_hook(st2)    # 铁架台挂钩隐藏
    flip_solution_stopper(st2)   # 溶液瓶盖倒放桌面（密封面朝上，同 d4s/d4l）
    fix_bottle_materials(st2)    # 溶液瓶玻璃透明化（瓶液透出）
    fix_dropper_materials(st2)   # 滴管玻璃透明化（吸液/滴落透出）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
