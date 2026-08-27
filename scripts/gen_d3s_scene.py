# -*- coding: utf-8 -*-
"""生成 d3s_acid_reagent.usd —— D3-S 固体样品 + 酸性试剂滴加反应场景（烘平自包含）。

D3-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加酸性试剂」：
  - 挖粉动作完全不变：药匙/表面皿/粉末/试管/试管架 相对位置与 d2s **逐字一致**（用户
    要求，机械臂与粉末/试管相对位置都不变，挖粉轨迹才不会跑偏）
  - 洗瓶 (0.370,0.525) 位置改成 酸滴管（进主试管架空孔，与药匙同架）+ 盐酸瓶（原洗瓶附近）
  - 现象：白粉在管底 → 酸逐滴滴入 → 气泡/沉淀/液体变色（同 d3l 酸现象，几何实现）

基于 lab_001.usd 副本（d2s 同款：白名单删器材 + 抬台面 0.80 + 烘平自包含），叠加 d3l 的
瓶/滴管/试管玻璃透明化 + 去瓶塞 + 酸瓶 1mm 液面盘隐藏 + 气泡/变色液柱预烘焙。

布局（d2s 实测坐标 2026-08-14 起，2026-08-25 酸瓶/滴管新增，2026-08-26 药匙/滴管同移第一列；
台面顶 z=0.80）：
  TestTubeRack  (0.6803, 0.3607)  工作区右侧（同 d2s，不动）
  TestTube      (0.659,  0.241, 0.806)  架近侧左孔=第一列第1排（同 d2s，管口 z=0.9593）
  Spatula       (0.659, 0.3209, 0.828, rotZ -180°)  第一列第3排竖插（2026-08-26 用户改位，第二列清空）
  SurfaceDish   (0.5365, 0.105, 0.80)  表面皿（同 d2s）
  SamplePowder  (0.5383, 0.0992, 0.7988, scale 0.4)  粉末（同 d2s）
  DropperAcid   (0.659, 0.4002, 0.806)  酸滴管立插主试管架第一列第5排（2026-08-26 三次改位：①右后远端
                                         0.6993,0.4804 实测 IK 够不着→②第二列第3排与药匙同列相邻穿模→
                                         ③"药匙第一列第三排、滴管第一列第5排、第二列不放物品"=本布局）
  HClBottle     (0.370, 0.30)  盐酸试剂瓶（瓶口 rim 0.870、液面 0.840，去瓶塞后）

用法：python scripts/gen_d3s_scene.py   （运行环境：本地 conda env 有 pxr）
"""
import math
import os
import random
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d3s_acid_reagent")
OUT = os.path.join(SCENE_DIR, "d3s_acid_reagent.usd")
LAB001 = os.path.join(REPO, "assets", "scenes", "base", "lab_001", "lab_001.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# lab_001 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# 试管中心（孔心）：所有管内效果/气泡/变色液柱/滴落 home 位都以它为基准（d2s 试管位）
TUBE_CX, TUBE_CY = 0.659, 0.241
TUBE_BOTTOM = 0.806          # 管底（架孔底面）世界 z
TUBE_MOUTH = 0.9593          # 管口世界 z

# (prim, asset_file, translate, scale, rot_z)   tz=None 表示动态贴台面（资产底座 min z -> 0.80）
# 皿/粉/试管/试管架坐标与 d2s 完全一致（挖粉轨迹不动）；洗瓶位换成滴管架+酸滴管+酸瓶。
# 2026-08-26 用户：药匙移到第一列第3排 (0.659,0.3209)、滴管第一列第5排 (0.659,0.4002)、第二列清空
# （原第二列第3排滴管 + 第4排药匙同列相邻，夹一个穿模另一个）。
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.6803, 0.3607, None), None, None),
    ("TestTube", "test_tube.usd", (0.659, 0.241, 0.806), None, None),
    ("Spatula", "spatula.usd", (0.659, 0.3209, 0.828), None, -180.0),
    ("SurfaceDish", "sample_dish.usd", (0.5365, 0.105, 0.80), None, None),
    ("SamplePowder", "powder.usd", (0.5383, 0.0992, 0.7988), 0.4, None),
    # —— 酸试剂（滴管进主试管架第一列第5排，与药匙同列相隔一排；盐酸瓶在原洗瓶附近）——
    ("DropperAcid", "dropper.usd", (0.659, 0.4002, TUBE_BOTTOM), None, None),
    ("HClBottle", "hcl_bottle.usd", (0.370, 0.30, None), None, None),
]

# 液体/固体/现象材质配方（d3l 同款键名：color / opacity / roughness / ior / emissive）
# 白粉（固体样品，d2s SOLUBILITY_COLORS["white"] 同款）
POWDER = dict(color=(0.93, 0.93, 0.94), opacity=1.0, roughness=0.5)
# 酸性试剂液（d3l ACID：微绿区分酸液，roughness 0.05 + ior 1.33 真液面）
ACID = dict(color=(0.66, 0.86, 0.76), opacity=0.70, roughness=0.05, ior=1.33)
# 沉淀/浑浊云：全哑光乳白（d3l 同款）
OPAQUE_WHITE = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85,
                    emissive=(1.0, 0.95, 0.80))
CLOUD_MILK = dict(color=(0.82, 0.80, 0.74), opacity=1.0, roughness=0.85,
                  emissive=(1.0, 0.95, 0.80))
# 滴落液滴：酸色更亮更不透（透过玻璃可见）
DROP = dict(color=ACID["color"], opacity=0.90, roughness=0.05, ior=1.33)

# 药匙上粉末效果（d2s BUILTIN）：勺尖上方小粉堆（隐藏初始位 = 药匙家用，task 每帧跟随勺尖）
SPOON_POWDER_R = 0.005
SPOON_POWDER_H = 0.005
SPOON_POWDER_T = (0.659, 0.3209, 0.965)

# 管内白色固体样品粉末柱（⑬ 倒粉后显示，同 d2s TubeSample 单色白）。
# 2026-08-26 用户"试管还没摇晃时看不到粉末沉淀"：粉末从管底 0.806 起堆 12mm（0.806..0.818），
# 贴管底平铺——旧 0.84 悬在酸液柱中部，白粉被不透明酸柱完全盖住。酸液柱改从粉末顶面
# （LIQUID_BASE=0.818）往上长，白粉始终露在酸液下方，倒粉后立即可见。
TUBE_SAMPLE_R = 0.006
TUBE_SAMPLE_H = 0.012
LIQUID_BASE = TUBE_BOTTOM + TUBE_SAMPLE_H   # 0.818：酸液柱底面 = 白粉顶面（粉不被酸盖）
TUBE_SAMPLE_T = (TUBE_CX, TUBE_CY, TUBE_BOTTOM + TUBE_SAMPLE_H / 2)   # 中心 0.812

# 药粉下落效果（task._step_powder_anim 驱动，d2s 同款）
POWDER_DROPS = 14
POWDER_DROP_R = 0.003
POWDER_DROP_COLOR = (0.93, 0.93, 0.94)

# ========== 滴加酸后液体变色（d3l 同款，坐标 → D3-S 试管）==========
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
TUBE_COLOR_R = 0.0086

# ========== 气泡方案（d3l 同款：Ø4.4mm ×40 池，颜色跟随液体变色）==========
BUBBLE_GROUPS = {
    "clear":  dict(color=(0.72, 0.85, 1.0), opacity=1.0, roughness=0.3, emissive=(0.7, 1.0, 1.8)),
    "red":    dict(color=(0.05, 0.02, 0.02), opacity=1.0, roughness=0.3, emissive=(2.6, 0.12, 0.12)),
    "blue":   dict(color=(0.02, 0.04, 0.10), opacity=1.0, roughness=0.3, emissive=(0.15, 0.45, 2.6)),
    "green":  dict(color=(0.02, 0.10, 0.04), opacity=1.0, roughness=0.3, emissive=(0.15, 2.4, 0.15)),
    "purple": dict(color=(0.10, 0.02, 0.12), opacity=1.0, roughness=0.3, emissive=(2.3, 0.18, 2.6)),
}
BUBBLE_R = 0.0022


def _gen_bubbles(n_center=30, n_wall=10, seed=42, cx=TUBE_CX, cy=TUBE_CY):
    """生成 40 个基准位（固定种子可复现）：中心盘状区 r≤0.0035 + 近管壁环 0.0055~0.0063。
    cx/cy = 试管孔中心（D3-S = 0.659,0.241）。z 全写 0.806（task 每帧覆盖）。"""
    rng = random.Random(seed)
    out = []
    for _ in range(n_center):
        r = 0.0035 * math.sqrt(rng.random())
        a = 2.0 * math.pi * rng.random()
        out.append((cx + r * math.cos(a), cy + r * math.sin(a), 0.806))
    for _ in range(n_wall):
        r = 0.0055 + 0.0008 * rng.random()
        a = 2.0 * math.pi * rng.random()
        out.append((cx + r * math.cos(a), cy + r * math.sin(a), 0.806))
    return out


BUBBLES = _gen_bubbles()
DROPS_PER_GROUP = 4


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


def remove_lab001_equipment(stage):
    """真正删除 lab_001 自带器材/家具（白名单外全删，含 Cabinet 离线 payload）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        name = child.GetName()
        if name in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(name)
    print(f"[remove] deleted {len(removed)} lab_001 prims: {removed}")


def raise_worktop(stage, target_top=TABLE_TOP):
    """Cube/table 顶面抬到 target_top。"""
    cube = stage.GetPrimAtPath("/World/Cube")
    if not cube.IsValid():
        print("[worktop] /World/Cube not found, skip")
        return
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    top = bc.ComputeWorldBound(cube).ComputeAlignedRange().GetMax()[2]
    delta = target_top - top
    for path in ("/World/Cube", "/World/table"):
        p = stage.GetPrimAtPath(path)
        if not p.IsValid():
            continue
        ops = UsdGeom.Xformable(p).GetOrderedXformOps()
        tr = ops[0].Get()
        ops[0].Set(Gf.Vec3d(tr[0], tr[1], tr[2] + delta))
    print(f"[worktop] surface top {top:.4f} -> {target_top:.2f} (delta {delta:+.4f})")


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot_z=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot_z is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, rot_z))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" scale {scale}" if scale else "") + (f" rotZ {rot_z}" if rot_z is not None else ""))


def _make_cyl(stage, name, r, h, t, recipe, visible):
    geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
    geom.CreateRadiusAttr(r)
    geom.CreateHeightAttr(h)
    geom.CreateAxisAttr("Z")
    geom.AddTranslateOp().Set(Gf.Vec3d(*t))
    translucent = recipe.get("opacity", 1.0) < 1.0
    add_material(stage, geom.GetPrim(), recipe["color"], recipe["opacity"],
                 roughness=recipe.get("roughness", 0.5), ior=recipe.get("ior"),
                 emissive=recipe.get("emissive"), double_sided=translucent)
    if not visible:
        UsdGeom.Imageable(geom).MakeInvisible()
    print(f"[effect] {name} {'visible' if visible else 'hidden'} at {t} (op {recipe['opacity']})")


def add_effects(stage):
    """内建效果 prim：药匙粉末 + 药粉下落 + 管内固体粉末 + 酸液柱 + 沉淀 + 浑浊云。"""
    # 药匙上粉堆（⑨ 挖粉显示，跟随勺尖）
    _make_cyl(stage, "PowderOnSpoon", SPOON_POWDER_R, SPOON_POWDER_H, SPOON_POWDER_T,
              POWDER, False)
    # 药粉下落：父 PowderDrop + N 颗粉粒（父+单粒全隐藏，task 下落动画逐颗驱动）
    drop = UsdGeom.Xform.Define(stage, "/World/PowderDrop")
    for i in range(POWDER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/PowderDrop/Drop_{i}")
        sph.CreateRadiusAttr(POWDER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(TUBE_CX, TUBE_CY, TUBE_MOUTH))
        add_material(stage, sph.GetPrim(), POWDER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(drop).MakeInvisible()
    print(f"[effect] PowderDrop hidden ({POWDER_DROPS} powder grains)")
    # 管内白色固体样品粉末（⑬ 倒粉后显示；白粉固定，反应变色由液体/气泡承载）
    _make_cyl(stage, "TubeSample", TUBE_SAMPLE_R, TUBE_SAMPLE_H, TUBE_SAMPLE_T, POWDER, False)
    # 管内酸液柱（滴入后逐滴生长，task._grow_tube_level 驱动；从白粉顶面 LIQUID_BASE 往上长）
    _make_cyl(stage, "TubeDrops", 0.009, 0.030, (TUBE_CX, TUBE_CY, LIQUID_BASE + 0.015), ACID, False)
    # 沉淀（固体颗粒沉降，贴管底）/ 浑浊云（酸加入瞬间乳白，从液柱底往上盖）（task 几何驱动）
    _make_cyl(stage, "Precipitate", 0.0088, 0.003, (TUBE_CX, TUBE_CY, TUBE_BOTTOM + 0.0015),
              OPAQUE_WHITE, False)
    _make_cyl(stage, "PrecipitateCloud", 0.0089, 0.003, (TUBE_CX, TUBE_CY, LIQUID_BASE + 0.0015),
              CLOUD_MILK, False)


def add_color_liquid(stage):
    """候选色变色液柱（滴加酸后液体变色，d3l 同款）：/World/TubeDropsInput_<色>（输入色，
    滴入后、震荡前）+ /World/TubeDropsColor_<色>（输出色，震荡反应后）。初始全隐藏、height 0；
    task 按 cfg.input_color / cfg.liquid_color 分别 show 对应一根，逐滴/逐帧改 height。"""
    for prefix in ("TubeDropsInput", "TubeDropsColor"):
        for name, m in LIQUID_COLORS.items():
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{prefix}_{name}")
            geom.CreateRadiusAttr(TUBE_COLOR_R)
            geom.CreateHeightAttr(0.0)
            geom.CreateAxisAttr("Z")
            # 底面 = 白粉顶面 LIQUID_BASE（task 逐滴/逐帧把 center 设到 LIQUID_BASE + h/2）
            geom.AddTranslateOp().Set(Gf.Vec3d(TUBE_CX, TUBE_CY, LIQUID_BASE))
            translucent = m.get("opacity", 1.0) < 1.0
            add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                         roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                         emissive=m.get("emissive"), double_sided=translucent)
            UsdGeom.Imageable(geom).MakeInvisible()
            print(f"[effect] {prefix}_{name} hidden (r={TUBE_COLOR_R})")


def add_bubbles(stage):
    """气泡组 ×5（颜色跟随液体变色）：/World/Bubbles_<色> 每组 40 颗球，初始全隐藏。"""
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
    """挤胶头滴落串：/World/DropperDrop 父 Xform + Drop_0.._N 酸色小球（r=0.003）。
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。整体初始隐藏。"""
    g = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    for i in range(DROPS_PER_GROUP):
        s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/Drop_{i}")
        s.CreateRadiusAttr(0.003)
        s.AddTranslateOp().Set(Gf.Vec3d(TUBE_CX, TUBE_CY, 0.820))
        add_material(stage, s.GetPrim(), DROP["color"], DROP["opacity"],
                     roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DropperDrop hidden ({DROPS_PER_GROUP} drop spheres)")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属药匙/玻璃件在无环境反射下反黑。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def strip_dome_lights(st2):
    """扫除 flametest 残留的嵌套 DomeLight，只保留 /World/env_light。"""
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


def remove_rack_env_lights(st2):
    """试管架/滴管架资产自带 flametest 残留 DomeLight，逐个删（两架都清）。"""
    for name in ("TestTubeRack",):
        rack = st2.GetPrimAtPath(f"/World/{name}")
        if not rack.IsValid():
            print(f"[clean] /World/{name} not found, skip")
            continue
        paths = [p.GetPath() for p in Usd.PrimRange(rack)
                 if p.GetTypeName() == "DomeLight" or "env_light" in p.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed rack light {path}")
        if not paths:
            print(f"[clean] no DomeLight in {name}")


def remove_stoppers(st2):
    """去瓶塞：盐酸瓶已开瓶，删自带的 stopper + stopper_mat（瓶口 rim 0.870）。"""
    for name in ("HClBottle",):
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


def fix_env_light(st2):
    """修 env 贴图路径断链：烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def brighten_lights(st2):
    """主光太弱：CylinderLight 2000 → 12000（药匙细金属杆/玻璃件 headless 反黑）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（d2s/d3l 同款，去试管反光）。"""
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


def brighten_spatula(stage):
    """药匙 = 普通不锈钢（银黑）：metallic 1.0 + low roughness + 深灰 diffuse（d2s 同款）。"""
    sh = stage.GetPrimAtPath("/World/Spatula/material/stainless_steel")
    if not sh.IsValid() or sh.GetTypeName() != "Shader":
        print("[spatula] material not found, skip")
        return
    ush = UsdShade.Shader(sh)
    ush.GetInput("metallic").Set(1.0)
    ush.GetInput("roughness").Set(0.45)
    ush.GetInput("diffuseColor").Set(Gf.Vec3f(0.24, 0.24, 0.27))
    ush.GetInput("emissiveColor").Set(Gf.Vec3f(0.0, 0.0, 0.0))
    print("[spatula] stainless: metallic 1.0, roughness 0.45, diffuse 0.24, emissive 0")


def cleanup_dish(stage):
    """表面皿：去 flametest 残留 env_light，粉末子集重绑皿材质（d2s 同款）。"""
    dish = stage.GetPrimAtPath("/World/SurfaceDish")
    if not dish.IsValid():
        print("[dish] not found, skip")
        return
    for child in list(dish.GetChildren()):
        if child.GetTypeName() == "DomeLight" or "env_light" in child.GetName():
            stage.RemovePrim(child.GetPath())
            print(f"[dish] removed {child.GetPath()}")
    dish_mat = stage.GetPrimAtPath("/World/SurfaceDish/_materials/dish_mat_002_002")
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


def powder(stage):
    """粉末收尾：防御性清理 + 纹理路径重定位（d2s 同款）。"""
    pw = stage.GetPrimAtPath("/World/SamplePowder")
    if not pw.IsValid():
        print("[powder] not found, skip")
        return
    to_rm = []

    def collect(p):
        for c in p.GetChildren():
            if c.GetName() in ("Object_0", "Object_2", "env_light"):
                to_rm.append(str(c.GetPath()))
            collect(c)

    collect(pw)
    for path in sorted(set(to_rm)):
        stage.RemovePrim(path)
        print(f"[powder] removed {path}")

    scene_dir = os.path.dirname(OUT)
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Shader":
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path and v.path.replace("\\", "/").startswith("./textures/"):
                base = os.path.basename(v.path.replace("\\", "/"))
                newp = os.path.relpath(os.path.join(EQ, "textures", base), scene_dir).replace("\\", "/")
                inp.Set(Sdf.AssetPath(newp))
                print(f"[powder] texture {base} -> {newp}")


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


# 瓶/滴管/试管玻璃配方：真玻璃 op 0.25 / rough 0.10 / ior 1.5（d3l 同款）
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（material:binding → shader）。"""
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
    """酸瓶玻璃透明化 + 1mm 液面盘隐藏（d3l 同款，D3-S 只有 HClBottle）。"""
    for name in ("HClBottle",):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh":
                continue
            if c.GetName() == "liquid":
                UsdGeom.Imageable(c).MakeInvisible()
                print(f"[mat] hid {c.GetPath()} (1mm liquid disc)")
            else:
                if override_bound_shader(st2, c, GLASS):
                    UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def fix_dropper_materials(st2):
    """滴管玻璃透明化（dropper.usd glass_001 op 1.0 不透明遮管内液柱）。"""
    for name in ("DropperAcid",):
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
    for name in ("DropperAcid",):
        g = st2.GetPrimAtPath(f"/World/{name}/glass_body_mesh/glass_body_mesh_001")
        if g.IsValid() and g.GetTypeName() == "Mesh":
            UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] {g.GetPath()} doubleSided")


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光（op 0.35→0.12 / ior 1.5 / rough 0.05→0.25）。"""
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
    """自检：打印各器材/效果世界 bbox；断言气泡/变色液柱不变量（纯 pxr）。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "Spatula", "SurfaceDish", "SamplePowder",
             "DropperAcid", "HClBottle",
             "PowderOnSpoon", "PowderDrop", "TubeSample", "TubeDrops",
             "Precipitate", "PrecipitateCloud", "DropperDrop"]
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:15s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 气泡组不变量：每组数量==len(BUBBLES)==task.N_BUBBLES、半径==BUBBLE_R、泡缘不插管壁、初始隐藏
    TUBE_INNER_R = 0.009
    for gname in BUBBLE_GROUPS:
        bubbles = st2.GetPrimAtPath(f"/World/Bubbles_{gname}")
        assert bubbles.IsValid(), f"Bubbles_{gname} missing"
        n = len([c for c in bubbles.GetChildren() if c.GetTypeName() == "Sphere"])
        assert n == len(BUBBLES), f"Bubbles_{gname} children {n} != len(BUBBLES)={len(BUBBLES)}"
        for i, (bx, by, bz) in enumerate(BUBBLES):
            p = st2.GetPrimAtPath(f"/World/Bubbles_{gname}/Bubble_{i}")
            assert p.IsValid(), f"Bubbles_{gname}/Bubble_{i} missing"
            r = UsdGeom.Sphere(p).GetRadiusAttr().Get()
            assert abs(r - BUBBLE_R) < 1e-9, f"{gname} Bubble_{i} r={r} != BUBBLE_R={BUBBLE_R}"
            dr = math.hypot(bx - TUBE_CX, by - TUBE_CY)
            assert dr + r <= TUBE_INNER_R, \
                f"{gname} Bubble_{i} clips wall: dr+r={dr + r:.4f} > inner {TUBE_INNER_R}"
        assert UsdGeom.Imageable(bubbles).ComputeVisibility() == "invisible", \
            f"Bubbles_{gname} should be hidden initially"
        print(f"[verify] Bubbles_{gname} OK: {n} spheres r={BUBBLE_R}, all inside tube, hidden")
    # 候选色变色液柱不变量（输入色 + 输出色两组）
    for prefix in ("TubeDropsInput", "TubeDropsColor"):
        for name in LIQUID_COLORS:
            p = st2.GetPrimAtPath(f"/World/{prefix}_{name}")
            assert p.IsValid(), f"{prefix}_{name} missing"
            r = UsdGeom.Cylinder(p).GetRadiusAttr().Get()
            assert abs(r - TUBE_COLOR_R) < 1e-9, f"{prefix}_{name} r={r} != TUBE_COLOR_R={TUBE_COLOR_R}"
            vis = UsdGeom.Imageable(p).ComputeVisibility() == "invisible"
            assert vis, f"{prefix}_{name} should be hidden initially"
    print(f"[verify] LiquidColor OK: 2x{len(LIQUID_COLORS)} tubes "
          f"(Input+Color) r={TUBE_COLOR_R} all hidden")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB001)
    raise_worktop(stage)
    remove_lab001_equipment(stage)
    for name, asset, t, scale, rot_z in EQUIP:
        add_equip(stage, name, asset, t, scale, rot_z)
    add_effects(stage)
    add_color_liquid(stage)   # 候选色变色液柱（滴加酸后液体变色，初始全隐藏）
    add_bubbles(stage)
    add_dropper_drops(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_001 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    cleanup_dish(st2)            # 表面皿去 env_light + 粉末子集重绑皿材质
    powder(st2)                  # 粉末：防御性清理 + 纹理重定位
    strip_dome_lights(st2)       # 扫除残留 DomeLight（flametest 黑贴图压暗环境）
    remove_rack_env_lights(st2)  # 试管架残留 DomeLight
    remove_stoppers(st2)         # 盐酸瓶去瓶塞
    brighten_spatula(st2)        # 药匙银黑不锈钢
    fix_bottle_materials(st2)    # 酸瓶玻璃透明化 + 1mm 液面盘隐藏
    fix_dropper_materials(st2)   # 滴管玻璃透明化
    fix_tube_material(st2)       # 试管玻璃去反光
    fix_env_light(st2)           # env 贴图路径断链 → 场景目录
    brighten_lights(st2)         # 主光 2000→12000
    set_cylinder_light_x(st2, x=-10.0)   # 主光挪远侧
    relocate_absolute_textures(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
