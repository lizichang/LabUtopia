# -*- coding: utf-8 -*-
"""生成 d2s_water_solubility.usd —— D2-S 固体样品水溶性测试场景（烘平自包含，真实器材）。

基于 lab_001.usd 副本（Usd.Stage.Open 编辑 + stage.Export 烘平，不写回 lab_001）：
- 白名单删除 lab_001 自带器材/家具（含 Cabinet 离线 http payload，一并消除加载警告）
- 抬高工作台面 table+Cube，顶面 -> 0.80（flametest 约定）
- 引用 assets/equipment/ 真实器材 + 设 translate（架高按资产 bbox 动态贴台面）
- 内建效果 prim（PowderOnSpoon/TubeSample/TubeWater，初始隐藏，task 动画驱动）

导出 stage.Export()：单层自包含，无引用弧，加载不依赖 assets/equipment/，
默认根带 lab_001 的 defaultPrim=/World（Isaac Sim 空视口问题用引用式+无 default
prim 会出现，烘平后不存在）。

布局（用户 temp_d2s.usd 实测坐标，2026-08-14 二次重排避开 Franka 底座 (0.25,0.32)；台面顶 z=0.80）：
  TestTubeRack  (0.6803, 0.3607)  工作区右侧，底座落台面
  TestTube      (0.659,  0.241)    架近侧左孔（Ø19.2×153mm，尺寸固化在 equipment；2026-08-24 用户要求从 +Y 最远侧移到最近侧孔——够得着）
  Spatula       (0.6993, 0.3608, rotZ -180°)  架中心孔，竖插（用户 2026-08-14 转了 -180°，勺头扁平面沿 X，为后续机械臂旋转铺路）
  SurfaceDish   (0.5365, 0.105)   架正后方，表面皿（粉末在皿上，舀取时药匙水平插入；2026-08-14 晚用户要求皿+粉 -X 移 15cm 给挖粉留间隙防穿模；2026-08-24 用户要求皿+粉 +Y 移 6.5cm 让⑧平移量改小、终点脱离贴底座失效区）
  SamplePowder  (0.5383, 0.0992)  表面皿上（powder.usd scale 0.4，离群废料/env_light 由 cleanup 删；随皿 -X 移 15cm、2026-08-24 +Y 6.5cm）
  WashBottle    (0.370, 0.525)   工作区近侧（远离机械臂，倒水时再取）；2026-08-25 rotZ -180°（用户「移动到 x:0.370,y:0.525 后，+Y→+X 转 90°」）：红色嘴尖朝 +X，从 +X 侧挤水

烘平后处理（单层里已是真实 prim）：
  - 保留试管架完整结构（4 角柱 + 3 层板；曾有 cleanup 按宽高比误删角柱，已移除）
  - 扫除 flametest 残留的嵌套 DomeLight（试管架/洗瓶等资产自带 color_0C0C0C.exr
    近黑贴图，把环境压暗、金属药匙无反射反黑），只留 /World/env_light
  - 表面皿去 env_light（flametest 残留光）+ 粉末子集重绑皿材质
  - 粉末（powder.usd）删离群废料 Object_0/Object_2 + env_light，纹理重定位到 equipment/textures
  - env 贴图路径烘平后重定位到场景目录（Export 会把 ./textures/ 解析到 lab_001）
  - 主光 CylinderLight 2000→12000（药匙细金属杆光照不足反黑）

用法：python scripts/gen_d2s_scene.py   （运行环境：本地 conda env 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility")
OUT = os.path.join(SCENE_DIR, "d2s_water_solubility.usd")
LAB001 = os.path.join(REPO, "assets", "scenes", "base", "lab_001", "lab_001.usd")
EQ = os.path.join(REPO, "assets", "equipment")

TABLE_TOP = 0.80
# lab_001 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (prim, asset_file, translate, scale, rot_z)   tz=None 表示动态贴台面（资产底座 min z -> 0.80）
# 坐标来源：用户 temp_d2s.usd（2026-08-14 二次重排——原布局试管架(0.3273,0.1441)与
#   Franka 底座(当时 0.05,0.17；2026-08-23 已退至 [-0.15,0.05,0.71]，以 config 为准)重叠，整组右移 +X≈0.36；Spatula 用户更新为
#   (0.6993,0.3608,rotZ -180°)，勺头扁平面沿 X 为后续机械臂旋转铺路）。
# 注意：试管/药匙相对架的偏移与上一版完全一致（整组平移），表面皿/粉末/洗瓶独立移动。
# 试管 Ø19.2×153mm 已固化进 equipment/test_tube.usd（原 Ø15 放大 1.2779），勿再场景放大
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.6803, 0.3607, None), None, None),
    ("TestTube", "test_tube.usd", (0.659, 0.241, 0.806), None, None),
    ("Spatula", "spatula.usd", (0.6993, 0.3608, 0.828), None, -180.0),
    ("SurfaceDish", "sample_dish.usd", (0.5365, 0.105, 0.80), None, None),
    ("SamplePowder", "powder.usd", (0.5383, 0.0992, 0.7988), 0.4, None),
    ("WashBottle", "wash_bottle.usd", (0.370, 0.525, 0.80), None, -180.0),  # rotZ -180°（2026-08-25 用户「移动到 x:0.370,y:0.525 后，+Y→+X 转90°」）：红色嘴尖朝 +X
]

# 内建效果 prim: (name, radius, height, translate, color, opacity)
# PowderOnSpoon 在药匙尖端（spatula tip world z=0.828+0.135=0.963，xy 随药匙新坐标）
# TubeSample/TubeWater 改为预烘焙色变体（add_tube_effect_variants），见 SOLUBILITY_COLORS
BUILTIN = [
    ("PowderOnSpoon", 0.005, 0.005, (0.6993, 0.3608, 0.965), (0.93, 0.93, 0.94), 1.0),
]

# 试管内现象效果配色（2026-08-25 用户：终端输入溶解度+液体颜色，颜色=粉末色兼溶解后液体色，默认白）。
# headless 下运行时改材质/纹理不渲染（记忆 headless-render-ignores-materials），故预烘焙各色变体，
# task 按 cfg.liquid_color 只 show 对应那个；浑浊云 Cloud 单 prim 震荡中升起/停震褪去。
# 饱和色用「近黑 diffuse + 单通道 emissive」（d3l LIQUID_COLORS 同款），CylinderLight 12000 下才不被洗白。
SOLUBILITY_COLORS = {
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
# 微溶：液体 = 输入色更浅（emissive ×0.4 + opacity 0.8 透出底部粉末）
SOLUBILITY_LIGHT = {
    c: dict(r, opacity=0.80,
            emissive=(tuple(v * 0.4 for v in r["emissive"]) if r.get("emissive") else None))
    for c, r in SOLUBILITY_COLORS.items()
}
# 浑浊云（震荡中盖满液柱，粉末悬浮）＝乳白底混入粉末色 → 浑浊带粉末颜色（用户 2026-08-25
# 「浑浊的表现都是灰色吗」→ 按输入色着色：蓝粉浑浊=淡蓝乳白、红粉=淡粉乳白…；白粉=现在的乳白灰）。
# diffuse 偏向乳白（0.4 粉 + 0.6 奶），emissive 偏向奶色（0.35 粉 + 0.65 奶）——不透明几何 headless 可见。
_CLOUD_MILK_D = (0.82, 0.80, 0.74)
_CLOUD_MILK_E = (1.0, 0.95, 0.80)
CLOUD_COLORS = {
    c: dict(
        diffuse=tuple(0.4 * r["diffuse"][i] + 0.6 * _CLOUD_MILK_D[i] for i in range(3)),
        opacity=1.0,
        roughness=0.85,
        emissive=tuple(0.35 * (r.get("emissive") or (0.9, 0.9, 0.9))[i]
                       + 0.65 * _CLOUD_MILK_E[i] for i in range(3)),
    )
    for c, r in SOLUBILITY_COLORS.items()
}
TUBE_SAMPLE_R = 0.006     # 粉末柱半径（管内，同旧 TubeSample）
TUBE_SAMPLE_H = 0.012     # 粉末柱高
TUBE_LIQUID_R = 0.007     # 液体柱半径（同旧 TubeWater）
TUBE_LIQUID_H = 0.035     # 液体柱高（洗瓶注水一次）
CLOUD_R = 0.0065          # 浑浊云半径（略小于液体柱，盖液柱不穿管壁）

# 药粉下落效果（task._step_powder_anim 驱动）：父 PowderDrop + N 颗小粉粒，仿 D2L/D3L
# DropperDrop（父 Xform + Drop_0..N 球）。⑬ 药匙竖直后粉粒从勺尖错帧坠落进试管。
POWDER_DROPS = 14            # 粉粒数（连续细粉流观感）
POWDER_DROP_R = 0.003        # 粉粒半径（同 D2L 滴球 r=0.003）
POWDER_DROP_COLOR = (0.93, 0.93, 0.94)

# 挤水水流（S4 挤水）：父 WaterStream + N 颗小水滴球（task._step_water_anim 驱动）。
# 用户「水流太粗/草率就是一个圆柱体」→ 改细水滴（比吸管 Ø3.4mm 细）沿抛物线从红嘴
# 坠入试管口（「x坐标再增大1cm」→ 终点 x=0.659=管口中心，非原 0.649 直柱）。
WATER_DROPS = 16             # 水滴池大小（round-robin 复用，task 挤水时循环发射）
WATER_DROP_R = 0.0015        # 水滴半径 1.5mm（Ø3mm，比吸管 Ø3.4mm 略细，符合物理）
WATER_DROP_COLOR = (0.35, 0.65, 0.95)   # 水蓝色（与 TubeWater 同系，稍饱和更可见）
WATER_START = (0.649, 0.231, 0.994)     # 红嘴尖（水平射出起点）
WATER_END = (0.659, 0.241, 0.9593)      # 试管口中心（水滴 home 位）


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, emissive=None,
                 double_sided=False):
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
    """Cube/table 顶面抬到 target_top（编辑 lab_001 内存副本，Export 后不写回）。"""
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
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (f" scale {scale}" if scale else "") + (f" rotZ {rot_z}" if rot_z is not None else ""))


def add_effects(stage):
    for name, r, h, t, color, opacity in BUILTIN:
        geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(h)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        add_material(stage, geom.GetPrim(), color, opacity)
        UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} hidden at {t}")
    # 药粉下落：父 PowderDrop + N 颗粉粒（父+单粒全隐藏，task 下落动画逐颗驱动）。
    # home 位放试管口（同 D2L DropperDrop 惯例），task 每帧写实际坐标。
    drop = UsdGeom.Xform.Define(stage, "/World/PowderDrop")
    for i in range(POWDER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/PowderDrop/Drop_{i}")
        sph.CreateRadiusAttr(POWDER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(0.659, 0.241, 0.9593))
        add_material(stage, sph.GetPrim(), POWDER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(drop).MakeInvisible()
    print(f"[effect] PowderDrop hidden ({POWDER_DROPS} powder grains)")
    # 挤水水流：父 WaterStream + N 颗小水滴球（父+单粒全隐藏，task 挤水时沿抛物线
    # 逐颗错帧坠落，仿 PowderDrop 队列动画）。home 位放红嘴尖 WATER_START，task 每帧写实际坐标。
    water = UsdGeom.Xform.Define(stage, "/World/WaterStream")
    for i in range(WATER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/WaterStream/Drop_{i}")
        sph.CreateRadiusAttr(WATER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(*WATER_START))
        add_material(stage, sph.GetPrim(), WATER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(water).MakeInvisible()
    print(f"[effect] WaterStream hidden ({WATER_DROPS} water drops)")


def add_tube_effect_variants(stage):
    """试管内现象效果 prim（全部初始隐藏，task 按 cfg.liquid_color/solubility 驱动）：
      TubeSample_<色>         粉末柱（输入色，⑬ 倒粉后显示）
      TubeWater               溶剂水柱（淡蓝，S4 注水后显示；不溶最终态液体）
      TubeSolution_<色>       溶解液柱（输入色全，可溶最终态）
      TubeSolutionLight_<色>  溶解液柱（输入色浅，微溶最终态）
      Cloud_<色>              浑浊云（带粉末色，震荡升起、停震褪去；三档都先浑浊）
    几何中心：粉末 0.84、液体 0.855、云 0.806（管底），xy=试管孔 (0.659,0.241)。
    """
    tx, ty = 0.659, 0.241

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

    for c, m in SOLUBILITY_COLORS.items():
        make_cyl(f"TubeSample_{c}", TUBE_SAMPLE_R, TUBE_SAMPLE_H, (tx, ty, 0.84), m)
    # 蒸馏水（S4 注水）：无色透明（用户 2026-08-25 更正「滴加的水做成无色透明的这才应该是蒸馏水」）。
    # 不用自发光淡蓝——真正蒸馏水就是透明无色；opacity 0.10 仅保留微弱液面/折射边缘感，
    # 现象全由粉末+浑浊云+溶解液变体承载（不溶终态=无色水里粉留底）
    make_cyl("TubeWater", TUBE_LIQUID_R, TUBE_LIQUID_H, (tx, ty, 0.855),
             dict(diffuse=(0.90, 0.95, 1.0), opacity=0.10, roughness=0.1, ior=1.33))
    for c, m in SOLUBILITY_COLORS.items():
        make_cyl(f"TubeSolution_{c}", TUBE_LIQUID_R, TUBE_LIQUID_H, (tx, ty, 0.855), m)
    for c, m in SOLUBILITY_LIGHT.items():
        make_cyl(f"TubeSolutionLight_{c}", TUBE_LIQUID_R, TUBE_LIQUID_H, (tx, ty, 0.855), m)
    for c, m in CLOUD_COLORS.items():
        make_cyl(f"Cloud_{c}", CLOUD_R, 0.0, (tx, ty, 0.806), m)
    print("[effect] tube variants hidden (5 powder / 1 water / 5 solution / 5 light / 5 cloud)")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属药匙在无环境反射下反黑不可见（用户
    报"看不到药匙"）。flametest 的 color_0C0C0C.exr 是 1×1 暗灰（461B），金属照
    样反黑；改用自己的亮实验室环境图 env_bright.png（天花板亮带 → 金属顶部高光，
    四周中灰、地面暗），intensity 2000 不至于过曝。

    注意：贴图路径用相对 ./textures/ 会在 stage.Export 时按 lab_001 层解析成
    不存在的 lab_001/textures/env_bright.png（断链 bug），烘平后由
    fix_env_light() 在场景层重新指向 textures/env_bright.png。
    """
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def strip_dome_lights(st2):
    """扫除 flametest 残留的嵌套 DomeLight，只保留 /World/env_light。

    试管架/洗瓶等 equipment 资产自带 env_light（贴图是 flametest 的 1×1 近黑
    color_0C0C0C.exr），烘平后全部进入场景。RTX 里多个 DomeLight 叠加/取暗，
    金属药匙的环境反射=黑 → 黑杆看不见（用户多次报"看不到药匙"）。
    从源资产删会影响 flametest，故只在 d2s 场景烘平后清理。
    """
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
    """修 env 贴图路径断链：add_env_light 的相对 ./textures/ 在 Export 时被
    解析到 lab_001 目录（该文件不存在，实际在场景目录），导致环境灯不亮、
    金属药匙无反射反黑。烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确
    指向场景目录下的 env_bright.png。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def brighten_lights(st2):
    """主光太弱：CylinderLight 强度 2000（lab_001 自带，位于台面远侧高处）照不亮
    药匙细金属杆——headless 实测杆投影区纯黑 (0,0,0)、隐藏后变化像素全暗。
    提到 12000 后药匙杆可见（max 179，全帧 mean 81 不过曝）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """CylinderLight 的 translate.x 设为绝对值（d2l/d3l 同款）：lab_001 自带灯位 x=2.1
    偏 +X 侧，正对试管/药匙近距离直射，玻璃反光强（用户 2026-08-25「反光太强看不清
    试管里面的现象」）。挪到 x=-10 远离工作区，从远处侧面打光，玻璃高光不再直射相机。
    只动 translate.x，y/z 保持。"""
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
    """药匙 = 普通不锈钢（银黑）：metallic 1.0 + low roughness + 深灰 diffuse。

    早期为救"黑杆不可见"曾降 metallic、提亮 diffuse、加 emissive 兜底——但那是
    灯光坏（CylinderLight 2000、环境贴图断链）时的补丁。灯光已修（12000 + 环境
    贴图恢复）后，那套覆写把药匙洗成纯白（emissive 0.35 自发光）。改回标准不锈钢：
    金属光泽由灯光/环境反射呈现，不需 emissive。值域与 assets/equipment/spatula.usd
    源材质一致（幂等）。
    """
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
    """表面皿：去 flametest 残留 env_light，粉末子集重绑皿材质（先不放粉末）。

    sample_dish.usd 是双材质皿：单 mesh 带 powder_mat / dish_mat 两个 GeomSubset
    （粉丘是 mesh 的一部分，flametest 残留）。真实粉末由 SamplePowder（powder.usd）
    摆在皿上，故把资产自带 powder 子集重绑到 dish 材质避免双份粉丘。
    """
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
    """粉末收尾：离群废料/ env_light 已从 powder.usd 资产本体删掉（Object_1 即粉堆），
    这里仅保留防御性清理 + 纹理路径重定位（烘平若产出相对 ./textures 会失效）。"""
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

    # 纹理重定位：./textures/x -> <scene> 相对 equipment/textures 的路径
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


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数。烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply（会告警），故直接用 material:binding relationship
    取材质路径再找 shader（照搬 d3l gen 的 override_bound_shader）。"""
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


def fix_tube_material(st2):
    """试管玻璃透明化 + 去反光（照搬 d3l fix_tube_material）：test_tube.usd 自带玻璃
    opacity 0.35 / roughness 0.05（极光滑），曲面上 CylinderLight 12000 + DomeLight 2000
    的锐利竖向高光带正好盖住管内溶解现象，用户 2026-08-25「反光太强看不清试管里面的
    现象」→ opacity 0.35→0.12 更透明、roughness 0.05→0.25 柔化反光、补 ior 1.5 真玻璃
    + doubleSided（透过玻璃看后壁，不设会漏空）。遍历 /World/TestTube 下 mesh，取
    material:binding 覆写 shader（同 d3l 瓶玻璃修法）。"""
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
    os.makedirs(SCENE_DIR, exist_ok=True)
    stage = Usd.Stage.Open(LAB001)
    raise_worktop(stage)
    remove_lab001_equipment(stage)
    for name, asset, t, scale, rot_z in EQUIP:
        add_equip(stage, name, asset, t, scale, rot_z)
    add_effects(stage)
    add_tube_effect_variants(stage)   # 试管内现象效果（TubeSample_<色>/溶液/浅溶液/云，全隐藏）
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_001 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    cleanup_dish(st2)        # 表面皿去 env_light + 粉末子集重绑皿材质
    powder(st2)              # 粉末：防御性清理 + 纹理重定位（资产本体已删废料/env_light）
    strip_dome_lights(st2)   # 扫除试管架/洗瓶等残留 DomeLight（flametest 黑贴图压暗环境）
    brighten_spatula(st2)    # 药匙 = 银黑不锈钢（metallic 1.0 + 深灰 diffuse，去 emissive 防发白）
    fix_tube_material(st2)   # 试管玻璃去反光（rough 0.05→0.25 + op 0.12 + ior 1.5，d3l 同款）
    fix_env_light(st2)       # env 贴图路径断链（Export 解析到 lab_001）→ 场景目录
    brighten_lights(st2)     # 主光 2000→12000：药匙细金属杆 headless 实测纯黑
    set_cylinder_light_x(st2, x=-10.0)   # 主光挪远侧（d2l/d3l 同款，去试管反光）
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
