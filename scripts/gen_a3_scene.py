# -*- coding: utf-8 -*-
"""生成 a3_conductivity.usd —— A3 电导率测量场景（烘平自包含，真实器材）。

基于 lab_clean.usd（lab_001 副本，台面 Cube 顶 0.80，x/y ∈ [−1,1]）：
- 白名单删除 lab_001 自带器材/家具
- 引用 assets/equipment/ 真实器材 + 设 translate/rot（架高按资产 bbox 动态贴台面，
  或按需显式 tz：天平盘顶的称量纸/粉末、平躺的玻璃棒）
- 内建效果 prim（倒粉：PowderOnDish 皿上粉堆 + PowderDrop 粉粒 + BeakerPowder 烧杯粉团；
  挤水：WaterStream 水滴 + BeakerLiquid 烧杯内液面）

布局（2026-08-28 三改 = 用户 Isaac 重摆后 scene-realign，tmp=a3_tmp.usd 为真相；
机械臂底座改 (0,0.12)，右区器材整体左移靠拢底座）：
  机器人底座      (0, 0.12)    config robot.position [0,0.12,0.71]（用户指定）
  ── 测量区 ──
  Meter          (0.4349,-0.2130) rotZ+90（电极从 +x 转向 +y/烧杯；机身 y 变长→整体 -y 挪 6cm 远离烧杯）
  SampleBeaker   (0.4120,0.0807,0.80) 直立 烧杯 样品杯（2026-08-29 用户改 beaker.usd 正立烧杯：内建
                                 rotateXYZ(-135,0,0)，直立 Ø75×高90，底座 z=0 贴台面；场景直接引用不额外旋转）
                                 T(0.4120,0.0807,0.80)，bbox 0.374..0.450 / 0.037..0.118 / 0.80..0.890
  ── 称量区（简化：无药匙/称量纸；表面皿+粉直接叠天平盘）──
  Balance        (0.3442, 0.5550) 分析天平
  SurfaceDish    表面皿 Ø60 放天平盘顶（盘顶 z=0.8475；皿底 0.8474 顶 0.8540）
  PowderOnDish   程序化粉堆圆柱 Ø22×6 贴皿顶（可 shrink 倒下，随皿 6-DOF）
  ── 水/工具区 ──
  WashBottle     (0.3536, 0.3062) 洗瓶 rotZ180（tmp 里被翻转：红嘴朝 +X，对试管架方向）
  TestTubeRack   (0.5630, 0.1989) 试管架（玻璃棒插在其中）
  GlassRod       (0.5434, 0.1995) 玻璃棒 Ø6×261 立架内（底贴台面 0.80，顶 1.061
                                 高出架顶 0.917 上 0.144 供抓取）

站间 bbox 净距（用户重摆后紧凑）：最紧 Meter~SampleBeaker ~0.039m、WashBottle~Rack ~0.055m；
最远抓点 玻璃棒顶 (0.543,0.199,1.061) 距底座 (0,0.12) 3D 0.65m ≤ 0.855m 臂展（用户
中央底座布局，全器材围绕，均达记）。电极插座到烧杯 ~0.32m（动态线缆 0.8m 内安全）。

用法：python scripts/gen_a3_scene.py   （conda env labutopia 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "a_instrument", "a3_conductivity")
OUT = os.path.join(SCENE_DIR, "a3_conductivity.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")

TABLE_TOP = 0.80
BALANCE_PAN_TOP = 0.8475     # 天平称盘顶 z（资产 bbox 顶 0.047 + 台面 0.80）
# 表面皿(Ø60)叠天平盘顶：皿本地底 z=0.0001 → 皿底 0.8474、皿顶 0.8540
DISH_TZ = BALANCE_PAN_TOP - 0.0001
DISH_TOP = DISH_TZ + 0.0066
# 皿上粉堆 = 程序化圆柱（可 shrink，仿 d2s PowderOnSpoon）：不用 powder.usd 静态资产，
# 否则倒粉时整块闪现消失、没有「倒下」现象。贴皿顶 0.854，中心 z=皿顶+h/2。
POWDER_BLOB_R = 0.011        # 粉堆半径（直径 22mm，同旧 scale 0.25 半大小粉堆宽度）
POWDER_BLOB_H = 0.006        # 粉堆高（扁平）
POWDER_ORIG_REST_Z = DISH_TOP + POWDER_BLOB_H / 2.0   # 粉堆中心 z（贴皿顶）
# ---- 倒粉效果 prim（task 运行时驱动，初始全隐藏；数值与 meta_actions/constants.py 同步）----
POWDER_DROPS = 14              # 粉粒数（同 d2s PowderDrop；constants POWDER_DROPS 必须一致）
POWDER_DROP_R = 0.003          # 粉粒半径
POWDER_DROP_COLOR = (0.93, 0.93, 0.94)   # 白粉
# 烧杯内粉末（倒粉后 task 显；扁平粉团，尺寸匹配皿上粉堆 Ø22，估位待用户目检）
BEAKER_POWDER_T = (0.412, 0.0807, 0.815)
BEAKER_POWDER_R = 0.011
BEAKER_POWDER_H = 0.005
# ---- 挤水效果 prim（task 运行时驱动，初始全隐藏；数值与 meta_actions/constants.py 同步）----
WATER_DROPS = 16                 # 水滴池大小（同 constants WATER_DROPS）
WATER_DROP_R = 0.004             # 水滴半径
WATER_DROP_COLOR = (1.0, 1.0, 1.0)      # 无色透明水（2026-08-30 用户「水的颜色应无色透明」淡蓝→白）
BEAKER_MOUTH_TOP = (0.412, 0.0807, 0.8904)   # 水落点 = 烧杯口顶中心（水滴 home 位）
# 烧杯内液面（挤水时 task 随水流上涨；淡蓝半透明圆柱，可透见杯内粉末）
BEAKER_LIQUID_R = 0.030          # 液柱半径（烧杯内径 ~Ø60，< 外 Ø76）
BEAKER_LIQUID_H0 = 0.004         # 初始液面高（≈0，几乎不可见）
BEAKER_LIQUID_T = (0.412, 0.0807, TABLE_TOP + 0.002)   # 底贴烧杯内底 0.80

# ---- 屏幕读数（2026-08-30 用户「显示屏输出」；A1 折光仪同款预烘焙贴图 + 切显）----
# 屏 = 机身 front 面板竖直矩形（screen_glass 8 点，局部 x[-0.125,0.021] y[0.097,0.099]
# z[0.028,0.088]，中心局部 (-0.052,0.098,0.058)）；rotZ90+平移后世界中心 (0.3369,-0.265,0.858)、
# 朝 -X（局部 +Y → 世界 -X）、宽 0.146（世界 y）高 0.06（世界 z）。quad 贴 front 面（局部 y 0.099
# → 世界 x 0.3359），再前移 0.4mm 防 z-fighting（A2 屏同款）。
SCREEN_C = (0.3355, -0.265, 0.858)
SCREEN_HALF_W = 0.073          # 世界 y 半宽（14.6cm）
SCREEN_HALF_H = 0.030          # 世界 z 半高（6cm）
SCREEN_TEX_MEASURING_TPL = "textures/screen_measuring_{step:02d}.png"
SCREEN_TEX_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"  # 仪器数码等宽
PROGRESS_STEPS = 16            # 测量进度条帧数（须与 meta_actions/constants.py 一致）
CONDUCTIVITY_OPTIONS = ["0.012", "0.250", "1.413", "12.88"]   # mS/cm（蒸馏水/自来水/0.01M KCl/0.1M KCl）
CONDUCTIVITY_DEFAULT = "1.413"
CONDUCTIVITY_KEY = lambda v: v.replace(".", "_")              # 1.413 → 1_413（贴图名/prim 名档位）

# ---- 机顶按钮「start」标签（2026-08-30 用户「上面可以加上一个 start」）----
# 白字小 quad 贴按钮顶面（Ø32 圆顶，世界中心 (0.3549,-0.133,0.911)），作为
# /World/Meter/start_button 的子 prim（按钮局部 z 顶 0.111），随按钮下沉动画一起动。
START_LABEL_HALF_W = 0.012     # 字面半宽 12mm（Ø32 顶内）
START_LABEL_HALF_H = 0.005     # 字面半高 5mm
START_LABEL_Z = 0.0032         # 按钮局部 z（顶 0.111 上 0.2mm 防 z-fighting）
START_LABEL_TEX = "textures/start_label.png"
# 环境贴图源（d2s 同款；DomeLight 贴图断链会整场发黑）
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")
# lab_clean 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (name, asset, translate, scale, rot_x, rot_z)
#   translate tz=None → 动态贴台面（资产底座 min z -> 0.80）
#   表面皿/粉堆显式 tz（叠天平盘）；玻璃棒显式 tz（立架内，底贴台面）
EQUIP = [
    ("Meter",         "conductivity_meter.usd",   (0.4349, -0.2130, None), None, None, 90),
    ("SampleBeaker",  "beaker.usd",              (0.4120,  0.0807, TABLE_TOP), None, None, None),
    ("Balance",       "analytical_balance.usd",   (0.3442,  0.5550, None), None, None, None),
    ("SurfaceDish",   "sample_dish.usd",          (0.3442,  0.5550, DISH_TZ), None, None, None),
    ("WashBottle",    "wash_bottle.usd",          (0.3536,  0.3062, None), None, None, 180),
    ("TestTubeRack",  "test_tube_rack.usd",       (0.5630,  0.1989, None), None, None, None),
    ("GlassRod",      "glass_rod_6x6x261.usd",    (0.5434,  0.1995, TABLE_TOP), None, None, None),
]


def remove_lab001_equipment(stage):
    """真正删除 lab_001 自带器材/家具（白名单外全删）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        name = child.GetName()
        if name in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(name)
    print(f"[remove] deleted {len(removed)} lab_clean prims: {removed}")


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot_x=None, rot_z=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot_x is not None or rot_z is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(rot_x or 0, 0, rot_z or 0))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})"
          + (f" rotX{rot_x}" if rot_x else "")
          + (f" rotZ{rot_z}" if rot_z is not None else "")
          + (f" scale {scale}" if scale else ""))


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属/不锈钢无环境反射会反黑（d2s 同款）。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, emissive=None,
                 double_sided=False):
    """给 prim 绑一个 UsdPreviewSurface 材质（d2s 同款）。"""
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


def add_effects(stage):
    """倒粉效果 prim（全隐藏，task 运行时按动画逐颗显）：父 PowderDrop + N 颗粉粒 +
    烧杯内 BeakerPowder 粉末团。"""
    # 皿上粉堆（程序化圆柱，可 shrink，仿 d2s PowderOnSpoon）：初始可见（场景预置粉末在皿上），
    # task 倒粉时 _shrink_powder_blob 随粉粒落定缩小、落完隐藏 → 有「倒下」现象。
    blob = UsdGeom.Cylinder.Define(stage, "/World/PowderOnDish")
    blob.CreateRadiusAttr(POWDER_BLOB_R)
    blob.CreateHeightAttr(POWDER_BLOB_H)
    blob.CreateAxisAttr("Z")
    blob.AddTranslateOp().Set(Gf.Vec3d(0.3442, 0.5550, POWDER_ORIG_REST_Z))
    add_material(stage, blob.GetPrim(), POWDER_DROP_COLOR, 1.0)
    print(f"[effect] PowderOnDish visible at (0.3442, 0.5550, {POWDER_ORIG_REST_Z})")
    # 药粉下落：父 PowderDrop + N 颗粉粒（父+单粒全隐藏，task._step_powder_anim 逐颗驱动）。
    # home 位放倒粉终点 POWDER_LAND，task 每帧写实际坐标。
    drop = UsdGeom.Xform.Define(stage, "/World/PowderDrop")
    for i in range(POWDER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/PowderDrop/Drop_{i}")
        sph.CreateRadiusAttr(POWDER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(*BEAKER_POWDER_T))
        add_material(stage, sph.GetPrim(), POWDER_DROP_COLOR, 1.0)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(drop).MakeInvisible()
    print(f"[effect] PowderDrop hidden ({POWDER_DROPS} powder grains)")

    # 烧杯内粉末团（倒粉后 task 显）：扁平圆盘（粉堆），白色不透明，放倾斜烧杯口内。
    bp = UsdGeom.Cylinder.Define(stage, "/World/BeakerPowder")
    bp.CreateRadiusAttr(BEAKER_POWDER_R)
    bp.CreateHeightAttr(BEAKER_POWDER_H)
    bp.CreateAxisAttr("Z")
    bp.AddTranslateOp().Set(Gf.Vec3d(*BEAKER_POWDER_T))
    add_material(stage, bp.GetPrim(), POWDER_DROP_COLOR, 1.0)
    UsdGeom.Imageable(bp).MakeInvisible()
    print(f"[effect] BeakerPowder hidden at {BEAKER_POWDER_T}")

    # 挤水水流（⑥ 挤水）：父 WaterStream + N 颗水滴球（父+单粒全隐藏，task 挤水时沿抛物线
    # 逐颗错帧发射，仿 PowderDrop）。home 位放水落点 BEAKER_MOUTH_TOP，task 每帧写实际坐标。
    water = UsdGeom.Xform.Define(stage, "/World/WaterStream")
    for i in range(WATER_DROPS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/WaterStream/Drop_{i}")
        sph.CreateRadiusAttr(WATER_DROP_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(*BEAKER_MOUTH_TOP))
        add_material(stage, sph.GetPrim(), WATER_DROP_COLOR, 0.85)
        UsdGeom.Imageable(sph).MakeInvisible()
    UsdGeom.Imageable(water).MakeInvisible()
    print(f"[effect] WaterStream hidden ({WATER_DROPS} water drops)")

    # 烧杯内液面（挤水时 task 随水流上涨）：淡蓝半透明圆柱（可透见杯内粉末），底贴烧杯内底。
    liquid = UsdGeom.Cylinder.Define(stage, "/World/BeakerLiquid")
    liquid.CreateRadiusAttr(BEAKER_LIQUID_R)
    liquid.CreateHeightAttr(BEAKER_LIQUID_H0)
    liquid.CreateAxisAttr("Z")
    liquid.AddTranslateOp().Set(Gf.Vec3d(*BEAKER_LIQUID_T))
    add_material(stage, liquid.GetPrim(), WATER_DROP_COLOR, 0.35)
    UsdGeom.Imageable(liquid).MakeInvisible()
    print(f"[effect] BeakerLiquid hidden at {BEAKER_LIQUID_T}")


# ---- 屏幕读数 + start 标签（2026-08-30 用户「显示屏输出」「上面可以加上一个 start」）----
def make_screen_textures(tex_dir):
    """用 PIL 生成导电率仪屏幕贴图 + 按钮 start 标签（labutopia conda env 有 PIL 11.3/numpy）。
    真实电导率仪屏：主读数大字（mS/cm）+ 样品温度小字 + 状态进度条（测量中红 → 完成绿）。
    读数由输入档位 CONDUCTIVITY_OPTIONS 决定，每档一张 result 贴图 screen_result_<key>.png。
    start_label.png = 按钮红底白字（机顶按钮顶面标签）。"""
    from PIL import Image, ImageDraw, ImageFont

    def font(size):
        return ImageFont.truetype(SCREEN_TEX_FONT, size)

    W, H = 640, 256
    BG = (10, 14, 24)          # 近黑蓝屏底（不发光，仅亮字/进度条自发光）
    BAR_OUT = (95, 100, 115)   # 进度条边框（仪器灰）
    GREEN = (46, 220, 90)      # 完成绿
    RED = (238, 72, 60)        # 测量红
    MAIN = (160, 250, 185)     # 主读数绿白
    TEMP = (205, 214, 224)     # 温度灰白
    bx0, by0, bx1, by1 = 24, 216, 616, 240   # 进度条（宽 592、高 24、留边 2px 边框）

    # —— measuring：红进度条 0%..100% 多帧 + "Measuring…" ——
    for i in range(PROGRESS_STEPS):
        frac = i / (PROGRESS_STEPS - 1)
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle([bx0, by0, bx1, by1], outline=BAR_OUT, width=2)
        d.rectangle([bx0 + 3, by0 + 3,
                     bx0 + 3 + int((bx1 - bx0 - 6) * frac), by1 - 3], fill=RED)
        f = font(44)
        t = "Measuring…"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 96), t, font=f, fill=(255, 255, 255))
        img.save(os.path.join(tex_dir, f"screen_measuring_{i:02d}.png"))

    # —— result：绿满进度条 + 大字 <读数> mS/cm + 小字 25.0°C，每档一张 ——
    for cond in CONDUCTIVITY_OPTIONS:
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle([bx0, by0, bx1, by1], outline=BAR_OUT, width=2)
        d.rectangle([bx0 + 3, by0 + 3, bx1 - 3, by1 - 3], fill=GREEN)
        f = font(46)
        t = f"{cond} mS/cm"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 78), t, font=f, fill=MAIN)
        f = font(28)
        t = "25.0 °C"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 174), t, font=f, fill=TEMP)
        img.save(os.path.join(tex_dir, f"screen_result_{CONDUCTIVITY_KEY(cond)}.png"))

    # —— start 标签（机顶按钮顶面白字；按钮红底 diffuse(0.85,0.12,0.1)≈(217,31,26)）——
    img = Image.new("RGB", (128, 64), (217, 31, 26))
    d = ImageDraw.Draw(img)
    f = font(36)
    t = "start"
    bb = d.textbbox((0, 0), t, font=f)
    d.text(((128 - (bb[2] - bb[0])) / 2 - bb[0], (64 - (bb[3] - bb[1])) / 2 - bb[1]),
           t, font=f, fill=(255, 255, 255))
    img.save(os.path.join(tex_dir, "start_label.png"))
    print(f"[screen] textures -> {tex_dir} ({PROGRESS_STEPS} measuring frames + "
          f"{len(CONDUCTIVITY_OPTIONS)} result {'/'.join(CONDUCTIVITY_OPTIONS)} mS/cm + start_label)")


def add_screen_tex_quad(stage, name, tex_path):
    """屏幕 mesh（竖直矩形贴合屏幕 front 面，朝 -X）+ st UV + 贴图发光材质，初始隐藏。
    贴图经 UsdUVTexture 接 emissiveColor：亮字/进度条自发光、近黑屏底不发（A1 折光仪同款）。
    屏幕宽沿世界 y（14.6cm）、高沿世界 z（6cm），中心 SCREEN_C。task 按测量状态显隐
    ScreenMeasuring（测量中）/ScreenGlow（完成读数）。"""
    cx, cy, cz = SCREEN_C
    hw, hh = SCREEN_HALF_W, SCREEN_HALF_H
    # 朝 -X 的竖直屏：viewer 从 -X 看，右 = -Y、上 = +Z → st(0,0) 左下 = (+y, 下)、
    # st(1,1) 右上 = (-y, 上)。顶点序 0..3 = 左下/右下/右上/左上（faceVarying 贴图直立不翻转）。
    pts = [
        Gf.Vec3f(cx, cy + hw, cz - hh),   # 0 左下（+y，下）
        Gf.Vec3f(cx, cy - hw, cz - hh),   # 1 右下（-y，下）
        Gf.Vec3f(cx, cy - hw, cz + hh),   # 2 右上（-y，上）
        Gf.Vec3f(cx, cy + hw, cz + hh),   # 3 左上（+y，上）
    ]
    gl = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    gl.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gl.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(gl).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array,
                                               UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mat = UsdShade.Material.Define(stage, f"/World/{name}_mat")
    sh = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03, 0.04, 0.06))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    reader = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Reader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"/World/{name}_mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_path))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(gl).Bind(mat)
    UsdGeom.Gprim(gl).CreateDoubleSidedAttr().Set(True)
    UsdGeom.Imageable(gl).MakeInvisible()
    print(f"[screen] {name} hidden (texture {tex_path})")


def add_screen_effects(stage):
    """屏幕读数 prim（初始隐藏，task._ButtonLifecycle 切换）：ScreenMeasuring_<i> 测量中进度条
    多帧（红条 0%→100%，task 测量期逐帧切显）→ ScreenGlow_<key> 完成读数（导电率，key=去小数点
    档位，task 按 cfg.conductivity 选一个）。"""
    for i in range(PROGRESS_STEPS):
        add_screen_tex_quad(stage, f"ScreenMeasuring_{i:02d}",
                            SCREEN_TEX_MEASURING_TPL.format(step=i))
    for c in CONDUCTIVITY_OPTIONS:
        add_screen_tex_quad(stage, f"ScreenGlow_{CONDUCTIVITY_KEY(c)}",
                            f"textures/screen_result_{CONDUCTIVITY_KEY(c)}.png")


def add_start_label(stage):
    """机顶按钮顶面的「start」白字小 quad（/World/Meter/start_button 子 prim，随按钮下沉一起动）。
    贴按钮顶（局部 z 顶 0.111），水平朝 +Z，白字自发光（贴图）、近黑底。"""
    hw, hh = START_LABEL_HALF_W, START_LABEL_HALF_H
    pts = [
        Gf.Vec3f(-hw, -hh, START_LABEL_Z),
        Gf.Vec3f(hw, -hh, START_LABEL_Z),
        Gf.Vec3f(hw, hh, START_LABEL_Z),
        Gf.Vec3f(-hw, hh, START_LABEL_Z),
    ]
    gl = UsdGeom.Mesh.Define(stage, "/World/Meter/start_button/start_label")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    gl.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gl.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(gl).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array,
                                               UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mat = UsdShade.Material.Define(stage, "/World/Meter/start_button/start_label_mat")
    sh = UsdShade.Shader.Define(stage, "/World/Meter/start_button/start_label_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03, 0.04, 0.06))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    reader = UsdShade.Shader.Define(stage, "/World/Meter/start_button/start_label_mat/Reader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, "/World/Meter/start_button/start_label_mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(START_LABEL_TEX))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(gl).Bind(mat)
    UsdGeom.Gprim(gl).CreateDoubleSidedAttr().Set(True)
    print(f"[screen] start_label visible on button top (texture {START_LABEL_TEX})")


# ---- post-export 修复（d2s/d3l/a2 同款） ----------------------------------------
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


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（d2s/d3l 同款）。"""
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


def fix_beaker_glass(st2, beaker_path):
    """烧杯玻璃透明化（同 B3/B2 试管配方）：beaker.usd 自带 opacity 1.0 实心不透明 + rough 0.05
    + specular 0.5 反光镜面 → 压到 opacity 0.12 真玻璃 + ior 1.5 + roughness 0.25（去曲面强反光带）
    + specular 0.0（2026-08-30 用户「烧杯还像镜子，根本不是玻璃」——原材质 specular 0.5 致镜面反光，
    强制归零；metallic 本就 0.0 非根因）+ doubleSided。"""
    p = st2.GetPrimAtPath(beaker_path)
    if not p.IsValid():
        print(f"[mat] {beaker_path} not found, skip")
        return
    # beaker.usd 结构为 /World/SampleBeaker/beaker_111x75x116/beaker_111x75x116_008（嵌套 Xform），
    # 用 PrimRange 递归找 Mesh（同 gen_b3_scene.py fix_beaker_material）。
    for c in Usd.PrimRange(p):
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.12, "ior": 1.5, "roughness": 0.25,
                                          "metallic": 0.0, "specular": 0.0}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] beaker glass {c.GetPath()} -> op 0.12 / ior 1.5 / rough 0.25 / specular 0 / doubleSided")


def fix_glass_rod(st2, rod_path):
    """玻璃棒透明化（同 E1 玻璃棒配方）：glass_rod_6x6x261.usd 自带 opacity 1.0 实心 →
    压到 opacity 0.30 / ior 1.5 / roughness 0.10（细棒保持一定透光，别全透明到看不清）。"""
    p = st2.GetPrimAtPath(rod_path)
    if not p.IsValid():
        print(f"[mat] {rod_path} not found, skip")
        return
    for c in Usd.PrimRange(p):
        if c.GetTypeName() != "Mesh":
            continue
        if override_bound_shader(st2, c, {"opacity": 0.30, "ior": 1.5, "roughness": 0.10}):
            UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
            print(f"[mat] glass rod {c.GetPath()} -> op 0.30 / ior 1.5 / rough 0.10")


def post_fix(st2):
    strip_dome_lights(st2)
    fix_env_light(st2)
    brighten_lights(st2)
    set_cylinder_light_x(st2, -10.0)
    for name in ("SampleBeaker",):
        fix_beaker_glass(st2, f"/World/{name}")
    fix_glass_rod(st2, "/World/GlassRod")


def _points_bbox(st, prim_path):
    """世界坐标 points-based 包围盒（避开 BBoxCache 的 extent 陈旧/旋转失真）。

    beaker.usd 的 mesh extent 是旋转前的局部 bbox，BBoxCache 变换后把直立烧杯误报成
    160mm 高（真身 90mm）→ 对带旋转的器材用实际 points 求世界 bbox（同 gen_b3_scene.py）。
    返回 (Gf.Vec3d min, Gf.Vec3d max)。
    """
    p = st.GetPrimAtPath(prim_path)
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


def _wbb(st, bc, path):
    p = st.GetPrimAtPath(path)
    assert p.IsValid(), f"{path} missing"
    r = bc.ComputeWorldBound(p).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def clear_gap(lo1, hi1, lo2, hi2):
    """两个 AABB 的最小净距：重叠轴贡献 0，取各轴分离量的欧氏范数。"""
    s = []
    for i in range(3):
        if hi2[i] < lo1[i]:
            s.append(lo1[i] - hi2[i])
        elif hi1[i] < lo2[i]:
            s.append(lo2[i] - hi1[i])
        else:
            s.append(0.0)
    return (sum(c * c for c in s)) ** 0.5


def verify():
    """场景世界 bbox 校验：器材就位/贴台/关键高度 + 站间净距 ≥0.35（用户要求大间隔）。"""
    st = Usd.Stage.Open(OUT)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    checks = []

    def check(name, cond):
        checks.append((name, cond))
        print(f"[verify] {name}: {'OK' if cond else 'FAIL'}")

    # ---- 就位 / 贴台 / 关键高度（世界 bbox，以左下角为准）----
    lo, hi = _wbb(st, bc, "/World/Meter")
    check("Meter 贴台面 z0.80", abs(lo[2] - 0.80) < 1e-3)
    check("Meter 左下 (0.327,-0.361)", abs(lo[0] - 0.327) < 0.01 and abs(lo[1] + 0.361) < 0.01)
    check("Meter 高 0.232", abs(hi[2] - lo[2] - 0.232) < 0.01)

    lo, hi = _points_bbox(st, "/World/SampleBeaker")
    # 2026-08-29 用户改 beaker.usd 直立烧杯：T(0.4120,0.0807,0.80) 无场景旋转，底座贴台面
    check("SampleBeaker 左下 (0.374,0.037,0.80)", abs(lo[0] - 0.374) < 0.01
          and abs(lo[1] - 0.037) < 0.01 and abs(lo[2] - 0.80) < 0.01)
    check("SampleBeaker 右上 (0.450,0.118,0.890)", abs(hi[0] - 0.450) < 0.01
          and abs(hi[1] - 0.118) < 0.01 and abs(hi[2] - 0.890) < 0.01)

    lo, hi = _wbb(st, bc, "/World/Balance")
    check("Balance 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("Balance 左下 (0.244,0.450)", abs(lo[0] - 0.244) < 0.01 and abs(lo[1] - 0.450) < 0.01)

    lo, hi = _wbb(st, bc, "/World/SurfaceDish")
    check("SurfaceDish 贴天平盘顶 0.8475", abs(lo[2] - 0.8475) < 1e-3)
    check("SurfaceDish 左下 (0.314,0.525)", abs(lo[0] - 0.314) < 0.01 and abs(lo[1] - 0.525) < 0.01)

    lo, hi = _wbb(st, bc, "/World/PowderOnDish")
    check("PowderOnDish 底贴皿顶 0.854", abs(lo[2] - 0.854) < 1e-3)
    check("PowderOnDish 左下 (0.333,0.544)", abs(lo[0] - 0.333) < 0.01 and abs(lo[1] - 0.544) < 0.01)
    check("PowderOnDish 顶 0.860 (高 0.006)", abs(hi[2] - 0.860) < 1e-3)

    lo, hi = _wbb(st, bc, "/World/WashBottle")
    check("WashBottle 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("WashBottle 左下 (0.322,0.274)", abs(lo[0] - 0.322) < 0.01 and abs(lo[1] - 0.274) < 0.01)

    lo, hi = _wbb(st, bc, "/World/TestTubeRack")
    check("TestTubeRack 贴台面", abs(lo[2] - 0.80) < 0.01)
    check("TestTubeRack 左下 (0.520,0.056)", abs(lo[0] - 0.520) < 0.01 and abs(lo[1] - 0.056) < 0.01)

    lo, hi = _wbb(st, bc, "/World/GlassRod")
    check("GlassRod 立架内 底贴台面", abs(lo[2] - 0.80) < 0.01)
    check("GlassRod 顶 1.061 (高 0.261)", abs(hi[2] - 1.061) < 0.01)

    # ---- 倒粉效果 prim（task 运行时驱动，初始隐藏）----
    drop = st.GetPrimAtPath("/World/PowderDrop")
    check("PowderDrop 存在", drop.IsValid())
    n_grains = sum(1 for c in drop.GetChildren() if c.IsA(UsdGeom.Sphere)) if drop.IsValid() else 0
    check(f"PowderDrop {POWDER_DROPS} 粉粒", n_grains == POWDER_DROPS)
    bp = st.GetPrimAtPath("/World/BeakerPowder")
    check("BeakerPowder 存在", bp.IsValid())
    ws = st.GetPrimAtPath("/World/WaterStream")
    check("WaterStream 存在", ws.IsValid())
    n_water = sum(1 for c in ws.GetChildren() if c.IsA(UsdGeom.Sphere)) if ws.IsValid() else 0
    check(f"WaterStream {WATER_DROPS} 水滴", n_water == WATER_DROPS)
    check("BeakerLiquid 存在", st.GetPrimAtPath("/World/BeakerLiquid").IsValid())

    # ---- 屏幕读数 prim（初始隐藏，带 st UV + 贴图；task 按档位切显）+ start 标签 ----
    for sname in ([f"ScreenMeasuring_{i:02d}" for i in range(PROGRESS_STEPS)]
                  + [f"ScreenGlow_{CONDUCTIVITY_KEY(c)}" for c in CONDUCTIVITY_OPTIONS]):
        sp = st.GetPrimAtPath(f"/World/{sname}")
        check(f"{sname} 存在", sp.IsValid())
        if sp.IsValid():
            st_pv = UsdGeom.PrimvarsAPI(sp).GetPrimvar("st")
            check(f"{sname} st UV", st_pv.GetAttr().IsValid())
            check(f"{sname} 初始隐藏",
                  UsdGeom.Imageable(sp).ComputeVisibility() == "invisible")
    sl = st.GetPrimAtPath("/World/Meter/start_button/start_label")
    check("start 标签存在", sl.IsValid())
    for tex in ([SCREEN_TEX_MEASURING_TPL.format(step=i) for i in range(PROGRESS_STEPS)]
                + [f"textures/screen_result_{CONDUCTIVITY_KEY(c)}.png" for c in CONDUCTIVITY_OPTIONS]
                + ["textures/start_label.png"]):
        check(f"贴图存在 {tex}", os.path.exists(os.path.join(SCENE_DIR, tex)))

    # ---- 站间净距 ≥0.02（2026-08-27 用户中央底座重摆后紧凑：Meter~Beaker ~0.03、
    #      WashBottle~Rack ~0.06 为最紧对；阈值只拦真实重叠，不拦用户有意紧凑摆放）----
    station_bbox = {
        "Meter": _wbb(st, bc, "/World/Meter"),
        "SampleBeaker": _points_bbox(st, "/World/SampleBeaker"),
        "WashBottle": _wbb(st, bc, "/World/WashBottle"),
    }
    bbs = [_wbb(st, bc, p) for p in ("/World/Balance", "/World/SurfaceDish", "/World/PowderOnDish")]
    station_bbox["WeighStation"] = (tuple(min(bb[0][i] for bb in bbs) for i in range(3)),
                                    tuple(max(bb[1][i] for bb in bbs) for i in range(3)))
    bbs = [_wbb(st, bc, p) for p in ("/World/TestTubeRack", "/World/GlassRod")]
    station_bbox["RackStation"] = (tuple(min(bb[0][i] for bb in bbs) for i in range(3)),
                                   tuple(max(bb[1][i] for bb in bbs) for i in range(3)))

    names = list(station_bbox)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            lo1, hi1 = station_bbox[names[i]]
            lo2, hi2 = station_bbox[names[j]]
            g = clear_gap(lo1, hi1, lo2, hi2)
            check(f"净距 {names[i]}~{names[j]} ≥0.01 ({g:.3f})", g >= 0.01)

    ok = all(passed for _, passed in checks)
    assert os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")), \
        "env_bright.png missing (DomeLight would be black)"
    assert ok, "A3 scene verify FAIL"
    print("[verify] all OK ->", OUT)


def main():
    os.makedirs(SCENE_DIR, exist_ok=True)
    # 环境贴图（DomeLight 断链 = 整场发黑，d2s/a2 同款）
    import shutil
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")
    make_screen_textures(os.path.join(SCENE_DIR, "textures"))

    stage = Usd.Stage.Open(LAB_CLEAN)
    remove_lab001_equipment(stage)
    for name, asset, t, scale, rx, rz in EQUIP:
        add_equip(stage, name, asset, t, scale, rx, rz)
    add_effects(stage)
    add_screen_effects(stage)
    add_start_label(stage)
    add_env_light(stage)
    stage.Export(OUT)
    print(f"[export] {OUT}")

    st2 = Usd.Stage.Open(OUT)
    post_fix(st2)
    st2.GetRootLayer().Save()
    print("[save] post-fix")

    verify()
    print("DONE")


if __name__ == "__main__":
    main()
