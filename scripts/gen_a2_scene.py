# -*- coding: utf-8 -*-
"""生成 a2_polarimeter.usd —— A2 旋光仪测量场景（烘平自包含，真实器材）。

基于 lab_clean.usd（lab_001 副本，台面 Cube 顶 0.80）：
- 白名单删除 lab_001 自带器材/家具（含 Cabinet 离线 http payload）
- 引用 assets/equipment/ 真实器材 + 设 translate（架高按资产 bbox 动态贴台面）
- 内建效果 prim：TubePowder（试管预装白粉，初始可见）+ TestTubeWater（蒸馏水，
  隐藏，洗瓶注水后显示）+ TubeLiquid（旋光管内液柱，PolarimeterTube 子 prim，
  随管移动，滴入后显示）+ WaterStream/DropperDrip（注水/滴液 16 滴水流，隐藏，
  task 逐帧写位置）+ 屏幕读数 ScreenMeasuring_<i>/ScreenGlow_<key>（贴图发光，隐藏，
  task 测量期逐帧切显进度条、完成后按档位定格旋光角读数）

导出 stage.Export()：单层自包含，无引用弧。

布局（2026-08-27 滴管转移改造：倒液 → 胶头滴管吸3次挤进加液口；台面 Cube
      顶面 2m×2m、x/y ∈ [−1,1]；台面顶 z=0.80）：
  d2s 同款三件（坐标与 gen_d2s_scene.py 一字不差，复用其已证可行包络）：
    TestTubeRack (0.6803, 0.3607)   右前，自动贴台面
    TestTube     (0.659, 0.241, 0.806)  架近侧孔（偏移架 −0.021,−0.120），预装白粉
    WashBottle   (0.370, 0.525)     rotZ −180°（红嘴朝 +X，对试管方向，同 d2s）
  A2 专用三件（全部在机械臂 +x 侧）：
    Polarimeter    (0.48, −0.24)   rotZ+90°：屏幕朝 −x（相机2 从 −x 拍）、光源 +x、
                                    lid 铰链沿世界+X（近侧 y−0.184）已掀 120°（资产内
                                    掀开态，板伸向深侧 −y），资产 min z=0 贴台面
    Dropper        (0.6993, 0.4003, 0.806)  胶头滴管立插主试管架第二列第5排
                                    （尖嘴底=架孔底 0.806，夹胶头 TCP 0.936）
    PolarimeterTube (0.5265, 0.241, 0.811)  1dm 空管桌面横放 rotZ-90°（轴 X、泡 +x、
                                    加液口 +x 端顶 0.830）；加液口 = 滴管滴液点
                                    （试管口 0.659 往 −x 10cm → 0.559）；管身伸 −x 到
                                    0.4685 清开试管 6.5cm（rotZ+90 泡 −x 会顶试管，弃用）；
                                    min z=−0.011（泡底）→贴台面 0.811

简化流程：试管预装粉 → 洗瓶加水溶解 → 胶头滴管从试管吸 3 次、每次挤进旋光管加液口
（滴管全程横夹 ORIENT_FWD，无倒液旋转）→ 旋光管放导轨（tube_rails 顶 z≈0.201 世界
1.001，管中心 1.0075；槽已后移 30mm 至开口 y −0.155..0.095）→ 屏幕读数。无浓度称量。

机器人底座 (-0.15, 0.05, 0.71) 写在 config（场景不含 robot）= d2s 同款；旋光仪
(0.48,−0.24) rotZ+90 在底座右后方（按钮 (0.30,−0.24) 0.64m/导轨 (0.51,−0.24) 0.78m，
+x/−y 方向达记，见下），旋光管 (0.5265,0.241) 与试管同 y（加液口 0.559 滴液点泡在上方，
抓点 3D 0.55m），滴管 (0.6993,0.4003) 架第二列第5排（抓点 3D 0.946m 贴 d2s 已证包络
上限，若 IK FAIL 回退列1第5排 0.659,0.4003）。无倒液走廊（不再横移倒液）。
放管下探须过屏幕/面板（世界 z~1.05-1.10）进腔室（管中心 ~1.008、腔室开口顶 1.050）。
包络提示：按钮 0.64m/导轨 0.78m（+x 主向、−y 至 −0.24），超旧 A2 0.62m 包络，
属达记项——若运行时 IK FAIL 需把旋光仪收回或再旋转朝向。

用法：python scripts/gen_a2_scene.py   （本地 conda env 有 pxr）
"""
import math
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "a_instrument", "a2_polarimeter")
OUT = os.path.join(SCENE_DIR, "a2_polarimeter.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# lab_clean 里保留的结构件/灯光/物理（其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}

# (prim, asset_file, translate, scale, rot_z)   tz=None 表示动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    # d2s 同款三件（坐标与 gen_d2s_scene.py 一字不差，复用其已证可行包络）
    ("TestTubeRack", "test_tube_rack.usd", (0.6803, 0.3607, None), None, None),
    ("TestTube", "test_tube.usd", (0.659, 0.241, 0.806), None, None),
    ("WashBottle", "wash_bottle.usd", (0.370, 0.525, None), None, -180.0),  # 红嘴朝 +X（对试管）
    # A2 专用三件（远离 d2s 三件）：旋光仪 rotZ+90（2026-08-27 用户「把机器以z为轴旋转90度」
    # = 逆时针）：屏幕朝 -x、按钮 (0.30,-0.24)、导轨 (0.51,-0.24)；滴管立插主试管架
    # 第二列第5排（同 d3s 酸滴管持握）；旋光管 rotZ-90（2026-08-27 滴管转移改造：加液口 =
    # 滴管滴液点（试管口 0.659 往 -x 10cm → 0.559），泡 +x 朝滴液点、管身伸 -x 清开试管，
    # 放导轨时管轴方向才与导轨一致——纯平移持握不转管）
    ("Polarimeter", "polarimeter.usd", (0.48, -0.24, None), None, 90.0),
    ("Dropper", "dropper.usd", (0.6993, 0.4003, 0.806), None, None),
    ("PolarimeterTube", "polarimeter_tube_1dm.usd", (0.5265, 0.241, None), None, -90.0),
]

# 试管内效果坐标基准（试管世界位置，同 EQUIP TestTube；d2s 同款）
TUBE_X, TUBE_Y, TUBE_BOT = 0.659, 0.241, 0.806
TUBE_SAMPLE_R = 0.006    # 粉末柱半径（管内）
TUBE_SAMPLE_H = 0.012    # 粉末柱高
TUBE_LIQUID_R = 0.007    # 液体柱半径
TUBE_LIQUID_H = 0.035    # 液体柱高（洗瓶注水一次）

# 旋光管内液柱（PolarimeterTube 子 prim，管局部系：管轴沿 y、泡 +y、加液口 +z）
TUBE_LIQ_R = 0.0048      # 内径 Ø10 → 略细
TUBE_LIQ_LEN = 0.10      # 管身全长（1dm 管身 116mm 内）

# 水流/液滴滴（task 动画驱动，抛物线坠入）：/World/WaterStream 洗瓶注水（16 球）、
# /World/DropperDrip 滴管滴液（16 球），父 Xform + Drop_<i> 子球，task 写逐帧位置 + 显隐。
STREAM_DROPS = 16
STREAM_DROP_R = 0.002    # Ø4mm 水滴（256px 相机 ~2px，可辨）
WATER_COLOR = (0.90, 0.95, 1.0)      # 蒸馏水（无色透明）
DRIP_COLOR = (0.85, 0.80, 0.55)      # 糖水（旋光液，同 TubeLiquid；滴管滴入加液口）

# 屏幕读数（Polarimeter 资产自带 screen_glass/screen_bezel，局部系烘焙进 mesh、无 xform；
#   前表面局部 bbox x ±0.089 / y 0.2752..0.2942 / z 0.2789..0.3396：屏幕玻璃是竖直平板
#   （局部 y 面，前表面 rotZ+90 后 = 世界 x=0.1858 竖直面），上方向 = 局部 +z。
#   rotZ+90° 后：前表面世界法线朝 −x、宽轴世界 +y、上方向 = 世界 +z（竖直，无倾斜）。
#   （2026-08-27 五改：旧 SCREEN_UP=(0.2588,0,0.9659) 把 quad 倾斜 15°，但玻璃是竖直的
#   → quad 顶缘突出机外 5mm/底缘嵌入机内 7mm = 用户报「悬空的模糊矩形」；改竖直贴合。）
#   贴图发光 quad 竖直贴合前表面（a1 同款，见 a1 add_screen_tex_quad）：
#   宽沿 W=(0,1,0)、高沿 UP=(0,0,1)。世界（Polarimeter 平移 (0.48,-0.24,0.80) rotZ+90）：
#   玻璃前表面世界 x=0.1858；quad 中心 x 向相机 −x 再偏 2mm = 0.1838（2026-08-28：旧版 quad
#   与玻璃前表面完全共面 x0.1858 → z-fighting → 用户报「悬空的模糊矩形」不是数字；偏 2mm 脱离
#   共面 + winding 反向使法线朝 −x 相机（[0,3,2,1]）→ 数字不镜像，见 add_screen_tex_quad）。
SCREEN_C = (0.48 - 0.2942 - 0.002, -0.24, 0.80 + (0.2789 + 0.3396) / 2)   # (0.1838, -0.24, 1.10925)
SCREEN_W = (0.0, 1.0, 0.0)          # 宽轴：rotZ+90 后局部 +x（宽）→ 世界 +y
SCREEN_UP = (0.0, 0.0, 1.0)         # 上方向：屏幕玻璃竖直，quad 竖直贴合（勿倾斜=悬空）
SCREEN_HW = 0.050         # 半宽 5cm（显示区 10cm 居中；勿铺满整块玻璃=贴平板，a1 同款比例）
SCREEN_HH = 0.022         # 半高 2.2cm（显示区 4.4cm ~73% 玻璃高；宽高比 2.27:1 ≈ 贴图 640×256）
# 测量进度条帧数（红条 0%→100%，task 测量期逐帧切显；须与 constants.py PROGRESS_STEPS 一致）
PROGRESS_STEPS = 16
SCREEN_TEX_MEASURING_TPL = "textures/screen_measuring_{step:02d}.png"
SCREEN_TEX_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"  # 仪器数码等宽
# 旋光角读数档位（须与 constants.py ROTATION_OPTIONS 及 config experiment_result.rotation_angle
#   .options 一致，勿单边改）：屏显读数由输入 cfg.rotation_angle 决定（headless 下运行时改材质
#   不渲染 → 按档位预烘焙 screen_result_<key>.png + ScreenGlow_<key> prim，task 按档位 show 一个）
ROTATION_OPTIONS = ["+12.5", "+8.4", "+15.0", "-3.2"]
ROTATION_DEFAULT = "+12.5"
ROTATION_KEY = lambda v: v.replace("+", "p").replace("-", "m").replace(".", "_")
SCREEN_TEX_RESULT_TPL = "textures/screen_result_{key}.png"


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, emissive=None):
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
    print(f"[remove] deleted {len(removed)} lab_clean prims: {removed}")


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
    """内建效果 prim：
      TubePowder   试管内预装白粉（可见）
      TestTubeWater 蒸馏水液柱（隐藏，洗瓶注水后 task 显示；旋光用糖水无色透明）
      TubeLiquid   旋光管内液柱（隐藏，PolarimeterTube 子 prim 随管移动，倒液后显示）
    """
    # 试管预装白粉（可见）
    pw = UsdGeom.Cylinder.Define(stage, "/World/TubePowder")
    pw.CreateRadiusAttr(TUBE_SAMPLE_R)
    pw.CreateHeightAttr(TUBE_SAMPLE_H)
    pw.CreateAxisAttr("Z")
    pw.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, TUBE_BOT + 0.034))
    add_material(stage, pw.GetPrim(), (0.93, 0.93, 0.94), 1.0, roughness=0.5)
    print(f"[effect] TubePowder visible at ({TUBE_X}, {TUBE_Y}, {TUBE_BOT + 0.034:.3f})")

    # 蒸馏水（隐藏）：无色透明（同 d2s TubeWater 约定——真正蒸馏水透明无色）
    tw = UsdGeom.Cylinder.Define(stage, "/World/TestTubeWater")
    tw.CreateRadiusAttr(TUBE_LIQUID_R)
    tw.CreateHeightAttr(TUBE_LIQUID_H)
    tw.CreateAxisAttr("Z")
    tw.AddTranslateOp().Set(Gf.Vec3d(TUBE_X, TUBE_Y, TUBE_BOT + 0.049))
    add_material(stage, tw.GetPrim(), (0.90, 0.95, 1.0), 0.10, roughness=0.1, ior=1.33)
    UsdGeom.Imageable(tw).MakeInvisible()
    print(f"[effect] TestTubeWater hidden at ({TUBE_X}, {TUBE_Y}, {TUBE_BOT + 0.049:.3f})")

    # 旋光管内液柱（隐藏，PolarimeterTube 子 prim：管局部系管轴沿 y，中心在管身）
    liq = UsdGeom.Cylinder.Define(stage, "/World/PolarimeterTube/TubeLiquid")
    liq.CreateRadiusAttr(TUBE_LIQ_R)
    liq.CreateHeightAttr(TUBE_LIQ_LEN)
    liq.CreateAxisAttr("Y")
    liq.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.001))
    # 微琥珀色（糖水），opacity 0.5 透管壁可见
    add_material(stage, liq.GetPrim(), (0.85, 0.80, 0.55), 0.55, roughness=0.1, ior=1.33)
    UsdGeom.Imageable(liq).MakeInvisible()
    print("[effect] TubeLiquid hidden (child of PolarimeterTube, local (0,0,-0.001))")

    # 水流/倒液滴（task 动画驱动，父 Xform 整体隐藏，task 逐颗写位置+显隐）
    add_streams(stage)
    # 屏幕读数（测量中红进度条多帧 + 完成旋光角读数，贴图发光，初始都隐藏）
    for i in range(PROGRESS_STEPS):
        add_screen_tex_quad(stage, f"ScreenMeasuring_{i:02d}",
                            SCREEN_TEX_MEASURING_TPL.format(step=i))
    for v in ROTATION_OPTIONS:
        add_screen_tex_quad(stage, f"ScreenGlow_{ROTATION_KEY(v)}",
                            SCREEN_TEX_RESULT_TPL.format(key=ROTATION_KEY(v)))


def add_streams(stage):
    """洗瓶注水/滴管滴液：/World/WaterStream、/World/DropperDrip 父 Xform + Drop_<i> 小球。
    整体初始隐藏；task._step_water_anim / _step_drip_anim 逐帧写 Drop_i 位置（抛物线坠入）
    并切显隐，故 Drop 初始 translate 任意（未显示前不渲染）。"""
    for parent, color in (("WaterStream", WATER_COLOR), ("DropperDrip", DRIP_COLOR)):
        g = UsdGeom.Xform.Define(stage, f"/World/{parent}")
        for i in range(STREAM_DROPS):
            s = UsdGeom.Sphere.Define(stage, f"/World/{parent}/Drop_{i}")
            s.CreateRadiusAttr(STREAM_DROP_R)
            add_material(stage, s.GetPrim(), color, 0.60, roughness=0.1, ior=1.33)
        UsdGeom.Imageable(g).MakeInvisible()
        print(f"[effect] {parent} hidden ({STREAM_DROPS} drops)")


def make_screen_textures(tex_dir):
    """用 PIL 生成旋光仪屏幕贴图（labutopia conda env 有 PIL/numpy；base python 无）。
    真实旋光仪屏（2026-08-27 调研）：α 旋光角大字 + 温度小字 + 状态进度条（测量中红 →
    完成绿）。读数 α 由输入档位 ROTATION_OPTIONS 决定，每档一张 result 贴图
    screen_result_<key>.png（text 显示该档读数 + '°'），温度固定 20.0°C。
    屏幕显示区 10cm×4.4cm → 640×256（2.5:1）。"""
    from PIL import Image, ImageDraw, ImageFont

    def font(size):
        return ImageFont.truetype(SCREEN_TEX_FONT, size)

    W, H = 640, 256
    BG = (10, 14, 24)          # 近黑蓝屏底（不发光，仅亮字/进度条自发光）
    BAR_OUT = (95, 100, 115)   # 进度条边框（仪器灰）
    GREEN = (46, 220, 90)      # 完成绿（result 满进度条，a1 同款）
    RED = (238, 72, 60)        # 测量红（measuring 进度条）
    ALPHA = (160, 250, 185)    # 主读数绿白
    TEMP = (205, 214, 224)     # 温度灰白
    bx0, by0, bx1, by1 = 24, 216, 616, 240   # 进度条（宽 592、高 24、留边 2px 边框）

    # —— measuring：红进度条 0%..100% 多帧 + "Measuring…"（task 测量期逐帧切显）——
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

    # —— result：绿满进度条 + 大字 α <档位>° + 小字 20.0 °C，每档一张（a1 同款）；
    #    quad 已缩到 10×4.4cm 居中（勿铺满整块玻璃=平板），进度条与 a1 一致 ——
    for v in ROTATION_OPTIONS:
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle([bx0, by0, bx1, by1], outline=BAR_OUT, width=2)
        d.rectangle([bx0 + 3, by0 + 3, bx1 - 3, by1 - 3], fill=GREEN)
        f = font(56)
        t = f"α {v}°"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 72), t, font=f, fill=ALPHA)
        f = font(28)
        t = "20.0 °C"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 172), t, font=f, fill=TEMP)
        img.save(os.path.join(tex_dir, f"screen_result_{ROTATION_KEY(v)}.png"))
    print(f"[screen] textures -> {tex_dir} ({PROGRESS_STEPS} measuring frames + "
          f"{len(ROTATION_OPTIONS)} result α {'/'.join(ROTATION_OPTIONS)}° green bars)")


def add_screen_tex_quad(stage, name, tex_path):
    """屏幕 mesh（倾斜矩形贴合 polarimeter 屏幕前表面）+ st UV + 贴图发光材质，初始隐藏。
    贴图经 UsdUVTexture 接 emissiveColor：屏上亮字/进度条自发光、近黑屏底不发。
    task 按测量状态显隐 ScreenMeasuring（测量中）/ScreenGlow（完成 α 读数）。"""
    cx, cy, cz = SCREEN_C
    wx, wy, wz = SCREEN_W     # 宽轴单位向量（rotZ+90 后 = 世界 +y）
    upx, upy, upz = SCREEN_UP  # 上方向单位向量（世界 +z，竖直贴合屏幕玻璃）
    hw, hh = SCREEN_HW, SCREEN_HH
    # 四角 = 中心 ± hw·W ± hh·UP（顶点序 0..3 = 左下/右下/右上/左上 → 贴图直立不翻转）
    pts = [
        Gf.Vec3f(cx - hw * wx - hh * upx, cy - hw * wy - hh * upy, cz - hw * wz - hh * upz),
        Gf.Vec3f(cx + hw * wx - hh * upx, cy + hw * wy - hh * upy, cz + hw * wz - hh * upz),
        Gf.Vec3f(cx + hw * wx + hh * upx, cy + hw * wy + hh * upy, cz + hw * wz + hh * upz),
        Gf.Vec3f(cx - hw * wx + hh * upx, cy - hw * wy + hh * upy, cz - hw * wz + hh * upz),
    ]
    gl = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    # winding 反向 [0,3,2,1] → 法线朝 −x（相机侧）：旧 [0,1,2,3] 法线 +x 朝机内，相机（−x 侧）
    # 见的是背面 → 数字镜像（用户报「模糊矩形」另一因）。反向绕序后 front face 朝相机，贴图直立。
    gl.CreateFaceVertexIndicesAttr([0, 3, 2, 1])
    gl.CreateSubdivisionSchemeAttr("none")
    # st UV（每顶点，顶点序 0..3 = 左下/右下/右上/左上 → 贴图直立不翻转）
    pv = UsdGeom.PrimvarsAPI(gl).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array,
                                               UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    # 材质：近黑 diffuse + 贴图 emissive（UsdUVTexture 读 st → emissiveColor）
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


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：金属药匙/不锈钢在无环境反射下反黑（d2s 同款）。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def strip_dome_lights(st2):
    """扫除 flametest 残留的嵌套 DomeLight，只保留 /World/env_light（d2s 同款）。"""
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
    """修 env 贴图路径断链：Export 时 ./textures/ 解析到 lab_clean 目录（d2s 同款）。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def brighten_lights(st2):
    """主光太弱：CylinderLight 强度 2000 -> 12000（d2s 同款）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def set_cylinder_light_x(st2, x=-10.0):
    """主光挪远侧 x=-10（d2s/d3l 同款）：正对试管/玻璃近距离直射反光强。"""
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
    """重写 prim 绑定材质的 shader 参数（d2s 同款，照搬 d3l）。"""
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
    """试管玻璃透明化 + 去反光（d3l/d2s 同款）：opacity 0.12、roughness 0.25、ior 1.5、
    doubleSided——否则曲面上强反光带盖住管内预装粉/液现象。"""
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


def verify():
    """场景世界 bbox 校验：器材各就各位、贴台面、无穿模（几何自检，用户目检观感）。"""
    st = Usd.Stage.Open(OUT)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])

    def wbb(path):
        p = st.GetPrimAtPath(path)
        assert p.IsValid(), f"{path} missing"
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        return r.GetMin(), r.GetMax()

    checks = []
    lo, hi = wbb("/World/Polarimeter")
    checks.append(("Polarimeter 贴台面", abs(lo[2] - 0.80) < 1e-3))
    # rotZ+90 后原 x 宽 0.375 ↔ y 宽 0.613 互换（绕 translate (0.48,-0.24) 转，中心不变）
    checks.append(("Polarimeter x 宽 ~0.613", abs((hi[0] - lo[0]) - 0.613) < 0.02))
    # 资产 x 本就不对称（side_switch 凸出 +x，rotZ+90 后凸到 ±y），中心允许 5mm 偏差
    checks.append(("Polarimeter x 中心 ~0.48", abs((lo[0] + hi[0]) / 2 - 0.48) < 0.005))
    checks.append(("Polarimeter y 宽 ~0.375", abs((hi[1] - lo[1]) - 0.375) < 0.02))

    lo, hi = wbb("/World/PolarimeterTube")
    checks.append(("PolarimeterTube 泡贴台面", abs(lo[2] - 0.80) < 1e-3))
    # rotZ-90 后局部 y±0.058（管长）→ 世界 x、局部 x±0.011（管径）→ 世界 y：中心 (0.5265,0.241)
    checks.append(("PolarimeterTube 中心 (0.5265,0.241)", abs(lo[0] + hi[0] - 1.053) < 1e-3 and abs(lo[1] + hi[1] - 0.482) < 1e-3))

    lo, hi = wbb("/World/Dropper")
    checks.append(("Dropper 尖嘴底 0.806", abs(lo[2] - 0.806) < 1e-3))
    checks.append(("Dropper 中心 (0.6993,0.4003)", abs(lo[0] + hi[0] - 1.3986) < 1e-3 and abs(lo[1] + hi[1] - 0.8006) < 1e-3))

    lo, hi = wbb("/World/TestTube")
    checks.append(("TestTube 管底 0.806", abs(lo[2] - 0.806) < 1e-3))
    checks.append(("TestTube 顶 0.959", abs(hi[2] - 0.959) < 1e-3))
    checks.append(("TestTube xy (0.659,0.241)", abs(lo[0] - 0.649) < 0.02 and abs(lo[1] - 0.231) < 0.02))

    lo, hi = wbb("/World/WashBottle")
    checks.append(("WashBottle 贴台面", abs(lo[2] - 0.80) < 0.01))
    # 抓点 (0.370,0.525) 落在瓶身上（资产几何绕自身原点偏 ~4cm，bbox 中心 ≠ translate）
    checks.append(("WashBottle 抓点 (0.370,0.525) 在瓶身上", lo[0] - 0.01 < 0.370 < hi[0] + 0.01 and lo[1] - 0.01 < 0.525 < hi[1] + 0.01))

    lo, hi = wbb("/World/TestTubeRack")
    checks.append(("TestTubeRack 贴台面", abs(lo[2] - 0.80) < 0.01))

    # 启动按钮（资产自带 /root/start_button → /World/Polarimeter/start_button）：
    # 局部 translate (0,0.18,0.253) → rotZ+90 世界 (0.30,-0.24)，顶 z=0.80+0.256=1.056
    #（task 按下下沉 5mm 到 0.248）
    btn = st.GetPrimAtPath("/World/Polarimeter/start_button")
    assert btn.IsValid(), "start_button prim missing (asset)"
    assert btn.GetTypeName() == "Cylinder", "start_button should be cylinder"
    ops = [op.GetOpName() for op in UsdGeom.Xformable(btn).GetOrderedXformOps()]
    checks.append(("start_button 有 translate op", ops == ["xformOp:translate"]))
    blo, bhi = wbb("/World/Polarimeter/start_button")
    checks.append(("start_button 顶 1.056", abs(bhi[2] - 1.056) < 0.001))
    checks.append(("start_button x 中心 0.30", abs((blo[0] + bhi[0]) / 2 - 0.30) < 0.001))

    # 水流/倒液：父 Xform 隐藏，各 16 颗 Drop 球（task 动画驱动）
    for parent in ("WaterStream", "DropperDrip"):
        g = st.GetPrimAtPath(f"/World/{parent}")
        assert g.IsValid(), f"{parent} missing"
        # Drop_<i> 球 + Drop_<i>_mat 材质同为父 Xform 子 prim，只数 Sphere 类型
        nd = sum(1 for c in g.GetChildren() if c.GetTypeName() == "Sphere")
        checks.append((f"{parent} 16 滴", nd == STREAM_DROPS))
        checks.append((f"{parent} 隐藏", UsdGeom.Imageable(g).ComputeVisibility() == "invisible"))

    # 屏幕读数：测量中 ScreenMeasuring_<i> 16 帧 + 完成 ScreenGlow_<key> 4 档，
    # 初始都隐藏、各带 st UV + 贴图，quad 贴合屏幕前表面 (SCREEN_C, 半宽 7.5cm)
    for sname in ([f"ScreenMeasuring_{i:02d}" for i in range(PROGRESS_STEPS)]
                  + [f"ScreenGlow_{ROTATION_KEY(v)}" for v in ROTATION_OPTIONS]):
        sp = st.GetPrimAtPath(f"/World/{sname}")
        assert sp.IsValid(), f"{sname} missing"
        assert UsdGeom.Imageable(sp).ComputeVisibility() == "invisible", f"{sname} should be hidden"
        sr = bc.ComputeWorldBound(sp).ComputeAlignedRange()
        # rotZ+90 后宽沿世界 +y：max y = SCREEN_C[1] + 半宽
        assert abs(sr.GetMax()[1] - (SCREEN_C[1] + SCREEN_HW)) < 0.002, f"{sname} width off"
        assert abs((sr.GetMax()[0] + sr.GetMin()[0]) / 2 - SCREEN_C[0]) < 0.004, f"{sname} x off"
        assert abs((sr.GetMax()[2] + sr.GetMin()[2]) / 2 - SCREEN_C[2]) < 0.004, f"{sname} z off"
        st_pv = UsdGeom.PrimvarsAPI(sp).GetPrimvar("st")
        assert st_pv.GetAttr().IsValid(), f"{sname} st UV primvar missing"
    for tex in ([SCREEN_TEX_MEASURING_TPL.format(step=i) for i in range(PROGRESS_STEPS)]
                + [SCREEN_TEX_RESULT_TPL.format(key=ROTATION_KEY(v)) for v in ROTATION_OPTIONS]):
        assert os.path.exists(os.path.join(SCENE_DIR, tex)), f"screen texture missing: {tex}"
    assert os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")), \
        "env_bright.png missing (DomeLight would be black)"

    ok = True
    for name, passed in checks:
        print(f"[verify] {name}: {'OK' if passed else 'FAIL'}")
        ok = ok and passed
    assert ok, "A2 scene verify FAIL"
    print(f"[verify] all OK -> {OUT} "
          f"(按钮顶1.056/水流倒液各{STREAM_DROPS}滴隐藏/"
          f"屏幕{PROGRESS_STEPS + len(ROTATION_OPTIONS)}张隐藏+st UV+贴图存在)")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")
    make_screen_textures(os.path.join(SCENE_DIR, "textures"))

    stage = Usd.Stage.Open(LAB_CLEAN)
    remove_lab001_equipment(stage)
    for name, asset, t, scale, rot_z in EQUIP:
        add_equip(stage, name, asset, t, scale, rot_z)
    add_effects(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    strip_dome_lights(st2)   # 扫除器材残留 DomeLight（flametest 黑贴图压暗环境）
    fix_tube_material(st2)   # 试管玻璃去反光（管内预装粉/液可见）
    fix_env_light(st2)       # env 贴图路径断链 → 场景目录
    brighten_lights(st2)     # 主光 2000→12000
    set_cylinder_light_x(st2, x=-10.0)   # 主光挪远侧（去试管反光）
    st2.GetRootLayer().Save()
    verify()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
