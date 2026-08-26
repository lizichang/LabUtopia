# -*- coding: utf-8 -*-
"""生成 a1_refractometer.usd —— A1 折光率测量（折光仪）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材：阿贝折光仪（abbemat_advanced，**棱镜盖保留**——资产已内置
  -50° 掀开态，直接滴样不用掀盖、末步「合盖」由 task 驱动；屏幕朝前）、待测液体样品瓶
  （sample_bottle，**保留瓶塞**——取瓶/归瓶步需机械臂拔/盖）、试管架 + 胶头滴管（滴管插架左孔）。
- 布局（2026-08-25 重排：A1 简化去掉「掀盖」+「移到操作位」两步，各器件拉开间距、离机械臂
  底座 ≥0.27m；机械臂底座 (0.25,0.57,0.71) 在后 +Y、前 = -Y、臂展 0.855m、底座 y 失效区 <0.15m）：
      折光仪 (0.30,0.00) 中央（棱镜朝顶、屏幕朝 -y=前方）
      样品瓶 (0.10,0.34) 折光仪左侧（瓶身 Ø36、口 rim 0.870、白塞 0.868..0.879，距底座 0.27m）
      试管架 (0.55,0.20) 右侧；滴管插左孔 (0.531,0.1996,0.806)（距底座 0.47m）
- 去资产自带 env_light 残留（重复 DomeLight）；CylinderLight 2000→12000；样品瓶玻璃透明化
  （bottle 真玻璃，**stopper 保持白盖不透明**）；滴管玻璃透明化（管内液柱透出）。
- 内建效果 prim（task 动画驱动）：瓶内液面 SampleLiquid（可见）、棱镜液滴 PrismDrop（隐藏）、
  滴管尖液柱 DropperFill（隐藏）、挤胶头滴落球 DropperDrop（隐藏）、屏幕读数
  ScreenMeasuring（测量中红进度条）/ScreenGlow（完成 nD 读数，贴图发光，初始均隐藏）。

用法：python scripts/gen_a1_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "a_instrument", "a1_refractometer")
OUT = os.path.join(SCENE_DIR, "a1_refractometer.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 布局锚点（世界坐标，米，Z-up）——
# 机械臂底座 (0.25,0.57,0.71) 在后 +Y，向前 -Y 够工作区；臂展 0.855m，底座 y 失效区 <0.15m。
# 折光仪：asset min z=0（机身底/脚贴原点）→ tz=None 贴台面 0.80。棱镜朝顶、盖已内置 -50° 掀开态、屏幕朝 -y。
#   机身 body  x ±0.1125 y -0.165..+0.165 z 0..0.115
#   棱镜 prism 局部 (0,0.110,0.1165) 顶 0.1175 → 世界 (0.30,0.11,0.9175)
#   测量键 start_button 局部 (0,0.05,0.115..0.121) → 世界 (0.30,0.05,0.915..0.921)：
#     棱镜正前方 -y 6cm、机顶凸起 6mm 的红色测量键（2026-08-25 加，机械臂滴样后按下触发测量）
REFRACT_X, REFRACT_Y = 0.30, 0.00
PRISM_CY = REFRACT_Y + 0.110          # 棱镜世界 y 中心（滴样落点，距底座 0.46m）

# 样品瓶：asset min z=0 → 贴台面。口 rim 0.870，白塞 0.868..0.879（保留，机械臂拔/盖）。
# 距底座 0.27m（>0.15 失效区），折光仪左侧留 7cm 间隙。
BOTTLE_X, BOTTLE_Y = 0.10, 0.34

# 试管架（放滴管）：asset min z=-0.0965 → 架原点 z=0.8965。孔心列 x=±0.019 中排 y=-0.0004，
# 底层板顶(孔底) z=-0.0905 → 世界 0.806。滴管插左孔。距底座 0.47m，折光仪右侧留 9.5cm 间隙。
RACK_X, RACK_Y = 0.55, 0.20
RACK_Z = TABLE_TOP + 0.0965          # 0.8965 架原点
HOLE_Z = RACK_Z - 0.0905             # 0.806 孔底
DROPPER_XY = (RACK_X - 0.019, RACK_Y - 0.0004)   # (0.531, 0.1996)

# (prim, asset_file, translate, scale, rot180)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("Refractometer", "abbemat_advanced.usd", (REFRACT_X, REFRACT_Y, None), None, False),
    ("SampleBottle", "sample_bottle.usd", (BOTTLE_X, BOTTLE_Y, None), None, False),
    ("TestTubeRack", "test_tube_rack.usd", (RACK_X, RACK_Y, None), None, False),
    ("Dropper", "dropper.usd", (DROPPER_XY[0], DROPPER_XY[1], HOLE_Z), None, False),
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


# ---- A1 效果 prim 材质配方（待测有机液，淡琥珀，nD≈1.40）----
SAMPLE = dict(color=(0.95, 0.90, 0.72), opacity=0.75, roughness=0.08, ior=1.40)  # 瓶内液面 / 棱镜液滴
FILL = dict(color=(0.92, 0.80, 0.50), opacity=0.90, roughness=0.05, ior=1.40)    # 滴管尖内吸起液柱
DROP = dict(color=(0.92, 0.80, 0.50), opacity=0.90, roughness=0.05, ior=1.40)    # 挤胶头滴落液滴
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)

# 瓶内液面（(BOTTLE_X,BOTTLE_Y) 台面 0.80..液面 0.840 = 半瓶；Ø36 内 Ø~34）
BOTTLE_LIQ_R = 0.014
BOTTLE_LIQ_H = 0.040            # 0.80..0.84
BOTTLE_LIQ_CZ = TABLE_TOP + BOTTLE_LIQ_H / 2      # 0.820

# 棱镜液滴（落在棱镜面，初始隐藏；棱镜世界中心 (0.30,0.11) 顶 z 0.9175）
PRISM_TOP_Z = TABLE_TOP + 0.1175                  # 0.9175
PRISM_DROP_R = 0.009
PRISM_DROP_H = 0.0012
PRISM_DROP_CZ = PRISM_TOP_Z + PRISM_DROP_H / 2    # 0.9181

# 滴管尖内吸起液柱（截锥，translate=尖嘴 0.806；同 d3l/d4l 约定）
FILL_R = (0.001, 0.0035)
FILL_H = 0.060

# 挤胶头滴落串：一次挤 DROPS_PER_GROUP 滴连续坠落
DROPS_PER_GROUP = 4
DROP_BALL_R = 0.003
DROP_HOME = (REFRACT_X, PRISM_CY, PRISM_TOP_Z + 0.02)  # 棱镜上方，task 动画才写实际坐标

# 屏幕读数（屏幕局部中心 (0,-0.149,0.0537)，后倾 ~19.5°；按测量状态显隐）
SCREEN_C = (REFRACT_X, REFRACT_Y - 0.156, TABLE_TOP + 0.0537)   # (0.30,-0.156,0.8537)
SCREEN_UP = (0.0, 0.3335, 0.9427)   # 屏幕"向上"单位向量（底前 0.0081→顶后 0.0993）
# 屏幕贴图（PIL 生成，见 make_screen_textures；labutopia env 有 PIL/numpy）：屏幕 10cm×4cm → 640×256（2.5:1）。
#   测量中进度条按 PROGRESS_STEPS 预烘焙成多帧 screen_measuring_<i>.png（红条 0%→100% +
#   "Measuring…"）。headless 下运行时改材质/贴图不渲染 → 每帧一个 prim，task 测量期逐帧切显
#   （用户 2026-08-26：先显示进度条，~4s 走完最后显示结果）。
#   screen_result_<key>.png 完成读数：绿满进度条 + 大字 nD <档位> + 小字 20.0°C。贴图经
#   UsdUVTexture 接 emissiveColor → 屏上亮字/进度条自发光、近黑屏底不发。
PROGRESS_STEPS = 16             # 测量进度条帧数（0%..100%，每帧 ~0.25s@240帧/4s；须与 constants.py 一致）
SCREEN_TEX_MEASURING_TPL = "textures/screen_measuring_{step:02d}.png"
SCREEN_TEX_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"  # 仪器数码等宽
# 折光率读数档位（须与 catalogue/.../meta_actions/constants.py N_D_OPTIONS 及
# config/level2_A1Refractometer.yaml experiment_result.n_d.options 一致，勿单边改）：
# 屏幕 nD 读数由输入 cfg.n_d 决定（d3l 同款——headless 下运行时改材质不渲染，故按档位
# 预烘焙贴图 screen_result_<key>.png + 屏幕 prim ScreenGlow_<key>，task 按档位 show 一个）。
N_D_OPTIONS = ["1.3300", "1.3610", "1.4000", "1.4600"]   # 常见液体折射率（水/乙醇/琥珀液/高折射）
N_D_DEFAULT = "1.4000"
N_D_KEY = lambda v: v.replace(".", "_")                  # 1.4000 → 1_4000（贴图名/prim 名档位）

# —— A1 合盖圆盘（重建，2026-08-25；2026-08-26 方板改圆形）——
# 实测 2026-08-25：资产自带 cover 是坏件（世界 bbox y 0.277..0.310 / z 0.843..0.874，
# 悬在机器后方空中、离棱镜 0.18m；rotateX -65→0 会甩到 z≈1.05 不会合平）。
# 真实 Abbemat 棱镜盖 = 圆形磁吸样品盖（Anton Paar「Magnetic Sample Cover」，O 型圈
# 密封、磁吸快速取放、可合盖测量），非方形翻板（用户 2026-08-26 视频里看着像方形）。
# → 在 well 后沿重建真铰链圆盖：铰链 = X 轴过 (0.30, 0.127, 0.9215)（well 后沿顶、
#   well bbox y 0.093..0.127，圆形 well 中心 (0.30,0.11) 半径 0.017）。
#   圆盘 Ø34 覆盖 well（圆心在铰链 -y 17mm，圆盘后缘贴铰链、前缘 -y 34mm）。
#   掀开态 rotateX=-55（立起后仰，front rim 抬到 z≈0.950）；CloseCoverPass 推 -y → task
#   把 rotateX 平滑转到 0 = 合平盖住 well。旧坏 cover 后处理隐藏（hide_old_cover）。
COVER_HINGE = (REFRACT_X, 0.127, TABLE_TOP + 0.1215)   # (0.30, 0.127, 0.9215)
COVER_OPEN_ANGLE = -55.0                               # 掀开态 rotateX（后仰）
COVER_HALF_W = 0.017        # 圆盖半径（= well 半径，Ø34：x ±0.017 / y 中心铰链 -y 0.017）
COVER_THK = 0.002           # 盘厚 2mm
COVER_MAT = (0.55, 0.56, 0.60)   # 与资产 cover_mat 同灰（浅仪器灰）


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


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。
    16 段圆周 + 底/顶 cap，subdivisionScheme=none。"""
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


def add_dropper_drops(stage):
    """挤胶头滴落串：/World/DropperDrop 父 Xform + Drop_0.._N 琥珀小球。整体初始隐藏，
    task._on_drop 每次挤生成一串、_step_drop_anim 逐滴错帧坠落。"""
    g = UsdGeom.Xform.Define(stage, "/World/DropperDrop")
    for i in range(DROPS_PER_GROUP):
        s = UsdGeom.Sphere.Define(stage, f"/World/DropperDrop/Drop_{i}")
        s.CreateRadiusAttr(DROP_BALL_R)
        s.AddTranslateOp().Set(Gf.Vec3d(*DROP_HOME))
        add_material(stage, s.GetPrim(), DROP["color"], DROP["opacity"],
                     roughness=DROP["roughness"], ior=DROP["ior"], double_sided=True)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] DropperDrop hidden ({DROPS_PER_GROUP} drop spheres)")


def add_cover(stage):
    """A1 合盖圆盘（重建真铰链，真实 Abbemat 是圆形磁吸盖）：/World/Refractometer/Cover
    铰链在 well 后沿 (0.30,0.127,0.9215)，X 轴。圆盘 Ø34（半径 COVER_HALF_W、厚 2mm）
    圆心在铰链 -y 17mm → 圆盘后缘贴铰链、覆盖 well（x ±0.017 / y 0.093..0.127）。
    掀开态 rotateX=-55（立起后仰）；CloseCoverPass 推 -y 后 task 把 rotateX 平滑转 0 = 合平。
    旧资产坏 cover（/World/Refractometer/cover）由 fix_old_cover 后处理隐藏。"""
    cov = UsdGeom.Xform.Define(stage, "/World/Refractometer/Cover")
    cov.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.127, 0.1215))   # 铰链点（相对机原点）
    cov.AddRotateXOp().Set(COVER_OPEN_ANGLE)                 # 掀开态
    # 圆盘 mesh（UsdGeom.Cylinder，axis=Z 平盘；半径/高直接写 attrs，无 scale，
    #   天然避免 Cube 版 xformOpOrder scale 先平移后缩放吃位移的坑）。
    disc = UsdGeom.Cylinder.Define(stage, "/World/Refractometer/Cover/Disc")
    disc.CreateRadiusAttr(COVER_HALF_W)
    disc.CreateHeightAttr(COVER_THK)
    disc.CreateAxisAttr("Z")
    disc.AddTranslateOp().Set(Gf.Vec3d(0.0, -COVER_HALF_W, COVER_THK / 2.0))
    add_material(stage, disc.GetPrim(), COVER_MAT, 1.0, roughness=0.4)
    print(f"[cover] new hinged round cover (Ø{2*COVER_HALF_W*1000:.0f}mm) at well back "
          f"(hinge 0.127, open {COVER_OPEN_ANGLE} deg)")


def make_screen_textures(tex_dir):
    """用 PIL 生成折光仪屏幕贴图（labutopia conda env 有 PIL 11.3/numpy；base python 无）。
    真实 Abbemat 屏（2026-08-26 调研）：nD 主读数大字 + 样品温度小字 + 状态进度条
    （测量中红色 → 完成变绿）。读数 nD 由输入档位 N_D_OPTIONS 决定，每档一张 result 贴图
    screen_result_<key>.png（text 显示该档读数），温度固定 20.0°C。屏幕 10cm×4cm → 640×256（2.5:1）。"""
    from PIL import Image, ImageDraw, ImageFont

    def font(size):
        return ImageFont.truetype(SCREEN_TEX_FONT, size)

    W, H = 640, 256
    BG = (10, 14, 24)          # 近黑蓝屏底（不发光，仅亮字/进度条自发光）
    BAR_OUT = (95, 100, 115)   # 进度条边框（仪器灰）
    GREEN = (46, 220, 90)      # 完成绿
    RED = (238, 72, 60)        # 测量红
    ND = (160, 250, 185)       # 主读数绿白
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

    # —— result：绿满进度条 + 大字 nD <档位读数> + 小字 20.0°C，每档一张 ——
    for n_d in N_D_OPTIONS:
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle([bx0, by0, bx1, by1], outline=BAR_OUT, width=2)
        d.rectangle([bx0 + 3, by0 + 3, bx1 - 3, by1 - 3], fill=GREEN)
        f = font(54)
        t = f"nD {n_d}"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 76), t, font=f, fill=ND)
        f = font(28)
        t = "20.0 °C"
        bb = d.textbbox((0, 0), t, font=f)
        d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 174), t, font=f, fill=TEMP)
        img.save(os.path.join(tex_dir, f"screen_result_{N_D_KEY(n_d)}.png"))
    print(f"[screen] textures -> {tex_dir} ({PROGRESS_STEPS} measuring frames + "
          f"{len(N_D_OPTIONS)} result nD{'/'.join(N_D_OPTIONS)} green bars)")


def add_screen_tex_quad(stage, name, tex_path):
    """屏幕 mesh（倾斜矩形贴合屏幕前表面）+ st UV + 贴图发光材质，初始隐藏。
    贴图经 UsdUVTexture 接 emissiveColor：屏上亮字/进度条自发光、近黑屏底不发。
    task 按测量状态显隐 ScreenMeasuring（测量中）/ScreenGlow（完成 nD 读数）。"""
    cx, cy, cz = SCREEN_C
    upx, upy, upz = SCREEN_UP
    hw, hh = 0.05, 0.02          # 半宽 5cm / 半高 2cm
    pts = [
        Gf.Vec3f(cx - hw, cy - hh * upy, cz - hh * upz),
        Gf.Vec3f(cx + hw, cy - hh * upy, cz - hh * upz),
        Gf.Vec3f(cx + hw, cy + hh * upy, cz + hh * upz),
        Gf.Vec3f(cx - hw, cy + hh * upy, cz + hh * upz),
    ]
    gl = UsdGeom.Mesh.Define(stage, f"/World/{name}")
    gl.CreatePointsAttr(pts)
    gl.CreateFaceVertexCountsAttr([4])
    gl.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
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


def add_a1_effects(stage):
    """内建效果 prim：瓶内液面（可见）+ 棱镜液滴（隐藏）+ 滴管尖液柱（隐藏）
    + 屏幕读数发光（隐藏）。"""
    # 样品瓶内液面：淡琥珀半透明柱（瓶底 0.80..液面 0.840 = 半瓶），可见（吸液源）
    sl = UsdGeom.Cylinder.Define(stage, "/World/SampleLiquid")
    sl.CreateRadiusAttr(BOTTLE_LIQ_R)
    sl.CreateHeightAttr(BOTTLE_LIQ_H)
    sl.CreateAxisAttr("Z")
    sl.AddTranslateOp().Set(Gf.Vec3d(BOTTLE_X, BOTTLE_Y, BOTTLE_LIQ_CZ))
    add_material(stage, sl.GetPrim(), SAMPLE["color"], SAMPLE["opacity"],
                 roughness=SAMPLE["roughness"], ior=SAMPLE["ior"], double_sided=True)
    print(f"[effect] SampleLiquid visible (bottle {BOTTLE_LIQ_H:.3f}m to top 0.840)")

    # 棱镜液滴：淡琥珀薄圆盘落在棱镜面（顶 0.9175），初始隐藏，滴样后显示
    pd = UsdGeom.Cylinder.Define(stage, "/World/PrismDrop")
    pd.CreateRadiusAttr(PRISM_DROP_R)
    pd.CreateHeightAttr(PRISM_DROP_H)
    pd.CreateAxisAttr("Z")
    pd.AddTranslateOp().Set(Gf.Vec3d(REFRACT_X, PRISM_CY, PRISM_DROP_CZ))
    add_material(stage, pd.GetPrim(), SAMPLE["color"], SAMPLE["opacity"],
                 roughness=SAMPLE["roughness"], ior=SAMPLE["ior"], double_sided=True)
    UsdGeom.Imageable(pd).MakeInvisible()
    print(f"[effect] PrismDrop hidden on prism top {PRISM_TOP_Z:.4f}")

    # 滴管尖内吸起液柱（截锥 mesh，底心贴尖嘴 0.806）：初始隐藏，吸液后 task 跟随尖嘴
    fill = add_frustum(stage, "DropperFill", FILL_R[0], FILL_R[1], FILL_H)
    fill.AddTranslateOp().Set(Gf.Vec3d(DROPPER_XY[0], DROPPER_XY[1], HOLE_Z))
    add_material(stage, fill.GetPrim(), FILL["color"], FILL["opacity"],
                 roughness=FILL["roughness"], ior=FILL["ior"], double_sided=True)
    UsdGeom.Imageable(fill).MakeInvisible()
    print(f"[effect] DropperFill frustum hidden at tip (r {FILL_R} h {FILL_H})")

    # 屏幕读数（初始隐藏，task._ButtonLifecycle 切换）：ScreenMeasuring_<i> 测量中进度条
    # 多帧（红条 0%→100%，task 测量期逐帧切显）→ ScreenGlow_<key> 完成读数（绿满条 + 对应
    # 档位 nD，task 按 cfg.n_d 选一个）
    for i in range(PROGRESS_STEPS):
        add_screen_tex_quad(stage, f"ScreenMeasuring_{i:02d}",
                            SCREEN_TEX_MEASURING_TPL.format(step=i))
    for n_d in N_D_OPTIONS:
        add_screen_tex_quad(stage, f"ScreenGlow_{N_D_KEY(n_d)}",
                            f"textures/screen_result_{N_D_KEY(n_d)}.png")


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
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效）。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去器材资产自带的 flametest 残留 DomeLight（/root/env_light）。"""
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


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数（烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply，直接用 material:binding relationship 取材质路径）。"""
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
    """样品瓶玻璃透明化（磨砂 op0.8 → 真玻璃 op0.25 + ior 1.5 + doubleSided），
    瓶内 SampleLiquid 液面才透得出来。**只改 bottle mesh，stopper 保持白盖不透明**（
    取瓶/归瓶步机械臂要拔/盖塞，塞是实体盖不是玻璃）。"""
    p = st2.GetPrimAtPath("/World/SampleBottle")
    if not p.IsValid():
        print(f"[mat] /World/SampleBottle not found, skip")
        return
    for c in p.GetChildren():
        if c.GetTypeName() != "Mesh":
            continue
        if c.GetName() == "bottle":
            if override_bound_shader(st2, c, GLASS):
                UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)
        else:
            print(f"[mat] {c.GetPath()} ({c.GetName()}) kept as-is (stopper stays opaque)")


def fix_dropper_materials(st2):
    """滴管玻璃透明化：dropper.usd 的 glass_001 是 opacity=1.0 不透明光面，
    改成真玻璃 op 0.25（同瓶玻璃配方），管内 DropperFill 液柱才透得出来；胶头保持不透明。"""
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
    g = st2.GetPrimAtPath("/World/Dropper/glass_body_mesh/glass_body_mesh_001")
    if g.IsValid() and g.GetTypeName() == "Mesh":
        UsdGeom.Gprim(g).CreateDoubleSidedAttr().Set(True)
        print(f"[mat] {g.GetPath()} doubleSided")
    else:
        print(f"[mat] Dropper glass mesh not found for doubleSided, skip")


def fix_old_cover(st2):
    """隐藏资产自带坏 cover（实测悬浮在机器后方 y 0.277..0.310 / z 0.843..0.874，
    离棱镜 0.18m，rotateX -65→0 甩到 z≈1.05 不会合平）。合盖由重建的 /Cover 承担。"""
    c = st2.GetPrimAtPath("/World/Refractometer/cover")
    if c.IsValid():
        UsdGeom.Imageable(c).MakeInvisible()
        print("[cover] hid broken asset cover /World/Refractometer/cover")
    else:
        print("[cover] /World/Refractometer/cover not found, skip")


def verify(st2):
    """自检：打印各器材世界 bbox，断言布局关系：
    折光仪贴台面（棱镜顶 0.9175、盖保留 -50° 掀开态）、样品瓶贴台面（塞保留未删）、
    架贴台面、滴管插孔（底落孔底 0.806）、瓶内液面可见、棱镜液滴/滴管液柱/滴落球/读数发光
    初始隐藏。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["Refractometer", "SampleBottle", "TestTubeRack", "Dropper",
             "SampleLiquid", "PrismDrop", "DropperFill", "DropperDrop"]
    names += [f"ScreenMeasuring_{i:02d}" for i in range(PROGRESS_STEPS)]
    names += [f"ScreenGlow_{N_D_KEY(v)}" for v in N_D_OPTIONS]
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

    # 折光仪：机身底贴台面，棱镜/盖在顶
    rmn, rmx = boxes["Refractometer"]
    assert abs(rmn[2] - TABLE_TOP) < 0.002, f"refractometer base z {rmn[2]} != table {TABLE_TOP}"
    assert rmx[2] > PRISM_TOP_Z, f"refractometer top {rmx[2]} below prism top {PRISM_TOP_Z}"
    # 重建的合盖翻板：铰链在 well 后沿 (0.30,0.127,0.9215)，掀开态立起后仰
    # （旧资产坏 cover 已隐藏，本 verify 不检查它）
    prism = st2.GetPrimAtPath("/World/Refractometer/prism")
    assert prism.IsValid(), "prism prim missing"
    ncv = st2.GetPrimAtPath("/World/Refractometer/Cover")
    assert ncv.IsValid(), "new cover prim missing (A1 hinged cover for close step)"
    nbm = st2.GetPrimAtPath("/World/Refractometer/Cover/Disc")
    assert nbm.IsValid(), "cover disc missing"
    ncr = bc.ComputeWorldBound(ncv).ComputeAlignedRange()
    ncmn, ncmx = ncr.GetMin(), ncr.GetMax()
    print(f"[verify] newCover min({ncmn[0]:+.4f},{ncmn[1]:+.4f},{ncmn[2]:+.4f}) "
          f"max({ncmx[0]:+.4f},{ncmx[1]:+.4f},{ncmx[2]:+.4f})")
    pr = bc.ComputeWorldBound(prism).ComputeAlignedRange()
    print(f"[verify] prism  min({pr.GetMin()[0]:+.4f},{pr.GetMin()[1]:+.4f},{pr.GetMin()[2]:+.4f}) "
          f"max({pr.GetMax()[0]:+.4f},{pr.GetMax()[1]:+.4f},{pr.GetMax()[2]:+.4f})")
    assert abs(pr.GetMax()[2] - PRISM_TOP_Z) < 0.002, f"prism top {pr.GetMax()[2]} != {PRISM_TOP_Z}"
    assert abs((pr.GetMin()[0] + pr.GetMax()[0]) / 2 - REFRACT_X) < 0.003, "prism x center off"
    assert abs((pr.GetMin()[1] + pr.GetMax()[1]) / 2 - PRISM_CY) < 0.003, "prism y center off"
    # 翻盖（open 态，rotateX=-55，板绕铰链后沿 X 轴立起）：
    #   x 中心 0.30；板前缘（-y）随旋转抬到 z≈0.950、y≈0.1075，铰链端 y=0.127 贴 well 后沿。
    assert abs((ncmn[0] + ncmx[0]) / 2 - REFRACT_X) < 0.005, "new cover x center off"
    assert ncmn[1] < REFRACT_Y + 0.112, f"new cover open front {ncmn[1]} not raised (-y)"
    assert abs(ncmx[1] - (REFRACT_Y + 0.127)) < 0.006, f"new cover hinge y {ncmx[1]} off 0.127"
    assert ncmx[2] > TABLE_TOP + 0.148, f"new cover open top {ncmx[2]} not raised (open state)"
    # 合平（rotateX=0）必须正好盖住 well（x 0.283..0.317 / y 0.093..0.127 / z 0.9215..0.9235）：
    # 这才是 合盖 的目标位姿。临时拨 0 断言后恢复 -55（st2 内存副本，最后才 Save）。
    for op in UsdGeom.Xformable(ncv).GetOrderedXformOps():
        if "rotateX" in op.GetName():
            op.Set(0.0)
    bc2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    ccr = bc2.ComputeWorldBound(nbm).ComputeAlignedRange()
    cmn, cmx = ccr.GetMin(), ccr.GetMax()
    print(f"[verify] cover CLOSED min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs((cmn[0] + cmx[0]) / 2 - REFRACT_X) < 0.005, "closed cover x center off"
    assert abs(cmn[1] - (REFRACT_Y + 0.093)) < 0.006, f"closed cover y min {cmn[1]} off well front 0.093"
    assert abs(cmx[1] - (REFRACT_Y + 0.127)) < 0.006, f"closed cover y max {cmx[1]} off hinge 0.127"
    assert abs(cmn[2] - (TABLE_TOP + 0.1215)) < 0.004, f"closed cover z min {cmn[2]} off hinge z 0.9215"
    assert abs(cmx[2] - (TABLE_TOP + 0.1215 + COVER_THK)) < 0.004, \
        f"closed cover z max {cmx[2]} off 0.9235"
    for op in UsdGeom.Xformable(ncv).GetOrderedXformOps():
        if "rotateX" in op.GetName():
            op.Set(COVER_OPEN_ANGLE)
    # 旧坏 cover 已隐藏
    ocv = st2.GetPrimAtPath("/World/Refractometer/cover")
    assert ocv.IsValid(), "asset cover should still exist (hidden)"
    assert UsdGeom.Imageable(ocv).ComputeVisibility() == "invisible", \
        "asset cover should be hidden (broken, replaced by new hinged cover)"

    # 机顶测量按钮（棱镜正前方 -y 6cm、凸起 6mm，机械臂滴样后按下触发测量）
    btn = st2.GetPrimAtPath("/World/Refractometer/start_button")
    assert btn.IsValid(), "start_button prim missing (A1 top measure button)"
    br = bc.ComputeWorldBound(btn).ComputeAlignedRange()
    bmn, bmx = br.GetMin(), br.GetMax()
    assert abs((bmn[0] + bmx[0]) / 2 - REFRACT_X) < 0.005, "start button x center off"
    assert abs((bmn[1] + bmx[1]) / 2 - (REFRACT_Y + 0.05)) < 0.004, "start button y center off"
    assert abs(bmn[2] - TABLE_TOP - 0.115) < 0.002, f"start button bottom {bmn[2]} not on machine top"
    assert bmx[2] > TABLE_TOP + 0.120, f"start button top {bmx[2]} below 0.920"

    # 样品瓶：贴台面，塞保留（未删），瓶口 rim 0.870
    bmn, bmx = boxes["SampleBottle"]
    assert abs(bmn[2] - TABLE_TOP) < 0.002, f"bottle bottom {bmn[2]} not on table"
    assert bmx[2] > TABLE_TOP + 0.078, f"bottle top {bmx[2]} below stopper top"
    sbp = st2.GetPrimAtPath("/World/SampleBottle")
    stoppers = [pp.GetName() for pp in Usd.PrimRange(sbp) if pp.GetName() == "stopper"]
    assert stoppers, f"stopper missing (A1 keeps it for 拔/盖): {stoppers}"
    print(f"[verify] bottle stopper kept: {stoppers}")

    # 架贴台面、滴管插孔（底落孔底 0.806）
    kmn, kmx = boxes["TestTubeRack"]
    assert abs(kmn[2] - TABLE_TOP) < 0.002, f"rack bottom {kmn[2]} not on table"
    dmn, dmx = boxes["Dropper"]
    assert abs(dmn[2] - HOLE_Z) < 0.002, f"dropper bottom {dmn[2]} != hole bottom {HOLE_Z}"
    assert abs((dmn[0] + dmx[0]) / 2 - DROPPER_XY[0]) < 0.003, f"dropper x center off left hole"

    # 效果 prim：瓶内液面可见、顶 ≤0.841；棱镜液滴隐藏且在棱镜顶；滴管液柱隐藏贴尖嘴；
    # 滴落球隐藏 4 球；读数发光隐藏
    sl = boxes.get("SampleLiquid")
    assert sl is not None, "SampleLiquid missing"
    assert sl[1][2] <= TABLE_TOP + 0.041, f"sample liquid top {sl[1][2]} above 0.841"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/SampleLiquid")).ComputeVisibility() != "invisible", \
        "SampleLiquid should be visible"
    pd = boxes["PrismDrop"]
    assert abs(pd[0][2] - PRISM_TOP_Z) < 0.002, f"prism drop bottom {pd[0][2]} not on prism top {PRISM_TOP_Z}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/PrismDrop")).ComputeVisibility() == "invisible", \
        "PrismDrop should be hidden"
    fl = boxes["DropperFill"]
    assert abs(fl[0][2] - HOLE_Z) < 0.002, f"DropperFill bottom {fl[0][2]} not at tip {HOLE_Z}"
    assert UsdGeom.Imageable(st2.GetPrimAtPath("/World/DropperFill")).ComputeVisibility() == "invisible", \
        "DropperFill should be hidden"
    dd = st2.GetPrimAtPath("/World/DropperDrop")
    assert dd.IsValid(), "DropperDrop missing"
    nd = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere")
    assert nd == DROPS_PER_GROUP, f"DropperDrop spheres {nd} != {DROPS_PER_GROUP}"
    assert UsdGeom.Imageable(dd).ComputeVisibility() == "invisible", "DropperDrop should be hidden"
    # 屏幕读数 prim：测量中 ScreenMeasuring_<i> 多帧 + 每档完成读数 ScreenGlow_<key>，
    # 初始都隐藏、各带 st UV + 贴图（headless 运行时改材质不渲染 → 预烘焙，task 逐帧/按档位切显）
    for sname in ([f"ScreenMeasuring_{i:02d}" for i in range(PROGRESS_STEPS)]
                  + [f"ScreenGlow_{N_D_KEY(v)}" for v in N_D_OPTIONS]):
        sp = st2.GetPrimAtPath(f"/World/{sname}")
        assert sp.IsValid(), f"{sname} missing"
        assert UsdGeom.Imageable(sp).ComputeVisibility() == "invisible", f"{sname} should be hidden"
        sr = bc.ComputeWorldBound(sp).ComputeAlignedRange()
        assert abs(sr.GetMax()[0] - (SCREEN_C[0] + 0.05)) < 0.002, f"{sname} width off (screen quad)"
        st_pv = UsdGeom.PrimvarsAPI(sp).GetPrimvar("st")
        assert st_pv.GetAttr().IsValid(), f"{sname} st UV primvar missing"
    for tex in ([SCREEN_TEX_MEASURING_TPL.format(step=i) for i in range(PROGRESS_STEPS)]
                + [f"textures/screen_result_{N_D_KEY(v)}.png" for v in N_D_OPTIONS]):
        assert os.path.exists(os.path.join(SCENE_DIR, tex)), f"screen texture missing: {tex}"
    print(f"[verify] OK: 折光仪贴台(棱镜顶0.9175/盖保留-50°掀开) / 瓶贴台(塞保留) / 架贴台 / 滴管插孔(0.806) "
          f"/ 瓶液面可见+棱镜滴隐藏+滴管液柱隐藏+滴球{nd}隐藏+"
          f"屏幕{PROGRESS_STEPS + len(N_D_OPTIONS)}张隐藏"
          f"(进度条{PROGRESS_STEPS}帧+每档nD读数)+各带st UV+贴图存在")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")
    make_screen_textures(os.path.join(SCENE_DIR, "textures"))

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot180 in EQUIP:
        add_equip(stage, name, asset, t, scale, rot180)
    add_cover(stage)             # 重建合盖翻板（铰链 well 后沿）
    add_a1_effects(stage)
    add_dropper_drops(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    fix_old_cover(st2)           # 隐藏资产自带坏 cover（替换成重建翻盖）
    fix_bottle_materials(st2)    # 瓶玻璃透明化（**塞保留不透明**）
    fix_dropper_materials(st2)   # 滴管玻璃透明化（管内 DropperFill 液柱透出）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
