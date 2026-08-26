# -*- coding: utf-8 -*-
"""生成 e2_magnetic.usd —— E2「磁性检测」场景（烘平自包含，真实器材）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材（lab_clean 干净，无需删 prim、无需抬台面）
- 待测固体已预先取出铺在表面皿上（DishPowder 常显），无需药匙/试管架/样品瓶/瓶盖
- 表面皿保持资产原貌（dish 玻璃壁 + powder 金属底两个材质子集），另铺本场景 DishPowder 薄层
- 效果 prim（初始隐藏，task 动画驱动）：
    MagnetGrains  磁性颗粒簇（磁性检测时被磁铁吸起，cfg.magnetic=magnetic 才显示）

布局（E2 目录简化版「磁铁中央前方，表面皿磁铁右侧」，操作区前移 y=0.20 远离底座）：
  BarMagnet    (0.40, 0.20)   条形磁铁 100×15×15mm 平放（长轴 X，N 红 -X / S 蓝 +X）
  SurfaceDish  (0.56, 0.20)   表面皿 Ø60×6.5mm（去粉子集，铺 DishPowder 薄层=待测固体）

用法：python scripts/gen_e2_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
import math
import random
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "e_physical", "e2_magnetic")
OUT = os.path.join(SCENE_DIR, "e2_magnetic.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 布局坐标（TCP/世界坐标，米，Z-up）——
# 2026-08-26 用户反馈「磁铁离机械臂太近抓不到/跳变」→ 操作区整体前移（-Y 远离底座
# y=0.57）：磁铁 0.40m、皿 0.48m 距底座，与 E1 抓取件（玻璃棒 (0.319,0.117) 距 0.46m）同级。
MAGNET_XY = (0.40, 0.20)     # 磁铁（平放，长轴 X）
DISH_XY = (0.56, 0.20)       # 表面皿

# —— 表面皿上铺开的待测固体粉末层（预铺常显，明显可见）——
# 皿 Ø60×6.5mm，皿顶 0.8066。粉末层盖皿中央：r 0.025、h 0.004（加厚到 4mm），顶 0.8106。
# 2026-08-26 用户反馈「物质一开始不明显」→ 加厚 + 加深颜色，让待测固体一眼可见。
DISH_POWDER_R, DISH_POWDER_H = 0.025, 0.004
DISH_TOP_Z = TABLE_TOP + 0.0066                      # 皿顶 0.8066
DISH_POWDER_Z = DISH_TOP_Z + DISH_POWDER_H / 2       # 中心 0.8086
DISH_POWDER_TOP_Z = DISH_TOP_Z + DISH_POWDER_H       # 顶 0.8106

# —— 铁粉颗粒簇（待测固体的表层细铁粉，一开始可见；cfg.magnetic=magnetic 时被磁铁吸起）——
GRAIN_R = 0.0015               # 颗粒半径 1.5mm（细铁粉；随机散布，1024px 下清晰不糊）
GRAIN_COLOR = (0.12, 0.12, 0.15)     # 深铁粉黑灰，明显区别于底层粉末（吸附时清晰可辨）
GRAIN_PARENT_Z = DISH_POWDER_TOP_Z   # 颗粒簇父 prim 静止 z（贴粉末层顶）
GRAIN_N = 120                  # 颗粒目标数量（更细更多，密集细粉感）
GRAIN_SPREAD_R = 0.022         # 撒布半径 22mm（粉末层内）
GRAIN_MIN_GAP = 2 * GRAIN_R + 0.0002  # 最小间距 3.2mm（拒绝采样避免重叠粘连）


def _scatter_grains(n, spread_r, min_gap, seed=20260826):
    """在半径 spread_r 的圆内随机撒 n 个点（面积均匀，不聚中心），拒绝采样保证间距
    ≥ min_gap。固定 seed 使场景生成可复现（确定性）。"""
    rng = random.Random(seed)
    pts = []
    tries = 0
    while len(pts) < n and tries < n * 500:
        tries += 1
        rr = spread_r * math.sqrt(rng.random())   # sqrt 使圆内面积均匀（不聚中心）
        aa = 2 * math.pi * rng.random()
        x, y = rr * math.cos(aa), rr * math.sin(aa)
        if all((x - px) ** 2 + (y - py) ** 2 >= min_gap ** 2 for px, py in pts):
            pts.append((x, y))
    return pts


GRAINS = _scatter_grains(GRAIN_N, GRAIN_SPREAD_R, GRAIN_MIN_GAP)

# 粉末（待测固体主体）颜色：中灰铁粉感，明显可见（原 0.45 太浅，皿上几乎看不出）
POWDER_COLOR = (0.30, 0.30, 0.33)
POWDER_ROUGH = 0.90

# (prim, asset_file, translate, scale, rot)   tz=None → 动态贴台面；rot=(rx,ry,rz)°
EQUIP = [
    ("BarMagnet", "bar_magnet.usd", (MAGNET_XY[0], MAGNET_XY[1], None), None, None),
    ("SurfaceDish", "sample_dish.usd", (DISH_XY[0], DISH_XY[1], TABLE_TOP), None, None),
]


def add_material(stage, prim, recipe, double_sided=False):
    """UsdPreviewSurface 材质（同 E1）。recipe 键：color/diffuseColor, opacity, roughness,
    ior, emissive。透材质（opacity<1）自动 doubleSided。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    diffuse = recipe.get("diffuseColor", recipe.get("color", (0.9, 0.9, 0.9)))
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(recipe.get("opacity", 1.0))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(recipe.get("roughness", 0.5))
    if recipe.get("ior") is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(recipe["ior"])
    if recipe.get("emissive") is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*recipe["emissive"]))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
    if double_sided and prim.IsA(UsdGeom.Gprim):
        UsdGeom.Gprim(prim).CreateDoubleSidedAttr().Set(True)


def asset_local_min_z(asset_file):
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale, rot=None):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(os.path.abspath(os.path.join(EQ, asset)))
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if rot is not None:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(rot[0], rot[1], rot[2]))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx},{ty},{tz})"
          + (f" rot{rot}" if rot else "") + (f" scale {scale}" if scale else ""))


def add_dish_powder(stage):
    """表面皿上铺开的薄层粉末（待测固体，预铺常显）。"""
    cyl = UsdGeom.Cylinder.Define(stage, "/World/DishPowder")
    cyl.CreateRadiusAttr(DISH_POWDER_R)
    cyl.CreateHeightAttr(DISH_POWDER_H)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(DISH_XY[0], DISH_XY[1], DISH_POWDER_Z))
    add_material(stage, cyl.GetPrim(),
                 dict(color=POWDER_COLOR, roughness=POWDER_ROUGH))
    print(f"[effect] DishPowder visible (top {DISH_POWDER_TOP_Z})")


def add_magnet_grains(stage):
    """铁粉颗粒簇：父 Xform + 64 颗细小球，初始可见（物质一开始就铺在皿上）。task 在
    磁铁靠近 + cfg.magnetic=magnetic 时动画父 prim z 上升（被吸到磁铁底）再回落。"""
    parent = UsdGeom.Xform.Define(stage, "/World/MagnetGrains")
    parent.AddTranslateOp().Set(Gf.Vec3d(DISH_XY[0], DISH_XY[1], GRAIN_PARENT_Z))
    for i, (gx, gy) in enumerate(GRAINS):
        sph = UsdGeom.Sphere.Define(stage, f"/World/MagnetGrains/grain_{i}")
        sph.CreateRadiusAttr(GRAIN_R)
        sph.AddTranslateOp().Set(Gf.Vec3d(gx, gy, GRAIN_R))
        add_material(stage, sph.GetPrim(),
                     dict(color=GRAIN_COLOR, roughness=0.30))
    print(f"[effect] MagnetGrains visible ({len(GRAINS)} grains r={GRAIN_R})")


def add_env_light(stage):
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity -> 12000")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_stray_env_lights(st2):
    """扫除烘平后残留的嵌套 DomeLight，只留 /World/env_light。

    test_tube_rack 等 equipment 资产自带 env_light（贴图是 flametest 的 1×1 近黑
    color_0C0C0C.exr），烘平后进入场景，把环境压暗、金属磁铁无反射反黑（同 d2s）。
    Usd.PrimRange 深度遍历 /World 所有后代，删掉非 /World/env_light 的 DomeLight。
    """
    keep = {"/World/env_light"}
    root = st2.GetPrimAtPath("/World")
    paths = [p.GetPath() for p in Usd.PrimRange(root)
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString not in keep]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed stray DomeLight {path}")
    if not paths:
        print("[clean] no stray DomeLight under /World")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for name in ["BarMagnet", "SurfaceDish", "DishPowder", "MagnetGrains"]:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        cx = (mn[0] + mx[0]) / 2
        cy = (mn[1] + mx[1]) / 2
        cz = (mn[2] + mx[2]) / 2
        print(f"[verify] {name:14s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f}) "
              f"center({cx:+.4f},{cy:+.4f},{cz:+.4f})")
    # 不变量：待测固体已铺在皿上（DishPowder 常显）；铁粉颗粒簇初始也可见（物质一开始就明显）
    dish_powder = st2.GetPrimAtPath("/World/DishPowder")
    assert UsdGeom.Imageable(dish_powder).ComputeVisibility() != "invisible", \
        "DishPowder should be visible (sample pre-placed)"
    grains = st2.GetPrimAtPath("/World/MagnetGrains")
    assert UsdGeom.Imageable(grains).ComputeVisibility() != "invisible", \
        "MagnetGrains should be visible initially (iron powder on dish)"
    # 磁铁平放：底面贴台面 0.80、顶 0.815
    mag = st2.GetPrimAtPath("/World/BarMagnet")
    mr = bc.ComputeWorldBound(mag).ComputeAlignedRange()
    assert abs(mr.GetMin()[2] - TABLE_TOP) < 1e-4, \
        f"BarMagnet bottom z={mr.GetMin()[2]} != {TABLE_TOP}"
    assert abs(mr.GetMax()[2] - (TABLE_TOP + 0.015)) < 1e-4, \
        f"BarMagnet top z={mr.GetMax()[2]} != {TABLE_TOP + 0.015}"
    print(f"[verify] DishPowder visible (top {DISH_POWDER_TOP_Z}), "
          f"MagnetGrains visible, magnet flat 0.80..{TABLE_TOP + 0.015}")
    stray = [p.GetPath().pathString for p in Usd.PrimRange(st2.GetPrimAtPath("/World"))
             if p.GetTypeName() == "DomeLight" and p.GetPath().pathString != "/World/env_light"]
    assert not stray, f"stray DomeLight remains: {stray}"
    print("[verify] no stray DomeLight")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print("[env] copied env_bright.png")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale, rot in EQUIP:
        add_equip(stage, name, asset, t, scale, rot)
    add_dish_powder(stage)
    add_magnet_grains(stage)
    add_env_light(stage)
    stage.Export(OUT)

    st2 = Usd.Stage.Open(OUT)
    remove_stray_env_lights(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
