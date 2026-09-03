# -*- coding: utf-8 -*-
"""D1 酸碱滴定 · 定稿场景烘焙（忠实 d1_tmp 原坐标，单产出）——D5 思路重写版。

历史：2026-09-03 D1 三相机全黑。旧 gen 走「/root → 改名 /World」的 Sdf 关系改写手术，
排查到静态维度（灯逐值=B5 亮场景、材质/洗瓶/桌/合成）全部等价却仍黑；同天只有
gen_d5_final.py 烘的 D5 确认渲染亮。故按用户「重新写一遍」要求，把 D1 的烘焙整体改写成
gen_d5_final.py 的**同款代码路径**（唯一 2026-09-03 当天确认亮的配方），内容仍是 D1。

输入：用户 Blender 手摆 d1_tmp.usd（defaultPrim=/root，几何贴 /root/World/*，权威布局）。
处理（与 gen_d5_final 同构）：
  1) 塌平嵌套 /root/World Scope —— 保清单子级(九件器材)用 Sdf.CopySpec 提到 /root 直下，
     删除空 Scope。材质绑定目标都在 /root/_materials 下，CopySpec 保持绝对目标路径不变，
     引用到运行 /World 时由 USD 把层内 /root→/World 自动重映射，无需改字符串。
     顺带清 tmp 碎片：垃圾 beaker 残留 x2、空 _materials_003/004/005、空 Looks、假灯组
     CylinderLight(Xform 壳,内 SphereLight r0)、旧 env_light DomeLight。
  2) 玻璃透明化：tmp 的 Blender 材质是 UsdPreviewSurface(子 Shader Principled_BSDF)，
     按 GLASS_RECIPES 覆写 shader 输入（锥形瓶 = B3L 式 alpha 半透明 opacity0.2/spec0.2，
     stopper 磨砂 op0.85），绑定 mesh 置 doubleSided。
  3) 塞子搬出瓶口平放桌面（指示剂瓶已开盖 = P1 滴管可直接伸进去吸）。
  4) 铁架台 + 酸式滴定管（独立顶层 prim，同 clamp）整体向 +X 平移 10cm（用户要求，
     原铁架台 x0.372-0.649 → 0.472-0.749，滴定管心 0.375 → 0.475）。
  5) 效果 prim 预建（task.py 驱动，运行时只切 visibility）：锥形瓶内 NaOH 无色/粉两根液柱、
     指示剂瓶内酚酞液面、滴管吸上液柱(独立/跟随)、坠滴 3 粉球(独立)。
  6) 光（同 D5 配方）：/root/env_light DomeLight env_bright.png I=LIGHT_DOME_INTENSITY +
     /root/CylinderLight 标准圆柱主光（radius5 length100 extent[-5,-5,-50..5,5,50]
     intensity=LIGHT_KEY_INTENSITY @(2.1,1.0572,7) orient(0.5,0.5,0.5,0.5)）。
     ⚠ RTX 下灯必须带 authored extent，否则光体无效≈无光（D1 老根因，见记忆）。
输出：d1_acid_base_titration.usd（defaultPrim=/root，内容全为其子级，可运行）。

verify：断言 defaultPrim 结构塌平干净、器材世界 bbox 对 tmp 真值、效果 prim 可见性、
        玻璃 shader 覆写（锥形瓶含 transmission）、铁架台/滴定管位移、灯逐值（含 extent），
        并在临时台把 OUT 引用到 /World 断言
        task.py 引用的 6 条路径全在。

用法：python scripts/gen_d1_acid_base_titration_scene.py   （base python 有 pxr）
"""
import os
import shutil

from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d1_acid_base_titration")
TMP = os.path.join(SCENE_DIR, "d1_tmp.usd")
OUT = os.path.join(SCENE_DIR, "d1_acid_base_titration.usd")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

# 光照强度（照搬 D5 确认亮配方 LIGHT_KEY=7000/DOME=600 —— 2026-09-03 当天唯一验证过渲染亮的
# 灯参数。D5 是玻璃台面场景、此参数渲染亮；想微调亮度：改这两个常量后重跑本脚本即可，勿手改 usd）。
LIGHT_KEY_INTENSITY = 7000.0
LIGHT_DOME_INTENSITY = 600.0

# —— 玻璃材质名（tmp 中 material:binding → /root/_materials/<名>；真玻璃透明化，stopper 磨砂）——
# recipe 键: opacity/diffuseColor/metallic/roughness；None = 不覆盖该项
GLASS_RECIPES = {
    # 锥形瓶（要看清瓶内 NaOH 滴酚酞变粉）→ **alpha 半透明**配方，同 B3L 封闭样品瓶那套实证能透的玻璃
    # （实测 /World/SolutionBottle/bottle_mat：opacity0.25/rough0.1/spec0.5/ior1.5，用户看清瓶内变蓝）。
    # ⚠ 勿上 transmission：opacity1.0+transmission1.0 在这台 RTX 落成"实心不透明"（已试，用户报完全不透明）。
    # D1 锥形瓶早先"镜子"= spec0.5 高反射 + 瓶内几乎空(无色底一薄层)使反射主导，不是 alpha 本身坏 →
    # specular 压到 0.2 去反光。doubleSided 已置 True。
    "conical_flask_glass_005": dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.20,
                                    metallic=0.0, roughness=0.10, specular=0.2),
    "glass_001_005":          dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.20,
                                    metallic=0.0, roughness=0.10),   # 滴管玻璃管（看吸上的液柱）
    "glass_012":              dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.18,
                                    metallic=0.0, roughness=0.10),   # 滴定管管身（看弯月面）
    "bottle_mat_010":         dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.20,
                                    metallic=0.0, roughness=0.10),   # 指示剂瓶瓶身
    "stopper_mat_010":        dict(diffuseColor=(0.90, 0.90, 0.92), opacity=0.85,
                                    metallic=0.0, roughness=0.40),   # 指示剂瓶磨砂塞（搬出后仍可见）
    "bottle_mat_011":         dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.20,
                                    metallic=0.0, roughness=0.10),   # 盐酸瓶瓶身
    "stopper_mat_011":        dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.85,
                                    metallic=0.0, roughness=0.40),   # 盐酸瓶磨砂塞
}

# —— 效果几何（局部坐标，全部由 tmp 真值推导；父级器材自带世界位姿）——
# 锥形瓶 W：translate(0.2751,0.1844,0.80) rotZ-180（瓶口顶 0.9645）。NaOH 无色液柱约 20mL：
# 烧瓶底部内径约 Ø66 → r0.033 底贴内底 local 0.004、高 0.012，液面世界 z=0.816
# （略加高便于 camera 斜视角看清粉色液层；瓶底宽壁直段能容下）。
FLASK_NAOH = dict(r=0.033, bottom_local=0.004, h=0.012)     # 中心 local z = bottom + h/2；顶 local 0.016
NAOH_CLEAR = dict(diffuse=(0.93, 0.97, 1.00), opacity=0.45, emissive=None)     # 无色(水样更透)
# 酚酞浅粉：用户嫌"完全紫红太深/不够透明" → diffuse 调浅(0.98,0.48,0.78)、opacity 0.92实心→0.55半透明、
# emissive 调弱，液面以下像淡粉水溶液而非不透明色块。
NAOH_PINK = dict(diffuse=(0.98, 0.48, 0.78), opacity=0.55, emissive=(0.20, 0.02, 0.10))
# 指示剂瓶：translate(0.2685,0.3925,0.80)；瓶身 0.80-0.87（瓶口 rim 0.87），内径约 Ø33 →
# 液柱 r0.013，底 local 0.003、顶 local 0.028（液面世界 0.828 = 滴管尖下探目标之下的可吸层）。
IND_LIQUID = dict(r=0.013, bottom_local=0.003, top_local=0.028)   # 中心 local 0.0155
IND_COLOR = dict(diffuse=(0.99, 0.95, 0.97), opacity=0.45)         # 酚酞 ≈ 无色微粉
# 滴管填液柱（独立 prim，跟随滴管，task 运行时摆位）：Ø5 短柱；酚酞近无色 → 极淡粉便于观察
FILL_COLOR = dict(diffuse=(0.99, 0.86, 0.93), opacity=0.75)
# 坠滴（独立 prim，task 运行时摆位）：Ø4.4 粉球 x3（无色滴看不出，给微粉辨识 + 暗示后续变粉）
# 颜色随 NAOH_PINK 调浅(0.98,0.60,0.85) 保持一致
DROP_COLOR = dict(diffuse=(0.98, 0.60, 0.85), opacity=0.85)
DROP_R = 0.0022

# —— /root/World 里要塌进 /root 的保清单（九件器材）——
KEEP_WORLD = ["table", "IronStand", "WashBottle", "CollisionMesh", "IndicatorBottle",
              "BuretteAcid", "conical_flask_93x93x165", "Dropper", "HclBottle"]
# —— /root 顶层垃圾（tmp 残片，塌平前删）——
DROP_ROOT = ["beaker_111x75x116_008", "beaker_111x75x116_008_001",
             "_materials_003", "_materials_004", "_materials_005", "env_light"]


# ----------------------------------------------------------------------
# 1) 塌平 /root/World 空壳 + 清 tmp 垃圾（纯 Sdf，单层）
# ----------------------------------------------------------------------
def collapse_and_clean(layer):
    """把 /root/World/* 里 KEEP_WORLD 提到 /root 直下，删 /root/World；删 /root 顶层垃圾。"""
    root_spec = layer.GetPrimAtPath("/root")
    assert root_spec, "d1_tmp defaultPrim 不是 /root"
    world = root_spec.nameChildren.get("World")
    assert world is not None and world.typeName in ("", "Scope", "Xform"), \
        "/root/World 缺失或类型不符"
    attrs = world.attributes or {}
    has_xform = any("xform" in k.lower() for k in attrs.keys())
    assert not has_xform, \
        "/root/World 带非恒等变换，上提子级会改世界位姿（忠实 tmp 前提破坏）"

    # 1a) 删 /root 顶层垃圾（beaker 残留 / 空 _materials_00x / 旧 env_light，会被重建）
    for name in DROP_ROOT:
        if name in root_spec.nameChildren:
            del root_spec.nameChildren[name]
            print("[clean] drop /root/%s" % name)

    # 1b) 塌 KEEP_WORLD 子级：/root/World/<child> -> /root/<child>（Sdf.CopySpec 保绝对材质目标）
    n = 0
    for cname in list(world.nameChildren.keys()):
        if cname not in KEEP_WORLD:
            print("[collapse] skip /root/World/%s（碎片：假灯组/空 Looks）" % cname)
            continue
        src = "/root/World/%s" % cname
        dst = "/root/%s" % cname
        if dst in root_spec.nameChildren:
            raise RuntimeError("塌平冲突 %s 已存在于 /root" % dst)
        Sdf.CopySpec(layer, src, layer, dst)
        n += 1
    del root_spec.nameChildren["World"]
    print("[collapse] /root/World -> %d 个子级上提到 /root，空 Scope 已删" % n)
    return n


# ----------------------------------------------------------------------
# 2) 几何测量（pxr 0.26.8 BBoxCache 一律空，纯手工遍历网格，同 gen_d5_final）
# ----------------------------------------------------------------------
def _measure(stage, path):
    """世界包围盒：遍历 prim 子树 purpose=default 的 Mesh，取 extent/采样 points，
    用父链世界矩阵把 8 角变换到世界累积 min/max。返回 ((min),(max)) 或 None。"""
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return None
    lo = None
    hi = None

    def grow(pts):
        nonlocal lo, hi
        for pt in pts:
            lo = (min(lo[0], pt[0]), min(lo[1], pt[1]), min(lo[2], pt[2])) if lo else tuple(pt)
            hi = (max(hi[0], pt[0]), max(hi[1], pt[1]), max(hi[2], pt[2])) if hi else tuple(pt)

    for pr in Usd.PrimRange(p):
        if pr.GetTypeName() != "Mesh":
            continue
        purpose = UsdGeom.Imageable(pr).GetPurposeAttr().Get() or UsdGeom.Tokens.default_
        if purpose != UsdGeom.Tokens.default_:
            continue
        xf = UsdGeom.Xformable(pr)
        wm = xf.ComputeLocalToWorldTransform(Usd.TimeCode(0.0))
        geom = UsdGeom.Gprim(pr)
        ext = geom.GetExtentAttr().Get() if geom.GetExtentAttr() else None
        corners = []
        if ext and len(ext) == 2:
            mn, mx = ext[0], ext[1]
            for i in range(8):
                corners.append((mn[0] if i & 1 else mx[0],
                                mn[1] if i & 2 else mx[1],
                                mn[2] if i & 4 else mx[2]))
        else:
            pts = UsdGeom.PointBased(pr).GetPointsAttr().Get()
            corners = list(pts) if pts else []
        if not corners:
            continue
        world = [tuple(wm.Transform(pt)) for pt in corners]
        grow(world)
    if lo is None:
        return None
    return (lo, hi)


# ----------------------------------------------------------------------
# 3) 材质助手（新效果材质，UsdPreviewSurface，同 gen_d5_final 风格）
# ----------------------------------------------------------------------
def _std_mat(stage, mat_name, diffuse, opacity, emissive=None, roughness=0.05, ior=None):
    mat_path = "/root/_materials/%s" % mat_name
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if ior is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _glass_cylinder(stage, path, r, h, cz_local, mat_name, color, visible=True):
    """建竖直(Z)透明液柱；父级(flask)自带 rotZ，local 平移即可（同旧 gen _liquid_cylinder）。"""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr().Set(r)
    cyl.CreateHeightAttr().Set(h)
    cyl.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(cyl.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, cz_local))
    mat = _std_mat(stage, mat_name, color["diffuse"], color["opacity"], color.get("emissive"))
    UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)
    UsdGeom.Gprim(cyl.GetPrim()).CreateDoubleSidedAttr().Set(True)
    if not visible:
        UsdGeom.Imageable(cyl.GetPrim()).MakeInvisible()
    return cyl


# ----------------------------------------------------------------------
# 4) 可视化后处理（玻璃 / 塞子搬出 / 效果 prim）
# ----------------------------------------------------------------------
def _set_shader(stage, mat_path, recipe):
    """对 Material 子 Principled_BSDF shader 覆写输入（tmp 统一 Material→Shader 结构）。"""
    mp = stage.GetPrimAtPath(mat_path)
    if not mp.IsValid() or mp.GetTypeName() != "Material":
        return False
    for c in mp.GetChildren():
        if c.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(c)
        for name, val in recipe.items():
            if val is None:
                continue
            vt = Sdf.ValueTypeNames.Color3f if name == "diffuseColor" else Sdf.ValueTypeNames.Float
            inp = sh.GetInput(name)
            if not inp:
                inp = sh.CreateInput(name, vt)
            inp.Set(Gf.Vec3f(*val) if name == "diffuseColor" else val)
        print("  [glass] %s -> %s" % (mat_path.split("/")[-1],
                                      {k: v for k, v in recipe.items() if v is not None}))
        return True
    return False


def glassify(stage):
    """玻璃透明化：命中 GLASS_RECIPES 的材质覆写 shader 输入；其绑定 mesh 置 doubleSided。"""
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        rel = p.GetRelationship("material:binding")
        if not rel:
            continue
        for tgt in rel.GetTargets():
            name = tgt.pathString.split("/")[-1]
            if name in GLASS_RECIPES:
                if _set_shader(stage, tgt.pathString, GLASS_RECIPES[name]):
                    UsdGeom.Gprim(p).CreateDoubleSidedAttr().Set(True)


def relocate_stopper(stage):
    """指示剂瓶瓶塞从瓶口搬出、平放桌面瓶旁（瓶已开盖供滴管伸入）。

    塞 mesh 局部 z∈[0.068,0.079]（原塞在瓶口：底 0.868≈瓶口 rim）。平放到桌面(世界 z=0.80)
    需塞底贴 0.80 → local z 平移 -0.068；x 移到瓶右侧 5cm（世界 0.335）→ 局部 x +0.0665。
    """
    st = stage.GetPrimAtPath("/root/IndicatorBottle/stopper")
    if not st.IsValid():
        print("  [stopper] /root/IndicatorBottle/stopper MISSING, skip")
        return
    UsdGeom.XformCommonAPI(st).SetTranslate(Gf.Vec3d(0.0665, 0.0, -0.068))
    print("  [stopper] IndicatorBottle/stopper -> 平放桌面 (0.335, 0.3925, z0.80)")


def shift_stand_burette(stage, dx=0.10):
    """铁架台 + 酸式滴定管整体向 +X 平移 dx（用户要求 10cm）。

    两者是**独立顶层 prim**（BuretteAcid 非 IronStand 子级，是挂在同 clamp 上的独立件），
    各带 translate + rotZ-180 + scale。父级 /root 恒等 → 改 translate 的 x 分量即世界 +X
    （rotZ 只镜像子内容、不影响 translate 作用的父空间）。两件平移量相同 → 相对位姿不变。
    """
    for name in ("IronStand", "BuretteAcid"):
        p = stage.GetPrimAtPath("/root/%s" % name)
        assert p.IsValid(), "/root/%s 缺失" % name
        xf = UsdGeom.Xformable(p)
        new_x = None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
                continue
            v = op.Get()
            new_x = v[0] + dx
            op.GetAttr().Set(Gf.Vec3d(new_x, v[1], v[2]))
        assert new_x is not None, "/root/%s 无 translate op" % name
        print("  [shift] /root/%s +X %.2f -> x0=%.3f" % (name, dx, new_x))


def add_effects(stage):
    """效果 prim 预建（全部可见性初始可控，运行时只切 visibility——坑21）。"""
    # 1) 锥形瓶 W 内 NaOH：无色可见 / 粉隐藏（同几何 r0.033 h0.008 底 local 0.004）
    f = FLASK_NAOH
    cz = f["bottom_local"] + f["h"] / 2.0
    _glass_cylinder(stage, "/root/conical_flask_93x93x165/FlaskNaOH",
                    f["r"], f["h"], cz, "flask_naoh_mat", NAOH_CLEAR, visible=True)
    _glass_cylinder(stage, "/root/conical_flask_93x93x165/FlaskNaOHPink",
                    f["r"], f["h"], cz, "flask_pink_mat", NAOH_PINK, visible=False)
    print("  [effect] FlaskNaOH/FlaskNaOHPink 底local0.004 顶local%.3f (无色可见/粉隐藏)"
          % (f["bottom_local"] + f["h"]))

    # 2) 指示剂瓶液柱（酚酞微粉，静态可见——真实感：瓶本来半满）
    i = IND_LIQUID
    ih = i["top_local"] - i["bottom_local"]
    ic = i["bottom_local"] + ih / 2.0
    _glass_cylinder(stage, "/root/IndicatorBottle/IndicatorLiquid",
                    i["r"], ih, ic, "indicator_liquid_mat", IND_COLOR, visible=True)
    print("  [effect] IndicatorLiquid 液面世界 z=%.3f" % (0.80 + i["top_local"]))

    # 3) 滴管填液柱（独立 prim 跟随滴管；默认隐藏，task 吸上后显示+摆位）
    _glass_cylinder(stage, "/root/DropperFill",
                    0.0025, 0.045, 0.0, "dropper_fill_mat", FILL_COLOR, visible=False)
    print("  [effect] DropperFill (滴管内吸上液柱, 隐藏初始, task 摆位)")

    # 4) 坠滴动画 3 颗粉球（独立 prim；默认隐藏，task 落地显示+摆位）
    UsdGeom.Xform.Define(stage, "/root/DropperDrop")
    drop_mat = _std_mat(stage, "drop_mat", DROP_COLOR["diffuse"], DROP_COLOR["opacity"])
    for k in range(3):
        sph = UsdGeom.Sphere.Define(stage, "/root/DropperDrop/Drop_%d" % k)
        sph.CreateRadiusAttr().Set(DROP_R)
        UsdShade.MaterialBindingAPI(sph.GetPrim()).Bind(drop_mat)
        UsdGeom.Gprim(sph.GetPrim()).CreateDoubleSidedAttr().Set(True)
        UsdGeom.Imageable(sph.GetPrim()).MakeInvisible()
    print("  [effect] DropperDrop/Drop_0..2 (Ø4.4 粉球 x3, 隐藏初始, task 摆位)")


def fix_lights(stage):
    """修灯：/root/env_light DomeLight + /root/CylinderLight 圆柱主光 —— 逐值照搬 gen_d5_final
    （2026-09-03 当天唯一确认渲染亮的配方）。历史教训：主光被 tmp 移走 / 重建漏 authored extent
    → 光体无效≈无光 → 全黑（见记忆 d1-cameras-black-light-fix）。"""
    # 1) 删可能残留的假主光壳 / 旧 DomeLight（塌平已删，这里防御）
    shell = stage.GetPrimAtPath("/root/CylinderLight")
    if shell.IsValid():
        stage.RemovePrim("/root/CylinderLight")
    env_old = stage.GetPrimAtPath("/root/env_light")
    if env_old.IsValid():
        stage.RemovePrim(env_old.GetPath())

    # 2) env dome → env_bright.png（相对路径，同 D5；文件拷进本场景 textures/）
    tex_dir = os.path.join(SCENE_DIR, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    if not os.path.exists(os.path.join(tex_dir, "env_bright.png")) and os.path.exists(ENV_TEX_SRC):
        shutil.copy(ENV_TEX_SRC, os.path.join(tex_dir, "env_bright.png"))
    dome = UsdLux.DomeLight.Define(stage, "/root/env_light")
    dome.GetIntensityAttr().Set(LIGHT_DOME_INTENSITY)
    dome.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    dome.GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[light] /root/env_light DomeLight env_bright.png I%.0f" % LIGHT_DOME_INTENSITY)

    # 3) 标准圆柱主光（radius5 length100 + authored extent；cone 不写 = 默认180 满开，同 D5）
    cyl = UsdLux.CylinderLight.Define(stage, "/root/CylinderLight")
    cyl.GetIntensityAttr().Set(LIGHT_KEY_INTENSITY)
    cyl.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    cyl.GetRadiusAttr().Set(5.0)
    cyl.GetLengthAttr().Set(100.0)
    ext_attr = cyl.GetPrim().CreateAttribute("extent", Sdf.ValueTypeNames.Float3Array)
    ext_attr.Set([Gf.Vec3f(-5, -5, -50), Gf.Vec3f(5, 5, 50)])
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3f(2.1, 1.057155977178849, 7.0))
    xf.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, 0.5, 0.5))   # wxyz 同 D5/B5
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    print("[light] /root/CylinderLight r5 l100 ext[-5..5,-50..50] I%.0f @(2.1,1.057,7)"
          % LIGHT_KEY_INTENSITY)


# ----------------------------------------------------------------------
# 5) verify：结构/bbox/效果/灯逐值 + 引用到 /World 的 task 路径断言
# ----------------------------------------------------------------------
def verify_ref_world(out_path):
    """在临时台把 out 引用到 /World，断言 task.py 需要的路径全部存在（材质自动重映射）。"""
    st = Usd.Stage.CreateInMemory()
    w = st.DefinePrim("/World", "Xform")
    st.SetDefaultPrim(w)
    w.GetReferences().AddReference(os.path.abspath(out_path))
    need = ["conical_flask_93x93x165/FlaskNaOH", "conical_flask_93x93x165/FlaskNaOHPink",
            "Dropper", "DropperFill", "DropperDrop", "IndicatorBottle/IndicatorLiquid"]
    missing = [n for n in need if not st.GetPrimAtPath("/World/" + n).IsValid()]
    sample = [c.GetName() for c in st.GetPrimAtPath("/World").GetChildren()][:26]
    print("[verify-ref] /World children sample: %s" % sample)
    assert not missing, "task 路径缺失(引用到 /World 后): %s" % missing
    # 材质绑定应重映射到 /World/_materials（取锥形瓶一个 Mesh 验证非悬挂）
    bound = None
    for p in Usd.PrimRange(st.GetPrimAtPath("/World/conical_flask_93x93x165")):
        if p.GetTypeName() != "Mesh":
            continue
        got = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
        src = got[0] if isinstance(got, (tuple, list)) else got
        if src is not None and hasattr(src, "GetPath") \
                and src.GetPath().pathString.startswith("/World/"):
            bound = src.GetPath().pathString
            break
    assert bound, "材质全悬挂或未解析到 /World/_materials"
    print("[verify-ref] OK: task 路径齐 + 材质重映射 %s" % bound)


def verify(stage):
    ok = True
    print("[verify] defaultPrim = %s" % stage.GetDefaultPrim().GetPath())

    # 1) 结构：无 /root/World、无垃圾顶层（env_light/CylinderLight 是重建后的灯，允许）
    leftover = []
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        ps = str(p.GetPath())
        if ps.startswith("/root/World") or "/Looks" in ps \
                or ps.endswith("beaker_111x75x116_008"):
            leftover.append(ps)
    if leftover:
        ok = False
        print("[verify] FAIL: leftover: %s" % leftover[:5])

    # 2) 器材世界 bbox 对照 tmp 真值（Dropper/锥形瓶/试管架）
    expect = {
        "Dropper":       ((0.4942, 0.4457, 0.8000), (0.5052, 0.4567, 0.9502)),
        "conical_flask_93x93x165": ((0.2289, 0.1381, 0.8000), (0.3214, 0.2307, 0.9645)),
        "TestTubeRack":  ((0.4754, 0.3861, 0.7961), (0.5609, 0.6717, 0.9131)),
    }
    for name, (mn, mx) in expect.items():
        bb = _measure(stage, "/root/" + name)
        if bb is None:
            ok = False
            print("[verify] FAIL: /root/%s MISSING" % name)
            continue
        lo, hi = bb
        d1 = max(abs(lo[i] - mn[i]) for i in range(3))
        d2 = max(abs(hi[i] - mx[i]) for i in range(3))
        good = d1 < 0.005 and d2 < 0.005
        ok &= good
        print("[verify] %-24s bbox %.3f..%.3f vs tmp %.4f/%.4f %s"
              % (name, lo[0], hi[2], d1, d2, "OK" if good else "FAIL"))

    # 3) stopper 已搬出：瓶身 bbox 顶回到 0.87（无塞），塞子单独落在桌面
    ib = _measure(stage, "/root/IndicatorBottle")
    sb = _measure(stage, "/root/IndicatorBottle/stopper")
    if ib and sb:
        ib_top = ib[1][2]
        s_min, s_max = sb
        moved = ib_top < 0.872 and abs(s_min[2] - 0.80) < 0.002 \
            and abs((s_min[0] + s_max[0]) / 2 - 0.335) < 0.005
        ok &= moved
        print("[verify] IndicatorBottle 顶 z=%.4f (<0.872 无塞) stopper 落 z底%.3f 心x%.3f %s"
              % (ib_top, s_min[2], (s_min[0] + s_max[0]) / 2, "OK" if moved else "FAIL"))

    # 4) 效果 prim 存在 + 初始可见性
    checks = [("FlaskNaOH", "/root/conical_flask_93x93x165/FlaskNaOH", True),
              ("FlaskNaOHPink", "/root/conical_flask_93x93x165/FlaskNaOHPink", False),
              ("IndicatorLiquid", "/root/IndicatorBottle/IndicatorLiquid", True),
              ("DropperFill", "/root/DropperFill", False),
              ("Drop_0", "/root/DropperDrop/Drop_0", False)]
    for prim_name, ppath, want_vis in checks:
        pp = stage.GetPrimAtPath(ppath)
        cur = UsdGeom.Imageable(pp).GetVisibilityAttr().Get() != "invisible" if pp.IsValid() else None
        good = pp.IsValid() and cur == want_vis
        ok &= good
        print("[verify] %-16s 初始可见=%s (期望 %s) %s"
              % (prim_name, cur, want_vis, "OK" if good else "FAIL"))

    # FlaskNaOH 几何应落在锥形瓶内（世界 z 0.804..0.816，心 x=0.2751 y=0.1844）
    nb = _measure(stage, "/root/conical_flask_93x93x165/FlaskNaOH")
    if nb:
        lo, hi = nb
        in_flask = abs(lo[2] - 0.804) < 0.003 and abs(hi[2] - 0.816) < 0.003 \
            and abs((lo[0] + hi[0]) / 2 - 0.2751) < 0.003 \
            and abs((lo[1] + hi[1]) / 2 - 0.1844) < 0.003
        ok &= in_flask
        print("[verify] FlaskNaOH @ z%.3f..%.3f 心(%.3f,%.3f) %s"
              % (lo[2], hi[2], (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                 "OK" if in_flask else "FAIL"))

    # 5) 玻璃配方已生效：锥形瓶 = B3L 式 alpha 半透明(opacity0.20, spec0.2)，stopper 磨砂 op0.85
    for mat_name, want_op, want_tx in (("conical_flask_glass_005", 0.20, None),
                                       ("stopper_mat_010", 0.85, None)):
        mp = stage.GetPrimAtPath("/root/_materials/%s" % mat_name)
        got = got_tx = None
        if mp.IsValid():
            for c in mp.GetChildren():
                if c.GetTypeName() == "Shader":
                    sh = UsdShade.Shader(c)
                    got = sh.GetInput("opacity").Get()
                    if want_tx is not None:
                        tx_inp = sh.GetInput("transmission")
                        got_tx = tx_inp.Get() if tx_inp else None
        good = got is not None and abs(got - want_op) < 1e-3
        if want_tx is not None:
            good = good and got_tx is not None and abs(got_tx - want_tx) < 1e-3
        ok &= good
        print("[verify] %s: opacity=%s transmission=%s (期望 %s/%s) %s"
              % (mat_name, got, got_tx, want_op, want_tx, "OK" if good else "FAIL"))

    # 5b) 铁架台 + 酸式滴定管已向 +X 平移 10cm：IronStand 底座 min_x 0.372→0.472、
    #     BuretteAcid 心 x 0.375→0.475（tmp 真值 +0.10，见 shift_stand_burette）
    sb_i = _measure(stage, "/root/IronStand")
    sb_b = _measure(stage, "/root/BuretteAcid")
    if sb_i and sb_b:
        iron_min_x = sb_i[0][0]
        bure_cx = (sb_b[0][0] + sb_b[1][0]) / 2.0
        s_ok = abs(iron_min_x - 0.472) < 0.01 and abs(bure_cx - 0.475) < 0.01
        ok &= s_ok
        print("[verify] IronStand min_x=%.3f (期望0.472) BuretteAcid 心x=%.3f (期望0.475) %s"
              % (iron_min_x, bure_cx, "OK" if s_ok else "FAIL"))

    # 6) 灯逐值（含 authored extent —— RTX 光体有效关键）
    env = stage.GetPrimAtPath("/root/env_light")
    tex = UsdLux.DomeLight(env).GetTextureFileAttr().Get() if env.IsValid() else None
    good_env = env.IsValid() and tex is not None \
        and str(tex).strip("@").endswith("env_bright.png") \
        and abs(UsdLux.DomeLight(env).GetIntensityAttr().Get() - LIGHT_DOME_INTENSITY) < 1.0
    ok &= good_env
    print("[verify] env_light DomeLight I=%s tex=%s %s"
          % (UsdLux.DomeLight(env).GetIntensityAttr().Get() if env.IsValid() else None,
             tex, "OK" if good_env else "FAIL"))
    cyl = stage.GetPrimAtPath("/root/CylinderLight")
    if cyl.IsValid() and cyl.IsA(UsdLux.CylinderLight):
        intens = UsdLux.CylinderLight(cyl).GetIntensityAttr().Get()
        rad = UsdLux.CylinderLight(cyl).GetRadiusAttr().Get()
        wx = UsdGeom.XformCache().GetLocalToWorldTransform(cyl).ExtractTranslation()[0]
        ext = cyl.GetAttribute("extent").Get()
        ext_ok = ext is not None and tuple(ext[0]) == (-5.0, -5.0, -50.0) \
            and tuple(ext[1]) == (5.0, 5.0, 50.0)
        good_cyl = abs(intens - LIGHT_KEY_INTENSITY) < 1.0 and rad == 5.0 \
            and abs(wx - 2.1) < 0.05 and ext_ok
        ok &= good_cyl
        print("[verify] key CylinderLight I=%.0f r=%.1f world_x=%.3f extent=%s %s"
              % (intens, rad, wx, ext, "OK" if good_cyl else "FAIL"))

    print("[verify] PASS" if ok else "[verify] FAIL")
    assert ok, "verify FAIL"


def main():
    assert os.path.exists(TMP), "缺 d1_tmp.usd: %s" % TMP

    # 第一遍：塌平 + 清垃圾（纯层导出到 OUT，不回写 tmp）
    layer = Sdf.Layer.FindOrOpen(os.path.abspath(TMP))
    assert not layer.subLayerPaths, "d1_tmp 带 subLayers，需先烤平: %s" % layer.subLayerPaths
    collapse_and_clean(layer)
    layer.Export(os.path.abspath(OUT))
    print("[save] 塌平层 ->", OUT)

    # 第二遍：重开 OUT 为 stage，玻璃/塞子/效果/灯，压平写回 OUT（覆盖中间态）
    st = Usd.Stage.Open(os.path.abspath(OUT))
    glassify(st)
    relocate_stopper(st)
    shift_stand_burette(st, 0.10)   # 铁架台+酸式滴定管整体 +X 10cm（用户要求）
    add_effects(st)
    fix_lights(st)
    verify(st)
    st.GetRootLayer().Save()
    print("[save] OUT(玻璃+效果+灯) =", OUT)

    # 第三遍：引用到 /World 的 task 路径断言
    verify_ref_world(OUT)
    print("\nDONE: ", OUT)


if __name__ == "__main__":
    main()
