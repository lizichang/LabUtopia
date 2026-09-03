# -*- coding: utf-8 -*-
"""D5 蒸馏分离 · 定稿场景烘焙（忠实 tmp 原坐标，单产出）。

输入：用户 Isaac 搭好的 d5_distillation_tmp.usd（defaultPrim=/root，几何压平自带材质）。
处理：
  1) 塌平嵌套 /root/World Scope —— 其子级(六件加热集群+table+Match+锥形瓶+主光)用
     Sdf.CopySpec 提到 /root 直下，删除空 Scope。
     （所有材质绑定目标都在 /root/_materials 下，CopySpec 保持绝对目标路径不变，
       引用到运行 /World 时由 USD 把层内 /root→/World 自动重映射，无需改字符串。）
  2) 清碎片：删除名字含 '_materials' 且不含任何 Material/Mesh 后代的空 Xform。
  3) 按实测坐标补 runtime 现象 prim（task.py 驱动）：烧瓶内气泡组(隐藏)、冷凝管出口
     液滴串(隐藏)、接液瓶内馏出液柱(隐藏)、酒精灯火焰外/内焰(隐藏)。
     —— 烧瓶样品液+沸石 tmp 已内建，不动。
  4) 保留 defaultPrim 名字 'root'（改名会打断绑定），内容全为 defaultPrim 子级，
     运行 main.py add_reference 到 /World 后即得 /World/<内容>。
输出：assets/scenes/d_wetchem/d5_distillation/d5_distillation.usd（覆盖弃用的 V1 预组装版）。

verify：断言 task 引用路径在"引用到 /World 的临时台"上存在且材质不悬挂，
        打印各件世界 bbox 与推荐同步的现象常量（跑阶段 task.py/constants.py 再对齐）。

用法：python scripts/gen_d5_final.py   （labutopia env 有 pxr）
"""
import os
import random
import math
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

SCENE_DIR = os.path.join("assets", "scenes", "d_wetchem", "d5_distillation")
TMP = os.path.join(SCENE_DIR, "d5_distillation_tmp.usd")
OUT = os.path.join(SCENE_DIR, "d5_distillation.usd")

TABLE_TOP = 0.80

# 光照强度（D5 组装件材质吃光比 B5/D1 强；用户反馈 I12000+D2000 过曝 → 降到下值）。
# 想再微调：改这里两个常量后重跑本脚本 + python main.py 即可，勿手改 usd。
LIGHT_KEY_INTENSITY = 7000.0
LIGHT_DOME_INTENSITY = 600.0

# 现象几何参数（沿用 gen_d5_scene.py 尺寸）
BUBBLE_R = 0.002
N_BUBBLES = 30
DROP_BALL_R = 0.003
N_DROPS = 8
RECV_LIQ_R = 0.020
FLAME_OUTER_R = 0.009
FLAME_INNER_R = 0.005

# 温度计红柱初始液位（gen 烘焙态 = 室温；运行时 task 按温度每帧重写）。
# 红毛细管 mesh 局部 extent z[0.006,0.2533]（全高=110°C）；0° 刻度在底上方 52.4mm、
# 每 °C 升 1.765mm（tmp 刻度实测 linear）→ 室温 25°C: (0.0524+25×0.001765)/0.2473 ≈ 0.39。
RED_INIT_FRAC = 0.39
RED_Z0_MARK_OFF = 0.0524      # 红底到 0° 刻度 (m)
RED_M_PER_DEG = 0.001765      # 每 °C 红柱上升 (m)


# ----------------------------------------------------------------------
# 1) 塌平 /root/World 空壳 + 清 _materials 碎片（纯 Sdf，单层）
# ----------------------------------------------------------------------
def collapse_and_clean(layer):
    root_spec = layer.GetPrimAtPath("/root")
    assert root_spec, "tmp defaultPrim 不是 /root"
    target = None
    for cname, cs in list(root_spec.nameChildren.items()):
        type_name = getattr(cs, "typeName", None) or ""
        if cname == "World" and type_name in ("", "Scope", "Xform"):
            has_xform = "xformOpOrder" in getattr(cs, "attributes", {})
            assert not has_xform, \
                "/root/World 带非恒等变换，上提子级会改世界位姿（忠实 tmp 前提破坏）"
            target = (cname, cs)
            break
    if target is None:
        raise RuntimeError("未找到可塌平的 /root/World（Xform/Scope，恒等）")
    scope_name, scope_spec = target
    n = 0
    for cname in list(scope_spec.nameChildren.keys()):
        src = "/root/%s/%s" % (scope_name, cname)
        dst = "/root/%s" % cname
        if dst in root_spec.nameChildren:
            print("[collapse] 冲突 %s 已存在，跳过 %s" % (dst, src))
            continue
        Sdf.CopySpec(layer, src, layer, dst)
        n += 1
    del root_spec.nameChildren[scope_name]
    print("[collapse] /root/World -> %d 个子级上提到 /root，空 Scope 已删" % n)
    return n


def drop_inert_materials(layer):
    """删除不含任何 Material/Mesh/light 后代的 _materials* 空 Xform（Blender 导入碎片）。"""

    def has_content(path):
        sp = layer.GetPrimAtPath(path)
        if sp is None:
            return False
        if sp.typeName in ("Material", "Mesh", "Sphere", "Cylinder", "Cone",
                           "DomeLight", "SphereLight", "CylinderLight", "RectLight"):
            return True
        for cn in sp.nameChildren.keys():
            if has_content(path + "/" + cn):
                return True
        return False

    root_spec = layer.GetPrimAtPath("/root")
    dropped = []
    for cname in list(root_spec.nameChildren.keys()):
        if not cname.startswith("_materials"):
            continue
        if not has_content("/root/" + cname):
            del root_spec.nameChildren[cname]
            dropped.append(cname)
    print("[clean] 删除空 _materials 碎片: %s" % (dropped if dropped else "无"))
    return dropped


# ----------------------------------------------------------------------
# 2) 现象 prim（建在 defaultPrim /root 直下，引用到 /World 即成 /World/*）
# ----------------------------------------------------------------------
def _measure(stage, path):
    """世界包围盒，纯手工（本 pxr 0.26.8 的 UsdGeom.BBoxCache 一律返回空，不可用）。

    遍历 prim 子树里 purpose=default 的 Mesh，取网格 extent(物体空间，缺则采样 points)，
    用其父链世界矩阵把盒子 8 角变换到世界并累积 min/max。返回 ((min),(max)) 或 None。
    """
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
    return mat


def add_shared_material(stage, mat_path, diffuse, opacity, prims, roughness=0.5, emissive=None):
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


def _add_droplet_flame_grp(stage, name, r, z_b, z_a, emissive, x, y):
    """水滴形火焰组 /root/<name>_grp：pivot=火焰底 (x,y,z_b)（B5/C3 同构，供 task 每帧 flicker）。

    组 op 序 translate→rotateXYZ→scale：底层 Xform 先 scale 后 rotate 再 translate 到 pivot，
    因此对火焰的每帧缩放(高/宽)+侧摆都绕"火焰底"发生，不漂移。task 的
    _apply_flame_flicker 只写组的 Scale op + RotateXYZ op；球/锥子 prim 用局部坐标，
    球心=底+r，锥从球顶收到 apex。初始全部叶子 prim MakeInvisible。
    """
    grp = UsdGeom.Xform.Define(stage, "/root/%s_grp" % name)
    grp.AddTranslateOp().Set(Gf.Vec3d(x, y, z_b))
    grp.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    grp.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    zc = r
    h = (z_a - z_b) - r
    sph = UsdGeom.Sphere.Define(stage, "/root/%s_grp/%s_sphere" % (name, name))
    sph.CreateRadiusAttr(r)
    UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, zc))
    cone = UsdGeom.Cone.Define(stage, "/root/%s_grp/%s" % (name, name))
    cone.CreateAxisAttr("Z")
    cone.GetHeightAttr().Set(h)
    cone.GetRadiusAttr().Set(r)
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, zc + h / 2.0))
    for prim in (sph, cone):
        pname = prim.GetPath().name
        mat = UsdShade.Material.Define(stage, "/root/%s_grp/%s_mat" % (name, pname))
        sh = UsdShade.Shader.Define(stage, "/root/%s_grp/%s_mat/Shader" % (name, pname))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.01, 0.01, 0.01))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(prim).Bind(mat)
        UsdGeom.Imageable(prim).MakeInvisible()
    print("[flame] grp %s pivot(%.4f,%.4f,%.4f) r%.3f apex%.4f (hidden)"
          % (name, x, y, z_b, r, z_a))
    return grp


def infer_effect_anchors(stage):
    """从烘焙后的场景实测几何推断现象坐标；打印给跑阶段对齐。"""
    a = {}
    a["fb"] = _measure(stage, "/root/DistillationFlask")
    a["liq"] = _measure(stage, "/root/DistillationFlask/SampleLiquid_001")
    a["lamp"] = _measure(stage, "/root/AlcoholLamp")
    a["gauze"] = _measure(stage, "/root/AsbestosGauze_001")
    a["recv"] = _measure(stage, "/root/conical_flask_77x77x97_001")
    a["toff"] = _measure(stage, "/root/take_off_tube")
    wick_box = None  # 灯芯件世界包围盒（XY 即灯焰轴线，比整灯 bbox 准——灯 bbox 含搁旁的灯帽）
    for sub in ("wick_top_001", "wick_tip_001", "wick_001"):
        b = _measure(stage, "/root/AlcoholLamp/" + sub)
        if b:
            wick_box = b
            break
    fb = a["fb"]
    a["flask_xy"] = (((fb[0][0] + fb[1][0]) / 2, (fb[0][1] + fb[1][1]) / 2)) if fb else None
    a["flask_bottom_z"] = fb[0][2] if fb else TABLE_TOP
    a["liq_top"] = a["liq"][1][2] if a["liq"] else (a["flask_bottom_z"] + 0.024)
    lamp = a["lamp"]
    # 灯焰 XY/芯顶 z：优先灯芯件；整灯 bbox 兜底（灯 bbox 可能含搁旁的灯帽而偏）
    if wick_box:
        a["wick_z"] = wick_box[1][2]
        a["lamp_xy"] = ((wick_box[0][0] + wick_box[1][0]) / 2,
                        (wick_box[0][1] + wick_box[1][1]) / 2)
    else:
        a["wick_z"] = lamp[1][2] if lamp else TABLE_TOP + 0.1
        a["lamp_xy"] = (((lamp[0][0] + lamp[1][0]) / 2,
                         (lamp[0][1] + lamp[1][1]) / 2)) if lamp else None
    recv = a["recv"]
    a["recv_xy"] = (((recv[0][0] + recv[1][0]) / 2, (recv[0][1] + recv[1][1]) / 2)) if recv else None
    a["recv_bottom_z"] = recv[0][2] if recv else TABLE_TOP
    a["recv_mouth_z"] = recv[1][2] if recv else (TABLE_TOP + 0.08)
    toff = a["toff"]
    a["toff_tip"] = ((toff[0][0], toff[0][1], toff[0][2])) if toff else None
    a["gauze_bottom_z"] = a["gauze"][0][2] if a["gauze"] else None
    return a


def add_effects(stage, a):
    """建 runtime 现象 prim（隐藏，待 task 驱动）。"""
    # 烧瓶内气泡组（30 球，绕烧瓶轴散布于液区，隐藏）
    bx, by = a["flask_xy"]
    bub_prims = []
    rng = random.Random(7)
    z0 = a["flask_bottom_z"] + 0.010
    for i in range(N_BUBBLES):
        rr = 0.020 * math.sqrt(rng.random())
        ang = 2.0 * math.pi * rng.random()
        sp = UsdGeom.Sphere.Define(stage, "/root/FlaskBubbles/bubble_%d" % i)
        sp.CreateRadiusAttr(BUBBLE_R)
        sp.AddTranslateOp().Set(Gf.Vec3d(bx + rr * math.cos(ang), by + rr * math.sin(ang), z0))
        UsdGeom.Imageable(sp).MakeInvisible()
        bub_prims.append(sp.GetPrim())
    add_shared_material(stage, "/root/FlaskBubbles/bubble_mat", (0.72, 0.85, 1.0), 1.0,
                        bub_prims, roughness=0.3, emissive=(0.7, 1.0, 1.8))
    UsdGeom.Imageable(UsdGeom.Xform.Define(stage, "/root/FlaskBubbles")).MakeInvisible()

    # 馏出液滴串（8 球，牛角管下尖端下缘，父组隐藏）
    if a["toff_tip"]:
        tx, ty, tz = a["toff_tip"]
        home = (tx, ty, tz - 0.006)
    else:
        home = (a["recv_xy"][0], a["recv_xy"][1], a["recv_mouth_z"] + 0.02)
    for i in range(N_DROPS):
        sp = UsdGeom.Sphere.Define(stage, "/root/DistillateDrop/Drop_%d" % i)
        sp.CreateRadiusAttr(DROP_BALL_R)
        sp.AddTranslateOp().Set(Gf.Vec3d(*home))
        # 馏出液 = 无色透明水（用户反馈蓝液不符；淡水蓝 diffuse + 低 opacity 透出瓶内光）
        add_material(stage, sp.GetPrim(), (0.95, 0.98, 1.0), 0.55, roughness=0.02, ior=1.33)
        UsdGeom.Imageable(sp).MakeInvisible()
    UsdGeom.Imageable(UsdGeom.Xform.Define(stage, "/root/DistillateDrop")).MakeInvisible()

    # 接液瓶内馏出液（h0，隐藏，task 逐滴生长）= 无色透明水（用户反馈蓝液不符；
    # 真水在 RTX = 低 opacity 高透明；微冷蓝 diffuse 只是让液面在透明玻璃里读得出高度）
    rx, ry = a["recv_xy"]
    rl = UsdGeom.Cylinder.Define(stage, "/root/ReceivingLiquid")
    rl.CreateRadiusAttr(RECV_LIQ_R)
    rl.CreateHeightAttr(0.0)
    rl.CreateAxisAttr("Z")
    rl.AddTranslateOp().Set(Gf.Vec3d(rx, ry, a["recv_bottom_z"]))
    add_material(stage, rl.GetPrim(), (0.93, 0.97, 1.0), 0.40, roughness=0.02, ior=1.33,
                 emissive=(0.02, 0.04, 0.09))
    UsdGeom.Imageable(rl).MakeInvisible()

    # 酒精灯火焰（外/内 = B5 水滴形火焰组，pivot=灯芯顶下 2mm；尖到石棉网底/内部短黄芯）
    # 用户「火焰应该像 B5 一样动起来」→ 必须组结构（task 每帧写组 scale/rotateXYZ flicker），
    # 散装 translate 火焰只能平移不能缩放成焰体。task 只认叶子路径见 constants.FLAME_PRIMS。
    lx, ly = a["lamp_xy"]
    base_z = a["wick_z"] - 0.002
    gauze_bottom = a["gauze_bottom_z"]
    apex_z = (gauze_bottom - 0.002) if (gauze_bottom and gauze_bottom - base_z > 0.02) \
        else base_z + 0.055
    _add_droplet_flame_grp(stage, "flame_outer", FLAME_OUTER_R, base_z, apex_z,
                           (0.35, 0.55, 2.40), lx, ly)
    _add_droplet_flame_grp(stage, "flame_inner", FLAME_INNER_R, base_z,
                           min(base_z + 0.030, apex_z), (2.80, 0.55, 0.20), lx, ly)

    print("[effects] FlaskBubbles/DistillateDrop/ReceivingLiquid/flame_grp 已建（初始隐藏）")
    return home


def fix_lights(stage):
    """修灯。历史：太暗 = 用户 tmp 主光是 radius0/extent0 的 SphereLight≈无光，env dome 是
    1×1 黑 EXR 强度1；对齐工作套路(B5/D1) 后过曝 = D5 组装件材质吃光强于 B5/D1。现取折中：
    /root/CylinderLight 标准圆柱主光（radius5 length100 extent[-5,-5,-50..5,5,50]
    intensity=LIGHT_KEY_INTENSITY @(2.1,1.0572,7)，orient(0.5,0.5,0.5,0.5)），
    env dome = env_bright.png 强度=LIGHT_DOME_INTENSITY。
    ⚠ RTX 下灯必须带 authored extent，否则光体无效≈无光（D1 教训）。"""
    # 1) 删假主光壳（Xform + 内嵌 SphereLight）
    shell = stage.GetPrimAtPath("/root/CylinderLight_001")
    if shell.IsValid():
        stage.RemovePrim("/root/CylinderLight_001")
        print("[light] removed fake key shell /root/CylinderLight_001 (SphereLight r0/ext0)")
    else:
        print("[light] no fake key shell found")
    # 2) 建标准圆柱主光
    cyl = UsdLux.CylinderLight.Define(stage, "/root/CylinderLight")
    cyl.GetIntensityAttr().Set(LIGHT_KEY_INTENSITY)
    cyl.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    cyl.GetRadiusAttr().Set(5.0)
    cyl.GetLengthAttr().Set(100.0)
    ext_attr = cyl.GetPrim().CreateAttribute("extent", Sdf.ValueTypeNames.Float3Array)
    ext_attr.Set([Gf.Vec3f(-5, -5, -50), Gf.Vec3f(5, 5, 50)])
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(2.1, 1.057155977178849, 7.0))
    xf.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, 0.5, 0.5)))
    xf.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    print("[light] /root/CylinderLight r5 l100 ext[-5..5,-50..50] I%.0f @(2.1,1.057,7)"
          % LIGHT_KEY_INTENSITY)
    # 3) env dome → env_bright.png
    env = stage.GetPrimAtPath("/root/env_light")
    if env.IsValid() and env.IsA(UsdLux.DomeLight):
        dome = UsdLux.DomeLight(env)
        dome.GetIntensityAttr().Set(LIGHT_DOME_INTENSITY)
        dome.GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
        print("[env] /root/env_light -> textures/env_bright.png, intensity %.0f"
              % LIGHT_DOME_INTENSITY)
    else:
        print("[env] WARN /root/env_light DomeLight 缺失")


def fix_glass_and_red(stage):
    """玻璃透明化 + 温度计红柱压到室温（用户 2026-09-03 反馈）。

    ① 蒸馏烧瓶"完全白"= tmp glass_005 opacity 1.0。烧瓶是**开敞泡体**（大颈开口、薄壁），
    RTX 下低 opacity alpha 直接透 → 压 0.20 即透（用户已确认烧瓶对了，勿动）。
    ② 锥形接液瓶 = **大封闭壳**（窄口、厚曲面）。曾走"透射配方" opacity1+transmission1，
    但 transmission=1 在这台 RTX 实测=完全不透明实心（D1 已推翻，见 d1-flask-mirror 记忆）
    → 回 **alpha 配方**：opacity 0.20 + specular 压 0.2 去镜面反光 + rough 0.10
    （D1 锥形瓶最终同款修法），瓶内无色馏出液液面/滴落才透得出。
    ③ 温度计"一开始就 100°C"= 红毛细管是整高静态 mesh（顶=110°C 刻度），无任何 xform
    op 控制液位。烘焙态压到~室温：给 capillary_liquid_001 Xform 加 (translate,scale) 两 op，
    绕底 pivot（mesh extent 底 z0）按 f=RED_INIT_FRAC 缩，底部用 translate.z=z0(1-f) 钉住
    不抬升。运行时 task 温度模型再每帧重写这两 op 让红柱随点燃缓慢上升。
    """
    # 1) 玻璃透明（改 tmp 自带材质 UsdPreviewSurface 着色器输入，直接覆盖；两件配方不同）
    specs = {
        # 开敞泡体 → alpha 半透明（用户已确认对，勿动）
        "/root/_materials/glass_005": dict(opacity=0.20, roughness=0.02),
        # 大封闭壳 → 回 alpha 配方：transmission=1 这台 RTX 实测=实心不透明（D1 已推翻），
        #   必须 opacity<1 + 压 spec 去镜面反光，照 d1-flask-mirror 最终修法
        "/root/_materials/conical_flask_77x77x97_glass_001": dict(opacity=0.20, roughness=0.10,
                                                                   specular=0.2),
    }
    for mpath, s in specs.items():
        mp = stage.GetPrimAtPath(mpath)
        if not mp.IsValid():
            print("[glass] WARN %s 缺失" % mpath)
            continue
        sh = next((c for c in mp.GetChildren() if c.GetTypeName() == "Shader"), None)
        if sh is None:
            print("[glass] WARN %s 无 Shader 子级" % mpath)
            continue
        shader = UsdShade.Shader(sh)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(s["opacity"])
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)   # 去镜面
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(s["roughness"])
        if s.get("specular") is not None:
            shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(s["specular"])
        if s.get("transmission") is not None:
            shader.CreateInput("transmission", Sdf.ValueTypeNames.Float).Set(s["transmission"])
        elif shader.GetInput("transmission"):
            shader.RemoveInput("transmission")   # 清残留：transmission=1=实心不透明
        print("[glass] %s opacity->%.2f rough->%.2f spec->%s (metallic 0)"
              % (mpath, s["opacity"], s["roughness"], s.get("specular")))

    # 2) 温度计红柱压到室温
    rx = stage.GetPrimAtPath("/root/Thermometer_001/Thermometer/capillary_liquid_001")
    if not rx.IsValid():
        print("[red] WARN capillary_liquid_001 缺失")
        return
    child = next((c for c in rx.GetChildren() if c.GetTypeName() == "Mesh"), None)
    ext = None
    if child is not None:
        g = UsdGeom.Gprim(child)
        ext = g.GetExtentAttr().Get() if g.GetExtentAttr() else None
    if not ext or len(ext) != 2:
        print("[red] WARN 红柱 mesh 无 extent，跳过")
        return
    z0 = ext[0][2]
    red_len = ext[1][2] - z0
    f0 = RED_INIT_FRAC
    xf = UsdGeom.Xformable(rx)
    for o in list(xf.GetOrderedXformOps()):
        xf.RemoveXformOp(o)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z0 * (1.0 - f0)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, f0))
    print("[red] 红柱初始 f=%.3f (~室温25°C) pivot z0=%.4f L=%.4f (translate,z0(1-f)/scale.z=f)"
          % (f0, z0, red_len))


# ----------------------------------------------------------------------
# 3) verify：引用到 /World 的临时台断言 + 现象坐标建议
# ----------------------------------------------------------------------
def verify_ref_world(out_path):
    """在临时台把 out 引用到 /World，断言 task 需要的路径全部存在（含材质不悬挂）。"""
    st = Usd.Stage.CreateInMemory()
    w = st.DefinePrim("/World", "Xform")
    st.SetDefaultPrim(w)
    w.GetReferences().AddReference(os.path.abspath(out_path))
    need = ["DistillationFlask", "AlcoholLamp", "Match", "AsbestosGauze_001",
            "IronStand", "ClampCondenser", "Thermometer_001",
            "conical_flask_77x77x97_001", "take_off_tube", "condenser_cradle_stand",
            "table", "FlaskBubbles", "DistillateDrop", "ReceivingLiquid",
            "flame_outer_grp", "flame_outer_grp/flame_outer",
            "flame_outer_grp/flame_outer_sphere",
            "flame_inner_grp", "flame_inner_grp/flame_inner",
            "flame_inner_grp/flame_inner_sphere"]
    missing = [n for n in need if not st.GetPrimAtPath("/World/" + n).IsValid()]
    fb = st.GetPrimAtPath("/World/FlaskBubbles")
    dd = st.GetPrimAtPath("/World/DistillateDrop")
    n_bub = sum(1 for c in fb.GetChildren() if c.GetTypeName() == "Sphere") if fb.IsValid() else 0
    n_drop = sum(1 for c in dd.GetChildren() if c.GetTypeName() == "Sphere") if dd.IsValid() else 0
    flask_mesh = next((p for p in Usd.PrimRange(st.GetPrimAtPath("/World/DistillationFlask"))
                       if p.GetTypeName() == "Mesh"), None)
    mat_txt = None
    if flask_mesh:
        for p in Usd.PrimRange(st.GetPrimAtPath("/World/DistillationFlask")):
            if p.GetTypeName() != "Mesh":
                continue
            got = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
            src = got[0] if isinstance(got, (tuple, list)) else got
            if src is not None and hasattr(src, "GetPath"):
                txt = src.GetPath().pathString
                if txt.startswith("/World/"):
                    mat_txt = txt
                    break
    sample = [c.GetName() for c in st.GetPrimAtPath("/World").GetChildren()][:26]
    print("[verify-ref] /World children sample: %s" % sample)
    assert not missing, "task 路径缺失: %s" % missing
    assert n_bub == N_BUBBLES and n_drop == N_DROPS, "气泡%d/液滴%d 数量不符" % (n_bub, n_drop)
    assert mat_txt, "材质全悬挂或未解析（应至少一个解析到 /World/_materials）"
    print("[verify-ref] OK: 任务路径齐 / 气泡%d 液滴%d / 材质重映射 %s" % (n_bub, n_drop, mat_txt))


def main():
    os.makedirs(SCENE_DIR, exist_ok=True)
    assert os.path.exists(TMP), "缺少 %s" % TMP
    layer = Sdf.Layer.FindOrOpen(os.path.abspath(TMP))
    assert not layer.subLayerPaths, "tmp 带 subLayers，需先烤平: %s" % layer.subLayerPaths
    collapse_and_clean(layer)
    drop_inert_materials(layer)
    # 第一遍导出塌平结果到 OUT（纯层导出，不回写 tmp）；tmp 磁盘文件不触碰
    layer.Export(os.path.abspath(OUT))
    print("[save] 塌平层 ->", OUT)

    # 第二遍：重开 OUT 为 stage，实测几何补现象 prim，再压平写回 OUT（覆盖中间态）
    stage = Usd.Stage.Open(os.path.abspath(OUT))
    anchors = infer_effect_anchors(stage)
    add_effects(stage, anchors)
    fix_lights(stage)
    fix_glass_and_red(stage)
    stage.Export(OUT)
    print("[save] OUT(含现象 prim + 主光) =", os.path.abspath(OUT))

    # 现象常量打印（跑阶段对齐 task.py / constants.py 参考）
    a = anchors
    print("\n[效果坐标（供 task.py 现象常量对齐）]")
    if a["lamp_xy"]:
        print("  LAMP_XY        = (%.4f, %.4f)" % (a["lamp_xy"][0], a["lamp_xy"][1]))
    print("  FLASK_BOTTOM_Z = %.4f   (DistillationFlask 底)" % a["flask_bottom_z"])
    print("  SAMPLE_LIQ_TOP = %.4f   (SampleLiquid_001 顶 = 气泡消失面)" % a["liq_top"])
    print("  WICK_Z         = %.4f   (酒精灯芯顶 = 点火/火焰底参考)" % a["wick_z"])
    if a["recv_xy"]:
        print("  RECV_XY / RECV_BOTTOM_Z = (%.4f, %.4f) / %.4f"
              % (a["recv_xy"][0], a["recv_xy"][1], a["recv_bottom_z"]))
    if a["toff_tip"]:
        print("  DROP_HOME(take_off 下尖) = (%.4f, %.4f, %.4f)"
              % (a["toff_tip"][0], a["toff_tip"][1], a["toff_tip"][2]))

    st2 = Usd.Stage.Open(os.path.abspath(OUT))
    print("\n[verify 定稿场景 bbox（手工网格测量）]")
    for name in ("DistillationFlask", "AlcoholLamp", "Match", "AsbestosGauze_001",
                 "IronStand", "ClampCondenser", "Thermometer_001",
                 "conical_flask_77x77x97_001", "take_off_tube",
                 "condenser_cradle_stand", "rubber_stopper", "table"):
        r = _measure(st2, "/root/" + name)
        if not r:
            print("  %-24s (无网格/空)" % name)
            continue
        mn, mx = r
        print("  %-24s min(%+.4f,%+.4f,%+.4f) max(%+.4f,%+.4f,%+.4f)"
              % (name, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]))
    # 灯自检（太暗修复回归）
    cp = st2.GetPrimAtPath("/root/CylinderLight")
    assert cp.IsValid() and cp.IsA(UsdLux.CylinderLight), "主光缺失/类型错"
    assert UsdLux.CylinderLight(cp).GetRadiusAttr().Get() == 5.0
    ext = cp.GetAttribute("extent").Get()
    assert ext and not (ext[0] == ext[1]), "主光缺 authored extent（RTX 光体无效→黑）"
    assert abs(UsdLux.CylinderLight(cp).GetIntensityAttr().Get() - LIGHT_KEY_INTENSITY) < 1.0
    ep = st2.GetPrimAtPath("/root/env_light")
    assert ep.IsValid() and ep.IsA(UsdLux.DomeLight), "env dome 缺失"
    assert abs(UsdLux.DomeLight(ep).GetIntensityAttr().Get() - LIGHT_DOME_INTENSITY) < 1.0
    tex = UsdLux.DomeLight(ep).GetTextureFileAttr().Get()
    assert tex and "env_bright" in tex.path, "dome 未指向 env_bright.png"
    print("[light-verify] OK CylinderLight r5/ext/l100/I%.0f + DomeLight env_bright@%.0f"
          % (LIGHT_KEY_INTENSITY, LIGHT_DOME_INTENSITY))

    # 玻璃透明化 + 红柱室温回归（烧瓶/锥形瓶皆 alpha 配方，锥形瓶另压 spec；无 transmission）
    chk = {"/root/_materials/glass_005": dict(opacity=0.20),
           "/root/_materials/conical_flask_77x77x97_glass_001": dict(opacity=0.20, specular=0.2)}
    for mpath, want in chk.items():
        mp = st2.GetPrimAtPath(mpath)
        assert mp.IsValid(), "材质 %s 缺失" % mpath
        sh = next((c for c in mp.GetChildren() if c.GetTypeName() == "Shader"), None)
        assert sh is not None, "材质 %s 无 Shader" % mpath
        sd = UsdShade.Shader(sh)
        op = sd.GetInput("opacity")
        got = op.Get() if op else None
        assert got is not None and abs(got - want["opacity"]) < 0.05, \
            "玻璃 opacity 未生效 %s=%.2f (want %.2f)" % (mpath, got, want["opacity"])
        if want.get("specular") is not None:
            sp = sd.GetInput("specular")
            gsp = sp.Get() if sp else None
            assert gsp is not None and abs(gsp - want["specular"]) < 0.05, \
                "玻璃 specular 未生效 %s=%.2f (want %.2f)" % (mpath, gsp, want["specular"])
        tr = sd.GetInput("transmission")
        assert tr is None or tr.Get() is None, \
            "玻璃不得残留 transmission 配方(这台机器=实心不透明) %s" % mpath
    rp = st2.GetPrimAtPath("/root/Thermometer_001/Thermometer/capillary_liquid_001")
    rops = UsdGeom.Xformable(rp).GetOrderedXformOps() if rp.IsValid() else []
    assert len(rops) >= 2, "红柱 xform 缺 translate/scale op"
    assert abs(UsdGeom.Xformable(rp).GetOrderedXformOps()[0].Get()[-1]) > 0, "红柱未绕底钉 pivot"
    print("[glass-red-verify] OK glass_005/conical opacity0.20 + 红柱 translate/scale op 在")

    verify_ref_world(OUT)
    print("\nDONE: ", OUT)


if __name__ == "__main__":
    main()
