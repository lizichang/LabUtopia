# -*- coding: utf-8 -*-
"""生成 d3l_acid_reagent.usd —— D3-L 酸性试剂滴加反应（液体样品）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，无器材，defaultPrim=/World）：
- 直接引用 assets/equipment/ 真实器材（lab_clean 干净，无需删任何 prim、无需抬台面）
- 试管 + 两支胶头滴管插进试管架孔里（底面 z=0.806 = 架z−0.0905，孔心对齐顶层板孔位）
- 去瓶塞（样品/酸瓶已开瓶，瓶口 rim=0.070 → 世界 0.870）+ 去试管架残留 env_light
- 内建效果 prim：SampleLiquid（样品瓶体积）/AcidLiquid（酸瓶体积，可见）
  /TubeDrops（管内液滴）/Bubbles（气泡）/Precipitate（沉淀），后三初始隐藏（task 动画驱动）
- 最逼真液体配方：roughness 0.05 光洁水面 + ior 1.33 水折射 + opacity 0.45 + doubleSided
  （参考 lab_003 酒精灯独立 liquid mesh 方案，但比它的灰色不透明 op1.0 更像水）
- hcl_bottle.usd 自带 liquid 实测是 1mm 厚薄盘（z 0.040..0.041，非半瓶）——隐藏它，
  改由 AcidLiquid 体积圆柱表现酸液（更真实）
- 瓶玻璃透明化（assets 的 bottle_mat 是 op 0.8/rough 0.33 磨砂玻璃，隔它看不清液体）
  → op 0.25 / rough 0.1 / ior 1.5 真玻璃

布局（V7：试管架中央/样品瓶正后方 10cm/HCl 左前方 15cm/两支滴管插架孔）：
  TestTubeRack  (0.30,  0.00)  底座贴台面 z=0.8965
  TestTube      (0.2787, 0.1193, 0.806)  前排左孔（d2s 校准坐标）
  DropperSample (0.281, 0.0788, 0.806)   2 排左孔 立放
  DropperAcid   (0.319, 0.0788, 0.806)   2 排右孔 立放
  SampleBottle  (0.4045, 0.3585)  样品瓶（用户调整：台面前方偏右），底座贴台面
  HClBottle     (0.1696, 0.361)   HCl 试剂瓶（用户调整：台面前方偏左），底座贴台面

用法：python scripts/gen_d3l_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import math
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d3l_acid_reagent")
OUT = os.path.join(SCENE_DIR, "d3l_acid_reagent.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# 孔位（skill 坑 33 + 实测）：顶层板 14 孔 2 列×7 行，孔心列 x=架x±0.019；插孔底面 z=架z−0.0905
HOLE_BOTTOM = 0.806  # = 架 translate z(0.8965) − 0.0905

# (prim, asset_file, translate, scale)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.00, None), None),
    ("TestTube", "test_tube.usd", (0.2787, 0.1193, HOLE_BOTTOM), None),
    ("DropperSample", "dropper.usd", (0.281, 0.0788, HOLE_BOTTOM), None),
    ("DropperAcid", "dropper.usd", (0.319, 0.0788, HOLE_BOTTOM), None),
    ("SampleBottle", "sample_bottle.usd", (0.4045, 0.3585, None), None),
    ("HClBottle", "hcl_bottle.usd", (0.1696, 0.361, None), None),
]

# 液体材质配方（最逼真）：roughness 0.05 光洁水面 + ior 1.33 水折射 + opacity 0.70
# （2026-08-14 用户反馈"液体痕迹不明显"：0.45 太透、隔着玻璃看不清 → 提到 0.70+提亮）
# acid 微绿区分酸液/水样
WATER = dict(color=(0.58, 0.78, 0.98), opacity=0.70, roughness=0.05, ior=1.33)
ACID = dict(color=(0.66, 0.86, 0.76), opacity=0.70, roughness=0.05, ior=1.33)
OPAQUE_WHITE = dict(color=(0.93, 0.93, 0.94), opacity=1.0, roughness=0.5)

# 内建效果 prim: (name, type, radius, height, translate, 材质配方 dict, visible)
# SampleLiquid = 样品瓶内半瓶液体（cyl 从瓶底 0.80 到液面 0.84）
# AcidLiquid   = 酸瓶内半瓶液体（hcl 资产自带 1mm 薄盘被隐藏，改用真体积）
# TubeDrops/Precipitate = 管内液滴/沉淀；Bubbles 单独建（小球簇）
# DropperFill = 滴管尖内吸起的液体柱（skill 坑 18：尖端容器内腔是锥形，直圆柱会悬空穿模）。
#   滴管玻璃体实测：尖嘴 Ø1.6mm(z=0) → 30mm 处 Ø8mm → 直管 Ø8mm 到胶头。吸液后液体填满
#   收窄尖端，故用截锥 mesh：下底 Ø2mm(尖) → 上底 Ø8mm(体)，高 40mm（覆盖收窄段 0..30mm
#   + 直管 10mm）。柱底贴尖嘴（task 逐帧跟随），初始隐藏。
# TubeDrops = 管内液体（0.008→0.009 贴管壁 Ø19.2 内缘、0.020→0.030 更高更显眼）
EFFECTS = [
    ("SampleLiquid", "cylinder", 0.014, 0.040, (0.4045, 0.3585, 0.820), WATER, True),
    ("AcidLiquid", "cylinder", 0.014, 0.040, (0.1696, 0.361, 0.820), ACID, True),
    ("TubeDrops", "cylinder", 0.009, 0.030, (0.2787, 0.1193, 0.821), WATER, False),
    ("Precipitate", "cylinder", 0.008, 0.003, (0.2787, 0.1193, 0.8075), OPAQUE_WHITE, False),
    # frustum 的 r = (r_bottom, r_top)：下底 Ø2mm 贴尖嘴、上底 Ø8mm 贴玻璃体。translate
    # 是 mesh 底心（底在局部 z=0）→ 落在尖嘴 z=0.806，柱体 0.806..0.846 整体在玻璃体
    # 0..0.12 内、不露在尖嘴外（task._set_fill_follow 用同一约定：translate=尖嘴）
    ("DropperFill", "frustum", (0.001, 0.004), 0.040, (0.281, 0.0788, 0.806), WATER, False),
]
# 气泡：试管内液体区 5 颗小白球（世界坐标，r=0.002，均在管壁 Ø19.2 内）
BUBBLES = [
    (0.2807, 0.1203, 0.812), (0.2767, 0.1213, 0.818), (0.2797, 0.1173, 0.822),
    (0.2777, 0.1193, 0.815), (0.2827, 0.1183, 0.820),
]
BUBBLE_R = 0.002


def add_material(stage, prim, diffuse, opacity, roughness=0.5, ior=None, double_sided=False):
    """UsdPreviewSurface 材质。透材质（opacity<1）自动设 doubleSided，
    否则从外看透过液体时后壁不渲染会像空容器。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if ior is not None:
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
    if double_sided and prim.IsA(UsdGeom.Gprim):
        UsdGeom.Gprim(prim).CreateDoubleSidedAttr().Set(True)


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (f" scale {scale}" if scale else ""))


def add_frustum(stage, name, r_bottom, r_top, h):
    """截锥 mesh（锥台）：下底 r_bottom、上底 r_top、高 h，底心在原点，+Z 向上。

    skill 坑 18：收口/尖端容器（滴管、锥形瓶…）内腔是锥形，直圆柱液体会悬空穿模
    （液面不贴壁、露出空隙），须用锥台贴合。下底 r=底部内半径、上底 r=液面高度处内半径。
    16 段圆周 + 底/顶 cap，subdivisionScheme=none（memory: 自建 mesh 必须设，否则不显色）。
    """
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


def add_effects(stage):
    for name, kind, r, h, t, m, visible in EFFECTS:
        if kind == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{name}")
            geom.CreateRadiusAttr(r)
            geom.CreateHeightAttr(h)
            geom.CreateAxisAttr("Z")
        elif kind == "frustum":
            r_bottom, r_top = r
            geom = add_frustum(stage, name, r_bottom, r_top, h)
        else:
            geom = UsdGeom.Sphere.Define(stage, f"/World/{name}")
            geom.CreateRadiusAttr(r)
        geom.AddTranslateOp().Set(Gf.Vec3d(*t))
        translucent = m.get("opacity", 1.0) < 1.0
        add_material(stage, geom.GetPrim(), m["color"], m["opacity"],
                     roughness=m.get("roughness", 0.5), ior=m.get("ior"),
                     double_sided=translucent)
        if not visible:
            UsdGeom.Imageable(geom).MakeInvisible()
        print(f"[effect] {name} {'visible' if visible else 'hidden'} at {t} "
              f"(op {m['opacity']} rough {m.get('roughness', 0.5)} ior {m.get('ior')})")


def add_bubbles(stage):
    g = UsdGeom.Xform.Define(stage, "/World/Bubbles")
    for i, (x, y, z) in enumerate(BUBBLES):
        s = UsdGeom.Sphere.Define(stage, f"/World/Bubbles/Bubble_{i}")
        s.CreateRadiusAttr(BUBBLE_R)
        s.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        add_material(stage, s.GetPrim(), (0.9, 0.9, 0.9), 0.9)
    UsdGeom.Imageable(g).MakeInvisible()
    print(f"[effect] Bubbles hidden ({len(BUBBLES)} spheres)")


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃滴管/试管在无环境反射下照不亮。
    贴图路径用相对 ./textures/ 会在 stage.Export 时按 lab_clean 层解析成不存在路径，
    烘平后由 fix_env_light() 在场景层重新指向 textures/env_bright.png。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件
    （d2s 药匙反黑经验：金属/玻璃细杆需 12000）。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效），
    烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向场景目录下的贴图。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_stoppers(st2):
    """去瓶塞：实验已开瓶，删样品/酸瓶自带的 stopper + stopper_mat
    （覆盖在瓶口上 0.068..0.079，删后瓶口 rim=0.070 → 世界 0.870）。

    注意：stage.Export 烘平会把引用资产的 root 包装 Xform 合并进引用 prim
    （不是 /World/<瓶>/root/stopper，而是 /World/<瓶>/stopper），故用遍历匹配。"""
    for name in ("SampleBottle", "HClBottle"):
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


def remove_rack_env_light(st2):
    """试管架资产自带 flametest 残留 DomeLight（root/TestTubeRack/env_light），
    会与场景 env_light 双灯，删掉。"""
    rack = st2.GetPrimAtPath("/World/TestTubeRack")
    if not rack.IsValid():
        print("[clean] /World/TestTubeRack not found, skip")
        return
    paths = [p.GetPath() for p in Usd.PrimRange(rack)
             if p.GetTypeName() == "DomeLight" or "env_light" in p.GetName()]
    for path in paths:
        st2.RemovePrim(path)
        print(f"[clean] removed rack light {path}")
    if not paths:
        print("[clean] no DomeLight in TestTubeRack")


def relocate_absolute_textures(st2):
    """坑 12：烘平后材质贴图 asset 属性若为仓库内绝对路径 → 改为场景相对路径。
    env_bright.png 已由 fix_env_light 指向场景相对，这里兜底器材自带纹理。"""
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


def verify(st2):
    """自检：打印各器材/效果世界 bbox，确认孔位/瓶口高度符合设计。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["TestTubeRack", "TestTube", "DropperSample", "DropperAcid",
             "SampleBottle", "HClBottle", "SampleLiquid", "AcidLiquid",
             "TubeDrops", "Precipitate", "Bubbles", "DropperFill"]
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {name:15s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")


# 瓶玻璃配方：assets 自带 bottle_mat 是 op0.8/rough0.33 磨砂玻璃，隔它看不清液体
# → 真玻璃 op 0.25 / rough 0.1 / ior 1.5，液体才透得出来
GLASS = dict(diffuseColor=(0.85, 0.90, 0.95), opacity=0.25, roughness=0.10, ior=1.5)


def override_bound_shader(st2, prim, recipe):
    """重写 prim 绑定材质的 shader 参数。烘平后材质绑定在 mesh prim 上但
    MaterialBindingAPI 未 apply（会告警），故直接用 material:binding relationship
    取材质路径再找 shader。"""
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
    """瓶玻璃透明化 + 酸瓶 1mm 液面盘隐藏（改用 AcidLiquid 体积表现）。

    hcl_bottle 自带 liquid 实测 z 0.040..0.041 = 1mm 厚薄盘（非半瓶），
    视觉上只是一片膜，隐藏掉；酸液由内建 AcidLiquid 圆柱（0.80..0.84）表现。"""
    for name in ("SampleBottle", "HClBottle"):
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[mat] /World/{name} not found, skip")
            continue
        for c in p.GetChildren():
            if c.GetTypeName() != "Mesh":
                continue
            if c.GetName() == "liquid":
                UsdGeom.Imageable(c).MakeInvisible()
                print(f"[mat] hid {c.GetPath()} (1mm liquid disc, replaced by AcidLiquid)")
            else:
                if override_bound_shader(st2, c, GLASS):
                    UsdGeom.Gprim(c).CreateDoubleSidedAttr().Set(True)


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale in EQUIP:
        add_equip(stage, name, asset, t, scale)
    add_effects(stage)
    add_bubbles(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_stoppers(st2)
    remove_rack_env_light(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    relocate_absolute_textures(st2)
    fix_bottle_materials(st2)   # 瓶玻璃透明化 + 酸瓶 1mm 液面盘隐藏（AcidLiquid 取代）
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
