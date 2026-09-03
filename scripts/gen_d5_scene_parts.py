# -*- coding: utf-8 -*-
"""生成 d5_distillation_parts.usd —— D5 蒸馏分离「散件摆桌版」场景（人工组装用，烘平自包含）。

背景：文档 D11 注「装置组装建议人工完成、机械臂仅加热收集」。预组装成品（gen_d5_scene.py
产 d5_distillation.usd）被用户否掉后走「我摆散件、你组装」。2026-09-02 用户在 Isaac 里把
核心加热台（铁架台 / 石棉网 / 温度计 / 蒸馏烧瓶 / 酒精灯(帽单搁) / 试管夹 六件）相对位置
调好并存到 d5_distillation_tmp.usd；本脚本把该六件**逐字照抄**成已就位的冻结集群，再补上
被孤儿化看不见的冷凝管 + 冷凝管夹（松散件）让用户手动连成联通装置。改完的散件场景会
从 lab_clean 0.80 桌面重建，用户在原 Isaac 里重开本文件继续组装。

照抄来源（tmp 实测世界坐标，冻结——用户要求不得改六件相对位置，绝对位置可微调但此处照抄）：
  集群在桌面左侧 x≈-0.55..-0.38, y≈0.08..0.10：
    AlcoholLamp     (-0.4513, 0.0980, 0.8002) rotZ180（帽单独搁：cap 子件局部
                    translate (0.0012,0.1926,-0.0808) 翻放灯旁）
    IronStand       (-0.5500, 0.1000, 0.8000)         铁架台（base 贴台 0.80，杆高至 1.26）
    AsbestosGauze   (-0.4513, 0.0973, 0.9199)         石棉网（已抬到环高 ~0.92）
    Thermometer     (-0.4579, 0.0988, 0.9480)         温度计（泡尖探入烧瓶）
    DistillationFlask(-0.4580,0.0992, 0.9091)         蒸馏烧瓶（底悬 0.909，预装液+沸石）
    ClampCondenser  (-0.5080, 0.0789, 0.8144)         试管夹=夹烧瓶颈连铁架台；
                     爪头子件 polySurface7_Black_0_002 局部 z 抬 +0.21065 → 爪 1.01~1.04
松散补件（未连接，供用户抓取组装）：
    Condenser        (0.05,0.35) rotY90 横躺          直形冷凝管（已拆内部 Scope 修可见）
    CondenserClamp   (0.22,0.52)                      冷凝管夹·第2支撑（新增 test_tube_clamp）
    ReceivingFlask   (0.35,-0.35)                     接液锥形瓶
    Match            (0.40,-0.06,0.813)               火柴锚点（勿动）

冷凝管可见性修复根因：condenser_reflux.usd 的 body/plastic_top 两个 Mesh 直接挂在
Scope `condenser_reflux` 下（其余资产均 Xform 嵌套），Isaac 导入/压平时 Scope 被孤儿化，
body/plastic_top 被甩到 /root 顶层、/World/Condenser 成空壳 → 用户看不到也调不了。
修复 = 烘平后用 Sdf.CopySpec 把两个 Mesh 从 Scope 提到 /World/Condenser 直下（等效
conical 的 Xform→Mesh 结构），删除空 Scope，材质绑定路径保持不变。

台面：lab_clean 底场景 defaultPrim=/World；/World/Cube 台面顶 z=0.80（span x/y∈[-1,1]）。
用法：python scripts/gen_d5_scene_parts.py    （运行环境有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdLux, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d5_distillation")
OUT = os.path.join(SCENE_DIR, "d5_distillation_parts.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80

# —— 火柴锚点（机械臂点火坐标，勿动；本散件场景仅作定位参考，无机器人）——
MATCH_X, MATCH_Y, MATCH_T = 0.40, -0.06, 0.813

# —— 冻结六件（2026-09-02 用户 tmp 实测世界坐标，逐字照抄；相对位置不得改）——
# (prim, asset_file, world translate xyz, rotXYZ 角度, 说明)
FROZEN_SIX = [
    ("AlcoholLamp", "alcohol_lamp.usd",
     (-0.451284, 0.098029, 0.800180), (0, 0, 180),
     "酒精灯·冻结（帽单独搁见 cap 局部偏移）"),
    ("IronStand", "iron_stand.usd",
     (-0.550000, 0.100000, 0.800000), (0, 0, 0),
     "铁架台·冻结"),
    ("AsbestosGauze", "asbestos_gauze.usd",
     (-0.451349, 0.097299, 0.919890), (0, 0, 0),
     "石棉网·冻结（资产自带 scale0.7461，已抬到环高 0.92）"),
    ("Thermometer", "thermometer.usd",
     (-0.457934, 0.098766, 0.947976), (0, 0, 0),
     "温度计·冻结（整 defaultPrim 引用保红液材质）"),
    ("DistillationFlask", "distillation_flask.usd",
     (-0.457988, 0.099222, 0.909126), (0, 0, 0),
     "蒸馏烧瓶·冻结（底悬 0.909；预装样品液+沸石子prim）"),
    ("ClampCondenser", "test_tube_clamp.usd",
     (-0.508017, 0.078864, 0.814367), (0, 0, 0),
     "试管夹·冻结（夹烧瓶颈连铁架台；爪头已抬 +0.21065）"),
]
# 冻结件专属局部编辑（tmp 用户动作，照抄）：
CAP_ASIDE_TRANS = (0.001167, 0.192592, -0.080759)   # AlcoholLamp/cap 局部 translate（帽单搁）
CLAMP_JAW_Z = 0.210652                              # ClampCondenser/.../polySurface7_Black_0_002
                                                    #   局部 translate.z 额外抬升（爪到瓶颈高 1.02）

# —— 松散补件（add_part 自动贴台 0.80；冷凝管 rotY90 横躺）——
# (prim, asset_file, (x,y), rot, ref_path, 说明)
LOOSE_GROUNDED = [
    ("Condenser", "condenser_reflux.usd", (0.05, 0.35), (0, 90, 0), None,
     "直形冷凝管·松散（已拆 Scope 修可见，横躺）"),
    ("CondenserClamp", "test_tube_clamp.usd", (0.22, 0.52), (0, 0, 0), None,
     "冷凝管夹·松散（第2支撑，新增）"),
    ("ReceivingFlask", "conical_flask_77x77x97.usd", (0.35, -0.35), (0, 0, 0), "/root",
     "接液锥形瓶·松散"),
]

# 烧瓶内预装内容（作 /World/DistillationFlask 子 prim，随瓶移动；数值照 gen_d5_scene.py 内部换算）
SAMPLE_LIQ_R = 0.028          # 液柱半径（烧瓶内径 ~0.030）
SAMPLE_LIQ_H = 0.012          # 液柱高 12mm（~20mL）
SAMPLE_LIQ_CZ = SAMPLE_LIQ_H / 2 + 0.002   # 液心离瓶底 8mm（本地 z，随瓶 origin）
ZEO_Z = 0.006                 # 沸石底离瓶底 6mm
ZEO_XY = [(0.002, 0.002), (-0.005, 0.004), (0.004, -0.005), (0.0, -0.008)]  # 沸石×4
# 样品液配方（同 d2l 单通道主导防洗白：近黑 diffuse + 单通道主导 emissive）
SAMPLE_LIQ = dict(color=(0.05, 0.06, 0.12), opacity=0.95, roughness=0.05, ior=1.33,
                  emissive=(0.15, 0.35, 2.0))


def _reference(prim, asset, ref_path):
    asset_path = os.path.abspath(os.path.join(EQ, asset))
    if ref_path:
        prim.GetPrim().GetReferences().AddReference(asset_path, Sdf.Path(ref_path))
    else:
        prim.GetPrim().GetReferences().AddReference(asset_path)


def place_exact(stage, name, asset, xyz, rot=(0, 0, 0), ref_path=None, note=""):
    """精确摆放：引用资产 + translate=xyz 原样（不贴台自动算）。冻结集群用。"""
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    _reference(prim, asset, ref_path)
    tr = prim.AddTranslateOp()
    tr.Set(Gf.Vec3d(*xyz))
    if rot != (0, 0, 0):
        prim.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    print(f"[frozen] {name:16s} <- {asset} xyz({xyz[0]:+.4f},{xyz[1]:+.4f},{xyz[2]:+.4f}) "
          f"rot{rot}  {note}")
    return prim


def add_part(stage, name, asset, xy, rot=(0, 0, 0), ref_path=None, note=""):
    """贴台松散件：引用资产 + 自动算 translate.z 使底=0.80（几何带旋转后数值 minz 动态算；
    op 顺序 [translate, rotate] = T·R，旋转绕资产本地原点 → 先算 rotate-only minz，再补 z）。"""
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    _reference(prim, asset, ref_path)
    tr = prim.AddTranslateOp()
    tr.Set(Gf.Vec3d(0, 0, 0))
    if rot != (0, 0, 0):
        prim.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    r = bc.ComputeWorldBound(prim.GetPrim()).ComputeAlignedRange()
    tz = TABLE_TOP - r.GetMin()[2]
    tr.Set(Gf.Vec3d(xy[0], xy[1], tz))
    print(f"[loose] {name:16s} <- {asset} xy({xy[0]:.3f},{xy[1]:.3f}) rot{rot} "
          f"ref={ref_path or 'defaultPrim'} -> z {tz:.4f}  {note}")
    return prim


def add_match(stage):
    """火柴锚点：贴 runnable 的 z=0.813 抬高位（机械臂抓取坐标勿动）。"""
    prim = UsdGeom.Xform.Define(stage, f"/World/{MATCH_NAME}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, "match.usd")))
    prim.AddTranslateOp().Set(Gf.Vec3d(MATCH_X, MATCH_Y, MATCH_T))
    print(f"[loose] {MATCH_NAME} <- match.usd at ({MATCH_X},{MATCH_Y},{MATCH_T}) (anchor)")


MATCH_NAME = "Match"


def add_flask_contents(stage):
    """烧瓶预装内容：样品液柱 + 沸石×4，作 /World/DistillationFlask 子 prim（本地坐标，
    随烧瓶 Xform 移动；本地 z 0 = 烧瓶底/origin）。"""
    liq = UsdGeom.Cylinder.Define(stage, "/World/DistillationFlask/SampleLiquid")
    liq.CreateRadiusAttr(SAMPLE_LIQ_R)
    liq.CreateHeightAttr(SAMPLE_LIQ_H)
    liq.CreateAxisAttr("Z")
    liq.AddTranslateOp().Set(Gf.Vec3d(0, 0, SAMPLE_LIQ_CZ))
    UsdGeom.Gprim(liq).CreateDoubleSidedAttr().Set(True)
    mat = UsdShade.Material.Define(stage, "/World/DistillationFlask/SampleLiquid_mat")
    sh = UsdShade.Shader.Define(stage, "/World/DistillationFlask/SampleLiquid_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*SAMPLE_LIQ["color"]))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQ["opacity"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQ["roughness"])
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(SAMPLE_LIQ["ior"])
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*SAMPLE_LIQ["emissive"]))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(liq).Bind(mat)
    print("[content] DistillationFlask/SampleLiquid visible (r %.3f h %.3f at local z %.3f)"
          % (SAMPLE_LIQ_R, SAMPLE_LIQ_H, SAMPLE_LIQ_CZ))

    for i, (zx, zy) in enumerate(ZEO_XY):
        zp = UsdGeom.Xform.Define(stage, f"/World/DistillationFlask/Zeolite{i}")
        zp.GetPrim().GetReferences().AddReference(
            os.path.abspath(os.path.join(EQ, "zeolite.usd")))
        zp.AddTranslateOp().Set(Gf.Vec3d(zx, zy, ZEO_Z))
    print(f"[content] DistillationFlask/Zeolite0..{len(ZEO_XY) - 1} at flask bottom")


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
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def _all_part_names():
    return [row[0] for row in FROZEN_SIX] + [row[0] for row in LOOSE_GROUNDED] + [MATCH_NAME]


def remove_asset_env_lights(st2):
    """铁架台 / 石棉网 / 试管夹等资产自带 env_light DomeLight → 移除（避免嵌套暗光）。"""
    for name in _all_part_names():
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")


def flatten_condenser_scope(st2):
    """冷凝管几何孤儿化修复：把 /World/Condenser/condenser_reflux(Scope) 下的两个 Mesh
    (body/plastic_top) 用 Sdf.CopySpec 提到 /World/Condenser 直下，再删空 Scope。
    Scope 无变换，几何世界位置不变；材质绑定（/World/Condenser/Looks/*）路径保持有效。
    修复后结构与 conical（Xform→Mesh）一致，Isaac 重开/导入不再甩孤儿到 /root。"""
    scope = st2.GetPrimAtPath("/World/Condenser/condenser_reflux")
    if not scope.IsValid():
        print("[fix] /World/Condenser/condenser_reflux not found — already flat, skip")
        return
    layer = st2.GetRootLayer()
    base = "/World/Condenser"
    moved = []
    for child in list(scope.GetChildren()):
        if child.GetTypeName() != "Mesh":
            continue
        dst = Sdf.Path(f"{base}/{child.GetName()}")
        ok = Sdf.CopySpec(layer, child.GetPath(), layer, dst)
        print(f"[fix] condenser {child.GetName()} {child.GetPath()} -> {dst} (copy={ok})")
        moved.append(dst)
    st2.RemovePrim(scope.GetPath())
    print(f"[fix] removed empty Scope {scope.GetPath()}; meshes now direct children: {moved}")


def _set_local_translate(st2, path, vec):
    """就地改写已存在的 xformOp:translate（保持 op 顺序），类型匹配原 op。"""
    p = st2.GetPrimAtPath(path)
    if not p.IsValid():
        raise SystemExit(f"[err] {path} not found for translate edit")
    xf = UsdGeom.Xformable(p)
    for o in xf.GetOrderedXformOps():
        if o.GetOpName() == "xformOp:translate":
            o.Set(o.Get().__class__(*vec))
            print(f"[edit] {path} translate -> {tuple(vec)}")
            return
    raise SystemExit(f"[err] {path} has no xformOp:translate to edit")


def apply_frozen_subedits(st2):
    """照抄 tmp 里用户的局部动作：酒精灯 cap 单独搁、冻结试管夹爪头抬升。"""
    _set_local_translate(st2, "/World/AlcoholLamp/cap", CAP_ASIDE_TRANS)
    _set_local_translate(
        st2, "/World/ClampCondenser/polySurface7_Black_0_002", (0.0, 0.0, CLAMP_JAW_Z))


def _set_shader_glass(sh, opacity=0.25):
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.85, 0.92, 0.98))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)


def bind_or_override_glass(st2, mesh_prim, opacity=0.25):
    """给烧瓶玻璃 Mesh 透明化：已有可解析 material:binding → 重写其 shader；否则新建绑定。"""
    UsdGeom.Gprim(mesh_prim).CreateDoubleSidedAttr().Set(True)
    rel = mesh_prim.GetRelationship("material:binding")
    if rel:
        for tgt in rel.GetTargets():
            mat = st2.GetPrimAtPath(tgt)
            if not mat.IsValid():
                continue
            for c in mat.GetChildren():
                if c.GetTypeName() == "Shader":
                    _set_shader_glass(UsdShade.Shader(c), opacity)
                    print(f"[mat] override {mesh_prim.GetPath()} -> {c.GetPath()}")
                    return
    mat = UsdShade.Material.Define(st2, str(mesh_prim.GetPath()) + "_glass")
    sh = UsdShade.Shader.Define(st2, str(mesh_prim.GetPath()) + "_glass/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    _set_shader_glass(sh, opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh_prim).Bind(mat)
    print(f"[mat] bind new glass {mesh_prim.GetPath()}")


def fix_receiver_glass(st2):
    """接液锥形瓶：其 mesh 材质绑定指向 /root 引用外 /World/Looks（越界失效）→ 绑一块
    高不透明玻璃（op0.95，勿用真 alpha——Blender 摆位副本里薄玻璃会渲染成不可见）。
    烧瓶/冷凝管保持各自原生材质不动（散件场景是给人抓取组装的，必须实体可见）。"""
    p = st2.GetPrimAtPath("/World/ReceivingFlask")
    if not p.IsValid():
        print("[mat] /World/ReceivingFlask not found, skip")
        return
    meshes = [c for c in Usd.PrimRange(p) if c.GetTypeName() == "Mesh"]
    for m in meshes:
        bind_or_override_glass(st2, m, opacity=0.95)
    print(f"[mat] ReceivingFlask: {len(meshes)} mesh -> opaque glass op0.95 (Blender-visible)")


def _translate_of(st2, path):
    xf = UsdGeom.Xformable(st2.GetPrimAtPath(path))
    for o in xf.GetOrderedXformOps():
        if o.GetOpName() == "xformOp:translate":
            v = o.Get()
            return (float(v[0]), float(v[1]), float(v[2]))
    return None


def verify(st2):
    """自检（纯 pxr，非视觉）：
    1) 冻结六件 translate 逐字 = FROZEN_SIX（照抄回归）；cap/试管夹爪局部偏移已应用。
    2) 冷凝管已拆 Scope、body/plastic_top 是 /World/Condenser 直下 Mesh 且 bbox 非空。
    3) 松散件（冷凝管/冷凝管夹/接液瓶/火柴）贴台 0.80、在桌面界内，彼此及与冻结集群不重叠。
    4) 酒精灯火焰仍 invisible（未点燃）；火柴锚点坐标不变。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode(0.0), ["default"])
    boxes = {}

    # 1) 冻结六件照抄回归 + 记录 bbox（供集群包围盒）
    for name, _asset, xyz, rot, _note in FROZEN_SIX:
        path = f"/World/{name}"
        p = st2.GetPrimAtPath(path)
        if not p.IsValid():
            raise SystemExit(f"[verify] {path} MISSING")
        t = _translate_of(st2, path)
        assert t is not None and all(abs(a - b) < 1e-4 for a, b in zip(t, xyz)), \
            f"{name} translate {t} != frozen {xyz}"
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        boxes[name] = (tuple(r.GetMin()), tuple(r.GetMax()))
        print(f"[verify] {name:16s} frozen xyz{t} bbox_min({r.GetMin()[0]:+.4f},{r.GetMin()[1]:+.4f},"
              f"{r.GetMin()[2]:+.4f}) max_z {r.GetMax()[2]:+.4f}")

    # cap 单搁 & 试管夹爪抬升局部编辑
    cap_t = _translate_of(st2, "/World/AlcoholLamp/cap")
    assert cap_t and all(abs(a - b) < 1e-4 for a, b in zip(cap_t, CAP_ASIDE_TRANS)), \
        f"cap not set aside: {cap_t}"
    jaw_t = _translate_of(st2, "/World/ClampCondenser/polySurface7_Black_0_002")
    assert jaw_t and abs(jaw_t[2] - CLAMP_JAW_Z) < 1e-4, f"clamp jaw not raised: {jaw_t}"
    print("[verify] lamp cap set aside, flask clamp jaw raised to neck height")

    # 2) 冷凝管已拆 Scope
    assert not st2.GetPrimAtPath("/World/Condenser/condenser_reflux").IsValid(), \
        "condenser scope still present"
    for child in ("body", "plastic_top"):
        m = st2.GetPrimAtPath(f"/World/Condenser/{child}")
        assert m.IsValid() and m.GetTypeName() == "Mesh", \
            f"/World/Condenser/{child} not direct Mesh"
    r = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/Condenser")).ComputeAlignedRange()
    boxes["Condenser"] = (tuple(r.GetMin()), tuple(r.GetMax()))
    assert r.GetMin()[2] > TABLE_TOP - 1e-3 and r.GetMax()[0] > 0.1, \
        "condenser flat not on table / not along +X"
    print(f"[verify] condenser flattened + loose: min({r.GetMin()[0]:+.4f},{r.GetMin()[1]:+.4f},"
          f"{r.GetMin()[2]:+.4f}) max({r.GetMax()[0]:+.4f},{r.GetMax()[1]:+.4f},{r.GetMax()[2]:+.4f})")

    # 3) 松散件贴台 & 边界
    cluster_min = [min(boxes[n][0][i] for n, *_ in FROZEN_SIX) for i in range(3)]
    cluster_max = [max(boxes[n][1][i] for n, *_ in FROZEN_SIX) for i in range(3)]
    print(f"[verify] frozen cluster box min{tuple(round(v,3) for v in cluster_min)} "
          f"max{tuple(round(v,3) for v in cluster_max)}")

    loose_names = [row[0] for row in LOOSE_GROUNDED] + [MATCH_NAME]
    for name in loose_names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            raise SystemExit(f"[verify] /World/{name} MISSING")
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = tuple(r.GetMin()), tuple(r.GetMax())
        boxes[name] = (mn, mx)
        print(f"[verify] {name:16s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
        assert mn[0] >= -0.995 and mx[0] <= 0.995 and mn[1] >= -0.995 and mx[1] <= 0.995, \
            f"{name} out of bench"
        if name == MATCH_NAME:
            assert abs(mn[0] - MATCH_X) < 0.01 and abs(mn[1] - MATCH_Y) < 0.01 and mn[2] > TABLE_TOP + 0.008
        else:
            assert abs(mn[2] - TABLE_TOP) < 0.003, f"{name} bottom {mn[2]:.4f} != 0.80"

    # 松散件彼此不重叠 & 与冻结集群 xy 不重叠（集群内六件互相堆叠不算）
    def xy_overlap(a, b):
        ax0, ay0 = a[0][0], a[0][1]
        ax1, ay1 = a[1][0], a[1][1]
        bx0, by0 = b[0][0], b[0][1]
        bx1, by1 = b[1][0], b[1][1]
        return min(ax1, bx1) - max(ax0, bx0) > 1e-4 and min(ay1, by1) - max(ay0, by0) > 1e-4

    for i in range(len(loose_names)):
        a = boxes[loose_names[i]]
        for j in range(i + 1, len(loose_names)):
            b = boxes[loose_names[j]]
            assert not xy_overlap(a, b), f"loose OVERLAP {loose_names[i]} vs {loose_names[j]}"
        cl = ((cluster_min[0], cluster_min[1], 0), (cluster_max[0], cluster_max[1], 9))
        assert not xy_overlap(a, cl), f"loose {loose_names[i]} overlaps frozen cluster in xy"

    # 4) 酒精灯火焰 invisible
    for n in ("flame_outer", "flame_inner"):
        fp = st2.GetPrimAtPath(f"/World/AlcoholLamp/{n}")
        assert fp.IsValid() and UsdGeom.Imageable(fp).ComputeVisibility() == "invisible", \
            f"lamp {n} should stay invisible (unlit)"

    print("\n[verify] OK: 冻结六件逐字照抄 / cap+试管夹局部偏移 / 冷凝管已拆Scope可见 / "
          "松散件贴台在界内不重叠 / 灯焰未点燃 / 火柴锚点不变")
    print("\n[组装提示] 左侧六件冻结集群已就位（勿改相对位置），剩你要手动连的松散件：")
    print("  1) 冷凝管（横躺 0.05,0.35 附近）——一端对烧瓶侧支管口 ≈(-0.383,0.099,1.05)（支管朝 +X）")
    print("     另一端斜下伸向接液锥形瓶（现放 0.35,-0.35）瓶口，整根斜架 ~-58°（参考 runnable）。")
    print("  2) 冷凝管夹（0.22,0.52，新增）——夹冷凝管中段做第 2 支撑（铁架台另有试管夹夹瓶颈）。")
    print("  3) 接液锥形瓶（0.35,-0.35）——放冷凝管出口正下方收集馏出液。")
    print("  4) 酒精灯/火柴勿动相对位置。装好后告诉我，我据此烘焙最终现象场景+同步点火坐标。")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, xyz, rot, note in FROZEN_SIX:
        place_exact(stage, name, asset, xyz, rot, note=note)
    for name, asset, xy, rot, ref_path, note in LOOSE_GROUNDED:
        add_part(stage, name, asset, xy, rot, ref_path, note=note)
    add_match(stage)
    add_flask_contents(stage)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    flatten_condenser_scope(st2)
    apply_frozen_subedits(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    fix_receiver_glass(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
