# -*- coding: utf-8 -*-
"""生成量筒（100mL）资产（单位：米，纯 pxr 管线，无需 Blender）。

结构规格（100mL 硼硅玻璃量筒，按 GB 12804-2011 / JIS R 3505）：
  筒身    外径 Ø32（r=16mm），壁厚 1.5mm（内径 Ø29；GB 要求壁厚≥1mm），
          筒身直段高 241.5mm，口沿外翻小唇（r 16→17.5mm，高 2.5mm）
  嘴部    倒水尖嘴（pour spout）：口沿一圈在 +X 方位被 V 形凹口打断并去除
          （尖端=筒壁直段顶，左右对称尖口），无外凸引流嘴
  底座    六角形玻璃棱柱（6 段 lathe 即六棱柱），对角 ≈84mm / 对边 ≈73mm，厚 6mm
  全高    250mm ✓ GB 12804-2011：全高 250±10mm；JIS R 3505：外径 φ32、全高 250
  刻度    100mL 分度 1mL（inventory：量筒100mL 刻度精度1mL）：
          刻度带 0→100mL，1mL 短细线 / 5mL 中长线 / 10mL 长线+数字(10..100)
          白油墨独立 marks 组（不透明），凸出壁面 0.2mm
  材质    玻璃：diffuse 淡蓝白，opacity 0.35（半透明），ior 1.45，roughness 0.05
          底座玻璃更厚，opacity 0.5 略实；刻度白墨 opacity 1.0

分组（group 名 = USD prim 路径，任务代码依赖）：
  body / base / marks

原点约定：底座底面 z=0（z 向上），轴在 X=Y=0；刻度与数字朝向 +Y（前）。
"""
import math
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "cylinder_assets_obj")

S = 48  # 回转体段数（够圆）

# ---- 尺寸（米），按 GB 12804-2011 / JIS R 3505 ----
R_OUT = 0.016        # 筒身外径半径（Ø32）
R_IN = 0.0145        # 筒身内径半径（Ø29，壁厚 1.5mm）
H_TUBE = 0.2415      # 筒身直段高
LIP_R = 0.0175       # 口沿外翻半径
LIP_H = 0.2440       # 口沿顶 z（相对筒底）= 241.5 + 2.5
BASE_R = 0.042       # 六角底座角半径（对角 84mm / 对边 73mm）
BASE_H = 0.006       # 底座厚（全高 = 6 + 244 = 250mm）

# ---- 尖嘴 V 形凹口（pour spout）：口沿在 +X 方位被打断成 V 形凹口 ----
SPOUT_HALF = 0.40    # 凹口半张角 rad（≈23°，口沿在此范围内被去除）
NOTCH_DEPTH = 0.0023 # 凹口深度（口沿顶 250→247.7mm，尖端距筒壁顶 0.2mm，无外凸）

# ---- 刻度线 + 数字（100mL，分度 1mL，白油墨）----
FRONT = 0.5 * math.pi        # 刻度/数字朝向（+Y，前）
MARK_RAISE = 0.0002          # 刻度凸出壁面量（白墨层）
Z_MARK_BOT = 0.012           # 0mL 刻度 z
Z_MARK_TOP = 0.242           # 100mL 刻度 z
MARK_H_1 = 0.0015            # 1mL 线高
MARK_H_5 = 0.0022            # 5mL 线高
MARK_H_10 = 0.0030           # 10mL 线高
MARK_A_1 = 0.35              # 1mL 半宽弧 rad（≈±20°）
MARK_A_5 = 0.49              # 5mL 半宽弧 rad（≈±28°）
MARK_A_10L = 0.70            # 10mL 左弧 rad
MARK_A_10R = 0.52            # 10mL 右弧 rad（右侧留数字位）
NUM_TH_OFF = 0.82            # 数字中心相对 FRONT 方位偏移 rad
DIGIT_W = 0.002              # 数字宽（沿方位弧长，m）
DIGIT_H = 0.003              # 数字高（沿 z，m）
DIGIT_GAP = 0.0005           # 数字间距
DIGIT_STROKE = 0.0005        # 笔画宽

GROUPS = ["body", "base", "marks"]


def build_cylinder(mb):
    """筒身立在底座顶（z=BASE_H），总高 BASE_H+LIP_H。"""
    z0 = BASE_H  # 筒身底 = 底座顶

    # 筒身外壁直段（口沿/尖嘴在 build_mouth 里做）；close_bottom 生成底圆盘封底
    mb.lathe([(R_OUT, z0), (R_OUT, z0 + H_TUBE)], S, "body", close_bottom=True)
    # 内壁（法线朝内，同 bunsen 管壁写法）
    mb.lathe([(R_IN, z0 + LIP_H), (R_IN, z0)], S, "body", reverse=True)
    # 口沿 + 倒水尖嘴（玻璃边缘本身鼓出成犄角）
    build_mouth(mb, "body")

    # 底座：六角玻璃棱柱（6 段 lathe 即正六棱柱），上下面封闭
    mb.lathe([(BASE_R, 0.0), (BASE_R, BASE_H)], 6, "base",
             close_bottom=True, close_top=True)

    # 刻度线 + 数字（白油墨，独立 marks 组）
    add_marks(mb, "marks")


def _belt(mb, group, P0, P1, orient):
    """两环 P0→P1 间铺四边形带（等长、同方位对齐）。
    orient='radial' 法线朝外 / 'up' 法线朝上。顶点法线 = 相邻面法线平均。"""
    N = len(P0)
    fn = np.zeros((N, 3))
    for i in range(N):
        j = (i + 1) % N
        a, b, c = np.array(P0[i]), np.array(P0[j]), np.array(P1[j])
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        n = n / ln if ln > 0 else np.array([1.0, 0.0, 0.0])
        mid = (np.array(P0[i]) + np.array(P0[j])) * 0.5
        ref = np.array([mid[0], mid[1], 0.0]) if orient == "radial" \
            else np.array([0.0, 0.0, 1.0])
        ref /= (np.linalg.norm(ref) or 1.0)
        if np.dot(n, ref) < 0:
            n = -n
        fn[i] = n
    vn = np.zeros((N, 3))
    for i in range(N):
        vn[i] = fn[i] + fn[(i - 1) % N]
    nn = np.linalg.norm(vn, axis=1)
    nn[nn == 0] = 1.0
    vn = vn / nn[:, None]
    i0 = [mb._add_vert(P0[i], tuple(vn[i])) for i in range(N)]
    i1 = [mb._add_vert(P1[i], tuple(vn[i])) for i in range(N)]
    for i in range(N):
        j = (i + 1) % N
        mb._add_quad(i0[i], i0[j], i1[j], i1[i], group)


def build_mouth(mb, group):
    """口沿 + 尖嘴 V 形凹口：口沿一圈在 +X 方位被 V 形凹口打断并去除，
    无外凸引流嘴；凹口尖端=筒壁直段顶（该处口沿完全消失），左右对称尖口。
    环：ring0=筒壁直段顶 → ring1=口沿外缘（嘴部回落到筒壁顶成 V 尖）→ ring2=口沿内缘。
    面 A=ring0→ring1 外壁（嘴部为 V 形回落面）；面 B=ring1→ring2 顶面（嘴部为斜面）。"""
    z0 = BASE_H
    z_wall = z0 + H_TUBE
    z_lip = z0 + LIP_H
    angs = [2 * math.pi * i / S for i in range(S)]

    def edge(a):
        u = abs(a) / SPOUT_HALF
        if u >= 1.0:
            return LIP_R, z_lip
        # V 形线性凹口：|a| SPOUT_HALF→0，口沿顶从 z_lip 线性回落到 z_wall+0.2mm，
        # 半径回到筒壁（无外凸引流嘴）
        return R_OUT, z_lip - NOTCH_DEPTH * (1.0 - u)

    r0 = [(R_OUT * math.cos(a), R_OUT * math.sin(a), z_wall) for a in angs]
    r1 = []
    for a in angs:
        er, ez = edge(a)
        r1.append((er * math.cos(a), er * math.sin(a), ez))
    r2 = [(R_IN * math.cos(a), R_IN * math.sin(a), z_lip) for a in angs]
    _belt(mb, group, r0, r1, "radial")   # 口沿外壁（嘴部 V 形回落面）
    _belt(mb, group, r1, r2, "up")       # 口沿顶面（嘴部 V 形斜面）


# ---- 刻度线 + 数字（笔画字体）----
DIGITS = {
    "0": [((0.2, 0.1), (0.8, 0.1)), ((0.8, 0.1), (0.8, 0.9)), ((0.8, 0.9), (0.2, 0.9)), ((0.2, 0.9), (0.2, 0.1))],
    "1": [((0.5, 0.1), (0.5, 0.9))],
    "2": [((0.2, 0.9), (0.8, 0.9)), ((0.8, 0.9), (0.8, 0.55)), ((0.8, 0.55), (0.2, 0.55)), ((0.2, 0.55), (0.2, 0.1)), ((0.2, 0.1), (0.8, 0.1))],
    "3": [((0.2, 0.9), (0.8, 0.9)), ((0.8, 0.9), (0.8, 0.55)), ((0.2, 0.55), (0.8, 0.55)), ((0.8, 0.55), (0.8, 0.1)), ((0.2, 0.1), (0.8, 0.1))],
    "4": [((0.2, 0.9), (0.2, 0.55)), ((0.2, 0.55), (0.8, 0.55)), ((0.8, 0.9), (0.8, 0.1))],
    "5": [((0.2, 0.9), (0.8, 0.9)), ((0.2, 0.9), (0.2, 0.55)), ((0.2, 0.55), (0.8, 0.55)), ((0.8, 0.55), (0.8, 0.1)), ((0.2, 0.1), (0.8, 0.1))],
    "6": [((0.2, 0.9), (0.8, 0.9)), ((0.2, 0.9), (0.2, 0.1)), ((0.2, 0.1), (0.8, 0.1)), ((0.8, 0.55), (0.2, 0.55)), ((0.8, 0.55), (0.8, 0.1))],
    "7": [((0.2, 0.9), (0.8, 0.9)), ((0.8, 0.9), (0.45, 0.1))],
    "8": [((0.2, 0.9), (0.8, 0.9)), ((0.8, 0.9), (0.8, 0.55)), ((0.2, 0.55), (0.8, 0.55)), ((0.8, 0.55), (0.8, 0.1)), ((0.2, 0.9), (0.2, 0.1)), ((0.2, 0.1), (0.8, 0.1))],
    "9": [((0.2, 0.9), (0.8, 0.9)), ((0.2, 0.9), (0.2, 0.55)), ((0.2, 0.55), (0.8, 0.55)), ((0.8, 0.9), (0.8, 0.1))],
}


def add_marks(mb, group):
    """刻度线 + 数字标签（100mL 量筒，分度 1mL，白油墨独立组）。
    刻度带 z 从 Z_MARK_BOT(0mL) 到 Z_MARK_TOP(100mL)；1/5/10mL 逐级加长加宽；
    10mL 长线右侧标数字（10..100）。朝向前方 FRONT（+Y）。"""
    R = R_OUT + MARK_RAISE

    def z_of(ml):
        return Z_MARK_BOT + (Z_MARK_TOP - Z_MARK_BOT) * (ml / 100.0)

    def mark(z, h, a0, a1):
        thm = 0.5 * (a0 + a1)
        n = (math.cos(thm), math.sin(thm), 0.0)
        a = (R * math.cos(a0), R * math.sin(a0), z)
        b = (R * math.cos(a1), R * math.sin(a1), z)
        c = (R * math.cos(a1), R * math.sin(a1), z + h)
        d = (R * math.cos(a0), R * math.sin(a0), z + h)
        mb._add_quad(mb._add_vert(a, n), mb._add_vert(b, n),
                     mb._add_vert(c, n), mb._add_vert(d, n), group)

    for ml in range(0, 101):
        z = z_of(ml)
        if ml % 10 == 0:
            mark(z, MARK_H_10, FRONT - MARK_A_10L, FRONT + MARK_A_10R)
            if ml >= 10:
                add_number(mb, group, str(ml), z, FRONT + NUM_TH_OFF)
        elif ml % 5 == 0:
            mark(z, MARK_H_5, FRONT - MARK_A_5, FRONT + MARK_A_5)
        else:
            mark(z, MARK_H_1, FRONT - MARK_A_1, FRONT + MARK_A_1)


def _stroke(mb, group, th0, z0, th1, z1, t, R):
    """墙面上从 (th0,z0) 到 (th1,z1) 的细条带（宽 t，单位米，沿壁面切平面）。"""
    dth, dz = th1 - th0, z1 - z0
    L = math.hypot(dth * R, dz)
    if L < 1e-9:
        return
    tu, tv = dth * R / L, dz / L     # 切向（弧长, z）平面内
    pu, pv = -tv, tu                 # 垂直向
    hw = t / 2.0
    thm = 0.5 * (th0 + th1)
    n = (math.cos(thm), math.sin(thm), 0.0)

    def P(th, z, off):
        a = th + off * pu / R
        zz = z + off * pv
        return (R * math.cos(a), R * math.sin(a), zz)

    a = P(th0, z0, -hw)
    b = P(th0, z0, hw)
    c = P(th1, z1, hw)
    d = P(th1, z1, -hw)
    mb._add_quad(mb._add_vert(a, n), mb._add_vert(b, n),
                 mb._add_vert(c, n), mb._add_vert(d, n), group)


def add_number(mb, group, text, zc, th_center):
    """在墙面水平渲染数字（笔画字体），数字框中心 (zc, th_center)。"""
    R = R_OUT + MARK_RAISE
    n = len(text)
    W = DIGIT_W * n + DIGIT_GAP * (n - 1)
    x0 = th_center - W / R / 2.0
    for k, ch in enumerate(text):
        cx = x0 + (k * DIGIT_W + k * DIGIT_GAP + DIGIT_W / 2.0) / R
        for (u0, v0), (u1, v1) in DIGITS[ch]:
            th0 = cx + (u0 - 0.5) * DIGIT_W / R
            th1 = cx + (u1 - 0.5) * DIGIT_W / R
            z0 = zc + (v0 - 0.5) * DIGIT_H
            z1 = zc + (v1 - 0.5) * DIGIT_H
            _stroke(mb, group, th0, z0, th1, z1, DIGIT_STROKE, R)


# ---------------------------------------------------------------- 材质
USD_MATS = {
    "body": dict(diffuse=(0.85, 0.92, 0.98), opacity=0.35, ior=1.45, roughness=0.05),
    "base": dict(diffuse=(0.85, 0.92, 0.98), opacity=0.50, ior=1.45, roughness=0.05),
    "marks": dict(diffuse=(0.92, 0.94, 0.96), opacity=1.0, roughness=0.5),
}


def make_usd(obj_path, out_usd, mat_specs):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/{g}", verts, faces, vns)
        spec = mat_specs.get(g)
        if spec is not None:
            mat_path = f"/root/{g}_mat"
            mat = UsdShade.Material.Define(stage, mat_path)
            sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*spec["diffuse"]))
            if spec.get("opacity", 1.0) < 1.0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
            if spec.get("ior") is not None:
                sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(spec["ior"])
            if spec.get("metallic", 0.0) > 0:
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd):
    """验证：bbox、z0=0、up=Z、mpu=1.0、法线全为单位向量。"""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    lo, hi, bad_n = None, None, 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = p.GetAttribute("points").Get()
        nrm = p.GetAttribute("normals").Get()
        if nrm is not None:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            lens = np.linalg.norm(N, axis=1)
            bad_n += int(np.sum((lens > 1.0001) | (lens < 0.9999)))
        P = np.array([[v[0], v[1], v[2]] for v in pts])
        mn, mx = P.min(0), P.max(0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
    sz = (hi - lo) * 1000
    print(f"verify: bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2]:.3f} "
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)} "
          f"bad_normals={bad_n}")


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    build_cylinder(mb)
    obj_path = os.path.join(OUT_OBJ, "graduated_cylinder_100ml.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(GROUPS))
    out_usd = os.path.join(OUT_USD, "graduated_cylinder_100ml.usd")
    make_usd(obj_path, out_usd, USD_MATS)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
