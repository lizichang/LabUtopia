# -*- coding: utf-8 -*-
"""生成 E3「密度测定」四样资产：10mL 量筒 / 5mL 移液管+洗耳球 / 移液管架。单位：米。

按 labutopia-assets skill 实物调研通法，尺寸来源（标准实验室规格，非记忆）：
  量筒 10mL   —— 硼硅玻璃，GB/JIS 10mL 系列：外径约 Ø15mm、壁厚 1mm（内径 Ø13mm）、
                 直段高约 140mm、全高约 148mm（含六角底座）；刻度精度 0.1mL（inventory）
  移液管 5mL  —— 玻璃，5mL 刻度移液管：外径 Ø7mm、壁厚 0.7mm（内径 Ø5.6mm）、
                 全长约 300mm、尖端收口；刻度精度 0.05mL（inventory），需配洗耳球。
                 【2026-08-28 用户反馈：移液管太大远超量筒/样品瓶 → 压缩为约 185mm
                  （管身 140 + 洗耳球 45），与量筒 148mm 成比例；5mL 名义容量不改】
  洗耳球      —— 橡胶，约 Ø28mm×45mm（压缩为比例），顶部圆头 + 底部锥形嘴（套移液管顶）
  移液管架    —— 塑料，圆盘底座 Ø60×20mm + 中央空心插孔（Ø8mm 内径 × 30mm 高）

材质规范（玻璃 transmission 由后处理补，本脚本用 opacity 半透明逼近）：
  玻璃 diffuse 淡蓝白 opacity 0.35；橡胶红棕；塑料灰；白油墨刻度 opacity 1.0。

分组（group 名 = USD prim 路径，任务代码依赖）：
  量筒      body / base / marks
  移液管    tube / bulb / marks
  移液管架  stand

原点约定：底面 z=0（z 向上），轴在 X=Y=0；刻度与数字朝向 +Y（前）。

用法：python scripts/gen_e3_assets.py   （运行环境：labutopia conda env 有 pxr）
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
OUT_OBJ = os.path.join(tempfile.gettempdir(), "e3_assets_obj")

S = 48  # 回转体段数
FRONT = 0.5 * math.pi  # 刻度/数字朝向（+Y，前）


# ================================================================ 量筒 10mL
CYL_R_OUT = 0.0075        # 筒身外径半径（Ø15）
CYL_R_IN = 0.0065         # 筒身内径半径（Ø13，壁厚 1mm）
CYL_H_TUBE = 0.140        # 筒身直段高
CYL_LIP_R = 0.0082        # 口沿外翻半径
CYL_LIP_H = 0.1425        # 口沿顶 z（相对筒底）
CYL_BASE_R = 0.026        # 六角底座角半径（对角 52mm / 对边 45mm）
CYL_BASE_H = 0.005        # 底座厚（全高 = 5 + 142.5 ≈ 148mm）
CYL_SPOUT_HALF = 0.40     # 尖嘴 V 形凹口半张角 rad
CYL_NOTCH_DEPTH = 0.0023  # 凹口深度
CYL_Z_MARK_BOT = 0.015    # 0mL 刻度 z
CYL_Z_MARK_TOP = 0.132    # 10mL 刻度 z


# ================================================================ 移液管 5mL
PIPE_R_OUT = 0.0035       # 管身外径半径（Ø7）
PIPE_R_IN = 0.0028        # 管身内径半径（Ø5.6，壁厚 0.7mm）
PIPE_TIP_LEN = 0.020      # 尖端收口段长（尖端 z=0）
PIPE_TUBE_TOP = 0.140     # 管身顶 z（管身 140mm，压缩为与量筒成比例）
PIPE_Z_5ML = 0.050        # 5mL 刻度 z（近尖端，下）
PIPE_Z_0ML = 0.125        # 0mL 刻度 z（近管顶，上）
# 洗耳球（套在管顶上方，z 从 PIPE_TUBE_TOP 起，球总高约 45mm → 顶 ≈0.185）
BULB_TIP_R = 0.0005       # 球顶收尖半径


# ================================================================ 移液管架
STD_BASE_R = 0.030        # 圆盘底座半径（Ø60）
STD_BASE_H = 0.020        # 底座厚（20mm）
STD_SOCKET_R_OUT = 0.006  # 插孔外径半径（Ø12）
STD_SOCKET_R_IN = 0.004   # 插孔内径半径（Ø8）
STD_SOCKET_H = 0.030      # 插孔高（30mm，总高 50mm）


def _belt(mb, group, P0, P1, orient):
    """两环 P0→P1 间铺四边形带（等长、同方位对齐）。orient='radial' 法线朝外 / 'up' 朝上。"""
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


def _stroke(mb, group, th0, z0, th1, z1, t, R):
    """墙面上从 (th0,z0) 到 (th1,z1) 的细条带（宽 t，沿壁面切平面）。"""
    dth, dz = th1 - th0, z1 - z0
    L = math.hypot(dth * R, dz)
    if L < 1e-9:
        return
    tu, tv = dth * R / L, dz / L
    pu, pv = -tv, tu
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


def add_number(mb, group, text, zc, th_center, R, dw, dh, gap, stroke):
    """在墙面水平渲染数字（笔画字体），数字框中心 (zc, th_center)。"""
    n = len(text)
    W = dw * n + gap * (n - 1)
    x0 = th_center - W / R / 2.0
    for k, ch in enumerate(text):
        cx = x0 + (k * dw + k * gap + dw / 2.0) / R
        for (u0, v0), (u1, v1) in DIGITS[ch]:
            th0 = cx + (u0 - 0.5) * dw / R
            th1 = cx + (u1 - 0.5) * dw / R
            z0 = zc + (v0 - 0.5) * dh
            z1 = zc + (v1 - 0.5) * dh
            _stroke(mb, group, th0, z0, th1, z1, stroke, R)


def _marks_3tier(mb, group, z_of, n_div, R, mark_h1, mark_h5, mark_h10,
                 a1, a5, a10l, a10r, num_off, num_scale, labels=True):
    """三档刻度（1/5/10 单位）+ 数字标签。ml 取 0..n_div 整数，z_of(ml) 给 z。
    labels=True 时 10 单位线标数字（数字文本 = ml//10，如 1..10 或 1..5）。"""
    def mark(z, h, a0, a1):
        thm = 0.5 * (a0 + a1)
        n = (math.cos(thm), math.sin(thm), 0.0)
        a = (R * math.cos(a0), R * math.sin(a0), z)
        b = (R * math.cos(a1), R * math.sin(a1), z)
        c = (R * math.cos(a1), R * math.sin(a1), z + h)
        d = (R * math.cos(a0), R * math.sin(a0), z + h)
        mb._add_quad(mb._add_vert(a, n), mb._add_vert(b, n),
                     mb._add_vert(c, n), mb._add_vert(d, n), group)

    dw, dh, gap, stroke = num_scale
    for ml in range(0, n_div + 1):
        z = z_of(ml)
        if ml % 10 == 0:
            mark(z, mark_h10, FRONT - a10l, FRONT + a10r)
            if labels and ml >= 10:
                add_number(mb, group, str(ml // 10), z, FRONT + num_off,
                           R, dw, dh, gap, stroke)
        elif ml % 5 == 0:
            mark(z, mark_h5, FRONT - a5, FRONT + a5)
        else:
            mark(z, mark_h1, FRONT - a1, FRONT + a1)


# ================================================================ 量筒构建
def build_cylinder(mb):
    """筒身立在底座顶（z=CYL_BASE_H），总高 CYL_BASE_H + CYL_LIP_H。"""
    z0 = CYL_BASE_H
    # 外壁 + 内壁 + 口沿（尖嘴 V 形凹口）
    mb.lathe([(CYL_R_OUT, z0), (CYL_R_OUT, z0 + CYL_H_TUBE)], S, "body", close_bottom=True)
    mb.lathe([(CYL_R_IN, z0 + CYL_LIP_H), (CYL_R_IN, z0)], S, "body", reverse=True)

    # 口沿 + 尖嘴
    z_wall = z0 + CYL_H_TUBE
    z_lip = z0 + CYL_LIP_H
    angs = [2 * math.pi * i / S for i in range(S)]

    def edge(a):
        u = abs(a) / CYL_SPOUT_HALF
        if u >= 1.0:
            return CYL_LIP_R, z_lip
        return CYL_R_OUT, z_lip - CYL_NOTCH_DEPTH * (1.0 - u)

    r0 = [(CYL_R_OUT * math.cos(a), CYL_R_OUT * math.sin(a), z_wall) for a in angs]
    r1 = []
    for a in angs:
        er, ez = edge(a)
        r1.append((er * math.cos(a), er * math.sin(a), ez))
    r2 = [(CYL_R_IN * math.cos(a), CYL_R_IN * math.sin(a), z_lip) for a in angs]
    _belt(mb, "body", r0, r1, "radial")
    _belt(mb, "body", r1, r2, "up")

    # 底座：六角玻璃棱柱
    mb.lathe([(CYL_BASE_R, 0.0), (CYL_BASE_R, CYL_BASE_H)], 6, "base",
             close_bottom=True, close_top=True)

    # 刻度：10mL 分度 0.1mL，1/0.5/0.1 三档 + 数字 1..10
    R = CYL_R_OUT + 0.0002

    def z_of(ml):
        return CYL_Z_MARK_BOT + (CYL_Z_MARK_TOP - CYL_Z_MARK_BOT) * (ml / 100.0)

    _marks_3tier(mb, "marks", z_of, 100, R,
                 0.0012, 0.0018, 0.0025,          # 线高 1/5/10mL
                 0.35, 0.49, 0.70, 0.52,          # 弧半宽
                 0.82, (0.0015, 0.0025, 0.0004, 0.0004))


# ================================================================ 移液管构建
def build_pipette(mb):
    """移液管：玻璃管身 + 尖端收口 + 顶套洗耳球 + 刻度。z 向上，尖端 z=0。"""
    # 管身外壁（尖端收口段 → 直段），close_bottom 封尖端
    mb.lathe([(0.0005, 0.0), (PIPE_R_OUT, PIPE_TIP_LEN), (PIPE_R_OUT, PIPE_TUBE_TOP)],
             S, "tube", close_bottom=True)
    # 内壁（法线朝内，从管顶内沿下行到尖端内沿）
    mb.lathe([(PIPE_R_IN, PIPE_TUBE_TOP), (PIPE_R_IN, PIPE_TIP_LEN), (0.0003, 0.0)],
             S, "tube", reverse=True)
    # 管顶环带（外沿 → 内沿，封住开口环）
    mb.lathe([(PIPE_R_OUT, PIPE_TUBE_TOP), (PIPE_R_IN, PIPE_TUBE_TOP)], S, "tube")

    # 洗耳球（橡胶）：底部锥形嘴套管顶 → 球身 → 顶部圆头（Ø28×45 压缩版）
    mb.lathe([(PIPE_R_IN, PIPE_TUBE_TOP - 0.004), (PIPE_R_OUT, PIPE_TUBE_TOP),
              (0.006, PIPE_TUBE_TOP + 0.008), (0.012, PIPE_TUBE_TOP + 0.016),
              (0.014, PIPE_TUBE_TOP + 0.026), (0.011, PIPE_TUBE_TOP + 0.036),
              (0.006, PIPE_TUBE_TOP + 0.042), (0.0, PIPE_TUBE_TOP + 0.045)],
             S, "bulb")

    # 刻度：5mL 分度（0.1mL 三档 + 数字 1..5）；ml 0..50 → z 从 PIPE_Z_0ML 到 PIPE_Z_5ML
    R = PIPE_R_OUT + 0.0001

    def z_of(ml):
        return PIPE_Z_0ML + (PIPE_Z_5ML - PIPE_Z_0ML) * (ml / 50.0)

    _marks_3tier(mb, "marks", z_of, 50, R,
                 0.0008, 0.0012, 0.0016,          # 线高 0.1/0.5/1mL
                 0.30, 0.42, 0.60, 0.45,          # 弧半宽
                 0.75, (0.0012, 0.0020, 0.0003, 0.0003))


# ================================================================ 移液管架构建
def build_pipette_stand(mb):
    """移液管架：圆盘底座 + 中央空心插孔（移液管尖端插入）。"""
    # 底座圆盘（实心）
    mb.lathe([(STD_BASE_R, 0.0), (STD_BASE_R, STD_BASE_H)], S, "stand",
             close_bottom=True, close_top=True)
    # 插孔外壁 + 内壁（空心管，上口开放）
    z1 = STD_BASE_H + STD_SOCKET_H
    mb.lathe([(STD_SOCKET_R_OUT, STD_BASE_H), (STD_SOCKET_R_OUT, z1)], S, "stand")
    mb.lathe([(STD_SOCKET_R_IN, z1), (STD_SOCKET_R_IN, STD_BASE_H)], S, "stand", reverse=True)
    # 插孔顶环带（外沿 → 内沿）
    mb.lathe([(STD_SOCKET_R_OUT, z1), (STD_SOCKET_R_IN, z1)], S, "stand")


# ================================================================ 材质 / USD
USD_MATS = {
    "cylinder": {
        "body": dict(diffuse=(0.85, 0.92, 0.98), opacity=0.35, ior=1.45, roughness=0.05),
        "base": dict(diffuse=(0.85, 0.92, 0.98), opacity=0.50, ior=1.45, roughness=0.05),
        "marks": dict(diffuse=(0.92, 0.94, 0.96), opacity=1.0, roughness=0.5),
    },
    "pipette": {
        "tube": dict(diffuse=(0.85, 0.92, 0.98), opacity=0.35, ior=1.45, roughness=0.05),
        "bulb": dict(diffuse=(0.55, 0.20, 0.18), opacity=1.0, roughness=0.55),
        "marks": dict(diffuse=(0.92, 0.94, 0.96), opacity=1.0, roughness=0.5),
    },
    "stand": {
        "stand": dict(diffuse=(0.55, 0.57, 0.60), opacity=1.0, roughness=0.5),
    },
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
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd):
    """验证：bbox（mm）、z0=0、up=Z、mpu=1.0、法线单位长度。"""
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
    print(f"verify: bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2]:.4f} "
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)} "
          f"bad_normals={bad_n}")


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)

    jobs = [
        ("graduated_cylinder_10ml", build_cylinder, USD_MATS["cylinder"]),
        ("pipette", build_pipette, USD_MATS["pipette"]),
        ("pipette_stand", build_pipette_stand, USD_MATS["stand"]),
    ]
    for name, build, mat in jobs:
        mb = MeshBuilder()
        build(mb)
        obj_path = os.path.join(OUT_OBJ, f"{name}.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(mb.to_obj(list(mat.keys())))
        out_usd = os.path.join(OUT_USD, f"{name}.usd")
        make_usd(obj_path, out_usd, mat)
        verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
