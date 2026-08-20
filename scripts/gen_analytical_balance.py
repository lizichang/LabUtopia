# -*- coding: utf-8 -*-
"""生成 薄型电子台秤（分析天平/简易电子秤）资产，单位：米。

按用户建模决定：薄、接近正方形的底座 + 前面板（按钮使深度略长）+ 顶部大称盘。
无防风罩/防风门。
  机身   薄矩形底座 200(W)×210(D)×40(H)，前斜面轻微内收，哑光白
  显示屏 前面板顶部亮屏（背光 LCD 观感，供读取）
  按键   3 键顶面前排，水平朝上（平行 xOy，机械臂垂直下按即可）
         （中间 TARE 键蓝色，两侧浅灰）
  立柱+称盘 不锈钢短柱 + Φ136 圆盘，置于顶面中央
  脚垫   4× 黑色橡胶水平脚

分组（group 名 = 未来 USD prim 路径，任务代码依赖）：
  body / screen / key_1..key_3 / stem / pan / foot_1..foot_4

原点约定：箱底 z=0（z 向上），X 居中，前 = -Y。
"""
import os
import sys
import tempfile
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "analytical_balance_obj")

S = 40   # 回转体段数
ARC = 8  # 机身圆角弧段数

# ---- 机身几何参数（米） ----
BODY_W   = 0.200   # X 宽（接近正方形，略窄）
BODY_D   = 0.210   # 底部深度（Y，前面板按钮使略长）
BODY_H   = 0.040   # 高（Z，薄）
SLOPE    = 0.006   # 前斜面顶部内收量（Y）
RB, RT   = 0.006, 0.005  # 底/顶圆角
HX       = BODY_W / 2
HY_FB    = BODY_D / 2
HY_FT    = HY_FB - SLOPE
HY_BACK  = HY_FB


# ---------------------------------------------------------------- helpers
def unit(v):
    n = np.linalg.norm(v)
    return (v[0] / n, v[1] / n)


def _dedup(pts, p, eps=1e-12):
    if pts and abs(pts[-1][0] - p[0]) < eps and abs(pts[-1][1] - p[1]) < eps:
        return
    pts.append(p)


def _arc(pts, cx, cy, r, a0, a1, seg):
    for i in range(seg + 1):
        a = math.radians(a0 + (a1 - a0) * i / seg)
        _dedup(pts, (cx + r * math.cos(a), cy + r * math.sin(a)))


def wedge_loop(hy_fb, hy_ft, hy_back, sz, rb, rt, seg):
    """(y,z) 逆时针闭环：前倾面（前底 -hy_fb → 前顶 -hy_ft）+ 后直立 + 四圆角。"""
    pts = []
    _dedup(pts, (-hy_fb + rb, 0.0))
    _dedup(pts, (hy_back - rb, 0.0))
    _arc(pts, hy_back - rb, rb, rb, 270, 360, seg)
    _dedup(pts, (hy_back, sz - rt))
    _arc(pts, hy_back - rt, sz - rt, rt, 0, 90, seg)
    _dedup(pts, (-hy_ft + rt, sz))
    _arc(pts, -hy_ft + rt, sz - rt, rt, 90, 180, seg)
    _dedup(pts, (-hy_fb, rb))
    _arc(pts, -hy_fb + rb, rb, rb, 180, 270, seg)
    if len(pts) > 1 and abs(pts[-1][0] - pts[0][0]) < 1e-9 and abs(pts[-1][1] - pts[0][1]) < 1e-9:
        pts.pop()
    return pts


def prism_x(mb, pts, hx, group):
    """沿 X 挤出 (y,z) 闭环为棱柱：侧面 ring + 两端盖，法线外指。"""
    n = len(pts)
    cy = sum(p[0] for p in pts) / n
    cz = sum(p[1] for p in pts) / n
    for i in range(n):
        y0, z0 = pts[i]
        y1, z1 = pts[(i + 1) % n]
        dy, dz = y1 - y0, z1 - z0
        if abs(dy) < 1e-12 and abs(dz) < 1e-12:
            continue
        nrm = None
        for (ny, nz) in ((dz, -dy), (-dz, dy)):
            mx_pt = ((y0 + y1) / 2 - cy, (z0 + z1) / 2 - cz)
            if ny * mx_pt[0] + nz * mx_pt[1] > 0:
                nrm = (ny, nz)
                break
        ln = math.hypot(nrm[0], nrm[1])
        nrm = (nrm[0] / ln, nrm[1] / ln)
        a = mb._add_vert((-hx, y0, z0), (0, nrm[0], nrm[1]))
        b = mb._add_vert((hx, y0, z0), (0, nrm[0], nrm[1]))
        c = mb._add_vert((hx, y1, z1), (0, nrm[0], nrm[1]))
        d = mb._add_vert((-hx, y1, z1), (0, nrm[0], nrm[1]))
        if (nrm[0], nrm[1]) == (dz, -dy):
            mb._add_quad(a, b, c, d, group)
        else:
            mb._add_quad(a, d, c, b, group)
    cap_m = mb._add_vert((-hx, cy, cz), (-1, 0, 0))
    cap_p = mb._add_vert((hx, cy, cz), (1, 0, 0))
    ring_m = [mb._add_vert((-hx, y, z), (-1, 0, 0)) for (y, z) in pts]
    ring_p = [mb._add_vert((hx, y, z), (1, 0, 0)) for (y, z) in pts]
    for i in range(n):
        j = (i + 1) % n
        mb._add_tri(ring_m[i], ring_m[j], cap_m, group)
        mb._add_tri(ring_p[j], ring_p[i], cap_p, group)


def slanted_plate(mb, C, w, h, thick, u, n, group, proud=0.0005):
    """倾斜薄板：中心 C，宽 w(X)、高 h(沿 u)、厚 thick(沿外法线 n)。"""
    hw, hu = w / 2, h / 2
    ny, nz = n
    uy, uz = u
    cx, cy, cz = C

    def corner(fb, sx, sy):
        x = cx + sx * hw
        y = cy + sy * hu * uy + (proud if fb == 'f' else proud - thick) * ny
        z = cz + sy * hu * uz + (proud if fb == 'f' else proud - thick) * nz
        return (x, y, z)

    def Q(verts, n3):
        idx = [mb._add_vert(v, n3) for v in verts]
        mb._add_quad(idx[0], idx[1], idx[2], idx[3], group)

    Q([corner('f', -1, -1), corner('f', -1, 1), corner('f', 1, 1), corner('f', 1, -1)],
      (0, ny, nz))
    Q([corner('b', 1, -1), corner('b', 1, 1), corner('b', -1, 1), corner('b', -1, -1)],
      (0, -ny, -nz))
    Q([corner('f', -1, 1), corner('f', 1, 1), corner('b', 1, 1), corner('b', -1, 1)],
      (0, uy, uz))
    Q([corner('f', 1, -1), corner('f', -1, -1), corner('b', -1, -1), corner('b', 1, -1)],
      (0, -uy, -uz))
    Q([corner('f', -1, -1), corner('b', -1, -1), corner('b', -1, 1), corner('f', -1, 1)],
      (-1, 0, 0))
    Q([corner('f', 1, 1), corner('f', 1, -1), corner('b', 1, -1), corner('b', 1, 1)],
      (1, 0, 0))


def lathe_at(mb, cx, cy, profile, segments, group, **kw):
    n0 = len(mb.verts)
    mb.lathe(profile, segments, group, **kw)
    for i in range(n0, len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x + cx, y + cy, z)


# ---------------------------------------------------------------- 部件
def build_balance(mb):
    # 机身：薄矩形底座，前斜面轻微内收（前底 y=-0.105 → 前顶 y=-0.099）
    pts = wedge_loop(HY_FB, HY_FT, HY_BACK, BODY_H, RB, RT, ARC)
    prism_x(mb, pts, HX, "body")

    # 前面板参数（贴板用）：从 (-HY_FB, RB) 到 (-HY_FT, BODY_H-RT)
    fy0, fz0 = -HY_FB, RB
    fy1, fz1 = -HY_FT, BODY_H - RT
    u = unit((fy1 - fy0, fz1 - fz0))
    n = unit((-(fz1 - fz0), fy1 - fy0))

    def face(t):
        return (fy0 + (fy1 - fy0) * t, fz0 + (fz1 - fz0) * t)

    # 显示屏：前面板上部，背光
    ys, zs = face(0.65)
    slanted_plate(mb, (0.0, ys, zs), 0.100, 0.016, 0.003, u, n, "screen")

    # 3 键：顶面前排，水平朝上（平行 xOy，机械臂垂直下按即可）
    for i, kx in enumerate([-0.035, 0.0, 0.035]):
        lathe_at(mb, kx, -0.082, [(0.008, BODY_H), (0.008, BODY_H + 0.004)],
                 S, f"key_{i + 1}", close_bottom=True, close_top=True)

    # 立柱 + 称盘：顶面中央（顶面 y 中心）
    pan_cy = ((-HY_FT + RT) + (HY_BACK - RT)) / 2
    lathe_at(mb, 0.0, pan_cy, [(0.010, BODY_H), (0.007, BODY_H + 0.003)],
             S, "stem", close_bottom=True, close_top=True)
    lathe_at(mb, 0.0, pan_cy, [(0.068, BODY_H + 0.003), (0.068, BODY_H + 0.007),
                               (0.065, BODY_H + 0.0075)],
             S, "pan", close_bottom=True, close_top=True)

    # 脚垫 4× Φ20×5，四角（避开底圆角）
    for g, (fx, fy) in zip(
        ["foot_1", "foot_2", "foot_3", "foot_4"],
        [(-0.080, -0.090), (0.080, -0.090), (-0.080, 0.090), (0.080, 0.090)],
    ):
        lathe_at(mb, fx, fy, [(0.010, 0.0), (0.010, 0.005)], S, g,
                 close_bottom=True, close_top=True)


# ---------------------------------------------------------------- 材质
USD_MATS = {
    "body": dict(diffuse=(0.92, 0.93, 0.95), roughness=0.50),
    "screen": dict(diffuse=(0.05, 0.05, 0.06), roughness=0.10,
                   emissive=(0.16, 0.17, 0.18)),
    "key_1": dict(diffuse=(0.75, 0.76, 0.78), roughness=0.45),
    "key_2": dict(diffuse=(0.20, 0.45, 0.85), roughness=0.40),   # TARE 键（蓝）
    "key_3": dict(diffuse=(0.75, 0.76, 0.78), roughness=0.45),
    "stem": dict(diffuse=(0.72, 0.73, 0.76), metallic=1.0, roughness=0.15),
    "pan": dict(diffuse=(0.78, 0.79, 0.82), metallic=1.0, roughness=0.12),
    "foot_1": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_2": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_3": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_4": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
}

GROUPS = ["body", "screen", "key_1", "key_2", "key_3",
          "stem", "pan", "foot_1", "foot_2", "foot_3", "foot_4"]


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
            if spec.get("emissive") is not None:
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*spec["emissive"]))
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
    build_balance(mb)
    obj_path = os.path.join(OUT_OBJ, "analytical_balance.obj")
    obj = mb.to_obj(GROUPS)
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(obj)
    out_usd = os.path.join(OUT_USD, "analytical_balance.usd")
    make_usd(obj_path, out_usd, USD_MATS)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
