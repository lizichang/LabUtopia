# -*- coding: utf-8 -*-
"""生成 Abbemat Advanced 全自动折光仪资产（单位：米）。

按用户建模描述（Anton Paar 官方造型）：
  机身    前倾后直流线楔形（前低后高，人体工程视线角），四角大圆角，
          哑光医疗白（rough 0.5）
  触摸屏  前置倾斜宽屏，贴合前倾面（绕底前缘后倾 15°），亮黑玻璃 + 弱背光
  样品槽  顶后部凹陷不锈钢圆环杯（金属度 1.0），蓝宝石棱镜凹于杯内
  掀盖    半透明防护盖，铰接杯后缘，掀开
  品牌线  屏幕下方前倾面上一道红色装饰条
  脚垫    4× 黑色橡胶

分组（group 名 = 未来 USD prim 路径，任务代码依赖）：
  body / screen / brand / well / prism / cover / foot_1..foot_4

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
OUT_OBJ = os.path.join(tempfile.gettempdir(), "abbemat_assets_obj")

S = 40  # 回转体段数
ARC = 8  # 机身圆角弧段数


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
    """(y,z) 逆时针闭环：前倾面（前底 -hy_fb → 前顶 -hy_ft）+ 后直立 + 四圆角。
    rb=底圆角半径, rt=顶圆角半径。"""
    pts = []
    # 底边（前->后）
    _dedup(pts, (-hy_fb + rb, 0.0))
    _dedup(pts, (hy_back - rb, 0.0))
    # 底后角 270°->360°
    _arc(pts, hy_back - rb, rb, rb, 270, 360, seg)
    # 背边（下->上）
    _dedup(pts, (hy_back, sz - rt))
    # 背后角 0°->90°
    _arc(pts, hy_back - rt, sz - rt, rt, 0, 90, seg)
    # 顶边（后->前）
    _dedup(pts, (-hy_ft + rt, sz))
    # 顶前角 90°->180°
    _arc(pts, -hy_ft + rt, sz - rt, rt, 90, 180, seg)
    # 前边（上->下）
    _dedup(pts, (-hy_fb, rb))
    # 底前角 180°->270°
    _arc(pts, -hy_fb + rb, rb, rb, 180, 270, seg)
    # 末点与首点（浮点近零差）重合则剔除，避免闭环零长度边
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
            continue  # 零长度边（不应出现），跳过
        # 2D 外法线：垂直于边方向、背离质心
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
        # quad(a,b,c,d) 法线 ∝ (0,-dz,dy)
        if (nrm[0], nrm[1]) == (dz, -dy):
            mb._add_quad(a, b, c, d, group)
        else:
            mb._add_quad(a, d, c, b, group)
    # 端盖（法线 ±X）
    cap_m = mb._add_vert((-hx, cy, cz), (-1, 0, 0))
    cap_p = mb._add_vert((hx, cy, cz), (1, 0, 0))
    ring_m = [mb._add_vert((-hx, y, z), (-1, 0, 0)) for (y, z) in pts]
    ring_p = [mb._add_vert((hx, y, z), (1, 0, 0)) for (y, z) in pts]
    for i in range(n):
        j = (i + 1) % n
        mb._add_tri(ring_m[i], ring_m[j], cap_m, group)   # -X 盖
        mb._add_tri(ring_p[j], ring_p[i], cap_p, group)   # +X 盖


def slanted_plate(mb, C, w, h, thick, u, n, group, proud=0.0005):
    """倾斜薄板：中心 C=(x,y,z)，宽 w(X)、高 h(沿 y-z 单位方向 u)、厚 thick(沿外法线 n)。
    proud: 安装面外凸量（防共面闪烁）。每面独立顶点+法线。"""
    hw, hu = w / 2, h / 2
    ny, nz = n
    uy, uz = u
    cx, cy, cz = C

    def corner(fb, sx, sy):
        """fb='f'前(外)/'b'后(内)，sx=±1 (X)，sy=±1 (沿 u)。"""
        x = cx + sx * hw
        y = cy + sy * hu * uy + (proud if fb == 'f' else proud - thick) * ny
        z = cz + sy * hu * uz + (proud if fb == 'f' else proud - thick) * nz
        return (x, y, z)

    def Q(verts, n3):
        idx = [mb._add_vert(v, n3) for v in verts]
        mb._add_quad(idx[0], idx[1], idx[2], idx[3], group)

    # 前(外) BL,TL,TR,BR；法线 n
    Q([corner('f', -1, -1), corner('f', -1, 1), corner('f', 1, 1), corner('f', 1, -1)],
      (0, ny, nz))
    # 后(内) BR,TR,TL,BL；法线 -n
    Q([corner('b', 1, -1), corner('b', 1, 1), corner('b', -1, 1), corner('b', -1, -1)],
      (0, -ny, -nz))
    # 顶边（沿 u）；法线 u
    Q([corner('f', -1, 1), corner('f', 1, 1), corner('b', 1, 1), corner('b', -1, 1)],
      (0, uy, uz))
    # 底边；法线 -u
    Q([corner('f', 1, -1), corner('f', -1, -1), corner('b', -1, -1), corner('b', 1, -1)],
      (0, -uy, -uz))
    # 左(X-)边；法线 -X
    Q([corner('f', -1, -1), corner('b', -1, -1), corner('b', -1, 1), corner('f', -1, 1)],
      (-1, 0, 0))
    # 右(X+)边；法线 +X
    Q([corner('f', 1, 1), corner('f', 1, -1), corner('b', 1, -1), corner('b', 1, 1)],
      (1, 0, 0))


def lathe_at(mb, cx, cy, profile, segments, group, **kw):
    """绕 (cx,cy) 回转 profile（lathe 只能绕原点，平移只作用于新增顶点）。"""
    n0 = len(mb.verts)
    mb.lathe(profile, segments, group, **kw)
    for i in range(n0, len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x + cx, y + cy, z)


def tilt_x(mb, n0, pivot, deg):
    """把自 n0 起新增的顶点+法线绕 X 轴旋转 deg°（pivot=(py,pz)，x 不变）。"""
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    py0, pz0 = pivot
    for i in range(n0, len(mb.verts)):
        x, y, z = mb.verts[i]
        dy, dz = y - py0, z - pz0
        mb.verts[i] = (x, py0 + dy * c - dz * s, pz0 + dy * s + dz * c)
        nx, ny, nz = mb.norms[i]
        mb.norms[i] = (nx, ny * c - nz * s, ny * s + nz * c)


def annulus(mb, cx, cy, z0, z1, r_in, r_out, seg, group):
    """圆环板（真孔）：顶/底/外壁/内壁，法线正确。"""
    v_top_o, v_top_i, v_bot_i, v_bot_o = [], [], [], []
    for j in range(seg):
        th = 2 * np.pi * j / seg
        c, s = np.cos(th), np.sin(th)
        v_top_o.append(mb._add_vert((cx + r_out * c, cy + r_out * s, z1), (0, 0, 1)))
        v_top_i.append(mb._add_vert((cx + r_in * c, cy + r_in * s, z1), (0, 0, 1)))
        v_bot_i.append(mb._add_vert((cx + r_in * c, cy + r_in * s, z0), (0, 0, -1)))
        v_bot_o.append(mb._add_vert((cx + r_out * c, cy + r_out * s, z0), (0, 0, -1)))
    for j in range(seg):
        j2 = (j + 1) % seg
        mb._add_quad(v_top_o[j], v_top_o[j2], v_top_i[j2], v_top_i[j], group)    # 顶面
        mb._add_quad(v_bot_i[j], v_bot_i[j2], v_bot_o[j2], v_bot_o[j], group)    # 底面
        mb._add_quad(v_bot_o[j], v_bot_o[j2], v_top_o[j2], v_top_o[j], group)    # 外壁
        mb._add_quad(v_bot_i[j2], v_bot_i[j], v_top_i[j], v_top_i[j2], group)    # 内壁


# ---------------------------------------------------------------- 部件
def build_abbemat(mb):
    # 机身：前倾楔形 225(W)×330(D)底×115(H)，前底 y=-0.165 → 前顶 y=-0.135，后直立 y=+0.165
    pts = wedge_loop(0.165, 0.135, 0.165, 0.115, 0.008, 0.015, ARC)
    prism_x(mb, pts, 0.1125, "body")

    # 触摸屏：贴合前倾面（从 (-0.165,0.008) 到 (-0.135,0.100)），绕底前缘后倾 15°
    u = unit((0.030, 0.092))
    n = unit((-0.092, 0.030))                                   # 外法线（-Y,+Z）
    C = (0.0, (-0.165 + -0.135) / 2, (0.008 + 0.100) / 2)       # (-0.150, 0.054)
    slanted_plate(mb, C, 0.195, 0.095, 0.003, u, n, "screen")

    # 红色品牌线：屏幕下方前倾面，z≈0.012
    t = (0.012 - 0.008) / (0.100 - 0.008)                       # 前倾面插值
    Cb = (0.0, -0.165 + t * 0.030, 0.012)
    slanted_plate(mb, Cb, 0.070, 0.004, 0.0015, u, n, "brand")

    # 样品槽（顶后部）：不锈钢圆环杯 z∈[0.1155,0.1215]，蓝宝石棱镜凹于杯内
    well_cy = 0.11
    annulus(mb, 0.0, well_cy, 0.1155, 0.1215, 0.011, 0.017, S, "well")
    lathe_at(mb, 0.0, well_cy, [(0.010, 0.1155), (0.010, 0.1175)], S, "prism",
             close_bottom=True, close_top=True)

    # 掀盖：半透明圆盘，铰接杯后缘，绕 X 轴 -50° 掀开
    hinge = (well_cy + 0.017, 0.1215)                           # (0.127, 0.1215)
    n0 = len(mb.verts)
    mb.lathe([(0.016, 0.0), (0.016, 0.002)], S, "cover", close_bottom=True, close_top=True)
    for i in range(n0, len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x, y + hinge[0], z + hinge[1])
    tilt_x(mb, n0, hinge, -50.0)

    # 脚垫 4× Ø8×5，四角（避开底圆角）
    for g, (fx, fy) in zip(
        ["foot_1", "foot_2", "foot_3", "foot_4"],
        [(-0.090, -0.140), (0.090, -0.140), (-0.090, 0.140), (0.090, 0.140)],
    ):
        lathe_at(mb, fx, fy, [(0.004, 0.0), (0.004, 0.005)], S, g,
                 close_bottom=True, close_top=True)


# ---------------------------------------------------------------- 材质
USD_MATS = {
    "body": dict(diffuse=(0.92, 0.93, 0.95), roughness=0.50),
    "screen": dict(diffuse=(0.03, 0.03, 0.04), roughness=0.15,
                   emissive=(0.010, 0.015, 0.030)),
    "brand": dict(diffuse=(0.85, 0.15, 0.12), roughness=0.40),
    "well": dict(diffuse=(0.72, 0.73, 0.76), metallic=1.0, roughness=0.10),
    "prism": dict(diffuse=(0.04, 0.10, 0.28), opacity=0.85, ior=1.77, roughness=0.05),
    "cover": dict(diffuse=(0.55, 0.56, 0.60), opacity=0.35, roughness=0.20),
    "foot_1": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_2": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_3": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
    "foot_4": dict(diffuse=(0.08, 0.08, 0.10), roughness=0.85),
}

GROUPS = ["body", "screen", "brand", "well", "prism", "cover",
          "foot_1", "foot_2", "foot_3", "foot_4"]


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
    build_abbemat(mb)
    obj_path = os.path.join(OUT_OBJ, "abbemat_advanced.obj")
    obj = mb.to_obj(GROUPS)
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(obj)
    out_usd = os.path.join(OUT_USD, "abbemat_advanced.usd")
    make_usd(obj_path, out_usd, USD_MATS)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
