# -*- coding: utf-8 -*-
"""生成 手持蓝色钴玻璃（焰色反应隔钴玻璃观察）资产，单位：米。

真实钴玻璃（蓝色钴玻璃）：实验室观钾焰色反应用，浅蓝色薄片，常用 50×107mm
（也 100×100mm），厚 2.5-3mm；靠钴蓝(CoAl₂O₄)吸收 Na 的 589nm 黄线、露出 K 紫焰。
本资产 = 方形钴玻璃片 + 金属包边(bezel) + 手持柄 + 柄根金属箍：
  玻璃  glass   100×100×3mm 圆角方片（r=8mm），深钴蓝半透明
  包边  frame   环绕玻璃四周的金属窄边（外 108×108，边宽 4mm，同厚 3mm）
  箍    collar  柄与包边交界处金属箍（Φ5×7mm）
  柄    handle  细圆柄（Φ4×80mm，末端圆头，火柴粗细），机械臂像抓火柴一样横夹
原点约定：玻璃平放桌面，底面 z=0（贴桌面），玻璃中心在原点，柄沿 +X；
柄/箍轴线=玻璃中面（把手圆截面圆心在玻璃平面上，lollipop 居中）。
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
OUT_OBJ = os.path.join(tempfile.gettempdir(), "cobalt_glass_obj")

SEG = 14   # 圆角弧段数

# ---- 几何参数（米）----
GLASS_HX, GLASS_HY = 0.050, 0.050   # 玻璃半宽/半高（100×100mm）
GLASS_T = 0.003                     # 玻璃厚度 3mm
GLASS_R = 0.008                     # 玻璃圆角半径 8mm
FRAME_EXT = 0.004                   # 包边外扩宽度 4mm
FRAME_R = GLASS_R + FRAME_EXT       # 包边外圆角 12mm
FRAME_HX = GLASS_HX + FRAME_EXT     # 0.054
FRAME_HY = GLASS_HY + FRAME_EXT
AXIS_Z = GLASS_T / 2                # 柄/箍轴线=玻璃中面（过把手圆心，lollipop 居中）
COLLAR_R = 0.0025                   # 箍半径 2.5mm（Φ5）
COLLAR_X0, COLLAR_X1 = 0.053, 0.060  # 箍轴向范围（骑在包边外缘）
HANDLE_R = 0.002                    # 柄半径 2mm（Φ4，火柴粗细，居中且不凸出桌面）
HANDLE_X0, HANDLE_X1 = 0.058, 0.138  # 柄轴向范围（80mm）


# ---------------------------------------------------------------- helpers
def rounded_rect_loop(hx, hy, r, seg):
    """(x,y) 逆时针闭环：圆角矩形（4 段直边 + 4 段 1/4 圆弧）。"""
    pts = []

    def arc(cx, cy, a0, a1):
        for i in range(seg + 1):
            a = a0 + (a1 - a0) * i / seg
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))

    arc(hx - r, -hy + r, -math.pi / 2, 0.0)         # 右下角
    pts.append((hx, hy - r))                        # 右缘上端
    arc(hx - r, hy - r, 0.0, math.pi / 2)           # 右上角
    pts.append((-hx + r, hy))                       # 上缘左端
    arc(-hx + r, hy - r, math.pi / 2, math.pi)      # 左上角
    pts.append((-hx, -hy + r))                      # 左缘下端
    arc(-hx + r, -hy + r, math.pi, 3 * math.pi / 2)  # 左下角
    pts.append((hx - r, -hy))                       # 下缘右端
    return pts


def _side_wall(mb, loop, cx, cy, z0, z1, group, outward=True):
    """把 (x,y) 闭环挤出 z 向侧壁（法线按 outward 决定朝外/朝内）。"""
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        x0, y0 = loop[i]
        x1, y1 = loop[j]
        dx, dy = x1 - x0, y1 - y0
        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            continue
        nrm = None
        for (nx, ny) in ((dy, -dx), (-dy, dx)):
            mxp = ((x0 + x1) / 2 - cx, (y0 + y1) / 2 - cy)
            if nx * mxp[0] + ny * mxp[1] > 0:
                nrm = (nx, ny)
                break
        if nrm is None:
            nrm = (dy, -dx)
        if not outward:
            nrm = (-nrm[0], -nrm[1])
        ln = math.hypot(nrm[0], nrm[1])
        nx, ny = nrm[0] / ln, nrm[1] / ln
        a = mb._add_vert((x0, y0, z0), (nx, ny, 0))
        b = mb._add_vert((x1, y1, z0), (nx, ny, 0))
        c = mb._add_vert((x1, y1, z1), (nx, ny, 0))
        d = mb._add_vert((x0, y0, z1), (nx, ny, 0))
        mb._add_quad(a, b, c, d, group)


def prism_z(mb, loop, z0, z1, group):
    """沿 Z 挤出 (x,y) 闭环为实心棱柱（上/下盖 + 侧壁）。"""
    n = len(loop)
    cx = sum(p[0] for p in loop) / n
    cy = sum(p[1] for p in loop) / n
    cb = mb._add_vert((cx, cy, z0), (0, 0, -1))
    ct = mb._add_vert((cx, cy, z1), (0, 0, 1))
    rb = [mb._add_vert((x, y, z0), (0, 0, -1)) for (x, y) in loop]
    rt = [mb._add_vert((x, y, z1), (0, 0, 1)) for (x, y) in loop]
    for i in range(n):
        j = (i + 1) % n
        mb._add_tri(rb[i], rb[j], cb, group)
        mb._add_tri(rt[j], rt[i], ct, group)
    _side_wall(mb, loop, cx, cy, z0, z1, group, outward=True)


def annulus_rim(mb, outer, inner, z0, z1, group):
    """圆角矩形环（上/下环面 + 外壁，无内壁）。内缘与玻璃外缘重合，贴合包边。"""
    n = len(outer)
    ot = [mb._add_vert((x, y, z1), (0, 0, 1)) for (x, y) in outer]
    it = [mb._add_vert((x, y, z1), (0, 0, 1)) for (x, y) in inner]
    ob = [mb._add_vert((x, y, z0), (0, 0, -1)) for (x, y) in outer]
    ib = [mb._add_vert((x, y, z0), (0, 0, -1)) for (x, y) in inner]
    for i in range(n):
        j = (i + 1) % n
        mb._add_quad(ot[i], ot[j], it[j], it[i], group)   # 上环面
        mb._add_quad(ob[j], ob[i], ib[i], ib[j], group)   # 下环面
    ocx = sum(p[0] for p in outer) / n
    ocy = sum(p[1] for p in outer) / n
    _side_wall(mb, outer, ocx, ocy, z0, z1, group, outward=True)


# ---------------------------------------------------------------- 部件
def build_cobalt(mb):
    glass_loop = rounded_rect_loop(GLASS_HX, GLASS_HY, GLASS_R, SEG)
    prism_z(mb, glass_loop, 0.0, GLASS_T, "glass")

    frame_outer = rounded_rect_loop(FRAME_HX, FRAME_HY, FRAME_R, SEG)
    frame_inner = rounded_rect_loop(GLASS_HX, GLASS_HY, GLASS_R, SEG)
    annulus_rim(mb, frame_outer, frame_inner, 0.0, GLASS_T, "frame")

    # 箍 + 柄（沿 +X，轴线 AXIS_Z，底贴桌面）
    mb.h_cylinder((COLLAR_X0, 0, AXIS_Z), (COLLAR_X1, 0, AXIS_Z),
                  COLLAR_R, SEG, "collar", cap=True)
    mb.h_cylinder((HANDLE_X0, 0, AXIS_Z), (HANDLE_X1, 0, AXIS_Z),
                  HANDLE_R, SEG, "handle", cap=True)
    mb.sphere(HANDLE_X1, 0, AXIS_Z, HANDLE_R, 8, SEG, "handle")


# ---------------------------------------------------------------- 材质
USD_MATS = {
    "glass": dict(diffuse=(0.03, 0.08, 0.35), opacity=0.78,
                  roughness=0.12, ior=1.5),
    "frame": dict(diffuse=(0.18, 0.19, 0.22), metallic=0.9, roughness=0.30),
    "collar": dict(diffuse=(0.18, 0.19, 0.22), metallic=0.9, roughness=0.30),
    "handle": dict(diffuse=(0.06, 0.06, 0.09), metallic=0.1, roughness=0.55),
}

GROUPS = ["glass", "frame", "collar", "handle"]


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
        if spec is None:
            continue
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
    import numpy as np
    stage = Usd.Stage.Open(out_usd)
    lo, hi, bad_n = None, None, 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get(0.0)
        if not pts:
            continue
        P = np.array([[v[0], v[1], v[2]] for v in pts])
        mn, mx = P.min(0), P.max(0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
        nrm = m.GetNormalsAttr().Get(0.0)
        if nrm is not None:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            lens = np.linalg.norm(N, axis=1)
            bad_n += int(np.sum((lens > 1.0001) | (lens < 0.9999)))
    sz = (hi - lo) * 1000
    print(f"verify: bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2]:.4f} "
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)} "
          f"bad_normals={bad_n}")


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    build_cobalt(mb)
    obj_path = os.path.join(OUT_OBJ, "cobalt_glass.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(GROUPS))
    out_usd = os.path.join(OUT_USD, "cobalt_glass.usd")
    make_usd(obj_path, out_usd, USD_MATS)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
