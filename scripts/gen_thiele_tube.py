# -*- coding: utf-8 -*-
"""生成 B5 熔点测定器材：提勒管（Thiele tube / b形管）+ 毛细管（熔点管）。单位：米。

依据（2026-08-30 调研）：
  - 提勒管 = 主管（直管 Ø25×150，圆底，口部珠边）+ 侧管（Ø8 弯成 b 形环，下口/上口
    两处与主管熔接相通，无磨口，整件玻璃吹制一体）。
  - 毛细管（熔点管）= 外径 Ø1.5、内径 Ø1.0、长 100mm，一端圆钝封口、一端开口。
  - 主管尺寸锁定经典 25×150mm（AGN / MEDILAB / Eisco / Pyrex 四方一致）；侧管 b 形环
    弯曲比例按教科书装置图推导（上下叉管口、侧管下弯处=加热点、水银球居两叉口中间）。
  - 玻璃材质沿用旋光管：diffuse 浅蓝白 + opacity + ior 1.5 + roughness 0.05。

分组（group 名 = 未来 USD prim 路径，任务代码依赖）：
  提勒管：body（主管）/ side_arm（侧管 b 形环）
  毛细管：tube

原点约定：主管轴沿 Z（竖直），圆底在 z=0、口在 z=+0.150，侧管环鼓向 +X（y=0 平面）。
毛细管轴沿 Z，封口端在 z=0、开口端在 z=+0.100。
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "thiele_tube_obj")

S = 96  # 回转体段数（Ø25 主管 0.82mm/面，开孔孔边圆滑）

# ---- 提勒管尺寸（米）----
R_OUT = 0.0125   # 主管外径 Ø25
R_IN = 0.0105    # 主管内径 Ø21（壁厚 2mm）
H = 0.150        # 主管总高 150mm
R_RIM = 0.0130   # 口部珠边外径 Ø26
R_SIDE = 0.004   # 侧管外径 Ø8

# 侧管与主管连通：主管壁在上下接口处开孔，侧管两端穿入主管腔
HOLE_CZ = [0.018, 0.085]   # 上下接口孔中心 z（下口 18mm，上口 85mm；上下叉口关于 V 顶点对称）
HOLE_R = 0.0045            # 主管壁开孔半径（Ø9，略大于侧管 Ø8，孔管同心对齐）
SIDE_INNER_X = R_IN        # 侧管叉口中心线 x = 主管内壁（侧管口与内壁齐平，不深插）

# ---- 毛细管尺寸 ----
CAP_R = 0.00075  # 毛细管外径 Ø1.5（内径 Ø1.0，壁厚 0.25）
CAP_H = 0.100    # 毛细管长 100mm


def catmull_rom(ctrl, n_per_seg=20):
    """Catmull-Rom 样条过控制点，返回稠密 3D 采样点列（含首尾）。"""
    ctrl = [np.array(p, dtype=float) for p in ctrl]
    m = len(ctrl)
    out = []
    for i in range(m - 1):
        p0 = ctrl[max(i - 1, 0)]
        p1 = ctrl[i]
        p2 = ctrl[i + 1]
        p3 = ctrl[min(i + 2, m - 1)]
        for j in range(n_per_seg):
            t = j / n_per_seg
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                              + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(ctrl[-1])
    return out


def _rings(cz, half, step):
    """返回 [cz-half, cz-half+step, ..., cz+half] 的 z 列表（含端点，用于接口细环）。"""
    n = int(round(2 * half / step))
    return [cz - half + i * step for i in range(n + 1)]


def punch_hole(mb, cz, radius, group):
    """删除 group 中完全落在"水平孔柱"内的面（主管壁开孔，侧管穿入处）。

    孔轴线沿 +X，通过 (0, 0, cz)；顶点到轴线距离 sqrt(y^2+(z-cz)^2) < radius 且 x > 0
    （只在 +X 侧开孔，避免误删背面 th=180° 处）判在孔内。
    一个面的所有顶点都在孔内才删，避免误删孔边缘的整圈壁面。"""
    to_del = set()
    for f in mb._group_faces.get(group, []):
        if all(np.hypot(mb.verts[v][1], mb.verts[v][2] - cz) < radius
               and mb.verts[v][0] > 0
               for v in mb.faces[f]):
            to_del.add(f)
    if not to_del:
        return 0
    remap = {}
    new_faces = []
    for f, face in enumerate(mb.faces):
        if f in to_del:
            continue
        remap[f] = len(new_faces)
        new_faces.append(face)
    mb.faces = new_faces
    for g, flist in mb._group_faces.items():
        mb._group_faces[g] = [remap[f] for f in flist if f not in to_del]
    return len(to_del)


def build_thiele(mb):
    """主管（空心圆底 + 珠边口，+X 侧上下开两圆孔）+ 侧管 b 形环（两端水平穿入，孔管同心对齐）。"""
    # 接口细环：孔中心 z 附近 1mm 间距的密环，让开孔（删面）的孔边逼近圆形
    lo_rings = _rings(HOLE_CZ[0], 0.005, 0.001)
    hi_rings = _rings(HOLE_CZ[1], 0.005, 0.001)

    # 主管外壁：圆底半球（R=0.0125，球心 z=0.0125，22.5° 分弧）→ 直壁（含接口细环）→ 珠边口
    outer = [
        (0.0, 0.0000),
        (0.004784, 0.000951),
        (0.008839, 0.003661),
        (0.011549, 0.007716),
        (R_OUT, 0.0125),   # 赤道 = 壁起点
    ]
    outer += [(R_OUT, z) for z in lo_rings]
    outer += [(R_OUT, z) for z in hi_rings]
    outer += [(R_OUT, 0.146), (0.0128, 0.149), (R_RIM, H)]
    mb.lathe(outer, S, "body")

    # 主管内壁（reverse 法线朝内）：内半球（R=0.0105，同球心）→ 内直壁（同接口细环）
    inner = [
        (0.0, 0.0020),
        (0.004018, 0.002799),
        (0.007425, 0.005075),
        (0.009701, 0.008482),
        (R_IN, 0.0125),   # 内赤道（与外赤道同高 z=0.0125）
    ]
    inner += [(R_IN, z) for z in lo_rings]
    inner += [(R_IN, z) for z in hi_rings]
    inner += [(R_IN, H)]
    mb.lathe(inner, S, "body", reverse=True)

    # 底部壁厚环带：封住赤道 z=0.0125 处 外壁(0.0125)↔内壁(0.0105) 的开口（圆底与壁的连接面）
    mb.lathe([(R_OUT, R_OUT), (R_IN, R_OUT)], S, "body")

    # 口部珠边环带：外沿(0.0130) → 内沿(0.0105) 平环（管的壁厚顶面）
    mb.lathe([(R_RIM, H), (R_IN, H)], S, "body")

    # 主管壁开孔：上下接口处 +X 侧挖圆孔（水平圆柱），让侧管水平穿入（孔管同心对齐）
    for cz in HOLE_CZ:
        n = punch_hole(mb, cz, HOLE_R, "body")
        assert n > 0, f"接口 z={cz} 未开孔，检查接口细环/孔半径"

    # 侧管 V 形环：控制点（下叉口→斜上鼓出到 V 顶点加热点→斜上收回上叉口），
    # Catmull-Rom 平滑后扫掠。两端接口处水平（z 恒定）穿入主管壁与孔同心对齐，
    # 出壁后两腿笔直斜交于最鼓的 V 顶点（= 加热点），上下叉口关于顶点对称（横 V）。
    # 两腿取中点保证笔直（下腿 (0.015,0.018)→(0.045,0.0515)、上腿 (0.045,0.0515)→(0.015,0.085)）。
    # 2026-08-31 用户「再突出一点，拐角角度太大，加大管子长度减小拐角角度」：顶点 x 0.028→0.045
    # （突出 32.5mm，V 夹角 137.6°→~96° 更尖锐）；上下叉口 z 不变（0.018/0.085 对称、顶点 0.0515）。
    ctrl = [
        (SIDE_INNER_X, 0, HOLE_CZ[0]),   # 下叉口（贴内壁，z=0.018）
        (0.015, 0, HOLE_CZ[0]),          # 水平穿出下壁
        (0.030, 0, 0.03475),             # 下腿中点（斜上向外）
        (0.045, 0, 0.0515),              # V 顶点 = 加热点（上下叉口正中，最鼓）
        (0.030, 0, 0.06825),             # 上腿中点（斜上向内）
        (0.015, 0, HOLE_CZ[1]),          # 水平穿回上壁（z=0.085）
        (SIDE_INNER_X, 0, HOLE_CZ[1]),   # 上叉口（贴内壁，z=0.085）
    ]
    centerline = catmull_rom(ctrl, n_per_seg=20)
    mb.sweep_tube(centerline, R_SIDE, S, "side_arm")


def build_capillary(mb):
    """毛细管：极细玻璃管，一端圆钝封口（熔封）一端开口。"""
    mb.lathe([
        (0.0,     0.0),        # 封口尖端（r=0，自动收口）
        (0.0005,  0.0004),     # 熔封圆钝过渡
        (CAP_R,   0.0012),     # 到全径
        (CAP_R,   CAP_H),      # 直管到开口端
    ], S, "tube")


GLASS = dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05)

USD_MATS = {
    "body":     GLASS,
    "side_arm": GLASS,
    "tube":     dict(diffuse=(0.90, 0.95, 0.98), opacity=0.35, ior=1.5, roughness=0.05),
}


def make_usd(obj_path, out_usd, mat_specs, groups):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups_parsed = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g in groups:
        faces = groups_parsed[g]
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


def _bbox(out_usd):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    lo = hi = None
    bad_n = 0
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
    return lo, hi, bad_n


def verify(out_usd, name):
    lo, hi, bad_n = _bbox(out_usd)
    sz = (hi - lo) * 1000
    print(f"[verify] {name}: bbox=[{sz[0]:.1f}x{sz[1]:.1f}x{sz[2]:.1f}]mm "
          f"z[{lo[2]*1000:.0f},{hi[2]*1000:.0f}] y[{lo[1]*1000:.1f},{hi[1]*1000:.1f}] "
          f"x[{lo[0]*1000:.1f},{hi[0]*1000:.1f}] bad_normals={bad_n}")
    return lo, hi, bad_n


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)

    # 提勒管
    mb = MeshBuilder()
    build_thiele(mb)
    obj_path = os.path.join(OUT_OBJ, "thiele_tube.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(["body", "side_arm"]))
    out_usd = os.path.join(OUT_USD, "thiele_tube.usd")
    make_usd(obj_path, out_usd, USD_MATS, ["body", "side_arm"])
    lo, hi, bad_n = verify(out_usd, "thiele_tube")
    assert bad_n == 0, "thiele_tube 存在非法法线"
    assert abs(hi[2] - H) < 1e-3 and abs(lo[2]) < 1e-3, "主管高/底 z 异常"
    assert abs(hi[1] - R_OUT) < 1e-3 and abs(lo[1] + R_OUT) < 1e-3, "主管直径异常"
    assert abs(lo[0] + R_OUT) < 1e-3, "主管左壁 x 异常"
    assert 0.043 <= hi[0] <= 0.052, f"侧管 V 顶点 x 异常 {hi[0]:.3f}"

    # 毛细管
    mb = MeshBuilder()
    build_capillary(mb)
    obj_path = os.path.join(OUT_OBJ, "capillary_tube.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(["tube"]))
    out_usd = os.path.join(OUT_USD, "capillary_tube.usd")
    make_usd(obj_path, out_usd, USD_MATS, ["tube"])
    lo, hi, bad_n = verify(out_usd, "capillary_tube")
    assert bad_n == 0, "capillary_tube 存在非法法线"
    assert abs(hi[2] - CAP_H) < 1e-4 and abs(lo[2]) < 1e-4, "毛细管长度异常"
    assert abs(hi[1] - CAP_R) < 1e-4 and abs(lo[1] + CAP_R) < 1e-4, "毛细管直径异常"

    print("DONE")


if __name__ == "__main__":
    main()
