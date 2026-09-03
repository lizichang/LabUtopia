# -*- coding: utf-8 -*-
"""生成滴定管箍环（burette ring clamp）资产，单位：米。

2026-08-27 用户定稿：不要蝴蝶夹，一一复刻铁架台(iron_stand)上那套铁环总成的
三件结构，只把"大铁环"缩成正好箍住酸式滴定管管身的小环。
v4 贴管缩（2026-09-02）：burette_acid 已缩 25mL（tube Ø10.2），铁圈内孔从 Ø17
缩到 Ø11.5（0.65mm 径向余量），环心 RING_CX 钉在管轴 0.113 不变，支臂随环缩
自动加长到环内缘切点（原 3 倍长在 v3 是外移基准，此处以 RING_CX 为不变锚点）。
v3 渐变融合：两处连接不再硬拼——
- collar 端：Ø8 焊球（sphere，圆心在甜甜圈 +X 管心）吞并甜甜圈管端，Ø6 臂杆
  从球内平滑穿出 → 无台阶、圆润过渡；
- ring 端：箍管环管径 = 臂杆管径（同一根铁条弯成环），臂杆终点止于环管心 →
  一体交汇无翻边。

复刻依据（pxr 实测 iron_stand.usd /root/root/ring/ring_002 点云）：
- collar 甜甜圈：套在竖杆上的水平圆环（固定件），外 Ø32 内 Ø16 厚 8mm
  （铁架台竖杆为 12×12mm 方杆，collar 内孔套在杆上）
- arm   圆柱支臂：圆杆（非方杆），从甜甜圈 +X 外缘伸向大环，长 ~31mm
- ring  大铁环：水平圆环，原外 Ø108 厚 8mm → 复刻后缩成内 Ø11.5 外 Ø23.5
  厚 Ø6，环心在 x=0.113（管身中心），竖直滴定管管身穿过环心被箍住

坐标：竖杆轴=原点（collar 环心在原点、管轴沿 z 竖直）；arm 沿 +X；
ring 环心在 x=RING_CX=0.113（= 滴定管管身中心，gen_d1 以 0.113 放滴定管）。
管身 Ø10.2 竖直穿过环心，环内孔 Ø11.5 留 0.65mm 单边余量防共面闪烁；
arm 外缘到管身外缘 0.65mm 间隙（不穿进管身孔）。

材质：铸铁深灰金属（metallic 0.85 / roughness 0.30）。

用法: python gen_burette_ring_clamp.py
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402
from obj2usd import parse_obj, write_mesh  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "burette_ring_obj")

# 尺寸（米）——酸式滴定管 25mL 缩后管身实测 Ø10.2（burette_acid tube_body）
TUBE_OD = 0.0102     # 管身外径 Ø10.2
RING_ID = 0.0115     # 箍管环内径 Ø11.5（贴管缩：箍住 Ø10.2，0.65mm 单边余量）
ARM_R = 0.003        # 臂杆半径 Ø6（圆杆，v4 不变）
RING_TUBE = ARM_R    # 箍管环管径 = 臂杆管径（同一根铁条弯成环 → 一体交汇）
RING_R = RING_ID / 2 + RING_TUBE   # 箍管环环心半径 5.75+3 = 8.75mm

COLLAR_R = 0.012     # 甜甜圈环心半径（外 Ø32 内 Ø16 厚 Ø8 = 复刻铁架台 collar）
COLLAR_TUBE = 0.004  # 甜甜圈截面半径 Ø8
ARM_X0 = COLLAR_R    # 0.012 臂杆起点=甜甜圈 +X 管心（埋进焊球内，端面隐藏）

# 环心 RING_CX 钉在管轴 x=0.113（不变锚点）：gen_d1 以 RING_OFFSET_X=0.113 放滴定管/
# 锥形瓶（世界 0.533），gen_iron_stand 也 verify ring 世界 x == RING_CX。缩环只改半径、
# 不挪环心 → 支臂随环内缘切点自动外移到 ARM_X1 = RING_CX - RING_R（v3 的"3 倍长"
# 是让环随杆端外移，v4 反转为环心钉死不外移、杆长由环半径唯一决定）。
RING_CX = 0.113                    # 箍管环环心 x（= 滴定管管身中心，勿改）
ARM_X1 = RING_CX - RING_R          # 0.10425 臂杆终点=箍管环环心（同径一体交汇）
ARM_LEN = ARM_X1 - (ARM_X0 + COLLAR_TUBE)  # 0.08825 可见杆长（焊球外缘 0.016 → 0.10425）

SEG_U, SEG_V = 48, 24   # 圆环周向/截面段数


def build(mb):
    # ① 甜甜圈 collar：套竖杆的水平圆环（固定件），复刻铁架台 collar 外Ø32 内Ø16
    mb.torus(0.0, 0.0, 0.0, COLLAR_R, COLLAR_TUBE, SEG_U, SEG_V, "collar")
    # ② 圆柱支臂 + 渐变融合：
    #    collar 端：焊球 sphere Ø8（圆心在甜甜圈 +X 管心）吞并甜甜圈管端，
    #               Ø6 臂杆从球内平滑穿出 → 无台阶圆润过渡
    #    ring 端：臂杆管径 = 箍管环管径（同一根铁条弯成环）→ 一体交汇无翻边
    mb.sphere(ARM_X0, 0.0, 0.0, COLLAR_TUBE, 12, SEG_V, "arm")
    mb.h_cylinder((ARM_X0, 0.0, 0.0), (ARM_X1, 0.0, 0.0), ARM_R, 24, "arm")
    # ③ 箍管环 ring：管径=臂径（一体弯管），内孔Ø11.5 箍住竖直穿过环心的滴定管
    mb.torus(RING_CX, 0.0, 0.0, RING_R, RING_TUBE, SEG_U, SEG_V, "ring")


GROUPS = ["collar", "arm", "ring"]

MAT = dict(diffuse=(0.28, 0.28, 0.30), metallic=0.85, roughness=0.30)


def make_usd(obj_path, out_usd):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/{g}", verts, faces, vns)
        mat_path = f"/root/{g}_mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*MAT["diffuse"]))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(MAT["metallic"])
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(MAT["roughness"])
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    bounds = {}
    bad_n = 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = np.array([[v[0], v[1], v[2]] for v in p.GetAttribute("points").Get()])
        nrm = p.GetAttribute("normals").Get()
        if nrm is not None:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            lens = np.linalg.norm(N, axis=1)
            bad_n += int(np.sum((lens > 1.0001) | (lens < 0.9999)))
        bounds[p.GetName()] = (pts.min(0), pts.max(0))
    cmin, cmax = bounds["collar"]
    amin, amax = bounds["arm"]
    rmin, rmax = bounds["ring"]
    # collar: 甜甜圈外 Ø32×厚8，环心原点
    ok1 = abs(cmax[0] - cmin[0] - 2 * (COLLAR_R + COLLAR_TUBE)) < 1e-3 \
        and abs(cmax[2] - cmin[2] - 2 * COLLAR_TUBE) < 1e-3 \
        and abs((cmax[0] + cmin[0]) / 2) < 1e-4 and abs((cmax[1] + cmin[1]) / 2) < 1e-4
    # arm: 焊球 x∈[0.008,0.016]（球心0.012-半径0.004）→ 臂杆止 0.10425；y/z=Ø8（球）
    ok2 = abs(amax[0] - amin[0] - (ARM_X1 - (ARM_X0 - COLLAR_TUBE))) < 1e-3 \
        and abs(amax[1] - amin[1] - 2 * COLLAR_TUBE) < 1e-3 \
        and abs(amax[2] - amin[2] - 2 * COLLAR_TUBE) < 1e-3
    # ring: 外 Ø23.5，环心 x=RING_CX（管身中心），内孔 Ø11.5，厚 Ø6
    ok3 = abs(rmax[0] - rmin[0] - 2 * (RING_R + RING_TUBE)) < 1e-3 \
        and abs(rmax[1] - rmin[1] - 2 * (RING_R + RING_TUBE)) < 1e-3 \
        and abs(rmax[2] - rmin[2] - 2 * RING_TUBE) < 1e-3 \
        and abs((rmax[0] + rmin[0]) / 2 - RING_CX) < 1e-3
    # 支臂外缘不穿进管身孔（arm 外缘 < 管身外缘-余量）
    ok4 = (amax[0] + ARM_R) <= (RING_CX - TUBE_OD / 2)
    # 箍管环内孔单边余量 ~0.65mm：内孔半径(RING_R-RING_TUBE) 比 管半径(TUBE_OD/2) 大 0.0006~0.0007
    ok5 = 0.0005 < (RING_R - RING_TUBE) - TUBE_OD / 2 < 0.0010
    ok = ok1 and ok2 and ok3 and ok4 and ok5 and bad_n == 0
    print(f"[verify] collar=[{(cmax[0]-cmin[0])*1000:.0f}x{(cmax[2]-cmin[2])*1000:.0f}]mm"
          f"(外Ø{2*(COLLAR_R+COLLAR_TUBE)*1000:.0f} 内Ø{2*(COLLAR_R-COLLAR_TUBE)*1000:.0f}) "
          f"arm=[{(amax[0]-amin[0])*1000:.0f}x{(amax[1]-amin[1])*1000:.0f}x{(amax[2]-amin[2])*1000:.0f}]mm"
          f"(焊球Ø{2*COLLAR_TUBE*1000:.0f}+臂杆Ø{2*ARM_R*1000:.0f}) "
          f"ring=[{(rmax[0]-rmin[0])*1000:.0f}x{(rmax[1]-rmin[1])*1000:.0f}x{(rmax[2]-rmin[2])*1000:.0f}]mm"
          f"(内孔Ø{RING_ID*1000:.0f} 外Ø{2*(RING_R+RING_TUBE)*1000:.0f}) 环心x={RING_CX*1000:.0f}mm "
          f"arm_to_tube_gap={(RING_CX-TUBE_OD/2)-(amax[0]+ARM_R):.2f}mm bad_normals={bad_n} "
          f"-> {'OK' if ok else 'FAIL'}")
    assert ok, "verify FAIL"


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    build(mb)
    obj_path = os.path.join(OUT_OBJ, "burette_ring_clamp.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(GROUPS))
    out_usd = os.path.join(OUT_USD, "burette_ring_clamp.usd")
    make_usd(obj_path, out_usd)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
