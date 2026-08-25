# -*- coding: utf-8 -*-
"""重建 assets/equipment/thermometer.usd 的几何（2026-08-24）。

背景：温度计资产只有 prim 骨架（名称 + 材质绑定），所有 mesh 的 points 为空，
只有 /World/Thermometer/hanging_ring（加环脚本产物）有几何 —— 温度计不可见。

本脚本在现有结构上补几何，保留材质绑定 / 挂环 / physics 属性：
    stem              Ø8 玻璃圆柱 z[0.008,0.268]
    bulb              Ø10 玻璃球 r=0.005 心(0,0,0.003) z[-0.002,0.008]
    bulb_liquid       红液球 r=0.0042 心(0,0,0.003)
    capillary_liquid  红液柱 r=0.0012 z[0.005,0.245]（**全量程** -20..110°C，原只到
                      室温 25°C 刻度；任务用锚定缩放 transform 驱动柱顶随实时温度爬升，
                      底锚 z=0.005 不变）
    white_backing     白底条 z[0.018,0.24]（毛细管后衬，红色读数清晰）
    scale/*           刻度：major 每10°C / med 每5°C / minor 每1°C（-20..110°C），
                      黑色小盒 + translate op 定位；num_* 空 prim 删除（无字形）
    hanging_ring      保留（孔轴 X，套铁架台横臂）

刻度位置：z(T) = 0.02 + (T+20)/130*0.22  (T=-20..110 -> z 0.02..0.24)
metersPerUnit = 1.0；subdivisionScheme = none（坑26）。
"""
import math
import shutil
import os

from pxr import Usd, UsdGeom, Sdf, Gf

PATH = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/equipment/thermometer.usd"
THERMO = "/World/Thermometer"

# ---- 尺寸 ----
STEM_R = 0.004          # 杆 Ø8
STEM_Z0, STEM_Z1 = 0.008, 0.268
BULB_R = 0.005          # 泡 Ø10
BULB_CZ = 0.003         # 泡心 z -> 泡 z[-0.002,0.008]
LIQ_R = 0.0042          # 泡内红液球
CAP_R = 0.0012          # 毛细红液柱 Ø2.4
T_ROOM = 25             # 室温液位对应刻度
T_MIN, T_MAX = -20, 110
Z_SCALE_LO, Z_SCALE_HI = 0.02, 0.24


def z_of(T):
    return Z_SCALE_LO + (T - T_MIN) / (T_MAX - T_MIN) * (Z_SCALE_HI - Z_SCALE_LO)


def cylinder_geo(r, z0, z1, seg=36, cap_top=True, cap_bot=True):
    pts, nrm, fvc, fvi = [], [], [], []
    for i in range(seg):
        th = 2 * math.pi * i / seg
        pts.append((r * math.cos(th), r * math.sin(th), z0))
        nrm.append((math.cos(th), math.sin(th), 0.0))
    for i in range(seg):
        th = 2 * math.pi * i / seg
        pts.append((r * math.cos(th), r * math.sin(th), z1))
        nrm.append((math.cos(th), math.sin(th), 0.0))
    for i in range(seg):
        i2 = (i + 1) % seg
        fvi += [i, i2, seg + i2, seg + i]
        fvc.append(4)
    if cap_bot:
        ci = len(pts)
        pts.append((0.0, 0.0, z0))
        nrm.append((0.0, 0.0, -1.0))
        for i in range(seg):
            i2 = (i + 1) % seg
            fvi += [ci, i2, i]
            fvc.append(3)
    if cap_top:
        ci = len(pts)
        pts.append((0.0, 0.0, z1))
        nrm.append((0.0, 0.0, 1.0))
        for i in range(seg):
            i2 = (i + 1) % seg
            fvi += [ci, seg + i, seg + i2]
            fvc.append(3)
    return pts, nrm, fvc, fvi


def sphere_geo(r, cx, cy, cz, stacks=24, slices=36):
    pts, nrm, fvc, fvi = [], [], [], []
    for i in range(stacks + 1):
        phi = math.pi * i / stacks
        for j in range(slices + 1):
            th = 2 * math.pi * j / slices
            sp, cp = math.sin(phi), math.cos(phi)
            pts.append((cx + r * sp * math.cos(th), cy + r * sp * math.sin(th), cz + r * cp))
            nrm.append((sp * math.cos(th), sp * math.sin(th), cp))
    for i in range(stacks):
        for j in range(slices):
            a = i * (slices + 1) + j
            b = a + slices + 1
            fvi += [a, b, b + 1, a + 1]
            fvc.append(4)
    return pts, nrm, fvc, fvi


def box_geo(hx, hy, hz):
    """以原点为中心的盒（faceVarying 法线，每角一个）。"""
    x0, x1, y0, y1, z0, z1 = -hx, hx, -hy, hy, -hz, hz
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [((0, 1, 2, 3), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)),
             ((0, 1, 5, 4), (0, -1, 0)), ((3, 2, 6, 7), (0, 1, 0)),
             ((0, 3, 7, 4), (-1, 0, 0)), ((1, 2, 6, 5), (1, 0, 0))]
    fvi, fvc, nrm = [], [], []
    for idxs, n in faces:
        fvi += list(idxs)
        fvc.append(4)
        nrm += [n] * 4
    return pts, nrm, fvc, fvi


def set_mesh(prim, pts, nrm, fvc, fvi):
    m = UsdGeom.Mesh(prim)
    m.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
    m.GetNormalsAttr().Set([Gf.Vec3f(*n) for n in nrm])
    m.GetFaceVertexCountsAttr().Set(fvc)
    m.GetFaceVertexIndicesAttr().Set(fvi)
    m.GetSubdivisionSchemeAttr().Set("none")
    assert len(fvi) == sum(fvc), f"face refs mismatch {prim.GetPath()}"
    assert max(fvi) < len(pts), f"index OOB {prim.GetPath()}"
    if len(nrm) == len(pts):
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    elif len(nrm) == len(fvi):
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    else:
        raise AssertionError(f"nrm count {len(nrm)} invalid for {prim.GetPath()}")


def set_translate(prim, t):
    xf = UsdGeom.Xformable(prim)
    tr = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            tr = op
            break
    if tr is None:
        tr = xf.AddTranslateOp()
    tr.Set(Gf.Vec3d(*t))


def main():
    if not os.path.exists(PATH + ".bak2"):
        shutil.copy2(PATH, PATH + ".bak2")
    stage = Usd.Stage.Open(PATH)
    tl = stage.GetRootLayer()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    T = stage.GetPrimAtPath(THERMO)
    assert T.IsValid(), "thermometer prim missing"

    # stem / bulb / 液体 / 白底 —— 直接填几何
    set_mesh(stage.GetPrimAtPath(THERMO + "/stem"), *cylinder_geo(STEM_R, STEM_Z0, STEM_Z1))
    set_mesh(stage.GetPrimAtPath(THERMO + "/bulb"), *sphere_geo(BULB_R, 0, 0, BULB_CZ))
    set_mesh(stage.GetPrimAtPath(THERMO + "/bulb_liquid"), *sphere_geo(LIQ_R, 0, 0, BULB_CZ))
    # 毛细柱扩展到全量程 z[0.005, 0.245]（-20..110°C 刻度区顶），任务用锚定缩放
    # transform 驱动柱顶随实时温度爬升（底锚 z=0.005）。原 z_of(25)=0.0962 只够
    # 显示室温，不够显示沸点。
    set_mesh(stage.GetPrimAtPath(THERMO + "/capillary_liquid"),
             *cylinder_geo(CAP_R, 0.005, Z_SCALE_HI + 0.005))
    set_mesh(stage.GetPrimAtPath(THERMO + "/white_backing"),
             *box_geo(0.0023, 0.0002, 0.111))
    set_translate(stage.GetPrimAtPath(THERMO + "/white_backing"), (0.0, -0.0008, 0.129))

    # 刻度：major/med/minor 黑盒 + translate；num_* 空 prim 删除
    scale = stage.GetPrimAtPath(THERMO + "/scale")
    n_num = 0
    for prim in list(scale.GetChildren()):
        nm = prim.GetName()
        if nm.startswith("num_"):
            stage.RemovePrim(prim.GetPath())
            n_num += 1
    tick_spec = {  # name_prefix -> (hx, hy, hz)
        "major": (0.0009, 0.0002, 0.0012),
        "med":   (0.0007, 0.00015, 0.0009),
        "minor": (0.0005, 0.00015, 0.0007),
    }
    for prim in list(scale.GetChildren()):
        nm = prim.GetName()
        kind = next((k for k in tick_spec if nm.startswith(k + "_")), None)
        if kind is None:
            continue
        T_val = int(nm.split("_")[-1].replace("neg", "-"))
        set_mesh(prim, *box_geo(*tick_spec[kind]))
        set_translate(prim, (0.0010, 0.0004, z_of(T_val)))

    # 自检
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bad = 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if p.GetTypeName() != "Mesh":
            continue
        n = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if n is None or len(n) == 0:
            print(f"  EMPTY {p.GetPath()}")
            bad += 1
    r = bc.ComputeWorldBound(T).ComputeAlignedRange()
    print(f"[verify] thermometer bbox x[{r.GetMin()[0]:+.4f},{r.GetMax()[0]:+.4f}] "
          f"y[{r.GetMin()[1]:+.4f},{r.GetMax()[1]:+.4f}] "
          f"z[{r.GetMin()[2]:+.4f},{r.GetMax()[2]:+.4f}]  npts-ok")
    assert bad == 0, f"{bad} empty meshes remain"
    assert abs(r.GetMin()[2] + 0.002) < 0.001, "bulb bottom should be z=-0.002"
    assert abs(r.GetMax()[2] - 0.2762) < 0.002, "ring top should be ~0.2762"
    tl.Save()
    print(f"SAVED {PATH} (deleted {n_num} empty num_* prims)")


if __name__ == "__main__":
    main()
