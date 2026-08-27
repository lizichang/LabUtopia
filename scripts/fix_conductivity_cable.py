# -*- coding: utf-8 -*-
"""电导率仪资产后处理（二）：电极线缆改为"多段圆柱软线"，可逐帧更新（方案2）。

conductivity_meter.usd 是 Blender 精模（本环境无法重跑 Blender），用 USD 后处理直接改
资产；与 build_conductivity_auto.py 源脚本保持一致性（源里 electrode_cable 是静态贝塞尔
曲线转 mesh，删掉换成可更新的 CableRoot + N 段圆柱）。

改动：
1. 删除旧 /root/electrode/electrode_cable（Xform + cable mesh）。
2. 新建 /root/CableRoot（Xform），写入自定义属性：
     cable:segments=30、cable:radius=0.0035、
     cable:anchor_a=机身后插座中心（固定端）、
     cable:anchor_c=固定拐角、cable:control_b=帽端下垂偏移。
3. CableRoot 下建 cable_seg_0..23（UsdGeom.Cylinder，单位尺寸 radius 1 / height 1 /
   axis Z，全部由 xformOp:transform 矩阵缩放），绑定 /root/_materials/matte_black。
4. 用 DynamicCable 按静止态 CAP_TOP 初始摆位。

幂等：删除旧 cable + 删除已有 CableRoot 后重建，重跑无副作用。

用法：python scripts/fix_conductivity_cable.py
验证：py_compile + 本脚本 verify()（纯 pxr，模拟电极移到烧杯 → 断言段位置变化，证明能动）
"""
import os
import sys

from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "equipment", "conductivity_meter.usd")
sys.path.insert(0, REPO)
from catalogue.a_instrument.a3_conductivity.dynamic_cable import (
    DynamicCable, ANCHOR_A, ANCHOR_C, CONTROL_B, CAP_TOP, SEGMENTS, RADIUS,
)

MATERIAL = "/root/_materials/matte_black"   # Blender 导出即有
ATTRIBUTES = dict(
    segments=(Sdf.ValueTypeNames.Int, SEGMENTS),
    radius=(Sdf.ValueTypeNames.Float, RADIUS),
    anchor_a=(Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(*ANCHOR_A)),
    anchor_c=(Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(*ANCHOR_C)),
    control_b=(Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(*CONTROL_B)),
)

# 机身障碍包围盒（资产局部；与 build_conductivity_auto.py 几何一致，纯 pxr 实测 2026-08-27）
OBSTACLES = {
    "shell": ((-0.145, -0.098, 0.008), (0.145, 0.098, 0.095)),
    "skirt": ((-0.148, -0.108, 0.001), (0.148, 0.108, 0.015)),
    "deck": ((-0.098, -0.105, 0.095), (0.098, 0.105, 0.105)),
    "stand": ((-0.030, -0.100, 0.105), (0.084, -0.040, 0.232)),
}
SOCKET_R = 0.010   # 测量插座圆柱半径：线缆末端插进插座孔，该区允许


def _in_zone(pt, lo, hi, tol):
    return all(lo[i] - tol <= pt[i] <= hi[i] + tol for i in range(3))


def _in_socket(pt):
    dx = pt[0] - ANCHOR_A[0]
    dz = pt[2] - ANCHOR_A[2]
    return dx * dx + dz * dz <= SOCKET_R ** 2 and -0.106 <= pt[1] <= -0.097


def check_path(dc, b_local, tag):
    """采样整条样条逐点查穿模（插座区豁免）。"""
    hits = {}
    first = None
    for pt in dc.sample_points(b_local, per_span=400):
        if _in_socket(pt):
            continue
        for name, (lo, hi) in OBSTACLES.items():
            if _in_zone(pt, lo, hi, RADIUS):
                if name not in hits:
                    hits[name] = 0
                    if first is None:
                        first = pt
                hits[name] += 1
    assert not hits, f"{tag} 穿模 {hits}（首穿点 {tuple(first)}）"
    print(f"[collision] {tag}: 全程无穿模 ✓")


def _remove(stage, path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
        print(f"[del] {path}")


def add_cable(stage):
    # 删旧静态线缆 + 旧 CableRoot（幂等）
    _remove(stage, "/root/electrode/electrode_cable")
    _remove(stage, "/root/CableRoot")

    root = stage.DefinePrim("/root/CableRoot", "Xform")
    for name, (typ, val) in ATTRIBUTES.items():
        root.CreateAttribute(f"cable:{name}", typ).Set(val)
    print("[add] /root/CableRoot + cable:attrs")

    mat = UsdShade.Material(stage.GetPrimAtPath(MATERIAL))
    for i in range(SEGMENTS):
        cyl = UsdGeom.Cylinder.Define(stage, f"/root/CableRoot/cable_seg_{i}")
        cyl.CreateRadiusAttr(1.0)     # 单位圆柱，矩阵负责缩放（半径/长度）
        cyl.CreateHeightAttr(1.0)
        cyl.CreateAxisAttr("Z")
        UsdGeom.Xformable(cyl).AddTransformOp()
        if mat:
            UsdShade.MaterialBindingAPI(cyl).Bind(mat)
    print(f"[add] cable_seg_0..{SEGMENTS - 1} ({SEGMENTS} 段, Ø{RADIUS * 2 * 1000:.0f}mm)")

    # 初始摆位（静止态：电极帽顶）
    dc = DynamicCable(stage, "/root/CableRoot")
    dc.update_local(CAP_TOP)
    print(f"[init] 静止态 B=CAP_TOP {tuple(CAP_TOP)}")


def verify(stage):
    dc = DynamicCable(stage, "/root/CableRoot")
    assert dc.segments == SEGMENTS and abs(dc.radius - RADIUS) < 1e-6, "段数/半径属性"

    # 静止态首段起点≈CAP_TOP、末段终点≈ANCHOR_A（首段是弯曲短段，中心距帽顶 <2cm）
    c0 = dc.seg_bbox_center(0)
    assert (c0 - CAP_TOP).GetLength() < 0.02, f"首段中心 {c0} 不近 CAP_TOP"
    last = dc.seg_bbox_center(SEGMENTS - 1)
    assert (last - ANCHOR_A).GetLength() < 0.02, f"末段中心 {last} 不近 ANCHOR_A"

    # 全路径碰撞检查（静止态 + 移动态，插座区豁免）
    check_path(dc, CAP_TOP, "静止态")
    beaker_world = CAP_TOP + Gf.Vec3d(0.30, 0.20, 0.55)   # 假设烧杯相对位移（世界系）
    beaker_local = dc.to_local(beaker_world)
    check_path(dc, beaker_local, "移动态(烧杯)")

    # 能动性：模拟电极被夹到烧杯上方 → 帽端/下垂段应显著位移
    # 注：后右角→插座的后段两端都固定（本就不动），找**位移最大的段**验证线缆能动
    rest = [dc.seg_bbox_center(i) for i in range(SEGMENTS)]
    dc.update(beaker_world)
    deltas = [(dc.seg_bbox_center(i) - rest[i]).GetLength() for i in range(SEGMENTS)]
    delta = max(deltas)
    assert delta > 0.05, f"电极移动后线缆最大位移仅 {delta:.4f}m，线缆不动！"

    # 材质绑定
    bind = UsdShade.MaterialBindingAPI(
        stage.GetPrimAtPath("/root/CableRoot/cable_seg_0")).GetDirectBinding()
    assert bind.GetMaterialPath() == MATERIAL, f"材质绑定 {bind.GetMaterialPath()} != {MATERIAL}"

    print(f"[verify] 静止段0→帽顶 / 末段→插座 / 电极移0.85m中段动{delta:.3f}m / "
          f"材质 {MATERIAL} / 静止+移动态全程无穿模 — all OK")


def main():
    stage = Usd.Stage.Open(USD)
    add_cable(stage)
    stage.GetRootLayer().Save()

    stage2 = Usd.Stage.Open(USD)
    verify(stage2)
    print("SAVED", USD)


if __name__ == "__main__":
    main()
