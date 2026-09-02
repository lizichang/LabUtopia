# -*- coding: utf-8 -*-
"""把 burette_acid.usd 缩到 25mL 真实尺寸（859mm → 550mm，scale 0.64）——烘焙版。

v12 docx D10 指定「酸式滴定管（25 mL）」，真实 25mL Class A 滴定管总长 ~55cm；
现有 burette_acid 859mm 是 50mL 级（超长），且全高会让管口到 ~1.85m 超机械臂臂展
（装液步骤够不到管口）。等比缩 0.64（ptfe 旋塞/白条/蓝线/刻度/数字同步缩），
绕 /World 原点（管尖 z≈0.001）缩 → 管尖保持原地、管口降到 ~0.55m。

**烘焙版（非 scale op）**：把 0.64 直接乘进每个 mesh 的 frame0 points（几何在
timeSamples），移除 /World 的 scale op。原因：scale op 在 xformOp 合成里位于最外层，
场景引用时会把场景加的 translate 也一起 ×0.64（滴定管定位跑偏到 z=0.637 而非 0.995）；
烘焙进 points 后资产几何即 550mm，引用时 translate 干净。

幂等：检测 /World 是否已无 scale op 且高度≈550mm → 已烘焙则跳过（避免重复 ×0.64）。
原地 Save（这是本次要改的资产文件本身，非被引用源文件，见 skill 坑 19）。

用法: python scripts/fix_burette_acid_25ml.py
"""
import os
from pxr import Usd, UsdGeom, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "equipment", "burette_acid.usd")

SCALE = 0.64  # 859mm -> 550mm（25mL）
TC = Gf.TimeCode(0.0)


def _height(stage):
    bc = UsdGeom.BBoxCache(TC, [UsdGeom.Tokens.default_])
    rng = bc.ComputeWorldBound(stage.GetPrimAtPath("/World")).ComputeAlignedRange()
    return rng.GetMax()[2] - rng.GetMin()[2]


def main():
    st = Usd.Stage.Open(USD)
    world = st.GetPrimAtPath("/World")
    assert world and world.IsA(UsdGeom.Xform), "/World 缺失或非 Xform"

    # 幂等：已无 scale op 且高度≈550mm → 已烘焙，跳过
    xf = UsdGeom.Xformable(world)
    has_scale = any(o.GetOpName() == "xformOp:scale" for o in xf.GetOrderedXformOps())
    h_now = _height(st)
    if not has_scale and 0.50 < h_now < 0.60:
        print(f"[skip] 已烘焙（无 scale op，高 {h_now*1000:.1f}mm），无需重做")
        return

    # 烘焙：每个 mesh 的 frame0 points ×SCALE
    n = 0
    for p in Usd.PrimRange(world):
        if not p.IsA(UsdGeom.Mesh):
            continue
        attr = p.GetAttribute("points")
        pts = attr.Get(TC)  # 几何在 timeSamples，default 为空（见 usd-asset-frame0 记忆）
        if pts is None or len(pts) == 0:
            continue
        attr.Clear()
        attr.Set([Gf.Vec3f(v[0]*SCALE, v[1]*SCALE, v[2]*SCALE) for v in pts], TC)
        n += 1
    print(f"[bake] {n} meshes points ×{SCALE}")

    # 移除 scale op（若存在）并清空 xformOpOrder
    for o in xf.GetOrderedXformOps():
        if o.GetOpName() == "xformOp:scale":
            o.GetAttr().Clear()
            print("[bake] cleared scale op")
    world.RemoveProperty("xformOpOrder")
    st.GetRootLayer().Save()

    # 验证 bbox：管尖贴原点、管口 ~0.55m、旋塞 ~0.037m
    st2 = Usd.Stage.Open(USD)
    h = _height(st2)
    print(f"[verify] burette_acid 高={h*1000:.1f}mm (期望 ~550mm) -> "
          f"{'OK' if 0.50 < h < 0.60 else 'FAIL'}")
    assert 0.50 < h < 0.60, "高度不在 25mL 预期区间"
    bc = UsdGeom.BBoxCache(TC, [UsdGeom.Tokens.default_])
    plug = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/burette_acid/ptfe_plug")).ComputeAlignedRange()
    print(f"[verify] ptfe_plug z=[{plug.GetMin()[2]:.4f},{plug.GetMax()[2]:.4f}] "
          f"(旋塞保持贴底 ~0.037m) -> {'OK' if plug.GetMin()[2] < 0.05 else 'FAIL'}")
    assert plug.GetMin()[2] < 0.05, "旋塞被缩离管底"
    xf2 = UsdGeom.Xformable(st2.GetPrimAtPath("/World"))
    print(f"[verify] 剩余 xformOp: {[o.GetOpName() for o in xf2.GetOrderedXformOps()]}（应为空）")
    print("DONE")


if __name__ == "__main__":
    main()
