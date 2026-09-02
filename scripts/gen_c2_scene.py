# -*- coding: utf-8 -*-
"""生成 c2_cobalt_glass.usd —— C2「焰色反应（隔钴玻璃观察）」场景。

基于 c1_flame_wire_solid.usd（C1 焰色反应场景，器材已烘平自包含）：复制整目录 →
加一个固定钴玻璃（竖立、面朝 +Y、柄朝下触桌面），摆酒精灯 +Y 侧，供 camera2 从
+Y 朝 -Y 透过玻璃看火焰。C2 = C1 完整灼烧流程 + 多一块固定钴玻璃（机械臂不再抓玻璃）。

钴玻璃资产（assets/equipment/cobalt_glass.usd，defaultPrim /root，Z-up 米制）：
  玻璃 glass 100×100×3mm 圆角方片（底面 z=0，中心在原点，法线 +Z）
  包边 frame 108×108×3mm 金属边
  箍   collar Φ5×7mm（x 0.053→0.060）
  柄   handle Φ4×80mm + 末端圆头（x 0.058→0.138，沿 +X）
原点 = 玻璃底面中心。

竖立摆放：旋转四元数 (w,x,y,z)=(0.5,-0.5,0.5,0.5)（绕 (1,-1,-1)/√3 转 120°）：
  +X(柄) → -Z（柄朝下）、+Z(玻璃法线) → +Y（面朝 +Y）、+Y → -X。
玻璃底面中心原点放 (0.5132, 0.68, 0.938)（酒精灯 0.5132,0.5256 的 +Y 侧 0.154m）：
  → 玻璃片 z 0.888–0.988（罩住火焰 z 0.898–0.940）、柄尖 z=0.800（触桌面）。

用法：python scripts/gen_c2_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
C1_DIR = os.path.join(REPO, "assets", "scenes", "c_flame", "c1_flame_wire_solid")
C2_DIR = os.path.join(REPO, "assets", "scenes", "c_flame", "c2_cobalt_glass")
C1_USD = os.path.join(C1_DIR, "c1_flame_wire_solid.usd")
C2_USD = os.path.join(C2_DIR, "c2_cobalt_glass.usd")
EQ = os.path.join(REPO, "assets", "equipment")

GLASS_ASSET = "cobalt_glass.usd"
# 玻璃底面中心（资产 /root 原点）世界坐标：酒精灯 +Y 侧，玻璃片罩火焰高度
GLASS_POS = (0.5132, 0.68, 0.938)
# Gf.Quatd(w,x,y,z)：+X→-Z（柄朝下）、+Z→+Y（玻璃法线朝 +Y，面朝相机）
GLASS_QUAT = (0.5, -0.5, 0.5, 0.5)


def add_glass(stage):
    xf = UsdGeom.Xform.Define(stage, "/World/CobaltGlass")
    # 相对引用（相对 c2_cobalt_glass.usd 所在目录 → assets/equipment/cobalt_glass.usd）
    xf.GetPrim().GetReferences().AddReference("../../../equipment/" + GLASS_ASSET)
    xf.AddTranslateOp().Set(Gf.Vec3d(*GLASS_POS))
    xf.AddOrientOp().Set(Gf.Quatf(*GLASS_QUAT))
    print(f"[glass] CobaltGlass <- {GLASS_ASSET} at {GLASS_POS} orient{GLASS_QUAT}")


def verify(st2):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    p = st2.GetPrimAtPath("/World/CobaltGlass")
    assert p.IsValid(), "/World/CobaltGlass missing"
    kids = [c.GetName() for c in p.GetChildren()]
    print(f"[verify] CobaltGlass children: {kids}")
    for sub in ["glass", "frame", "collar", "handle"]:
        sp = st2.GetPrimAtPath(f"/World/CobaltGlass/{sub}")
        if not sp.IsValid():
            print(f"[verify] /World/CobaltGlass/{sub} MISSING")
            continue
        r = bc.ComputeWorldBound(sp).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        print(f"[verify] {sub:8s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")

    gr = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/CobaltGlass/glass")).ComputeAlignedRange()
    gmn, gmx = gr.GetMin(), gr.GetMax()
    gsz = gmx - gmn
    assert abs(gsz[1] - 0.003) < 0.002, f"玻璃片厚 {gsz[1]} 应为 ~3mm（竖立）"
    assert gmn[2] > 0.885 and gmx[2] < 0.990, f"玻璃片 z [{gmn[2]},{gmx[2]}] 应罩火焰 0.898–0.940"
    assert abs((gmn[0] + gmx[0]) / 2 - GLASS_POS[0]) < 0.002, "玻璃片 x 中心应≈灯 x"
    print(f"[verify] glass disc: size {gsz[0]*1000:.0f}x{gsz[1]*1000:.0f}x{gsz[2]*1000:.0f}mm "
          f"center y≈{(gmn[1]+gmx[1])/2:.4f}  (face +Y, z 罩火焰)")

    hr = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/CobaltGlass/handle")).ComputeAlignedRange()
    hmn, hmx = hr.GetMin(), hr.GetMax()
    assert hmn[2] < 0.805 and hmx[2] > 0.87, f"柄 z [{hmn[2]},{hmx[2]}] 应朝下触桌面"
    print(f"[verify] handle: z [{hmn[2]:.4f},{hmx[2]:.4f}] (tip≈0.800 桌面)")


def main():
    if os.path.exists(C2_DIR):
        shutil.rmtree(C2_DIR)
    shutil.copytree(C1_DIR, C2_DIR)
    os.rename(os.path.join(C2_DIR, "c1_flame_wire_solid.usd"), C2_USD)

    stage = Usd.Stage.Open(C2_USD)
    add_glass(stage)
    stage.GetRootLayer().Save()
    print("SAVED", C2_USD)

    st2 = Usd.Stage.Open(C2_USD)
    verify(st2)
    print("DONE")


if __name__ == "__main__":
    main()
