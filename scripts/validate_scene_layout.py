#!/usr/bin/env python3
"""Validate that asset-reference + optional rotateZ180 + translate reproduces target world bboxes.

用法:
    python3 scripts/validate_scene_layout.py layout.json

layout.json（从 dump_scene_layout.py 输出的 tmp bbox 抄录）：
    {
      "eq": "assets/equipment",
      "cases": [
        {"name":"IronStand","asset":"iron_stand.usd","t":[0.6286,0.0029,0.80],"rot180":true,
         "min":[0.4746,-0.0621,0.80],"max":[0.7810,0.0679,1.26]},
        {"name":"AlcoholLamp","asset":"alcohol_lamp.usd","t":[0.5286,0.0029,0.8002],"rot180":true,
         "min":[0.4850,-0.0407,0.80],"max":[0.5722,0.0465,0.9007]}
      ]
    }

对每项同时试两种 op 顺序：
    A [R,T]：AddRotateXYZOp(180) 先、AddTranslateOp 后  → pxr 净效果=先平移再绕局部原点旋转
    B [T,R]：AddTranslateOp 先、AddRotateXYZOp(180) 后 → pxr 净效果=先绕局部原点旋转再平移
与目标 bbox 吻合的那个顺序就是 gen 脚本该用的 op 序（B2 实证=B 正确）。

相关 skill: labutopia-scene-realign（用户改 tmp 位置 → 更新生成脚本）。
"""
import json
import sys
from pxr import Usd, UsdGeom, Gf


def wbbox(stage, prim_path):
    bc = UsdGeom.BBoxCache(Gf.TimeCode(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
    return tuple(r.GetMin()), tuple(r.GetMax())


def approx(a, b, tol=0.002):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1], encoding="utf-8") as f:
        cfg = json.load(f)
    eq = cfg["eq"]
    for c in cfg["cases"]:
        name, asset, t, rot = c["name"], c["asset"], c["t"], c.get("rot180", False)
        emn, emx = c["min"], c["max"]
        ok = {}
        for label, order in (("A[R,T]", 0), ("B[T,R]", 1)):
            st = Usd.Stage.CreateInMemory()
            prim = UsdGeom.Xform.Define(st, "/World/" + name)
            prim.GetPrim().GetReferences().AddReference(f"{eq}/{asset}")
            if order == 0:  # A: rotate then translate
                if rot:
                    prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
                prim.AddTranslateOp().Set(Gf.Vec3d(*t))
            else:           # B: translate then rotate
                prim.AddTranslateOp().Set(Gf.Vec3d(*t))
                if rot:
                    prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 180))
            amn, amx = wbbox(st, "/World/" + name)
            ok[label] = approx(amn, emn) and approx(amx, emx)
            print(f"  {label} min{tuple(round(x,4) for x in amn)} max{tuple(round(x,4) for x in amx)}")
        print(f"{name:<15} A[R,T]={'PASS' if ok['A[R,T]'] else 'FAIL'}  B[T,R]={'PASS' if ok['B[T,R]'] else 'FAIL'}")


if __name__ == "__main__":
    main()
