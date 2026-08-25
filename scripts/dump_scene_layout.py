#!/usr/bin/env python3
"""Dump a scene's top-level prims: local xform ops + world bbox.

用法:
    python3 scripts/dump_scene_layout.py <scene.usd> <prim1,prim2,...> [--depth N]

对每个指定 prim（可为场景根）打印其直接子 prim 的局部 xform ops（T/R/S）
与世界 bbox（Usd.TimeCode.Default() 时间）。用于解读用户在 Isaac/UE 里手搭
的 tmp 场景：先 dump 出新布局，再与 gen 脚本当前值对比找差异。

相关 skill: labutopia-scene-realign（用户改 tmp 位置 → 更新生成脚本）。
"""
import sys
from pxr import Usd, UsdGeom, Gf

CACHE = None


def fmt_vec(v):
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def ops_str(xf):
    parts = []
    for op in xf.GetOrderedXformOps():
        t = op.GetOpType()
        v = op.Get()
        if t == UsdGeom.XformOp.TypeTranslate:
            parts.append(f"T{fmt_vec(list(v))}")
        elif t == UsdGeom.XformOp.TypeRotateXYZ:
            parts.append(f"R{fmt_vec(list(v))}")
        elif t == UsdGeom.XformOp.TypeScale:
            parts.append(f"S{fmt_vec(list(v))}")
        else:
            parts.append(t.name)
    return " ".join(parts)


def bbox_str(prim):
    rng = CACHE.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return ""
    mn, mx = rng.GetMin(), rng.GetMax()
    return (f"bbox min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
            f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")


def walk(prim, depth, max_depth):
    if depth > max_depth:
        return
    xf = UsdGeom.Xformable(prim)
    print("  " * depth + f"{prim.GetName():<24} {ops_str(xf):<52} {bbox_str(prim)}")
    for c in prim.GetChildren():
        if c.GetTypeName() in ("Scope", "Material", "Shader", "RenderPass"):
            continue
        walk(c, depth + 1, max_depth)


def main():
    global CACHE
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    depth = 1
    if "--depth" in sys.argv:
        depth = int(sys.argv[sys.argv.index("--depth") + 1])
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    path, roots = args[0], args[1].split(",")
    stage = Usd.Stage.Open(path)
    CACHE = UsdGeom.BBoxCache(Gf.TimeCode(), [UsdGeom.Tokens.default_])
    for r in roots:
        p = stage.GetPrimAtPath(r)
        if not p.IsValid():
            print(f"MISSING {r}")
            continue
        print(f"\n## {r}")
        for c in p.GetChildren():
            xf = UsdGeom.Xformable(c)
            print(f"  {c.GetName():<24} {ops_str(xf):<52} {bbox_str(c)}")
            if depth > 1:
                for cc in c.GetChildren():
                    walk(cc, 2, depth)


if __name__ == "__main__":
    main()
