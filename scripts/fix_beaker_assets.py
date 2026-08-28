# -*- coding: utf-8 -*-
"""修复烧杯资产的两个 bug（缺一烧杯就进不了场景）：

1. 坑 R3 时间采样：网格属性只在 time 0.0 有值、default 为空（attr.GetTimeSamples()=[0.0]、
   attr.Get()=None）→ 被引用后 default 时间不可见。修复：每个时间采样属性取 t=0 值 →
   Clear() → 在 default 写入（干净 default）。

2. defaultPrim=/World：材质在 /World/Looks/glass、网格在 /root/beaker_xxx。AddReference 只
   组合文件 defaultPrim，所以场景只带进 /World（=只有材质），网格 /root/... 整个丢失 → 烧杯
   在烘平场景里 ±inf（不可见）。修复：材质挪到 /root/Looks/glass、网格绑定改指 /root、
   defaultPrim 改 /root（网格+材质在同一子树，引用后绑定 remap 到 /World/<Beaker>/Looks/glass）。

幂等：defaultPrim 已是 /root 且无时间采样 → no-op。
"""
import os

from pxr import Usd, UsdGeom, UsdShade, Sdf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EQ = os.path.join(REPO, "assets", "equipment")
BEAKERS = ["beaker_111x75x116.usd", "beaker_156x105x162.usd"]

# 修复后预期 bbox（资产局部；Ø×Ø×高）
EXPECT = {
    # 2026-08-27 绕 Z 轴 yaw -90°（干净轮廓朝 +Y=camera2），x/y 互换，底座仍在 z=0
    "beaker_111x75x116.usd": ((-0.038, -0.056, 0.0), (0.038, 0.056, 0.116)),
    "beaker_156x105x162.usd": ((-0.078, -0.0525, 0.0), (0.078, 0.0525, 0.162)),
}


def fix_time_sampled_to_default(attr):
    """把只带时间采样的属性转成干净 default（t=0 值 → Clear → default Set）。返回是否改过。"""
    ts = attr.GetTimeSamples()
    if not ts:
        return False
    v0 = attr.Get(0.0)
    attr.Clear()                 # 清掉时间采样
    attr.Set(v0)                 # 写 default
    assert not attr.GetTimeSamples(), f"{attr.GetPath()} 仍有时间采样"
    assert attr.Get() is not None, f"{attr.GetPath()} default 仍为空"
    print(f"[fix] {attr.GetPath()} 采样 {len(ts)} 帧 -> 干净 default")
    return True


def restructure_default_prim(stage):
    """defaultPrim /World -> /root：材质 /World/Looks/glass 挪到 /root/Looks/glass、
    网格绑定改指 /root/Looks/glass、删除空 /World。幂等（已是 /root 则跳过）。"""
    if str(stage.GetDefaultPrim().GetPath()) == "/root":
        print("[ok] defaultPrim 已是 /root（幂等）")
        return False

    # 复制材质（UsdPreviewSurface shader 输入逐个拷到新位置）
    new_mat = UsdShade.Material.Define(stage, "/root/Looks/glass")
    old_shader = UsdShade.Shader(stage.GetPrimAtPath("/World/Looks/glass/shader"))
    new_shader = UsdShade.Shader.Define(stage, "/root/Looks/glass/shader")
    new_shader.CreateIdAttr(old_shader.GetIdAttr().Get())
    for inp in old_shader.GetInputs():
        new_shader.CreateInput(inp.GetBaseName(), inp.GetTypeName()).Set(inp.Get())
    new_mat.CreateSurfaceOutput().ConnectToSource(new_shader.ConnectableAPI(), "surface")

    # 网格绑定改指 /root/Looks/glass
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Mesh":
            continue
        rel = prim.GetRelationship("material:binding")
        if rel and rel.GetTargets():
            rel.SetTargets([Sdf.Path("/root/Looks/glass")])
            print(f"[fix] {prim.GetPath()} 绑定 -> /root/Looks/glass")

    # 删除旧材质 + 空 /World
    stage.RemovePrim("/World/Looks/glass")
    world = stage.GetPrimAtPath("/World")
    if world.IsValid() and not list(world.GetChildren()):
        stage.RemovePrim("/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/root"))
    print("[fix] defaultPrim /World -> /root；材质挪到 /root/Looks/glass")
    return True


def fix_file(filename):
    path = os.path.join(EQ, filename)
    stage = Usd.Stage.Open(path)
    touched = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Mesh":
            continue
        for attr in prim.GetAttributes():
            if fix_time_sampled_to_default(attr):
                touched += 1
    touched += 1 if restructure_default_prim(stage) else 0
    if touched:
        stage.GetRootLayer().Save()
        print(f"[save] {filename} ({touched} changes)")
    else:
        print(f"[ok] {filename} 已无时间采样、defaultPrim 已是 /root（幂等 no-op）")
    return touched


def verify():
    for filename in EXPECT:
        st = Usd.Stage.Open(os.path.join(EQ, filename))
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
        lo, hi = r.GetMin(), r.GetMax()
        exp_lo, exp_hi = EXPECT[filename]
        ok = all(abs(lo[i] - exp_lo[i]) < 1e-3 for i in range(3)) and \
             all(abs(hi[i] - exp_hi[i]) < 1e-3 for i in range(3))
        print(f"[verify] {filename} bbox {[round(c,3) for c in lo]}..{[round(c,3) for c in hi]} "
              f"期望 {exp_lo}..{exp_hi}: {'OK' if ok else 'FAIL'}")
        assert ok, f"{filename} bbox 不符"


def main():
    for f in BEAKERS:
        fix_file(f)
    verify()
    print("SAVED all beakers")


if __name__ == "__main__":
    main()
