# -*- coding: utf-8 -*-
"""生成滴定管专用铁架台（iron_stand_burette.usd）：底座+竖杆保留，删除原挂钩
和整组大铁环，把箍环几何【内联烘进】竖杆上——自包含无引用，任何查看器打开都可见。

用户 2026-08-27：安装到铁架台上然后把铁架台上的挂钩、原本的大铁环都去掉来看。
用户反馈：引用方式在 Isaac Sim 里不解析（只看到铁架台本体没看到箍环）→ 改内联。

- 源：assets/equipment/iron_stand.usd（共享资产，勿改）→ 复制到新文件再编辑
- 删除：/root/root/ring（大铁环总成）、/root/root/hook（挂钩）、/root/env_light
  （DomeLight，深灰 1×1 贴图会压黑）、孤儿材质（ring/arm/clamp/hook_*）
- 内联箍环：用 gen_burette_ring_clamp 的 build() 生成网格，写入
  /root/root/BuretteRingClamp/{collar,arm,ring} + 各材质，组 translate z=0.15
  （甜甜圈套竖杆，环在 +X 侧 x=0.113）

用法: python gen_iron_stand_burette.py
"""
import os
import sys
import tempfile
import shutil

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "assets", "equipment", "iron_stand.usd")
DST = os.path.join(REPO, "assets", "equipment", "iron_stand_burette.usd")

MOUNT_Z = 0.15   # 箍环安装高度（竖杆 z 0.01..0.46）

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obj_gen import MeshBuilder          # noqa: E402
from obj2usd import parse_obj, write_mesh  # noqa: E402
import gen_burette_ring_clamp as clamp_mod  # noqa: E402

# 删除的 prim 与孤儿材质
DEL_PRIMS = ["/root/root/ring", "/root/root/hook", "/root/env_light"]
DEL_MATS = ["/root/_materials/ring_mat_002", "/root/_materials/arm_mat_002",
            "/root/_materials/clamp_mat_002", "/root/_materials/hook_clamp_mat_002",
            "/root/_materials/hook_arm_mat_002", "/root/_materials/hook_mat_002"]

MAT = dict(diffuse=(0.28, 0.28, 0.30), metallic=0.85, roughness=0.30)


def add_clamp_meshes(stage, obj_path):
    """把箍环 OBJ 内联写进 stage，组 translate z=MOUNT_Z。"""
    from pxr import UsdGeom, UsdShade, Sdf, Gf
    verts, vns, groups = parse_obj(obj_path)
    group = UsdGeom.Xform.Define(stage, "/root/root/BuretteRingClamp")
    group.AddTranslateOp().Set((0.0, 0.0, MOUNT_Z))
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/root/BuretteRingClamp/{g}", verts, faces, vns)
        mat_path = f"/root/root/BuretteRingClamp/{g}_mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*MAT["diffuse"]))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(MAT["metallic"])
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(MAT["roughness"])
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(mesh).Bind(mat)


def main():
    shutil.copy(SRC, DST)
    stage = None
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(DST)
    for path in DEL_PRIMS:
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
    for path in DEL_MATS:
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
    # 生成箍环网格（内联，不走引用）
    mb = MeshBuilder()
    clamp_mod.build(mb)
    obj_dir = os.path.join(tempfile.gettempdir(), "burette_ring_obj")
    os.makedirs(obj_dir, exist_ok=True)
    obj_path = os.path.join(obj_dir, "burette_ring_clamp.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(clamp_mod.GROUPS))
    add_clamp_meshes(stage, obj_path)
    stage.GetRootLayer().Save()
    verify(DST)
    print("DONE")


def verify(dst):
    from pxr import Usd, UsdGeom, Gf
    stage = Usd.Stage.Open(dst)
    assert stage.GetPrimAtPath("/root/root/base/base_002").IsA(UsdGeom.Mesh)
    assert stage.GetPrimAtPath("/root/root/pole/pole_002").IsA(UsdGeom.Mesh)
    for path in DEL_PRIMS:
        assert not stage.GetPrimAtPath(path), f"未删除 {path}"
    # 箍环内联（无引用）且已到位
    cl = stage.GetPrimAtPath("/root/root/BuretteRingClamp")
    assert cl, "BuretteRingClamp 缺失"
    refs = cl.GetMetadata("references")
    assert refs is None, f"仍有引用 {refs}"
    for name in ("collar", "arm", "ring"):
        p = stage.GetPrimAtPath(f"/root/root/BuretteRingClamp/{name}")
        assert p and p.IsA(UsdGeom.Mesh), f"{name} 缺失"
    bbox = UsdGeom.BBoxCache(Gf.TimeCode(0.0), [UsdGeom.Tokens.default_])
    c_rng = bbox.ComputeWorldBound(stage.GetPrimAtPath("/root/root/BuretteRingClamp/collar")).ComputeAlignedRange()
    r_rng = bbox.ComputeWorldBound(stage.GetPrimAtPath("/root/root/BuretteRingClamp/ring")).ComputeAlignedRange()
    ring_cx = (r_rng.GetMin()[0] + r_rng.GetMax()[0]) / 2
    ring_cy = (r_rng.GetMin()[1] + r_rng.GetMax()[1]) / 2
    ring_cz = (r_rng.GetMin()[2] + r_rng.GetMax()[2]) / 2
    collar_cz = (c_rng.GetMin()[2] + c_rng.GetMax()[2]) / 2
    ok = abs(ring_cz - MOUNT_Z) < 1e-3 and abs(collar_cz - MOUNT_Z) < 1e-3 \
        and abs(ring_cx - clamp_mod.RING_CX) < 1e-3 and abs(ring_cy) < 1e-3
    print(f"[verify] 内联箍环：collar 世界 z={collar_cz:.3f} / ring 世界 "
          f"x={ring_cx:.3f} y={ring_cy:.3f} z={ring_cz:.3f} "
          f"(期望 x={clamp_mod.RING_CX} z={MOUNT_Z}) -> {'OK' if ok else 'FAIL'}")
    assert ok, "verify FAIL"


if __name__ == "__main__":
    main()
