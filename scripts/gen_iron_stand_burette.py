# -*- coding: utf-8 -*-
"""生成滴定管专用铁架台（iron_stand_burette.usd）：底座+竖杆保留，删除原挂钩
和整组大铁环，把箍环几何【内联烘进】竖杆上——自包含无引用，任何查看器打开都可见。

用户 2026-08-27：安装到铁架台上然后把铁架台上的挂钩、原本的大铁环都去掉来看。
用户反馈：引用方式在 Isaac Sim 里不解析（只看到铁架台本体没看到箍环）→ 改内联。

- 源：assets/equipment/iron_stand.usd（共享资产，勿改）→ 复制到新文件再编辑
- 删除：/root/root/ring（大铁环总成）、/root/root/hook（挂钩）、/root/env_light
  （DomeLight，深灰 1×1 贴图会压黑）、孤儿材质（ring/arm/clamp/hook_*）
- 竖杆加高：源 iron_stand 竖杆仅 450mm，滴定场景须夹住整支滴定管（颈 local z≈0.53）
  且管尖落在锥形瓶口上方 3cm → extend_pole() 把竖杆顶端从 0.46 拉到 POLE_TOP=0.78
- 内联箍环：用 gen_burette_ring_clamp 的 build() 生成网格，写入
  /root/root/BuretteRingClamp/{collar,arm,ring} + 各材质，组 translate z=MOUNT_Z=0.725
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

POLE_TOP = 0.78    # 竖杆加高后的顶端 z（原 0.46m 太短，夹不住整支滴定管、管尖够不到锥形瓶）
MOUNT_Z = 0.725   # 箍环安装高度（25mL 滴定管颈 local z≈0.53 对齐箍环 → 管尖落 ~0.995）

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


def extend_pole(stage, pole_path="/root/root/pole/pole_002", bottom=0.010, top=POLE_TOP):
    """竖杆（12×12 方杆）沿 z 拉伸到底=bottom、顶=top：底端贴底座不动，顶端抬升。

    源 iron_stand 竖杆 450mm（z 0.010..0.460）对滴定场景太短——滴定管须夹在颈部
    （local z≈0.53）而管尖还要落在锥形瓶口上方 3cm，箍环得装到 ~0.725m，竖杆须 ≥0.78m。
    统一 z 拉伸（z_new = bottom + (z-bottom)*k）只拉长、不变截面，方杆顶部平移到 top。
    """
    from pxr import UsdGeom, Vt, Gf
    mesh = stage.GetPrimAtPath(pole_path)
    assert mesh and mesh.IsA(UsdGeom.Mesh), f"{pole_path} 缺失"
    attr = mesh.GetAttribute("points")
    pts = attr.Get()
    zs = [p[2] for p in pts]
    z_min, z_max = min(zs), max(zs)
    assert abs(z_min - bottom) < 1e-3, f"竖杆底 {z_min} 与预期 {bottom} 不符"
    k = (top - bottom) / (z_max - bottom)
    is_double = isinstance(pts[0], Gf.Vec3d)
    new_pts = [Gf.Vec3d(p[0], p[1], bottom + (p[2] - bottom) * k) for p in pts]
    attr.Set(new_pts if is_double else Vt.Vec3fArray(
        [Gf.Vec3f(p[0], p[1], p[2]) for p in new_pts]))
    # 同步 extent：BBoxCache 读 extent 而非 points（stale extent 会让 bound 仍是旧高 0.46）
    xs = [p[0] for p in new_pts]
    ys = [p[1] for p in new_pts]
    zs_new = [p[2] for p in new_pts]
    UsdGeom.Mesh(mesh).CreateExtentAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(min(xs), min(ys), min(zs_new)),
        Gf.Vec3f(max(xs), max(ys), max(zs_new)),
    ]))


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
    extend_pole(stage)   # 竖杆加高到 POLE_TOP（滴定场景须夹住整支滴定管）
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
    # 竖杆加高校验（顶端须到 POLE_TOP、底端贴底座 0.010）
    _bbox = UsdGeom.BBoxCache(Gf.TimeCode(0.0), [UsdGeom.Tokens.default_])
    p_rng = _bbox.ComputeWorldBound(stage.GetPrimAtPath("/root/root/pole/pole_002")).ComputeAlignedRange()
    assert abs(p_rng.GetMin()[2] - 0.010) < 1e-3, f"竖杆底 {p_rng.GetMin()[2]} 偏离 0.010"
    assert abs(p_rng.GetMax()[2] - POLE_TOP) < 1e-3, f"竖杆顶 {p_rng.GetMax()[2]} 未到 {POLE_TOP}"
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
