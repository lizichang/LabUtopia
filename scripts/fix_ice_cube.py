# -*- coding: utf-8 -*-
"""修复下载的 ice_cube.usd（Sketchfab FBX 模型）→ 让它在本管线可用（v2：保留原贴图材质）。

问题（2026-08-29，用户重新下载后）：
  1. 尺寸过大：世界包围盒 ~40×45.6×36.7 cm，真实冰块 ~2cm → 整体缩到 2cm。
  2. 贴图路径断链：材质引 `./textures/Material_25_*.jpg`，但贴图实际在 `ice_cube_textures/`
     （下载解压的目录名），渲染会找不到贴图 → 把 file 路径改指 `./ice_cube_textures/...`。
  3. 残留 /root/env_light DomeLight → 压暗场景，删掉。

v1（20:11 那版）曾重建自带冰材质；现在贴图齐全，保留下载原贴图材质更真实。

做法：在 /root/scene 的 scale xform 上统一缩放（几何已底贴 z=0、x/y 居中，绕原点缩放
不会破坏贴底/居中）；改 3 个 UsdUVTexture shader 的 file 路径；删 env_light；bbox + 贴图
路径 + 无 env_light 自检。

用法：python scripts/fix_ice_cube.py   （运行环境：labutopia conda env 有 pxr+numpy）
"""
import os

from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ICE_USD = os.path.join(REPO, "assets", "equipment", "ice_cube.usd")

TARGET_M = 0.015                     # 目标：最大边长 1.5cm（B4 烧杯 Ø69 内径装 6 块，2cm 太挤 → 1.5cm）
SCENE_PATH = "/root/scene"

# 贴图旧前缀 -> 新前缀（文件实际在 ice_cube_textures/）
TEX_OLD = "./textures/"
TEX_NEW = "./ice_cube_textures/"


def _largest_world_dim(stage, prim):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    sz = r.GetMax() - r.GetMin()
    return max(sz[0], sz[1], sz[2])


def fix_scale(stage):
    scene = UsdGeom.Xformable(stage.GetPrimAtPath(SCENE_PATH))
    scale_op = None
    for op in scene.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    assert scale_op is not None, "scale op not found on /root/scene"

    cur = scale_op.Get()
    cur_largest = _largest_world_dim(stage, stage.GetPrimAtPath(SCENE_PATH))
    mult = TARGET_M / cur_largest
    new = Gf.Vec3f(*(cur[i] * mult for i in range(3)))
    scale_op.Set(new)
    print(f"[scale] largest {cur_largest*1000:.1f}mm x{mult:.4f} -> "
          f"scale {tuple(round(v,7) for v in new)} -> ~{TARGET_M*1000:.0f}mm")


def fix_texture_paths(stage):
    n = 0
    for p in Usd.PrimRange.Stage(stage):
        if p.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(p)
        if sh.GetIdAttr().Get() != "UsdUVTexture":
            continue
        f = sh.GetInput("file")
        if not f:
            continue
        asset = f.Get()
        if asset is None:
            continue
        path = str(asset.path) if hasattr(asset, "path") else str(asset)
        if path.startswith(TEX_OLD):
            new_path = TEX_NEW + path[len(TEX_OLD):]
            f.Set(Sdf.AssetPath(new_path))
            print(f"[tex] {p.GetPath()} -> @{new_path}@")
            n += 1
    print(f"[tex] fixed {n} texture path(s)")


def remove_env_light(stage):
    if stage.GetPrimAtPath("/root/env_light").IsValid():
        stage.RemovePrim("/root/env_light")
        print("[clean] removed /root/env_light DomeLight")


def verify(stage):
    mesh = None
    for p in Usd.PrimRange.Stage(stage):
        if p.GetTypeName() == "Mesh":
            mesh = p
            break
    assert mesh is not None, "no Mesh prim found"
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    r = bc.ComputeWorldBound(mesh).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    sz = mx - mn
    mm = tuple(round(v * 1000, 1) for v in sz)
    assert abs(mn[2]) < 1e-4, f"bottom z {mn[2]} != 0 (should rest flat on surface)"
    assert 12.0 < max(sz) * 1000 < 30.0, f"largest {max(sz)*1000:.1f}mm out of range"
    assert abs((mn[0] + mx[0]) / 2) < 1e-3, "x not centered"
    assert abs((mn[1] + mx[1]) / 2) < 1e-3, "y not centered"
    # 贴图路径全部指向 ice_cube_textures/ 且文件存在
    tex_paths = []
    for p in Usd.PrimRange.Stage(stage):
        if p.GetTypeName() != "Shader":
            continue
        sh = UsdShade.Shader(p)
        if sh.GetIdAttr().Get() != "UsdUVTexture":
            continue
        f = sh.GetInput("file")
        if f and f.Get():
            tex_paths.append(str(f.Get().path))
    assert tex_paths, "no texture file refs found"
    for tp in tex_paths:
        assert TEX_NEW in tp, f"texture path not repointed: {tp}"
        rel = os.path.join(REPO, "assets", "equipment", tp)
        assert os.path.exists(rel), f"texture file missing: {rel}"
    assert not stage.GetPrimAtPath("/root/env_light").IsValid(), "env_light still present"
    print(f"[verify] OK: bottom z=0 / centered / size {mm[0]}x{mm[1]}x{mm[2]}mm "
          f"/ {len(tex_paths)} textures -> ice_cube_textures/ / no env_light")


def main():
    stage = Usd.Stage.Open(ICE_USD)
    fix_scale(stage)
    fix_texture_paths(stage)
    remove_env_light(stage)
    stage.GetRootLayer().Save()
    print("SAVED", ICE_USD)

    st2 = Usd.Stage.Open(ICE_USD)
    verify(st2)


if __name__ == "__main__":
    main()
