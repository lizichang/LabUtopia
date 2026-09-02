# -*- coding: utf-8 -*-
"""生成 木条（带火星木条 glowing splint）资产，单位：米。

实物调研（lab_inventory.json「带火星木条」无尺寸 → 上网查规格）：
  glowing splint 标准约 150mm 长 × 6mm 宽 × 2mm 厚（Wikipedia/wikiHow/Bartleby：
  6 inch × ¼ inch；商用 Benzmicroscope 140×4×1mm）。本资产取 150mm 长、Ø6mm 圆柱
  （仿真里 2mm 厚截面渲染不可辨，用圆柱近似细木条；比火柴 Ø3mm 粗一倍）。
  点燃端在 +X（x=0.15，像火柴 head 在 +X）。

结构：
  stick  木条主体：Ø6mm 圆柱（沿 +X，握持端原点）x∈[0,0.12] + 点燃端变细段 Ø4.4mm
         x∈[0.12,0.15]（烧后炭化变细——若不收细，炭黑区 r2.2mm 会被 Ø6mm 木条整体遮挡，
         看不到「燃烧变细」现象；收细后 /World/SplintChar r2.4mm 包住变细端、可见且比主体细）

火焰/余烬**不**烘焙进资产（RTX 铁律：火焰/余烬必须 /World 顶层 prim，
由 gen 场景建 SplintFlame/SplintEmber 顶层 effect prim，task 每帧钉到木条端）。
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "wood_splint_obj")

SPLINT_LEN = 0.150   # 木条长 150mm
SPLINT_R = 0.003     # 木条主体半径 3mm（Ø6mm）
TIP_LEN = 0.030      # 点燃端变细区长度 30mm（烧后炭化变细）
TIP_R = 0.0022       # 点燃端变细半径 2.2mm（比主体 Ø6mm 细，供炭黑区覆盖可见）


def build_wood_splint(mb):
    S = 24
    # 主体（握持端/原点 → 变细区起点 0.12）：Ø6mm 圆柱，轴沿 X，cap 平端
    mb.h_cylinder((0, 0, 0), (SPLINT_LEN - TIP_LEN, 0, 0), SPLINT_R, S, "stick", cap=True)
    # 点燃端变细段（0.12 → 0.15）：Ø4.4mm 圆柱，贴主体端形成台阶（烧后炭化变细）
    mb.h_cylinder((SPLINT_LEN - TIP_LEN, 0, 0), (SPLINT_LEN, 0, 0), TIP_R, S, "stick", cap=True)


USD_MATS = {
    "stick": dict(diffuse=(0.62, 0.47, 0.30), roughness=0.85),  # 木质浅棕，粗糙
}

GROUPS = ["stick"]


def make_usd(obj_path, out_usd, mat_specs):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/{g}", verts, faces, vns)
        spec = mat_specs.get(g)
        if spec is None:
            continue
        mat_path = f"/root/{g}_mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*spec["diffuse"]))
        if spec.get("opacity", 1.0) < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
        if spec.get("emissive") is not None:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*spec["emissive"]))
        if spec.get("ior") is not None:
            sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(spec["ior"])
        if spec.get("metallic", 0.0) > 0:
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd):
    from pxr import Usd, UsdGeom
    import numpy as np
    stage = Usd.Stage.Open(out_usd)
    lo, hi, bad_n = None, None, 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get(0.0)
        if not pts:
            continue
        P = np.array([[v[0], v[1], v[2]] for v in pts])
        mn, mx = P.min(0), P.max(0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
        nrm = m.GetNormalsAttr().Get(0.0)
        if nrm is not None:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            lens = np.linalg.norm(N, axis=1)
            bad_n += int(np.sum((lens > 1.0001) | (lens < 0.9999)))
    sz = (hi - lo) * 1000
    print(f"verify: bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2]:.4f} "
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)} "
          f"bad_normals={bad_n}")


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    build_wood_splint(mb)
    obj_path = os.path.join(OUT_OBJ, "wood_splint.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(GROUPS))
    out_usd = os.path.join(OUT_USD, "wood_splint.usd")
    make_usd(obj_path, out_usd, USD_MATS)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
