# -*- coding: utf-8 -*-
"""生成称量纸资产（weighing_paper.usd）：实验室方形称量纸（薄纸片）。

尺寸（真实 lab 称量纸 10~15cm 见方，估 12cm×12cm，厚 1mm）：
  12cm × 12cm × 1mm，中心在原点（min z = −0.0005，底座贴地 z=0）。
  材质：白色半哑光纸（diffuse 近白、roughness 0.35），不透明。

生成方式：MeshBuilder 手拼一个 8 顶点 6 面的薄盒 → to_obj → obj2usd.write_mesh
（自动 subdivisionScheme=none + doubleSided）。

场景里放分析天平称盘（盘顶 z=0.0475）上，托盘中心贴盘顶。
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "weighing_paper_obj")

PAPER_H = 0.06    # 半宽 60mm → 120mm 见方
THK = 0.0005      # 半厚 0.5mm → 1mm


def build_paper(mb):
    """12cm 见方薄盒：顶点 0..3 底、4..7 顶（同侧顺序）。"""
    h, t = PAPER_H, THK
    v = [
        (-h, -h, -t), (h, -h, -t), (h, h, -t), (-h, h, -t),   # 底环
        (-h, -h, t), (h, -h, t), (h, h, t), (-h, h, t),       # 顶环
    ]
    face_n = [  # (face, normal)
        ((4, 5, 6, 7), (0, 0, 1)),    # 顶
        ((1, 0, 3, 2), (0, 0, -1)),   # 底
        ((3, 7, 6, 2), (0, 1, 0)),    # +y
        ((0, 1, 5, 4), (0, -1, 0)),   # -y
        ((1, 2, 6, 5), (1, 0, 0)),    # +x
        ((0, 4, 7, 3), (-1, 0, 0)),   # -x
    ]
    idx = []
    for i, p in enumerate(v):
        idx.append(mb._add_vert(p, (0.0, 0.0, 1.0)))   # 先占位，下面重写法线
    # 每顶点法线 = 相邻面法线平均（盒角正确光照）
    for i, p in enumerate(v):
        acc = [0.0, 0.0, 0.0]
        for face, n in face_n:
            if i in face:
                for k in range(3):
                    acc[k] += n[k]
        ln = sum(c * c for c in acc) ** 0.5
        mb.norms[idx[i]] = tuple(c / ln for c in acc)
    for face, _ in face_n:
        a, b, c, d = face
        mb._add_quad(idx[a], idx[b], idx[c], idx[d], "paper")


def make_usd(obj_path, out_usd):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = write_mesh(stage, "/root/weighing_paper", verts, groups["paper"], vns)
    mat_path = "/root/weighing_paper_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.96, 0.96, 0.98))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: OK")


def verify(out_usd):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    ok = (abs(lo[0] + PAPER_H) < 1e-4 and abs(hi[0] - PAPER_H) < 1e-4
          and abs(lo[1] + PAPER_H) < 1e-4 and abs(hi[1] - PAPER_H) < 1e-4
          and abs(lo[2] + THK) < 1e-5 and abs(hi[2] - THK) < 1e-5)
    print(f"[verify] weighing_paper bbox {[round(c,4) for c in lo]}..{[round(c,4) for c in hi]} "
          f"→ {round(hi[0]-lo[0],3)}x{round(hi[1]-lo[1],3)}x{round(hi[2]-lo[2],4)}m: {'OK' if ok else 'FAIL'}")
    assert ok, "weighing_paper verify FAIL"


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    build_paper(mb)
    obj_path = os.path.join(OUT_OBJ, "weighing_paper.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(["paper"]))
    out_usd = os.path.join(OUT_USD, "weighing_paper.usd")
    make_usd(obj_path, out_usd)
    verify(out_usd)
    print("DONE")


if __name__ == "__main__":
    main()
