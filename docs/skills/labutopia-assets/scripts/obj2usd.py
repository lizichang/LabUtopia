# -*- coding: utf-8 -*-
"""OBJ -> USD 转换：分组转独立 prims（法线保留，米单位，defaultPrim=root）。

用法: python obj2usd.py <in.obj> <out.usd>
"""
import sys
import numpy as np
from pxr import Usd, UsdGeom, Gf


def parse_obj(path):
    """解析 OBJ，返回 {group: (verts, faces)}，verts/faces 为全局索引。"""
    verts, vns = [], []
    groups = {}
    cur = None
    with open(path) as f:
        for line in f:
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                verts.append([float(x) for x in t[1:4]])
            elif t[0] == "vn":
                vns.append([float(x) for x in t[1:4]])
            elif t[0] == "g":
                cur = t[1]
                groups.setdefault(cur, [])
            elif t[0] == "f" and cur is not None:
                idx = [int(i.split("/")[0]) - 1 for i in t[1:]]
                groups[cur].append(idx)
    return np.array(verts), np.array(vns), groups


def write_mesh(stage, prim_path, verts, faces, normals):
    """把三角/四边面网格写入 UsdGeom.Mesh。

    verts/faces/normals 为全局索引；此处只把该 prim 实际用到的顶点写入
    （重映射为局部索引），避免每个 prim 携带全部顶点导致 bbox 失真。
    """
    used = sorted(set(idx for f in faces for idx in f))
    remap = {g: l for l, g in enumerate(used)}
    local_verts = verts[used]
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(local_verts)
    # 展平 faceVertexIndices；faceVertexCounts（重映射后的局部索引）
    counts = []
    indices = []
    for f in faces:
        counts.append(len(f))
        indices.extend(remap[i] for i in f)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    # 法线：顶点法线（每顶点一个），与局部顶点一一对应
    mesh.CreateNormalsAttr(normals[used])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    # 细分关闭 + 双面渲染兜底
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    return mesh


def main():
    in_obj, out_usd = sys.argv[1], sys.argv[2]
    verts, vns, groups = parse_obj(in_obj)
    stage = Usd.Stage.CreateNew(out_usd)
    # 必须显式设为米单位！Usd.Stage.CreateNew 默认 metersPerUnit=0.01（厘米），
    # 不设会导致 Blender 导入时 root 被缩放到 1%（scale=0.01），物体肉眼不可见。
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        write_mesh(stage, f"/root/{g}", verts, faces, vns)
    stage.GetRootLayer().Save()
    print(f"{out_usd}: {len(groups)} prims, {len(verts)} verts, {sum(len(f) for f in groups.values())} faces")


if __name__ == "__main__":
    main()
