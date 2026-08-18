# -*- coding: utf-8 -*-
"""从 lab_001 / lab_003 底场景提取可复用器材，产出 assets/equipment/<名>.usd。

每个器材一个 USD：世界变换 bake 进 mesh points（米单位、Z-up、origin=底心），
材质统一用 UsdPreviewSurface 玻璃配方（与 dropper.usd 一致），subdivisionScheme=none。
源场景只读（stage.Open），绝不 Save；新资产用 stage.Export 输出。

运行：python scripts/extract_base_equipment.py
"""
import os
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

SRC_BASE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "assets", "scenes", "base")
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "assets", "equipment")

# (scene, 源 prim, 输出名) —— 输出名含实测包围盒规格 X×Y×Z(mm)
SOURCES = [
    ("lab_001", "conical_bottle01", "conical_flask_77x77x97"),
    ("lab_001", "conical_bottle02", "conical_flask_93x93x165"),
    ("lab_003", "conical_bottle04", "conical_flask_113x113x197"),
    ("lab_001", "conical_bottle04", "conical_flask_141x141x234"),
    ("lab_001", "conical_bottle03", "conical_flask_193x113x193"),
    ("lab_003", "beaker_1",        "beaker_52x68x58"),
    ("lab_003", "beaker_2",        "beaker_65x83x72"),
    ("lab_003", "beaker_3",        "beaker_75x98x83"),
    ("lab_003", "beaker_4",        "beaker_84x113x94"),
    ("lab_003", "beaker_5",        "beaker_110x128x123"),
    ("lab_001", "beaker2",         "beaker_111x75x116"),
    ("lab_001", "beaker1",         "beaker_156x105x162"),
    ("lab_003", "glass_rod",       "glass_rod_6x6x261"),
]

# 玻璃配方（dropper.usd glass_001 同款，已在仿真验证可见）
GLASS = dict(diffuse=(0.85, 0.92, 0.98), ior=1.45, roughness=0.05,
             metallic=0.0, opacity=1.0, specular=0.5)


def gather_meshes(stage, prim_path):
    """收集 prim 下所有 Mesh：本地 points/normals/faces + local->world 矩阵。"""
    out = []
    prim = stage.GetPrimAtPath(prim_path)
    for c in Usd.PrimRange(prim):
        if not c.IsA(UsdGeom.Mesh):
            continue
        pts = c.GetAttribute("points").Get()
        if pts is None or len(pts) == 0:
            continue
        fvc = c.GetAttribute("faceVertexCounts").Get()
        fvi = c.GetAttribute("faceVertexIndices").Get()
        xf = UsdGeom.Xformable(c).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        M = np.array([[xf[i][j] for j in range(4)] for i in range(4)])
        P = np.array([[p[0], p[1], p[2], 1.0] for p in pts])
        world = (P @ M.T)[:, :3]
        # 法线：优先用源法线，逆转置变换到世界系；否则按几何重算
        nrm_pv = UsdGeom.PrimvarsAPI(c).GetPrimvar("normals")
        nrm = nrm_pv.Get() if nrm_pv else None
        interp = nrm_pv.GetInterpolation() if nrm_pv else None
        if nrm is not None and len(nrm) > 0:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            Nw = normalize_normals(N, M)
        else:
            Nw = compute_smooth_normals(world, np.asarray(fvc), np.asarray(fvi))
            interp = "faceVarying"
        out.append(dict(points=world, normals=Nw, fvc=np.asarray(fvc),
                        fvi=np.asarray(fvi), interp=interp))
    return out


def normalize_normals(N, M):
    """法线世界化：逆转置变换（处理旋转+非均匀缩放）。"""
    R = np.linalg.inv(M[:3, :3]).T
    Nw = N @ R.T
    norms = np.linalg.norm(Nw, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return Nw / norms


def compute_smooth_normals(P, fvc, fvi):
    """角度阈值平滑法线（faceVarying）：共面(<60°)融合，否则硬边。"""
    P = np.asarray(P)
    n_vert = len(P)
    # 面片三角化（fan）
    tri_faces = []
    idx = 0
    for cnt in fvc:
        loop = fvi[idx:idx + cnt]
        for k in range(1, cnt - 1):
            tri_faces.append([loop[0], loop[k], loop[k + 1]])
        idx += cnt
    tri_faces = np.asarray(tri_faces)
    # 面法线（面积加权向量）；退化面（面积≈0）法线置零，后续一律跳过
    v0 = P[tri_faces[:, 0]]
    v1 = P[tri_faces[:, 1]]
    v2 = P[tri_faces[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    fn_len = np.linalg.norm(fn, axis=1, keepdims=True)
    deg = (fn_len[:, 0] < 1e-9)
    fn_len[deg] = 1.0
    fn = fn / fn_len
    fn[deg] = 0.0
    # 每个顶点邻接面（仅保留非退化面）
    adj = [[] for _ in range(n_vert)]
    for t, (a, b, c) in enumerate(tri_faces):
        if fn[t].any():  # 跳过退化面（法线为零）
            adj[a].append(t)
            adj[b].append(t)
            adj[c].append(t)
    # faceVarying：每个 face-vert 一个法线
    normals = []
    idx = 0
    for cnt in fvc:
        loop = fvi[idx:idx + cnt]
        idx += cnt
        for k, vid in enumerate(loop):
            a = int(loop[0]); b = int(loop[(k + 1) % cnt]); c = int(loop[(k + 2) % cnt])
            anchor = None
            for t in adj[vid]:
                if set(tri_faces[t]) == {a, b, c} or set(tri_faces[t]) == {a, c, b}:
                    anchor = fn[t]
                    break
            if anchor is None:
                anchor = fn[adj[vid][0]] if adj[vid] else np.zeros(3)
            acc = anchor.copy()
            for t in adj[vid]:
                if np.dot(anchor, fn[t]) >= np.cos(np.radians(60)):
                    acc = acc + fn[t]
            n = np.linalg.norm(acc)
            normals.append(acc / n if n > 0 else anchor)
    return np.asarray(normals)


def make_usd(out_path, base_name, meshes):
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    # 材质
    mat = UsdShade.Material.Define(stage, f"/root/_materials/{base_name}_mat")
    sh = UsdShade.Shader.Define(stage, f"/root/_materials/{base_name}_mat/Principled_BSDF")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GLASS["diffuse"]))
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(GLASS["ior"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(GLASS["roughness"])
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(GLASS["metallic"])
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(GLASS["opacity"])
    sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(GLASS["specular"])
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

    for i, m in enumerate(meshes):
        P = m["points"]
        # rebase：底 z=0，X/Y 居中
        mn, mx = P.min(0), P.max(0)
        P = P - np.array([(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, mn[2]])
        prim_path = f"/root/{base_name}" if len(meshes) == 1 else f"/root/{base_name}_{i}"
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in P])
        mesh.CreateFaceVertexCountsAttr([int(v) for v in m["fvc"]])
        mesh.CreateFaceVertexIndicesAttr([int(v) for v in m["fvi"]])
        mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in m["normals"]])
        mesh.SetNormalsInterpolation(m["interp"])
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.Export(out_path)
    return stage


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for scene, src, out in SOURCES:
        src_file = os.path.join(SRC_BASE, scene, f"{scene}.usd")
        stage = Usd.Stage.Open(src_file)
        meshes = gather_meshes(stage, f"/World/{src}")
        if not meshes:
            print(f"SKIP {out}: no mesh under /World/{src}")
            continue
        out_path = os.path.join(OUT_DIR, f"{out}.usd")
        st = make_usd(out_path, out, meshes)
        # 验证：bbox
        lo, hi = None, None
        for p in Usd.PrimRange(st.GetPseudoRoot()):
            if p.IsA(UsdGeom.Mesh):
                pts = np.array([[v[0], v[1], v[2]] for v in p.GetAttribute("points").Get()])
                mn, mx = pts.min(0), pts.max(0)
                lo = mn if lo is None else np.minimum(lo, mn)
                hi = mx if hi is None else np.maximum(hi, mx)
        sz = (hi - lo) * 1000
        print(f"OK {out:22s} bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2]:.3f} up={UsdGeom.GetStageUpAxis(st)}")
    print("DONE")


if __name__ == "__main__":
    main()
