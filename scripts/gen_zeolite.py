# -*- coding: utf-8 -*-
"""生成沸石（boiling chip / 防暴沸颗粒）资产 zeolite.usd，白色多孔不规则颗粒，~10mm。

真实实验室沸石（2026-08-27 调研）：白色、不规则碎块、多孔粗糙表面（多孔空腔截留
空气形成成核位点防暴沸），常见材质氧化铝/瓷/碳化硅，粒径几毫米。旧 zeolite.usd 是
25 点规则方块（米白方块），形状与"多孔不规则"都不真 → 重建成不规则扁平颗粒。

几何：icosphere（正二十面体细分 2 次 = 320 三角面）顶点沿径向 ±20% 扰动 → 不规则
碎石棱角；z 方向乘 0.72 偏扁（真实沸石碎片扁平）。法线 = 顶点径向方向（对凸颗粒
近似够用，roughness 高看不出误差）。底 z=0（场景里贴表面皿顶）。

结构（defaultPrim=/World，米单位，Z-up，与旧资产一致 → 场景引用/烘平路径不变）：
  /World/Zeolite  —— 不规则颗粒 Mesh（~9.6mm 直径 × ~6.9mm 高，扰动 ±20%）
  /World/Looks/Ceramic —— 白色粗糙材质（diffuse 0.95 暖白 / roughness 0.85 哑光多孔感）

尺寸约束（2026-08-27 用户定 10mm，须满足）：能掉进试管口（内径 16.1mm）且沉到管底
（内径 11.5mm）→ 基础直径 9.6mm、扰动 ±20% 后最大 ~11.5mm，贴底不卡壁。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402
from obj2usd import write_mesh  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment", "zeolite.usd")

# —— 尺寸（米）——
R_XY = 0.0048       # x/y 基础半径（直径 ~9.6mm，扰动后 7.7~11.5mm，贴管底内径 11.5mm）
Z_FLAT = 0.72       # z 方向偏扁（真实沸石碎片扁平；z 高 ≈ 9.6*0.72 = 6.9mm）
PERTURB = 0.20      # 顶点径向扰动幅度（±20%，不规则碎石棱角）
SUBDIV = 2          # icosphere 细分次数（20 → 80 → 320 面）
SEED = 20260827     # 固定 seed（确定性：每次生成同一块沸石）


def _normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n


def icosphere(subdiv):
    """正二十面体 + 细分 subdiv 次，返回 (单位球面顶点列表, 三角面索引列表)。

    标准 icosphere：20 面三角无退化（不像经纬球两极顶点重合），细分后顶点仍归一化
    在单位球面上；法线 = 顶点方向即球面精确法线。
    """
    t = (1.0 + 5.0 ** 0.5) / 2.0
    verts = [
        _normalize((-1, t, 0)), _normalize((1, t, 0)),
        _normalize((-1, -t, 0)), _normalize((1, -t, 0)),
        _normalize((0, -1, t)), _normalize((0, 1, t)),
        _normalize((0, -1, -t)), _normalize((0, 1, -t)),
        _normalize((t, 0, -1)), _normalize((t, 0, 1)),
        _normalize((-t, 0, -1)), _normalize((-t, 0, 1)),
    ]
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    for _ in range(subdiv):
        mid_cache = {}

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in mid_cache:
                m = _normalize((verts[a] + verts[b]) / 2.0)
                mid_cache[key] = len(verts)
                verts.append(m)
            return mid_cache[key]

        new_faces = []
        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return verts, faces


def build_zeolite(mb):
    """生成不规则扁平颗粒，底 z=0。"""
    rng = np.random.RandomState(SEED)
    verts, faces = icosphere(SUBDIV)
    idx = [0] * len(verts)
    for i, d in enumerate(verts):
        r = 1.0 + PERTURB * rng.uniform(-1.0, 1.0)   # 径向扰动（不规则棱角）
        p = np.asarray(d, dtype=float) * r * np.array([R_XY, R_XY, R_XY * Z_FLAT])
        idx[i] = mb._add_vert(p, d)                  # 法线 = 径向方向
    for a, b, c in faces:
        mb._add_tri(idx[a], idx[b], idx[c], "zeolite")
    # 底 z 归零（最低点落到 z=0，场景里直接贴表面皿顶）
    zmin = min(v[2] for v in mb.verts)
    for i in range(len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x, y, z - zmin)


def add_material(stage, mesh):
    """白色粗糙材质（暖白哑光多孔感）。"""
    from pxr import UsdShade, Sdf, Gf
    mat_path = "/World/Looks/Ceramic"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.95, 0.95, 0.92))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)


def main():
    from pxr import Usd, UsdGeom

    mb = MeshBuilder()
    build_zeolite(mb)

    stage = Usd.Stage.CreateNew(OUT_USD)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    verts = np.array(mb.verts, dtype=float)
    norms = np.array(mb.norms, dtype=float)
    faces = [list(f) for f in mb.faces]
    mesh = write_mesh(stage, "/World/Zeolite", verts, faces, norms)
    add_material(stage, mesh)
    stage.GetRootLayer().Save()
    print(f"[zeolite] {len(verts)} verts, {len(faces)} faces -> {os.path.basename(OUT_USD)}")

    # —— 验证：bbox 尺寸范围 + 底 z=0 + 法线单位向量 ——
    stage2 = Usd.Stage.Open(OUT_USD)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(stage2.GetPrimAtPath("/World/Zeolite")).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    sz = (mx - mn) * 1000
    dx, dy, dz = sz[0], sz[1], sz[2]
    bad_n = 0
    m = UsdGeom.Mesh(stage2.GetPrimAtPath("/World/Zeolite"))
    nrm = m.GetNormalsAttr().Get()
    if nrm is not None:
        bad_n = int(sum(abs(np.linalg.norm(np.asarray(n)) - 1.0) > 1e-4 for n in nrm))
    ok = (abs(mn[2]) < 1e-4                              # 底贴 z=0
          and 5.0 < dx < 15.0 and 5.0 < dy < 15.0        # 直径 ~9.6mm（扰动后）
          and 3.0 < dz < 12.0                            # 高 ~6.9mm（扰动后）
          and abs((mn[0] + mx[0]) / 2) < 3e-3           # 中心 x≈0（对称，扰动留裕量）
          and abs((mn[1] + mx[1]) / 2) < 3e-3           # 中心 y≈0
          and bad_n == 0)
    print(f"[verify] bbox min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
          f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f}) "
          f"size({dx:.1f}x{dy:.1f}x{dz:.1f})mm bad_normals={bad_n} -> {'OK' if ok else 'FAIL'}")
    assert ok, "zeolite verify FAIL"
    print("SAVED", OUT_USD)


if __name__ == "__main__":
    main()
