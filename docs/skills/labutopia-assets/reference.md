# LabUtopia 资产制作参考（reference）

## 资产管线总览

```
gen_*_assets.py (MeshBuilder 构建 + 导出 OBJ)
        ↓
obj2usd.py (OBJ → USD，UsdPreviewSurface 材质)
        ↓
assets/chemistry_lab/<name>.usd
        ↓
gen_labXXX_scene.py (REMOVE/REFS/BUILTIN 组装)
        ↓
assets/chemistry_lab/lab_XXX/lab_XXX.usd
```

## 几何 helpers（MeshBuilder）

以 gen_dissolve_assets.py 为模板，常用 4 个 helper：

```python
def lathe(mb, profile, axis, radius, start, end, steps, S, name, color=None, opacity=None, flip=False):
    """旋转体：profile 为 [x, y] 列表（截面），axis 为旋转轴（'y'/'x'），
    radius 为 [r0, r1] 半径范围，start/end 为轴方向高度范围。"""

def h_cylinder(mb, radius, height, axis, center, S, name, color=None, opacity=None, flip=False):
    """水平圆柱（手柄、立柱等）。"""

def ellipsoid_half(mb, rx, ry, rz, x0, x1, ax, S, name, color=None, opacity=None, flip=False):
    """半椭球（勺头），ax=0 时中心在 x0。"""

def annulus(mb, inner, outer, height, axis, center, S, name, color=None, opacity=None, flip=False):
    """圆环/孔板（试管架孔板：inner 为孔径）。"""
```

材质模板（make_usd）：

```python
def make_usd(mesh, S, name, color=(0.8, 0.8, 0.8), opacity=1.0, metallic=0.0, roughness=0.5):
    # UsdPreviewSurface shader + UsdPreviewSurface 材质
    # glass: opacity=0.35, roughness=0.1
    # steel: metallic=0.85, roughness=0.3, color=(0.75,0.78,0.82)
```

## obj2usd.py 关键函数（修复版 write_mesh）

**必须重映射局部顶点索引**，否则每个 prim 都包含全部顶点，bbox/geometry_center 全错：

```python
def write_mesh(stage, path, verts, faces, normals=None):
    used = sorted(set(idx for f in faces for idx in f))
    remap = {g: l for l, g in enumerate(used)}
    local_verts = verts[used]
    indices = []
    for f in faces:
        indices.extend(remap[i] for i in f)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(local_verts)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr(indices)
    if normals is not None:
        mesh.CreateNormalsAttr(normals[used])
```

## 场景组装脚本骨架（gen_labXXX_scene.py）

```python
SRC = "assets/chemistry_lab/lab_001/lab_001.usd"   # 基础场景
OUT_DIR = "assets/chemistry_lab/lab_004/"
REMOVE = ["/World/xxx_prim", ...]                    # 删除不需要的 prim
REFS = {                                            # 资产引用 + 摆放
    "TestTubeRack": (src, (0.30, 0.08, 0.80)),
    "TestTube":     (src, (0.30, 0.08, 0.809)),     # 管底贴台面
    ...
}
BUILTIN = {                                         # 隐藏标记 prim（圆柱）
    "PowderOnSpoon": ((0.305, 0.13, 0.80), 0.004, 0.008, (0.6,0.45,0.2)),
    "TubeWater":     ((0.30, 0.08, 0.831), 0.006, 0.035, (0.55,0.75,0.95), 0.6),
}

for prim_path in REMOVE:
    stage.RemovePrim(prim_path)
for name, (src, pos) in REFS.items():
    prim = stage.DefinePrim(f"/World/{name}", "Xform")
    prim.GetReferences().AddInternalReference(src)
    ops = prim.GetOrderedXformOps()
    if ops:
        ops[0].Set(Gf.Vec3d(*pos))        # 幂等
    else:
        prim.AddTranslateOp().Set(Gf.Vec3d(*pos))
# BUILTIN 用 UsdGeom.Cylinder.Define + AddTranslateOp + 颜色/透明度
stage.Export(os.path.join(OUT_DIR, "lab_004.usd"))   # 禁止 Save()！
```

## 参考几何参数（D2 已标定）

| 物体 | 参数 |
|---|---|
| 试管 | 外径 r=7.5mm，内径 6.3mm（12.6mm），高 0.120；管底 z=0.809 → 管口 z=0.929 |
| 试管架 | 底座 z0-0.008，立柱 0.008-0.050，孔板 0.050-0.058，孔 r=0.0095 |
| 勺子 | 勺头宽 12mm（ellipsoid ay=0.006），手柄 h_cylinder；grasp 点 = 位置 + [0,0,0.0025] |
| 洗瓶 | body+spout lathe 0→0.135，cap 0.080-0.102；瓶颈 grasp_distance=0.018 |
| 场景工作区 | x[0.2,0.37] y[-0.1,0.2]，桌面 z=0.80 |

## 本地验证要点

```python
# 读 xform 世界坐标（行主序！）
m = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
rot, t = m[:3,:3], m[3,:3]     # translate 在 row 3
# bbox
ext = UsdGeom.Mesh(prim).ComputeExtent(0)  # 或用 GetWorldBoundingBox
```

验证项：bbox 尺寸、geometry_center 与参考点距离、参考点全部在工作区内、勺头 y 范围完全落入管口 y 范围。
