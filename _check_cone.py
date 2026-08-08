"""Check Cone attributes for flame stain prims."""
from pxr import Usd, UsdGeom

BASE = "/media/dky/Disk2TB/lizichang/LabUtopia"
SCENE = BASE + "/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
stage = Usd.Stage.Open(SCENE)

for color in ["yellow", "purple", "green", "red", "orange", "blue"]:
    path = f"/World/BunsenBurner/flame_stain_{color}"
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        continue
    cone = UsdGeom.Cone(p)
    r_attr = cone.GetRadiusAttr()
    h_attr = cone.GetHeightAttr()
    axis_attr = cone.GetAxisAttr()
    r = r_attr.Get() if r_attr.HasValue() else "?"
    h = h_attr.Get() if h_attr.HasValue() else "?"
    axis = axis_attr.Get() if axis_attr.HasValue() else "?"
    xf = UsdGeom.Xformable(p)
    ops = [(o.GetBaseName(), o.GetOpType(), o.Get()) for o in xf.GetOrderedXformOps()]
    print(f"{color}: radius={r} height={h} axis={axis}")
    for name, otype, val in ops:
        print(f"  {name} ({otype}) = {val}")

# Also check flame_outer mesh to understand flame shape
print("\nflame_outer mesh points sample:")
p = stage.GetPrimAtPath("/World/BunsenBurner/flame_outer")
mesh = UsdGeom.Mesh(p)
pts = mesh.GetPointsAttr().Get()
if pts:
    import numpy as np
    arr = np.array([(pt[0],pt[1],pt[2]) for pt in pts])
    print(f"  {len(pts)} points")
    print(f"  z range: {arr[:,2].min():.4f} - {arr[:,2].max():.4f}")
    print(f"  xy range at z_min: x=[{arr[arr[:,2]==arr[:,2].min()][:,0].min():.4f},{arr[arr[:,2]==arr[:,2].min()][:,0].max():.4f}]")
    print(f"  xy range at z_max: x=[{arr[arr[:,2]==arr[:,2].max()][:,0].min():.4f},{arr[arr[:,2]==arr[:,2].max()][:,0].max():.4f}]")
