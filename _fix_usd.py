"""Fix lab_flametest.usd on the server.

Fixes:
  1. Hide lounge_booth_table (invalid shader -> red background; unrelated asset)
  2. Hide SampleDish/powder (mysterious white circle on dish)
  3. Recenter stopper geometry at prim origin + add translate op
     (geometry was offset 7.35cm up, causing doubling on reset)
  4. Shrink flame_outer (4cm->2.4cm wide, 7.6cm->5cm tall)
  5. Shrink flame_inner proportionally
  6. Ensure cap/translate op exists
  7. Print verification bboxes
"""
import shutil
import sys
from pxr import Usd, UsdGeom, Gf
import numpy as np

SCENE = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
BACKUP = SCENE + ".bak"

# backup
shutil.copy2(SCENE, BACKUP)
print(f"Backup -> {BACKUP}")

stage = Usd.Stage.Open(SCENE)
assert stage, "failed to open stage"

def set_invisible(path):
    p = stage.GetPrimAtPath(path)
    if p.IsValid() and p.IsA(UsdGeom.Imageable):
        UsdGeom.Imageable(p).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        print(f"  hidden: {path}")
    else:
        print(f"  WARN invalid: {path}")

# ---------- 1. hide lounge_booth_table ----------
print("\n[1] Hiding lounge_booth_table ...")
set_invisible("/World/lounge_booth_table")

# ---------- 2. hide SampleDish/powder ----------
print("\n[2] Hiding SampleDish/powder ...")
set_invisible("/World/SampleDish/powder")

# ---------- 3. recenter stoppers ----------
print("\n[3] Recentering stoppers ...")

def recenter_child_mesh(parent_path, child_name):
    """Move child mesh points so their bbox center is at local origin,
    then add a translate op equal to the original center (world pos unchanged)."""
    child_path = f"{parent_path}/{child_name}"
    prim = stage.GetPrimAtPath(child_path)
    if not prim.IsValid():
        print(f"  WARN {child_path} invalid"); return
    mesh = UsdGeom.Mesh(prim)
    pts_attr = mesh.GetPointsAttr()
    pts = pts_attr.Get()
    if not pts:
        print(f"  WARN {child_path} no points"); return
    arr = np.array([(p[0],p[1],p[2]) for p in pts], dtype=float)
    mn = arr.min(axis=0); mx = arr.max(axis=0)
    center = (mn+mx)/2.0
    print(f"  {child_name}: local center=({center[0]:.5f},{center[1]:.5f},{center[2]:.5f}) "
          f"size=({mx[0]-mn[0]:.4f},{mx[1]-mn[1]:.4f},{mx[2]-mn[2]:.4f})")
    # shift points to origin
    new_pts = arr - center
    pts_attr.Set([Gf.Vec3f(float(r[0]),float(r[1]),float(r[2])) for r in new_pts])
    # add / set translate op = original center
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    t_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t_op = op; break
    if t_op is None:
        # insert translate op at the beginning (before any rotate)
        t_op = xf.AddTranslateOp(opSuffix="recenter")
        # move it to front
        xf.SetXformOpOrder([t_op] + [o for o in ops])
    t_op.Set(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    print(f"    -> points recentered, translate=({center[0]:.5f},{center[1]:.5f},{center[2]:.5f})")

recenter_child_mesh("/World/HClBottle", "stopper")
recenter_child_mesh("/World/SampleBottle", "stopper")

# ---------- 4 & 5. shrink flame meshes ----------
print("\n[4-5] Shrinking flame meshes ...")

def scale_mesh_around_base(path, sx, sy, sz):
    """Scale mesh points around base center (x,y center, z min)."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"  WARN {path} invalid"); return
    mesh = UsdGeom.Mesh(prim)
    pts_attr = mesh.GetPointsAttr()
    pts = pts_attr.Get()
    if not pts:
        print(f"  WARN {path} no points"); return
    arr = np.array([(p[0],p[1],p[2]) for p in pts], dtype=float)
    mn = arr.min(axis=0); mx = arr.max(axis=0)
    cx = (mn[0]+mx[0])/2.0
    cy = (mn[1]+mx[1])/2.0
    z_base = mn[2]
    print(f"  {path}: local center=({cx:.4f},{cy:.4f}) z_base={z_base:.4f} "
          f"size=({mx[0]-mn[0]:.4f},{mx[1]-mn[1]:.4f},{mx[2]-mn[2]:.4f})")
    new = arr.copy()
    new[:,0] = cx + (arr[:,0]-cx)*sx
    new[:,1] = cy + (arr[:,1]-cy)*sy
    new[:,2] = z_base + (arr[:,2]-z_base)*sz
    pts_attr.Set([Gf.Vec3f(float(r[0]),float(r[1]),float(r[2])) for r in new])
    mn2 = new.min(axis=0); mx2 = new.max(axis=0)
    print(f"    -> new size=({mx2[0]-mn2[0]:.4f},{mx2[1]-mn2[1]:.4f},{mx2[2]-mn2[2]:.4f})")

# flame_outer: 4cm -> 2.4cm (0.6), 7.6cm -> 5cm (0.66)
scale_mesh_around_base("/World/BunsenBurner/flame_outer", 0.60, 0.60, 0.66)
# flame_inner: 1.6cm -> 1.2cm (0.75), 3cm -> 2.4cm (0.8)
scale_mesh_around_base("/World/BunsenBurner/flame_inner", 0.75, 0.75, 0.80)

# ---------- 6. ensure cap has translate op (it does, just verify) ----------
print("\n[6] Checking cap translate op ...")
cap = stage.GetPrimAtPath("/World/BunsenBurner/cap")
if cap.IsValid():
    xf = UsdGeom.Xformable(cap)
    has_t = any(o.GetOpType()==UsdGeom.XformOp.TypeTranslate for o in xf.GetOrderedXformOps())
    print(f"  cap has translate op: {has_t}")

# ---------- save ----------
stage.GetRootLayer().Save()
print("\nSaved.")

# ---------- 7. verification ----------
print("\n[7] Verification (world bboxes) ...")
stage2 = Usd.Stage.Open(SCENE)
bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
def wb(path):
    p = stage2.GetPrimAtPath(path)
    if not p.IsValid(): return "INVALID"
    b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
    return f"min=({b.GetMin()[0]:.4f},{b.GetMin()[1]:.4f},{b.GetMin()[2]:.4f}) max=({b.GetMax()[0]:.4f},{b.GetMax()[1]:.4f},{b.GetMax()[2]:.4f})"
for n in ["/World/BunsenBurner/flame_outer","/World/BunsenBurner/flame_inner",
          "/World/HClBottle/stopper","/World/SampleBottle/stopper",
          "/World/lounge_booth_table","/World/SampleDish/powder","/World/BunsenBurner/cap"]:
    vis = "?"
    p = stage2.GetPrimAtPath(n)
    if p.IsValid() and p.IsA(UsdGeom.Imageable):
        vis = UsdGeom.Imageable(p).GetVisibilityAttr().Get()
    print(f"  {n}: vis={vis}  {wb(n)}")
