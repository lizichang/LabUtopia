
import sys
sys.path.insert(0, "/media/dky/Disk2TB/lizichang/LabUtopia")
from pxr import Usd, UsdGeom, UsdShade
import numpy as np

SCENE = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
stage = Usd.Stage.Open(SCENE)
print("=== STAGE OPENED:", SCENE)

def world_bbox(prim):
    bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    b = bcache.ComputeWorldBound(prim)
    r = b.ComputeAlignedRange()
    return r.GetMin(), r.GetMax()

# 1. Top-level prims under /World
print("\n=== /World CHILDREN ===")
world = stage.GetPrimAtPath("/World")
for c in world.GetChildren():
    print(f"  {c.GetPath()} [{c.GetTypeName()}]")

# 2. Flame bounds
print("\n=== FLAME / BURNER BOUNDS ===")
for name in ("flame_outer","flame_inner","flame_stain_yellow","flame_stain_purple",
             "flame_stain_green","flame_stain_red","flame_stain_orange","flame_stain_blue",
             "tube","collar","base","cap","burner_tube"):
    p = stage.GetPrimAtPath(f"/World/BunsenBurner/{name}")
    if p.IsValid():
        mn,mx = world_bbox(p)
        sz = mx-mn
        print(f"  {name}: min=({mn[0]:.4f},{mn[1]:.4f},{mn[2]:.4f}) max=({mx[0]:.4f},{mx[1]:.4f},{mx[2]:.4f}) size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f})")
        # check geometry type
        for attr in ["radius","height","size"]:
            a = p.GetAttribute(f"xformOp:transform")
        # if mesh, list points count
        if p.GetTypeName() == "Mesh":
            pts = p.GetAttribute("points").Get()
            print(f"    Mesh points: {len(pts) if pts else 0}")
        elif p.GetTypeName() == "Cone":
            r = p.GetAttribute("radius").Get()
            h = p.GetAttribute("height").Get()
            print(f"    Cone r={r} h={h}")
        elif p.GetTypeName() == "Cylinder":
            r = p.GetAttribute("radius").Get()
            h = p.GetAttribute("height").Get()
            print(f"    Cylinder r={r} h={h}")

# 3. Stopper xform ops
print("\n=== STOPPER / KIN XFORM OPS ===")
for path in ["/World/HClBottle/stopper","/World/SampleBottle/stopper",
             "/World/BunsenBurner/cap","/World/SampleDish","/World/Dropper",
             "/World/Match","/World/PlatinumWire","/World/HClBottle","/World/SampleBottle"]:
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        print(f"  {path}: INVALID"); continue
    xf = UsdGeom.Xformable(p)
    ops = xf.GetOrderedXformOps()
    print(f"  {path} [{p.GetTypeName()}]:")
    if not ops:
        print("    (NO xform ops)")
    for op in ops:
        print(f"    {op.GetOpType()} = {op.Get()}")
    mn,mx = world_bbox(p)
    print(f"    world bbox center=({(mn[0]+mx[0])/2:.4f},{(mn[1]+mx[1])/2:.4f},{(mn[2]+mx[2])/2:.4f})")

# 4. SampleDish children + powder
print("\n=== SampleDish CHILDREN ===")
sd = stage.GetPrimAtPath("/World/SampleDish")
if sd.IsValid():
    for c in sd.GetChildren():
        vis = UsdGeom.Imageable(c).ComputeVisibility(Usd.TimeCode.Default()) if c.IsA(UsdGeom.Imageable) else "?"
        mn,mx = world_bbox(c)
        sz = mx-mn
        print(f"  {c.GetName()} [{c.GetTypeName()}] vis={vis} size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f})")

# 5. Material / texture references with absolute paths
print("\n=== ABSOLUTE / WINDOWS PATH REFERENCES ===")
found = 0
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        val = attr.Get()
        sv = str(val)
        if isinstance(val, str) and (':/' in sv or '\\\\' in sv or '.jpg' in sv.lower() or '.png' in sv.lower()):
            if val.startswith('/World') or val.startswith('./') or val.startswith('../'):
                continue
            print(f"  {prim.GetPath()}.{attr.GetName()} = {sv[:140]}")
            found += 1
        elif hasattr(val, 'path') and isinstance(val.path, str):
            p = val.path
            if ':' in p and not p.startswith('/World'):
                print(f"  {prim.GetPath()}.{attr.GetName()} (asset) = {p[:140]}")
                found += 1
        elif hasattr(val, 'resolvedPath'):
            rp = str(val.resolvedPath)
            if ':' in rp:
                print(f"  {prim.GetPath()}.{attr.GetName()} (resolved) = {rp[:140]}")
                found += 1
    if found > 40:
        print("  ... (truncated)")
        break
if found == 0:
    print("  (none found)")

# 6. Sublayers / references / payloads
print("\n=== SUBLAYERS / REFERENCES ===")
for op in stage.GetRootLayer().subLayerPaths:
    print(f"  sublayer: {op}")
root = stage.GetRootLayer()
print(f"  root layer: {root.identifier}")
print(f"  real path: {root.realPath}")

# 7. Lounge booth / background prims
print("\n=== BACKGROUND / LOUNGE / TABLE PRIMS ===")
for prim in stage.Traverse():
    p = str(prim.GetPath()).lower()
    if any(k in p for k in ('lounge','booth','background','dome','table','floor','wall')):
        if prim.GetTypeName() in ('Xform','Mesh','Cube','Cylinder','Sphere','DomeLight'):
            print(f"  {prim.GetPath()} [{prim.GetTypeName()}]")

# 8. Droplet / WaterJet / DishAcid
print("\n=== EFFECT PRIMS ===")
for path in ["/World/Droplet","/World/WaterJet","/World/DishAcid"]:
    p = stage.GetPrimAtPath(path)
    if p.IsValid():
        vis = UsdGeom.Imageable(p).ComputeVisibility(Usd.TimeCode.Default()) if p.IsA(UsdGeom.Imageable) else "?"
        mn,mx = world_bbox(p)
        sz = mx-mn
        print(f"  {path} [{p.GetTypeName()}] vis={vis} size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f})")
    else:
        print(f"  {path}: INVALID")

# 9. List all materials bound
print("\n=== MATERIALS ON STAGE ===")
for prim in stage.Traverse():
    if prim.IsA(UsdShade.Material):
        print(f"  {prim.GetPath()}")
