"""Comprehensive scene inspection: positions, orientations, bboxes, stain prims."""
import numpy as np
from pxr import Usd, UsdGeom, Gf

BASE = "/media/dky/Disk2TB/lizichang/LabUtopia"
SCENE = BASE + "/assets/chemistry_lab/lab_flametest/lab_flametest.usd"

stage = Usd.Stage.Open(SCENE)
bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

def inspect(path, label=""):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        print(f"  [MISSING] {path}")
        return None
    xf = UsdGeom.Xformable(p)
    world_t = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = world_t.ExtractTranslation()
    # extract rotation as euler
    quat = world_t.ExtractRotationQuat()
    # get local transform ops
    ops = xf.GetOrderedXformOps()
    op_info = []
    for op in ops:
        ot = op.GetOpType()
        val = op.Get()
        if ot == UsdGeom.XformOp.TypeTranslate:
            op_info.append(f"translate=({val[0]:.4f},{val[1]:.4f},{val[2]:.4f})")
        elif ot == UsdGeom.XformOp.TypeRotateXYZ or ot == UsdGeom.XformOp.TypeRotateY:
            op_info.append(f"{op.GetBaseName()}={val}")
        elif ot == UsdGeom.XformOp.TypeScale:
            op_info.append(f"scale=({val[0]:.3f},{val[1]:.3f},{val[2]:.3f})")
        else:
            op_info.append(f"{op.GetBaseName()}")
    b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
    sz = b.GetMax() - b.GetMin()
    center = (b.GetMin() + b.GetMax()) / 2.0
    vis = UsdGeom.Imageable(p).GetVisibilityAttr().Get() if p.IsA(UsdGeom.Imageable) else "?"
    print(f"\n  [{label or path}]")
    print(f"    world_pos = ({pos[0]:.5f}, {pos[1]:.5f}, {pos[2]:.5f})")
    print(f"    bbox_center = ({center[0]:.5f}, {center[1]:.5f}, {center[2]:.5f})")
    print(f"    bbox_size = ({sz[0]:.4f}, {sz[1]:.4f}, {sz[2]:.4f})")
    print(f"    bbox_z = [{b.GetMin()[2]:.5f}, {b.GetMax()[2]:.5f}]")
    print(f"    bbox_xy = [{b.GetMin()[0]:.4f},{b.GetMin()[1]:.4f}]-[{b.GetMax()[0]:.4f},{b.GetMax()[1]:.4f}]")
    print(f"    vis = {vis}")
    print(f"    xform_ops = {op_info}")
    return {"pos": np.array([pos[0],pos[1],pos[2]]), "size": np.array([sz[0],sz[1],sz[2]]),
            "bbox_min": np.array([b.GetMin()[0],b.GetMin()[1],b.GetMin()[2]]),
            "bbox_max": np.array([b.GetMax()[0],b.GetMax()[1],b.GetMax()[2]])}

print("=" * 70)
print("ALL SCENE OBJECTS")
print("=" * 70)

# List all children of /World
world = stage.GetPrimAtPath("/World")
print("\n/World children:")
for child in world.GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

print("\n" + "-"*70)
print("KEY OBJECTS")
print("-"*70)

objs = {}
for path, label in [
    ("/World/SampleDish", "SampleDish (surface dish)"),
    ("/World/SampleDish/powder", "SampleDish/powder"),
    ("/World/HClBottle", "HClBottle"),
    ("/World/HClBottle/stopper", "HClBottle/stopper"),
    ("/World/Dropper", "Dropper"),
    ("/World/PlatinumWire", "PlatinumWire"),
    ("/World/SampleBottle", "SampleBottle"),
    ("/World/SampleBottle/stopper", "SampleBottle/stopper"),
    ("/World/BunsenBurner", "BunsenBurner"),
    ("/World/BunsenBurner/cap", "Burner cap"),
    ("/World/BunsenBurner/flame_outer", "flame_outer"),
    ("/World/BunsenBurner/flame_inner", "flame_inner"),
    ("/World/Match", "Match"),
    ("/World/WashBottle", "WashBottle"),
    ("/World/Droplet", "Droplet effect"),
    ("/World/WaterJet", "WaterJet effect"),
    ("/World/DishAcid", "DishAcid effect"),
]:
    objs[label] = inspect(path, label)

# Check for flame_stain prims
print("\n" + "-"*70)
print("FLAME STAIN PRIMS")
print("-"*70)
burner = stage.GetPrimAtPath("/World/BunsenBurner")
stain_found = False
for child in burner.GetChildren():
    name = child.GetName()
    if "stain" in name.lower():
        stain_found = True
        inspect(str(child.GetPath()), name)
if not stain_found:
    print("  *** NO flame_stain_* prims found! ***")

# Check HClBottle children for bottle opening geometry
print("\n" + "-"*70)
print("HClBottle CHILDREN (to find bottle mouth)")
print("-"*70)
for child in stage.GetPrimAtPath("/World/HClBottle").GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

print("\n" + "-"*70)
print("SampleBottle CHILDREN")
print("-"*70)
for child in stage.GetPrimAtPath("/World/SampleBottle").GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

print("\n" + "-"*70)
print("Dropper CHILDREN")
print("-"*70)
for child in stage.GetPrimAtPath("/World/Dropper").GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

print("\n" + "-"*70)
print("PlatinumWire CHILDREN")
print("-"*70)
for child in stage.GetPrimAtPath("/World/PlatinumWire").GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

print("\n" + "-"*70)
print("BunsenBurner ALL CHILDREN")
print("-"*70)
for child in stage.GetPrimAtPath("/World/BunsenBurner").GetChildren():
    print(f"  {child.GetPath()}  ({child.GetTypeName()})")

# Check table/ground
print("\n" + "-"*70)
print("TABLE / GROUND")
print("-"*70)
for path in ["/World/table", "/World/Table", "/World/Ground", "/World/ground", "/World/defaultGroundPlane"]:
    p = stage.GetPrimAtPath(path)
    if p.IsValid():
        inspect(path, path)

# Check for any other visible top-level objects that might be the "test tube"
print("\n" + "-"*70)
print("ALL VISIBLE TOP-LEVEL PRIMS")
print("-"*70)
for child in world.GetChildren():
    if child.IsA(UsdGeom.Imageable):
        vis = UsdGeom.Imageable(child).GetVisibilityAttr().Get()
        if vis != "invisible":
            b = bcache.ComputeWorldBound(child).ComputeAlignedRange()
            sz = b.GetMax() - b.GetMin()
            print(f"  {child.GetPath()}: vis={vis} size=({sz[0]:.3f},{sz[1]:.3f},{sz[2]:.3f}) "
                  f"z=[{b.GetMin()[2]:.3f},{b.GetMax()[2]:.3f}]")
