"""Inspect wire sub-parts, bottle liquid, dropper parts, and compute exact grasp offsets."""
import numpy as np
from pxr import Usd, UsdGeom, Gf

BASE = "/media/dky/Disk2TB/lizichang/LabUtopia"
SCENE = BASE + "/assets/chemistry_lab/lab_flametest/lab_flametest.usd"

stage = Usd.Stage.Open(SCENE)
bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

def inspect(path):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        print(f"  [MISSING] {path}")
        return None
    b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
    sz = b.GetMax() - b.GetMin()
    center = (b.GetMin() + b.GetMax()) / 2.0
    print(f"  {path}")
    print(f"    center=({center[0]:.5f},{center[1]:.5f},{center[2]:.5f}) "
          f"size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f}) "
          f"z=[{b.GetMin()[2]:.5f},{b.GetMax()[2]:.5f}] "
          f"x=[{b.GetMin()[0]:.5f},{b.GetMax()[0]:.5f}]")
    return b

print("=" * 60)
print("PlatinumWire SUB-PARTS")
print("=" * 60)
for child in stage.GetPrimAtPath("/World/PlatinumWire").GetChildren():
    inspect(str(child.GetPath()))

print("\n" + "=" * 60)
print("HClBottle SUB-PARTS (bottle, liquid)")
print("=" * 60)
for child in stage.GetPrimAtPath("/World/HClBottle").GetChildren():
    inspect(str(child.GetPath()))

print("\n" + "=" * 60)
print("SampleBottle SUB-PARTS")
print("=" * 60)
for child in stage.GetPrimAtPath("/World/SampleBottle").GetChildren():
    inspect(str(child.GetPath()))

print("\n" + "=" * 60)
print("Dropper SUB-PARTS")
print("=" * 60)
for child in stage.GetPrimAtPath("/World/Dropper").GetChildren():
    inspect(str(child.GetPath()))
    for sub in child.GetChildren():
        inspect(str(sub.GetPath()))

print("\n" + "=" * 60)
print("SamplePowderInBottle")
print("=" * 60)
inspect("/World/SamplePowderInBottle")

print("\n" + "=" * 60)
print("BunsenBurner tube (barrel)")
print("=" * 60)
inspect("/World/BunsenBurner/tube")
inspect("/World/BunsenBurner/base")

# Compute wire tip position relative to wire prim origin with rotation
print("\n" + "=" * 60)
print("WIRE GEOMETRY ANALYSIS")
print("=" * 60)
wire_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/PlatinumWire"))
world_t = wire_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
wire_origin = np.array([world_t.ExtractTranslation()[0],
                        world_t.ExtractTranslation()[1],
                        world_t.ExtractTranslation()[2]])
print(f"  wire prim origin (world) = ({wire_origin[0]:.5f},{wire_origin[1]:.5f},{wire_origin[2]:.5f})")

# Get the loop/wire tip bbox
for name in ["loop", "wire", "handle"]:
    p = stage.GetPrimAtPath(f"/World/PlatinumWire/{name}")
    if p.IsValid():
        b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
        center = (b.GetMin() + b.GetMax()) / 2.0
        print(f"  {name}: center=({center[0]:.5f},{center[1]:.5f},{center[2]:.5f}) "
              f"min_z={b.GetMin()[2]:.5f} max_z={b.GetMax()[2]:.5f} "
              f"min_x={b.GetMin()[0]:.5f} max_x={b.GetMax()[0]:.5f}")

# WIRE_GRASP = (0.228, -0.14, 0.898)
# Compute tip offset from grasp point
wire_grasp = np.array([0.228, -0.14, 0.898])
# The tip is at the lowest z point of the wire
wire_bbox = bcache.ComputeWorldBound(stage.GetPrimAtPath("/World/PlatinumWire")).ComputeAlignedRange()
tip_pos = np.array([wire_bbox.GetMin()[0], wire_bbox.GetMin()[1], wire_bbox.GetMin()[2]])
# But tip is not at min_x - it's at max_x (the loop end) with min_z
# Let's find the actual tip: with rotateY=150, the loop should be at +x, -z direction
# Check loop bbox
loop_b = bcache.ComputeWorldBound(stage.GetPrimAtPath("/World/PlatinumWire/loop")).ComputeAlignedRange()
loop_center = (loop_b.GetMin() + loop_b.GetMax()) / 2.0
print(f"\n  loop center = ({loop_center[0]:.5f},{loop_center[1]:.5f},{loop_center[2]:.5f})")
print(f"  loop min = ({loop_b.GetMin()[0]:.5f},{loop_b.GetMin()[1]:.5f},{loop_b.GetMin()[2]:.5f})")
print(f"  loop max = ({loop_b.GetMax()[0]:.5f},{loop_b.GetMax()[1]:.5f},{loop_b.GetMax()[2]:.5f})")

# Tip is approximately at loop center (the loop is at the end of the wire)
tip_approx = np.array([loop_center[0], loop_center[1], loop_b.GetMin()[2]])
offset_from_grasp = tip_approx - wire_grasp
print(f"\n  tip approx = ({tip_approx[0]:.5f},{tip_approx[1]:.5f},{tip_approx[2]:.5f})")
print(f"  offset from WIRE_GRASP = ({offset_from_grasp[0]:.5f},{offset_from_grasp[1]:.5f},{offset_from_grasp[2]:.5f})")
print(f"  current WIRE_TIP_OFFSET = (0.05475, 0.0, -0.09303)")

# Also check handle position relative to grasp
handle_b = bcache.ComputeWorldBound(stage.GetPrimAtPath("/World/PlatinumWire/handle")).ComputeAlignedRange()
handle_center = (handle_b.GetMin() + handle_b.GetMax()) / 2.0
print(f"\n  handle center = ({handle_center[0]:.5f},{handle_center[1]:.5f},{handle_center[2]:.5f})")
print(f"  WIRE_GRASP = ({wire_grasp[0]:.5f},{wire_grasp[1]:.5f},{wire_grasp[2]:.5f})")
print(f"  grasp at handle? offset = ({handle_center[0]-wire_grasp[0]:.5f},{handle_center[1]-wire_grasp[1]:.5f},{handle_center[2]-wire_grasp[2]:.5f})")

# Dropper analysis
print("\n" + "=" * 60)
print("DROPPER GEOMETRY ANALYSIS")
print("=" * 60)
drop_b = bcache.ComputeWorldBound(stage.GetPrimAtPath("/World/Dropper")).ComputeAlignedRange()
print(f"  dropper z range = [{drop_b.GetMin()[2]:.5f}, {drop_b.GetMax()[2]:.5f}]")
print(f"  dropper xy center = ({(drop_b.GetMin()[0]+drop_b.GetMax()[0])/2:.5f}, {(drop_b.GetMin()[1]+drop_b.GetMax()[1])/2:.5f})")
# Grasp at (0.12, -0.10, 0.90), nozzle at bottom z=0.80
print(f"  grasp z=0.90, nozzle z={drop_b.GetMin()[2]:.5f}, offset = {drop_b.GetMin()[2]-0.90:.5f}")
print(f"  top (bulb) z={drop_b.GetMax()[2]:.5f}, above grasp = {drop_b.GetMax()[2]-0.90:.5f}")

# Bottle opening analysis
print("\n" + "=" * 60)
print("BOTTLE OPENING ANALYSIS")
print("=" * 60)
for bname in ["HClBottle", "SampleBottle"]:
    p = stage.GetPrimAtPath(f"/World/{bname}")
    b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
    # Check bottle mesh specifically
    bp = stage.GetPrimAtPath(f"/World/{bname}/bottle")
    if bp.IsValid():
        bb = bcache.ComputeWorldBound(bp).ComputeAlignedRange()
        print(f"  {bname}/bottle: z=[{bb.GetMin()[2]:.5f},{bb.GetMax()[2]:.5f}] "
              f"xy=[{bb.GetMin()[0]:.4f},{bb.GetMin()[1]:.4f}]-[{bb.GetMax()[0]:.4f},{bb.GetMax()[1]:.4f}]")
    # liquid
    lp = stage.GetPrimAtPath(f"/World/{bname}/liquid")
    if lp.IsValid():
        lb = bcache.ComputeWorldBound(lp).ComputeAlignedRange()
        print(f"  {bname}/liquid: z=[{lb.GetMin()[2]:.5f},{lb.GetMax()[2]:.5f}] "
              f"center=({(lb.GetMin()[0]+lb.GetMax()[0])/2:.5f},{(lb.GetMin()[1]+lb.GetMax()[1])/2:.5f})")
    else:
        print(f"  {bname}/liquid: NOT FOUND")
    print(f"  {bname} top z = {b.GetMax()[2]:.5f}")

# Flame stain material check
print("\n" + "=" * 60)
print("FLAME STAIN YELLOW SHADER")
print("=" * 60)
shader = stage.GetPrimAtPath("/World/BunsenBurner/flame_stain_yellow_mat/Shader")
if shader.IsValid():
    for attr in shader.GetAttributes():
        name = attr.GetName()
        if "inputs:" in name:
            val = attr.Get()
            print(f"  {name} = {val}")
else:
    print("  Shader not found, checking children...")
    mat = stage.GetPrimAtPath("/World/BunsenBurner/flame_stain_yellow_mat")
    for child in mat.GetChildren():
        print(f"  {child.GetPath()} ({child.GetTypeName()})")
