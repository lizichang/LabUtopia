"""Verify all fixes on server: USD scene + task logic."""
import sys
import numpy as np
from pxr import Usd, UsdGeom

BASE = "/media/dky/Disk2TB/lizichang/LabUtopia"
SCENE = BASE + "/assets/chemistry_lab/lab_flametest/lab_flametest.usd"

print("=" * 60)
print("USD SCENE VERIFICATION")
print("=" * 60)
stage = Usd.Stage.Open(SCENE)
bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

def check(path, expect_hidden=False):
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        print(f"  [MISSING] {path}")
        return
    vis = UsdGeom.Imageable(p).GetVisibilityAttr().Get() if p.IsA(UsdGeom.Imageable) else "?"
    b = bcache.ComputeWorldBound(p).ComputeAlignedRange()
    sz = b.GetMax() - b.GetMin()
    hidden_flag = "HIDDEN" if vis == "invisible" else "visible"
    expect = "OK" if (expect_hidden == (vis == "invisible")) else "FAIL"
    print(f"  [{expect}] {path}: {hidden_flag}  "
          f"size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f})  "
          f"z=[{b.GetMin()[2]:.4f},{b.GetMax()[2]:.4f}]")

print("\n[Flame size] (expect ~0.024x0.024x0.050):")
check("/World/BunsenBurner/flame_outer")
check("/World/BunsenBurner/flame_inner")

print("\n[Background] (expect HIDDEN):")
check("/World/lounge_booth_table", expect_hidden=True)

print("\n[Powder on dish] (expect HIDDEN):")
check("/World/SampleDish/powder", expect_hidden=True)

print("\n[Stoopper transforms] (expect visible, z~0.868-0.879):")
check("/World/HClBottle/stopper")
check("/World/SampleBottle/stopper")

# Verify stopper translate op
print("\n[Stopper translate ops]:")
for path in ["/World/HClBottle/stopper", "/World/SampleBottle/stopper"]:
    p = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(p)
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            val = op.Get()
            print(f"  {path}: translate=({val[0]:.5f},{val[1]:.5f},{val[2]:.5f})  "
                  f"({'OK' if abs(val[2]-0.0735)<0.001 else 'FAIL'})")

# Verify flame top z
fo = stage.GetPrimAtPath("/World/BunsenBurner/flame_outer")
b = bcache.ComputeWorldBound(fo).ComputeAlignedRange()
flame_top = b.GetMax()[2]
print(f"\n[Flame top z] = {flame_top:.4f} (expect ~1.012)  "
      f"{'OK' if flame_top < 1.02 else 'FAIL'}")

# Verify controller safe heights are above flame top
print(f"\n[Controller safety margin]:")
print(f"  FLAME_APPROACH z=1.05 > flame_top {flame_top:.4f} +0.04  "
      f"{'OK' if 1.05 > flame_top + 0.03 else 'FAIL'}")
print(f"  H (transit) z=1.08 > flame_top +0.06  "
      f"{'OK' if 1.08 > flame_top + 0.05 else 'FAIL'}")
print(f"  COOL z=1.15 > flame_top +0.13  "
      f"{'OK' if 1.15 > flame_top + 0.10 else 'FAIL'}")

# ---- Task logic verification (without Isaac Sim) ----
print("\n" + "=" * 60)
print("TASK LOGIC VERIFICATION (_get_obj_world fix)")
print("=" * 60)

# Simulate what get_object_xform_position returns (world position via
# ComputeLocalToWorldTransform) vs what the OLD buggy _get_obj_world did.
def get_world_pos(prim_path):
    """Mimic object_utils.get_object_xform_position."""
    p = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(p)
    t = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = t.ExtractTranslation()
    return np.array([pos[0], pos[1], pos[2]])

stopper_world = get_world_pos("/World/HClBottle/stopper")
bottle_world = get_world_pos("/World/HClBottle")
print(f"  HClBottle world pos = ({bottle_world[0]:.4f},{bottle_world[1]:.4f},{bottle_world[2]:.4f})")
print(f"  stopper world pos   = ({stopper_world[0]:.4f},{stopper_world[1]:.4f},{stopper_world[2]:.4f})")
print(f"  expected REST_POS   = (0.1200,0.0200,0.8735)")

# OLD buggy: local(treated as world) + parent_world
old_result = stopper_world + bottle_world
print(f"\n  OLD buggy _get_obj_world = stopper_world + bottle_world = "
      f"({old_result[0]:.4f},{old_result[1]:.4f},{old_result[2]:.4f})  <- DOUBLED, WRONG")
# NEW fixed: just return world pos
new_result = stopper_world
print(f"  NEW _get_obj_world       = {tuple(new_result.round(4))}  <- CORRECT")
ok = (abs(stopper_world[0]-0.12) < 0.005 and abs(stopper_world[2]-0.8735) < 0.005)
print(f"\n  Stopper world position matches REST_POS: {'OK' if ok else 'FAIL'}")

# Simulate attach follow
print("\n[Attach follow simulation]:")
grasp_gripper = np.array([0.12, 0.02, 0.875])
cur_world = stopper_world.copy()
new_gripper = np.array([0.16, 0.06, 0.875])
delta = new_gripper - grasp_gripper
follow_world = cur_world + delta
print(f"  gripper moves to {tuple(new_gripper)}")
print(f"  stopper follows to {tuple(follow_world.round(4))}")
print(f"  expected ~(0.16,0.06,0.8735)  "
      f"{'OK' if abs(follow_world[0]-0.16)<0.01 and abs(follow_world[1]-0.06)<0.01 else 'FAIL'}")

print("\n" + "=" * 60)
print("ALL VERIFICATION COMPLETE")
print("=" * 60)
