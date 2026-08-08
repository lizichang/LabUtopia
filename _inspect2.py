
from pxr import Usd, UsdGeom, UsdShade, Sdf
SCENE = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
stage = Usd.Stage.Open(SCENE)

print("=== LOUNGE BOOTH SHADER INPUTS (texture refs) ===")
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if "lounge_booth" in p and prim.IsA(UsdShade.Shader):
        shader = UsdShade.Shader(prim)
        print(f"\n  Shader: {p}")
        print(f"    info:id = {shader.GetShaderId()}")
        for inp in shader.GetInputs():
            v = inp.Get()
            if v is not None:
                print(f"    {inp.GetName()} = {str(v)[:120]}")

print("\n=== GROUND PLANE ===")
gp = stage.GetPrimAtPath("/World/GroundPlane")
if gp.IsValid():
    for attr in gp.GetAttributes():
        v = attr.Get()
        if v is not None and not isinstance(v, (list,)):
            print(f"  {attr.GetName()} = {str(v)[:100]}")
    for c in gp.GetChildren():
        print(f"  child: {c.GetPath()} [{c.GetTypeName()}]")
        if c.IsA(UsdShade.Material):
            pass

print("\n=== LOOKS SCOPE CONTENTS ===")
looks = stage.GetPrimAtPath("/World/Looks")
if looks.IsValid():
    for c in looks.GetChildren():
        print(f"  {c.GetName()} [{c.GetTypeName()}]")

print("\n=== TABLE SHADER (wood?) ===")
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if p.startswith("/World/table/") and prim.IsA(UsdShade.Shader):
        shader = UsdShade.Shader(prim)
        print(f"  Shader: {p}  id={shader.GetShaderId()}")
        for inp in shader.GetInputs():
            v = inp.Get()
            if v is not None and ('texture' in inp.GetName().lower() or 'file' in inp.GetName().lower() or inp.GetName() in ('diffuse_color_constant','albedo','base_color')):
                print(f"    {inp.GetName()} = {str(v)[:120]}")

print("\n=== EXTERNAL REFERENCES (references/payloads/sublayers) ===")
for prim in stage.Traverse():
    for rel_spec in prim.GetMetadata('references') or []:
        print(f"  {prim.GetPath()} reference: {rel_spec}")
    pdata = prim.GetMetadata('payload')
    if pdata:
        print(f"  {prim.GetPath()} payload: {pdata}")

print("\n=== LOUNGE BOOTH VISIBILITY ===")
lb = stage.GetPrimAtPath("/World/lounge_booth_table")
if lb.IsValid():
    vis = UsdGeom.Imageable(lb).ComputeVisibility(Usd.TimeCode.Default())
    print(f"  lounge_booth_table visibility = {vis}")
    # check world bbox
    bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    b = bcache.ComputeWorldBound(lb)
    r = b.ComputeAlignedRange()
    print(f"  lounge bbox min={r.GetMin()} max={r.GetMax()}")

print("\n=== SERVER TASK FILE HEAD (check version) ===")
