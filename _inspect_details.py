"""Inspect stain materials and match geometry on the USD stage."""
from pxr import Usd, UsdGeom, UsdShade, Sdf

USD_PATH = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
stage = Usd.Stage.Open(USD_PATH)

# 1. Inspect stain material
print("=== Stain materials ===")
for color in ["yellow", "purple", "green", "red", "orange", "blue"]:
    stain_path = f"/World/BunsenBurner/flame_stain_{color}"
    prim = stage.GetPrimAtPath(stain_path)
    if not prim.IsValid():
        print(f"  {color}: NOT FOUND at {stain_path}")
        continue
    # Find material binding
    mat_api = UsdShade.MaterialBindingAPI.Apply(prim)
    bound_mat = mat_api.ComputeBoundMaterial()[0]
    if bound_mat:
        mat_path = bound_mat.GetPath()
        print(f"  {color}: bound to {mat_path}")
        shader = stage.GetPrimAtPath(f"{mat_path}/Shader")
        if shader.IsValid():
            for attr_name in ["inputs:opacity", "inputs:diffuse_color", "inputs:emissive_color",
                              "inputs:metallic", "inputs:roughness", "info:id"]:
                attr = shader.GetAttribute(attr_name)
                if attr and attr.HasValue():
                    print(f"    {attr_name} = {attr.Get()}")
    # Check xform ops
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        print(f"  {color} xform op: {op.GetOpType()} = {op.Get()}")

# 2. Inspect flame_outer material
print("\n=== Flame outer material ===")
for mat_name in ["flame_outer_mat", "flame_inner_mat"]:
    shader = stage.GetPrimAtPath(f"/World/BunsenBurner/{mat_name}/Shader")
    if shader.IsValid():
        print(f"  {mat_name}:")
        for attr_name in ["inputs:opacity", "inputs:diffuse_color", "inputs:emissive_color", "info:id"]:
            attr = shader.GetAttribute(attr_name)
            if attr and attr.HasValue():
                print(f"    {attr_name} = {attr.Get()}")

# 3. Inspect match geometry
print("\n=== Match geometry ===")
match_prim = stage.GetPrimAtPath("/World/Match")
if match_prim.IsValid():
    xform = UsdGeom.Xformable(match_prim)
    for op in xform.GetOrderedXformOps():
        print(f"  xform op: {op.GetOpType()} = {op.Get()}")
    # Find mesh children
    for child in match_prim.GetAllChildren():
        print(f"  child: {child.GetPath()} type={child.GetTypeName()}")
        if child.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(child)
            points = mesh.GetPointsAttr().Get()
            if points:
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                zs = [p[2] for p in points]
                print(f"    points: {len(points)}, local x=[{min(xs):.4f},{max(xs):.4f}] y=[{min(ys):.4f},{max(ys):.4f}] z=[{min(zs):.4f},{max(zs):.4f}]")
                print(f"    long axis: x_len={max(xs)-min(xs):.4f} y_len={max(ys)-min(ys):.4f} z_len={max(zs)-min(zs):.4f}")

# 4. Check HClBottle liquid level and neck
print("\n=== HClBottle structure ===")
hcl = stage.GetPrimAtPath("/World/HClBottle")
if hcl.IsValid():
    for child in hcl.GetAllChildren():
        print(f"  child: {child.GetName()} type={child.GetTypeName()}")
        if child.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(child)
            points = mesh.GetPointsAttr().Get()
            if points:
                zs = [p[2] for p in points]
                print(f"    z range: {min(zs):.4f} - {max(zs):.4f}")
        xform = UsdGeom.Xformable(child)
        for op in xform.GetOrderedXformOps():
            print(f"    xform: {op.GetOpType()} = {op.Get()}")

# 5. Check SampleBottle powder level
print("\n=== SampleBottle structure ===")
sb = stage.GetPrimAtPath("/World/SampleBottle")
if sb.IsValid():
    for child in sb.GetAllChildren():
        print(f"  child: {child.GetName()} type={child.GetTypeName()}")
        if child.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(child)
            points = mesh.GetPointsAttr().Get()
            if points:
                zs = [p[2] for p in points]
                print(f"    z range: {min(zs):.4f} - {max(zs):.4f}")
        xform = UsdGeom.Xformable(child)
        for op in xform.GetOrderedXformOps():
            print(f"    xform: {op.GetOpType()} = {op.Get()}")

# 6. Check dropper geometry
print("\n=== Dropper geometry ===")
dr = stage.GetPrimAtPath("/World/Dropper")
if dr.IsValid():
    xform = UsdGeom.Xformable(dr)
    for op in xform.GetOrderedXformOps():
        print(f"  xform: {op.GetOpType()} = {op.Get()}")
    for child in dr.GetAllChildren():
        print(f"  child: {child.GetName()} type={child.GetTypeName()}")
        if child.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(child)
            points = mesh.GetPointsAttr().Get()
            if points:
                zs = [p[2] for p in points]
                print(f"    z range: {min(zs):.4f} - {max(zs):.4f}")

stage.Close()
print("\nDone.")
