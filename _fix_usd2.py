"""Fix USD scene: stain cones (scale, opacity, emissive), match rotation, and any residual issues."""
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
import shutil, os

USD_PATH = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab/lab_flametest/lab_flametest.usd"
BAK_PATH = USD_PATH + ".bak_v18_stain"

if not os.path.exists(BAK_PATH):
    shutil.copy2(USD_PATH, BAK_PATH)
    print(f"Backup: {BAK_PATH}")
else:
    print(f"Backup already exists: {BAK_PATH}")

stage = Usd.Stage.Open(USD_PATH)

# ============================================================
# 1. Fix flame_stain_* cones: scale 0.7, semi-transparent, emissive
# ============================================================
print("\n=== Fixing flame stain cones ===")
FLAME_COLORS = {
    "yellow": (1.0, 0.85, 0.20),
    "purple": (0.70, 0.35, 1.0),
    "green":  (0.25, 0.90, 0.35),
    "red":    (1.0, 0.25, 0.15),
    "orange": (1.0, 0.55, 0.10),
    "blue":   (0.25, 0.55, 1.0),
}

for color, rgb in FLAME_COLORS.items():
    stain_path = f"/World/BunsenBurner/flame_stain_{color}"
    prim = stage.GetPrimAtPath(stain_path)
    if not prim.IsValid():
        print(f"  {color}: NOT FOUND, skipping")
        continue

    # Fix xform ops: set scale to (0.7, 0.7, 0.7) for localized glow
    xform = UsdGeom.Xformable(prim)
    scale_op = None
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op

    if scale_op is None:
        scale_op = xform.AddScaleOp()
    scale_op.Set(Gf.Vec3d(0.7, 0.7, 0.7))

    if translate_op is None:
        translate_op = xform.AddTranslateOp()
    # Keep translate at (0, 0, 0.2) for now; task code will reposition at runtime
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.2))

    # Fix material: semi-transparent + emissive glow
    mat_api = UsdShade.MaterialBindingAPI.Apply(prim)
    bound_mat = mat_api.ComputeBoundMaterial()[0]
    if bound_mat:
        mat_path = bound_mat.GetPath()
        shader = UsdShade.Shader(stage.GetPrimAtPath(f"{mat_path}/Shader"))
        if shader:
            # Opacity: 0.55 for semi-transparent additive look
            opacity_attr = shader.GetInput("opacity")
            if opacity_attr:
                opacity_attr.Set(0.55)
            else:
                shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.55)

            # Emissive color for glow effect
            emi = shader.GetInput("emissive_color")
            if emi:
                emi.Set(Gf.Vec3f(*rgb))
            else:
                shader.CreateInput("emissive_color",
                    Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))

            # Emissive intensity via diffuse color as well
            diff = shader.GetInput("diffuse_color")
            if diff:
                diff.Set(Gf.Vec3f(*rgb))
            else:
                shader.CreateInput("diffuse_color",
                    Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))

            print(f"  {color}: scale=0.7, opacity=0.55, emissive={rgb}")

# ============================================================
# 2. Fix match: rotateY=180 (horizontal, head toward burner)
# ============================================================
print("\n=== Fixing match rotation ===")
match_prim = stage.GetPrimAtPath("/World/Match")
if match_prim.IsValid():
    xform = UsdGeom.Xformable(match_prim)
    rot_op = None
    trans_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
            rot_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            trans_op = op

    if rot_op is None:
        rot_op = xform.AddRotateYOp()
    rot_op.Set(180.0)

    # Raise z by 3mm so the stick sits on the table (rotateY=180 flips local z)
    if trans_op is None:
        trans_op = xform.AddTranslateOp()
    trans_op.Set(Gf.Vec3d(0.50, 0.24, 0.803))

    print("  Match: rotateY=180, translate=(0.50, 0.24, 0.803)")
    print("  Stick extends from x=0.417 to 0.50 (toward burner), head at x=0.402-0.419")

# ============================================================
# 3. Ensure flame_outer/inner opacity stays at 0.98 (don't let task code lower it)
# ============================================================
print("\n=== Checking flame materials ===")
for mat_name in ["flame_outer_mat", "flame_inner_mat"]:
    shader_path = f"/World/BunsenBurner/{mat_name}/Shader"
    shader_prim = stage.GetPrimAtPath(shader_path)
    if shader_prim.IsValid():
        shader = UsdShade.Shader(shader_prim)
        opacity_attr = shader.GetInput("opacity")
        if opacity_attr:
            val = opacity_attr.Get()
            print(f"  {mat_name} opacity = {val}")
            if val < 0.9:
                opacity_attr.Set(0.98)
                print(f"    -> reset to 0.98")

# ============================================================
# 4. Check if SampleBottle has powder (hidden or part of bottle)
# ============================================================
print("\n=== Checking SampleBottle powder ===")
sb = stage.GetPrimAtPath("/World/SampleBottle")
if sb.IsValid():
    for child in sb.GetAllChildren():
        print(f"  child: {child.GetName()} type={child.GetTypeName()}")
        if "powder" in child.GetName().lower():
            print(f"    Found powder prim!")

# Also check for powder at /World level
powder_prim = stage.GetPrimAtPath("/World/SampleDish/powder")
if powder_prim.IsValid():
    print("  /World/SampleDish/powder exists (will be hidden by task code)")

stage.GetRootLayer().Save()
print("\n=== USD scene saved! ===")
