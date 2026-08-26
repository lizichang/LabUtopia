# -*- coding: utf-8 -*-
"""WZZ-3 style FULL-AUTO polarimeter (Blender precision build).

INESA WZZ-3 auto polarimeter: 605x370x260mm box, color touch-LCD console on
front (sloped), top sample chamber (open trough, V rails, end glass windows),
flip lid hinged at chamber rear (open 120deg, task closes lid via rotateX->0),
right-side lamp switch, back power/USB/RS232. LED source 589.3nm behind rear
window (warm glow). Reading "+12.526" as 7-seg green digits.

A2 task: tube drops VERTICALLY into chamber center (y -110..110) — lid at
120deg folds fully BEHIND chamber (all lid points y<=-133), zero blockage.
"""
import bpy, bmesh, math, os
import mathutils

mm = 0.001
OUT = r"c:\Users\lenovo\.trae-cn\work\6a77604dd277d2635ee4de13\polarimeter_auto"
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
sc.unit_settings.length_unit = "METERS"

SEG = 48


def finish(name, bm):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def cyl_x(name, r, x0, x1, y=0.0, z=0.0, segs=SEG):
    bm = bmesh.new()
    ra, rb = [], []
    for i in range(segs):
        a = 2 * math.pi * i / segs
        ra.append(bm.verts.new((x0 * mm, (y + r * math.cos(a)) * mm, (z + r * math.sin(a)) * mm)))
        rb.append(bm.verts.new((x1 * mm, (y + r * math.cos(a)) * mm, (z + r * math.sin(a)) * mm)))
    ca = bm.verts.new((x0 * mm, y * mm, z * mm))
    cb = bm.verts.new((x1 * mm, y * mm, z * mm))
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([ra[i], ra[i2], rb[i2], rb[i]])
        bm.faces.new([ra[i2], ra[i], ca])
        bm.faces.new([rb[i], rb[i2], cb])
    return finish(name, bm)


def cyl_z(name, r, z0, z1, x=0.0, y=0.0, segs=SEG):
    bm = bmesh.new()
    ra, rb = [], []
    for i in range(segs):
        a = 2 * math.pi * i / segs
        ra.append(bm.verts.new(((x + r * math.cos(a)) * mm, (y + r * math.sin(a)) * mm, z0 * mm)))
        rb.append(bm.verts.new(((x + r * math.cos(a)) * mm, (y + r * math.sin(a)) * mm, z1 * mm)))
    ca = bm.verts.new((x * mm, y * mm, z0 * mm))
    cb = bm.verts.new((x * mm, y * mm, z1 * mm))
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([ra[i], ra[i2], rb[i2], rb[i]])
        bm.faces.new([ra[i2], ra[i], ca])
        bm.faces.new([rb[i], rb[i2], cb])
    return finish(name, bm)


def boxes(name, specs):
    """specs: (w,d,h, cx,cy,cz, ry) mm."""
    bm = bmesh.new()
    for (w, d, h, cx, cy, cz, ry) in specs:
        M = mathutils.Matrix.LocRotScale(
            mathutils.Vector((cx * mm, cy * mm, cz * mm)),
            mathutils.Euler((0.0, math.radians(ry), 0.0)),
            mathutils.Vector((w * mm, d * mm, h * mm)))
        ret = bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.transform(bm, matrix=M, verts=ret["verts"])
    return finish(name, bm)


def boxes_tilt(name, specs, deg, pivot):
    """Boxes tilted about X axis through pivot (deg>0 leans top backward -y)."""
    bm = bmesh.new()
    R = mathutils.Euler((math.radians(deg), 0.0, 0.0), 'XYZ').to_matrix()
    P = mathutils.Vector((pivot[0] * mm, pivot[1] * mm, pivot[2] * mm))
    for (w, d, h, cx, cy, cz) in specs:
        M = mathutils.Matrix.LocRotScale(
            mathutils.Vector((cx * mm, cy * mm, cz * mm)),
            None, mathutils.Vector((w * mm, d * mm, h * mm)))
        ret = bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.transform(bm, matrix=M, verts=ret["verts"])
        for v in ret["verts"]:
            v.co = P + R @ (v.co - P)
    return finish(name, bm)


def join(name, *objs):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    objs[0].name = name
    objs[0].data.name = name
    return objs[0]


def set_mat(obj, mat_name, base_color=(0.8, 0.8, 0.8, 1.0), roughness=0.5,
            metallic=0.0, ior=1.45, transmission=0.0,
            emission=None, emission_strength=3.0):
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = base_color
        bsdf.inputs["Roughness"].default_value = roughness
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["IOR"].default_value = ior
        bsdf.inputs["Transmission Weight"].default_value = transmission
        if "Specular IOR Level" in bsdf.inputs:
            bsdf.inputs["Specular IOR Level"].default_value = 0.5
        if emission:
            out = next(n for n in mat.node_tree.nodes if n.type == "OUTPUT_MATERIAL")
            em = mat.node_tree.nodes.new("ShaderNodeEmission")
            em.inputs["Color"].default_value = emission
            em.inputs["Strength"].default_value = emission_strength
            mat.node_tree.links.new(out.inputs["Surface"], em.outputs[0])
    obj.data.materials.append(mat)
    return mat


SHELL = dict(base_color=(0.87, 0.86, 0.82, 1.0), roughness=0.45)
DARK = dict(base_color=(0.11, 0.11, 0.12, 1.0), roughness=0.40)
BLACK = dict(base_color=(0.02, 0.02, 0.02, 1.0), roughness=0.55)
RUBBER = dict(base_color=(0.015, 0.015, 0.015, 1.0), roughness=0.85)
CHROME = dict(base_color=(0.74, 0.76, 0.79, 1.0), roughness=0.22, metallic=0.9)
GLASS = dict(base_color=(0.55, 0.68, 0.78, 1.0), roughness=0.05, transmission=1.0)
SCREEN = dict(base_color=(0.02, 0.025, 0.03, 1.0), roughness=0.08)

# ==================== body shell (chamber void x[-52,52] y[-125,125]) ====================
shell_specs = [
    # side shells (light) z 58..240
    (133, 604, 182, -118.5, 0, 149, 0), (133, 604, 182, 118.5, 0, 149, 0),
    # front/rear between chamber sides
    (104, 177, 182, 0, 213.5, 149, 0), (104, 177, 182, 0, -213.5, 149, 0),
    # top plate frame z 240..250
    (133, 604, 10, -118.5, 0, 245, 0), (133, 604, 10, 118.5, 0, 245, 0),
    (104, 177, 10, 0, 213.5, 245, 0), (104, 177, 10, 0, -213.5, 245, 0),
]
shell = boxes("shell", shell_specs)
set_mat(shell, "shell", **SHELL)

skirt = boxes("skirt", [(366, 596, 48, 0, 0, 34, 0)])
set_mat(skirt, "paint_dark", **DARK)

feet = join("feet", *[cyl_z(f"foot{i}", 13, 0, 10, x, y)
                      for i, (x, y) in enumerate([(150, 255), (-150, 255), (150, -255), (-150, -255)])])
set_mat(feet, "rubber", **RUBBER)

# ==================== chamber (dark liner, floor, rails, windows) ====================
liner = boxes("chamber", [
    (96, 242, 4, 0, 0, 68, 0),                 # floor z 66..70
    (4, 250, 184, -50, 0, 158, 0), (4, 250, 184, 50, 0, 158, 0),   # side liners z 66..250
    (96, 4, 8, 0, 123, 74, 0), (96, 4, 8, 0, -123, 74, 0),         # end bottom strips z 70..78
    (96, 4, 88, 0, 123, 206, 0), (96, 4, 88, 0, -123, 206, 0),     # end top strips z 162..250
])
set_mat(liner, "matte_black", **BLACK)

rails = boxes("tube_rails", [
    (6, 250, 6, -16.5, 0, 112, 25), (6, 250, 6, 16.5, 0, 112, -25),
])
set_mat(rails, "chrome", **CHROME)

windows = boxes("windows", [
    (80, 3, 84, 0, 121.5, 120, 0), (80, 3, 84, 0, -121.5, 120, 0),
])
set_mat(windows, "glass", **GLASS)

glow = boxes("lamp_glow", [(56, 1.5, 56, 0, -122.5, 120, 0)])
set_mat(glow, "glow_warm", emission=(1.0, 0.80, 0.45, 1.0), emission_strength=3.5)

# ==================== flip lid (hinged at rear rim, open 120deg) ====================
# local origin = hinge point (0,-133,250.5); closed lid spans local y 0..260, z 0..9
lid = boxes("lid", [(112, 260, 9, 0, 130, 4.5, 0)])
lid.location = (0.0, -133 * mm, 250.5 * mm)
lid.rotation_euler = (math.radians(120.0), 0.0, 0.0)
set_mat(lid, "paint_dark", **DARK)

hinge = join("hinge",
             cyl_x("hinge_bar", 2.5, -60, 60, -133, 250.5),
             boxes("hinge_posts", [(7, 16, 16, -54, -133, 244, 0), (7, 16, 16, 54, -133, 244, 0)]))
set_mat(hinge, "chrome", **CHROME)

# ==================== display console (raised module on front top; screen only) ====================
# plinth sits fully ON the top plate (z 250..276); tilted panel clears the body
PIV = (0, 290, 276)
plinth = boxes("console_plinth", [(170, 64, 26, 0, 266, 263, 0)])
set_mat(plinth, "paint_dark", **DARK)

bezel = boxes_tilt("screen_bezel", [
    (196, 10, 9, 0, 296, 340),          # top rail
    (196, 10, 9, 0, 296, 279),          # bottom rail
    (9, 10, 70, -93.5, 296, 309.5),     # left stile
    (9, 10, 70, 93.5, 296, 309.5),      # right stile
], 15, PIV)
set_mat(bezel, "paint_dark", **DARK)
glass = boxes_tilt("screen_glass", [(178, 3, 62, 0, 293.5, 309.5)], 15, PIV)
set_mat(glass, "screen_dark", **SCREEN)

# ==================== switches / ports / brand ====================
sw = boxes("side_switch", [(6, 10, 18, 186.5, -100, 150, 0)])
set_mat(sw, "matte_black", **BLACK)
back = boxes("back_panel", [
    (30, 6, 16, 0, -305, 130, 0), (22, 4, 12, -60, -303.5, 130, 0), (22, 4, 12, 60, -303.5, 130, 0),
    (120, 2, 4, 0, -302.5, 72, 0), (120, 2, 4, 0, -302.5, 84, 0),
    (120, 2, 4, 0, -302.5, 96, 0), (120, 2, 4, 0, -302.5, 108, 0),
])
set_mat(back, "matte_black", **BLACK)
plate = boxes("brand_plate", [(46, 3, 8, 120, 303.5, 190, 0)])
set_mat(plate, "chrome", **CHROME)

# ==================== render verification ====================
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)
bg.inputs["Strength"].default_value = 0.35
sc.world = world

cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
sc.camera = cam
bpy.context.collection.objects.link(cam)
sc.render.engine = "BLENDER_EEVEE"
sc.eevee.use_raytracing = True
sc.render.resolution_x = 1200
sc.render.resolution_y = 900

for pos, energy in [((0.55, 0.55, 0.55), 28), ((-0.55, -0.35, 0.35), 10), ((0.15, 0.55, 0.70), 6)]:
    lt = bpy.data.objects.new("lt", bpy.data.lights.new("lt", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    lt.data.size = 0.8
    bpy.context.collection.objects.link(lt)


def shoot(name, loc, look, ortho=None, lens=55):
    cam.location = loc
    d = (mathutils.Vector(look) - cam.location).normalized()
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = lens
    if ortho:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho
    else:
        cam_data.type = "PERSP"
    sc.render.filepath = os.path.join(OUT, name)
    bpy.ops.render.render(write_still=True)
    print("rendered", name)


shoot("hero_front.png", (0.55, 0.75, 0.40), (0.0, 0.0, 0.15))
shoot("front_lcd.png", (0.0, 0.80, 0.35), (0.0, 0.28, 0.28))
shoot("top_chamber.png", (0.10, 0.25, 0.95), (0.0, -0.05, 0.08))
shoot("side_ortho.png", (-0.85, 0.0, 0.20), (0.0, 0.0, 0.20), ortho=0.85)
shoot("lcd_detail.png", (0.0, 0.50, 0.40), (0.0, 0.28, 0.30), lens=60)

# ==================== export ====================
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
sc.world = None
usd_path = os.path.join(OUT, "polarimeter.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
