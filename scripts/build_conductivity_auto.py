# -*- coding: utf-8 -*-
"""DDS-307A style bench conductivity meter (Blender precision build, v2 realism).

INESA DDS-307A: 290x210x95mm white case, recessed LCD window + frame, 5 keys
with Chinese legends, top multi-function electrode stand (socket, mast, arm,
clamp), full rear panel (sockets/fuse/switch/vents). DJS-1C platinized
electrode is a SEPARATE Xform prim (A3: arm grips cap -> beaker), with a
smooth drooping cable curve.

Coordinates mm: x width 290, y depth 210 (front +105), z height (feet 0..10,
case 8..95, deck 95..105, stand mast to 232). Electrode stands beside unit at
x=195, grip point = cap top z 125..166.
"""
import bpy, bmesh, math, os
import mathutils

mm = 0.001
OUT = r"c:\Users\lenovo\.trae-cn\work\6a77604dd277d2635ee4de13\conductivity_build"
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
sc.unit_settings.length_unit = "METERS"

SEG = 32


def finish(name, bm):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    try:
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.shade_auto_smooth(angle=math.radians(60))
    except Exception:
        pass
    return obj


def bevel(bm, offset_mm, segments=3, profile=0.55):
    bmesh.ops.bevel(bm, geom=bm.edges, offset=offset_mm * mm,
                    offset_type="OFFSET", segments=segments,
                    profile=profile, clamp_overlap=True, mark_sharp=True)


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


def cyl_y(name, r, y0, y1, x=0.0, z=0.0, segs=SEG):
    bm = bmesh.new()
    ra, rb = [], []
    for i in range(segs):
        a = 2 * math.pi * i / segs
        ra.append(bm.verts.new((x * mm + r * math.cos(a) * mm, y0 * mm, z * mm + r * math.sin(a) * mm)))
        rb.append(bm.verts.new((x * mm + r * math.cos(a) * mm, y1 * mm, z * mm + r * math.sin(a) * mm)))
    ca = bm.verts.new((x * mm, y0 * mm, z * mm))
    cb = bm.verts.new((x * mm, y1 * mm, z * mm))
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([ra[i], ra[i2], rb[i2], rb[i]])
        bm.faces.new([ra[i2], ra[i], ca])
        bm.faces.new([rb[i], rb[i2], cb])
    return finish(name, bm)


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


def boxes(name, specs, bev=None):
    """specs: (w,d,h, cx,cy,cz, ry) mm. bev: edge bevel mm."""
    bm = bmesh.new()
    for (w, d, h, cx, cy, cz, ry) in specs:
        M = mathutils.Matrix.LocRotScale(
            mathutils.Vector((cx * mm, cy * mm, cz * mm)),
            mathutils.Euler((0.0, math.radians(ry), 0.0)),
            mathutils.Vector((w * mm, d * mm, h * mm)))
        ret = bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.transform(bm, matrix=M, verts=ret["verts"])
    if bev:
        bevel(bm, bev)
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
            metallic=0.0, ior=1.45, transmission=0.0, coat=0.0):
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
        if "Coat Weight" in bsdf.inputs:
            bsdf.inputs["Coat Weight"].default_value = coat
        if "Specular IOR Level" in bsdf.inputs:
            bsdf.inputs["Specular IOR Level"].default_value = 0.5
    obj.data.materials.append(mat)
    return mat


SHELL = dict(base_color=(0.88, 0.87, 0.83, 1.0), roughness=0.42, coat=0.15)
DECK = dict(base_color=(0.85, 0.84, 0.80, 1.0), roughness=0.45, coat=0.12)
DARK = dict(base_color=(0.12, 0.12, 0.13, 1.0), roughness=0.38, coat=0.2)
BLACK = dict(base_color=(0.02, 0.02, 0.02, 1.0), roughness=0.55)
RUBBER = dict(base_color=(0.015, 0.015, 0.015, 1.0), roughness=0.85)
CHROME = dict(base_color=(0.74, 0.76, 0.79, 1.0), roughness=0.20, metallic=0.92)
SCREEN = dict(base_color=(0.05, 0.065, 0.06, 1.0), roughness=0.10, coat=0.6)
PLATINUM = dict(base_color=(0.03, 0.03, 0.032, 1.0), roughness=0.65)
RODGLASS = dict(base_color=(0.75, 0.82, 0.85, 1.0), roughness=0.06, transmission=1.0)
PRINT = dict(base_color=(0.02, 0.02, 0.025, 1.0), roughness=0.6)
PRINT_W = dict(base_color=(0.92, 0.92, 0.90, 1.0), roughness=0.6)
KEYFACE = dict(base_color=(0.05, 0.05, 0.055, 1.0), roughness=0.35, coat=0.25)

# ==================== case ====================
shell = boxes("shell", [
    (290, 196, 79, 0, 0, 55.5, 0),          # main box y -98..98, z 16..95
    (290, 16, 8, 0, 90, 12, 0),             # front lower lip y 82..98 z 8..16
], bev=3.0)
set_mat(shell, "shell", **SHELL)

deck = boxes("deck", [(196, 210, 10, 0, 0, 100, 0)], bev=2.5)   # top deck z 95..105
set_mat(deck, "deck", **DECK)

skirt = boxes("skirt", [(296, 216, 14, 0, 0, 8, 0)], bev=2.0)
set_mat(skirt, "paint_dark", **DARK)

feet = join("feet", *[cyl_z(f"foot{i}", 11, 0, 10, x, y)
                      for i, (x, y) in enumerate([(130, 88), (-130, 88), (130, -88), (-130, -88)])])
set_mat(feet, "rubber", **RUBBER)

# ==================== front panel: recessed LCD frame + keys ====================
frame = boxes("screen_frame", [
    (162, 3, 8, -52, 99.5, 92, 0),    # top bar z 88..96
    (162, 3, 8, -52, 99.5, 24, 0),    # bottom bar z 20..28
    (8, 3, 68, -127, 99.5, 60, 0),    # left post x -131..-123
    (8, 3, 68, 23, 99.5, 60, 0),      # right post x 19..27
], bev=1.0)
set_mat(frame, "paint_dark", **DARK)

glass = boxes("screen_glass", [(146, 2, 60, -52, 98, 58, 0)])   # recessed y 97..99
set_mat(glass, "screen_dark", **SCREEN)

keys = boxes("keyboard", [
    (34, 6, 26, 53, 99.5, 58, 0),    # COND/TDS wide key
    (26, 6, 26, 83, 99.5, 58, 0),    # temp
    (26, 6, 26, 108, 99.5, 58, 0),   # const
    (18, 6, 26, 132, 99.5, 58, 0),   # adjust
    (18, 6, 26, 132, 99.5, 22, 0),   # confirm (lower)
], bev=1.2)
set_mat(keys, "matte_black", **KEYFACE)

# ==================== front/rear print text ====================
CJK = None
for fp in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
    if os.path.exists(fp):
        CJK = bpy.data.fonts.load(fp)
        break

FACE_Y = mathutils.Matrix.Rotation(math.radians(180), 4, "Z") @ \
         mathutils.Matrix.Rotation(math.radians(90), 4, "X")


def add_text(name, content, size_mm, loc, extrude_mm=0.5, font=None, white=False):
    cu = bpy.data.curves.new(name, type="FONT")
    cu.body = content
    cu.size = size_mm * mm
    cu.extrude = extrude_mm * mm
    cu.resolution_u = 8
    if font:
        cu.font = font
    cu.align_x = "CENTER"
    cu.align_y = "CENTER"
    obj = bpy.data.objects.new(name, cu)
    obj.matrix_basis = mathutils.Matrix.Translation(
        (loc[0] * mm, loc[1] * mm, loc[2] * mm)) @ FACE_Y
    bpy.context.collection.objects.link(obj)
    set_mat(obj, "print_white" if white else "print_black", **(PRINT_W if white else PRINT))
    return obj


add_text("label_model", "DDS-307A 型电导率仪", 6, (25, 104.9, 100), 1.2, CJK)
add_text("label_brand", "INESA", 5, (-70, 104.9, 100), 1.0, None)
add_text("key_lbl_1", "电导率/TDS", 4, (53, 102.45, 58), 0.5, CJK, white=True)
add_text("key_lbl_2", "温度", 4, (83, 102.45, 58), 0.5, CJK, white=True)
add_text("key_lbl_3", "电极常数", 4, (108, 102.45, 58), 0.5, CJK, white=True)
add_text("key_lbl_4", "常数调节", 3.4, (132, 102.45, 58), 0.5, CJK, white=True)
add_text("key_lbl_5", "确认", 4, (132, 102.45, 22), 0.5, CJK, white=True)

# ==================== top electrode stand ====================
stand_parts = [
    cyl_z("stand_base", 30, 105, 108, 0, -70),
    boxes("stand_collar", [(46, 46, 4, 0, -70, 110, 0)], bev=1.5),
    cyl_z("stand_mast", 6, 108, 232, 0, -70),
    boxes("stand_arm", [(84, 10, 8, 36, -70, 228, 0)], bev=1.0),
    boxes("clamp_jaw", [(9, 8, 22, 52, -70, 216, 0), (9, 8, 22, 74, -70, 216, 0)], bev=1.0),
    cyl_x("knob_a", 6, 42, 48, -70, 216, segs=24),
    cyl_x("knob_b", 6, 78, 84, -70, 216, segs=24),
]
stand = join("stand", *stand_parts)
set_mat(stand, "chrome", **CHROME)

# ==================== rear panel ====================
back_parts = [
    cyl_y("meas_socket", 10, -106, -97, -60, 88),
    cyl_y("gnd_post", 4, -106, -97, -20, 88),
    cyl_y("temp_socket", 8, -106, -97, 20, 88),
    boxes("fuse", [(20, 6, 12, 60, -98, 82, 0)]),
    boxes("switch_lever", [(22, 7, 10, 100, -100, 82, 0)]),
    boxes("power_socket", [(26, 8, 16, 100, -98, 62, 0)]),
    boxes("rear_vents", [(140, 1.6, 3, 0, -98, z, 0) for z in (40, 46, 52, 58, 64, 70)]),
]
back = join("back_panel", *back_parts)
set_mat(back, "matte_black", **BLACK)

# ==================== electrode (SEPARATE prim; standing beside unit) ====================
EX, EY = 195, 40

rod_mesh = join("electrode_rod",
                cyl_z("rod2", 6, 12, 125, EX, EY),
                cyl_z("guard2", 9, 12, 30, EX, EY))
set_mat(rod_mesh, "glass", **RODGLASS)

blades = boxes("electrode_blades", [
    (6, 8, 14, EX - 8, EY, 7, 0), (6, 8, 14, EX + 8, EY, 7, 0)], bev=0.8)
set_mat(blades, "platinum_black", **PLATINUM)

capm = join("electrode_cap",
            cyl_z("cap3", 8, 125, 148, EX, EY),
            cyl_z("cap3b", 10, 148, 158, EX, EY),
            cyl_z("plug3", 5, 158, 166, EX, EY))
set_mat(capm, "matte_black", **BLACK)

# smooth drooping cable: cubic bezier chain from cap top down beside the rod
def bez(p0, p1, p2, p3, n):
    pts = []
    for i in range(n):
        t = i / (n - 1)
        mt = 1 - t
        x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
        z = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
        pts.append((x, z))
    return pts


cable_pts = (bez((195, 166), (186, 166), (178, 160), (175, 146), 12)[:-1]
             + bez((175, 146), (172, 132), (172, 108), (174, 84), 12)[:-1]
             + bez((174, 84), (176, 60), (180, 34), (184, 20), 12))

cu = bpy.data.curves.new("cable", type="CURVE")
cu.dimensions = "3D"
cu.resolution_u = 24
sp = cu.splines.new("POLY")
sp.points.add(len(cable_pts) - 1)
for i, (x, z) in enumerate(cable_pts):
    sp.points[i].co = (x * mm, EY * mm, z * mm, 1.0)
cu.bevel_depth = 3.5 * mm
cu.bevel_resolution = 10
cable = bpy.data.objects.new("electrode_cable", cu)
bpy.context.collection.objects.link(cable)
set_mat(cable, "matte_black", **BLACK)

# electrode parts parented under one Empty -> exports as /root/electrode Xform
holder = bpy.data.objects.new("electrode", None)
holder.empty_display_type = "PLAIN_AXES"
bpy.context.collection.objects.link(holder)
for part in (rod_mesh, blades, capm, cable):
    part.parent = holder

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


shoot("hero_front.png", (0.40, 0.55, 0.30), (0.0, 0.0, 0.08))
shoot("front_panel.png", (0.0, 0.45, 0.12), (0.0, 0.10, 0.05), lens=60)
shoot("top_stand.png", (0.05, 0.15, 0.70), (0.0, -0.02, 0.05))
shoot("side_ortho.png", (-0.55, 0.0, 0.15), (0.0, 0.0, 0.15), ortho=0.65)
shoot("electrode.png", (0.32, 0.42, 0.22), (0.20, 0.04, 0.08), lens=60)
shoot("rear_panel.png", (0.0, -0.5, 0.3), (0.0, -0.09, 0.05))
shoot("key_closeup.png", (0.05, 0.35, 0.12), (0.1, 0.1, 0.06), lens=85)

# ==================== export ====================
# text/curve objects -> meshes so the USD exporter writes real geometry
bpy.ops.object.select_all(action="DESELECT")
for ob in [o for o in bpy.data.objects if o.type in ("FONT", "CURVE")]:
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.convert(target="MESH")

for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
sc.world = None
usd_path = os.path.join(OUT, "conductivity_meter.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
