# -*- coding: utf-8 -*-
"""支管蒸馏烧瓶 100mL（Blender 精模管线）。

结构规格表（GB/T 15725.5-1995 × MEDILAB 100mL × Corning 250mL 交叉）:
- 球外径 65mm，全高 210mm（长颈型），颈外径 16mm 竖直，壁厚 1.2mm（GB/T 15±3 / 最小壁厚 0.8）
- 口部: beaded rim 翻边口（Corning 风格），口外缘 r 10.3mm
- 支管: 外径 8mm（GB/T 8±0.4），总长 72mm（用户定长 108*2/3），与瓶颈轴线夹角 75° 斜向下（Corning/HomeScience 75°），
  根部中心距口 60mm（HomeScience 250mL 参考），根部熔接埋入颈内腔
- 材质: 硼硅玻璃 transmission=1.0 ior=1.45 roughness=0.05
- 层级: /root/distillation_flask/{body, side_arm}，mPU=1.0

单位 mm。y 轴深度方向，x 宽度，z 高度。支管朝 +x。
"""
import bpy, bmesh, math, os
import mathutils

mm = 0.001
OUT = r"c:\Users\lenovo\.trae-cn\work\6a77604dd277d2635ee4de13\distillation_build"
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
sc.unit_settings.length_unit = "METERS"

SEG = 64

# ==================== 厚壁闭合旋转体 ====================
# 剖面 [r, z] mm: 底外中心 -> 外壁(球->肩->颈->翻边口) -> 内壁(颈->内腔球->内底中心)
PROFILE = [
    # ---- 外壁 ----
    (0.0, 0.0), (5.0, 0.7), (11.0, 2.2), (18.0, 5.0), (24.5, 9.0),
    (29.0, 14.0), (31.5, 20.0), (32.5, 26.0), (32.5, 32.5), (32.5, 39.0),
    (31.5, 45.0), (29.0, 51.0), (25.0, 56.0), (20.0, 60.0), (14.5, 62.6),
    (10.0, 63.9), (8.0, 64.4), (8.0, 203.0), (8.5, 204.6), (9.5, 206.2),
    (10.3, 207.6), (10.3, 208.8), (9.3, 209.7), (8.0, 210.0),
    # ---- 内壁（口往下）----
    (6.9, 208.9), (6.8, 207.2), (6.8, 64.0), (9.3, 62.4), (13.8, 61.0),
    (18.8, 58.2), (23.6, 54.2), (27.8, 48.6), (30.6, 41.4), (31.3, 32.5),
    (30.6, 24.4), (28.2, 16.6), (24.0, 9.6), (18.2, 4.6), (11.0, 2.0),
    (5.0, 1.5), (0.0, 1.5),
]


def add_closed_lathe(name, profile, segs=SEG):
    bm = bmesh.new()
    n = len(profile)
    ring = [[None] * n for _ in range(segs)]
    for i in range(segs):
        a = 2 * math.pi * i / segs
        ca, sa = math.cos(a), math.sin(a)
        for k, (r, z) in enumerate(profile):
            ring[i][k] = bm.verts.new((r * ca * mm, r * sa * mm, z * mm))
    for i in range(segs):
        i2 = (i + 1) % segs
        for k in range(n - 1):
            bm.faces.new([ring[i][k], ring[i][k + 1], ring[i2][k + 1], ring[i2][k]])
    # 两端轴心扇形封盖（profile 首尾 r=0）
    pole0 = bm.verts.new((0.0, 0.0, profile[0][1] * mm))
    poleN = bm.verts.new((0.0, 0.0, profile[-1][1] * mm))
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([pole0, ring[i][0], ring[i2][0]])
        bm.faces.new([poleN, ring[i2][n - 1], ring[i][n - 1]])
    bm.normal_update()
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()
    return obj


body = add_closed_lathe("body", PROFILE)

# ==================== 斜向空心支管 ====================
ARM_R_OUT, ARM_R_IN = 4.0, 3.0
ARM_LEN = 72.0                      # 108 * 2/3，用户定长
arm_deg = 75.0                     # 与瓶颈轴线（竖直）夹角，斜向下
d = mathutils.Vector((math.sin(math.radians(arm_deg)), 0.0,
                      -math.cos(math.radians(arm_deg))))
# 根部只埋入瓶颈右侧壁 3.5mm（不越过中轴），熔接处即真实玻璃烧制的加厚区
C_WALL = mathutils.Vector((8.0 * mm, 0.0, 150.0 * mm))   # 中心线与瓶颈外壁交点
p_root = C_WALL - d * (3.5 * mm)                      # 埋入壁内
p_tip = p_root + d * (ARM_LEN * mm)


def add_arm_tube(name, r_out, r_in, p0, p1, segs=48):
    """两端全开口空心管：外壁/内壁/两端环面。内孔与瓶内腔连通。"""
    dv = (p1 - p0).normalized()
    ref = mathutils.Vector((0.0, 1.0, 0.0))
    u = dv.cross(ref).normalized()
    v = dv.cross(u).normalized()

    def ring(center, r):
        pts = []
        for i in range(segs):
            a = 2 * math.pi * i / segs
            pts.append(bm.verts.new(center + (u * math.cos(a) + v * math.sin(a)) * r))
        return pts

    bm = bmesh.new()
    ro0, ro1 = ring(p0, r_out), ring(p1, r_out)
    ri0, ri1 = ring(p0, r_in), ring(p1, r_in)
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([ro0[i], ro0[i2], ro1[i2], ro1[i]])   # 外壁
        bm.faces.new([ro0[i2], ro0[i], ri0[i], ri0[i2]])   # 根部环面
        bm.faces.new([ro1[i], ro1[i2], ri1[i2], ri1[i]])   # 末端环面
        bm.faces.new([ri1[i2], ri1[i], ri0[i], ri0[i2]])   # 内壁
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()
    return obj


arm = add_arm_tube("side_arm", ARM_R_OUT * mm, ARM_R_IN * mm, p_root, p_tip)

# ==================== 玻璃材质 ====================
mat = bpy.data.materials.new("glass")
mat.use_nodes = True
mat.blend_method = "BLEND"          # alpha < 1 需 BLEND，否则 Eevee/视口仍按不透明画
mat.shadow_method = "NONE"          # 玻璃不投实心影
bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
bsdf.inputs["Base Color"].default_value = (0.92, 0.95, 0.96, 0.20)
bsdf.inputs["Roughness"].default_value = 0.05
bsdf.inputs["IOR"].default_value = 1.45
bsdf.inputs["Transmission Weight"].default_value = 1.0
# 关键：Blender 导 USD 时把材质 alpha → UsdPreviewSurface.opacity，而 Isaac Sim 只看
# opacity、忽略 transmission（transmission=1.0 那项白设，不透明 → 看不到瓶内液面/沸石/
# 沸腾）。原 alpha=1.0 导致玻璃完全不透明，降到 0.20（项目玻璃惯例 0.12~0.30，蒸馏烧瓶
# 薄壁 1.2mm 取中间档）。Base Color 第 4 分量与 Alpha 输入都设为 0.20，无论导出器读哪个都正确。
bsdf.inputs["Alpha"].default_value = 0.20
if "Specular IOR Level" in bsdf.inputs:
    bsdf.inputs["Specular IOR Level"].default_value = 0.5
for ob in (body, arm):
    ob.data.materials.append(mat)

# 两部件归入一个 Xform（导出为 /root/distillation_flask）
holder = bpy.data.objects.new("distillation_flask", None)
holder.empty_display_type = "PLAIN_AXES"
bpy.context.collection.objects.link(holder)
for part in (body, arm):
    part.parent = holder

# ==================== 渲染验证 ====================
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.20, 0.24, 0.30, 1.0)
bg.inputs["Strength"].default_value = 1.0
sc.world = world

cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
sc.camera = cam
bpy.context.collection.objects.link(cam)
sc.render.engine = "BLENDER_EEVEE"
sc.eevee.use_raytracing = True
sc.render.resolution_x = 1600
sc.render.resolution_y = 1200

for pos, energy in [((0.55, 0.55, 0.55), 25), ((-0.55, -0.35, 0.35), 10), ((0.15, 0.55, 0.70), 6)]:
    lt = bpy.data.objects.new("lt", bpy.data.lights.new("lt", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    lt.data.size = 0.8
    bpy.context.collection.objects.link(lt)


def shoot(name, loc, look, ortho=None, lens=55):
    cam.location = loc
    dvv = (mathutils.Vector(look) - cam.location).normalized()
    cam.rotation_euler = dvv.to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = lens
    if ortho:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho
    else:
        cam_data.type = "PERSP"
    sc.render.filepath = os.path.join(OUT, name)
    bpy.ops.render.render(write_still=True)
    print("rendered", name)


shoot("hero_front.png", (0.28, 0.30, 0.16), (0.03, 0, 0.10))
shoot("side_ortho.png", (0.0, 0.45, 0.11), (0.045, 0, 0.11), ortho=0.30)
shoot("front_ortho.png", (0.45, 0.0, 0.11), (0.0, 0, 0.11), ortho=0.30)
shoot("top_view.png", (0.03, 0.05, 0.50), (0.03, 0.0, 0.10))
shoot("mouth_closeup.png", (0.05, 0.15, 0.24), (0.0, 0.01, 0.205), lens=60)
shoot("arm_closeup.png", (0.065, 0.10, 0.17), (0.038, 0.0, 0.142), lens=60)

# ==================== 导出 ====================
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
sc.world = None
usd_path = os.path.join(OUT, "distillation_flask.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
