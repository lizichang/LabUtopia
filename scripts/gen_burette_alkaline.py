# -*- coding: utf-8 -*-
"""碱式滴定管 50mL 白底蓝线款（Blender 精模管线）。

结构规格表（调研依据，见参考文档）：
- 库存 lab_inventory.json: 碱式滴定管 50mL, 0.1mL 分度, 玻璃+乳胶管+玻璃珠
- 厂家表 (citotest 84303-1050/3050): 总长 860mm（含胶管+尖嘴），A级允差 ±0.05mL
- 结构（百科）: 管身(内径均匀+精确刻度) → 下端乳胶管(内含玻璃珠控制流速) → 底端尖嘴玻璃管；
  0 刻度在上，读数自上而下由小变大
- 尺寸标注（估）: 管身外径 13mm、壁厚 1mm；胶管外径 14mm、内径 13.5mm、长 60mm；
  尖嘴外径 6mm、长 50mm；玻璃珠 φ13.2mm；白底蓝线 z 175-800mm

装配（z 向上，尖嘴口 = 0）:
  尖嘴  z 0.000-0.050（上端 15mm 插入胶管）
  胶管  z 0.035-0.095（下端套尖嘴 15mm、上端套管身 10mm，中段鼓包包住玻璃珠）
  管身  z 0.085-0.860（下端 10mm 被胶管包住，顶端火抛光圆角收口）
  总高  = 50 + 60 + 775 - 25 = 860mm ✓ 与厂家总长一致
  白底蓝线 z 0.175-0.800（0 刻度≈距管口 60mm，50 刻度≈距管身下端 90mm，均"估"）

材质: 管身/尖嘴/珠子=玻璃(transmission 1.0 后处理补)；胶管=乳胶(transmission 0.25 后处理补)；
      白底=乳白瓷；蓝线=蓝色釉

用法: blender --background --python gen_burette_alkaline.py
后处理: D:/anaconda_3/python.exe post_fix_usd.py <OUT>/burette_alkaline.usd \
        --rules '{"glass": {"transmission": 1.0}, "latex": {"transmission": 0.25}}'
"""
import bpy, bmesh, math, os, mathutils

# ==================== 参数区 ====================
OUT = r"C:\Users\lenovo\.qoderworkcn\workspace\msgcgs8dcmvk5w2n\burette"
NAME = "burette_alkaline"
SEGS = 64
# ================================================

os.makedirs(OUT, exist_ok=True)

# ---------- 0) 干净场景 + 米单位 ----------
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.length_unit = "METERS"


# ---------- helpers ----------

def _fix_radial_normals(bm):
    """薄壳/管壁面法线指向径向向外（外表面）。"""
    for f in bm.faces:
        c = f.calc_center_median()
        r_vec = mathutils.Vector((c.x, c.y, 0.0))
        if r_vec.length > 1e-9 and f.normal.dot(r_vec) < 0:
            f.normal_flip()


def add_lathe(name, profile, segs=SEGS):
    """实心旋转体：剖面 [r, z] 列表绕 Z 轴旋转 + 底封。"""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    verts = {}
    for i in range(segs):
        a0 = 2 * math.pi * i / segs
        for k, (r, z) in enumerate(profile):
            verts[(i, k)] = bm.verts.new((r * math.cos(a0), r * math.sin(a0), z))
    center_bottom = bm.verts.new((0.0, 0.0, profile[0][1]))
    for i in range(segs):
        i2 = (i + 1) % segs
        for k in range(len(profile) - 1):
            bm.faces.new([verts[(i, k)], verts[(i, k + 1)],
                          verts[(i2, k + 1)], verts[(i2, k)]])
    for i in range(segs):
        i2 = (i + 1) % segs
        bm.faces.new([verts[(i, 0)], verts[(i2, 0)], center_bottom])
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def add_lathe_hollow(name, profile, segs=SEGS):
    """开放环带旋转体：剖面是 [r, z] 折线路径（如外壁→顶沿→内壁），不封口。
    用于管壁/胶管等中空件（底端/顶端开口）。"""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    verts = {}
    for i in range(segs):
        a0 = 2 * math.pi * i / segs
        for k, (r, z) in enumerate(profile):
            verts[(i, k)] = bm.verts.new((r * math.cos(a0), r * math.sin(a0), z))
    for i in range(segs):
        i2 = (i + 1) % segs
        for k in range(len(profile) - 1):
            bm.faces.new([verts[(i, k)], verts[(i, k + 1)],
                          verts[(i2, k + 1)], verts[(i2, k)]])
    bm.normal_update()
    _fix_radial_normals(bm)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def add_arc_shell(name, r_in, r_out, z_bot, z_top, a_start_deg, a_end_deg, segs_arc=48):
    """部分圆周薄壳：内外表面（弧段），端面省略（0.2-0.3mm 厚看不见）。
    用于贴壁的白底瓷条/蓝线（r_in 贴管身外壁，r_out 为外表面）。"""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    a0, a1 = math.radians(a_start_deg), math.radians(a_end_deg)
    verts = {}
    for i in range(segs_arc + 1):
        a = a0 + (a1 - a0) * i / segs_arc
        ca, sa = math.cos(a), math.sin(a)
        verts[(i, 0)] = bm.verts.new((r_out * ca, r_out * sa, z_bot))
        verts[(i, 1)] = bm.verts.new((r_out * ca, r_out * sa, z_top))
        verts[(i, 2)] = bm.verts.new((r_in * ca, r_in * sa, z_bot))
        verts[(i, 3)] = bm.verts.new((r_in * ca, r_in * sa, z_top))
    for i in range(segs_arc):
        j = i + 1
        # 外表面（法线朝外）
        bm.faces.new([verts[(i, 0)], verts[(i, 1)], verts[(j, 1)], verts[(j, 0)]])
        # 内表面（法线朝内）
        bm.faces.new([verts[(j, 2)], verts[(j, 3)], verts[(i, 3)], verts[(i, 2)]])
    bm.normal_update()
    _fix_radial_normals(bm)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def add_sphere(name, r, center, sx=1.0, sy=1.0, sz=1.0):
    """球/椭球：半径 r，中心 center，非等比缩放 sx/sy/sz。"""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=center)
    obj = bpy.context.active_object
    obj.name = name
    if (sx, sy, sz) != (1.0, 1.0, 1.0):
        obj.scale = (sx, sy, sz)
        bpy.ops.object.transform_apply(scale=True)
    return obj


def set_mat(obj, mat_name, base_color=(0.8, 0.8, 0.8, 1.0), roughness=0.5,
            metallic=0.0, ior=1.45, transmission=0.0, specular=0.5):
    """材质 helper（中文界面节点名被翻译，按 type 查找！）。
    同名材质复用。transmission 导出会丢，必须 post_fix_usd.py 后处理补。"""
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = base_color
        bsdf.inputs["Roughness"].default_value = roughness
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["IOR"].default_value = ior
        # Blender 5.0 中 "Specular" 已改名为 "Specular IOR Level"，兼容检测
        if "Specular" in bsdf.inputs:
            bsdf.inputs["Specular"].default_value = specular
        elif "Specular IOR Level" in bsdf.inputs:
            bsdf.inputs["Specular IOR Level"].default_value = specular
        bsdf.inputs["Transmission Weight"].default_value = transmission
    obj.data.materials.append(mat)
    return mat


# ---------- 1) 建模（按结构规格表逐部件） ----------
# 尖嘴玻璃管：外径 6mm、内径 4mm（壁 1mm，中空管供液体流过），长 50mm，z 0-0.050
add_lathe_hollow("tube_tip",
    [(0.003, 0.0), (0.003, 0.042), (0.0024, 0.050), (0.002, 0.050), (0.002, 0.0)])

# 乳胶管：z 0.035-0.095，外径 14mm（中段鼓包包珠），内径 13.5mm（hollow，不封口）
# 剖面折线：外壁下→鼓包→外壁上→上端内折→内壁下
rubber = add_lathe_hollow("rubber_tube",
    [(0.0072, 0.035), (0.0072, 0.055), (0.0078, 0.065), (0.0078, 0.078),
     (0.0072, 0.085), (0.0072, 0.095), (0.00675, 0.095), (0.00675, 0.035)])

# 玻璃珠：φ13.2mm，球心 z=0.065（胶管中段，嵌在管内，直径比胶管内径小 0.3mm）
ball = add_sphere("glass_ball", 0.0066, (0.0, 0.0, 0.065))

# 管身：外径 13mm、壁厚 1mm，z 0.085-0.860（hollow，顶端火抛光圆角收口）
# 剖面折线：外壁下→外壁上→顶端内收→内壁下
add_lathe_hollow("tube_body",
    [(0.0065, 0.085), (0.0065, 0.852), (0.0055, 0.858), (0.0055, 0.085)])

# 白底瓷条：弧 60°（宽≈7mm），贴管身外壁，z 0.175-0.800
add_arc_shell("white_strip", 0.0065, 0.0068, 0.175, 0.800, -30.0, 30.0)

# 蓝线：弧 8°（宽≈1mm），贴白底外，z 0.175-0.800
add_arc_shell("blue_line", 0.0068, 0.0070, 0.175, 0.800, -4.0, 4.0)

# 材质（同名复用：管身/尖嘴/珠子共用一个 glass 材质）
set_mat(rubber, "latex", base_color=(0.93, 0.91, 0.83, 1.0),
        roughness=0.45, transmission=0.25)
set_mat(ball, "glass", base_color=(0.85, 0.92, 0.98, 1.0),
        roughness=0.05, ior=1.45, transmission=1.0)
for o in bpy.data.objects:
    if o.name in ("tube_tip", "tube_body"):
        set_mat(o, "glass", base_color=(0.85, 0.92, 0.98, 1.0),
                roughness=0.05, ior=1.45, transmission=1.0)
for o in bpy.data.objects:
    if o.name == "white_strip":
        set_mat(o, "porcelain", base_color=(0.96, 0.96, 0.94, 1.0), roughness=0.3)
    if o.name == "blue_line":
        set_mat(o, "blue_ink", base_color=(0.05, 0.18, 0.78, 1.0), roughness=0.2)

# ---------- 2) 世界背景 + 相机 + 灯光（渲染验证用） ----------
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)  # 浅蓝天空，玻璃必需
bg.inputs["Strength"].default_value = 1.0
bpy.context.scene.world = world

H = 0.860  # 资产总高（米）
cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
bpy.context.collection.objects.link(cam)
bpy.context.scene.camera = cam

# 灯光按物体高度缩放
for pos, energy in [((1.4 * H, -1.4 * H, 1.6 * H), 2000.0),
                    ((-1.2 * H, 0.9 * H, 1.4 * H), 1200.0)]:
    lt = bpy.data.objects.new("key", bpy.data.lights.new("key", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    bpy.context.collection.objects.link(lt)

# ---------- 3) EEVEE 多视角渲染验证（玻璃透射必须开光线追踪） ----------
bpy.context.scene.render.engine = "BLENDER_EEVEE"
bpy.context.scene.eevee.use_raytracing = True
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 1024

views = [
    # (相机位置, 对准点, 文件名, (宽, 高))
    # 全貌：768x1536 竖幅，sensor fit VERTICAL(FOV_v=27°)，相机到物体中心 2.29m
    # 垂直视野约 1.1m，物体 0.86m 占画面 ~80% 高度
    ((1.29, -1.75, 1.124), (0.0, 0.0, H / 2), f"{NAME}_full", (768, 1536)),
    ((0.12, -0.15, 0.16), (0.0, 0.0, 0.065), f"{NAME}_tip", (1024, 1024)),
    ((0.14, -0.18, 0.55), (0.0, 0.0, 0.50), f"{NAME}_stripe", (1024, 1024)),
]
for loc, target, fname, (rx, ry) in views:
    cam.location = loc
    cam.rotation_euler = (mathutils.Vector(target) - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.render.resolution_x = rx
    bpy.context.scene.render.resolution_y = ry
    bpy.context.scene.render.filepath = os.path.join(OUT, f"{fname}.png")
    bpy.ops.render.render(write_still=True)
    print("RENDER SAVED", bpy.context.scene.render.filepath)

# ---------- 4) 导出前清理（相机/灯光/世界，否则混入 USD） ----------
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
bpy.context.scene.world = None
usd_path = os.path.join(OUT, f"{NAME}.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
