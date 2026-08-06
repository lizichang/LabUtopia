# -*- coding: utf-8 -*-
"""胶头滴管 15cm（Blender 精模管线）。

结构规格表（调研依据，见 labutopia-assets reference）：
- 库存 lab_inventory.json: 胶头滴管，长约 15cm，玻璃管身+橡胶胶头，
  胶头挤压可吸取液体，每滴约 0.05mL；另有"滴管（不含胶头）"同款玻璃件
- 补充尺寸（通用胶头滴管结构，标"估"）:
  玻璃管外径 8mm、壁厚 1mm（内径 6mm）
  尖嘴收口段长 30mm，出口孔径 1.6mm（0.05mL/滴对应小孔径）
  胶头外径 11mm、长 35mm（套住管身上端 5mm），顶部圆头
- 材质: 管身+尖嘴=玻璃(transmission 1.0 后处理补)；胶头=橡胶(浅灰不透明)

装配（z 向上，尖嘴出口 = 0）:
  玻璃件  z 0.000-0.120（尖嘴收口段 0-0.030，管身直段 0.030-0.120）
  胶头    z 0.115-0.150（套住管身上端 5mm，实心圆头）
  总高    = 150mm ✓ 与库存"长约15cm"一致

用法: blender --background --python gen_dropper.py
后处理: D:/anaconda_3/python.exe post_fix_usd.py <OUT>/dropper.usd \
        --rules '{"glass": {"transmission": 1.0}}'
"""
import bpy, bmesh, math, os, mathutils

# ==================== 参数区 ====================
OUT = r"C:\Users\lenovo\.qoderworkcn\workspace\mrizywidwxqhzovl\dropper_asset"
NAME = "dropper"
SEGS = 64
# ================================================

os.makedirs(OUT, exist_ok=True)

# ---------- 0) 干净场景 + 米单位 ----------
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.length_unit = "METERS"


# ---------- helpers（与 gen_burette_alkaline.py 一致） ----------

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
    用于管壁等中空件（底端/顶端开口）。"""
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
# 玻璃件（管身+尖嘴一体）：外径 8mm、壁厚 1mm，z 0-0.120
# 尖嘴收口段 z 0-0.030：外径 1.6mm→8mm，内径 1.2mm→6mm
# 管身直段 z 0.030-0.120：外径 8mm、内径 6mm
# 剖面折线：尖嘴出口外缘→收口外壁→管身外壁→顶端内折→内壁下行→出口内缘
add_lathe_hollow("glass_body",
    [(0.0008, 0.0), (0.004, 0.030), (0.004, 0.115), (0.0035, 0.120),
     (0.0025, 0.120), (0.003, 0.115), (0.003, 0.030), (0.0006, 0.0)])

# 胶头：橡胶实心旋转体，外径 11mm，z 0.115-0.150（套住管身上端 5mm），顶部圆头
add_lathe("rubber_bulb",
    [(0.0055, 0.115), (0.0055, 0.135), (0.0042, 0.147), (0.0, 0.150)])

# 材质
for o in bpy.data.objects:
    if o.name == "glass_body":
        set_mat(o, "glass", base_color=(0.85, 0.92, 0.98, 1.0),
                roughness=0.05, ior=1.45, transmission=1.0)
    if o.name == "rubber_bulb":
        set_mat(o, "rubber", base_color=(0.86, 0.85, 0.82, 1.0),
                roughness=0.55, metallic=0.0)

# ---------- 2) 世界背景 + 相机 + 灯光（渲染验证用） ----------
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)  # 浅蓝天空，玻璃必需
bg.inputs["Strength"].default_value = 1.0
bpy.context.scene.world = world

H = 0.150  # 资产总高（米）
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
    # 全貌：物体高 0.15m，相机距离 ~0.35m 即可完整入画（FOV 下自适应）
    ((0.25, -0.25, 0.20), (0.0, 0.0, H / 2), f"{NAME}_full", (768, 1536)),
    ((0.10, -0.12, 0.015), (0.0, 0.0, 0.010), f"{NAME}_tip", (1024, 1024)),
    ((0.10, -0.12, 0.135), (0.0, 0.0, 0.132), f"{NAME}_bulb", (1024, 1024)),
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
