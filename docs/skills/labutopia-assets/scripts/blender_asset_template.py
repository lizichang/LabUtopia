# -*- coding: utf-8 -*-
"""Blender bpy 精模资产模板（通法版）：旋转体 lathe + 基本体 helpers + Principled BSDF + EEVEE 渲染验证 + USD 导出。

用法: blender --background --python blender_asset_template.py
（Blender 5.0 headless；改参数区即可出新资产）

建模通法：实物结构规格表 → 每个部件选形状原语：
  - 旋转对称件 → add_lathe(剖面 [r,z] 米)    例：筒身/瓶颈/漏斗锥面/蒸发皿/坩埚/球泡
  - 圆柱/锥台 → add_cylinder                 例：玻璃棒/移液管/瓶塞/底座
  - 长方体组合 → add_box                     例：镊子臂/试管夹柄/坩埚钳
  - 椭球/半球 → add_sphere(缩放)             例：滴管胶头/研杵头/勺头
部件按相对位置摆放（套入/贴合/独立），材质用 set_mat 按材质表设定。

跑完后用 pxr 后处理补 transmission（Blender 导出器会丢）:
    D:/anaconda_3/python.exe scripts/post_fix_usd.py <OUT>/<NAME>.usd --rules '{"glass": {"transmission": 1.0}}'
"""
import bpy, bmesh, math, os, mathutils

# ==================== 参数区（每个新资产改这里） ====================
OUT = r""            # 输出目录，必填（如 r"D:/tmp/my_asset"）
NAME = "beaker"
# 旋转体剖面（米）：[半径 r, 高度 z] 列表，从底中心出发，z 朝上（Blender 约定）
PROFILE = [
    (0.0, 0.0),      # 底中心
    (0.0205, 0.0),   # 底部外沿（泰坦低型 50mL 口外径 41mm / 2）
    (0.0205, 0.063), # 壁顶
    (0.0212, 0.065)  # 口沿微外翻
]
SEGS = 64            # 圆周分段（越多越圆，64 足够）
# ===================================================================

if not OUT:
    raise SystemExit("参数区 OUT 未填写：请先设置输出目录（每个新资产必填）")
os.makedirs(OUT, exist_ok=True)

# ---------- 0) 干净场景 + 米单位 ----------
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.length_unit = "METERS"


# ---------- helpers：形状原语（通法落点） ----------

def add_lathe(name, profile, segs=SEGS):
    """旋转体：剖面 [r, z] 列表绕 Z 轴旋转，返回 obj。"""
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


def add_cylinder(name, r, h, z0):
    """圆柱：轴沿 Z，底在 z0，半径 r（米）。"""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    ring_a, ring_b = [], []
    for i in range(SEGS):
        a0 = 2 * math.pi * i / SEGS
        ring_a.append(bm.verts.new((r * math.cos(a0), r * math.sin(a0), z0)))
        ring_b.append(bm.verts.new((r * math.cos(a0), r * math.sin(a0), z0 + h)))
    ca, cb = bm.verts.new((0, 0, z0)), bm.verts.new((0, 0, z0 + h))
    for i in range(SEGS):
        i2 = (i + 1) % SEGS
        bm.faces.new([ring_a[i], ring_a[i2], ring_b[i2], ring_b[i]])
        bm.faces.new([ring_a[i2], ring_a[i], ca])
        bm.faces.new([ring_b[i], ring_b[i2], cb])
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def add_box(name, w, d, h, center):
    """长方体：宽 w(x) × 深 d(y) × 高 h(z)，中心在 center（米）。"""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=center)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (w, d, h)
    bpy.ops.object.transform_apply(scale=True)
    return obj


def add_sphere(name, r, center, sx=1.0, sy=1.0, sz=1.0):
    """球/椭球：半径 r，中心 center，非等比缩放 sx/sy/sz（如胶头 sz>1）。"""
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
    同名材质复用（多个部件共用一个材质）。
    注意：transmission 导出会丢，必须 post_fix_usd.py 后处理补。"""
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


# ---------- 1) 建模（按结构规格表逐部件，示例=烧杯单部件） ----------
obj = add_lathe(NAME, PROFILE)
set_mat(obj, "glass", base_color=(0.85, 0.92, 0.98, 1.0),
        roughness=0.05, ior=1.45, transmission=1.0)

# 多部件套合示例（容量瓶 100mL：瓶身 lathe + 瓶颈 lathe + 瓶塞 cylinder，同轴套合）：
# body = add_lathe("body", [(0,0),(0.028,0),(0.028,0.05),(0.005,0.13)])
# neck = add_lathe("neck", [(0.005,0.13),(0.005,0.165)])
# plug = add_cylinder("plug", 0.005, 0.008, 0.165)
# set_mat(body, "glass", base_color=(0.85,0.92,0.98,1.0), roughness=0.05, ior=1.45, transmission=1.0)
# set_mat(neck, "glass", ...)  # 同名材质自动复用

# 非旋转体示例（镊子：两臂 box + 后端连接块 box，V 形摆放）：
# arm1 = add_box("arm1", 0.12, 0.004, 0.0015, (0.0, -0.006, 0.06)); arm1.rotation_euler = (math.radians(8), 0, 0)
# arm2 = add_box("arm2", 0.12, 0.004, 0.0015, (0.0,  0.006, 0.06)); arm2.rotation_euler = (-math.radians(8), 0, 0)
# butt = add_box("butt", 0.012, 0.006, 0.004, (0.0, 0.0, 0.118))
# set_mat(arm1, "steel", base_color=(0.75,0.78,0.82,1.0), roughness=0.3, metallic=0.85)

# ---------- 2) 世界背景 + 相机 + 灯光（渲染验证用） ----------
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)  # 浅蓝天空，玻璃必需
bg.inputs["Strength"].default_value = 1.0
bpy.context.scene.world = world

H = max(p[1] for p in PROFILE)                # 物体总高（米），相机距离按高度缩放
cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
cam.location = (1.4 * H, -1.9 * H, 1.25 * H)   # 基准 (0.09,-0.12,0.08) 对应烧杯 H≈0.065；多部件资产按需微调
center = mathutils.Vector((0.0, 0.0, H / 2))   # 物体中心，让相机 -Z 精确对准
cam.rotation_euler = (center - cam.location).to_track_quat("-Z", "Y").to_euler()
bpy.context.collection.objects.link(cam)
bpy.context.scene.camera = cam

for pos, energy in [((0.3, -0.3, 0.35), 200), ((-0.25, 0.2, 0.3), 120)]:
    lt = bpy.data.objects.new("key", bpy.data.lights.new("key", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    bpy.context.collection.objects.link(lt)

# ---------- 3) EEVEE 渲染验证（玻璃透射必须开光线追踪） ----------
bpy.context.scene.render.engine = "BLENDER_EEVEE"
bpy.context.scene.eevee.use_raytracing = True
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 1024
bpy.context.scene.render.filepath = os.path.join(OUT, f"{NAME}_render.png")
bpy.ops.render.render(write_still=True)
print("RENDER SAVED", bpy.context.scene.render.filepath)

# ---------- 4) 导出前清理（相机/灯光/世界，否则混入 USD） ----------
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
bpy.context.scene.world = None
usd_path = os.path.join(OUT, f"{NAME}.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
