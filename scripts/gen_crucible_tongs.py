# -*- coding: utf-8 -*-
"""坩埚钳 200mm 圆钢镀铬款（Blender 精模管线）。

结构规格表（调研依据）：
- 库存 lab_inventory.json: 坩埚钳, 铁/镀铬, 用于夹取高温坩埚/蒸发皿
- 厂家表 (政采招标/scharlab/alibaba): 教学常用总长 200mm（±3mm）；钳口为弯头、
  端头有齿纹；中心铆钉铰接；圆钢镀铬
- 结构 (实物图+专利 xjishu 201720190815): 像剪刀/长镊钳；两根金属杆在中部偏前
  位置**交叉铰接成 X 形**（铆钉贯穿两杆）；手柄后端两柄分开；钳口端两片朝同一
  方向延伸，末端向内并向前弯成钩状夹持端；铆钉在中部偏前

核心结构（钳子/剪刀的本质）：两根杆**斜着交叉**穿过铆钉，不是平行杆。
  上臂：手柄端 y=+0.015 → 钳口端 y=-0.005（斜杆穿过铆钉 y=0）
  下臂：手柄端 y=-0.015 → 钳口端 y=+0.005（斜杆穿过铆钉 y=0）
  两杆在 z=0.130 处交叉，铆钉沿 X 轴贯穿两杆厚度

尺寸标注（估，按 200mm 教学款比例）：
  手柄段长 130mm，钳口段长 70mm（直段 40mm + 弯段 30mm）
  杆宽 6mm、厚 2.5mm；钳口段宽 8mm；铆钉 φ5mm、长 12mm（沿 X 轴贯穿）
  手柄端 Y 间距 ±15mm（分开握持），钳口端 Y 间距 ±5mm（夹持口）
  两杆 X 向错开 ±1.5mm（避免重叠，留铆钉贯穿空间）
  钳口弯段向前(+X)弯 38.7°（X 偏移 20mm / Z 投影 25mm）

装配（z 向上，手柄底 z=0，钳口朝上；Y 为两杆分开方向，X 为杆厚度方向）：
  上臂主杆（斜）  从 (x=+0.0015, y=+0.015, z=0) 斜到 (x=+0.0015, y=-0.005, z=0.170)
                  绕 X 轴 +6.7°，杆长 0.1712m，穿过铆钉 (0,0,0.130)
  下臂主杆（斜）  从 (x=-0.0015, y=-0.015, z=0) 斜到 (x=-0.0015, y=+0.005, z=0.170)
                  绕 X 轴 -6.7°，与上臂在 z=0.130 交叉
  铆钉            沿 X 轴 φ5×12mm，位置 (0,0,0.130)，贯穿两杆厚度
  上臂钳口弯段    从主杆末端向前(+X)弯 38.7°到 (0.0215,-0.005,0.195)
  下臂钳口弯段    对称，到 (0.0085,+0.005,0.195)
  总高 ≈ 0.195m，钳口末端 X 偏移 +0.020m ✓

材质: 全件镀铬金属（铁基底+镀铬层），metallic=0.95、roughness=0.22、不透明
      无 transmission，无需 post_fix 补透射（导出前已清理相机/灯光/世界）

用法: blender --background --python gen_crucible_tongs.py
"""
import bpy, bmesh, math, os, mathutils

# ==================== 参数区 ====================
OUT = r"E:\浙江大学\星辰计划\LabVLA_第一期轮转\_tmp_crucible"
NAME = "crucible_tongs"
SEGS = 64
# ================================================

os.makedirs(OUT, exist_ok=True)

# ---------- 0) 干净场景 + 米单位 ----------
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.length_unit = "METERS"


# ---------- helpers ----------

def add_box(name, w, d, h, center, rot_x_deg=0.0, rot_y_deg=0.0):
    """长方体：宽 w(x) × 深 d(y) × 高 h(z)，中心 center。
    rot_x_deg: 绕 X 轴旋转（杆在 yz 平面内倾斜，用于 X 形交叉）
    rot_y_deg: 绕 Y 轴旋转（杆在 xz 平面内倾斜，用于钳口向前弯）
    旋转在原点 apply 烘焙到 mesh 后再平移——避免 USD xformOpOrder 导致旋转绕世界原点。"""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 0.0, 0.0))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (w, d, h)
    bpy.ops.object.transform_apply(scale=True)
    if rot_x_deg != 0.0:
        obj.rotation_euler = (math.radians(rot_x_deg), 0.0, 0.0)
        bpy.ops.object.transform_apply(rotation=True)
    if rot_y_deg != 0.0:
        obj.rotation_euler = (0.0, math.radians(rot_y_deg), 0.0)
        bpy.ops.object.transform_apply(rotation=True)
    obj.location = center
    return obj


def add_cylinder_x(name, r, h, center):
    """圆柱：沿 X 轴，中心 center，半径 r、长 h（米）。用于铆钉贯穿两杆厚度。"""
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h, location=center)
    obj = bpy.context.active_object
    obj.name = name
    # 默认沿 Z 轴，绕 Y 轴旋转 90° → 沿 X 轴
    obj.rotation_euler = (0.0, math.radians(90.0), 0.0)
    bpy.ops.object.transform_apply(rotation=True)
    return obj


def set_mat(obj, mat_name, base_color=(0.8, 0.8, 0.8, 1.0), roughness=0.5,
            metallic=0.0, ior=1.45, transmission=0.0, specular=0.5):
    """材质 helper（中文界面节点名被翻译，按 type 查找）。同名材质复用。"""
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = base_color
        bsdf.inputs["Roughness"].default_value = roughness
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["IOR"].default_value = ior
        if "Specular" in bsdf.inputs:
            bsdf.inputs["Specular"].default_value = specular
        elif "Specular IOR Level" in bsdf.inputs:
            bsdf.inputs["Specular IOR Level"].default_value = specular
        bsdf.inputs["Transmission Weight"].default_value = transmission
    obj.data.materials.append(mat)
    return mat


# ---------- 1) 建模（按结构规格表逐部件） ----------
# 钳子核心：两根斜杆在 z=0.130 交叉，铆钉沿 X 轴贯穿。
# 上臂主杆（斜）：手柄端 y=+0.015 → 钳口端 y=-0.005，绕 X 轴 +6.7°
#   杆长 = sqrt(0.020^2 + 0.170^2) = 0.1712，中心 (+0.0015, +0.005, 0.085)
arm_u = add_box("arm_upper", 0.0025, 0.006, 0.1712,
                (0.0015, 0.005, 0.085), rot_x_deg=6.7)

# 下臂主杆（斜）：手柄端 y=-0.015 → 钳口端 y=+0.005，绕 X 轴 -6.7°
#   中心 (-0.0015, -0.005, 0.085)
arm_l = add_box("arm_lower", 0.0025, 0.006, 0.1712,
                (-0.0015, -0.005, 0.085), rot_x_deg=-6.7)

# 铆钉：沿 X 轴 φ5×12mm，贯穿两杆厚度（两杆 x=±0.0015，总厚 0.005+间隙）
rivet = add_cylinder_x("rivet", 0.0025, 0.012, (0.0, 0.0, 0.130))

# 上臂钳口弯段：从主杆末端 (+0.0015,-0.005,0.170) 向前(+X)弯到 (+0.0215,-0.005,0.195)
#   绕 Y 轴 +38.7°，杆长 sqrt(0.020^2+0.025^2)=0.032，中心 (0.0115,-0.005,0.1825)
arm_uj = add_box("arm_upper_jaw", 0.0025, 0.008, 0.032,
                 (0.0115, -0.005, 0.1825), rot_y_deg=38.7)

# 下臂钳口弯段：从 (-0.0015,+0.005,0.170) 向前弯到 (+0.0085,+0.005,0.195)
arm_lj = add_box("arm_lower_jaw", 0.0025, 0.008, 0.032,
                 (0.0085, 0.005, 0.1825), rot_y_deg=38.7)

# 材质：全件镀铬金属（同名复用）
CHROME = dict(base_color=(0.85, 0.87, 0.90, 1.0), roughness=0.22, metallic=0.95,
              specular=0.6)
for o in (arm_u, arm_l, rivet, arm_uj, arm_lj):
    set_mat(o, "chrome_steel", **CHROME)

# ---------- 2) 世界背景 + 相机 + 灯光（渲染验证用） ----------
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)  # 浅蓝天空
bg.inputs["Strength"].default_value = 1.0
bpy.context.scene.world = world

H = 0.210  # 资产总高（米）
cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
bpy.context.collection.objects.link(cam)
bpy.context.scene.camera = cam

for pos, energy in [((1.4 * H, -1.4 * H, 1.6 * H), 200.0),
                    ((-1.2 * H, 0.9 * H, 1.4 * H), 120.0)]:
    lt = bpy.data.objects.new("key", bpy.data.lights.new("key", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    bpy.context.collection.objects.link(lt)

# ---------- 3) EEVEE 多视角渲染验证 ----------
bpy.context.scene.render.engine = "BLENDER_EEVEE"
bpy.context.scene.eevee.use_raytracing = True
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 1024

views = [
    # (相机位置, 对准点, 文件名)
    # 全貌斜视：相机在 (+X,-Y,+Z)，看物体中心，能看到 X 形交叉和钳口弯曲
    ((0.28, -0.22, 0.24), (0.01, 0.0, 0.10), f"{NAME}_full"),
    # 正视（看 YZ 平面）：相机在 +X 方向，正对 YZ 平面，能清楚看到两杆 X 形交叉
    ((0.34, 0.0, 0.12), (0.0, 0.0, 0.10), f"{NAME}_side"),
    # 俯视铰接点：从上看铆钉贯穿两杆
    ((0.02, -0.02, 0.32), (0.0, 0.0, 0.13), f"{NAME}_top"),
]
for loc, target, fname in views:
    cam.location = loc
    cam.rotation_euler = (mathutils.Vector(target) - cam.location).to_track_quat("-Z", "Y").to_euler()
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
