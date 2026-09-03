# -*- coding: utf-8 -*-
"""冷凝管托架 / 托架式支撑（cradle stand，Blender 精模管线）。

结构规格表（d5_distillation_tmp.usd 实测几何 + 用户方案"底座+柱子+斜半圆柱壳"）:
- 冷凝管轴线实测: x=0.7048, z=0.9331+0.2864y → 倾角 16°, 方向 T=(0,0.9613,0.2756) (+y 上坡)
- 中段夹套外径 ~25.4mm (R12.7)；进出水侧管在 y 0.205-0.212 / 0.330-0.338（垂直向伸出23mm）
- 托架中心取 y=0.272（两侧水管间净区 0.215-0.328 中央），壳体 y 0.238-0.306 完全避开水管
- 托壳: 180° 半圆管, 内R14 (管R12.7+1.3间隙), 壁3mm, 长70mm, 轴线与冷凝管轴线重合
- 加强箍: 与壳同轴, 内R16.5(压入壳壁0.5), 外R23, 长30mm, 180°
- 立柱: Ø12 钢柱 z 11→190mm, 柱顶嵌入箍壁内(径向18.5-21.8, 焊接观感)
- 底座: 120×90×8 主板 + 108×78×3 台阶顶板（双层台阶精致感）
- 材质: 黑色珐琅喷塑 metallic=0 roughness=0.45
- 层级: /root/condenser_cradle_stand/{base_main,base_top,pole,collar,shell}, mPU=1.0
- 本资产局部坐标 = 场景坐标方向; 原点 = 底座中心底面
- 场景放置: translate=(0.7048, 0.272, 0.800) 无旋转（托壳轴线与冷凝管轴线按构造重合）

单位 mm。渲染验证用 Ø25.4 代理管（导出前删除）。
"""
import bpy, bmesh, math, os
import mathutils

mm = 0.001
OUT = r"c:\Users\lenovo\.trae-cn\work\6a77604dd277d2635ee4de13\cradle_build"
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
sc.unit_settings.length_unit = "METERS"

# ==================== 装置实测参数（场景提取，勿凭记忆改） ====================
H_MID = 0.211                                   # 托壳轴线高度 = 场景轴线z 1.0110 - 桌面 0.800
T = mathutils.Vector((0.0, 0.9613, 0.2756))     # 托壳轴向（冷凝管轴线方向, 16°上坡）
Wg = mathutils.Vector((0.0, 0.2756, -0.9613))   # 截面向下基向（垂直于T的"重力侧"）
XH = mathutils.Vector((1.0, 0.0, 0.0))
C = mathutils.Vector((0.0, 0.0, H_MID))         # 托壳轴线中心


def halfpipe(name, r_in, r_out, half_len, n_s, n_a):
    """180° 半圆管（托壳/加强箍）：轴过 C 方向 T，α 从 +x 水平侧经底部到 -x 水平侧。"""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    grids = {}
    for tag, r in (("out", r_out), ("in", r_in)):
        g = []
        for i in range(n_s + 1):
            s = (-half_len + 2.0 * half_len * i / n_s) * mm
            row = []
            for j in range(n_a + 1):
                a = math.radians(180.0 * j / n_a)
                u = XH * math.cos(a) + Wg * math.sin(a)
                row.append(bm.verts.new(C + T * s + u * (r * mm)))
            g.append(row)
        grids[tag] = g
    go, gi = grids["out"], grids["in"]
    for i in range(n_s):
        for j in range(n_a):
            bm.faces.new([go[i][j], go[i][j + 1], go[i + 1][j + 1], go[i + 1][j]])
            bm.faces.new([gi[i][j + 1], gi[i][j], gi[i + 1][j], gi[i + 1][j + 1]])
    for j in range(n_a):
        bm.faces.new([go[0][j + 1], go[0][j], gi[0][j], gi[0][j + 1]])
        bm.faces.new([go[n_s][j], go[n_s][j + 1], gi[n_s][j + 1], gi[n_s][j]])
    for i in range(n_s):
        bm.faces.new([go[i][0], go[i + 1][0], gi[i + 1][0], gi[i][0]])
        bm.faces.new([go[i + 1][n_a], go[i][n_a], gi[i][n_a], gi[i + 1][n_a]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()
    return obj


shell = halfpipe("shell", 14.0, 17.0, 35.0, 14, 24)
collar = halfpipe("collar", 16.5, 23.0, 15.0, 8, 24)


def box(name, w, d, h, z0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, z0 + h / 2))
    obj = bpy.context.active_object
    obj.name = name
    obj.data.name = f"{name}_mesh"
    obj.scale = (w, d, h)
    bpy.ops.object.transform_apply(scale=True)
    return obj


base_main = box("base_main", 0.120, 0.090, 0.008, 0.0)
base_top = box("base_top", 0.108, 0.078, 0.003, 0.008)

bpy.ops.mesh.primitive_cylinder_add(radius=0.006, depth=0.179, vertices=32,
                                    location=(0, 0, 0.011 + 0.179 / 2))
pole = bpy.context.active_object
pole.name = "pole"
pole.data.name = "pole_mesh"
try:
    bpy.ops.object.shade_auto_smooth(angle=math.radians(40))
except Exception:
    bpy.ops.object.shade_smooth()

# ==================== 材质（黑色珐琅喷塑） ====================
enamel = bpy.data.materials.new("enamel_dark")
enamel.use_nodes = True
bsdf = next(n for n in enamel.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
bsdf.inputs["Base Color"].default_value = (0.10, 0.105, 0.12, 1.0)
bsdf.inputs["Roughness"].default_value = 0.45
bsdf.inputs["Metallic"].default_value = 0.0
for ob in (base_main, base_top, pole, collar, shell):
    ob.data.materials.append(enamel)

# ==================== 层级 ====================
holder = bpy.data.objects.new("condenser_cradle_stand", None)
holder.empty_display_type = "PLAIN_AXES"
bpy.context.collection.objects.link(holder)
for part in (base_main, base_top, pole, collar, shell):
    part.parent = holder

# ==================== 渲染验证（含 Ø25.4 代理管，模拟冷凝管卧入托壳） ====================
glass = bpy.data.materials.new("glass")
glass.use_nodes = True
gb = next(n for n in glass.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
gb.inputs["Base Color"].default_value = (0.92, 0.95, 0.96, 1.0)
gb.inputs["Roughness"].default_value = 0.05
gb.inputs["IOR"].default_value = 1.45
gb.inputs["Transmission Weight"].default_value = 1.0
if "Specular IOR Level" in gb.inputs:
    gb.inputs["Specular IOR Level"].default_value = 0.5

bpy.ops.mesh.primitive_cylinder_add(radius=0.0127, depth=0.22, vertices=48,
                                    location=(0, 0, H_MID))
proxy = bpy.context.active_object
proxy.name = "proxy_tube"
proxy.rotation_euler = T.to_track_quat("Z", "Y").to_euler()
proxy.data.materials.append(glass)

world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.20, 0.24, 0.30, 1.0)
bg.inputs["Strength"].default_value = 1.0
sc.world = world

flat = bpy.data.materials.new("flat")
flat.use_nodes = True
fb = next(n for n in flat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
fb.inputs["Base Color"].default_value = (0.88, 0.90, 0.92, 1.0)
fb.inputs["Roughness"].default_value = 0.9

bpy.ops.mesh.primitive_plane_add(size=0.7, location=(0.0, -0.60, 0.12),
                                 rotation=(math.pi / 2, 0, 0))
backdrop = bpy.context.active_object
backdrop.name = "backdrop"
backdrop.data.materials.append(flat)
bpy.ops.mesh.primitive_plane_add(size=0.6, location=(0.0, 0.0, -0.0005))
ground = bpy.context.active_object
ground.name = "ground"
ground.data.materials.append(flat)

for pos, energy in [((0.35, -0.15, 0.35), 15), ((-0.30, 0.25, 0.20), 10), ((0.10, -0.35, 0.10), 8)]:
    lt = bpy.data.objects.new("lt", bpy.data.lights.new("lt", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    lt.data.size = 0.6
    bpy.context.collection.objects.link(lt)

cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
sc.camera = cam
bpy.context.collection.objects.link(cam)
sc.render.engine = "BLENDER_EEVEE"
sc.eevee.use_raytracing = True
sc.render.resolution_x = 1600
sc.render.resolution_y = 1200


def shoot(name, loc, look, ortho=None, lens=50):
    cam.location = loc
    dv = (mathutils.Vector(look) - cam.location).normalized()
    cam.rotation_euler = dv.to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = lens
    if ortho:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho
    else:
        cam_data.type = "PERSP"
    sc.render.filepath = os.path.join(OUT, name)
    bpy.ops.render.render(write_still=True)
    print("rendered", name)


shoot("hero_front.png", (-0.24, -0.40, 0.08), (0.01, 0.02, 0.12))
shoot("side_ortho.png", (-0.42, 0.02, 0.11), (0.0, 0.02, 0.11), ortho=0.30)
shoot("side_plusx.png", (0.40, -0.05, 0.10), (0.0, 0.02, 0.12))
shoot("uphill_view.png", (0.16, 0.42, 0.09), (0.0, 0.02, 0.12))
shoot("shell_closeup.png", (0.13, -0.11, 0.26), (0.0, 0.02, 0.20), lens=55)
shoot("top_view.png", (0.10, -0.16, 0.52), (0.0, 0.0, 0.10))

# ==================== 导出 ====================
for ob in [o for o in bpy.data.objects
           if o.type in ("CAMERA", "LIGHT") or o.name in ("proxy_tube", "backdrop", "ground")]:
    bpy.data.objects.remove(ob, do_unlink=True)
for m in (glass, flat):
    bpy.data.materials.remove(m, do_unlink=True)
sc.world = None
usd_path = os.path.join(OUT, "condenser_cradle_stand.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
