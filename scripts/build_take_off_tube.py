# -*- coding: utf-8 -*-
"""牛角管/尾接管（蒸馏接液管，Blender 精模管线）。

结构规格表（教材"105°接液管" × 商品调研 × D5 蒸馏装置配合）:
- 教材体系: 烧瓶支管 75°（与竖直）+ 接液管 105°（∠管内侧角）→ 本管下段竖直向下，
  上段沿支管同轴延长方向（与水平 -15°）——装置中蒸馏烧瓶/冷凝管/牛角管轴线共面衔接
- 变径管（牛角形渐缩，用户定）：口部外径 12mm/壁厚 1.3mm → 弯前 8.4mm → 弯后 6.8mm
  → 下段缓缩 6.2mm → 末端拉细至 3.6mm 滴尖/壁厚 0.8mm
- 上段直管 65mm + 弯弧 R30mm 转折 75° + 下段竖直 95mm，两端平口开孔全连通
- 材质: 硼硅玻璃 transmission=1.0 ior=1.45 roughness=0.05
- 层级: /root/take_off_tube/tube，mPU=1.0，原点=上段管口中心（插入端）

单位 mm。中心线在 xz 平面（y=0），支管朝 +x 侧。
"""
import bpy, bmesh, math, os
import mathutils

mm = 0.001
OUT = r"c:\Users\lenovo\.trae-cn\work\6a77604dd277d2635ee4de13\cowhorn_build"
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
sc.unit_settings.length_unit = "METERS"

SEG = 48          # 圆周分段
# ==================== 中心线参数 ====================
L1, R_ARC, L2 = 65.0, 30.0, 95.0  # 上段 / 弯弧半径 / 下段 mm
TH1 = -15.0                       # 上段方向与水平夹角（75°支管同轴延长）
TURN = 75.0                        # 轴线转折角（=180°-105°管内侧角）
ARC_LEN = R_ARC * math.radians(TURN)      # 弧长 39.3mm
TOTAL_LEN = L1 + ARC_LEN + L2             # 全管中心线 199.3mm
# 变径剖面（弧长 mm -> 外半径 mm）：口部最粗，全管连续渐缩，末端拉细滴尖
R_OUT_KEYS = [(0.0, 6.0), (L1, 4.2), (L1 + ARC_LEN, 3.4), (150.0, 3.1), (TOTAL_LEN, 1.8)]
WALL_KEYS = [(0.0, 1.3), (L1, 1.2), (L1 + ARC_LEN, 1.1), (TOTAL_LEN, 0.8)]

th1 = math.radians(TH1)
d1 = mathutils.Vector((math.cos(th1), 0.0, math.sin(th1)))     # 上段切向
P0 = mathutils.Vector((0.0, 0.0, 0.0))                          # 上段管口中心（原点）
B = P0 + d1 * (L1 * mm)                                         # 弧起点
n = mathutils.Vector((math.cos(th1 + math.pi / 2), 0.0,
                      math.sin(th1 + math.pi / 2)))              # 弧心方向（行进左侧）
C = B - n * (R_ARC * mm)                                        # 弧圆心
aB = th1 + math.pi / 2                                          # B 相对 C 方位角
aE = aB - math.radians(TURN)                                    # E 相对 C 方位角
E = C + R_ARC * mm * mathutils.Vector((math.cos(aE), 0.0, math.sin(aE)))
d2 = mathutils.Vector((math.cos(aE - math.pi / 2), 0.0, math.sin(aE - math.pi / 2)))
assert abs(d2.angle(mathutils.Vector((0, 0, -1)))) < 1e-4, "下段应竖直向下"

# ==================== 中心线采样（站点 + 切向） ====================
stations = []
NS1, NA, NS2 = 12, 18, 16        # 上段 / 弧 / 下段细分数
for i in range(NS1 + 1):
    stations.append((P0 + d1 * (L1 * mm * i / NS1), d1))
for i in range(1, NA + 1):
    a = aB - math.radians(TURN) * i / NA
    p = C + R_ARC * mm * mathutils.Vector((math.cos(a), 0.0, math.sin(a)))
    t = mathutils.Vector((math.cos(a - math.pi / 2), 0.0, math.sin(a - math.pi / 2)))
    stations.append((p, t))
for i in range(1, NS2 + 1):
    stations.append((E + d2 * (L2 * mm * i / NS2), d2))

# ==================== 变径空心管扫掠 ====================
def interp(keys, s):
    if s <= keys[0][0]:
        return keys[0][1]
    for (s0, v0), (s1, v1) in zip(keys, keys[1:]):
        if s <= s1:
            return v0 + (v1 - v0) * (s - s0) / (s1 - s0)
    return keys[-1][1]


# 站点弧长（mm，自口部起算）
svals = [0.0]
for k in range(1, len(stations)):
    svals.append(svals[-1] + (stations[k][0] - stations[k - 1][0]).length / mm)

bm = bmesh.new()
ring_out, ring_in = [], []
for k, (p, t) in enumerate(stations):
    r_out = interp(R_OUT_KEYS, svals[k])
    r_in = r_out - interp(WALL_KEYS, svals[k])
    u = t.cross(mathutils.Vector((0, 1, 0))).normalized()   # 截面基向量
    v = t.cross(u).normalized()
    ro, ri = [], []
    for i in range(SEG):
        a = 2 * math.pi * i / SEG
        w = u * math.cos(a) + v * math.sin(a)
        ro.append(bm.verts.new(p + w * (r_out * mm)))
        ri.append(bm.verts.new(p + w * (r_in * mm)))
    ring_out.append(ro)
    ring_in.append(ri)

N = len(stations)
for k in range(N - 1):
    for i in range(SEG):
        i2 = (i + 1) % SEG
        bm.faces.new([ring_out[k][i], ring_out[k][i2],
                      ring_out[k + 1][i2], ring_out[k + 1][i]])   # 外壁
        bm.faces.new([ring_in[k][i2], ring_in[k][i],
                      ring_in[k + 1][i], ring_in[k + 1][i2]])     # 内壁
for i in range(SEG):   # 两端环面（平口开孔）
    i2 = (i + 1) % SEG
    bm.faces.new([ring_out[0][i2], ring_out[0][i], ring_in[0][i], ring_in[0][i2]])
    bm.faces.new([ring_out[N - 1][i], ring_out[N - 1][i2],
                  ring_in[N - 1][i2], ring_in[N - 1][i]])
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

mesh = bpy.data.meshes.new("tube")
bm.to_mesh(mesh)
bm.free()
tube = bpy.data.objects.new("tube", mesh)
bpy.context.collection.objects.link(tube)
bpy.ops.object.select_all(action="DESELECT")
tube.select_set(True)
bpy.context.view_layer.objects.active = tube
bpy.ops.object.shade_smooth()

# ==================== 玻璃材质 ====================
mat = bpy.data.materials.new("glass")
mat.use_nodes = True
bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
bsdf.inputs["Base Color"].default_value = (0.92, 0.95, 0.96, 1.0)
bsdf.inputs["Roughness"].default_value = 0.05
bsdf.inputs["IOR"].default_value = 1.45
bsdf.inputs["Transmission Weight"].default_value = 1.0
if "Specular IOR Level" in bsdf.inputs:
    bsdf.inputs["Specular IOR Level"].default_value = 0.5
tube.data.materials.append(mat)

holder = bpy.data.objects.new("take_off_tube", None)
holder.empty_display_type = "PLAIN_AXES"
bpy.context.collection.objects.link(holder)
tube.parent = holder

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

for pos, energy in [((0.35, 0.45, 0.25), 20), ((-0.35, -0.30, 0.15), 10), ((0.20, 0.40, 0.45), 6)]:
    lt = bpy.data.objects.new("lt", bpy.data.lights.new("lt", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    lt.data.size = 0.8
    bpy.context.collection.objects.link(lt)

# 浅色背板（渲染衬托玻璃轮廓用，导出前删除）
bpy.ops.mesh.primitive_plane_add(size=0.35, location=(0.040, -0.12, -0.070), rotation=(math.pi / 2, 0, 0))
back = bpy.context.active_object
back.name = "backdrop"
bmat = bpy.data.materials.new("backdrop")
bmat.use_nodes = True
bb2 = next(n for n in bmat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
bb2.inputs["Base Color"].default_value = (0.88, 0.90, 0.92, 1.0)
bb2.inputs["Roughness"].default_value = 0.9
back.data.materials.append(bmat)


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


CENTER = (0.044 * 0.85, 0.0, -0.070 * 0.85)   # 大致几何中心（上管口在原点、下管口 z≈-0.141）
shoot("hero_front.png", (0.16, 0.22, 0.06), (0.040, 0.0, -0.075))
shoot("side_ortho.png", (0.045, 0.28, -0.070), (0.045, 0.0, -0.070), ortho=0.24)
shoot("front_ortho.png", (0.28, 0.0, -0.070), (0.045, 0.0, -0.070), ortho=0.24)
shoot("bend_closeup.png", (0.10, 0.05, -0.015), (0.079, 0.0, -0.030), lens=60)
shoot("mouth_closeup.png", (0.015, 0.035, 0.05), (0.0, 0.0, -0.006), lens=60)
shoot("tip_closeup.png", (0.105, 0.05, -0.115), (0.085, 0.0, -0.138), lens=60)

# ==================== 导出 ====================
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT") or o.name == "backdrop"]:
    bpy.data.objects.remove(ob, do_unlink=True)
sc.world = None
usd_path = os.path.join(OUT, "take_off_tube.usd")
bpy.ops.wm.usd_export(filepath=usd_path, export_materials=True)
print("USD SAVED", usd_path)
