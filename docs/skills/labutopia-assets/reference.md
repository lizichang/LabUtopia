# LabUtopia 资产制作参考（reference）

## 资产管线总览

```
管线 A（简单资产，快）：
scripts/gen_*_assets.py (MeshBuilder 构建 + 导出 OBJ，模板: gen_dissolve_assets.py)
        ↓
scripts/obj2usd.py (OBJ → USD，UsdPreviewSurface 材质，米单位)
        ↓
assets/chemistry_lab/<name>.usd

管线 B（逼真资产，Blender 精模）：
scripts/blender_asset_template.py (bpy: bmesh 旋转体建模 + Principled BSDF + EEVEE 渲染验证 + USD 导出)
        ↓
scripts/post_fix_usd.py (pxr 后处理: 删残留灯光 + 补 transmission/opacity)
        ↓
assets/chemistry_lab/<name>.usd

两条管线汇合 ↓
scripts/gen_lab004_scene.py (REMOVE/REFS/BUILTIN 组装模板)
        ↓
assets/chemistry_lab/lab_XXX/lab_XXX.usd
```

> 五个模板脚本都在仓库 `scripts/` 目录下（git 管理）。执行前先确认存在：`git ls-files scripts/`。

---

## 实物调研与结构拆解通法（新资产第 1 步，产出结构规格表）

**核心思想：不背尺寸，只背方法。** 器材种类永远覆盖不完（胶头滴管、移液管、容量瓶、蒸发皿、坩埚、镊子……），这套流程必须能套在**任意**新器材上。泰坦烧杯表只是"已验证示范"，每个新器材都要按下面方法自己调研。

**为什么要调研**：同容量器材不同厂家系列尺寸不同。实测：泰坦低型 50mL 烧杯口外径 41mm×高 65mm×壁厚 2mm（高≈直径 1.4 倍），另一常见系列是 40×58mm。凭记忆写 PROFILE 必变形（坑 16）。

### 信息获取优先级

| 优先级 | 来源 | 怎么做 |
|---|---|---|
| 1 | **项目库存 lab_inventory.json**（E:\浙江大学\星辰计划\LabVLA_第一期轮转\） | equipment 的 name/material/notes 常自带尺寸材质（"玻璃棒 长约20cm 直径6mm"、"试管 外径15mm 长125mm"）。仿真要复现的就是这批实物，**优先于一切网上数据** |
| 2 | 厂家规格表 | 搜"低型烧杯 规格 直径 高度"；泰坦 tansoole / 白鲨 biosharp / chem17 仪器网的产品"尺寸一览表"，一页拿到整个容量系列（容量×外径×高度×壁厚），页面实物图存下来做渲染对照 |
| 3 | 国标 | 搜"GB/T 15724 烧杯"；国家标准全文公开系统 std.samr.gov.cn | 权威，但主要规定质量要求，具体尺寸系列看厂家表 |
| 4 | 3D 模型库解剖 | GrabCAD / Thingiverse 搜 beaker / 量筒 / 锥形瓶，下载后 Blender 打开看分件与相对位置 | 与解剖 alcohol_lamp 同思路：直接学真实结构（哪个部件套哪个） |
| 5 | 实物测量 | 实验室有实物就用游标卡尺量外径/高度/壁厚/口径 | 最准，同一尺寸量 2-3 个点取均值 |
| 6 | 估算 | 都找不到就按常识估，规格表里标"估"，渲染验证时重点检查该项 | 兜底，避免卡死 |

**非标容量提示**：国标只定义常见档位（如 GB/T 15724 烧杯只有 50/100/150/250/500mL 档）。遇到国标无档位的容量（如 125mL 锥形瓶是美式规格，国标对应档位是 100/150mL），改查美式供应商（Fisherbrand / VWR 等英文"product dimensions"尺寸表）；实在查不到就按相邻国标档位插值估算，规格表标"估"，渲染对照时重点检查。

### 结构拆解三步法（对任意器材）

1. **找主轴**：器材沿哪个方向延伸/旋转对称？玻璃器皿几乎都是 Z 轴旋转体（剖面 [r,z] 一条线旋转）；细长件（玻璃棒/移液管）沿 Z 延伸；夹具（镊子/试管夹）沿 Z 延伸但左右分叉
2. **拆部件**：主体 + 附属件。每个部件回答：什么形状原语？关键尺寸？**相对主体的位置**（偏移多少、在哪个高度）？
3. **定连接**：部件间关系——套入（瓶塞进瓶颈）、贴合（底贴台面）、独立（镊子两臂、冷凝管内外管）

### 形状原语映射表（拆解结果 → 建模操作，通法的落点）

| 实物部件 | 形状原语 | 建模方式 |
|---|---|---|
| 筒身/瓶颈/球泡/漏斗锥面/蒸发皿/坩埚（任何旋转对称件） | lathe 旋转体 | bmesh 剖面 [r,z] 旋转（模板 lathe 函数），PROFILE 从规格表直接换算：半径=外径/2（米） |
| 玻璃棒/移液管/温度计/颈管 | 细长旋转体 | lathe 直剖面（半径恒定） |
| 镊子臂/试管夹柄/坩埚钳（非旋转体） | 长方体+渐变 | 基本体 helper（add_box），细长件前后端尺寸不同就分两段 box 拼接；**简单化原则**：仿真不需要机械铰接细节，两臂+后端即可 |
| 滴管胶头/研杵头/勺头 | 椭球/半球 | 基本体 helper（add_sphere + 非等比缩放） |
| 瓶塞/底座/滴管胶头座 | 圆柱/锥台 | 基本体 helper（add_cylinder/锥台=lathe 斜剖面） |
| 表面皿/石棉网/圆盘 | 薄圆盘 | lathe 扁平剖面（[0,0]→[r,0]→[r,h]→[0,h] 小高度） |
| 环（铁圈/管口） | 环形 | lathe 空心剖面 |

### 结构规格表模板（建模输入，逐部件实现，禁止边建边想）

| 部件 | 形状原语 | 关键尺寸（mm） | 相对位置 | 材质 |
|---|---|---|---|---|
| 筒身 | lathe | 口外径 41、高 65、壁厚 2 | 底 z=0 → 口 z=0.065 | 玻璃 |
| 瓶塞 | 锥台 | 上径 12、下径 10、高 8 | 套入瓶颈口，z=0.160-0.168 | 玻璃 |

### 三个形状类别的完整示例（示范拆解思路，尺寸建模前按方法自查）

**① 纯旋转体单件——烧杯（已实现并验证）**：1 部件。50mL 泰坦低型：口外径 41×高 65×壁厚 2。PROFILE（米）= `[(0,0),(0.0205,0),(0.0205,0.063),(0.0212,0.065)]`（底中心→底外沿→壁顶→口沿微外翻）。简单方案=外轮廓实心，壁厚只记录在规格表，液体显示靠场景 BUILTIN 圆柱。

**② 多部件套合件——容量瓶 100mL**：3 部件。瓶身=梨形 lathe（最大直径约 56mm，从底 z=0 到肩 z≈0.13）；瓶颈=细长 lathe（外径约 10mm，肩部延续到总高 z≈0.165）；瓶塞=锥台（套入瓶颈口）。连接关系：瓶颈贴瓶身肩部（同轴连续），塞子套入瓶颈（间隙按磨口配合≈0.5mm）。刻度线是表面纹理，几何省略。尺寸标记：示例值，建模前查厂家表。

**③ 非旋转体——镊子（不锈钢）**：3 部件。两臂=细长 box（约 120×4×1.5mm，前端渐细分两段 box），后端连接块=box（约 12×6×4mm）把两臂收拢为 V 形。简单化原则：不做铰接、不做夹持动作机构，视觉上"两臂+后端"即可。尺寸标记：示例值，建模前查。

### 已验证示范样例：低型烧杯规格表（泰坦产品尺寸一览表）

| 容量 | 口外径(mm) | 高度(mm) | 壁厚(mm) |
|---|---|---|---|
| 25mL | 38.0 | 53.0 | 1.5 |
| 50mL | 41.0 | 65.0 | 2 |
| 100mL | 55.0 | 78.0 | 2 |
| 250mL | 70.0 | 110.0 | 2 |
| 500mL | 85.0 | 150.0 | 2 |
| 1000mL | 107.0 | 173.0 | 2.5 |

---

## 管线 A：MeshBuilder → OBJ → USD

### 几何 helpers（MeshBuilder）

以 scripts/gen_dissolve_assets.py 为模板，常用 4 个 helper：

```python
def lathe(mb, profile, axis, radius, start, end, steps, S, name, color=None, opacity=None, flip=False):
    """旋转体：profile 为 [x, y] 列表（截面），axis 为旋转轴（'y'/'x'），
    radius 为 [r0, r1] 半径范围，start/end 为轴方向高度范围。"""

def h_cylinder(mb, radius, height, axis, center, S, name, color=None, opacity=None, flip=False):
    """水平圆柱（手柄、立柱等）。"""

def ellipsoid_half(mb, rx, ry, rz, x0, x1, ax, S, name, color=None, opacity=None, flip=False):
    """半椭球（勺头），ax=0 时中心在 x0。"""

def annulus(mb, inner, outer, height, axis, center, S, name, color=None, opacity=None, flip=False):
    """圆环/孔板（试管架孔板：inner 为孔径）。"""
```

材质模板（make_usd）：

```python
def make_usd(mesh, S, name, color=(0.8, 0.8, 0.8), opacity=1.0, metallic=0.0, roughness=0.5):
    # UsdPreviewSurface shader + UsdPreviewSurface 材质
    # glass: opacity=0.35, roughness=0.1
    # steel: metallic=0.85, roughness=0.3, color=(0.75,0.78,0.82)
```

### obj2usd.py 关键函数（修复版 write_mesh）

**单位必须显式设为米**：`Usd.Stage.CreateNew` 默认 metersPerUnit=0.01（厘米），必须在 main() 里加 `UsdGeom.SetStageMetersPerUnit(stage, 1.0)`，否则 Blender 导入会整体缩小 100 倍（root scale=0.01）看不到物体。生成后验证：`UsdGeom.GetStageMetersPerUnit(stage) == 1.0`。脚本 docstring 声明"米单位"，代码必须同步实现。

**必须重映射局部顶点索引**，否则每个 prim 都包含全部顶点，bbox/geometry_center 全错：

```python
def write_mesh(stage, path, verts, faces, normals=None):
    used = sorted(set(idx for f in faces for idx in f))
    remap = {g: l for l, g in enumerate(used)}
    local_verts = verts[used]
    indices = []
    for f in faces:
        indices.extend(remap[i] for i in f)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(local_verts)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr(indices)
    if normals is not None:
        mesh.CreateNormalsAttr(normals[used])
```

---

## 管线 B：Blender bpy 精模管线

**适用**：用户要求逼真外观（像克隆资产那样）的器材。克隆资产（alcohol_lamp.usd）解剖结论：Blender 建模嵌套分件（/root/holder_bot/holder_bot 等）、upAxis=Z、metersPerUnit=1.0、材质参数全默认——逼真在几何分件，不在材质；我们用主动设参 + 后处理，材质更精细。

**运行环境**：Blender 5.0.1 headless。路径 `D:\Program Files\Blender Foundation\Blender 5.0\blender.exe`。运行：`blender --background --python <脚本>`。Blender 自带 python（有 bpy），**没有 pxr**——USD 后处理必须用本地 anaconda base（D:/anaconda_3/python.exe，有 pxr）。

### 模板脚本（scripts/blender_asset_template.py）

参数区改 3 处即可跑：`OUT`（输出目录）、`PROFILE`（旋转体剖面 [r, z] 列表）、`NAME`（资产名）。核心骨架：

```python
import bpy, bmesh, math, os, mathutils

OUT = r"..."          # 输出目录
NAME = "beaker"
PROFILE = [           # 剖面（米）：底中心->底沿->壁->口沿，Blender 中 z 朝上
    (0.0, 0.0), (0.020, 0.0), (0.020, 0.058), (0.0212, 0.0615)
]
SEGS = 64

# 1) 干净场景 + 米单位
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.length_unit = "METERS"

# 2) bmesh 旋转体（lathe）：SEGS 个经度 × 剖面点 的四边形带 + 底部扇面
mesh = bpy.data.meshes.new(f"{NAME}_mesh")
bm = bmesh.new()
verts = {}
for i in range(SEGS):
    a0 = 2 * math.pi * i / SEGS
    for k, (r, z) in enumerate(PROFILE):
        verts[(i, k)] = bm.verts.new((r * math.cos(a0), r * math.sin(a0), z))
center_bottom = bm.verts.new((0.0, 0.0, 0.0))
for i in range(SEGS):
    i2 = (i + 1) % SEGS
    for k in range(len(PROFILE) - 1):
        bm.faces.new([verts[(i, k)], verts[(i, k + 1)], verts[(i2, k + 1)], verts[(i2, k)]])
for i in range(SEGS):
    i2 = (i + 1) % SEGS
    bm.faces.new([verts[(i, 0)], verts[(i2, 0)], center_bottom])
bm.normal_update()
bm.to_mesh(mesh)
bm.free()
obj = bpy.data.objects.new(NAME, mesh)
bpy.context.collection.objects.link(obj)

# 3) Principled BSDF 材质（节点名被中文化，按 type 查找！）
mat = bpy.data.materials.new("glass")
mat.use_nodes = True
bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
bsdf.inputs["Base Color"].default_value = (0.85, 0.92, 0.98, 1.0)  # 浅蓝玻璃
bsdf.inputs["Roughness"].default_value = 0.05
bsdf.inputs["IOR"].default_value = 1.45
bsdf.inputs["Transmission Weight"].default_value = 1.0   # 导出会丢，后处理补
obj.data.materials.append(mat)

# 4) 世界背景（EEVEE 玻璃必须，否则全黑）+ 相机 + 灯光
world = bpy.data.worlds.new("World")
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.82, 0.87, 0.95, 1.0)
bg.inputs["Strength"].default_value = 1.0
bpy.context.scene.world = world

H = max(p[1] for p in PROFILE)               # 物体总高（米），相机距离按高度缩放
cam_data = bpy.data.cameras.new("cam")
cam = bpy.data.objects.new("cam", cam_data)
cam.location = (1.4 * H, -1.9 * H, 1.25 * H)  # 基准 (0.09,-0.12,0.08) 对应烧杯 H≈0.065
center = mathutils.Vector((0.0, 0.0, H / 2))  # 物体中心，让相机 -Z 精确对准
cam.rotation_euler = (center - cam.location).to_track_quat("-Z", "Y").to_euler()
bpy.context.collection.objects.link(cam)
bpy.context.scene.camera = cam

for pos, energy in [((0.3, -0.3, 0.35), 200), ((-0.25, 0.2, 0.3), 120)]:
    lt = bpy.data.objects.new("key", bpy.data.lights.new("key", "AREA"))
    lt.location = pos
    lt.data.energy = energy
    bpy.context.collection.objects.link(lt)

# 5) EEVEE 渲染验证（玻璃透射要开光线追踪）
bpy.context.scene.render.engine = "BLENDER_EEVEE"
bpy.context.scene.eevee.use_raytracing = True
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 1024
bpy.context.scene.render.filepath = os.path.join(OUT, f"{NAME}_render.png")
bpy.ops.render.render(write_still=True)

# 6) 导出前清理：删相机/灯光 + 摘世界（否则 USD 混入 cam/key/env_light）
for ob in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
    bpy.data.objects.remove(ob, do_unlink=True)
bpy.context.scene.world = None
bpy.ops.wm.usd_export(filepath=os.path.join(OUT, f"{NAME}.usd"), export_materials=True)
```

### 材质映射表（Blender Principled BSDF → USD PreviewSurface）

| Principled BSDF 输入 | USD 属性 | 备注 |
|---|---|---|
| Base Color | inputs:diffuseColor | 映射 ✓ |
| Metallic | inputs:metallic | 映射 ✓ |
| Roughness | inputs:roughness | 映射 ✓ |
| IOR | inputs:ior | 映射 ✓ |
| Specular | inputs:specular | 映射 ✓ |
| Clearcoat | inputs:clearcoat | 映射 ✓ |
| **Transmission Weight** | **不导出！** | 导出后 opacity=1.0 全不透明，必须 post_fix_usd.py 补 inputs:transmission |
| Alpha | inputs:opacity | 需要半透明时后处理补 opacity<1 更稳 |

常用材质参数表（写进资产就比克隆资产精细）：

| 材质 | diffuseColor | roughness | metallic | ior | transmission |
|---|---|---|---|---|---|
| 玻璃（烧杯/量筒） | (0.85,0.92,0.98) | 0.05 | 0 | 1.45 | 1.0 |
| 金属（铁架台） | (0.75,0.78,0.82) | 0.3 | 0.85 | — | 0 |
| 陶瓷（蒸发皿） | (0.9,0.9,0.92) | 0.4 | 0 | — | 0 |
| 塑料（洗瓶） | (0.75,0.85,0.9) | 0.5 | 0 | — | 0 |
| 棉/纸（灯芯） | (0.9,0.88,0.8) | 0.9 | 0 | — | 0 |

### 后处理（scripts/post_fix_usd.py）

Blender 导出丢 transmission，且可能残留灯光。用 anaconda base python（有 pxr）跑：

```bash
D:/anaconda_3/python.exe scripts/post_fix_usd.py <out.usd> --rules '{"glass": {"transmission": 1.0}}'
```

脚本行为：删除 /root/env_light → 按 --rules 给匹配材质名的 Shader 补 inputs:xxx 参数 → `stage.Save()` 原地保存。规则键 = 材质名（bpy 里 materials.new("glass") 的名字），值 = {USD 属性名: 值}。半透明玻璃就补 opacity 0.3；多个材质传多个键。

> 注意：这里的原地 Save 是**预期行为**（处理的是本次新产出的资产副本，补 transmission 就是要改它），与场景脚本"禁止 Save"不冲突（坑 19）。输入参数只能是自己生成的资产文件，**严禁**把源场景/被引用资产（lab_001/lab_003.usd 等）当输入。

### 验证（导出后必查）

```bash
D:/anaconda_3/python.exe -c "
from pxr import Usd, UsdGeom
s = Usd.Stage.Open('out.usd')
print(UsdGeom.GetStageMetersPerUnit(s), UsdGeom.GetStageUpAxis(s))
for p in s.GetPseudoRoot().GetChildren(): print(p.GetPath())
sh = s.GetPrimAtPath('/root/_materials/glass').GetChildren()[0]
print({a.GetName(): a.Get() for a in sh.GetAttributes()})
"
```

必查项：metersPerUnit==1.0；无 cam/key/env_light；玻璃 Shader 有 inputs:transmission==1.0。多材质资产还要确认每个 Material 名与 rules 键一致（名称不匹配后处理就静默跳过，漏补的玻璃在 Isaac 里是不透明灰）。

---

## 场景组装脚本骨架（gen_labXXX_scene.py）

```python
SRC = "assets/chemistry_lab/lab_001/lab_001.usd"   # 基础场景
OUT_DIR = "assets/chemistry_lab/lab_004/"
REMOVE = ["/World/xxx_prim", ...]                    # 删除不需要的 prim
REFS = {                                            # 资产引用 + 摆放
    "TestTubeRack": (src, (0.30, 0.08, 0.80)),
    "TestTube":     (src, (0.30, 0.08, 0.809)),     # 管底贴台面
    ...
}
BUILTIN = {                                         # 隐藏标记 prim（圆柱；形状必须匹配容器内腔，见坑 18）
    "PowderOnSpoon": ((0.305, 0.13, 0.80), 0.004, 0.008, (0.6,0.45,0.2)),
    "TubeWater":     ((0.30, 0.08, 0.831), 0.006, 0.035, (0.55,0.75,0.95), 0.6),
}

for prim_path in REMOVE:
    stage.RemovePrim(prim_path)
for name, (src, pos) in REFS.items():
    prim = stage.DefinePrim(f"/World/{name}", "Xform")
    prim.GetReferences().AddInternalReference(src)
    ops = prim.GetOrderedXformOps()
    if ops:
        ops[0].Set(Gf.Vec3d(*pos))        # 幂等
    else:
        prim.AddTranslateOp().Set(Gf.Vec3d(*pos))
# BUILTIN 用 UsdGeom.Cylinder.Define + AddTranslateOp + 颜色/透明度
# 直壁容器（烧杯/试管）→ 圆柱（半径 < 内径）；收口容器（锥形瓶/容量瓶）→ 截锥
# （UsdGeom.Cone.Define 或 Cylinder 缩放），半径按内腔在液面高度处的实际半径，否则直液柱悬空穿模（坑 18）
stage.Export(os.path.join(OUT_DIR, "lab_004.usd"))   # 禁止 Save()！
```

---

## 参考几何参数（D2 已标定）

| 物体 | 参数 |
|---|---|
| 试管 | 外径 r=7.5mm，内径 6.3mm（12.6mm），高 0.120；管底 z=0.809 → 管口 z=0.929 |
| 试管架 | 底座 z0-0.008，立柱 0.008-0.050，孔板 0.050-0.058，孔 r=0.0095 |
| 勺子 | 勺头宽 12mm（ellipsoid ay=0.006），手柄 h_cylinder；grasp 点 = 位置 + [0,0,0.0025] |
| 洗瓶 | body+spout lathe 0→0.135，cap 0.080-0.102；瓶颈 grasp_distance=0.018 |
| 烧杯 | 外径 40mm，高 58mm，壁厚 1mm（PROFILE 见模板），口沿微外翻 |
| 场景工作区 | x[0.2,0.37] y[-0.1,0.2]，桌面 z=0.80 |

## 本地验证要点

```python
# 读 xform 世界坐标（行主序！）
m = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
rot, t = m[:3,:3], m[3,:3]     # translate 在 row 3
# bbox（管线 A 可用）
ext = UsdGeom.Mesh(prim).ComputeExtent(0)  # 或用 GetWorldBoundingBox
# 管线 B（Blender 导出）的 Mesh 用 ComputeExtent 会抛
# "Improper value for 'points'"，改用手动读 points 算 bbox：
# pts = np.array(prim.GetAttribute('points').Get())
# size = pts.max(axis=0) - pts.min(axis=0)   # [x, y, z] 三个方向尺寸
```

验证项：bbox 尺寸、geometry_center 与参考点距离、参考点全部在工作区内、勺头 y 范围完全落入管口 y 范围。
