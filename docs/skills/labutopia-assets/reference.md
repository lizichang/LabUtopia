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

### 几何 helpers（MeshBuilder 真实 API，来自 scripts/obj_gen.py）

**MeshBuilder 定义在 `scripts/obj_gen.py`**（随仓库 git 管理，与生成脚本同目录；生成脚本开头用 `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))` 找到它）。核心方法（真实签名，勿凭旧文档编造）：

```python
class MeshBuilder:
    def lathe(self, profile, segments, group, close_bottom=False, close_top=False,
              bottom_cap_r=0.0, top_cap_r=0.0, reverse=False):
        """旋转体：profile = [(r, z), ...] 绕 Z 轴旋转，z 朝上（米）。
        例：mb.lathe([(0.0075,0.0),(0.0075,0.12)], 40, "tube")  # 直管
            close_bottom=True 封底；reverse=True 法线朝内（做内壁）"""
    def h_cylinder(self, p0, p1, r, segments, group, cap=False):
        """圆柱：p0/p1 = 轴两端点 (x,y,z)，r 半径。例：手柄/立柱"""
    def sphere(self, cx, cy, cz, r, rings, segments, group):
        """球体"""
    def torus(self, cx, cy, cz, R, r_tube, segments_u, segments_v, group):
        """环面"""
    def to_obj(self, groups_order):
        """导出 OBJ 文本（groups_order 控制 prim 分组顺序）"""
    # 低层原语（写特殊形状时用）：
    #   _add_vert(p, n) -> 返回顶点索引
    #   _add_quad(a, b, c, d, group)
    #   _add_tri(a, b, c, group)
```

**gen_dissolve_assets.py 模板里自定义的 helper**（MeshBuilder 没有，新资产脚本照抄即可）：

```python
def shift(mb, dx, dy, dz):                    # 平移全部顶点（lathe 生成后移位置，如试管架立柱）
def annulus(mb, cx, cy, z0, z1, r_in, r_out, seg, group):
    """圆环板（真孔）：顶/底/外壁/内壁，法线正确。例：试管架孔板"""
def ellipsoid_half(mb, cx, cy, cz, ax, ay, az, rings, seg, group):
    """实心半椭球（z 从 0 到 cz，底面平，法线正确）。例：药匙勺头"""
```

### 管线 A 资产脚本完整骨架（gen_xxx_assets.py，单资产最小版）

完整多资产示例见 `scripts/gen_dissolve_assets.py`（6 个资产 + MTL 字典 + make_usd + main）。单资产最小骨架，照抄改 3 处（①②③ 注释标记）：

```python
# -*- coding: utf-8 -*-
"""生成 <新资产> OBJ+USD（单位：米）。"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py 同目录
from obj_gen import MeshBuilder  # noqa: E402

OUT_USD = r"E:/.../LabUtopia/assets/chemistry_lab"   # ① 资产输出目录（米单位）
OUT_OBJ = r"C:/.../outputs"                          # ② OBJ/MTL 暂存目录

# ---------- 建模：每个资产一个 build 函数 ----------
def build_xxx(mb):
    S = 40
    mb.lathe([(r1, z1), (r2, z2), ...], S, "body", close_bottom=True)  # ③ 主体剖面
    mb.h_cylinder(p0, p1, r, S, "handle", cap=True)                    #    附属件

# ---------- 材质参数表（键 = lathe/h_cylinder 的 group 名）----------
USD_MATS = {
    "body":   dict(diffuse=(0.85, 0.92, 0.98), opacity=0.35, roughness=0.05),  # 玻璃
    "handle": dict(diffuse=(0.70, 0.71, 0.74), metallic=0.85, roughness=0.35),  # 不锈钢
}
GROUPS = {"xxx": ["body", "handle"]}
BUILDERS = {"xxx": build_xxx}

# ---------- OBJ→USD（模板自带，照抄）----------
def make_usd(obj_path, out_usd, mat_specs):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh
    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)     # 米单位，勿省（坑 8）
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/{g}", verts, faces, vns)
        spec = mat_specs.get(g)
        if spec is not None:
            mat_path = f"/root/{g}_mat"
            mat = UsdShade.Material.Define(stage, mat_path)
            sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["diffuse"]))
            if spec.get("opacity", 1.0) < 1.0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
            if spec.get("metallic", 0.0) > 0:
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{out_usd}: {len(groups)} prims OK")

def main():
    for name in BUILDERS:
        mb = MeshBuilder()
        BUILDERS[name](mb)
        obj_path = os.path.join(OUT_OBJ, f"{name}.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(mb.to_obj(GROUPS[name]))
        make_usd(obj_path, os.path.join(OUT_USD, f"{name}.usd"), USD_MATS[name])
    print("DONE")

if __name__ == "__main__":
    main()
```

注意：多资产脚本在 `main()` 里循环 BUILDERS（OBJ/MTL 文本模板见 gen_dissolve_assets.py 的 MTL 字典，仅当目标渲染器需要 MTL 时写）；**运行环境是本地 anaconda base python（有 pxr），不是 Blender python**；make_usd 里 `stage.Save()` 是新建文件，安全（坑 2 管的是已有场景/源文件）。

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

## 场景组装脚本骨架（gen_labXXX_scene.py，完整可运行版）

完整示例见 `scripts/gen_lab004_scene.py`（D2 场景，REMOVE 6 个 prim + 5 个 REFS + 3 个 BUILTIN）。运行环境：本地 anaconda base python（有 pxr）。骨架如下，照抄改 3 个列表（① ② ③ 标记）：

```python
# -*- coding: utf-8 -*-
"""生成 lab_0XX.usd（基于 lab_001.usd 副本组装场景）。"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

SRC = r"E:/.../assets/chemistry_lab/lab_001/lab_001.usd"   # 基础场景（只当输入，严禁 Save）
OUT_DIR = r"E:/.../assets/chemistry_lab/lab_0XX"
ASSET = r"E:/.../assets/chemistry_lab"

REMOVE = ["/World/xxx_prim", ...]        # ① 要删除的 prim
REFS = [                                 # ② 资产引用 (name, 资产文件名, translate)
    ("TestTubeRack", "test_tube_rack.usd", (0.30, 0.08, 0.80)),
    ("TestTube", "test_tube.usd", (0.30, 0.08, 0.809)),    # 管底贴台面
]
BUILTIN = [                              # ③ 内建效果 prim（初始隐藏）
    # (name, kind, r, height, translate, color, opacity)
    #   kind="cylinder"：直壁容器（烧杯/试管）液柱，r < 内径
    #   kind="frustum" ：收口容器（锥形瓶/容量瓶）液体，r = 液面处内半径（坑 18）
    ("PowderOnSpoon", "cylinder", 0.004, 0.004, (0.305, 0.13, 0.80), (0.93, 0.93, 0.94), 1.0),
    ("TubeWater", "cylinder", 0.006, 0.035, (0.30, 0.08, 0.831), (0.55, 0.75, 0.95), 0.6),
]


def add_material(stage, prim, diffuse, opacity):
    """UsdPreviewSurface 材质（BUILTIN 标记用）。"""
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_frustum(stage, prim_path, r_bot, r_top, height, t, color, opacity):
    """截锥 Mesh（收口容器内液体，坑 18）：r_bot 底内半径、r_top 液面处内半径。
    UsdGeom.Cone 是尖锥（顶为一个点），不是截锥，不能用；Cylinder 是直壁。
    用 Mesh 手写：两个环 + 侧面四边形。"""
    import math
    seg = 32
    pts, idx = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        pts.append(Gf.Vec3f(r_bot * math.cos(a), r_bot * math.sin(a), 0.0))   # 底环 z=0
    for i in range(seg):
        a = 2 * math.pi * i / seg
        pts.append(Gf.Vec3f(r_top * math.cos(a), r_top * math.sin(a), height))  # 顶环
    for i in range(seg):
        j = (i + 1) % seg
        idx += [i, j, seg + j, seg + i]
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * seg)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    add_material(stage, mesh.GetPrim(), color, opacity)
    mesh.AddTranslateOp().Set(Gf.Vec3d(*t))
    UsdGeom.Imageable(mesh).MakeInvisible()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    stage = Usd.Stage.Open(SRC)                  # 打开副本；最后必须 Export 新路径
    root = stage.GetPrimAtPath("/World")
    assert root.IsValid(), "/World not found"

    for path in REMOVE:                          # 1) 删除不需要的 prim
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(Sdf.Path(path))

    for name, asset_file, t in REFS:             # 2) 引用资产 + translate（幂等）
        prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
        prim.GetPrim().GetReferences().AddReference(os.path.join(ASSET, asset_file))
        ops = prim.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(*t))             # 重复运行不报错
        else:
            prim.AddTranslateOp().Set(Gf.Vec3d(*t))

    for name, kind, r, h, t, color, opacity in BUILTIN:   # 3) 内建效果 prim
        prim_path = f"/World/{name}"
        if kind == "frustum":
            add_frustum(stage, prim_path, r, r, h, t, color, opacity)  # r_bot/r_top 按内腔调整
        else:
            geom = UsdGeom.Cylinder.Define(stage, prim_path)
            geom.CreateRadiusAttr(r)
            geom.CreateHeightAttr(h)
            geom.CreateAxisAttr("Z")
            geom.AddTranslateOp().Set(Gf.Vec3d(*t))
            add_material(stage, geom.GetPrim(), color, opacity)
            UsdGeom.Imageable(geom).MakeInvisible()   # 初始隐藏，事件触发后显示

    out_file = os.path.join(OUT_DIR, "lab_0XX.usd")
    stage.Export(out_file)                       # 禁止 Save()（坑 2）！
    print("SAVED", out_file)


if __name__ == "__main__":
    main()
```

要点：REFS 用 `AddReference(资产绝对路径)`（gen_lab004_scene.py 风格），不是 AddInternalReference；BUILTIN 初始 `MakeInvisible()`，由任务事件驱动显隐；液体形状必须匹配容器内腔（坑 18）——直壁容器圆柱（半径 < 内径），收口容器截锥（手写 Mesh，Cone 是尖锥不能用）。

---

## 场景配置与相机调参（YAML）

### YAML 配置文件结构

实验环境通过 `config/level2_<TaskName>.yaml` 配置（如 `level2_FlameTest.yaml`）。关键部分：

```yaml
usd_path: "assets/chemistry_lab/lab_flametest/lab_flametest_v17.usd"  # 场景文件（相对仓库根）

cameras:
  - prim_path: "/World/Camera1"
    name: "camera_1"
    translation: [0.75, 0.3, 1.35]     # [x, y, z] 世界坐标（米）
    resolution: [512, 512]              # 输出分辨率
    focal_length: 20                    # 焦距(mm)，越小视角越广
    orientation: [0.54151, 0.24438, 0.33089, 0.73318]  # 四元数(w,x,y,z)!
    image_type: "rgb"

  - prim_path: "/World/Camera2"
    name: "camera_2"
    translation: [0.25, 0, 1.5]
    resolution: [512, 512]
    focal_length: 12
    orientation: [0.70711, 0, 0, -0.70711]  # 俯视(绕X轴-90°)
    image_type: "rgb"

  - prim_path: "/World/Franka/panda_hand/arm_camera"
    name: "camera_3"                    # 机械臂腕部相机，无需设 translation/orientation
    resolution: [512, 512]
    image_type: "rgb"

robot:
  type: "franka"
  position: [-0.3, 0, 0.71]            # 机器人基座位置

task:
  max_steps: 30000                      # 最大步数（长任务如FlameTest需30000）
  obj_paths: []                         # 需随机化的物体路径（静态场景留空）
```

**相机参数在 YAML 里设，不是 USD 里**——改视角只需改 YAML 无需重建场景。

### 三个相机的角色

| 相机 | 角色 | 典型参数 | 备注 |
|---|---|---|---|
| Camera1 | 斜视特写，对准操作焦点（火焰/抓取点） | focal 14-25mm，靠近操作区 | 焦距越大画面越窄但细节越清晰 |
| Camera2 | 俯视全局，覆盖整个工作区 | focal 10-15mm，z 距桌面 0.5-1.0m | 焦距太小→大面积空白；太大→覆盖不全 |
| Camera3 | 机械臂腕部相机，跟随末端运动 | 仅设 resolution | 无需手动设 translation/orientation |

### FOV 与覆盖范围计算

```
FOV(水平) = 2 * atan(sensor_width / (2 * focal_length))    # sensor_width ≈ 36mm
地面覆盖宽度 = 2 * 相机高度 * tan(FOV / 2)
```

示例：
- focal=5mm → FOV≈158°，z=2.5m → 覆盖 ~7m（太大，90%空白）
- focal=12mm → FOV≈82°，z=1.5m → 覆盖 ~2.6m
- focal=12mm → FOV≈82°，z=1.5m → 覆盖 ~1.2m（对齐1m工作区，Camera2 推荐）

调参流程：先确定目标覆盖宽度 W（工作区尺寸）→ 算 z = W/(2*tan(FOV/2)) → 调 focal 使 FOV 合适 → 用 `--snapshot 2` 验证。

### 常见配置对比

| 参数 | 通用实验默认 | FlameTest 调整后 | 原因 |
|---|---|---|---|
| Camera2 z | 2.5 | 1.5 | 降高度缩小覆盖范围 |
| Camera2 focal | 5 | 12 | 增焦距收窄 FOV，对齐 1m 工作区 |
| Camera2 x | 0.1 | 0.25 | 对准工作区中心 |
| Camera1 focal | 5 | 14-20 | 特写需长焦距放大火焰/操作细节 |
| Camera1 resolution | 256 | 512 | 焰色反应需高分辨率辨色 |
| Camera1 translation | [2,0,2] | [0.75,0.3,1.35] | 靠近操作区获得特写 |

> 通用实验（TransportBeaker/HeatLiquid 等）Camera2 z=2.5 + focal=5 之所以能用，是因为那些实验物体随机化范围大（x[0.13,0.34] y[-0.33,0.25]），需要覆盖更大区域。FlameTest 工作区固定且小（1m×0.5m），需收紧相机参数。

### orientation 四元数

- 顺序是 **(w, x, y, z)**，不是 (x, y, z, w)！写反会导致相机朝向完全错误
- 俯视（Camera2）：`[0.70711, 0, 0, -0.70711]` = 绕 X 轴旋转 -90°（光轴朝下，看向 -Z）
- 斜视（Camera1）：需用四元数计算器或 Blender 辅助确定，让相机 -Z 轴指向目标点
- 斜视常用值：`[0.61237, 0.35355, 0.35355, 0.61237]`（绕 X 45° + 绕 Y 45°，从右前上方看向原点）

### 快速验证

```bash
# 导出 2 帧（Camera1 + Camera2 各一张），不跑完整实验
python main.py --config config/level2_FlameTest.yaml --snapshot 2
```

检查输出图片中：
- 操作区域居中，无大面积空白
- 关键物体清晰可辨（如火焰颜色、器材轮廓）
- 相机未穿过桌面/物体（穿模会导致画面异常）

---

## USD 资产引用架构（场景组装进阶）

### 引用 vs 内嵌

| 方式 | 语法 | 优点 | 缺点 |
|---|---|---|---|
| **内嵌几何** | 直接在场景 USD 里写 Mesh points/faces | 自包含，不依赖外部文件 | 改器材需逐场景修改，不可维护 |
| **引用资产** | `references = [@../asset.usd@</root>]` | 修改一处全局生效，资产可复用 | 需管理引用路径，单位/材质需对齐 |

**原则：所有器材都用引用，不内嵌。** 场景 USD 只负责布局（translate/scale/rotate），不存几何数据。

### 引用语法

```usda
def Xform "BunsenBurner"
{
    # 引用外部 USD 文件的 /root prim
    references = [@../bunsen_burner.usd@</root>]

    # 摆放位置（相对 /World）
    xformOp:translate = (0.36, 0.18, 0.8)
    xformOpOrder = ["xformOp:translate"]
}

def Xform "TestTubeRack"
{
    # 引用 + 缩放（源资产用 mm，场景用 m）
    references = [@test_tube_rack_detailed.usd@</TestTubeRack>]
    xformOp:translate = (0.38, -0.14, 0.8965)
    xformOp:scale = (0.001, 0.001, 0.001)
    xformOpOrder = ["xformOp:translate", "xformOp:scale"]
}

def Xform "PlatinumWire"
{
    # 引用 + 旋转
    references = [@../platinum_wire.usd@</root>]
    xformOp:translate = (0.368, -0.14, 0.895)
    xformOp:rotateXYZ = (0, 0, 90)
    xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
}
```

要点：
- `@路径@` 内是相对当前 USD 文件的路径（如 `../bunsen_burner.usd`），**禁止绝对路径**（坑 21）
- `</root>` 是被引用 USD 中的 prim 路径（不是文件路径），需先打开源资产确认顶层 prim 名
- 单位不匹配（源 mm，场景 m）时加 `xformOp:scale = (0.001, 0.001, 0.001)`
- `xformOpOrder` 必须列出所有 xformOp，顺序即变换顺序

### 引用资产的碰撞与材质继承

引用外部资产时，碰撞属性和材质绑定**随引用继承**：

- **碰撞**：源资产 USD 中 Mesh 有 PhysicsCollisionAPI + physics:collisionEnabled=True → 引用后碰撞自动生效。如果源资产没有碰撞，引用后也没有——需要在**源资产**里加，不是在场景引用 prim 上加
- **材质**：源资产 USD 中 Material + MaterialBinding → 引用后材质自动生效。如需覆盖材质（如换颜色），在场景 prim 上重新 `MaterialBindingAPI.Bind()` 会覆盖引用的材质
- **可见性**：源资产的 visibility 设为 invisible → 引用后也不可见。需在源资产或场景中设为 inherited

### 从内嵌转为引用的操作步骤

```python
from pxr import Usd, UsdGeom, Sdf, Gf
import os

ASSET_DIR = "E:/.../assets/chemistry_lab"

# 1. 删除内嵌的器材 prim
stage.RemovePrim(Sdf.Path("/World/BunsenBurner"))

# 2. 创建新 Xform 并添加引用
prim = UsdGeom.Xform.Define(stage, "/World/BunsenBurner")
prim.GetPrim().GetReferences().AddReference(
    os.path.join(ASSET_DIR, "bunsen_burner.usd"),  # 资产路径
    "/root"                                          # 被引用的 prim 路径
)
prim.AddTranslateOp().Set(Gf.Vec3d(0.36, 0.18, 0.8))

# 3. 验证引用已添加
assert stage.GetPrimAtPath("/World/BunsenBurner").HasAuthoredReferences()

# 4. 导出（禁止 Save，用 Export）
stage.Export(out_path)
```

### 引用架构的维护优势

修改器材只需改源资产 USD，所有引用该资产的场景自动更新：

```
assets/chemistry_lab/bunsen_burner.usd  ← 改这一个文件
    ↑ references
    ├── lab_flametest/lab_flametest_v17.usd  ← 自动更新
    ├── lab_005/lab_005.usd                   ← 自动更新
    └── ...其他场景                            ← 自动更新
```

用 Blender 打开整体场景 USD（如 lab_flametest_v17.usd）即可预览引用效果——Blender 会自动加载所有引用的外部 USD 文件，无需逐个文件检查。修改某个器材后，重新打开场景 USD 即可看到更新后的效果。

### 场景中表格可见性

场景 USD 中的桌子 prim（如 `/World/table`、`/World/lounge_booth_table`）默认 visibility 可能被设为 `invisible`，导致桌面不可见但物体悬空。修复：将 visibility 改为 `inherited`：

```python
from pxr import Usd, UsdGeom
table = stage.GetPrimAtPath("/World/table")
UsdGeom.Imageable(table).MakeVisible()  # 等价于 visibility = "inherited"
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
