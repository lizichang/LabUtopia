---
name: labutopia-assets
description: Creates LabUtopia (Isaac Sim chemistry simulator) 3D assets and scenes — test tubes, racks, spoons, wash bottles, beakers, powder piles — via 实物调研通法(inventory优先+结构拆解三步法+形状原语映射) → MeshBuilder → OBJ → USD pipeline, or the Blender bpy photorealistic pipeline (bmesh lathe + 基本体 helpers + Principled BSDF + EEVEE render check + USD export), and assembles lab_XXX scenes with references, builtin prims and kinematic markers. Use when the user asks to 制作资产/创建实验器材/生成场景, or mentions labutopia assets, USD asset creation, Blender bpy 资产, scene assembly, gen_*_assets.py, obj2usd, blender_asset_template, post_fix_usd, lab_00X scenes.
version: 1.3.0
---

# LabUtopia 资产制作

为 LabUtopia 实验创建几何资产（试管、试管架、勺子、洗瓶、烧杯、粉末堆等）和组装场景（lab_00X.usd）。

**核心思想：不背尺寸，只背方法。新器材永远会出现，这套流程必须能套在任意器材上——先调研（通法）再建模。**

## 工作流

1. **实物调研**（每个新资产必做，产出"结构规格表"；通法见 reference.md「实物调研与结构拆解通法」）：
   - ① 先读项目库存 `E:\浙江大学\星辰计划\LabVLA_第一期轮转\lab_inventory.json`——equipment 的 name/material/notes 常自带尺寸材质（例："玻璃棒 长约20cm 直径6mm"、"试管 外径15mm 长125mm"）。这是仿真要复现的实物，**优先于任何网上数据**
   - ② 缺的尺寸再按信息优先级补：厂家规格表 > 国标 > 3D 模型库 > 实物测量 > 估算（标注"估"）。**严禁凭记忆写尺寸**（坑 16）
   - ③ 拆结构（三步法）：找主轴 → 拆部件（每部件：形状原语/关键尺寸/相对位置）→ 定连接（套入/贴合/独立）
   - ④ 定材质：每部件材质类别（玻璃/金属/陶瓷/塑料）→ 参数（材质参数表见 reference.md）
2. **需求分析**：确定资产清单、摆放位置、与其他物体的几何约束（如勺子头必须 < 试管内径）
3. **管线选择**（两条路，先问清楚用户要什么效果）：
   - **简单资产**（试管、粉末堆、隐藏标记、内部不展示的零件）→ 仿照 `scripts/gen_dissolve_assets.py` 用 USD MeshBuilder 构建 → `scripts/obj2usd.py` 转 USD。快，但外观是"数学几何体拼接"，精细度有限
   - **逼真资产**（用户要求"像克隆资产那样"精细外观的器材，如烧杯/量筒/酒精灯/玻璃器皿）→ **Blender bpy 精模管线**：`scripts/blender_asset_template.py`（bmesh 旋转体 + 基本体 helpers + Principled BSDF 材质 + EEVEE 渲染验证 + USD 导出）→ `scripts/post_fix_usd.py`（pxr 后处理补 transmission/opacity）。详见 reference.md「Blender 精模管线」
4. **建模**：按结构规格表逐部件实现——旋转对称件用 lathe（PROFILE 剖面），非旋转体件用基本体 helpers（cylinder/box/sphere），部件按相对位置摆放（套入/贴合/独立），不要边建边想
5. **渲染验证**：Blender 管线出 EEVEE 渲染 PNG，**与调研时存的实物图对照**形状/比例/部件齐全度；MeshBuilder 管线用本地 pxr 验证 bbox/geometry_center/参考点
6. **导出+后处理**：清相机灯光残留、post_fix_usd.py 补 transmission（玻璃）/opacity（半透明）
7. **场景组装**：写 `scripts/gen_lab004_scene.py` 风格的场景脚本（REMOVE / REFS / BUILTIN 三部分，见 reference.md）
8. **交付**：输出覆盖清单 + 测试指令

## 关键约束

- 所有生成/验证脚本写在 workspace 暂存区，**用 Bash cp 复制到仓库**（Write 工具不能直接写 LabUtopia）
- 模板脚本（`scripts/gen_dissolve_assets.py`、`scripts/obj2usd.py`、`scripts/gen_lab004_scene.py`、`scripts/blender_asset_template.py`、`scripts/post_fix_usd.py`）随仓库 git 管理；若仓库缺失（如未 pull），先从 git 历史恢复（`git log --oneline -- scripts/` 找最近版本），不要自行重写
- 场景保存必须 `stage.Export(新路径)`，**禁止 `stage.Save()`**（会污染源文件，见坑 2）
- 材质规范（两套管线统一口径）：玻璃 → Principled BSDF transmission=1.0 + ior=1.45 + roughness≈0.05（USD 里后处理补 transmission，见坑 12）；金属 → metallic≈0.85 roughness≈0.3；陶瓷/塑料 → metallic=0、roughness≈0.4-0.6；纸/棉 → roughness≈0.9
- 资产文件放 `assets/chemistry_lab/`，场景放 `assets/chemistry_lab/lab_XXX/lab_XXX.usd`

## 常见坑（已踩过）

1. **obj2usd.py write_mesh 顶点索引 bug**：直接写全局顶点全集会让每个 prim 拥有相同 bbox/geometry_center。必须把顶点/法线重映射为局部索引（`used/remap/local_verts`），否则 `get_geometry_center` 的抓取点全错。
2. **stage.Save() 污染源文件**：Save 会写回 USD 源文件路径。用 `stage.Export(out_path)` 保存副本；若已污染，从服务器恢复原文件（校验大小/MD5）。
3. **Gf.Matrix4d 是行主序**：`np.array(xf)` 后 translate 在 `m[3,:3]`（不是 `m[:3,3]`），否则坐标全变零。
4. **勺子头宽度必须 < 试管内径**：D2 中勺头 20mm 宽卡不进 12.6mm 内径，改为 ellipsoid ay=0.006（12mm）。
5. **幂等 AddTranslateOp**：重复运行场景脚本会报 op 已存在。用 `ops[0].Set(...) if ops else prim.AddTranslateOp().Set(...)`。
6. **粉末堆不能超出工作区**：摆放时计算 scoop 参考点，确保 x/y 落在工作区 [0.2,0.37]×[-0.1,0.2] 内。
7. **隐藏标记 prim 的颜色/透明度**：BUILTIN 圆柱（如 TubeWater）用半透明浅蓝 (0.55,0.75,0.95) opacity 0.6，方便视觉区分。
8. **obj2usd.py 单位坑**：`Usd.Stage.CreateNew` 默认 metersPerUnit=0.01（厘米）。不设米单位的话，Blender 打开资产时整体缩到 1%（root scale=0.01，6.5cm 勺子变 0.65mm）肉眼看"空场景"。修复版已加 `UsdGeom.SetStageMetersPerUnit(stage, 1.0)`，勿回退；新写转换脚本必须显式设米单位。生成后检查：pxr 读 `UsdGeom.GetStageMetersPerUnit(stage)` 应为 1.0。
9. **Blender 中文界面节点名被翻译**：材质节点/世界节点名显示为"原理化 BSDF"/"背景"，直接按名字查找会 KeyError。用 `next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')` / `'BACKGROUND'` 按类型查找。
10. **Blender 5.0 渲染引擎枚举**：只有 `'BLENDER_EEVEE'`（EEVEE Next 已并入），不存在 `'BLENDER_EEVEE_NEXT'`，写错直接 TypeError。
11. **EEVEE 玻璃渲染全黑**：transmission 材质没有任何环境光时图像全黑（不是建模错了）。必须给场景加 World 背景（Background 节点浅色 + Strength 1.0）；EEVEE Next 里玻璃透射还要 `bpy.context.scene.eevee.use_raytracing = True`，否则玻璃显示为不透明灰。
12. **Blender USD 导出丢 transmission**：导出器只映射 diffuseColor/metallic/roughness/ior/specular/clearcoat，**Transmission Weight 不导出**（玻璃导出后 opacity=1.0、无 transmission，进 Isaac 是不透明灰）。必须用 `scripts/post_fix_usd.py` 后处理补 `inputs:transmission = 1.0`（半透明再补 opacity<1）。
13. **导出混入相机/灯光/世界**：USD 里出现 `/root/cam`、`/root/key`、`/root/env_light`。导出前必须删除相机和灯光对象（`bpy.data.objects.remove(ob, do_unlink=True)`）并摘掉世界背景（`bpy.context.scene.world = None`，否则生成 DomeLight）。
14. **相机对准**：手工 Euler 角度很难让物体居中。用 `cam.rotation_euler = (target - cam.location).to_track_quat('-Z', 'Y').to_euler()` 让相机 -Z 轴精确指向目标点。
15. **克隆资产(alcohol_lamp)材质其实全默认**：Blender 导出 + 默认参数（0.8 灰、roughness=1.0、ior=1.5）→ 在 Isaac 里也是"不透明灰玻璃"。它"逼真"在几何（Blender 建模的嵌套分件），不在材质。我们主动设 Principled BSDF 参数 + 后处理补 transmission，材质反而比克隆资产精细。
16. **尺寸凭记忆 = 变形**：同容量器材不同厂家系列尺寸不同。实测：泰坦低型 50mL 烧杯口外径 41mm×高 65mm×壁厚 2mm（高≈直径 1.4 倍），另一常见系列是 40×58mm。建模前必须查规格表（厂家"产品尺寸一览表"/国标），把调研结果写进结构规格表再动手，禁止凭记忆写 PROFILE。
17. **Blender 5.0 Principled BSDF 输入改名**：4.x 起 `Specular` 已改名为 `Specular IOR Level`，直接 `bsdf.inputs["Specular"]` 会 KeyError。模板 set_mat 已用兼容检测（`if "Specular" in bsdf.inputs ... elif "Specular IOR Level" in ...`），新写材质代码照抄，勿直接用旧名。
18. **液体/标记 BUILTIN 形状必须匹配容器内腔**：烧杯/试管等直壁容器内腔是圆柱，BUILTIN 液柱用圆柱（半径 < 内径）没问题；但锥形瓶/容量瓶等**收口容器内腔是锥形**（上窄下宽），直液柱会悬空穿模（液面不贴壁、露出空隙）。此类容器液体须用截锥/锥台形状（下半径 = 底部内半径，上半径 = 液面高度处的内半径，高 = 液面高度），或把液面压在锥面以下（低于收口起始高度）。判断依据：内腔轮廓是直上直下还是逐渐收窄。
19. **post_fix_usd.py 原地 `stage.Save()` 是预期行为**：该脚本处理的是资产副本本身（补 transmission 就是要改这个文件），与坑 2 的"禁止 Save"不冲突——坑 2 禁止的是把**源场景/被引用资产**（lab_001/lab_003.usd 等）当输入或让 Save 污染它们。区分方法：当前文件是本次工作新产出的资产/副本 → 可原地 Save；是已有场景或被引用的源文件 → 只能 Export 新路径。

## 检查清单

- [ ] 已读 lab_inventory.json 提取该器材的已知尺寸/材质（优先于网上数据）
- [ ] 结构规格表已填：查过规格（非记忆尺寸，估算项标"估"）、部件+相对位置齐全、材质已定
- [ ] 结构规格表覆盖全部部件（形状原语 + 相对位置），非旋转体件（镊子/试管夹等）用基本体 helper 而非硬啃旋转体
- [ ] 所有资产单独打开验证 bbox 与规格表尺寸一致
- [ ] 场景中每个 REFS prim 有正确的 translate（幂等写法）
- [ ] lab_XXX.usd 引用的资产路径存在且大小正常
- [ ] 参考点全部在工作区内，无互相穿透
- [ ] lab_001.usd 等源文件未被污染（MD5/大小与服务器一致）
- [ ] 生成脚本可重复运行（幂等）
- [ ] Blender 管线：渲染 PNG 非全黑、与实物图对照形状/比例正确（有世界背景 + use_raytracing）
- [ ] Blender 管线：渲染图中物体完整入画、未被裁切（相机距离按物体高度自适应，见 reference.md「相机取景」）
- [ ] 场景 BUILTIN 液体/标记形状匹配容器内腔（直壁容器=圆柱；锥形瓶/容量瓶等收口容器=截锥，不悬空穿模）
- [ ] Blender 管线：导出 USD 无 cam/key/env_light 残留，metersPerUnit == 1.0
- [ ] Blender 管线：玻璃 transmission 已后处理（pxr 读 inputs:transmission == 1.0）

## 附加资源

- 详细代码模板（实物调研方法、helpers 签名、write_mesh 修复版、场景脚本骨架、Blender 精模管线模板、材质映射表、规格表样例）见 [reference.md](reference.md)
- 组合参考几何参数表（试管/试管架/勺子/洗瓶尺寸）见 [reference.md](reference.md)
