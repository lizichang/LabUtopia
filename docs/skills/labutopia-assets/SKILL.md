---
name: labutopia-assets
description: Creates LabUtopia (Isaac Sim chemistry simulator) 3D assets and scenes — test tubes, racks, spoons, wash bottles, powder piles — via MeshBuilder → OBJ → USD pipeline, and assembles lab_XXX scenes with references, builtin prims and kinematic markers. Use when the user asks to 制作资产/创建实验器材/生成场景, or mentions labutopia assets, USD asset creation, scene assembly, gen_*_assets.py, obj2usd, lab_00X scenes.
version: 1.0.0
---

# LabUtopia 资产制作

为 LabUtopia 实验创建几何资产（试管、试管架、勺子、洗瓶、样品瓶、粉末堆等）和组装场景（lab_00X.usd）。

## 工作流

1. **需求分析**：确定资产清单、摆放位置、与其他物体的几何约束（如勺子头必须 < 试管内径）
2. **几何构建**：仿照 `gen_dissolve_assets.py` 用 USD MeshBuilder 构建，导出 OBJ
3. **转 USD**：用 `obj2usd.py`（**必须用修复版**，见坑 1）生成带材质的 .usd
4. **场景组装**：写 `gen_labXXX_scene.py`（REMOVE / REFS / BUILTIN 三部分，见 reference.md）
5. **本地验证**：用本地 pxr 模拟验证 bbox / geometry_center / 参考点，全部在工作区内
6. **交付**：输出覆盖清单 + 测试指令

## 关键约束

- 所有生成/验证脚本写在 workspace 暂存区，**用 Bash cp 复制到仓库**（Write 工具不能直接写 LabUtopia）
- 场景保存必须 `stage.Export(新路径)`，**禁止 `stage.Save()`**（会污染源文件，见坑 2）
- 材质：玻璃 opacity≈0.35，金属 metallic≈0.85（UsdPreviewSurface）
- 资产文件放 `assets/chemistry_lab/`，场景放 `assets/chemistry_lab/lab_XXX/lab_XXX.usd`

## 常见坑（已踩过）

1. **obj2usd.py write_mesh 顶点索引 bug**：直接写全局顶点全集会让每个 prim 拥有相同 bbox/geometry_center。必须把顶点/法线重映射为局部索引（`used/remap/local_verts`），否则 `get_geometry_center` 的抓取点全错。
2. **stage.Save() 污染源文件**：Save 会写回 USD 源文件路径。用 `stage.Export(out_path)` 保存副本；若已污染，从服务器恢复原文件（校验大小/MD5）。
3. **Gf.Matrix4d 是行主序**：`np.array(xf)` 后 translate 在 `m[3,:3]`（不是 `m[:3,3]`），否则坐标全变零。
4. **勺子头宽度必须 < 试管内径**：D2 中勺头 20mm 宽卡不进 12.6mm 内径，改为 ellipsoid ay=0.006（12mm）。
5. **幂等 AddTranslateOp**：重复运行场景脚本会报 op 已存在。用 `ops[0].Set(...) if ops else prim.AddTranslateOp().Set(...)`。
6. **粉末堆不能超出工作区**：摆放时计算 scoop 参考点，确保 x/y 落在工作区 [0.2,0.37]×[-0.1,0.2] 内。
7. **隐藏标记 prim 的颜色/透明度**：BUILTIN 圆柱（如 TubeWater）用半透明浅蓝 (0.55,0.75,0.95) opacity 0.6，方便视觉区分。

## 检查清单

- [ ] 所有资产单独打开验证 bbox 与预期尺寸一致
- [ ] 场景中每个 REFS prim 有正确的 translate（幂等写法）
- [ ] lab_XXX.usd 引用的资产路径存在且大小正常
- [ ] 参考点全部在工作区内，无互相穿透
- [ ] lab_001.usd 等源文件未被污染（MD5/大小与服务器一致）
- [ ] 生成脚本可重复运行（幂等）

## 附加资源

- 详细代码模板（helpers 签名、write_mesh 修复版、场景脚本骨架）见 [reference.md](reference.md)
- 组合参考几何参数表（试管/试管架/勺子/洗瓶尺寸）见 [reference.md](reference.md)
