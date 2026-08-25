---
name: labutopia-scene-realign
description: 用户手调 tmp 场景器材位置/旋转后，按 tmp 改动更新生成脚本（gen_XXX_scene.py）的固定流程，适用于任意实验场景（D2S/D3L/焰色/B2/D4L 等）的坐标/布局重排 — 读 tmp 世界布局 → 反推变换模型（绕轴旋转+平移）→ 改脚本（同步清单）→ 重生成+bbox 逐项核对 → 收尾。Use when the user says 我调整了/移动了/旋转了/挪了/重新摆了/改了下器材位置或布局，你根据我的 tmp 改动改一下, or mentions tmp.usd 布局, 整组绕Z旋转, 器材重排, scene re-align, dump_scene_layout, validate_scene_layout, XformOp 顺序, 重新生成场景坐标. 当需要新增资产/重建几何时先调 labutopia-assets skill。
version: 1.0.0
---

# LabUtopia 场景重排（用户改 tmp 位置 → 更新生成脚本）

用户在 Isaac/UE 里手搭好 tmp 场景（改动了器材位置/旋转），要求生成脚本同步。**核心思想：先读 tmp 的"真相"，再反推变换模型，再同步改脚本——绝不让生成脚本与 tmp 漂移。**

## 工作流

1. **读 tmp 世界布局**（真相来源）
   - `python3 scripts/dump_scene_layout.py <tmp.usd> <顶层prim路径>` 列出每个器材的 xform ops + 世界 bbox
   - 记录每个**动过**的器材：世界 translate、旋转、bbox；**没动**的也标出来（排除项）

2. **对比生成脚本当前值**
   - 读 gen 脚本的常量/EQUIP，列当前世界坐标，与 dump 结果对比 → 得出"动了哪些、怎么动"

3. **反推变换模型**
   - 常见模式 = **整组绕某轴旋转（常 Z180°）+ 平移**：`new_pos = R(θ)·old_local_offset + new_anchor`
   - 把 dump 的 bbox 抄进 layout.json，跑 `python3 scripts/validate_scene_layout.py layout.json`
   - **两种 op 顺序都试**，与 tmp bbox 吻合的那个就是 gen 该用的顺序（**坑 R1：`AddTranslateOp` 先 + `AddRotateXYZOp(0,0,180)` 后 → 净效果=先绕局部原点旋转再平移；反过来 [R,T] 得到错位置**）
   - 逐器材 bbox 全吻合才算模型成立，不吻合先别改脚本

4. **改 gen 脚本**（同步清单，漏一项就会漂移）
   - 常量：锚点坐标（如 STAND_X/Y）、派生坐标（如 TUBE_X = STAND_X − offset）
   - EQUIP 每项加旋转标志（哪些件跟着转、哪些不动）；`add_equip` 的 op 顺序
   - **依赖坐标的派生值**（最易漏）：
     - 效果 prim 基准（气泡/蒸汽/液柱相对试管中心，用 `TUBE_X` 派生而非硬编码旧坐标）
     - 被移动子件的目标点（**坑 R2：rot 会取反子件局部偏移**，如灯帽 tgt −0.467 → +0.467）
     - verify() 断言（世界 bbox 检查，随新坐标更新）
   - task/controller：grep 确认无布局坐标硬编码（一般按 prim 名定位不用改，**先 grep 确认**）

5. **重新生成 + 全量核对**
   - 重跑 gen，verify 断言全绿
   - `dump_scene_layout.py` dump 生成场景，与 tmp **逐项 bbox 对比**，全部吻合
   - **坑 R3：若某资产 default 时间 bbox 只看到部分几何**（如温度计只见挂环、不见杆身）→ 该资产网格属性是"时间采样在 0.0、default 空"（`attr.GetTimeSamples()` 非空且 `attr.Get()` 为空），用 `attr.Clear()+attr.Set(值@time0)` 转成干净 default（B2 温度计 450 attrs 一次修好）→ **坑 R4：改完资产必须重跑 gen 才烘进场景**（Export 烘平自包含）

6. **收尾同步**
   - config yaml 注释里的布局坐标更新
   - 记忆更新（场景布局记忆 + MEMORY.md 索引行）
   - **用户重排位置常是为了机械臂可达**（离臂远一点避开盲区/留操作空间、操作面朝臂）——同步布局时核对：最近器材在底座盲区外、最远器材 ≤ Franka 0.855m 臂展、被操作部位（钩/管口/夹）朝臂；工作区整体移动大时 `config robot.position` 可能要跟（底座向后=-Y）。布局设计的前瞻约束见 labutopia-assets 坑 34
   - **相机是共享常量不擅自改**：堆叠移动后视线可能偏，flag 给用户目检
   - 给用户运行指令亲自验证（verification-by-user 约定：Claude 不跑 Isaac Sim/截图）

## 关键约束

- tmp 是真相，gen 脚本必须重现 tmp 世界坐标——以 bbox 为准，不看中间值
- 有旋转必有 op 顺序问题；先 validate 再改脚本，别猜
- 改资产几何（时间采样修复）后必重跑 gen（烘平 Export 才有干净副本）
- 效果 prim 一律相对基准派生，禁止把旧布局坐标硬编码进新代码

## 快速检查

- [ ] dump 的 tmp bbox 与 validate PASS 的 op 顺序一致
- [ ] EQUIP 旋转标志覆盖所有该转的件
- [ ] 子件目标点已按旋转取反局部偏移
- [ ] verify 断言 + 生成场景 bbox 与 tmp 逐项吻合
- [ ] config 注释坐标已更新；相机待用户目检
- [ ] 已给用户运行指令

工具与坑详解见 reference.md。相关 [[b2-alcohol-heat-scene-layout]] [[labutopia-skills-registration]] [[pxr-matrix4d-row-vector-convention]]
