---
name: labutopia-compound-task
description: Builds LabUtopia compound experiments (level2-level4 复合实验任务) — a three-layer structure of task (world state + kinematic events), controller (phase state machine) and YAML config, e.g. the D2 dissolve test. Use when the user asks to 制作大动作/复合实验/level4 任务, or mentions LabUtopia task+controller+config, dissolve_task, phase machine, task_factory, level2_XXX.yaml.
version: 1.0.0
---

# LabUtopia 复合大动作制作

把多个元动作编排成完整实验（如 D2 蒸馏水溶解性测试：舀取 → 注水 → 摇动 → 观察），三层架构：task + controller + config。

## 三层架构

| 层 | 文件 | 职责 |
|---|---|---|
| Task | `tasks/<name>_task.py` | 世界状态、kinematic 吸附检测、隐藏 prim 显隐、阶段完成的物理判定 |
| Controller | `controllers/<name>_controller.py` | Phase 状态机，编排元动作（原子 controller）调用 |
| Config | `config/levelX_<name>.yaml` | 场景路径、prim 路径、offsets、thresholds、task_type/controller_type |

注册：`task_factory.register_task("<type>", Task)` + `controller_factory.register_controller("<type>", Controller)`，yaml 里 `task_type`/`controller_type` 对应 `<type>` 键。

## 工作流

1. **需求分析**：分解实验步骤（元动作序列）、每步的物理判定条件、视觉反馈（粉末显隐、水变色）
2. **场景与资产**：若缺资产/场景，先走 labutopia-assets 流程
3. **参考点计算**：在 task 的 reset() 里从 prim 位置推导全部参考点（偏移链见 reference.md）
4. **Task 状态机**：每个物体一个 `_update_<obj>()` 子状态机（rest → attached → dwell → 显隐 → released → 归位）
5. **Controller Phase 状态机**：Phase 枚举 + `_phase_action` + `_check_phase_success` + `_advance_phase`
6. **yaml 配置**：复制现有 level2_*.yaml 改路径/prim/阈值
7. **注册 + 验证**：factory 注册、py_compile、参考点 pxr 模拟验证

## 关键模式

- **Kinematic 吸附**：task 检测 `gripper 靠近目标 AND joint7 < 0.025` → attached → 对象镜像 gripper delta 移动 → joint7 > 0.03 → released。**task 只做判定，移动由 controller 的元动作完成**
- **参考点偏移链**：所有目标点从物体位置 + 固定偏移推导（grasp、scoop、transfer、pour、shake），不允许硬编码
- **dwell + 显隐**：controller 停留期间 task 数帧后 reveal/hide 隐藏 prim，实现"粉末上勺/试管变色"视觉
- **相位成功条件**：上一个 Phase 的 object state == 'released' 才推进
- **结束后归位**：`_return_to_origin` 把动过的物体传回初始 translate（不要用 _settle_on_table——会把物体放错高度）

## 常见坑（已踩过）

1. **阶段完成误触发**：dissolved 判定必须加前置门控（如 `water_added`），否则上一步停留位置离摇动点太近（0.067 < 阈值 0.15）会提前误判。
2. **参考点硬编码**：全部从 prim 初始位置推导，场景改了位置也能工作。
3. **grasp 点用 bbox center**：bbox 中心 ≠ 手柄中心（偏移 12mm）。用 `物体位置 + [0,0,0.0025]` 之类精确偏移。
4. **粉末堆超出工作区**：scoop 参考点要在工作区 [0.2,0.37]×[-0.1,0.2] 内，必要时调整资产摆放。
5. **阈值跳转 vs dwell 冲突**：transfer 后 gripper 停在管口附近，若无门控会触发后续阶段判定——用显式状态条件隔离。
6. **yaml 的 events_dt 别写错**：ScoopController 11 事件必须 11 项，controller 初始化会校验。

## 检查清单

- [ ] 所有参考点从 prim 位置推导（无硬编码坐标）
- [ ] 每个 _update_ 状态机有 rest→attached→…→released→归位完整回路
- [ ] 阶段成功条件有显式门控（不靠位置距离单判）
- [ ] task/controller 已在 factory 注册，yaml 的 task_type/controller_type 键匹配
- [ ] 参考点 pxr 模拟验证通过（在工作区内、无穿透）
- [ ] py_compile 全部通过；yaml 可解析
- [ ] 测试指令：`python main.py --config-name levelX_<name>`

## 附加资源

- DissolveTask 状态机模板、参考点计算链、Phase 枚举模板、yaml 结构见 [reference.md](reference.md)
