---
name: labutopia-task
description: Builds LabUtopia task actions / 任务与元动作 (level2-level4 实验任务) — from the V7 任务文档 a big task IS one 元动作, split into phases (= one doc step each), implemented with the three-layer task + controller + config structure (kinematic grasping contracts, events_dt state machines, sticky phase flags). Use when the user asks to 制作元动作/原子动作/大动作/复合实验/新任务/level4 任务, or mentions LabUtopia task+controller+config, atomic_actions, events_dt, gripper contract, kinematic attach, dropper, scoop_controller, dissolve_task, phase machine, task_factory, level2_XXX.yaml. When the required 3D assets/scenes are missing, delegate to the labutopia-assets skill first.
version: 2.1.0
---

# LabUtopia 任务/元动作制作

为 LabUtopia 创建任务动作。**一个大任务 = V7 文档里的一个任务 = 一个元动作**（如 D3-S 酸性试剂滴加：取滴管 → 吸液 → 滴加）；**一个 phase = 文档里的一步**（步骤 5 取胶头滴管、步骤 8 逐滴加入……）。实现为三层结构：task（世界状态+判定）+ controller（编排/事件状态机）+ config（yaml）。

**核心思想：先理解步骤的物理语义，再写夹哪/移到哪。** 机械臂该抓哪个部位、该移动到哪个点，必须从"这一步为什么存在"推导（例：捏住胶头才能挤压排液、松开吸液；尖嘴不能碰瓶口外壁），不能凭感觉定——抓错位置是已踩过的坑（坑 18）。

## 工作流

1. **任务解析**：从 V7 文档选任务，按文档步骤拆 phase（一个 phase = 文档一步），标出每步的物理判定条件与视觉反馈（液滴显隐、粉末上勺）。判断动作类型（位置驱动 / 夹爪开合驱动 / velocity 驱动，见下）
2. **资产清点**：列出任务所需全部器材，对照 `lab_inventory.json`（项目根目录，notes 常带尺寸材质）与 `assets/chemistry_lab/*.usd`（场景类在 `lab_00x/`）核对。**任何缺失 → 先调用 labutopia-assets skill 生成**（实物调研通法），生成完再继续。grasp_distance、pickz_offset 等几何参数必须对照真实资产尺寸，资产不存在时这些值无法定
3. **场景布置**：按文档"场景布局提示"（如：胶头滴管在试管架前方、酸性试剂瓶在左前方 15cm 处）把器材摆到指定位置，用 labutopia-assets 的场景脚本模板生成 lab_00X.usd（REMOVE/REFS/BUILTIN 三部分）。**生成后必须检查 asset 属性无本地绝对路径**（坑 12：Stage.Open + Export 派生场景会把相对引用绝对化成 E:/ 路径，服务器红背景）。REFS 用相对路径（`../dropper.usd`），跨机器可解析
4. **逐 phase 实现**（先过抓取三问，再写三层）：
   - **抓取三问（必须）**：① 抓哪个部位？② 为什么是它（一句话物理语义）？③ 参数怎么从资产几何推导（读 USD extent 定"可抓部位区间"+ 余量）？三问答不上来不许写 forward。位置同理：移到哪个点、为什么、几何怎么定
   - **Task 层** `tasks/<name>_task.py`：参考点在 reset() 里从 prim 位置推导（偏移链，无硬编码）；每个物体一个 `_update_<obj>()` 子状态机（rest → attached → dwell → 显隐 → released → 归位）；**kinematic 跟随必须覆盖整个吸附期间**（坑 19）；多阶段动作必须提供粘性阶段标志（坑 13）
   - **Controller 层** `controllers/<name>_controller.py`：单动作 → 原子 controller（events_dt 事件状态机，见"事件状态机"）；多动作编排 → 复合 controller（Phase 枚举 + `_phase_action` + `_check_phase_success` + `_advance_phase`）
   - **Config 层** `config/levelX_<name>.yaml`：场景路径、prim 路径、offsets、thresholds、task_type/controller_type（factory 注册键）
5. **验证**：py_compile + 事件序列走查（stub cspace_controller 逐帧推进）→ 服务器冒烟（2 集，pkill 清场 + python -u，见 reference「部署与服务器冒烟」）→ **T 行核对**（抓取点实际 z 落在预期区间、物体位置全程跟随）。失败时先注入 debug 日志拿全数据，按判定三分法定位，不猜

## 三层架构

| 层 | 文件 | 职责 |
|---|---|---|
| Task | `tasks/<name>_task.py` | 世界状态、kinematic 吸附检测、隐藏 prim 显隐、阶段完成的物理判定 |
| Controller | `controllers/<name>_controller.py` | Phase 状态机，编排元动作（原子 controller）调用 |
| Config | `config/levelX_<name>.yaml` | 场景路径、prim 路径、offsets、thresholds、task_type/controller_type |

注册：`task_factory.register_task("<type>", Task)` + `controller_factory.register_controller("<type>", Controller)`，yaml 里 `task_type`/`controller_type` 对应 `<type>` 键。仓库惯例：复合 controller 直接 `from .atomic_actions.xxx_controller import XxxController`（scoop/cap/dip 均未进 `__init__.py` 照样用；`__init__.py` 只导出最常用 11 个，新动作注册可选）。

## 动作类型（先分类再写）

| 类型 | 语义 | 参考 | 关键点 |
|---|---|---|---|
| 位置驱动 | 夹爪按目标点移动，task 按位置/停留检测 | pick / dip / ignite / scoop | 平移 xy 跳转、到达 3D 跳转 |
| 夹爪开合驱动 | 夹爪距离本身是动作语义（抓/压/挤/放） | press / pressZ / dropper | 开合值必须对齐 task 检测阈值 |
| velocity 驱动 | 关节按速度转（倾倒） | pour | `switch_dof_control_mode(dof_index=N, mode="velocity")` |

模板与命名见 reference.md（动作类型表、统一签名、DropperController 14 事件模板、ScoopController 11 阶段模板、PourController、位置参数命名表）。

## 事件状态机（核心模式）

```python
if self._event < len(self._events_dt):
    self._t += self._events_dt[self._event]
    if self._t >= 1.0:
        self._event += 1
        self._t = 0
# is_done: self._event >= len(self._events_dt)
```

- **events_dt 长度必须严格等于事件数**（走查数一遍，长度校验抛 `f"events_dt length must be {N}, got {len(...)}"`）
- **dwell 换算（帧数 = 1/dt，每帧 `_t += dt`）**：0.004→250 帧、0.01→100 帧、0.02→50 帧（≈0.83s）、0.03→33 帧、0.04→25 帧、0.05→20 帧（≈0.33s，@60fps）。task 检测事件（闭爪/挤压/松开）dwell 用 0.02-0.05 均可行（cap 闭爪就用 0.05=20 帧），更稳妥用 0.02-0.03（50-33 帧）
- `ArticulationAction(joint_positions=[None] * n)` = 保持当前关节（dwell/完成事件都返回这个）
- **__init__ 只收 `(name, cspace_controller, events_dt=None, position_threshold=0.01)`**；所有几何/夹爪参数（pre_offset_z、lift_z、grasp_distance、squeeze_distance…）是 `forward()` 的参数，默认值写在 forward 签名里

### _start 第一帧模式（必须）

```python
self._start = True
# forward() 第一帧：
if self._start:
    return self._handle_start_state(current_joint_positions)  # 开夹爪 0.04/get_stage_units()
```

- 抓取类动作（pick/scoop/cap/dropper）：第一帧**打开**夹爪 0.04
- 工具已吸附类动作（dip 铂丝）：第一帧**保持闭合**（不设关节，直接 None）
- 例外：press/pressZ 是在 forward 内联处理 _start 的旧写法，新动作统一用 `_handle_start_state` 方法

### 单位换算（必须）

**所有夹爪关节值 = 米值 / get_stage_units()**：`target_joint_positions[7] = 0.04 / get_stage_units()`。漏掉换算关节值会错 100 倍（1m 单位场景）。`position_threshold` 标准是 0.01（米），不除 units（pressZ 除以 units 是特例，不要学）。

### 跳转规则（平移 vs 到达）

- **平移事件**（"移到某点上方/远处"）：只看 xy 平面距离 `np.linalg.norm(gripper_position[:2] - target[:2]) < threshold`，避免 z 误差卡死
- **到达事件**（"下探到某点/上提到某点"）：3D 距离 `np.linalg.norm(gripper_position - target) < threshold`
- **dwell / 夹爪开合事件**：保持目标，靠 _t 累积推进，不跳转

## 夹爪契约（必须遵守）

`joint_positions[7]/[8]` = 手指距离（米，小 = 闭合）。task 的 kinematic 检测只看 joint7：

| 检测 | joint7 阈值 | 说明 |
|---|---|---|
| 打开指令 | 0.04 / su | 所有动作第一帧/释放时用 |
| 吸附 | < 0.025 | 夹爪真实闭到此值以下 task 才 attached |
| 释放 | > 0.03 | task 判定物体脱离 |
| 挤压（胶头） | < 0.005 | 夹爪开合驱动类动作的"squeeze"状态（press 用 0.0015） |
| 松开吸液 | 0.005~0.025 之间 | 仍吸附但未挤压（dropper release 0.015） |

设计"夹爪开合即动作"（挤压/松开）时，**开合值必须落在 task 检测区间内**，否则 task 永远检测不到状态。grasp_distance 查表（pick_controller.get_gripper_distance）：pipette 0.008、tube 0.01、rod 0.003、Petri dish 0.005、microscope slide 0.002、beaker 0.022-0.03、conical_bottle 0.01-0.03、graduated_cylinder 0.005-0.03、Erlenmeyer flask 0.018；查不到默认 0.02。

## 关键模式

- **Kinematic 吸附**：task 检测 `gripper 靠近目标 AND joint7 < 0.025` → attached → 对象镜像 gripper delta 移动 → joint7 > 0.03 → released。**task 只做判定，移动由 controller 的元动作完成**。跟随必须覆盖 attached→…→released 全程（坑 19）
- **参考点偏移链**：所有目标点从物体位置 + 固定偏移推导（grasp、scoop、transfer、pour、shake），不允许硬编码
- **dwell + 显隐**：controller 停留期间 task 数帧后 reveal/hide 隐藏 prim，实现"粉末上勺/试管变色/液滴出现"视觉
- **粘性阶段标志**：多阶段动作的 task 提供 picked/filled/dropped 等标志（达成置位、reset 清零），复合 controller 读标志判定阶段成功，不读当前状态（坑 13）
- **阶段成功条件门控**：相位推进必须加显式状态条件（如 `water_added`），不能单靠位置距离（坑 14）
- **结束后归位**：`_return_to_origin` 把动过的物体传回初始 translate（不要用 `_settle_on_table`——会把物体放错高度）
- **IK 到位判定（verified IK + 3D 距离冻结）**：机械臂"乱动 / 固定偏移 / 夹不住"三症状的通用解法——解 IK 后跑 Lula FK 核对，FK 误差 >6mm 的解拒绝，且优先用当前关节作 warm start（近奇异抓点用固定 home 会偶发选到 FK 差 17cm 的坏分支、臂猛甩）；到位冻结用 **3D 距离 <1cm + 连续 3 帧**（勿用高度圆柱条件，会在目标上方 ~1.3cm 提前冻结 → 固定偏移 + 悬空合爪），冻结后保持关节不再追 IK（近奇异区同一 TCP 可被多个 IK 分支到达，追 IK 永不收敛）。详见 reference「机械臂 IK 控制与到位判定」
- **抓取平滑 + 近窗门禁**：物体不再"静止到闭合瞬间再 teleport"（闪现吸附）——夹爪开始合拢且进入近窗时 task `_ease_obj_world`（k≈0.18）逐帧平滑拉向持物位；`GRASP_NEAR_FRAMES`（3）连续近窗计数（非近窗 / 非最近物体 / 有物体附着即清零）防臂"路过"误抓。详见 reference「刚体落座与防抓取闪现」

## 常见坑（已踩过）

1. **__init__ 签名照抄旧文档**：几何参数是 forward() 参数，不是 __init__ 参数；__init__ 只有 4 个参数
2. **漏 get_stage_units() 换算**：夹爪关节值必须除以 stage units，否则错 100 倍
3. **漏 _start 第一帧**：第一帧必须开夹爪（抓取类），否则事件 0 会带夹爪旧状态开始
4. **events_dt 长度与事件数不匹配**：会 IndexError 或提前结束。写完后数一遍 + 长度校验（yaml 里的 events_dt 同样别写错）
5. **grasp_distance 与物体几何不匹配**：夹太松 joint7 下不到 0.025，kinematic 吸附永不触发。查 pick_controller.get_gripper_distance 表（见上）
6. **位置/参考点硬编码**：所有目标点必须来自 forward 参数，参考点全部从 prim 初始位置推导（场景改了位置也能工作），不能写死
7. **挤压/松开值不落在 task 检测区间**：squeeze 必须 < 0.005，release 必须在 0.005-0.025 之间
8. **dwell 太短**：task 每帧检测，dwell 0.02-0.05（50-20 帧）都可行，但 0.05 只有 20 帧是 cap 用过的下限，更稳妥用 0.02-0.03（50-33 帧）；换算记住帧数 = 1/dt
9. **资产缺失仍写动作**：动作所需器材没有 3D 资产就动手，grasp_distance/pickz_offset 只能瞎猜，场景也摆不出来。工作流第 2 步先查资产，缺了就调 labutopia-assets skill 生成
10. **Export 派生场景把相对引用绝对化**：`Stage.Open(源场景) + Export(新路径)` 生成测试场景时，源场景的相对引用（`SubUSDs/materials/...`、`SubUSDs/textures/...`）会被解析成本地绝对路径（`E:/浙江大学/...`）写入导出层 → 服务器加载贴图/MDL 全挂 → 视频红色背景。生成后必须检查 root layer 所有 asset 属性无本地绝对路径（pxr 遍历 primSpec.attributes，typeName=='asset'，把本地前缀替换为相对 `../`），再用 ExportToString 复查
11. **复合 controller 阶段判定读当前状态**：原子 controller 一次跑完整个事件序列（如 dropper 14 事件 = 抓取+吸液+滴加）时，is_done 后 task 状态机早已走到最终态（dropped），PICK 阶段查 `state=='attached'` 必误报失败——**动作全程正确仍报 pick failed 即此因**。多阶段动作的 task 必须提供**粘性阶段标志**（picked/filled/dropped，达成置位、reset 才清零），复合 controller 读标志判定阶段成功；这样失败归因也准（吸液没成功会报 fill failed 而不是 pick failed）
12. **pxr 26.8 AttributeSpec 无 timeSamples 属性**：遍历 time-sampled 值要用 `attr_spec.ListTimeSamples()` + `GetTimeSample(t)` + `SetTimeSample(t, v)`，直接 `.timeSamples.items()` 会 AttributeError（贴图路径通常只有 default 值，该分支可防御）
13. **冒烟前不清理残留进程 / python 输出不可见**：服务器可能有旧 main.py 进程残留（混写日志、2 集配置跑出 10 集、诊断被污染），冒烟前必须 `pkill -9 -f 'main[.]py'`（用 `main[.]py` 防匹配到 shell 自身）；重定向到文件时 print 被块缓冲看不到 → 必须 `python -u`
14. **阶段完成误触发**：dissolved 判定必须加前置门控（如 `water_added`），否则上一步停留位置离摇动点太近（0.067 < 阈值 0.15）会提前误判
15. **grasp 点用 bbox center**：bbox 中心 ≠ 手柄中心（偏移 12mm）。用 `物体位置 + [0,0,0.0025]` 之类精确偏移
16. **粉末堆超出工作区**：scoop 参考点要在工作区 [0.2,0.37]×[-0.1,0.2] 内，必要时调整资产摆放
17. **阈值跳转 vs dwell 冲突**：transfer 后 gripper 停在管口附近，若无门控会触发后续阶段判定——用显式状态条件隔离
18. **抓取部位错误（不看物理语义定抓点）**：dropper 曾抓玻璃管身中段（z=0.06）——真实持滴管是捏**胶头**（最上部，z=0.115-0.15，抓 z=0.13），只有捏住胶头才能挤压排液/松开吸液。教训：**每个 phase 动手前必须过"抓取三问"**（抓哪/为什么/几何怎么定），答案来自步骤的物理语义 + 资产 USD extent 区间，不是猜。冒烟 T 行必须核对抓取点实际 z 落在预期区间（判定全绿不代表抓对地方）
19. **kinematic 跟随只覆盖部分吸附期（物体悬空）**：dropper 跟随曾只在 attached 分支执行，进入 squeezed 后 set_position 不再调用 → 滴管在瓶口上方悬空冻结，但 TCP/joint7/状态机全对、任务判定"成功"（视觉错但判定过）。**任何 kinematic 附着物体，跟随必须提到状态机公共段，覆盖 attached/squeezed/filled/dropped 全程，released 才复位**；验证必须看物体实际坐标（debug T 行记录 dropper=(x,y,z)），只看判定全绿照样漏
20. **相机 orientation 四元数按 (x,y,z,w) 写 → 相机翻转全黑**：Isaac Sim 相机 `set_local_pose(orientation=..., camera_axes="usd")` 按 **(w,x,y,z)** 解析四元数（Gf.Quatf 约定）。用 look-at 数学算出 (x,y,z,w) 后必须重排为 (w,x,y,z) 再写 yaml，否则相机被翻转朝上/朝外 → 画面全黑且无任何报错。症状排查看法：camera 画面 mean=0 从第 0 帧开始。另：新相机位置/朝向必须"看得见目标"——相机高度要接近目标高度且视线对准（2m 高水平视看不到 1m 桌面上的灯口），位置也不能在物体包围盒内
21. **运行时改 UsdPreviewSurface 的 Shader input 不传导到渲染器**：点火时直接 `shader.GetInput('diffuseColor'/'emissiveColor').Set(...)`，USD 属性变了但 RTX 渲染器不刷新 → 火焰渲染成白色（判定全绿、visibility 生效、颜色无效）。**颜色变化必须用预制变体 + visibility 切换**：资产里预建 N 个同几何不同固定颜色材质的 prim（flame_outer_yellow/purple/…），全部初始隐藏，运行时只显示目标变体（visibility 已被证明即时生效）
22. **判定成功 ≠ 视觉可见（火焰 2cm 锥体在 256px 相机只有 1-2 像素）**：FlameTest 首次冒烟判定 2/2 全绿，但三相机零黄色像素——火焰锥体半径 2cm 高 7.6cm 在 256×256、focal 5mm 相机里只有 1-4 像素，且 camera_1 在 2m 高水平视完全看不到 1m 高的桌面。**验证视觉性必须数据驱动**：把物体世界坐标用相机外参（quat 注意 w,x,y,z）+内参投影到像素，采样该点邻域颜色确认符合预期（火焰区域实测 (241,227,140) 黄色），不能只看任务判定。物体太小就放大几何（火焰放大 2.5x → φ10cm×19cm），相机要拉近对准目标
23. **IK warm start 接受坏分支 → 机械臂乱动/到位错**：Lula `compute_inverse_kinematics(frame_name='right_gripper', warm_start=固定home)` 在近奇异抓点（match/cap/stopper 低 z）偶发选出"FK 位置摆到目标 17cm 外"的坏分支，臂朝错误方向猛甩后 force-done——表现为"抖动 + 夹不住"。修复：解后跑 Lula FK 核对，FK 误差 >6mm 拒绝；且优先用当前关节作 warm start（段间平滑、消除分支跳变），不行再 fallback 固定 home（flametest_controller `_solve_ik_verified`）
24. **到达判定用高度条件 → 固定偏移 + 悬空合爪**：v34b"进圈即冻"（cylinder z<1.5cm 且 xy<3cm）在垂直下探时会于目标上方 ~1.3cm 提前冻结——正是用户看到的"夹爪与器材固定偏移且夹不起来"（手指在空中合拢）。v43 改为 **3D 距离 <1cm 且连续 3 帧** 才冻结（IK 验证后 tool_center 距目标仅 ~3.4mm，1cm 阈值贴合目标，task attach 在正确位置触发）。**到位判定一律用 3D 距离 + 连续 N 帧，别用高度圆柱**
25. **真刚体凸包不建模口洞 → 落座顶飞**：瓶塞/灯帽转真刚体（RigidBodyAPI+CollisionAPI+convexDecomposition+contactOffset 0.002+restOffset -0.001，默认 kinematic）后，瓶口凸包碰撞不建模口洞（凸包鼓出盖住口），真动态落座把瓶塞顶飞（diag_rigid 实测 err=0.099，z 顶到 0.951）。修复：流程期保持 kinematic（teleport 不被物理覆盖），落座改"盖到位后 kinematic 锁住"折中 + task `_verify_settle` 读物理位姿判定（容差 0.025）。刚体无需回退——diag_rmp 证明贴刚体的 hcl_mouth 目标照样收敛
26. **抓取悬空合爪 + 闪现吸附**：物体静止到闭合瞬间再 teleport 到 gripper+HELD_OFFSET，RMP 2-8cm 误差下夹爪闭合时物体纹丝不动（悬空合爪）、随后一步跳进爪内（闪现吸附）。修复：task `_ease_obj_world`（k≈0.18 逐帧平滑拉向持物位，只在 near 时 ease 防合爪未遂拖走物体）+ `GRASP_NEAR_FRAMES=3` 连续近窗门禁（防臂路过误抓——P4 抓火柴路过灯帽曾误抓 cap）。diag_grasp 验证 max_jump=0.0054（旧码 ~0.03）
27. **RMP 到位慢是长期问题（先隔离归因，非刚体回归）**：完整运行大量 seg force-done（gripper 卡 dist 0.2-0.3 不收敛）时，先用 diag 脚本逐帧跟踪隔离归因再动代码。diag_rmp 5 目标结论：远目标 match 0.178 FAIL 但单调收敛（0.504→0.219）、acid_dip/flame/hcl_mouth 0.012 收敛、cap_grasp 0.098 FAIL；FAIL 的都是低 z+远 x/y 目标、单调缓慢收敛非卡死。"真刚体干扰 RMP"被否定（贴刚体的 hcl_mouth 收敛、远离刚体的 match 反而不收敛）。→ RMP 到位慢（2-8cm）是长期问题，与任务自身改动无关，**不要为它回退已验证的刚体/IK 改动**

## 检查清单

- [ ] 冒烟后做过视觉可见性验证：被操作/现象物体（火焰、颜色变化）世界坐标投影到相机像素，采样邻域颜色符合预期（判定全绿 ≠ 视觉可见，坑 22）
- [ ] 任务已从 V7 文档拆 phase（一个 phase = 文档一步），每步的物理语义已想清
- [ ] 每个 phase 过了抓取三问：抓哪个部位 / 为什么（语义）/ 参数怎么从资产几何定（USD extent 区间）
- [ ] 动作所需器材全部有 3D 资产（lab_inventory.json ↔ assets/chemistry_lab 核对过），缺失的已调 labutopia-assets 生成
- [ ] 场景 lab_00X.usd 已按文档布局提示布置，asset 属性无本地绝对路径（E:/ 残留 = 0，REFS 用相对路径）
- [ ] 动作已分类（位置/夹爪开合/velocity），选了对应模板
- [ ] 所有参考点从 prim 位置推导（无硬编码坐标）
- [ ] __init__ 只有 (name, cspace_controller, events_dt=None, position_threshold=0.01)
- [ ] 有 _start 第一帧（开夹爪 0.04/su 或保持闭合）
- [ ] 所有夹爪值除以 get_stage_units()
- [ ] events_dt 长度 = 事件数，且有长度校验
- [ ] 平移事件 xy 跳转、到达事件 3D 跳转
- [ ] 开合值对齐 task 检测区间（<0.025 吸附、>0.03 释放、<0.005 挤压）
- [ ] 每个 kinematic 附着的物体：跟随覆盖整个吸附期（attached→…→released），released 才复位
- [ ] 多阶段动作：task 有粘性阶段标志，复合 controller 读标志不用当前状态；阶段成功条件有显式门控
- [ ] 每个 _update_ 状态机有 rest→attached→…→released→归位完整回路
- [ ] task/controller 已在 factory 注册，yaml 的 task_type/controller_type 键匹配
- [ ] py_compile 通过 + stub 走查 0→N 全事件可达
- [ ] 服务器冒烟：pkill 清场 → python -u → 2 集 → 业务日志出现成功链（attached→filled→dropped/对应动作链）→ 进程退出 → h5 存在
- [ ] 冒烟后删 config_smoke；debug patch 还原（git checkout --）
- [ ] IK 解已 FK 验证（>6mm 拒绝）+ 优先当前关节 warm start（坑 23）
- [ ] 到位判定用 3D 距离 + 连续 N 帧，无高度圆柱提前冻结（坑 24）
- [ ] 真刚体落座用 kinematic 锁住 + _verify_settle 物理位姿判定（坑 25）
- [ ] 抓取用 _ease_obj_world 平滑 + 连续近窗门禁防路过误抓（坑 26）
- [ ] 冒烟大量 force-done 时已用 diag 脚本隔离归因（RMP 慢 vs 刚体 vs IK），未误回退（坑 27）

## 附加资源

- 动作类型详表、统一签名、DropperController/ScoopController/PourController 模板、夹爪契约速查、粘性标志代码、派生场景路径修复脚本、部署与冒烟、失败诊断三分法、DissolveTask 三层模板、yaml 结构 → [reference.md](reference.md)
- 机械臂 IK 控制与到位判定（verified IK / 3D 距离冻结 / RMP 归因）、刚体落座与防抓取闪现（kinematic 锁住 / _ease_obj_world / 近窗门禁）→ [reference.md](reference.md)
