---
name: labutopia-atomic-action
description: Creates LabUtopia atomic actions / 元动作 (motion controllers) — scooping, shaking, picking, uncapping, pouring, dropper dripping — using the events_dt state machine pattern with kinematic grasping contracts. Use when the user asks to 制作元动作/原子动作/新动作, or mentions LabUtopia controllers, atomic_actions, scoop_controller, shake_controller, events_dt, gripper contract, kinematic attach, dropper. When the required 3D assets are missing, delegate to the labutopia-assets skill first.
version: 1.2.1
---

# LabUtopia 元动作制作

为 LabUtopia 创建可复用的原子动作 controller（勺子舀取、摇动、拾取、开盖、倾倒、滴管滴加等），放在 `controllers/atomic_actions/`。

## 工作流

1. **动作分解**：把动作拆成事件序列（移动 → 停留 → 闭合夹爪 → 抬升 → 移动 → …），先确定"哪些事件靠位置跳转、哪些靠夹爪开合、哪些需要 task 配合检测"，同时列出该动作所需的所有器材/工具
2. **资产检查**：逐一核对动作所需器材是否已有 3D 资产——对照项目根目录 `lab_inventory.json` 与 `assets/chemistry_lab/*.usd`（场景类在 `assets/chemistry_lab/lab_00x/`）。**任何器材缺失 → 先调用 labutopia-assets skill 按其实物调研通法生成资产**，生成完成后再继续本流程。grasp_distance、pickz_offset 等几何参数必须对照真实资产尺寸（lab_inventory notes 常带尺寸），资产不存在时这些值无法定
3. **选参考实现**：见 reference.md 动作类型表——先判断新动作属于哪一类（位置驱动 / 夹爪开合驱动 / velocity 驱动），再选对应模板
4. **实现事件状态机**：`events_dt` 驱动，逐事件推进（见下）
5. **注册（可选）**：仓库惯例是复合任务里直接 `from .atomic_actions.xxx_controller import XxxController`（scoop/cap/dip 等 6 个都没进 `__init__.py` 照样用）。`__init__.py` 只导出最常用 11 个，新动作不导也可以
6. **验证**：py_compile + 事件序列走查（stub cspace_controller 逐帧推进）+ 与 task 的 kinematic 检测对齐

## 动作类型（先分类再写）

| 类型 | 语义 | 参考 | 关键点 |
|---|---|---|---|
| 位置驱动 | 夹爪按目标点移动，task 按位置/停留检测 | pick / dip / ignite / scoop | 平移 xy 跳转、到达 3D 跳转 |
| 夹爪开合驱动 | 夹爪距离本身是动作语义（抓/压/挤/放） | press / pressZ / dropper | 开合值必须对齐 task 检测阈值 |
| velocity 驱动 | 关节按速度转（倾倒） | pour | `switch_dof_control_mode(dof_index=N, mode="velocity")` |

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

设计"夹爪开合即动作"（挤压/松开）时，**开合值必须落在 task 检测区间内**，否则 task 永远检测不到状态。

## 常见坑（已踩过）

1. **__init__ 签名照抄旧文档**：几何参数是 forward() 参数，不是 __init__ 参数；__init__ 只有 4 个参数
2. **漏 get_stage_units() 换算**：夹爪关节值必须除以 stage units，否则错 100 倍
3. **漏 _start 第一帧**：第一帧必须开夹爪（抓取类），否则事件 0 会带夹爪旧状态开始
4. **events_dt 长度与事件数不匹配**：会 IndexError 或提前结束。写完后数一遍 + 长度校验
5. **grasp_distance 与物体几何不匹配**：夹太松 joint7 下不到 0.025，kinematic 吸附永不触发。查 pick_controller.get_gripper_distance 表（pipette 0.008、beaker 0.022-0.03、rod 0.003、Petri dish 0.005、tube 0.01、conical_bottle 0.01-0.03、graduated_cylinder 0.005-0.03、Erlenmeyer flask 0.018）
6. **位置硬编码**：所有目标点必须来自 forward 参数，不能写死
7. **挤压/松开值不落在 task 检测区间**：squeeze 必须 < 0.005，release 必须在 0.005-0.025 之间
8. **dwell 太短**：task 每帧检测，dwell 0.02-0.05（50-20 帧）都可行，但 0.05 只有 20 帧是 cap 用过的下限，更稳妥用 0.02-0.03（50-33 帧）；换算记住帧数 = 1/dt
9. **资产缺失仍写动作**：动作所需器材没有 3D 资产就动手，grasp_distance/pickz_offset 只能瞎猜，场景也摆不出来。工作流第 2 步先查资产，缺了就调 labutopia-assets skill 生成

## 检查清单

- [ ] 动作所需器材全部有 3D 资产（lab_inventory.json ↔ assets/chemistry_lab 核对过），缺失的已调 labutopia-assets 生成
- [ ] 动作已分类（位置/夹爪开合/velocity），选了对应模板
- [ ] __init__ 只有 (name, cspace_controller, events_dt=None, position_threshold=0.01)
- [ ] 有 _start 第一帧（开夹爪 0.04/su 或保持闭合）
- [ ] 所有夹爪值除以 get_stage_units()
- [ ] events_dt 长度 = 事件数，且有长度校验
- [ ] 平移事件 xy 跳转、到达事件 3D 跳转
- [ ] 开合值对齐 task 检测区间（<0.025 吸附、>0.03 释放、<0.005 挤压）
- [ ] py_compile 通过 + stub 走查 0→N 全事件可达
