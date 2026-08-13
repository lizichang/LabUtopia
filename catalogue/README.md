# LabUtopia 动作库（catalogue）

依据 **`LabUtopia_Action_Catalogue_v10.docx`**（v10）建立的 39 个动作分类骨架。

> **v10 摘要**：39 个独立动作 | 1,421 次有效实验操作 | 覆盖率 100%。每个动作**独立场景、步骤自包含**（含采样/配液预处理），物质用通用名称（"待测固体/液体样品"、"指定××试剂"），不依赖其他动作结果。

## 目录结构

```
catalogue/
├── README.md          # 本文件：39 动作总索引 + 映射说明
├── factory.py         # 注册入口（实现到哪个动作就注册哪个）
├── _shared/           # 共享底座（base_task / base_controller / constants）
├── a_instrument/      # A 仪器测量 (3)
├── b_thermal/         # B 热操作 (5)
├── c_flame/           # C 焰色燃烧 (4)
├── d_wetchem/         # D 湿化学 (23) —— 无 D1，-S/-L 各算一个
└── e_physical/        # E 物理检测 (4)
```

每个动作子目录：`__init__.py`（描述）+ 后续实现的 `task.py` / `controller.py` / `config.yaml`。

## 分类规则（与 v10 一致）

- **D 类无 D1**，编号从 D2-S 起。
- **-S（固体样品）与 -L（液体样品）拆分为两个独立动作**（v10 原文：*"同时适用于固体和液体样品的动作已拆分为两个独立动作"*）。D 类 23 = 8 对 -S/-L（16）+ D10-D16（7）。
- 已移除视觉观察/PPE/嗅觉步骤，读数与颜色判别由感知子系统单独处理。

## 39 动作总索引

### A 仪器测量 (3)

| 编号 | 目录 | 名称 | 类型 | 状态 |
|---|---|---|---|---|
| A1 | `a_instrument/a1_refractometer/` | 折光率测量 | 单动作 | 待开发 |
| A2 | `a_instrument/a2_polarimeter/` | 旋光仪测量 | 复合宏 | 待开发 |
| A3 | `a_instrument/a3_conductivity/` | 电导率测量 | 复合宏 | 待开发 |

### B 热操作 (5)

| 编号 | 目录 | 名称 | 类型 | 状态 |
|---|---|---|---|---|
| B1 | `b_thermal/b1_alcohol_heat_solid/` | 酒精灯加热（固体样品） | 单动作 | 待开发 |
| B2 | `b_thermal/b2_alcohol_heat_liquid/` | 酒精灯加热（液体/沸点测定） | 单动作 | 待开发 |
| B3 | `b_thermal/b3_water_bath/` | 水浴加热 | 单动作 | 待开发 |
| B4 | `b_thermal/b4_ice_bath/` | 冰浴/冷却 | 单动作 | 待开发 |
| B5 | `b_thermal/b5_melting_point/` | 熔点测定（毛细管法） | 单动作 | 待开发 |

### C 焰色燃烧 (4)

| 编号 | 目录 | 名称 | 类型 | 状态 |
|---|---|---|---|---|
| C1 | `c_flame/c1_flame_wire_solid/` | 焰色反应（铂丝蘸取固体） | 单动作 | 已填（复用 flametest） |
| C2 | `c_flame/c2_cobalt_glass/` | 焰色反应（隔钴玻璃观察） | 单动作 | 待开发（需钴玻璃场景，现有 flametest 不含） |
| C3 | `c_flame/c3_combustion_solid/` | 燃烧试验（固体） | 单动作 | 待开发 |
| C4 | `c_flame/c4_combustion_liquid/` | 燃烧试验（液体） | 单动作 | 待开发 |

### D 湿化学 (23) —— 无 D1

| 编号 | 目录 | 名称 | 类型 | 状态 |
|---|---|---|---|---|
| D2-S | `d_wetchem/d2s_water_solubility/` | 固体样品水溶性测试 | 单动作 | 待重写（原 dissolve 已废弃删除） |
| D2-L | `d_wetchem/d2l_water_solubility/` | 液体样品水溶性测试 | 单动作 | 待开发 |
| D3-S | `d_wetchem/d3s_acid_reagent/` | 酸性试剂滴加反应（固） | 单动作 | 模板 → dropperdrip |
| D3-L | `d_wetchem/d3l_acid_reagent/` | 酸性试剂滴加反应（液） | 单动作 | 模板 → dropperdrip |
| D4-S | `d_wetchem/d4s_alkali_reagent/` | 碱性试剂滴加反应（固） | 单动作 | 模板 → dropperdrip |
| D4-L | `d_wetchem/d4l_alkali_reagent/` | 碱性试剂滴加反应（液） | 单动作 | 模板 → dropperdrip |
| D5-S | `d_wetchem/d5s_precipitation/` | 沉淀检测试剂滴加（固） | 单动作 | 模板 → dropperdrip |
| D5-L | `d_wetchem/d5l_precipitation/` | 沉淀检测试剂滴加（液） | 单动作 | 模板 → dropperdrip |
| D6-S | `d_wetchem/d6s_redox/` | 氧化还原试剂滴加（固） | 单动作 | 模板 → dropperdrip |
| D6-L | `d_wetchem/d6l_redox/` | 氧化还原试剂滴加（液） | 单动作 | 模板 → dropperdrip |
| D7-S | `d_wetchem/d7s_organic_qual/` | 有机定性试剂滴加（固） | 单动作 | 模板 → dropperdrip |
| D7-L | `d_wetchem/d7l_organic_qual/` | 有机定性试剂滴加（液） | 单动作 | 模板 → dropperdrip |
| D8-S | `d_wetchem/d8s_complex_color/` | 络合/显色试剂滴加（固） | 单动作 | 模板 → dropperdrip |
| D8-L | `d_wetchem/d8l_complex_color/` | 络合/显色试剂滴加（液） | 单动作 | 模板 → dropperdrip |
| D9-S | `d_wetchem/d9s_gas_indicator/` | 气体检测/指示剂滴加（固） | 单动作 | 模板 → dropperdrip |
| D9-L | `d_wetchem/d9l_gas_indicator/` | 气体检测/指示剂滴加（液） | 单动作 | 模板 → dropperdrip |
| D10 | `d_wetchem/d10_solid_reagent_add/` | 固体试剂添加反应（液） | 单动作 | 待开发 |
| D11 | `d_wetchem/d11_testpaper_gas/` | 试纸气体检测 | 单动作 | 待开发 |
| D12 | `d_wetchem/d12_testpaper_gas2/` | 试纸气体检测（另一类型） | 单动作 | 待开发 |
| D13 | `d_wetchem/d13_multi_drip/` | 多步试剂连续滴加反应 | 复合 | 待开发 |
| D14 | `d_wetchem/d14_virtual_tube/` | 同管多试剂串联（VirtualTube） | 链式宏 | 待开发 |
| D15 | `d_wetchem/d15_acid_base_titration/` | 酸碱滴定 | 双臂协同 | 待开发 |
| D16 | `d_wetchem/d16_distillation/` | 蒸馏分离 | 低优先级 | 待开发 |

### E 物理检测 (4)

| 编号 | 目录 | 名称 | 类型 | 状态 |
|---|---|---|---|---|
| E1 | `e_physical/e1_ph_testpaper/` | pH 试纸检测 | 单动作 | 待开发 |
| E2 | `e_physical/e2_magnetic/` | 磁性检测 | 单动作 | 待开发 |
| E3 | `e_physical/e3_density/` | 密度测定 | 单动作 | 待开发 |
| E4 | `e_physical/e4_weighing/` | 称量（独立操作） | 单动作 | 待开发 |

**合计：39 个动作**（A3 + B5 + C4 + D23 + E4）。

## 已有实现映射（不迁移，原样保留）

| v10 动作 | 现有实现（config / task / controller） |
|---|---|
| C1 焰色反应（铂丝蘸取固体） | `config/level2_FlameTest.yaml` + `tasks/flametest_task.py` + `controllers/flametest_controller.py`（catalogue 已填，见 `c_flame/c1_flame_wire_solid/`） |
| D3-D9 试剂滴加模板 | `config/level2_DropperDrip.yaml` + `tasks/dropperdrip_task.py` + `controllers/dropperdrip_controller.py` |
| B 类加热（近 B2） | `config/level2_HeatLamp.yaml` + `tasks/heatlamp_task.py` + `controllers/heatlamp_controller.py` |
| 点火前置（B/C 类共用） | `config/level2_IgniteLamp.yaml` + `tasks/ignitelamp_task.py` + `controllers/ignitelamp_controller.py` |

> **C2（隔钴玻璃观察）说明**：现有 flametest 场景与流程**不含钴玻璃**（已 grep 确认无 cobalt 引用）。
> 隔钴玻璃观察需要新场景（钴玻璃片 + 观察步骤，用于滤黄焰以辨 K 紫），C2 归为待开发。

## catalogue 内已填动作的运行方式

每个已填动作目录含自包含 `config.yaml`（键 = 目录 snake_case，工厂已注册），直接：

```bash
python main.py --config-dir catalogue/c_flame/c1_flame_wire_solid --config-name config --backend gpu
```

原 `config/level2_*.yaml`（键如 `flametest`）继续可用，两条路径指向同一实现类。

## 新增动作流程

1. 在该动作子目录实现 `task.py`（继承 `_shared/base_task.py` 或 `tasks/base_task.py`）、`controller.py`（继承 `_shared/base_controller.py` 或 `controllers/base_controller.py`）、`config.yaml`。
2. 在 `factory.py` 注册（键 = snake_case，与 config 的 `task_type`/`controller_type` 一致）。
3. 场景 USD 资产放 `assets/`，由 config 的 `usd_path` 引用。
4. 至少跑一次 `python main.py --config-name <config>` + `python tests/test_single_config.py`（AGENTS.md §6）。

## 复用与参考

- **复合实验样板**：AGENTS.md §3.7 —— `controllers/dissolve_controller.py`（Phase 枚举 + `_phase_action` + `_check_phase_success` + `_advance_phase` + `set_initial_position` 注入）。
- **原子动作**：`controllers/atomic_actions/` 下 15 个通用原子动作（pick/place/pour/scoop/shake/dropper/uncap/cap/ignite/match_ignite/press/move 等）直接 import 复用，不注册 factory。
- **焰色专属 IK 原子动作**：`controllers/atomic_actions/flametest/`（IkMotionEngine + MoveAction/GripAction/HoldAction）供 C 类复用；元动作基类模板见 `controllers/flametest_meta_actions/_base.py`（BaseMetaAction + mv/grip/hold 工厂）。
- **技能文档**：`docs/skills/labutopia-task/`（SKILL.md + reference.md，含 30 个已踩坑）。
