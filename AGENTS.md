# AGENTS.md

本文件面向在本仓库中工作的 coding agents（如 Codex、Claude Code、Cursor Agent 等）。
目标是帮助代理在尽量少踩坑的前提下，完成代码修改、验证和交付。

## 1. 项目概览

- 项目名：LabUtopia
- 类型：基于 NVIDIA Isaac Sim 的高保真实验室仿真与具身智能 benchmark
- 主要语言：Python 3.11
- 关键依赖：Isaac Sim 5.1、PyTorch、Hydra / OmegaConf、h5py、loguru
- 运行环境：README 标注为 Ubuntu 24.04 + NVIDIA RTX GPU + CUDA；开发/同步在 Windows 本机进行

高层入口：

- `main.py`：仿真主入口，负责 `config -> factory -> task/controller -> simulation loop`
- `tests/test_single_config.py`：单个配置的烟雾测试
- `tests/test_config_files.py`：批量配置测试

训练不在本仓库树内进行：`policy/` 下是四个 VLA 子模块（Isaac-GR00T、lerobot、
lingbot-vla、openpi），训练在各子模块内部完成。树内 `train.py` / `train-muilt.py`
（Diffusion UNet / ACT）为历史遗留入口，仅供参考，不要作为新训练流程的起点。

最近开发（2026-08）：D2 蒸馏水溶解性测试（level2_Dissolve），是"资产 → 元动作 →
复合实验"三层开发链路的完整范例，见 `docs/skills/`。

## 2. 目录速览

- `config/`：任务 YAML 配置，按 Level 1-5 组织（文件名 CamelCase 与注册键 snake_case 混用，见 §7）
- `tasks/`：任务环境，负责场景、物体、相机、观测、kinematic 事件检测
- `controllers/`：控制逻辑，负责动作生成、成功判定、数据采集/推理/回放调度
- `controllers/atomic_actions/`：可复用元动作（scoop / shake / pick / uncap / cap / dip / ignite 等 18 个）
- `factories/`：task/controller/robot/collector 注册与构造
- `robots/`：Franka、Ridgebase、Piper 等机器人定义
- `data_collectors/`：HDF5 采集逻辑
- `policy/`：策略训练（git 子模块：Isaac-GR00T、lerobot、lingbot-vla、openpi）
- `packages/openpi-client/`：内置的远程推理 client 子包
- `assets/`：USD 场景和机器人资源，体积大且容易误伤
- `utils/`：`ObjectUtils`、相机处理、回放加载、A* 等工具
- `docs/skills/`：LabUtopia 开发技能（资产 / 元动作 / 复合实验），**新增能力前必读**

## 3. 核心架构约束

### 3.1 Task / Controller 分工

- `Task` 负责场景状态和观测数据。
- `Controller` 负责机器人动作和成功条件判断。
- 两者由 `main.py` 独立创建，再在运行时拼接。

不要把场景初始化逻辑塞进 controller，也不要把动作决策逻辑塞进 task，除非你在做明确的架构调整。

### 3.2 Task 基类约束

所有任务最终继承自 `tasks/base_task.py` 中的 `BaseTask`。

最低要求：

- 实现 `step(self) -> Optional[Dict[str, Any]]`
- 需要支持确定性回放时，实现或扩展 `reset_with_init_state`

可直接复用的基础能力：

- `setup_cameras()`
- `setup_objects()`
- `setup_materials()`
- `setup_lighting()`
- `get_basic_state_info(...)`
- `reset()` / `reset_with_init_state(...)`

状态字典里会自动包含 `init_state`，用于 replay。

### 3.3 Controller 基类约束

所有控制器最终继承自 `controllers/base_controller.py` 中的 `BaseController`。

子类必须实现：

- `_step_collect(self, state) -> tuple[Any, bool, bool]`
- `_step_infer(self, state) -> tuple[Any, bool, bool]`
- `_check_success(self) -> bool`

注意：

- `_step_replay()` 已在 `BaseController` 中实现，通常不要覆写
- `BaseController.step()` 会先把最新观测写入 `self.state`
- 成功判定通常依赖 `self.check_success_counter >= self.REQUIRED_SUCCESS_STEPS`
- 当前默认 `REQUIRED_SUCCESS_STEPS = 60`
- `_last_failure_reason` 在本仓库里应始终是 `str`，无错误时用 `""`，不要写成 `None`

### 3.4 Factory 注册模式

工厂均采用显式注册，不是自动发现：

- `factories/task_factory.py`
- `factories/controller_factory.py`
- `factories/robot_factory.py`
- `factories/collector_factory.py`

新增 task/controller/robot/collector 时，除了写类本身，还必须注册到对应 factory。

配置里的键必须是已注册的 key，**区分大小写**（注册键有 CamelCase 也有 snake_case，见 §7）。

`task_type` 和 `controller_type` 不一定相同：

- `level1_open_door.yaml` 使用 `task_type: "open_close"` + `controller_type: "open"`
- `level1_close_door.yaml` 使用 `task_type: "open_close"` + `controller_type: "close"`

不要想当然把二者改成同名。

### 3.5 Singleton / 配置约束

- `ObjectUtils` 必须通过 `ObjectUtils.get_instance()` 获取，不要直接实例化
- 注册 OmegaConf resolver 之前，先用 `OmegaConf.has_resolver(...)` 判断
- 日志统一使用 `loguru.logger`

### 3.6 元动作模式（atomic_actions）

Level2+ 复合任务的可复用动作放在 `controllers/atomic_actions/`（如
`scoop_controller.py`、`shake_controller.py`），不单独注册 factory，由复合 controller
直接 import 调用。

统一模式：

- **events_dt 事件状态机**：`_t += events_dt[event]`，`_t >= 1.0` 推进事件；`events_dt`
  长度必须严格等于事件数；位置到位（阈值 0.01）可提前跳转；dwell 事件 0.02 ≈ 50 帧
  （@60fps，需 ≥ task 检测窗口）
- **夹爪契约**：`joint_positions[7]/[8]` = 手指距离（小 = 闭合）；task 的 kinematic 吸附
  检测只看实测 `joint7 < 0.025`（不依赖物理夹持），释放检测 `joint7 > 0.03`；
  grasp_distance 目标需与物体几何匹配（勺子 0.006、洗瓶瓶颈 0.018）
- **位置参数化**：动作发生位置必须可传入（如 ShakeController 的
  `set_initial_position`），禁止硬编码坐标

### 3.7 复合实验模式（level2-4，D2 范例）

`dissolve_task.py` + `dissolve_controller.py` + `level2_Dissolve.yaml` 是最新完整范例：

- **Task 侧**：每个动过物体一个 `_update_<obj>()` 子状态机（rest → attached → dwell →
  隐藏 prim 显隐 → released → `_return_to_origin`）；参考点全部在 `reset()` 里从 prim
  位置 + 偏移链推导（禁止硬编码坐标）；阶段完成的判定条件必须有显式门控（如
  `water_added`），不能只靠距离阈值，否则上一步停留位置会误触发
- **Controller 侧**：`Phase` 枚举（如 SCOOP_POWDER → POUR_WATER → SHAKE → OBSERVE →
  FINISHED）+ `_phase_action` + `_check_phase_success`（上一个 phase 的 object state
  == 'released' 才推进）+ `_advance_phase`（进入新 phase 前先 `set_initial_position`
  等参数注入）
- 同一元动作可通过不同参数复用（ScoopController 舀粉 / 注水两个用法）

## 4. 修改建议

### 4.1 优先修改哪里

- 改任务逻辑：优先看 `tasks/`
- 改动作流程或成功判定：优先看 `controllers/`
- 新增任务类型：通常要同时改 `tasks/`、`controllers/`、`factories/`、`config/`
- 新增资产/场景：参考 `docs/skills/labutopia-assets/`（MeshBuilder → OBJ → USD 管线）
- 新增元动作：参考 `docs/skills/labutopia-atomic-action/`（events_dt 状态机模板）
- 新增复合实验：参考 `docs/skills/labutopia-compound-task/`（三层架构模板 + 检查清单）
- 改训练逻辑：在 `policy/` 下对应的 VLA 子模块内改（注意子模块有独立的 git 仓库）
- 改远程推理：看 `controllers/inference_engines/` 和 `packages/openpi-client/`

### 4.2 尽量不要碰哪里

- `assets/`：大部分是 USD/材质/模型资源，除非用户明确要求，不要编辑
- `outputs/`、测试输出、数据集目录：不要把生成物混进提交
- `packages/openpi-client/`：这是内置子包，只在远程推理链路相关变更时再改

### 4.3 命名与风格

- task 文件：`{action}_task.py`，controller 文件：`{action}_controller.py`
- task 类：`{Action}Task`，controller 类：`{Action}TaskController` 或项目里已有的对应命名
- **factory 注册键**：新建时用 snake_case（如 `dissolve`）；复用已有键时以
  `factories/*_factory.py` 中的实际注册值为准
- camera 观测键：`{camera_name}_{image_type}`
- episode 文件：`episode_{NNNN}.h5`

代码风格按仓库现有配置执行：

- 顶层 `pyproject.toml` 使用 `ruff` 和 `mypy`
- 公共方法尽量补齐类型标注
- import 分组为：stdlib -> third-party -> first-party
- 不要把 import 塞进函数体，除非有非常明确的延迟导入需求

## 5. 常用工作流

### 5.1 运行仿真

建议总是显式指定配置名（**默认值不可用，必须传**，见 §7）：

```bash
python main.py --config-name level1_pick
python main.py --config-name level2_Dissolve
python main.py --config-name level5_Navigation --no-video
```

说明：

- `mode` 由 YAML 配置控制，常见值为 `collect` / `infer` / `replay`
- 输出目录通常由配置中的 `hydra.run.dir` 和 `multi_run.run_dir` 决定
- `--backend gpu` 会切换到 GPU 物理设置；默认是 `numpy`

### 5.2 训练策略

先用 `scripts/convert_labsim_data_to_lerobot.py` 把采集数据导出为 LeRobot 格式，
再进入 `policy/` 下对应子模块（Isaac-GR00T / lerobot / lingbot-vla / openpi）
按其 README 训练（合并/增注脚本：`scripts/merge_dataset.py`、
`scripts/add_language_instructions.py`）。

### 5.3 轻量验证

静态检查：

```bash
ruff check .
ruff format --check .
mypy .
```

说明：

- 顶层 `ruff` / `mypy` 默认排除了 `packages/`、`assets/`、`outputs/`
- 如果你改了 `packages/openpi-client/`，请额外对该子包做定向检查

仿真相关烟雾测试（需要 Isaac Sim 环境）：

```bash
python tests/test_single_config.py level1_pick
python tests/test_single_config.py level1_open_door
python tests/test_config_files.py
```

这两个测试脚本会把配置临时改为：

- `mode: collect`
- `max_episodes: 8`
- `collector.type: mock`

因此它们是比真实采集更轻的验证方式。

### 5.4 本地-服务器开发流程（本项目约定）

- 本机无 Isaac Sim 运行环境，**修改一律在本地仓库进行**，由用户手动把文件覆盖到
  服务器（10.98.19.29，`/media/dky/Disk2TB/lizichang/LabUtopia`）并运行测试
- 交付时给出：覆盖文件清单 + 测试指令（如 `python main.py --config-name level2_Dissolve`）
  + 预期行为检查表；用户反馈 `xxx failed!` 日志后再迭代
- 本机与服务器通过 SFTP 同步；diff 时注意本机 `core.autocrlf=true` 会产生 CRLF 假差异，
  必须用 MD5/规范化（去 `\r`）对比，不要只看大小或行数
- 服务器仓库存在未提交的本地修改与 untracked 文件（_bak 备份、散装资产等），
  同步时以 git 跟踪状态为准，不要整体覆盖

## 6. 新增能力时的最小变更清单

### 6.1 新增任务

至少检查以下几点：

1. 在 `tasks/` 中新增任务类
2. 在 `controllers/` 中新增或复用控制器
3. 在对应 factory 中注册（键与 yaml 的 `task_type` / `controller_type` 一致）
4. 新增 `config/*.yaml`
5. 若涉及资产/场景，走 `docs/skills/labutopia-assets/` 流程
6. 至少做一次 `test_single_config.py` 或最小手动运行

### 6.2 新增配置

至少确认：

- `usd_path` 存在
- `robot.type` 已注册
- `collector.type` 已注册
- `task.max_steps` 合理
- 相机配置里的 `prim_path`、`resolution`、`image_type` 正确
- 如果依赖 replay / infer，相关路径或服务参数已经设置
- 复合实验的 `offsets` / `thresholds`（joint7 吸附/释放、dwell 帧数）合理

## 7. 已知陷阱

- `main.py` 的 `--config-name` 默认值为 `level3_Heat_Liquid`，与配置文件
  `level3_HeatLiquid.yaml` 不一致，**不传会直接报错**；运行时必须显式传 `--config-name`
- `main.py` 虽然解析了 `--headless`，但当前 `SimulationApp` 初始化仍写死为
  `"headless": False`；不要假设这个参数已经生效
- **registry key 大小写不统一**：`OpenTransportPour`、`LiquidMixing` 是 CamelCase，
  `dissolve`、`ignitelamp`、`flametest`、`heatlamp`、`openclose` 等是 snake_case；
  写配置前先 grep `factories/*_factory.py` 确认实际注册值，不要照搬其他 yaml
- 资产生成脚本保存场景必须 `stage.Export(新路径)`，**禁止 `stage.Save()`**（会写回并污染源 USD）
- `obj2usd.py` 的 write_mesh 必须把顶点重映射为局部索引，否则 bbox/geometry_center 全错
- kinematic 吸附（joint7 < 0.025）不依赖物理夹持，controller 必须真实闭到阈值以下
- 工作区可能已经有用户未提交改动；除非用户明确要求，不要回退他人的变更
- git 层面：仓库启用 LFS（assets/**、*.usd、*.mdl 等）；本机 `core.autocrlf=true`，
  commit 前注意行尾差异；`image1.jpg` / `image1.JPG` 大小写双文件是历史遗留，勿增新副本

## 8. 提交前检查

在结束修改前，尽量完成以下事项：

1. 只改与任务直接相关的文件
2. 避免误改 `assets/` 和生成物目录
3. 跑至少一轮与改动匹配的最小验证
4. 若未能验证，明确说明原因（如本机无 Isaac Sim / 无 GPU / 无数据）
5. 在说明中写清楚是否改了 config、factory 注册项或运行命令
