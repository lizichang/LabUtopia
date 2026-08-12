# LabUtopia 任务/元动作制作参考（reference）

## 抓取三问（每个 phase 动手前必须回答，答不上来不许写 forward）

机械臂"抓哪、移到哪"必须从**步骤的物理语义**推导，再拿**资产几何**定参数：

1. **抓哪个部位？** 可抓部位来自资产 USD 的 extent 区间（读 `extent`：如滴管胶头 z 0.115-0.15、玻璃管 z 0-0.12、勺柄中心、瓶盖顶面）
2. **为什么是它（一句话物理语义）？** 例：捏胶头才能挤压排液/松开吸液（吸液靠胶头弹力回吸）；移出滴管不能碰瓶口外壁（污染）；舀取用勺头、转移靠勺头对准管口
3. **参数怎么定？** 抓取点 = 资产原点 + 部位区间内的偏移（留余量：胶头 0.115-0.15 → 抓 0.13）；grasp_distance 对照部位直径查表；dip/target 高度 = 液面/管口 + 抓取偏移（尖端在 TCP 下方 grasp_offset.z 处）

**验证闭环**：冒烟 debug T 行记录抓取点实际 z，核对落在预期区间（dropper 抓取点实测 0.938 = 原点 0.800 + 0.138，落在胶头区间 ✓）。判定全绿（TCP/joint7/状态机对）不代表抓对地方——抓错部位的案例：dropper 曾抓玻璃管身 z=0.06，用户纠正"滴管要捏在最上部的胶头"。

**移动到哪同理**：dip 点为什么是瓶口中心（尖端要浸入液面下方）、target 点为什么是试管口（滴加进管内）、路径约束（移出瓶口时高于瓶口）。

## 动作类型与模板选择

| 类型 | 判断依据 | 模板 |
|---|---|---|
| 位置驱动 | 动作语义 = 夹爪到某点 + 停留（task 按位置/停留检测） | scoop / dip / ignite / pick |
| 夹爪开合驱动 | 动作语义 = 夹爪距离变化（捏、压、挤、放） | dropper（挤压吸液）/ press（闭爪前压） |
| velocity 驱动 | 关节按速度旋转（倾倒液体） | pour |

## 资产依赖检查（缺资产先调用 labutopia-assets）

动作分解完成后、写代码之前，先确认每个器材都有 3D 资产：

1. 从动作分解列出所需器材清单（如：胶头滴管、试剂瓶、试管、废液杯）
2. 核对 `lab_inventory.json`（项目根目录，equipment 的 notes 常带尺寸材质）与 `assets/chemistry_lab/*.usd`（场景类资产在 `assets/chemistry_lab/lab_00x/`）
3. **缺失的器材 → 调用 labutopia-assets skill**（实物调研通法：inventory 优先 + 结构拆解三步法 + 形状原语映射），等资产生成完成再继续
4. 几何参数必须对照真实资产尺寸：grasp_distance（夹爪距）来自资产直径，pickz_offset 来自资产高度，pre_offset_z/lift_z 参考资产尺寸设定
5. 资产文件名要与 controller 里传的 object_name 一致（pick 的 get_gripper_distance/get_pickz_offset 按名字查表），新资产命名后同步查表项

## 统一签名（所有新动作必须遵守）

```python
def __init__(self, name, cspace_controller, events_dt=None, position_threshold=0.01):
    super().__init__(name=name)
    self._event = 0
    self._t = 0
    if events_dt is None:
        self._events_dt = [...]          # 长度 = 事件数
    else:
        # 校验：list/np.ndarray，长度必须等于事件数
        if not isinstance(self._events_dt, (np.ndarray, list)):
            raise Exception("events_dt must be a list or numpy array")
        if isinstance(self._events_dt, np.ndarray):
            self._events_dt = events_dt.tolist()
        if len(self._events_dt) != N:
            raise Exception(f"events_dt length must be {N}, got {len(self._events_dt)}")
    self._cspace_controller = cspace_controller
    self._start = True
    self._position_threshold = position_threshold

def forward(self, <位置参数...>, current_joint_positions, gripper_position, gripper_control,
            end_effector_orientation=None, <几何/夹爪参数...>):
    # 几何参数（pre_offset_z/lift_z/grasp_distance/squeeze_distance...）默认值写在这里
```

forward 内的模式（统一）：

```python
if self._start:
    return self._handle_start_state(current_joint_positions)   # 抓取类：开夹爪 0.04/su
if end_effector_orientation is None:
    end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
target_joint_positions = self._execute_phase(...)
if self._event < len(self._events_dt):
    self._t += self._events_dt[self._event]
    if self._t >= 1.0:
        self._event += 1
        self._t = 0
return target_joint_positions
```

## DropperController 夹爪开合驱动模板（胶头滴管：抓取→吸液→滴加，14 事件）

实现在 `controllers/atomic_actions/dropper_controller.py`（2026-08 验证通过）。核心：**挤压/松开 = 夹爪距离变化 + dwell，task 按 joint7 区间检测状态**。抓取点 = 胶头（z=0.13，胶头区间 0.115-0.15），勿抓玻璃管身。

```python
class DropperController(BaseController):
    """14 事件。挤压/松开语义：
    joint7 < 0.005 -> "squeeze"（排空气/滴加）
    0.005 < joint7 < 0.025 -> 仍吸附（松开吸液时用 release_distance=0.015）
    joint7 > 0.03 -> 释放
    """
    # events_dt（14 项）:
    # [0.004, 0.004, 0.02, 0.05, 0.004, 0.004, 0.05, 0.004, 0.05, 0.004, 0.004, 0.004, 0.05, 0.006]
    def forward(self, grasp_position, dip_position, target_position,
                current_joint_positions, gripper_position, gripper_control,
                end_effector_orientation=None,
                pre_offset_z=0.12, lift_z=0.10,
                grasp_distance=0.008,        # 参照 pick 表 pipette
                squeeze_distance=0.002,      # < 0.005 -> task 检测 squeeze
                release_distance=0.015):     # 0.005~0.025 -> 仍吸附 + 未挤压
        # event 0: 移到 grasp 上方 pre_offset_z（xy 跳转）
        # event 1: 下探到 grasp_position（3D 跳转）
        # event 2: dwell 0.02
        # event 3: 闭爪 grasp_distance（task 吸附）
        # event 4: 上提 lift_z（3D 跳转）
        # event 5: 移到 dip 上方 pre_offset_z（xy 跳转）
        # event 6: 闭爪 squeeze_distance（挤压排空气，dwell 等 task 检测）
        # event 7: 下探到 dip_position 浸入液面（3D 跳转）
        # event 8: 张爪 release_distance（松开吸液，dwell 等 task 检测）
        # event 9: 上提 lift_z（3D 跳转）
        # event 10: 移到 target 上方 pre_offset_z（xy 跳转）
        # event 11: 下探到 target_position（3D 跳转）
        # event 12: 闭爪 squeeze_distance（挤压滴加，dwell 等 task 检测）
        # event 13: 完成（None 关节保持）
```

要点：挤压/松开事件（6/8/12）输出夹爪关节值后**不跳转**，靠 events_dt 0.05（20 帧 ≈ 0.33s）留足 task 检测窗口；task 端按 `joint7` 区间 + TCP 位置更新 `dropper_state`（'attached'/'squeezed'/'filled'/'dropped'），复合 controller 的 `_check_phase_success` 读它切换阶段。**注意：排空挤压与滴加挤压的 joint7 值相同（都是 squeeze_distance），task 必须联合 TCP 位置区分（液瓶口=排空、试管口=滴加），否则排空阶段就误判 dropped**。

## ScoopController 11 阶段模板（位置驱动，舀取/注水通用）

```python
class ScoopController(BaseController):
    """11 事件。wash 模式：scoop_position 传提升点即可当"抓起后移动到"用。"""
    def __init__(self, name, cspace_controller, events_dt=None, position_threshold=0.01):
        # events_dt（11 项）:
        # [0.004, 0.004, 0.02, 0.03, 0.004, 0.004, 0.04, 0.004, 0.004, 0.05, 0.05]
        ...

    def forward(self, grasp_position, scoop_position, transfer_position,
                current_joint_positions, gripper_position, gripper_control,
                end_effector_orientation=None, pre_offset_z=0.12, lift_z=0.10,
                grasp_distance=0.006, retract_offset=None):
        # event 0: 移动到 grasp 上方 pre_offset_z（xy 阈值 0.01 跳转）
        # event 1: 下探到 grasp_position（3D 阈值 0.01 跳转）
        # event 2: dwell（等待 task 检测 attached：joint7 < 0.025）
        # event 3: 闭合夹爪（grasp_distance）
        # event 4: 抬升 lift_z（3D 阈值 0.01 跳转）
        # event 5: 移动到 scoop_position（3D 阈值 0.01 跳转）
        # event 6: dwell（等待 task 检测"粉末已上勺"）
        # event 7: 移动到 transfer 上方（xy 跳转）
        # event 8: 下探到 transfer_position（3D 阈值 0.01 跳转）
        # event 9: dwell（等待 task 检测"已转移"）
        # event 10: 打开夹爪 + 保持（task 检测 released: joint7 > 0.03）
```

## PourController velocity 驱动模式（倾倒）

```python
# 与位置/夹爪模式完全不同：不设目标位置，直接控制关节速度
# 倾倒关节 = dof_index 6（手腕旋转）
articulation_controller.switch_dof_control_mode(dof_index=6, mode="velocity")
articulation_controller.set_joint_velocities(np.array([pour_speed]), indices=[6])
# 事件推进靠 _t 累积（duration 到点进入下一事件），无位置跳转
# 参考 pour_controller.py：get_pickz_offset 查表决定抬到多高再倾
```

## 位置参数命名表（新动作照此命名）

| 阶段 | 参数名 | 示例 |
|---|---|---|
| 抓取位 | grasp_position | scoop / dropper |
| 桌面帽位（盖帽） | cap_rest_position / cap_closed_position | cap |
| 浸入/蘸取位 | dip_position | dip / dropper |
| 转移/注入位 | transfer_position / target_position | scoop / dropper |
| 点火位 | ignite_position | ignite |

## 夹爪契约速查

| 项 | 值 |
|---|---|
| joint_positions[7]/[8] | 手指距离（米，小 = 闭合），必须除以 get_stage_units() |
| 打开指令 | 0.04 / su（所有动作第一帧） |
| task 吸附检测 | joint7 < 0.025（kinematic，不看物理） |
| task 释放检测 | joint7 > 0.03 |
| task 挤压检测 | joint7 < 0.005（press 闭爪 0.0015 同区间） |
| 松开吸液区间 | 0.005 < joint7 < 0.025（dropper release 0.015） |
| grasp_distance 查表 | pick_controller.get_gripper_distance：pipette 0.008、tube 0.01、rod 0.003、Petri dish 0.005、microscope slide 0.002、beaker 0.022-0.03、conical_bottle 0.01-0.03、graduated_cylinder 0.005-0.03、Erlenmeyer flask 0.018；查不到默认 0.02 |

## 注册与使用

```python
# 仓库惯例：复合 controller 直接 import（scoop/cap/dip 均未进 __init__.py 照样用）
from .atomic_actions.dropper_controller import DropperController
# __init__.py 只导出最常用的 11 个，新动作注册可选
```

## task 检测设计指引（新动作必须先对齐）

1. task 维护一个 state 字段（如 `dropper_state`），按 joint7 区间 + 位置更新
2. 复合 controller 的 `_check_phase_success` 读该字段切换阶段（参考 ignitelamp_controller：`cap_state=='placed'` / `flame_on`）
3. 每个需要 task 检测的事件必须留 dwell 窗口（帧数 = 1/dt：0.02→50 帧、0.05→20 帧；常用 0.02-0.05，更稳妥 0.02-0.03）
4. 挤压类动作：挤压值必须 < 0.005（squeeze 检测）且释放值必须 < 0.025（否则脱离吸附）
5. **kinematic 跟随必须覆盖整个吸附期间**（attached→squeezed→filled→dropped 全程，released 才复位到初始位置）——跟随只放 attached 分支会让物体在后续阶段悬空冻结（坑 19），T 行必须记录物体实际位置才能发现

### 多阶段动作：粘性标志模式（必须）

**原子 controller 一次跑完整个事件序列时，不能用"当前状态"做阶段判定。** 等 is_done 时状态机早已走到最终态（dropper 14 事件跑完 → dropper_state 已到 dropped），PICK 阶段查 `== 'attached'` 必误报失败——dropper 曾整序列动作全对仍报 "pick_dropper failed" 即此因。

```python
# task：粘性标志，达成置位、reset 才清零，随 state dict 输出
# __init__/reset:
self.flag_picked = False
self.flag_filled = False
self.flag_dropped = False
# 状态机转移处：
if near_grasp and gripper_opening < self.gripper_closed_threshold:
    self.dropper_state = "attached"
    self.flag_picked = True
# step() 的 additional_info 加 "picked"/"filled"/"dropped"

# 复合 controller：_check_phase_success 读标志
if self.current_phase == Phase.PICK_DROPPER:
    return self.state.get('picked')
elif self.current_phase == Phase.FILL_DROPPER:
    return self.state.get('filled')
elif self.current_phase == Phase.DRIP:
    return self.state.get('dropped')
```

附带收益：失败归因变准——吸液没成功会报 "fill_dropper failed!" 而不是笼统的 pick 失败。单阶段动作（uncap/ignite）读最终状态 `cap_state=='placed'` 没问题，因为最终态恰是阶段成功条件；**只要原子 controller 的事件序列覆盖多个语义阶段，就必须用粘性标志**。

## 三层架构模板（复合任务：task + controller + config）

### Task 状态机模板（D2 dissolve 已实现并验证）

```python
class DissolveTask(BaseTask):
    def __init__(self, cfg):
        # 读 cfg: paths(prim 路径) / offsets / thresholds
        # _read_translate 读取一次初始位置:
        self.spoon_orig_translate = self._read_translate(spoon_path)
        self.wash_orig_translate = self._read_translate(wash_path)

    def reset(self):
        # 参考点偏移链（全部从 prim 位置推导）:
        table_z = powder_pos[2]
        spoon_grasp_pos = spoon_pos + np.array([0.0, 0.0, 0.0025])      # 手柄中心
        powder_center = powder_pos + [0, 0, 0.0035]
        powder_scoop_pos = powder_center + scoop_insert_offset - spoon_head_offset
        tube_mouth_pos = tube_pos + [0, 0, 0.115]
        tube_transfer_pos = tube_mouth_pos - spoon_head_offset
        wash_grasp_pos = wash_orig + wash_grasp_offset
        wash_lift_pos = wash_grasp + [0, 0, 0.10]
        wash_pour_pos = tube_mouth + [0, 0, wash_pour_lift]
        shake_pos = tube_mouth + shake_offset

    def _update_spoon(self, ...):
        # rest -> (gripper 近 + joint7<0.025) attached
        #      -> [dwell 25f -> PowderOnSpoon reveal]
        #      -> [transfer dwell 25f -> TubeSample reveal, PowderOnSpoon hide]
        #      -> (joint7>0.03) released -> _return_to_origin
    def _update_wash(self, ...):
        # rest -> attached -> [gripper 到 pour 点停留 25f -> TubeWater reveal]
        #      -> released -> _return_to_origin
    def _update_dissolving(self, ...):
        # 关键门控: if not self.dissolved and self.water_added:
        #     gripper 距 shake_pos < 0.15 持续 60 帧 -> dissolved=True, TubeSample hide
        #     obs_done 30 帧
```

### Controller Phase 状态机模板

```python
class DissolveTaskController(BaseController):
    class Phase(Enum):
        SCOOP_POWDER / POUR_WATER / SHAKE / OBSERVE / FINISHED

    def _check_phase_success(self, state):
        # 相位推进条件（上一个动作的 object state 到位）:
        SCOOP_POWDER: spoon_state == 'released'
        POUR_WATER:   wash_state == 'released'
        SHAKE:        state['dissolved'] is True
        OBSERVE:      state['obs_done'] is True

    def _phase_action(self, phase, state, actions, ...):
        SCOOP_POWDER: scoop_controller.forward(
            grasp=state['spoon_grasp_position'],
            scoop=state['powder_scoop_position'],
            transfer=state['tube_transfer_position'],
            grasp_distance=0.006)
        POUR_WATER:   同一 scoop_controller（wash 模式），
            grasp=wash_grasp, scoop=wash_lift, transfer=wash_pour, grasp_distance=0.018
        SHAKE:        shake_controller.forward(current_joint_positions, orientation)
    def _advance_phase(self):
        if self._phase == SHAKE:
            self.shake_controller.set_initial_position(state['shake_position'])
            # 进入 SHAKE 前把摇动点喂给 shake 控制器（位置参数化）
```

### yaml 配置结构

```yaml
name: Level2_Dissolve
task_type: dissolve          # -> task_factory / controller_factory 的注册键
controller_type: dissolve
usd_path: assets/chemistry_lab/lab_004/lab_004.usd
prims:                        # /World/... 路径
  spoon: /World/Spoon
  powder: /World/SamplePowder
  ...
offsets: {...}               # 参考点偏移（可调参数）
thresholds: {...}            # joint7 吸附/释放、dwell 帧数
task:
  max_steps: 4000
cameras: [...]               # 复制现有 level2_*.yaml 的相机配置
robot: {...}                 # 复制现有配置
```

### 验证方法

1. **参考点 pxr 模拟**（本地无需 Isaac Sim）：
   - 加载场景 → 读 prim 世界坐标（`np.array(xf)[3,:3]`，行主序！）
   - 验证每个参考点：在工作区内、与物体几何对齐（勺头距粉末中心 2mm、距管口 0mm）
   - 验证勺头 y 范围完全落入管口 y 范围（无碰撞）
2. **静态检查**：py_compile、yaml 解析、factory 注册键匹配
3. **测试指令**：`python main.py --config-name levelX_<name>`，按检查表核对每个 phase 的日志输出

### 测试预期行为检查表（D2 示例）

- [ ] SCOOP_POWDER: 夹爪抓勺 → 舀取 → 粉末堆出现 → 转移至试管口 → 松开
- [ ] POUR_WATER: 夹爪抓洗瓶 → 提升 → 瓶口对试管口 → 试管中水柱出现 → 放回
- [ ] SHAKE: 夹爪到试管口附近摇动（y 方向摆动）→ 粉末消失（溶解）
- [ ] OBSERVE: 停留观察后任务结束
- [ ] 任何 phase 打印 "xxx failed!" → 把输出发回分析

## 派生场景 Export 绝对路径化（生成场景后必查）

`Stage.Open(源场景) + Export(新路径)` 生成测试场景时，源场景的相对引用（`SubUSDs/materials/xxx.mdl`、`SubUSDs/textures/xxx.png`）被解析成本地绝对路径（`E:/浙江大学/...`）写入导出层 → 服务器加载贴图/MDL 全挂 → 视频红色背景（物体可见但贴图全丢）。lab_001 原文件里是相对路径（grep 二进制可证实），flatten 进派生场景才变成绝对路径。

修复（pxr 遍历 root layer 替换前缀，本地 E:/ 前缀 → 相对 `../`，相对 lab_00X/ 目录解析到 assets/chemistry_lab/）：

```python
from pxr import Usd, Sdf
PREFIX = "E:/<本地仓库根>/LabUtopia/assets/chemistry_lab/"
def fix_spec(spec, count):
    for attr_spec in spec.attributes:
        if attr_spec.typeName == "asset":
            d = attr_spec.default
            if isinstance(d, Sdf.AssetPath) and d.path.startswith(PREFIX):
                attr_spec.default = Sdf.AssetPath("../" + d.path[len(PREFIX):])
                count[0] += 1
        # pxr 26.8：AttributeSpec 无 .timeSamples 属性（坑 12），用方法遍历
        for t in attr_spec.ListTimeSamples():
            v = attr_spec.GetTimeSample(t)
            if isinstance(v, Sdf.AssetPath) and v.path.startswith(PREFIX):
                attr_spec.SetTimeSample(t, Sdf.AssetPath("../" + v.path[len(PREFIX):]))
                count[0] += 1
    for child in spec.nameChildren:
        fix_spec(child, count)
# 改完 layer.Save()，ExportToString 复查无 "E:" 残留
```

注意：写死 PREFIX 只在单机生效，换成"读取任意含本地盘的绝对路径前缀再替换"更通用；服务器上贴图目录（`assets/chemistry_lab/lab_001/SubUSDs/textures/`）必须随 git 同步（git lfs pull）。

## 部署与服务器冒烟

### 部署

1. 本地 `git commit` + `git -c http.proxy= -c https.proxy= push origin main`（全局代理 7897 未开时单次绕过，勿改全局配置）
2. 服务器 `git pull`；**若服务器仓库有本地 patch 残留，先 `git checkout -- <文件>` 再 pull**
3. `pkill -9 -f 'main[.]py'` 清场（残留进程会混写日志、2 集配置跑出 10 集）

### 冒烟命令（2 集，验证用）

```bash
mkdir -p config_smoke && cp config/level2_<任务>.yaml config_smoke/
sed -i 's/max_episodes: 30/max_episodes: 2/' config_smoke/level2_<任务>.yaml
nohup timeout 420 /media/dky/Disk2TB/lizichang/miniconda3/envs/labutopia/bin/python -u \
    main.py --config-name level2_<任务> --config-dir config_smoke --headless --no-video \
    > /tmp/dtest/run.log 2>&1 < /dev/null &
```

- `--headless`（服务器无显示；main.py 已改 args.headless 生效）；`--no-video` 只用于冒烟，正式跑删掉它才会存视频
- `python -u` 必须（重定向到文件时 print 块缓冲，看不到业务输出）
- `--config-dir` 必须相对路径（Hydra 限制，绝对路径报 Exception）
- plink 启动命令会挂住（nohup 特性）——接受超时，**分开连接轮询日志**；环境是 labutopia 不是 labvla
- pkill 与启动命令合并到一条 plink 会返回 128——**分步执行**（pkill 单独一条连接）
- 冒烟后删 config_smoke（防正式跑用错配置）

### 成功判据（全过才算通）

1. 业务日志出现完整成功链（dropper：`Dropper attached! Switching to fill...` → `filled` → `Drip success!`；`Success Rate = 2/2`）
2. 进程自然退出（`ps aux | grep -c '[m]ain.py'` = 0）
3. `outputs/collect/<日期>/<任务>/dataset/episode_*.h5` 存在

### 失败诊断：先拿数据，不猜

一次失败可能多因叠加（材质路径 + 判定逻辑曾叠加）。**先注入 debug 日志跑一次拿全数据**，再二分定位：

- 注入点：task `_update_<obj>` 每帧写 `T<时间> st=状态 j7=夹爪值 gpos=TCP grasp=抓取点 d3d=距离 nG=判定`；controller `forward` 每帧写 `C<时间> ev=事件 t=累计 gpos=TCP grasp/dip/tgt=参考点`。**T 行必须同时记录被操作物体的实际位置（dropper=(x,y,z)）**——kinematic 跟随 bug（跟随只在 attached 分支、进入 squeezed 后物体冻结悬空）只有看物体实际坐标才能发现，只看 TCP/joint7 判定全绿照样漏；抓取点是否落在预期部位区间（胶头 z≈0.13）也要靠 T 行核对
- 判定三分法：
  - **物理/控制没到位**：TCP 没到参考点（d3d 大）、joint7 没到阈值、事件推进异常
  - **判定逻辑错**：TCP/joint7 数据全对、状态机演化完整（rest→attached→filled→dropped），但复合 controller 仍报失败 → 查 _check_phase_success 的判定条件（粘性标志！）
  - **跟随丢失（视觉错但判定过）**：状态机/TCP/joint7 全对、任务判定"成功"，但物体实际位置冻结（如悬在瓶口不动）→ 查 kinematic 跟随是否覆盖整个吸附期间（dropper 曾只在 attached 分支跟随，进入 squeezed 后冻结；修复：跟随提为状态机公共段，覆盖 attached/squeezed/filled/dropped 全程，released 才复位）
- 诊断完还原 patch（`git checkout --`）再 pull 正式修复

验证：`python -m py_compile <file>`；stub cspace_controller 走查每个事件进入条件（阈值/时长/夹爪值）与 task 检测区间对齐。

## 机械臂 IK 控制与到位判定（flametest 已验证）

焰色反应是长程复合任务（11 phase、8+ 抓取/落座/滴加），机械臂控制层的三条已验证经验：

**① IK warm start 与坏分支（"乱动"根因）**。近奇异抓点（match/cap/stopper 低 z）用固定 home warm start 时，Lula `compute_inverse_kinematics(frame_name='right_gripper', warm_start=固定home)` 偶发选出"FK 位置摆到目标 17cm 外"的坏分支，臂朝错误方向猛甩后 force-done——表现为"抖动 + 夹不住"。解法（flametest_controller `_solve_ik_verified`）：依次尝试当前关节（连续性 → 段间平滑、消除分支跳变）与固定 home，且解出后跑 Lula FK 核对，FK 距目标 >6mm 的解拒绝：

```python
def _solve_ik_verified(self, target, cur7):
    for ws in (cur7, self._ik_home):          # 当前关节优先，固定 home 兜底
        try:
            ik, ok = self._ik_solver.compute_inverse_kinematics(
                frame_name="right_gripper",
                target_position=target,
                target_orientation=self.orient,
                warm_start=np.asarray(ws, dtype=float))
        except Exception:
            continue
        if not ok or ik is None:
            continue
        ik = np.asarray(ik, dtype=float)
        fk_pos, _ = self._ik_solver.compute_forward_kinematics("right_gripper", ik)
        err = float(np.linalg.norm(np.asarray(fk_pos) - target))
        if err < 0.006:                         # FK 距目标 < 6mm 才接受
            return ik
    return None
```

**② 到位冻结判定（"固定偏移 + 夹不起来"根因）**。v34b"进圈即冻"用圆柱条件（z<1.5cm 且 xy<3cm），垂直下探时会于目标上方 ~1.3cm 提前冻结——夹爪与器材固定偏移、手指在空中合拢。v43 改为真正到位：3D 距离 <1cm 且连续 3 帧才冻结，冻结后保持关节不再追 IK（近奇异区同一 TCP 可被多个 IK 分支到达、关节永远收敛不到单一解，冻结即稳住抓点让 task attach 判定通过）：

```python
dist3d = np.linalg.norm(gripper_pos - seg["pos"])
if dist3d < 0.010:
    self.arrived_frames += 1
else:
    self.arrived_frames = 0
if self.arrived_frames >= 3:
    self._seg_hold_joints = joints[:7].copy()   # 冻结关节，等 dwell 结束
```

**③ RMP 到位慢（长期问题，先隔离归因）**。完整运行大量 seg force-done（gripper 卡 dist 0.2-0.3 不收敛）时，先用 diag 脚本逐帧跟踪隔离归因再动代码。`scripts/diag_rmp.py`（RMP_FRAMES=300 每目标记录最小距离 + 单调性）已知结论：FAIL 的都是低 z+远 x/y 目标、单调缓慢收敛非卡死；与刚体碰撞无关（贴刚体的 hcl_mouth 收敛、远离刚体的 match 反而不收敛）。此类"慢收敛"确认为已知长期问题后（见 memory `flametest-v24-state`），不要为它回退已验证的刚体/IK 改动。

**④ 垂直段约束（v46，修"斜着拿/穿模"，机械臂"按要求移动"的关键）**。单次解 IK + 每帧关节钳制的实现，joint 空间插值会把 TCP 拉成弧线——垂直下探/提出时斜着走、带抖动（用户报"斜着拿、穿模"）。v46 垂直约束：MoveAction 首帧检测"起点 xy 与目标 xy 偏差 <1.5cm"即判定垂直段——**xy 锁死目标值、z 每帧推进 VZ_STEP、逐帧沿这条竖直线重解 IK**（cur7 warm start + FK 验证），TCP 走严格直线。**调参关键 VZ_STEP=0.002**：每帧 z 步长必须小到 `MAX_JOINT_DELTA=0.015` 钳制不触发。初版 0.008 → z 步大 → 所需关节重配超过钳制 → 钳制后 TCP 横向拖滞 4~9cm（diag 轨迹 CSV 证实是"欠追踪累积"不是单帧尖峰）；0.002（≈0.12 m/s @60Hz）→ 钳制不触发 → dropper descend/lift 横向 0.7mm/0mm、cap 5mm/3.8mm。**通用规则：逐帧逼近的 target，每帧变化量必须小于钳制量所能跟上的幅度。**

```python
if not self._solved:
    self._solved = True
    self._vertical = float(np.linalg.norm(gp[:2] - self.pos[:2])) < 0.015
    if self._vertical:
        self._goal_z = gp[2]          # xy 锁死目标值，z 从当前开始
    else:
        self._ik_target = engine.solve_verified(self.pos, cur)
# 垂直段：每帧沿 z 线推进 VZ_STEP 并重解 IK（cur7 warm start，TCP 贴直线）
dz = self.pos[2] - self._goal_z
step = VZ_STEP if dz > 0 else -VZ_STEP
self._goal_z += step
tgt = np.array([self.pos[0], self.pos[1], self._goal_z])
ik = engine.solve_verified(tgt, cur)
cmd = cur + np.clip(ik - cur, -engine.MAX_JOINT_DELTA, engine.MAX_JOINT_DELTA)
```

**⑤ 跨元动作夹爪状态传播（修"铂丝夹紧后爪子松开"）**。每个元动作实例 `grip_target` 从 GRIP_OPEN 起步、只被自己的 GripAction 更新。物体跨多个元动作持握（铂丝跨 ④蘸酸→⑤灼烧→⑥循环→⑦冷却→⑧蘸粉→⑨显色）时，⑤⑥⑦⑨ 无 GripAction → 夹爪被命令张开，物体靠 task"开爪且近抓点"双条件释放判定不满足 → 吸附挂空（用户见"夹紧后又松开、物体还吸着"）。**持握状态的生命周期跨出单个元动作时必须提升到元动作之外**：controller 在元动作切换处复制夹爪状态，并逐项核对各元动作末尾 grip_target（⑤⑥⑦⑨=GRIP_WIRE 持丝、归架段显式 open 才到 GRIP_OPEN，全链无误夹）：

```python
if meta.is_done():
    self._meta_idx += 1
    if self._meta_idx < len(self.meta_actions):
        self.meta_actions[self._meta_idx].grip_target = meta.grip_target
```

**⑥ 提出高度 = 被持物最低点 > 沿途最高障碍（修"吸液后斜着穿瓶"）**。滴管吸完酸只提 2cm 就横向平移 → 嘴仍低于瓶口，穿模且看似斜着取出。滴管嘴 = TCP - 0.119（HELD_OFFSET.z），瓶口 0.877 → 横移前 gripper z 须 ≥ 1.0（实际提 H=1.15，嘴 1.031，高出 15cm）。**通用：横向平移前，被持物体最低点（TCP - HELD_OFFSET.z）必须高于沿途最高障碍物 + 余量。**

## 刚体落座与防抓取闪现（flametest 已验证）

**① 真刚体凸包不建模口洞 → 落座顶飞**。瓶塞/灯帽转真刚体（fix 脚本 `make_stopper_cap_rigid`：RigidBodyAPI+CollisionAPI+physics:approximation=convexDecomposition+contactOffset 0.002+restOffset -0.001，默认 kinematic）后，瓶口凸包碰撞不建模口洞（凸包鼓出盖住口），真动态落座把瓶塞顶飞（diag_rigid 实测 err=0.099，z 顶到 0.951）。修复：流程期保持 kinematic（每帧 teleport 不被物理覆盖），落座用"盖到位后 kinematic 锁住"折中，成功判定读物理位姿（LiquidMixing 式）：

```python
def _verify_settle(self, name, expected):
    pos = self._get_rigid_world(name)      # 读真刚体 mesh 物理世界位姿（带 LOCAL_GEOM_OFFSET 补偿）
    tol = self.RIGID_SETTLE_TARGET[name][1]
    err = float(np.linalg.norm(pos - expected))
    return err < tol                       # 容差 0.025；teleport 未生效/被撞偏会 WARN
```

**② 抓取平滑 + 连续近窗门禁（消除悬空合爪 / 闪现吸附 / 路过误抓）**。

- `_ease_obj_world`（k≈0.18）：夹爪开始合拢且进入近窗时，物体几何中心逐帧向持物位 `gripper + HELD_OFFSET` 平滑移动，消除"静止到闭合瞬间再 teleport"的闪现吸附。只在 near 时 ease（合爪未遂不把物体拖离原位）。
- `GRASP_NEAR_FRAMES`（3）：连续近窗计数——非近窗即清零、非最近物体即清零、有物体附着即清零，>=3 才允许附着。防臂从物体旁"路过"误抓（P4 抓火柴路过灯帽曾误抓 cap）。

```python
self._grasp_near_frames[name] = self._grasp_near_frames[name] + 1 if near else 0
if near and gripper_opening < self.gripper_open_threshold:
    self._ease_obj_world(name, gripper_pos + self.HELD_OFFSETS[name])
if (near and self._grasp_near_frames[name] >= self.GRASP_NEAR_FRAMES
        and gripper_opening < closed_thresh):
    obj["state"] = "attached"
```

验证：`scripts/diag_grasp.py`（模拟 RMP 偏移 ~3cm 的抓取，判据 max_jump<0.010 且 attached）；`scripts/diag_rigid.py`（三个落座物理位姿 err≈0）。

## 焰色反应整实验模板（v44 分层：小动作 → 10 元动作 → 整个实验）

大任务 = "**整个实验 = 10 个元动作顺序执行**"（比"一个 phase = 一个元动作"更高一层）。三层分层（v44 定型，用户两次纠正），**机械臂按要求移动的全部经验（①②③④⑤⑥）都在这一层**：

| 层 | 位置 | 职责 |
|---|---|---|
| 小动作（运动原语） | `controllers/atomic_actions/flametest/` | IkMotionEngine + MoveAction / GripAction / HoldAction（Lula IK 驱动） |
| 元动作 | `controllers/flametest_meta_actions/` | **一类一文件**（open_hcl_stopper / drip_hcl_acid / ignite_lamp / dip_wire_acid / burn_clean / repeat_dip_burn / cool / dip_powder / burn_stain / extinguish），每个 = 一串原子动作 |
| 整个实验 | `controllers/flametest_controller.py` | 瘦排序器：实例化 10 元动作、按序 forward、is_done 切换 + 夹爪状态传播（⑤）、全完成 → success |

- **小动作 3 类 = 9 个运动原语的参数化实例**：`approach=Move((pt.xy,H))`、`descend=Move(pt)`、`translate=Move(pt)`、`lift_vertical=Move((pt.xy,z),dwell)`、`dip_hold=Move(pt,n)`、`close=Grip(grip,n)`（原地合爪，臂不动）、`open=Grip(GRIP_OPEN,n)`、`settle=Hold(n)`、`hold=Hold(n)`。共享 IkMotionEngine（Lula IK + FK<6mm 验证 + MAX_JOINT_DELTA 钳制 + 到达冻结）。
- **元动作 = BaseMetaAction 子类实现 `_build_actions()` 返回原子动作列表**；`forward(state)` 顺序推进、`grip_target` 内部持久（GripAction 完成时记 width，Move/Hold 每帧发送）；`reset()` 从头重跑整串（序列确定，非运行时循环，reset 语义与其它元动作一致）。工厂函数 `mv(pos,dwell=0)` / `grip(width,dwell=25)` / `hold(n)`。
- **controller 只做"整个实验"**：`_step_collect` 对当前元动作 `forward`，`is_done()` 后 `self._meta_idx += 1` 并传播 grip_target（⑤）；`is_success()` = 全部跑完。坐标常量集中 `constants.py`（H/SETTLE/GRIP_*/抓点）。
- **为什么这样分**：10 类各一文件高内聚可读；每个元动作组合一组可复用的小动作；controller 瘦到只剩排序；跨元动作共享状态（夹爪、坐标）在 controller/constants.py 统一管理，不散落。
- **验证（完整运行 ~285s，exit 0）**：10 元动作全过、0 force-done、0 IK FAIL、0 settle WARN、`success=True ignite=True stain=True extinguish=True`；垂直段铁证看 freeze 行 `gripper=[x,y,z]` 的 xy 与目标逐位相同。环境注意：`PYTHONUNBUFFERED=1`（否则 print 丢失）、GPU 需提权、`--config-name level2_FlameTest` 显式传。
