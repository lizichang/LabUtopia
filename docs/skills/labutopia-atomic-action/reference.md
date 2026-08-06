# LabUtopia 元动作制作参考（reference）

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

实现在 `controllers/atomic_actions/dropper_controller.py`（2026-08 验证通过）。核心：**挤压/松开 = 夹爪距离变化 + dwell，task 按 joint7 区间检测状态**。

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

验证：`python -m py_compile <file>`；stub cspace_controller 走查每个事件进入条件（阈值/时长/夹爪值）与 task 检测区间对齐。
