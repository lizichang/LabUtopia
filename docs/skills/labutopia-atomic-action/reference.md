# LabUtopia 元动作制作参考（reference）

## ScoopController 11 阶段模板（舀取/注水通用）

```python
class ScoopController(BaseController):
    """11 事件状态机。wash 模式：scoop_position 传提升点即可当"抓起后移动到"用。"""
    def __init__(self, name, cspace_controller, events_dt, grasp_distance=0.006,
                 pre_offset_z=0.12, lift_z=0.10, retract_offset=0.05):
        # events_dt 长度必须为 11:
        # [0.004, 0.004, 0.02, 0.03, 0.004, 0.004, 0.04, 0.004, 0.004, 0.05, 0.05]
        ...

    def forward(self, grasp_position, scoop_position, transfer_position,
                current_joint_positions, gripper_position, gripper_control,
                end_effector_orientation, pre_offset_z=0.12, lift_z=0.10,
                grasp_distance=0.006, retract_offset=0.05):
        # event 0: 移动到 grasp 上方 pre_offset_z（xy 阈值 0.01 跳转）
        # event 1: 下探到 grasp_position（3D 阈值 0.01 跳转）
        # event 2: dwell（等待 task 检测 attached：joint7 < 0.025）
        # event 3: 闭合夹爪（grasp_distance）
        # event 4: 抬升 lift_z（3D 阈值 0.01 跳转）
        # event 5: 移动到 scoop_position（3D 阈值 0.01 跳转）
        # event 6: dwell（等待 task 检测"粉末已上勺"）
        # event 7: 移动到 transfer 上方（above transfer）
        # event 8: 下探到 transfer_position（3D 阈值 0.01 跳转）
        # event 9: dwell（等待 task 检测"已转移"）
        # event 10: 打开夹爪 + 保持（task 检测 released: joint7 > 0.03）
```

## ShakeController 模式（含位置参数化修改）

```python
class ShakeController(BaseController):
    def __init__(self, name, cspace_controller, events_dt,
                 initial_position: typing.Optional[np.ndarray] = None):
        # 坑：原始版硬编码 np.array([0.25, 0, 1.0])，必须支持传入
        if initial_position is None:
            self._initial_position = np.array([0.25, 0, 1.0])
        else:
            self._initial_position = np.array(initial_position, dtype=float)

    def set_initial_position(self, position: np.ndarray) -> None:
        """复合任务在进入 SHAKE 阶段前调用，指定摇动发生位置。"""
        self._initial_position = np.array(position, dtype=float)

    # 事件 0-1: 移动到 _initial_position
    # 事件 2-7: 在 y 方向 ±shake_distance(0.1) 摆动
    # 事件 8: 回到中心
```

## events_dt 状态机细节

```python
def forward(self, ...):
    self._t += self.events_dt[self._event]
    if self._t >= 1.0:
        self._event += 1
        self._t = 0.0
    if self._event >= len(self.events_dt):
        return True  # is_done
    # 每事件分支：设置目标位置/夹爪
    if self._event == 0:
        target = grasp_pos + [0, 0, pre_offset_z]
        if abs(cur[0]-target[0]) < 0.01 and abs(cur[1]-target[1]) < 0.01:
            self._event += 1; self._t = 0.0   # xy 阈值提前跳转
    ...
```

- dwell 事件：保持目标不变，`_t` 自然累积到 1.0
- 0.02 → 50 帧 ≈ 0.83s，0.05 → 125 帧 ≈ 2.1s（@60fps）
- task 检测一般需要 ≥ 25 帧窗口，dwell 不能太短

## 夹爪契约速查

| 项 | 值 |
|---|---|
| joint_positions[7]/[8] | 手指距离（小 = 闭合） |
| task 吸附检测 | joint7 < 0.025（kinematic，不看物理） |
| task 释放检测 | joint7 > 0.03 |
| grasp_distance 目标 | 勺子 0.006、洗瓶瓶颈 0.018 |
| 打开指令 | gripper_control 置开，joint 值回弹 > 0.03 |

## 注册与使用

```python
# controllers/atomic_actions/__init__.py 导出
from controllers.atomic_actions.scoop_controller import ScoopController
# 复合 controller 内直接用（无需 factory 注册——原子动作不单独注册）
```

验证：`python -m py_compile <file>`；走查每个事件的进入条件（阈值/时长）与 task 检测条件对齐。
