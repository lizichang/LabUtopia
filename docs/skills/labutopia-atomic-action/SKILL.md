---
name: labutopia-atomic-action
description: Creates LabUtopia atomic actions / 元动作 (motion controllers) — scooping, shaking, picking, uncapping, pouring — using the events_dt state machine pattern with kinematic grasping contracts. Use when the user asks to 制作元动作/原子动作/新动作, or mentions LabUtopia controllers, atomic_actions, scoop_controller, shake_controller, events_dt, gripper contract, kinematic attach.
version: 1.0.0
---

# LabUtopia 元动作制作

为 LabUtopia 创建可复用的原子动作 controller（勺子舀取、摇动、拾取、开盖、倾倒等），放在 `controllers/atomic_actions/`。

## 工作流

1. **动作分解**：把动作拆成事件序列（移动 → 停留 → 闭合夹爪 → 抬升 → 移动 → …）
2. **选参考实现**：舀取参考 `scoop_controller.py`（11 阶段），摇动参考 `shake_controller.py`，抓取参考 `pick_controller.py`（夹爪查找表）
3. **实现事件状态机**：`events_dt` 驱动，逐事件推进（见下）
4. **注册**：`controllers/atomic_actions/__init__.py` 导出 + 复合任务中直接 import 使用
5. **验证**：py_compile + 事件序列走查 + 与 task 的 kinematic 检测对齐

## 事件状态机（核心模式）

```python
self._t += self.events_dt[self._event]
if self._t >= 1.0:
    self._event += 1
    self._t = 0.0
# 或：位置到位提前跳转（阈值 0.01）
if np.linalg.norm(target - cur) < 0.01 and not self._flag:
    self._event += 1
# is_done: self._event >= len(self.events_dt)
```

- **events_dt 长度必须严格等于事件数**（Scoop 11 个事件 = 11 项）
- dwell 事件 0.02 ≈ 50 帧，0.05 ≈ 125 帧（@60fps）
- 每个事件在 `forward()` 里按 `self._event` 分支设置目标位置/夹爪指令

## 夹爪契约（必须遵守）

- `joint_positions[7]/[8]` = 手指距离（小 = 闭合）
- 目标夹距 `grasp_distance`：勺子 0.006，洗瓶瓶颈 0.018
- **task 的 kinematic 吸附检测只看 joint7 < 0.025**（不依赖物理夹持）——controller 必须真实把夹爪闭到该值以下，否则永远 attached 不了
- 打开阈值 0.03（task 判定释放）

## 常见坑（已踩过）

1. **events_dt 长度与事件数不匹配**：会 IndexError 或提前结束。写完后数一遍。
2. **grasp_distance 与物体几何不匹配**：夹太松 joint7 下不到 0.025，kinematic 吸附永不触发。对照参考：勺头扁 0.006、瓶颈圆 0.018。
3. **位置硬编码**：shake 原始版把位置硬编码为 (0.25,0,1.0)，导致只能在固定点摇。需要位置参数的必须加参数 + setter（`set_initial_position`），不能写死。
4. **阈值跳转误用**：只有平移事件适合 0.01 阈值跳转；dwell 事件保持目标、不跳转。
5. **事件 0 的 xy 跳转**：先平移到位（只看 xy 平面）再进入下一事件，避免 z 方向误差卡死。

## 检查清单

- [ ] events_dt 长度 = 事件数
- [ ] 所有目标点来自参数（无硬编码坐标）
- [ ] 闭合事件的 grasp_distance 与物体匹配，能确保 joint7 < 0.025
- [ ] dwell 时长足够 task 检测（≥ 25 帧）
- [ ] 打开/释放事件存在且阈值 0.03
- [ ] py_compile 通过

## 附加资源

- 完整 ScoopController 11 阶段模板 + ShakeController 模式见 [reference.md](reference.md)
- 夹爪查找表（get_gripper_distance 等）参考 `pick_controller.py`
