"""catalogue 动作库共享任务基类。

39 个 v10 动作的 task 统一从这里派生，继承现有 `tasks/base_task.py` 的 BaseTask
（场景装配、相机、material、`get_basic_state_info` 等能力全部复用）。
"""

from tasks.base_task import BaseTask


class CatalogueBaseTask(BaseTask):
    """catalogue 任务基类。

    具体动作的 task 各自实现（AGENTS.md §3.7 复合实验模式）：
    - 每个动过物体的 `_update_<obj>()` 子状态机（rest → attached → dwell → released）
    - 参考点在 `reset()` 里从 prim 位置 + 偏移链推导，**禁止硬编码坐标**
    - 阶段完成需显式门控（如 `water_added`），不能只靠距离阈值
    - `step()` 返回 `get_basic_state_info(additional_info={...})`，经 state dict
      把世界参考点（grasp/scoop/transfer/drip 位置）暴露给 controller

    v10 中反复出现的两个共用运动模板，可在后续动作里沉淀成通用 helper：
    1. **三段式轨迹**：垂直下放夹持 → 竖直抬升至安全中转点（高位）→ 水平平移至
       目标正上方 → 垂直缓慢下放。垂直段 z 推进用 VZ_STEP=0.002（m/帧，
       见 controllers/atomic_actions/flametest/move_action.py），xy 锁死。
    2. **管口悬停精细中转点**：滴管/移液管在试管口上方先悬停再下探，防止撞口。
    """

    # 桌面高度（Z-up，世界坐标）。各动作场景若用相同桌高可统一；不同则按 config 覆盖。
    TABLE_Z = 0.80
