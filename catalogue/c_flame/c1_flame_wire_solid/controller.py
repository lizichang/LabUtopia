"""C1 焰色反应（铂丝蘸取固体样品）—— 控制器。

已有实现（不迁移，re-export 复用）：`controllers/flametest_controller.py` 的
FlameTestTaskController，整个实验 = 顺序执行 10 个 IK 元动作（v44 分层：
atomic_actions/flametest/ 的小动作 + flametest_meta_actions/ 的 10 个元动作）。

跨元动作的夹爪目标（grip_target）由 controller 传播（修 bug6），
完成判定见 `_step_collect` 的 meta 顺序推进。
"""

from controllers.flametest_controller import FlameTestTaskController

__all__ = ["FlameTestTaskController"]
