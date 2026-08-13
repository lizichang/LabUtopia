"""C1 焰色反应（铂丝蘸取固体样品）—— 任务。

已有实现（不迁移，re-export 复用）：`tasks/flametest_task.py` 的 FlameTestTask，
对应 V7 文档 C1 的 13 步完整流程（旋塞开瓶 → 滴酸 → 点火 → 蘸酸灼烧 →
蘸粉显色 → 灯帽盖灭）。场景 `assets/scenes/c_flame/c1_flame_wire_solid/c1_flame_wire_solid.usd`。

焰色由 config 的 `flame_color` 决定（yellow/purple/green/red/orange/blue），
受染判定、灯焰熄灭、落座判定见 FlameTestTask 内的状态机与里程碑门控。
"""

from tasks.flametest_task import FlameTestTask

__all__ = ["FlameTestTask"]
