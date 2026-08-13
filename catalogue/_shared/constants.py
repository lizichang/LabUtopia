"""catalogue 动作库共享常量。

汇总 flametest（v44-v46）已踩坑验证过的通用值，供 39 个动作复用。
坐标系约定：所有坐标 = TCP（right_gripper）世界坐标，米，Z-up。
"""

# —— 桌面/场景 ——
TABLE_Z = 0.80        # 桌面高度（Z-up，与 flametest_task.TABLE_Z 一致）
H = 1.15              # 安全高位（跨越桌面障碍的水平平移高度）

# —— 运动 ——
VZ_STEP = 0.002       # 垂直段每帧 z 推进量（m/帧），必须小于 MAX_JOINT_DELTA 钳制
SETTLE = 12           # 到点 settle 帧数（v46：5→12，合爪前多停稳）
GRIP_OPEN = 0.04      # 夹爪开度（open 宽度）

# —— 夹爪检测阈值（joint_positions[7]/[8] = 手指距离，米，小=闭合）——
GRIPPER_CLOSED = 0.025   # 吸附判定：joint7 < 0.025
GRIPPER_RELEASED = 0.03  # 释放判定：joint7 > 0.03
GRIPPER_SQUEEZED = 0.005 # 挤压判定（滴管排气/吸液）：joint7 < 0.005
# 吸液松开区间：0.005 ~ 0.025（滴管吸液后松开一段）

# —— 动作类型（v10 中已出现的通用操作，随实现补充）——
# 三段式轨迹 / 管口悬停精细中转点的几何推导规则见 _shared/base_task.py 文档。
