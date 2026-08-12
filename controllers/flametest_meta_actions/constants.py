"""焰色反应元动作共享常量：坐标 / 抓取 / 高度（与 v17 USD 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
从原 flametest_controller.py 提取整理；新增 STO_DESK / SSTO_DESK 桌面抓取点
（修 bug5：task 近窗 z=0.810，见 tasks/flametest_task._grasp_point）。
"""
import numpy as np

# —— 高度常量 ——
H = 1.15            # 安全高位（跨越桌面障碍）
SETTLE = 5          # 到点 settle 帧数

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_STOPPER = 0.0126
GRIP_DROPPER = 0.004
GRIP_MATCH = 0.0015
GRIP_WIRE = 0.0055
GRIP_CAP = 0.0185

# —— HCl 瓶（0.12,-0.28）——
STO_GRASP = (0.1200, -0.2800, 0.8770)   # 瓶口抓瓶塞
STO_SIDE = (0.1600, -0.2400, 0.8770)    # 桌面旁侧放置位（旧逻辑高度）
STO_DESK = (0.1600, -0.2400, 0.8100)    # 桌面再抓取（修 bug5：task 近窗 z=0.810）

# —— 滴管（试管架）——
DROP_GRASP = (0.5070, -0.0420, 0.9310)
DROP_XY = (0.5070, -0.0420)
DROP_LIFT = 1.07

# —— 样品瓶（-0.05,0.30）——
SSTO_GRASP = (-0.05, 0.30, 0.8770)
SSTO_SIDE = (-0.01, 0.26, 0.8770)
SSTO_DESK = (-0.01, 0.26, 0.8100)

# —— 火柴 ——
MATCH_GRASP = (0.8868, 0.5939, 0.8150)
MATCH_HIGH = 1.05

# —— 铂丝 ——
WIRE_GRASP = (0.5456, -0.0417, 0.9770)
WIRE_XY = (0.5456, -0.0417)
WIRE_LIFT = 1.12

# —— 酒精灯 / 火焰（LAMP_POS=(0.5132,0.5256,0.80)）——
IGNITE = (0.6026, 0.5256, 0.9005)       # 火柴触灯芯
FLAME_APPROACH = (0.4532, 0.5056, 1.12)
FLAME_HOLD = (0.5132, 0.5256, 1.088)
COOL_POS = (0.4332, 0.5256, 1.15)

# —— 表面皿（0.5174,0.2407,0.80）——
DISH_DRIP = (0.5174, 0.2407, 0.97)      # 滴酸（在皿上方停留）
ACID_DIP = (0.5174, 0.2407, 0.972)      # 铂丝蘸酸

# —— 灯帽（桌面 rest 0.8155 → 抓近顶 0.824）——
CAP_GRASP = (0.6132, 0.5456, 0.8240)
CAP_HIGH = 1.00
CAP_BURNER = (0.5132, 0.5256, 0.92)     # 盖灭（扣灯口）

# —— 蘸液/蘸粉深度 ——
HCL_DIP = (0.12, -0.28, 0.95)           # 滴管吸 HCl
POWDER_DIP = (-0.05, 0.30, 1.015)       # 铂丝蘸样品粉
