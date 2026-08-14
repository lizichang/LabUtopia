"""D2-S 元动作共享常量：坐标 / 抓取 / 高度（与 d2s_water_solubility.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-14 用 pxr 读 d2s_water_solubility.usd 世界包围盒。
坐标系与 flametest 一致（复用其 IkMotionEngine / BaseMetaAction，RMP 对低 z
下探发散，见 flametest-v24-state：RMP 定论）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数（v46：5→12，合爪前多停稳，attach 近窗更稳）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_TUBE = 0.009   # 试管 Ø19.2mm，半径 0.0096；目标略小于半径让手指压住管壁

# —— 试管架（/World/TestTubeRack，中心 (0.30,0.00)，底座落台面）——
RACK_XY = (0.30, 0.00)
RACK_TOP_Z = 0.914  # 顶层板顶（管底必须高出此值才可安全水平平移）

# —— 试管（/World/TestTube，Ø19.2×153mm，竖立，前排左孔）——
TUBE_XY = (0.2787, 0.1193)   # 前排左孔孔心
TUBE_TOP_Z = 0.9593          # 管口
TUBE_BOTTOM_Z = 0.806        # 管底（坐架底层板顶）
TUBE_CENTER_Z = 0.8826       # 几何中心
TUBE_GRASP_Z = 0.945         # 抓点：管口下 14mm，手指夹在架顶 0.914 之上的露出段
TUBE_HELD_Z = -0.0624        # 试管几何中心相对夹爪 z（中心在夹爪下方 6.24cm）
TUBE_CLOSED_THRESH = 0.012   # attach 判定：停位 0.0096 + 2.4mm 裕量
                             # （坑：阈值必须留 ≥1mm 裕量，停位会漂移，见 flametest v45）

# —— 操作位（试管取出后的放置位，v11 步骤1「竖直下放至操作位后松开」）——
# 待用户确认：建议架前方空地 (0.30, 0.20)，试管竖立、管底贴桌面 0.80。
# OP_DROP_Z = 试管中心(0.80+0.0767=0.8767) - TUBE_HELD_Z(-0.0624) = 0.939
OP_XY = (0.30, 0.20)
OP_DROP_Z = 0.939
