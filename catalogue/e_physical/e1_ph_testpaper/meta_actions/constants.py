"""E1「pH 试纸检测」元动作共享常量：坐标 / 抓取 / 高度 / 朝向（与 e1_ph_testpaper.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-25 用 pxr 读 e1_ph_testpaper.usd 世界包围盒（gen_e1_scene.py verify）。
坐标系与 d2s/flametest 一致（复用其 IkMotionEngine / BaseMetaAction，Lula IK 驱动）。

本实验无手腕翻转：试纸条已默认铺在白瓷板上（不再夹取），玻璃棒竖直平移（蘸取/点触/归位），
故 task 只对玻璃棒做平移跟随（set_object_position），不需要 d2s 的 _T_HELD 旋转合成。
"""

# —— 高度 / 停留 ——
# 安全高位（跨越桌面障碍的水平平移高度）。棒尖（底）= H − ROD_TIP_TO_GRIP（棒挂在夹爪下
# 0.20m），必须高于试管口 0.9593 至少 ~15mm，否则水平平移时棒尖低于管口会穿试管口沿。
# 旧 H=1.15 → 棒尖 0.95 < 管口 0.9593（穿模）；1.20 → 棒尖 1.00，高出 40mm。
H = 1.20
SETTLE = 12         # 到点 settle 帧数（蘸取/点触后停稳）
WAIT_FRAMES = 150   # 等待显色 2.5s（点触后原地 hold）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_ROD = 0.003        # 玻璃棒 Ø6mm → 半宽 3mm（约定：开度=2×值，同 d2s GRIP_TUBE=Ø/2）

# —— 夹爪朝向 ——
# 手指朝 +X 朝向 camera1（同 d2s ORIENT_FWD，pxr 验证 attach 零跳变）。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)

# —— 白瓷板 / 试纸（/World/WhitePlate 中心 (0.46,0.32)；试纸条 7×70×0.5mm 已默认铺在
#    板中央（不再夹取，场景里预制），棒尖点触中央时显示 pH 色斑）——
PLATE_XY = (0.46, 0.32)

# —— 玻璃棒（/World/GlassRod，Ø6×261mm 竖直插架前排右孔，底 z=0.806 顶 z=1.067）——
# 抓点 = 尖端（底）上方 0.20m：蘸取时夹爪须保持在管口(0.9593)之上，棒尖（底）才能伸到
# 液面(0.811)下 14.8cm 的试管底部。抓点 z=1.006（架顶 0.917 上 8.9cm，可握段）。
ROD_XY = (0.319, 0.117)
ROD_BOTTOM_Z = 0.806
ROD_TIP_TO_GRIP = 0.20
ROD_GRASP_Z = ROD_BOTTOM_Z + ROD_TIP_TO_GRIP    # 1.006
ROD_GRASP = (ROD_XY[0], ROD_XY[1], ROD_GRASP_Z)
ROD_REST_POS = (0.319, 0.117, 0.806)            # 棒静止位（translate=棒底，reset 用）

# —— 试管 / 蘸取（/World/TestTube 前排左孔 (0.2787,0.1193)，口 z=0.9593）——
# 待测溶液 1mL 液柱顶 z=0.811、底 z=0.805。棒尖（底）蘸取目标 z=0.808（液面下 3mm、不触底）。
TUBE_XY = (0.2787, 0.1193)
TUBE_MOUTH_Z = 0.9593
DIP_TIP_Z = 0.808
DIP_GRASP_Z = DIP_TIP_Z + ROD_TIP_TO_GRIP         # 1.008

# —— 点触转移（棒尖碰试纸中央 (0.46,0.32)，试纸顶 z≈0.8070）——
# 白瓷板顶 0.806 + 0.5mm 抬高（防共面）+ 试纸 0.5mm → 试纸顶 0.8070。棒尖（底）点触目标。
TOUCH_TIP_Z = 0.8070
TRANSFER_GRASP_Z = TOUCH_TIP_Z + ROD_TIP_TO_GRIP  # 1.0070
