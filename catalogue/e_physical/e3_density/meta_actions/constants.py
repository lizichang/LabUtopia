"""E3「密度测定」元动作共享常量：坐标 / 抓取 / 高度 / 朝向（与 e3_density.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-28 用 pxr 读 e3_density.usd 世界包围盒（gen_e3_scene.py verify）。
坐标系与 e1/d6/d7 一致（复用 IkMotionEngine / BaseMetaAction，Lula IK 驱动）。

动作流程（2026-08-28 v2 加天平完整测密度；量筒预置天平称盘上不动、样品瓶预开盖、
移液管压缩 185mm 后抓点改为尖端上方 0.09m）：
  PickPipette → DrawPipette → TransferPipette → ReturnPipette
移液管全程竖直平移（手指朝前 ORIENT_FWD 侧面横夹管身），task 只对移液管做平移跟随。
"""

# —— 高度 / 停留 ——
# 安全高位（跨越桌面障碍的水平平移高度）。移液管尖端 = H − PIPE_TIP_TO_GRIP = 1.11，
# 高于量筒口 0.995 约 11.5cm，水平平移时尖端不撞任何容器口。
H = 1.20
DRAW_HOLD = 60         # 吸液停留（模拟挤压洗耳球吸满，1s）
DISPENSE_HOLD = 90     # 放液停留（模拟挤压洗耳球排出，1.5s）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_PIPETTE = 0.0035  # 移液管 Ø7mm → 半宽 3.5mm（约定：开度=2×值，同 d2s GRIP_TUBE=Ø/2）

# —— 夹爪朝向 ——
# 手指朝 +X 朝向 camera1（同 d2s ORIENT_FWD，pxr 验证 attach 零跳变）。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)

# —— 移液管（/World/Pipette，尖端在 (0.22,0.17,0.82) 架孔内；顶 1.005）——
# 抓点 = 尖端上方 0.09m（管身 z0.02..0.14 中段，洗耳球底 0.96 下 5cm 可握管身段），
# 架内抓点 z=0.91。
PIPE_XY = (0.22, 0.17)
PIPE_TIP_Z = 0.82             # 架内尖端 z（架底座顶 0.82）
PIPE_TIP_TO_GRIP = 0.09
PIPE_GRASP_Z = PIPE_TIP_Z + PIPE_TIP_TO_GRIP    # 0.91
PIPE_GRASP = (PIPE_XY[0], PIPE_XY[1], PIPE_GRASP_Z)
PIPE_REST_POS = (PIPE_XY[0], PIPE_XY[1], PIPE_TIP_Z)

# —— 样品瓶（/World/SampleBottle (0.40,0.00)，口 z=0.87，液面 0.831 底 0.803）——
# 吸液尖端入液：目标 z=0.815（液面下 16mm、不触底 0.803），下探冻结裕量 1cm 后
# 尖端最坏停在 0.825 仍浸没（0.825 < 0.831）。
BOTTLE_XY = (0.40, 0.00)
BOTTLE_MOUTH_Z = 0.87
BOTTLE_DRAW_TIP_Z = 0.815

# —— 量筒（/World/GraduatedCylinder 在天平称盘上 (0.40,0.17)，口 z=0.995，
#      5mL 液面 0.921）——
# 放液尖端伸进筒口：目标 z=0.95（口下 4.5cm、5mL 液面 0.921 上方 2.9cm），
# 冻结裕量后最坏 0.96 仍低于筒口 0.995。
CYL_XY = (0.40, 0.17)
CYL_MOUTH_Z = 0.995
CYL_DISPENSE_TIP_Z = 0.95

# —— 密度读数（天平+量筒法 ρ=Δm/5mL，天平前面板屏贴图读数）——
# 密度为任意数值输入（config experiment_result.density type:number，非枚举），
# task 运行时用 PIL 烘焙 balance_result.png（m2+ρ）覆写场景贴图同路径（headless 下改
# 贴图路径不渲染，须覆写同名文件）。m1=35.00g（空量筒）；m2=m1+ρ×5.00，ρ 显示 3 位。
M1_GRAMS = 35.00       # 空量筒质量（固定）
TRANSFER_ML = 5.0      # 移液管转移体积（mL）
DENSITY_DEFAULT = 1.0  # 待测液体密度默认（g/mL）

