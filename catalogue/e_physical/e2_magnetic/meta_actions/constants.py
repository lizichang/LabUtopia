"""E2「磁性检测」元动作共享常量：坐标 / 抓取 / 高度 / 朝向（与 e2_magnetic.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-26 pxr 读 e2_magnetic.usd 世界包围盒（gen_e2_scene.py verify）。
坐标系与 d2s/flametest/e1 一致（复用 IkMotionEngine / BaseMetaAction，Lula IK 驱动）。

2026-08-26 用户简化设计：待测固体已预先取出铺在表面皿上（DishPowder 常显），删掉
药匙/试管架/样品瓶/瓶盖 → 只剩「夹磁铁 → 下探检测 → 归位」两个元动作。磁铁
100×15×15mm 平放条形磁铁（长轴 X），手指朝下（默认朝向）竖夹 ±Y 面。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_MAGNET = 0.0075    # 磁铁 15mm 宽 → 半宽 7.5mm（开度 = 2×值 = 15mm，手指朝下夹 ±Y 面）

# —— 表面皿（/World/SurfaceDish (0.56,0.20)，皿顶 0.8066；待测固体已铺上，常显）——
DISH_XY = (0.56, 0.20)
DISH_TOP_Z = 0.8066
DISH_POWDER_TOP_Z = 0.8106   # 皿上待测固体粉末层顶（加厚 4mm，明显可见，预铺常显）

# —— 磁铁（/World/BarMagnet (0.40,0.20) 平放，100×15×15mm，长轴 X，底 0.80 顶 0.815）——
# 手指朝下（默认朝向，非 ORIENT_FWD）垂直下探夹 ±Y 面：磁铁是贴台面的扁平 15mm 物体，
# 手指朝下竖直（长 57mm）可从容罩住磁铁、夹 ±Y 面；ORIENT_FWD 横夹在低 z 近奇异 →
# Lula IK 落 home 分支跳变（用户 2026-08-26 报「马上抓到磁铁就跳」），故弃横夹改竖夹。
# 抓点 z=0.83 → 指尖 0.805（离台面 5mm）、覆盖磁铁上段 0.805..0.815。持握 = 纯平移跟随
# （磁铁 translate=底中心 = tool_center + (0,0,-0.03)，抓点处=rest 0.80 零跳变，磁铁挂指尖下）。
MAGNET_XY = (0.40, 0.20)
MAGNET_TOP_Z = 0.815         # 磁铁顶（平放）
MAGNET_GRASP_Z = 0.83        # 抓点（tool_center；手指朝下指尖 0.805 离台面 5mm）
MAGNET_GRASP = (MAGNET_XY[0], MAGNET_XY[1], MAGNET_GRASP_Z)
MAGNET_HELD_OFFSET = (0.0, 0.0, -0.03)    # 磁铁 translate(=底) = tool_center - 0.03
MAGNET_PRE_Z = 0.90          # 预抓/预放中转高度（磁铁上方，垂直下探前的中间节点）
MAGNET_DETECT_GRASP_Z = 0.86 # 检测：磁铁底 = 0.86-0.03 = 0.83，粉顶 0.8106 上 ~19mm（留颗粒上浮空间）
MAGNET_DETECT_DWELL = 120    # 检测停留帧（~2s @60Hz，磁性动画期间）
