"""D2-S 元动作共享常量：坐标 / 抓取 / 高度 / 朝向（与 d2s_water_solubility.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-14 用 pxr 读 d2s_water_solubility.usd 世界包围盒 +
Spatula/body mesh 局部点云（z 分带宽度剖面，判明勺头/柄杆分布）。
坐标系与 flametest 一致（复用其 IkMotionEngine / BaseMetaAction，RMP 对低 z
下探发散，见 flametest-v24-state：RMP 定论）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数（合爪/插入后停稳，task attach 近窗更稳）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_SPATULA = 0.008    # 药匙柄杆 Ø8mm（mesh 实测），目标开度 = 杆径，手指压住杆壁

# —— 药匙朝向（w,x,y,z 四元数，与 ik_engine 约定一致；wxyz scalar-first）——
# 引擎默认（euler [0,π,0]）= 手指朝下 = (0,0,1,0)。药匙持握沿夹爪局部 Z（指侧）：
#   DOWN  勺头吊在夹持点下方 134mm（R_y(π)+t(0,0,0.112) 持握，见 task）
#   HORIZ  R_y(-90°) → 勺头转朝 -X（水平插入粉末）
#   POUR   R_y(-135°) → 勺头朝 -X 下 45°（倾斜倒入，勺尖正好落试管口）
ORIENT_DOWN = (0.0, 0.0, 1.0, 0.0)
ORIENT_HORIZ = (0.7071068, 0.0, -0.7071068, 0.0)
ORIENT_POUR = (0.3826834, 0.0, -0.9238795, 0.0)

# —— 药匙（/World/Spatula，竖插于架前排右孔；勺头在下、柄杆在上）——
# mesh：勺头 z 0.806-0.830（22mm 宽扁平），柄杆 z 0.830-0.963（Ø8mm 圆杆）。
# 原点（xform translate）在柄勺交界 z=0.828；抓点 = 原点上方 0.112m = 柄杆 z 0.94。
# 坐标已按用户 temp_d2s.usd（2026-08-14 二次重排避开 Franka 底座）更新：Spatula (0.6996,0.3611,rotZ-90°)。
SPAT_XY = (0.6996, 0.3611)          # 药匙原点世界坐标（柄勺交界）
SPAT_GRASP_Z = 0.94                  # 抓点 z（柄杆上，架顶 0.917 之上可握段）
SPAT_GRASP = (SPAT_XY[0], SPAT_XY[1], SPAT_GRASP_Z)
SPAT_APPROACH = (0.8196, 0.3611, SPAT_GRASP_Z)   # +X 侧横向接近点（横向夹持，非从上到下）
SPAT_HEAD_DIST = 0.134               # 勺头尖到夹持点距离（勺头方向 = 夹爪局部 Z 指侧）
SPAT_HANDLE_DIST = 0.023             # 柄顶到夹持点距离（反方向）
SPAT_HELD_T = (0.0, 0.0, 0.112)      # 药匙相对夹爪平移（R_y(π) + t(0,0,0.112)）

# —— 表面皿 / 粉末（/World/SurfaceDish (0.6865,0.0402,0.80)，粉末在皿内）——
# 粉丘实测 bbox：x 0.6688-0.7042，y 0.0166-0.064，z 0.8021-0.8141。
# 皿沿（rim）顶 z=0.8066 → 插入 z 必须 > 0.8066（过沿）且 < 0.8141（沉入粉）。
DISH_XY = (0.6865, 0.0402)
POWDER_TOP_Z = 0.8141               # 粉丘顶
POWDER_Z = 0.809                     # 插入 z：勺尖 5mm 沉入粉丘（高于皿沿 2.4mm）

# —— 舀取段（勺头朝 -X，勺尖 = TCP - 0.134X）——
SCOOP_APPROACH = (0.8382, 0.0403)     # 舀取下探/插入起始 TCP（勺尖在粉丘 +X 侧上方）
SCOOP_INSERT = (0.8152, 0.0403)       # 插入终点 TCP（勺尖到粉丘中心 0.687 一带）
SCOOP_LIFT_Z = 0.95                  # 舀取后垂直提出高度

# —— 倾倒段（勺头 45° 朝 -X 下，勺尖正好落管口）——
TUBE_XY = (0.659, 0.48)              # 试管口中心（管口 z=0.9593，随试管新坐标）
POUR_TCP = (0.7538, 0.48, 1.0541)    # 倾倒点 TCP（POUR 朝向下勺尖 → 管口，柄顶避开洗瓶）
POUR_HOLD = 30                       # 倾斜倒入停留帧数
