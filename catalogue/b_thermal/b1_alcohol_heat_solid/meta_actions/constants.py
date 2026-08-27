"""B1 酒精灯加热（固体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-27 pxr 读 b1_alcohol_heat_solid.usd 世界包围盒
（gen_b1_scene.py verify 输出）：
  酒精灯 AlcoholLamp (0.50,0.0029)：玻璃罩 z[0.80,0.8897]、灯芯顶 wick_top 0.9007；
    灯帽 cap 盖在灯口（Ø37.2×31mm 钟罩，世界中心 (0.50,0.0029,0.8917)）
  火柴 Match (0.40,-0.06) 原点 z=0.813（抬高 12mm），头=asset +X 端朝灯芯（同 B2）
  药匙/皿/粉/试管/试管架 = D2S 逐字（挖粉坐标才不跑偏），挖粉复用 d2s 元动作默认值

挖粉常量直接复用 d2s（SPAT_XY/H/SETTLE/GRIP_OPEN/ORIENT_FWD + 勺/皿/粉/试管坐标）；
本文件只放 B1 专属（酒精灯 / 灯帽 / 火柴 + 点火）。
"""
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.constants import (
    H, SETTLE, GRIP_OPEN, ORIENT_FWD,
    SPAT_XY, SPAT_GRASP, GRIP_SPATULA, SPAT_HEAD_DIST,
    TUBE_XY, TUBE_MOUTH_Z, DISH_XY, POWDER_X, POWDER_TOP_Z,
)

# —— 酒精灯（/World/AlcoholLamp (0.50,0.0029)，玻璃罩顶 0.8897、灯芯顶 0.9007）——
LAMP_XY = (0.50, 0.0029)
WICK = (LAMP_XY[0], LAMP_XY[1], 0.9005)   # 灯芯顶（pxr wick_top 0.9007 差 0.2mm，同 B2 取 0.9005）

# —— 灯帽（/World/AlcoholLamp/cap，盖在灯口；Ø37.2mm×31mm 钟罩）——
# 帽世界中心 (0.50,0.0029,0.8917)。lamp 无旋转、帽中心相对灯原点 +Z 0.09152 恒定
# （帽 ops = translate+rotateXYZ(90)+scale(0.01)，mesh 中心 local (0,9.152,0) → 世界 +Z）。
# 抓点 = 帽中心（火柴/沸石同款：TCP = 物体中心），手指夹帽壁（帽壁 y=±0.0186 在空心灯口
# 内、玻璃壁 y=±0.0435 之内 → 无碰撞；TCP=帽中心时手指不低到帽下，不碰灯体）。
# 持握 = 纯平移：帽世界中心 = 夹爪，task._set_cap_center 写帽 local translate =
#   center - CAP_CENTER_REST（只写现有 translate op，姿态不变）。
CAP_CENTER_REST = (0.50, 0.0029, 0.8917)   # 帽静止中心（盖在灯上，local translate=0）
CAP_GRASP = CAP_CENTER_REST                # 抓点 TCP = 帽中心
GRIP_CAP = 0.0185                          # 合爪开度 = 帽 Ø37.2mm / 2（flametest 同款，贴合帽壁）
# CAP_HIGH：2026-08-27 修「拿起/放下灯帽卡几秒」——原 1.10 无解（Lula 各 warm start
#   FAIL，(0.63,-0.06,1.10) IK 解不出 → 到位判定永不冻结 → 卡 2.3s/2s）。降为 0.95：
#   (0.50,0.0029,0.95) 与 (0.62,-0.13,0.95) 均 0.0mm 可解，帽底 0.9345 仍高于灯顶 0.8897
#   4.5cm（横移不碰灯）。
CAP_HIGH = 0.95                            # 取帽高位（帽底 0.9345 > 灯顶 0.8897，横移安全）
# CAP_ASIDE_XY：2026-08-27 用户「放远一点」——(0.63,-0.06)→(0.62,-0.13)（-y 更远，
#   距灯 18cm>14.5cm；Lula 行程/落位全 0.0mm 可解，落位 z=0.8155 帽底贴台面 0.80）
CAP_ASIDE_XY = (0.62, -0.13)               # 放一边位置（台面右前区更远，帽底贴台面）
CAP_ASIDE_CENTER = (0.62, -0.13, 0.8155)   # 帽放一边静止中心（底 0.80 + 半高 0.0155）
CAP_ASIDE_TCP = CAP_ASIDE_CENTER           # 放帽下探 TCP（= 帽中心，手指夹帽壁落位台面）

# —— 火柴（/World/Match (0.40,-0.06)，头朝灯芯；同 B2）——
# 火柴躺台面 (0.40,-0.06) 头朝灯芯（头=asset +X 端），原点 z=0.813（抬高 12mm 避免夹爪
# collider 扎桌面卡爪）。持握 = 纯平移 offset（火柴全程水平头朝 +X，不随夹爪旋转）。
MATCH_XY = (0.40, -0.06)
MATCH_REST_Z = 0.813            # 火柴原点（杆 -X 端）z
MATCH_GRASP_OFFSET = 0.04       # 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)
GRIP_MATCH = 0.0015             # 合爪开度 = 杆身 Ø3mm / 2
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)        # 头中心相对夹爪（头 x=0.0894 − 抓点 0.04）
MATCH_LIFT_Z = 0.90             # 夹起后低位运移高度（压灯体顶 0.8897 之下）
# 点火：火柴头落在灯芯顶 WICK。夹爪到 IGNITE（灯芯偏 -X 侧 4.94cm，夹爪 0.4506 在灯体
# min x 0.4564 之外），头中心 +0.0494 到 WICK (0.50,0.0029,0.9005)。
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])   # (0.4506,0.0029,0.9005)
MATCH_HIGH = 1.05               # 高位接近
