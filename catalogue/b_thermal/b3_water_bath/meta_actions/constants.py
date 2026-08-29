"""B3 水浴加热（固体样品熔化）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-29 gen_b3_scene.py verify 输出（pxr 读 b3_water_bath.usd 世界包围盒）：

  加热堆叠（x=0.5286 中心线，铁柱 R180）：烧杯 Beaker (0.5286,0.0029) 底 0.9205 口 1.0361，
    内装水浴水至 1.0055；试管 TestTube (0.5286,0.0029) 底 0.9555（浸入烧杯水 5cm）口 1.1088
    （试管夹 TestTubeClamp 夹上部 z1.0589..1.0876 固定铁柱）；酒精灯 AlcoholLamp 同轴灯芯顶 0.9005
    （火焰底 0.891、外焰 apex 0.9183 碰石棉网底）；铁架台钩顶 ~1.227（B3 无温度计，钩闲置）
  阶段A：玻璃皿 SurfaceDish (0.40,0.28) 顶 0.8065；固体样品白颗粒 ×2 SolidSample (0.39,0.28)/
    SolidSample2 (0.41,0.28) 叠皿上（中心 0.810）；火柴 Match (0.40,-0.06) 头朝灯芯（抬高 12mm）

持握约定（同 B2）：固体样品白颗粒（复用 zeolite.usd，Ø10.8×7.3mm 不规则颗粒）竖直夹 = 默认朝向
手指朝下、颗粒中心对齐夹爪（平移沿 tool+X 偏移半高 SOLID_CENTER_OFFSET）；旋转后手指朝前
ORIENT_FWD 水平横夹（像 d2s 夹药匙）。火柴/灯帽 = 纯平移持握（不随夹爪旋转，同 B2）。

B3 与 B2 差别：试管口更高（1.1088 vs 1.0939，因试管浸在烧杯水里），故固体放下高 SOLID_DROP_Z
= 1.135；**无移灯动作**（B3 简化：加热后直接盖帽灭火，灯不移走），CAP_BURNER 直指灯原位
(0.5286,0.0029,0.900)（B2 是灯移 20cm 后的 (0.5286,-0.1971,0.900)）。
"""
# —— 高度 / 停留 ——
H = 1.25
SETTLE = 12         # 到点 settle 帧数

# —— 朝向（引擎 [w,x,y,z] 存储，scipy [x,y,z,w] 读法，同 d2s）——
ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 手指 +X（水平横夹固体/横越）
TRANSIT_Z = 1.15                          # 低空横越高度（压铁架台钩支臂底 ~1.246 之下）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04

# —— 试管（/World/TestTube (0.5286,0.0029)，口 z=1.1088，预夹浸在烧杯水里）——
TUBE_XY = (0.5286, 0.0029)
TUBE_MOUTH_Z = 1.1088

# —— 固体样品（阶段A 放固体入试管；同 B2 沸石放法，白颗粒 Ø10.8mm）——
SOLID_XY = (0.39, 0.28)
SOLID_CENTER_Z = 0.810            # 固体中心（皿顶 0.8065 + 半高 0.0037）
SOLID_GRASP = (0.39, 0.28, SOLID_CENTER_Z)   # 竖直夹抓点 1（夹爪 = 固体中心，默认朝向手指朝下）
SOLID2_XY = (0.41, 0.28)
SOLID2_GRASP = (0.41, 0.28, SOLID_CENTER_Z)  # 竖直夹抓点 2（皿上右，与 1 并排 ±1cm）
GRIP_SOLID = 0.0055               # 合爪开度 = 固体直径 Ø10.8mm / 2
SOLID_CENTER_OFFSET = 0.0037      # 持握矩阵平移：固体底沿 tool+X 偏移半高（中心在夹爪）
SOLID_DROP_Z = 1.135              # 松爪时夹爪 z（固体中心管口上 2.6cm：口 1.1088+0.026）
SOLID_DROP_APPROACH_DX = 0.05     # 两段式放下：先到管口偏 -X 侧 5cm，再水平移 x 到管口正上方
SOLID_STACK_DZ = 0.0073           # 第二颗固体沉底叠加高（管底 Ø11.5mm 只容一颗并排，第二颗叠顶）

# —— 火柴（阶段B 点燃酒精灯，同 B2）——
MATCH_XY = (0.40, -0.06)
MATCH_REST_Z = 0.813
MATCH_GRASP_OFFSET = 0.04
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)  # (0.44,-0.06,0.8145)
GRIP_MATCH = 0.0015
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)
MATCH_LIFT_Z = 0.90
WICK = (0.5286, 0.0029, 0.9005)
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])  # (0.4792,0.0029,0.9005)
MATCH_HIGH = 1.35

# —— 灯帽（阶段C 盖帽灭火；同 B2，但灯不移走 → CAP_BURNER 直指灯原位）——
CAP_CENTER_DZ = 0.0915            # 帽中心到灯底座 z 偏移
CAP_REST = (0.42, -0.01, 0.8155)  # 帽静止位世界中心（盖帽动作在此夹帽）
CAP_GRASP = (0.42, -0.01, 0.824)  # 夹帽点（帽顶 0.8312 下 7mm）
CAP_HIGH = 1.00                   # 高位（取帽/运帽，高于火焰顶 0.9184 清障）
CAP_HELD_OFFSET = (0.0, 0.0, -0.0083)  # 纯平移持握：帽中心 = 夹爪 + offset
CAP_BURNER = (0.5286, 0.0029, 0.900)   # 盖灯口夹爪（灯不移走，原位盖帽；帽中心 0.8917 = 灯z0.8002 + 0.0915）
GRIP_CAP = 0.0185
CAP_CLOSED_THRESHOLD = 0.022
CAP_COVER_NEAR = 0.010            # 盖到位判定：夹爪距 CAP_BURNER < 1cm
CAP_EXTINGUISH_XY = 0.06          # 下落即熄火 xy 门控
CAP_EXTINGUISH_Z = 0.965          # 下落即熄火 z 门控（帽底罩过火焰顶 0.9184 才熄）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_BEAKER_BUBBLES = "/World/BeakerBubbles"   # 烧杯水浴气泡组（加热时逐个 reveal）
EFFECT_TUBE_MELT = "/World/TubeMelt"             # 试管内熔化液柱（前缀 + <色>，sample_phase=melted 时揭示）
