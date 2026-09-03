"""D5 蒸馏分离（预组装装置 + 机械臂仅点火收集）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：gen_d5_final.py 打印 + 定稿 usd 手工测量（2026-09-03，忠实用户 Isaac tmp
组装件）。装置集群在东侧：酒精灯/蒸馏烧瓶/冷凝管/接液瓶 x≈0.53-0.95、y≈0.04-0.52。
机械臂底座由用户定 = (0.18,0.12,0.71)（可达：火柴抓点 ~0.44m、点火 ~0.61m < 0.855m 臂展）。

火柴原点 = 火柴资产 = **尾端**（−X 端，非中心；本资产局部 x0 在尾端）：
  /World/Match translate (0.5295,0.2918,0.8043) = 尾端世界位，头朝 +X。
  棒体沿 +X 长 0.083，头 0.081..0.0978 → 世界 x[0.5295,0.6273]（y[0.2890,0.2946]，
  z[0.8030,0.8086]）。夹爪竖直夹杆身中部（纯平移持握，手指朝下默认朝向），火柴不随爪旋转，
  全程水平头朝 +X。抓点沿棒 0.048（仍木杆上），头前端在夹爪 +X 0.0498 处。

点火走廊：夹爪低位运移 z 固定 0.899（> 灯玻璃体顶 0.8897 + 火柴自身 z 半厚，< 烧瓶最低
0.9091 之下 1cm）。火柴棒前端 max x=0.6273 < 灯体 west 边 0.6545 → 先在火柴行 x=0.5775
纯 +Y 进到灯芯行 y=0.4610（西侧空档），再沿 y=0.4610 纯 +X 把火柴头推进灯芯 WICK，两段
直角，不会刮灯体/冷凝管（占用探测 x[0.58,0.72] y[0.28,0.50] z[0.88,0.912] 只有灯芯+烧瓶底）。

本实验仅 1 元动作：
  ① LightFlamePass  拿火柴点燃酒精灯（两段直角路径；火柴头触灯芯 → task 点火）。
蒸馏现象（加热→沸腾→冷凝→馏出液收集）由 task.py 驱动，机械臂点燃后不再动作。
"""
# —— 高度 / 停留 ——
H = 1.35            # 安全高位（火柴列无遮挡；B2/D9 同款）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_MATCH = 0.0015      # 火柴杆 Ø3mm / 2（同 B2/D9）

# —— 酒精灯 / 灯芯（/World/AlcoholLamp，tmp 实测）——
LAMP_XY = (0.6981, 0.4610)
WICK = (0.6981, 0.4610, 0.9007)     # 灯芯顶（点火触发点）

# —— 火柴（/World/Match 尾端原点 (0.5295,0.2918,0.8043)，头朝 +X，实测）——
MATCH_XY = (0.5295, 0.2918)
MATCH_REST_Z = 0.8043                # 尾端 z（棒体底 z 0.8030，中心线高 0.0015）
MATCH_GRASP_ALONG = 0.048            # 抓点距尾端沿棒长（杆身中部，头前伸 0.0498）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_ALONG, MATCH_XY[1], MATCH_REST_Z + 0.0015)
MATCH_HELD_OFFSET = (-MATCH_GRASP_ALONG, 0.0, -0.0015)  # 火柴原点(尾端) 相对夹爪
MATCH_TIP_OFFSET = (0.0498, 0.0, 0.0)    # 头前端相对夹爪（+X）

# —— 低位运移 / 点火（z=0.899：灯体顶 0.8897 上 1cm，烧瓶底 0.9091 下 1cm）——
LIFT_IGN_Z = 0.899
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], LIFT_IGN_Z)   # (0.6483,0.4610,0.899)
MATCH_HIGH = H                   # 高位接近

# —— 火焰组（gen_d5_final 建的 B5 水滴形组；task 每帧写组 scale/rotateXYZ flicker）——
FLAME_GRPS = ("/World/flame_outer_grp", "/World/flame_inner_grp")
FLAME_PRIMS = ("/World/flame_outer_grp/flame_outer",
               "/World/flame_outer_grp/flame_outer_sphere",
               "/World/flame_inner_grp/flame_inner",
               "/World/flame_inner_grp/flame_inner_sphere")
