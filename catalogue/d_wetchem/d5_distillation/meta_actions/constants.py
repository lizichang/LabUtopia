"""D5 蒸馏分离（预组装装置 + 机械臂仅点火收集）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：gen_d5_scene.py verify 输出（酒精灯/火柴坐标复用 B2/D9 已验证值：灯
(0.5286,0.0029)、火柴 (0.40,-0.06) 头朝灯芯；三脚架+石棉网+蒸馏烧瓶+温度计+冷凝管+
接液瓶预组装在灯上方，机械臂不碰装置，仅点燃酒精灯，蒸馏现象由 task 现象状态机驱动）。

本实验仅 1 元动作：
  ① LightFlamePass  拿火柴点燃酒精灯（照 B2/D9 逐字：火柴头触灯芯 → task 点火）。
蒸馏现象（加热→沸腾→冷凝→馏出液收集）由 task.py 驱动，机械臂点燃后不再动作。

火柴/灯芯坐标复用 B2/D9 已验证值（灯仍 (0.5286,0.0029)，火柴 (0.40,-0.06)），故机器人
底座须照 B2 的 [0.0,-0.08,0.71]（非 D2-S 的 [-0.15,0.05,0.71]）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.35            # 安全高位（清三脚架环 0.955 / 石棉网 0.956 / 烧瓶支管 1.099 之外）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_MATCH = 0.0015      # 火柴杆 Ø3mm / 2（同 B2/D9）

# —— 酒精灯（/World/AlcoholLamp R180 @ (0.5286,0.0029)，同 B2/D9）——
LAMP_XY = (0.5286, 0.0029)
WICK = (0.5286, 0.0029, 0.9005)     # 灯芯顶（点火触发点）

# —— 火柴（/World/Match (0.40,-0.06,0.813)，头 +X 朝灯芯，抬高 12mm；同 B2/D9）——
MATCH_XY = (0.40, -0.06)
MATCH_REST_Z = 0.813
MATCH_GRASP_OFFSET = 0.04        # 抓杆身 x=0.04
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)    # 头中心相对夹爪
MATCH_LIFT_Z = 0.90              # 低位运移高度
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])   # (0.4792,0.0029,0.9005)
MATCH_HIGH = H                   # 高位接近（同 B2/D9 = 1.35）
