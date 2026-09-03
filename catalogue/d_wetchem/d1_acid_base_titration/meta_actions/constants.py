"""D1 酸碱滴定（P1 加指示剂）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-09-02 pxr 读 d1_tmp.usd 世界包围盒（用户 Blender 手摆真值，
gen_d1_acid_base_titration_scene.py verify 输出）：
  TestTubeRack   (0.4754,0.3861,0.7961)..(0.5609,0.6717,0.9131)（顶板 0.9131）
  Dropper        (0.4997,0.4512)  直立竖插架内，尖嘴底=原点 z=0.80，胶头顶 z=0.9502
  IndicatorBottle(0.2685,0.3925)  瓶口 rim z=0.87（塞已搬出平放桌面，见 gen），
                                   瓶内液面 z=0.828（酚酞近无色，半瓶）
  conical_flask  (0.2751,0.1844)  独立清点 W，瓶口顶 z=0.9645；瓶内预装 NaOH 无色
                                   20mL：液面世界 z=0.812（FlaskNaOH 柱，r0.033 h0.008）

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，
flametest/d3l 同款）：滴管尖嘴 0.13m 吊在夹爪下方。故
  TCP z = 尖嘴 z + 0.13
抓点 = 立放位 + (0,0,0.13)：滴管尖嘴 0.80，抓点 0.93（架顶 0.913 之上，捏胶头 0.915-0.9502
中段；同 d3l 胶头 Ø11/2=0.0055 夹爪开度贴合胶头面）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度；尖嘴 1.02 须 > 锥形瓶口 0.9645）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm（pxr 实测 Dropper/球体 bbox dx=dy=0.0110）→ 贴合开度 0.0055（d3l 实测）
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m

# —— 滴管（/World/Dropper，竖插架内）——
DROP_XY = (0.4997, 0.4512)
DROP_GRASP = (0.4997, 0.4512, 0.80 + TIP_OFFSET)   # = 0.93（架顶 0.913 之上，捏胶头）
DROP_REST = (0.4997, 0.4512, 0.80)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.80）

# —— 指示剂瓶（/World/IndicatorBottle (0.2685,0.3925)，瓶口 rim 0.87，液面 0.828）——
IND_BOTTLE_XY = (0.2685, 0.3925)
IND_MOUTH_Z = 0.870          # 瓶口 rim 世界 z（塞已搬出，瓶可伸入）
IND_LIQUID_TOP_Z = 0.828     # 瓶内酚酞液面（IndicatorLiquid 顶）
IND_DIP_INSET = 0.052        # 浸液：尖嘴沉入瓶口下 52mm → 尖嘴 z=0.818 < 液面 0.828（入液下 10mm）
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.818 入液面下 10mm）的 TCP：
IND_SQUEEZE_TCP = (IND_BOTTLE_XY[0], IND_BOTTLE_XY[1],
                   IND_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
IND_DIP_TCP = (IND_BOTTLE_XY[0], IND_BOTTLE_XY[1],
               IND_MOUTH_Z - IND_DIP_INSET + TIP_OFFSET)      # = 0.948

# —— 锥形瓶 W（/World/conical_flask_93x93x165 (0.2751,0.1844)，瓶口顶 z=0.9645）——
FLASK_XY = (0.2751, 0.1844)
FLASK_MOUTH_Z = 0.9645       # 瓶口顶世界 z
FLASK_NAOH_TOP_Z = 0.812     # 瓶内 NaOH 无色液面（FlaskNaOH 顶）
# 滴加：尖嘴抬到瓶口上方 25mm（同 d3l：滴加时抬高一点，液滴从尖嘴坠落入瓶口可见）
FLASK_DROP_RAISE = 0.025
FLASK_DROP_TCP = (FLASK_XY[0], FLASK_XY[1],
                  FLASK_MOUTH_Z + FLASK_DROP_RAISE + TIP_OFFSET)   # ≈1.1195

# —— 效果 prim 路径（scene 内建，task 动画驱动；几何见 gen_d1_acid_base_titration_scene.py）——
EFFECT_FLASK_NAOH = "/World/conical_flask_93x93x165/FlaskNaOH"     # 锥形瓶内无色 NaOH（初始可见）
EFFECT_FLASK_PINK = "/World/conical_flask_93x93x165/FlaskNaOHPink"  # 同几何粉色（滴加后换显）
EFFECT_IND_LIQUID = "/World/IndicatorBottle/IndicatorLiquid"        # 指示剂瓶内酚酞液（静态）
EFFECT_DROPPER_FILL = "/World/DropperFill"    # 滴管尖内吸上液柱（跟随尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"    # 挤胶头滴落粉球（task 动画坠落，Drop_0..2）
