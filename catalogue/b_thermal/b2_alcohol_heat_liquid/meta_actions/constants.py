"""B2 沸点测定（滴加阶段）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-25 pxr 读 b2_alcohol_heat_liquid.usd 世界包围盒
（gen_b2_scene.py verify 输出 + 用户 tmp 布局）：
  加热堆叠（x=0.5286 中心线，R180）：试管 TestTube (0.5286,0.0029) 底 0.9206 口 1.0739；
    酒精灯同轴；铁架台钩顶 (0.5116,0.0029,1.2273) Ø6mm（阶段 D 挂温度计用）
  试管架 TestTubeRack (0.50,0.35) z 0.800..0.917：滴管插左孔 (0.481,0.3496)，
    孔底 0.806（尖嘴底=原点）；温度计插右孔 (0.519,0.3496)（阶段 D）
  样品瓶 SampleBottle (0.40,0.15)（阶段A 新增，去瓶塞后）：瓶口 rim z=0.870，
    瓶内液面 z=0.840（半瓶）；内径 Ø36mm → 液柱 r=0.014
  试管内初始空（TestTubeLiquid 隐藏 h0），滴加后液柱逐滴生长

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，
flametest 同款，见 task.py）：滴管尖嘴 0.13m 吊在夹爪下方。故
  TCP z = 尖嘴 z + 0.13
抓点 = 立放位 + (0,0,0.13)：滴管尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段）。
"""
# —— 高度 / 停留 ——
# H=1.25 安全高位：清铁架台钩顶 1.2273 + 夹爪球 Ø5.5mm 后仍有裕量；水平跨越时
# 不被试管夹/钩/温度计（阶段 D 挂上后）碰头。滴管尖嘴吊在夹爪下 0.13 → 1.12 仍
# 高于堆叠最高件（试管夹 1.053 / 钩 1.2273 之下的试管口 1.0739），安全。
H = 1.25
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm（同 d3l pxr 实测 DropperSample 球体 bbox dx=dy=0.0110，asset 同款）。
# Franka joint 开度 ≈ 指间距一半 → "正好贴胶头面"开度 = 0.011/2 = 0.0055（d3l 已验证）。
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# —— 滴管（/World/Dropper，架左孔 (0.481,0.3496)，孔底 0.806）——
DROP_XY = (0.481, 0.3496)
DROP_GRASP = (0.481, 0.3496, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上可握段）
DROP_REST = (0.481, 0.3496, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 样品瓶（/World/SampleBottle (0.40,0.15)，去瓶塞后口 rim 0.870，液面 0.840）——
SAMPLE_BOTTLE_XY = (0.40, 0.15)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去瓶塞后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840 ✓
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP：
BOTTLE_SQUEEZE_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
SAMPLE_DIP_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                  BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 试管（/World/TestTube (0.5286,0.0029)，口 z=1.0739；初始空，滴加生长）——
TUBE_XY = (0.5286, 0.0029)
TUBE_MOUTH_Z = 1.0739        # 管口世界 z
# 滴加：尖嘴抬到管口上方 25mm（d3l 用户反馈同款：沉入管口内液滴下落被管壁挡住看不清）
TUBE_DROP_RAISE = 0.025
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1],
                 TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)        # ≈1.2289

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TestTubeLiquid"   # 管内液柱（初始隐藏 h0，滴加逐滴生长）
EFFECT_DROPPER_FILL = "/World/DropperFill"    # 滴管尖内截锥液柱（吸液后显示，跟随尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"    # 挤胶头滴落时从尖嘴掉落的液滴（task 动画坠落）
