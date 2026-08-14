"""D3-L 酸性试剂滴加反应（液体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-14 pxr 读 d3l_acid_reagent.usd 世界包围盒（gen_d3l_scene.py
verify 输出）：
  TestTubeRack   (0.30,0.00)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  TestTube       (0.2787,0.1193,0.806)  前排左孔，管口 z=0.9593
  DropperSample  (0.281, 0.0788,0.806)  2 排左孔，尖嘴底=原点，胶头顶 z=0.9562
  DropperAcid    (0.319, 0.0788,0.806)  2 排右孔
  SampleBottle   (0.4045,0.3585)  瓶口 rim z=0.870，瓶内液面 z=0.840（半瓶）
  HClBottle      (0.1696,0.361)   瓶口 rim z=0.870，瓶内液面 z=0.840

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，
flametest 同款，见 task.py）：滴管尖嘴 0.13m 吊在夹爪下方。故
  TCP z = 尖嘴 z + 0.13
抓点 = 立放位 + (0,0,0.13)：滴管尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = 手指物理间距，m）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头直径 Ø11mm（pxr 实测 d3l 场景 DropperSample/球体 bbox dx=dy=0.0110，抓点 0.936
# 在球体内）。移动/持握时开合=胶头直径：手指正好贴胶头面——不陷不隔。
#  旧 0.008 → 手指陷进胶头 1.5mm/边（穿模、"歪"）；旧 0.015 → 手指离胶头 4mm（明显间隔）
DROPPER_DIAM = 0.011
GRIP_DROPPER = DROPPER_DIAM # 抓滴管：合爪到胶头直径
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = DROPPER_DIAM# 松胶头吸液后回持握宽=胶头直径（移动全程无缝隙）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# —— 取样滴管（/World/DropperSample，2 排左孔）——
DROP_SAMPLE_XY = (0.281, 0.0788)
DROP_SAMPLE_GRASP = (0.281, 0.0788, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上）
DROP_SAMPLE_REST = (0.281, 0.0788, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 加酸滴管（/World/DropperAcid，2 排右孔）——
DROP_ACID_XY = (0.319, 0.0788)
DROP_ACID_GRASP = (0.319, 0.0788, DROP_SAMPLE_GRASP[2])
DROP_ACID_REST = (0.319, 0.0788, 0.806)

# —— 样品瓶（/World/SampleBottle (0.4045,0.3585)，口 rim 0.870，液面 0.840）——
SAMPLE_BOTTLE_XY = (0.4045, 0.3585)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去瓶塞后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid/AcidLiquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840 ✓
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP：
BOTTLE_SQUEEZE_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
SAMPLE_DIP_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                  BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 酸性试剂瓶（/World/HClBottle (0.1696,0.361)，口 rim 0.870，液面 0.840）——
ACID_BOTTLE_XY = (0.1696, 0.361)
ACID_SQUEEZE_TCP = (ACID_BOTTLE_XY[0], ACID_BOTTLE_XY[1], BOTTLE_SQUEEZE_TCP[2])  # = 1.005
ACID_DIP_TCP = (ACID_BOTTLE_XY[0], ACID_BOTTLE_XY[1], SAMPLE_DIP_TCP[2])          # = 0.960

# —— 试管（/World/TestTube (0.2787,0.1193)，管口 z=0.9593）——
TUBE_XY = (0.2787, 0.1193)
TUBE_MOUTH_Z = 0.9593        # 管口世界 z
TUBE_DROP_INSET = 0.010      # 滴加：尖嘴沉入管口下 10mm → 尖嘴 z=0.9493（管口内）
# 两把滴管共用同一试管口：尖嘴 TCP
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z - TUBE_DROP_INSET + TIP_OFFSET)  # ≈1.079

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"       # 管内液滴（首滴后显示）
EFFECT_PRECIPITATE = "/World/Precipitate"    # 沉淀（cfg.has_precipitate）
EFFECT_BUBBLES = "/World/Bubbles"            # 气泡（cfg.has_bubbles）
EFFECT_DROPPER_FILL = "/World/DropperFill"   # 滴管尖内液体柱（吸液后显示，跟随尖嘴）
