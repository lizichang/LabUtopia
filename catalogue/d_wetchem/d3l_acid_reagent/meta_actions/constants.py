"""D3-L 酸性试剂滴加反应（液体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-14 pxr 读 d3l_acid_reagent.usd 世界包围盒（gen_d3l_scene.py
verify 输出）：
  TestTubeRack   (0.30,0.00)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  TestTube       (0.2787,0.1193,0.806)  前排左孔，管口 z=0.9593
  DropperSample  (0.2815,-0.1187,0.806)  后排左孔（离试管最远，用户 2026-08-14 调整），
                                         尖嘴底=原点，胶头顶 z=0.9562
  DropperAcid    (0.3202,-0.1187,0.806)  后排右孔
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

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm（pxr 实测 DropperSample/球体 bbox dx=dy=0.0110）。Franka joint
# 开度 ≈ 指间距一半 → "正好贴胶头面"的开度 = 0.011/2 = 0.0055。用户 2026-08-14 实测：
# 0.011 手指离胶头太远（胶头在指间"悬空"，看着松/歪、移动时跟夹爪碰/穿模），
# **减半 0.0055 正合适**（手指正好贴胶头面）。
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# —— 取样滴管（/World/DropperSample，后排左孔）——
DROP_SAMPLE_XY = (0.2815, -0.1187)
DROP_SAMPLE_GRASP = (0.2815, -0.1187, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上）
DROP_SAMPLE_REST = (0.2815, -0.1187, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 加酸滴管（/World/DropperAcid，后排右孔）——
DROP_ACID_XY = (0.3202, -0.1187)
DROP_ACID_GRASP = (0.3202, -0.1187, DROP_SAMPLE_GRASP[2])
DROP_ACID_REST = (0.3202, -0.1187, 0.806)

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
# 滴加：尖嘴抬到管口上方 25mm（用户 2026-08-14：滴加时把滴管抬高一点才看清液滴
# 从尖嘴坠落到管内的过程——原来沉入管口内 10mm，尖嘴被管壁挡住、液滴下落看不清）
TUBE_DROP_RAISE = 0.025
# 两把滴管共用同一试管口：尖嘴 TCP
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)  # ≈1.114

# —— 试管抓取与震荡（步9 拿起试管震荡来回）——
# 试管立插前排左孔 (0.2787,0.1193,0.806)，管口 z=0.9593，管高 0.153m；架顶 0.917。
# 抓管身中段（管口下 14mm）：TCP z=0.9453，管底吊在夹爪下 0.1393m（同滴管纯平移持握）。
# 震荡高度：管底 z=1.09-0.1393=0.9507 清架顶 0.917（裕量 34mm），管内液柱随管平移。
GRIP_TUBE = 0.0096            # 抓试管：开度≈管身 Ø19.2mm/2（同胶头 Ø11/2=0.0055 逻辑）
TUBE_GRASP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z - 0.014)   # ≈0.9453
SHAKE_CENTER_TCP = (TUBE_XY[0], TUBE_XY[1], 1.09)   # 震荡中心（高位，管底清架顶）
SHAKE_AMPLITUDE = 0.04        # 单向往复半幅 ±40mm（2026-08-16 用户：0.02 幅度不够大→翻倍）
SHAKE_PERIOD = 60             # 一个来回帧数（60Hz ≈1s/来回）。2026-08-16 用户：震荡 3→5 次、要更快
                              #   （周期 100→60，频率 ×1.67）。每帧 TCP 移量 ≈0.04·2π/60≈0.0042，略超
                              #   0.003 保守跟踪线，但小幅振荡+IK 多关节协调可跟踪；总帧数到即 done 不卡死。
                              #   震荡在 z≥0.951（架顶 0.917 上方 34mm）纯水平摆动，±40mm 全程在架上方
                              #   空气里，不碰架体/试管
# 用户 2026-08-14：震荡来回后在高位**停留 5 秒**再放回；现象从加酸滴入即出现、
# 随管震荡持续，震荡停止后**再持续 3 秒**消退（停留 5s 里前 3s 有现象）。
SHAKE_HOLD_FRAMES = 300       # 震荡结束后高位停留帧数（5s @ 60Hz）
SHAKE_TOP_Z = 1.02            # TCP z 高于此值=在震荡/停留区（震荡中心 z=1.09，容差 70mm）
SHAKE_STOP_EPS = 0.0005       # 夹爪单帧位移小于此值=视为"静止"（震荡峰附近单帧 0.0001-0.0003）
SHAKE_STILL_FRAMES = 20       # 静止需连续 ≥20 帧才判"震荡已停"：升到震荡高度的到位冻结+settle
                              #   （≈8 帧静止）在震荡前也会静止，帧数门槛把它们排除——只有
                              #   震荡**结束**后那 300 帧静止才满足（=停留 5s 起算 0.33s 判停）
PHENOMENA_FRAMES = 180        # 现象在震荡停止后还持续的帧数（3s @ 60Hz）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"       # 管内液滴（首滴后显示）
EFFECT_PRECIPITATE = "/World/Precipitate"    # 沉淀（cfg.has_precipitate）
EFFECT_BUBBLES = "/World/Bubbles"            # 气泡（cfg.has_bubbles）
EFFECT_DROPPER_FILL = "/World/DropperFill"   # 滴管尖内截锥液柱（吸液后显示，跟随尖嘴；
                                             #  几何在 gen_d3l_scene.py，task 只需 translate=尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"   # 挤胶头滴落时从尖嘴掉落的液滴（task 动画坠落）
