"""D8-L 络合/显色试剂滴加反应（液体样品，3 试剂）元动作共享常量。

复刻 d3l 模板（用户 2026-08-28："3 个试剂瓶 + 3 个胶头滴管 + 多输入，直接复刻 d3l 模板"）。
与 D3-L 同构，仅扩为 **3 支滴管 + 3 个试剂瓶**：取样滴管吸样品液 → 试剂 1 滴管吸试剂 1 →
试剂 2 滴管吸试剂 2，三支各自「吸液→滴入同一试管」，液体逐滴变色（3 段），最后一支
（试剂 2）滴入触发沉淀，震荡停后分层成形。无气泡。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：gen_d8l_scene.py verify 输出（3 滴管占架 2×2 孔、3 瓶一排）：
  TestTubeRack    (0.30,0.00)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  TestTube        (0.2787,0.1193,0.806)  前排左孔，管口 z=0.9593
  DropperSample   (0.2815,-0.1187,0.806) 后排左孔（离试管最远）
  DropperReagent1 (0.3202,-0.1187,0.806) 后排右孔
  DropperReagent2 (0.3202, 0.1193,0.806) 前排右孔（试管同排对侧）
  SampleBottle    (0.4045,0.3585) 口 rim z=0.870 液面 0.840（sample_bottle.usd）
  Reagent1Bottle  (0.1696,0.361)  口 rim z=0.870 液面 0.840（hcl_bottle.usd）
  Reagent2Bottle  (0.287, 0.361)  口 rim z=0.870 液面 0.840（hcl_bottle.usd）

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，flametest 同款，
见 task.py）：滴管尖嘴 0.13m 吊在夹爪下方。故 TCP z = 尖嘴 z + 0.13；抓点 = 立放位 +
(0,0,0.13)：尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm → 开度 = 0.011/2 = 0.0055（正好贴胶头面，d3l 用户实测减半正合适）
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# 三支滴管立放底面同 z=0.806（插孔底面），抓点 z 同 = 0.806 + TIP_OFFSET = 0.936
DROP_REST_Z = 0.806
DROP_GRASP_Z = 0.806 + TIP_OFFSET   # = 0.936（架顶 0.917 之上）

# —— 取样滴管（/World/DropperSample，后排左孔）——
DROP_SAMPLE_XY = (0.2815, -0.1187)
DROP_SAMPLE_GRASP = (0.2815, -0.1187, DROP_GRASP_Z)
DROP_SAMPLE_REST = (0.2815, -0.1187, DROP_REST_Z)

# —— 试剂 1 滴管（/World/DropperReagent1，后排右孔）——
DROP_REAGENT1_XY = (0.3202, -0.1187)
DROP_REAGENT1_GRASP = (0.3202, -0.1187, DROP_GRASP_Z)
DROP_REAGENT1_REST = (0.3202, -0.1187, DROP_REST_Z)

# —— 试剂 2 滴管（/World/DropperReagent2，前排右孔）——
DROP_REAGENT2_XY = (0.3202, 0.1193)
DROP_REAGENT2_GRASP = (0.3202, 0.1193, DROP_GRASP_Z)
DROP_REAGENT2_REST = (0.3202, 0.1193, DROP_REST_Z)

# —— 三瓶（口 rim 0.870、液面 0.840 全同；瓶型 sample_bottle / hcl_bottle）——
SAMPLE_BOTTLE_XY = (0.4045, 0.3585)
REAGENT1_BOTTLE_XY = (0.1696, 0.361)
REAGENT2_BOTTLE_XY = (0.287, 0.361)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去瓶塞后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid/Reagent*Liquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP z：
BOTTLE_SQUEEZE_TCP_Z = BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET   # = 1.005
DIP_TCP_Z = BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET          # = 0.960

# —— 试管（/World/TestTube (0.2787,0.1193)，管口 z=0.9593）——
TUBE_XY = (0.2787, 0.1193)
TUBE_MOUTH_Z = 0.9593        # 管口世界 z
TUBE_DROP_RAISE = 0.025      # 滴加时尖嘴抬到管口上方 25mm（看清液滴坠落，用户 2026-08-14）
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)  # ≈1.114

# —— 试管抓取与震荡（步9 拿起试管震荡来回）——
GRIP_TUBE = 0.0096            # 抓试管：开度≈管身 Ø19.2mm/2
TUBE_GRASP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z - 0.014)   # ≈0.9453
# 震荡中心（高位，管底清架顶）。2026-08-29 用户：震荡起始抬高——管底 = TCP−0.139 = 0.981，
# 清同排试剂2滴管顶 0.9562（旧 1.09 → 管底 0.951 与其重叠 5mm 穿模）。
SHAKE_CENTER_TCP = (TUBE_XY[0], TUBE_XY[1], 1.12)
SHAKE_AMPLITUDE = 0.04        # 单向往复半幅 ±40mm
SHAKE_PERIOD = 60             # 一个来回帧数（60Hz ≈1s/来回）
SHAKE_HOLD_FRAMES = 480       # 震荡结束后高位停留帧数（8s @ 60Hz；2026-08-29 用户：举久一点）
SHAKE_TOP_Z = 1.02            # TCP z 高于此值=在震荡/停留区
SHAKE_STOP_EPS = 0.0005       # 夹爪单帧位移小于此值=视为"静止"
SHAKE_STILL_FRAMES = 20       # 静止需连续 ≥20 帧才判"震荡已停"
PHENOMENA_FRAMES = 180        # 分层/沉淀沉降在震荡停止后的成形帧数（3s @ 60Hz）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"            # 管内液滴（首滴后显示）
EFFECT_PRECIPITATE = "/World/Precipitate"         # 沉淀（cfg.has_precipitate）
EFFECT_PRECIPITATE_CLOUD = "/World/PrecipitateCloud"  # 浑浊云（震荡盖满液柱=整管变白）
EFFECT_LAYER = "/World/LayerBottom"               # 分层底部有色液相（cfg.has_layer）
EFFECT_DROPPER_FILL = "/World/DropperFill"        # 滴管尖内截锥液柱（吸液后显示，跟随尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"        # 挤胶头滴落时从尖嘴掉落的液滴（task 动画坠落）
