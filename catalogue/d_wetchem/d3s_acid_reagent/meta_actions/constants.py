"""D3-S 固体样品 + 酸性试剂滴加反应 元动作共享常量。

D3-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加酸性试剂」。挖粉动作完全不变
（药匙/表面皿/粉末/试管/试管架坐标与 d2s 逐字一致，见 d2s 元动作包）；酸滴管
用 B2 同款水平横夹（ORIENT_FWD 手指朝前，滴管竖直挂夹爪下，同 d2s 夹药匙）。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何来源：2026-08-25 pxr 读 d3s_acid_reagent.usd（gen_d3s_scene.py verify 输出）：
  TestTubeRack  (0.6803,0.3607)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  TestTube      (0.659,0.241,0.806)  架近侧左孔，管口 z=0.9593（同 d2s）
  Spatula       (0.6993,0.3608)  挖粉动作不动（d2s 元动作包内坐标）
  SurfaceDish   (0.5365,0.105) / SamplePowder (0.5383,0.0992)
  DropperAcid   (0.6993,0.3209,0.806)  酸滴管立插主试管架第二列第3排（用户 2026-08-26：①原右近孔
                                       (0.6993,0.241) 在试管旁挡震荡/倒粉→移右后远端 (0.6993,0.4804)；
                                       ②远端排 IK 够不着→"第二列不变放第3排"=本位置）
  HClBottle     (0.370,0.30)  盐酸瓶（口 rim 0.870，液面 0.840，去瓶塞后）

滴管持握约定（水平横夹，B2 同款，见 task.py）：滴管 = _T_HELD · tool_world，_T_HELD 沿
tool+X 伸 0.13。ORIENT_FWD 时 tool+X=世界 -Z → 滴管竖直挂夹爪下、尖嘴朝下：
  TCP z = 尖嘴 z + 0.13
抓点 = 立放位 + (0,0,0.13)：滴管尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段，夹胶头）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度；无 B2 铁架台挂钩，单段 H 即可）
SETTLE = 12         # 到点 settle 帧数

# —— 朝向（引擎 [w,x,y,z] 存储，scipy [x,y,z,w] 读法，同 d2s/B2）——
# 酸滴管全程水平横夹：手指朝前 ORIENT_FWD（d2s 夹药匙同款），滴管竖直挂夹爪下、
# 尖嘴朝下。无需手指朝下竖直夹。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)   # 手指 +X（d2s 已验证）

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm（pxr 实测）。Franka joint 开度 ≈ 指间距一半 → "正好贴胶头面"
# 开度 = 0.011/2 = 0.0055（d3l/B2 已验证）。
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸酸后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 沿手指方向的偏移）——
TIP_OFFSET = 0.13           # 尖嘴 = 夹爪 + 0.13·tool+X（ORIENT_FWD → 世界 -Z，尖嘴吊夹爪下）

# —— 酸滴管（/World/DropperAcid (0.6993,0.3209)，主试管架第二列第3排；尖嘴底 0.806）——
# 2026-08-26 用户两次改位：①"放试管架另一端离试管最远"→ 右后远端孔 (0.6993,0.4804)，实测
# IK 够不着（该排整体解不出，底座固定 d2s）；②"第二列不变放第3排"→ 回第二列、第3排
# (0.6993,0.3209)。比药匙第4排 (0.6993,0.3608) 还近 → 必可达；y=0.3209≠试管 y=0.241 不在
# 震荡 X 扫掠平面，不冲突（试管 y 固定 0.241，滴管 8cm 外）。
DROP_ACID_XY = (0.6993, 0.3209)
DROP_ACID_GRASP = (0.6993, 0.3209, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上）
DROP_ACID_REST = (0.6993, 0.3209, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 盐酸试剂瓶（/World/HClBottle (0.370,0.30)，口 rim 0.870，液面 0.840）——
ACID_BOTTLE_XY = (0.370, 0.30)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去瓶塞后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（HClBottle 液面顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840 ✓
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸酸（尖嘴 0.830 入液面下 10mm）的 TCP：
ACID_SQUEEZE_TCP = (ACID_BOTTLE_XY[0], ACID_BOTTLE_XY[1],
                    BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
ACID_DIP_TCP = (ACID_BOTTLE_XY[0], ACID_BOTTLE_XY[1],
                BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 试管（/World/TestTube (0.659,0.241)，管口 z=0.9593，同 d2s 试管位）——
TUBE_XY = (0.659, 0.241)
TUBE_MOUTH_Z = 0.9593        # 管口世界 z
# 试管震荡中心（gripper z）：2026-08-26 用户"震荡抬高一点再震荡，现在跟试管架穿模"。
# 原 d2s SHAKE_CENTER_TCP z=1.09（管底 0.951 只清架顶 0.917 34mm，X 扫掠 ±40mm 会扫近试管旁
# 的滴管）→ 抬高到 1.18（管底 1.041，清架顶 12cm，扫掠不碰架/滴管）。d3s 自用，d2s 不动。
SHAKE_CENTER_TCP = (TUBE_XY[0], TUBE_XY[1], 1.18)
# 滴加：尖嘴抬到管口上方 25mm（同 d3l，看清液滴从尖嘴坠落入管，不沉入管内被壁挡住）
TUBE_DROP_RAISE = 0.025
# 酸滴管滴加 TCP：尖嘴 = 管口上方 25mm，TCP = 尖嘴 + 0.13
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)  # ≈1.114

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"       # 管内酸液柱（首滴后显示，逐滴生长）
EFFECT_PRECIPITATE = "/World/Precipitate"    # 沉淀（cfg.has_precipitate）
EFFECT_PRECIPITATE_CLOUD = "/World/PrecipitateCloud"  # 浑浊云（加酸瞬间乳白浑浊）
EFFECT_DROPPER_DROP = "/World/DropperDrop"   # 挤胶头滴落时从尖嘴掉落的液滴（task 动画坠落）
EFFECT_TUBE_SAMPLE = "/World/TubeSample"     # 管内白色固体样品粉末（⑬ 倒粉后显示）
