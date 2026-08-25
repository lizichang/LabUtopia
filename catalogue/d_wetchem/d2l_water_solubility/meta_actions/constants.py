"""D2-L 液体样品水溶性测试（①吸样品滴入）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-24 布局对齐 D2-S（底座实跑修正为 [0.1,0.05,0.71]：D2-S 底座
[-0.15,0.05] 是按手指朝前持药匙设计的，D2-L 持滴管手指朝下臂展 ~0.82m，旧底座 0.93-1.0m
全程 IK FAIL；拉近后最远目标 0.806m；洗瓶/样品瓶/试管架沿 y 共线 x≈0.68），坐标 pxr 读
d2l_water_solubility.usd 世界包围盒（gen_d2l_scene.py verify 输出）：
  TestTubeRack   (0.6803,0.3607)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  TestTube       (0.659,0.241,0.806)  架最近侧孔（D2-S 试管位），管口 z=0.9593
  DropperSample  (0.6993,0.3608,0.806)  架中心孔（D2-S 药匙位，0.904m 已验证可达），
                                         尖嘴底=原点，胶头顶 z=0.9562
  SampleBottle   (0.6809,-0.10)  液体样品瓶（与洗瓶/架共线，两者之间），口 rim z=0.870，
                                         瓶内液面 z=0.840（半瓶）
  WashBottle     (0.6809,-0.2241)  洗瓶（对齐 D2-S 坐标，绕 Z +90° → 瓶嘴 -Y）
                                         —— ②注水步骤（WashBottlePass）用

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，flametest 同款，
见 task.py）：滴管尖嘴 0.13m 吊在夹爪下方。故
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

# —— 取样滴管（/World/DropperSample，架中心孔，D2-S 药匙位）——
DROP_SAMPLE_XY = (0.6993, 0.3608)
DROP_SAMPLE_GRASP = (0.6993, 0.3608, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上）
DROP_SAMPLE_REST = (0.6993, 0.3608, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 样品瓶（/World/SampleBottle (0.6809,-0.10)，口 rim 0.870，液面 0.840）——
SAMPLE_BOTTLE_XY = (0.6809, -0.10)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去瓶塞后，样品瓶）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840 ✓
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP：
BOTTLE_SQUEEZE_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
SAMPLE_DIP_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                  BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 试管（/World/TestTube (0.659,0.241)，管口 z=0.9593）——
TUBE_XY = (0.659, 0.241)
TUBE_MOUTH_Z = 0.9593        # 管口世界 z
# 滴加：尖嘴抬到管口上方 25mm（用户 2026-08-14：滴加时把滴管抬高一点才看清液滴
# 从尖嘴坠落到管内的过程——原来沉入管口内 10mm，尖嘴被管壁挡住、液滴下落看不清）
TUBE_DROP_RAISE = 0.025
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)  # ≈1.114

# —— 洗瓶（/World/WashBottle (0.6809,-0.2241)，绕 Z +90° 瓶嘴 -Y；嘴尖世界 ≈(0.6809,-0.3296,0.844)；
#      v2 WashBottlePass 预留）——
WASH_XY = (0.6809, -0.2241)

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"       # 管内液柱（首滴后显示，液面逐滴涨）
EFFECT_LAYER_COLUMN = "/World/LayerColumn"   # 样品色分层柱（震荡前分层态，miscible 长满）
EFFECT_CLOUD = "/World/Cloud"                # 浑浊云（cloudy 档震荡盖满、停震褪去）
EFFECT_WASH_DROPS = "/World/WashDrops"       # 洗瓶出水串滴（②注水步骤用）
EFFECT_DROPPER_FILL = "/World/DropperFill"   # 滴管尖内样品液柱（吸液后显示，跟随尖嘴；
                                             #  几何在 gen_d2l_scene.py，task 只需 translate=尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"   # 挤胶头滴落时从尖嘴掉落的样品液滴（task 动画坠落）
