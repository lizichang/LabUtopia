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
  WashBottle     (0.370,0.525)  洗瓶（2026-08-25 对齐 D2-S：绕 Z -180° → 红嘴朝 +X）
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

# —— 夹爪朝向（四元数 = scipy [x,y,z,w] 序，直接喂 from_quat；同 flametest/d2s/d3l）——
# ORIENT_FWD：手指 tool+Z 朝 +X（朝向 camera1），洗瓶水平横夹肚子用（红嘴朝 +X，两指沿 ±Y
# 夹瓶身 ±Y 面）。滴管/试管保持默认手指朝下（euler(0,π,0)，controller 引擎默认朝向）。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)

# —— 洗瓶（/World/WashBottle，2026-08-25 对齐 D2-S：translate (0.370,0.525,0.80) rotZ -180°，
#     红嘴朝 +X；瓶身 Mesh_006 = 6.4×6.4×16.8cm 方柱可挤肚子，中心 (0.370,0.525)，
#     z 0.8001-0.9683）——
WASH_XY = (0.370, 0.525)          # 瓶身（肚子）中心 x,y（= translate）
WASH_GRASP_Z = 0.88               # 抓取高度：瓶身中部（z 0.80-0.97 中心 ≈0.884，抬高留俯仰空间）
WASH_APPROACH_X = 0.30            # 下探 x 偏移：避 +X 侧吸管与红嘴尖（嘴尖 x≈0.476，指端 +X
                                  #   伸出 0.0277 → 0.328 < 吸管起点 0.367 净空 3.9cm）
WASH_GRASP = (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z)   # (0.370,0.525,0.88)
GRIP_WASHBOT = 0.030              # 夹肚子开度（半开度）：6cm 开口压 6.4cm 软瓶身每侧 2mm
WASH_LIFT = WASH_GRASP_Z + 0.15   # ⑤ 抬升目标 z：0.88+15cm=1.03（用户「先抬升15cm」）
# ⑥⑦ 红嘴送到试管口上方（2026-08-25 用户「x移动少1cm、y方向再多1cm」微调）：转后红嘴中心
# (0.476,0.525,0.844)，拾起 15cm 后 (0.476,0.525,0.994)；管口中心 (0.659,0.241) 顶 z 0.9593。
WASH_TO_TUBE_X = WASH_XY[0] + 0.173   # ⑥ +X 0.173：红嘴 x 0.476→0.649（锁 y/z，TCP x 0.370→0.543）
WASH_TO_TUBE_Y = WASH_XY[1] - 0.294   # ⑦ -Y 0.294：红嘴 y 0.525→0.231（锁 x/z，TCP y 0.525→0.231）

# —— 挤水段（② 挤肚子出水，效果 prim 由 task 检测夹爪开度驱动）——
WASH_SQUEEZE = 0.020              # 挤水开度：夹爪 0.030→0.020（4cm 开口，压 6.4cm 瓶身 2cm 出水）
WASH_SQUEEZE_CLOSED = 0.025       # task 挤水判定：opening < 0.025 算正在挤（介于持握 0.030 与挤压 0.020）
WASH_SQUEEZE_DWELL = 150          # 挤水保持帧数（水流持续 ~2.5s @60Hz）
# 水流效果（挤水）：父 WaterStream + 16 颗小水滴球沿抛物线从红嘴坠入试管口（几何在
# gen_d2l_scene.py 的 WATER_*；动画在 task.py _step_water_anim，起始=红嘴尖、终点=管口中心）。
WATER_START = (0.649, 0.231, 0.994)   # 红嘴尖（水平射出起点）；终点=管口中心 (0.659,0.241,0.9593)

# —— 试管抓取与震荡（③ 拿起试管震荡使两液混合，参考 d3l TubeShakePass）——
# 试管 Ø19.2×153mm 立插架近侧孔 (0.659,0.241)，管底 z=0.806、管口 z=0.9593、架顶 0.917。
# 抓管身中段（管口下 14mm）：TCP z=0.9453，管底 0.1393m 吊在夹爪下方（纯平移保竖立）。
# 震荡高度 z=1.09：管底 0.9507 清架顶 0.917（裕量 34mm），管内液柱随管平移。
GRIP_TUBE = 0.0096                # 抓试管：开度≈管身 Ø19.2mm/2
TUBE_GRASP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z - 0.014)   # (0.659,0.241,0.9453)
SHAKE_CENTER_TCP = (TUBE_XY[0], TUBE_XY[1], 1.09)   # 震荡中心（高位，管底清架顶）
SHAKE_AMPLITUDE = 0.04            # 单向往复半幅 ±40mm
SHAKE_PERIOD = 60                 # 一个来回帧数（60Hz ≈1s/来回）
SHAKE_HOLD_FRAMES = 300           # 震荡结束后高位停留帧数（5s @60Hz，观察混合现象）
SHAKE_TOP_Z = 1.02                # TCP z 高于此值=在震荡/停留区（震荡中心 1.09，容差 70mm）
SHAKE_STOP_EPS = 0.0005           # 夹爪单帧水平位移小于此值=视为「静止」（震荡/停留判定）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_TUBE_DROPS = "/World/TubeDrops"       # 管内液柱（首滴后显示，液面逐滴涨）
EFFECT_LAYER_COLUMN = "/World/LayerColumn"   # 样品色分层柱（震荡前分层态，miscible 长满）
EFFECT_MIXED_LIQUID = "/World/MixedLiquid"   # miscible 终点均一混液（稀释样品色，随 sample_color 变体）
EFFECT_CLOUD = "/World/Cloud"                # 浑浊云（cloudy 档震荡盖满、停震褪去）
EFFECT_WATER_STREAM = "/World/WaterStream"   # 洗瓶挤水水流（16 滴水滴抛物线，②注水步骤用）
EFFECT_DROPPER_FILL = "/World/DropperFill"   # 滴管尖内样品液柱（吸液后显示，跟随尖嘴；
                                             #  几何在 gen_d2l_scene.py，task 只需 translate=尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"   # 挤胶头滴落时从尖嘴掉落的样品液滴（task 动画坠落）

# 样品色（2026-08-25 用户：样品色改为输入决定，同 d3l liquid_color 方法）。headless 运行时
# 改材质不渲染（记忆 headless-render-ignores-materials），故 gen 为每个候选色预烘焙
# SampleLiquid/LayerColumn/DropperFill/DropperDrop 各 <色> 变体，task 按 cfg.sample_color
# 拼 <色> 后缀 show 对应变体（其余隐藏）。默认 blue（非黄色）。
SAMPLE_COLOR_NAMES = ("clear", "red", "blue", "green", "purple")
EFFECT_SAMPLE_LIQUID = "/World/SampleLiquid"   # 样品瓶内液体（随 sample_color 预烘焙变体）
