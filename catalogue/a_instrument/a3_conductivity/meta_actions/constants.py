# -*- coding: utf-8 -*-
"""A3 电导率测量元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（台面顶 z=0.80）。
几何来源：2026-08-27 pxr 读 gen_a3_scene.py 世界包围盒 + sample_dish.usd 局部 bbox。

布局（2026-08-28 三改=用户 Isaac 重摆，tmp=a3_tmp.usd 为真相，机械臂底座 (0,0.12)；
站间净距 ≥0.02m）：
  分析天平 Balance    (0.3442, 0.5550)  盘顶 z=0.8475
  表面皿 SurfaceDish  (0.3442,0.5550,0.8474)  Ø60×6.5 贴盘顶，世界 bbox 底 0.8475 顶 0.854
     （bbox 中心 z=0.85075；皿 prim 原点=皿底 0.8474，mesh 中心在 origin+0.00335）
  粉堆 PowderOnDish   (0.3442,0.5550,0.857) 程序化圆柱 Ø22×6 贴皿顶、中心 z=0.857
     （可 shrink「倒下」，随皿 6-DOF 跟随）；不用 powder.usd 静态资产粉堆

抓取设计（竖直夹皿，引擎默认朝向手指朝下、开合沿 Y）：
  - **几何真相（2026-08-28 pxr 读 mesh）**：皿是浅碗，皿壁从 Ø6(底) 陡峭外翻到 Ø60(口沿)，
    口沿只高 0.6mm（local 0.0069..0.0075）；不是直壁圆柱。旧抓法只夹到口沿薄边 → 碗身悬在指端下方
  - **一改（皿不跟随）**：get_gripper_position() 返回 tool_center，比指端高 **0.027**（a2 旋光管
    已验证：TCP 0.83 / 指端 0.803）。旧 DISH_GRASP_Z=0.877 已让指端 0.850 落皿壁内
  - **二改（用户实测「爪子下方+没夹紧」→ 下探+闭合）**：DISH_GRASP_Z=0.877→**0.8745**（指端
    0.8475=皿底，指腹 0.8475..0.8745 覆盖整只碗）；GRIP_DISH=0.030→**0.027**（Ø54，指腹压住
    外翻碗壁 ~0.853 处，口沿 clip 3mm 可接受）→ 皿被托在指端之间，不再悬吊
  - **三改（用户「皿相对爪子太靠下，抓住时皿应相对爪子靠上→爪子再往下伸」）**：
    DISH_GRASP_Z=0.8745→**0.8670**。天平盘厚 4.5mm（0.8430..0.8475）、盘下只有中央 Ø20 立柱、
    立柱外盘沿与机身顶(0.84)间是 3mm 空隙 → 指端最深可探到 0.840=机身顶（再低进机身）。
    皿底 0.8474 落在指端上方 **7.4mm**，皿几乎居中于指腹（皿中心 0.85075 距指腹正中 0.8549
    只偏下 2.7mm），不再悬在指端
  - **四改（用户「还是不够靠上」→ 再往下伸，皿偏上）**：DISH_GRASP_Z=0.8670→**0.8620**。
    皿底 0.8474 在指端上方 **12.4mm**、皿中心 0.85075 高出指腹正中 0.8485 **2.25mm**（皿偏上）。
    指端 0.835 进天平机身顶(0.84)下方 5mm——无碰撞、仅接近时短暂穿入（悬空段在天平视野外）
  - **五改（用户「再往下伸 1cm」）**：DISH_GRASP_Z=0.8620→**0.8520**。
    皿底 0.8474 在指端上方 **22.4mm**、皿中心 0.85075 高出指端 **25.75mm**（皿更靠上）。
    指端 0.825 进天平机身顶(0.84)下方 15mm——无碰撞、仅接近时短暂穿入
  - **释放阈值 DISH_GRIP_OPEN=0.038（同 a2 洗瓶）**：GRIP 之上明显裕量，防 attach 后立即 release
  - **不传显式 orient**：低 z 处显式朝向的 FK 检查解不出 IK（A2 旋光管同款教训，
    "IK FAIL … force-done"→ 永不 attach），用引擎默认朝向
  - 皿纯平移持握（无旋转，set_object_position 写 translate op）；粉堆随皿同位移
"""

# ---- 基础 ----
H = 1.15                     # 高位横移高度（被持物最低点 > 沿途最高障碍）
SETTLE = 12                  # 到位稳定帧
GRIP_OPEN = 0.04             # 夹爪满开
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)   # 手指朝 +X（后续横夹动作用）
ORIENT_DOWN = (0.0, 0.0, 1.0, 0.0)              # 手指朝下（竖直，euler(0,π,0)=引擎默认）；
                                                #   放回皿从倾斜转回竖直时显式传（同 d2s ReturnSpatula
                                                #   「不传 orient 会首帧位置到位即冻、朝向没调直」教训）

# ---- 表面皿（浅碗：Ø6 底陡峭外翻到 Ø60 口沿，贴天平盘顶；分析天平 (0.3442,0.5550)）----
DISH_XY = (0.3442, 0.5550)
DISH_ORIG_REST_Z = 0.8474        # 皿 prim 原点（gen translate；= 皿底）
DISH_CENTER_Z = 0.85075          # 皿 bbox 中心 z（几何参考）
DISH_GRASP_Z = 0.8520            # 抓取 TCP 高度：tool_center 比指端高 0.027 → 指端 0.825（进天平机身顶
                                 #   15mm，无碰撞仅接近时短暂穿入）；皿底 0.8474 在指端上方 22.4mm、
                                 #   皿中心 0.85075 高出指端 25.75mm（五改，用户「再往下伸 1cm」）
DISH_GRASP = (DISH_XY[0], DISH_XY[1], DISH_GRASP_Z)   # 抓点（tool_center）
DISH_LIFT = (DISH_XY[0], DISH_XY[1], H)                # 竖直提出到 H
GRIP_DISH = 0.027                # Ø54/2（碗壁外翻，Ø54 处指腹压住 ~0.853；口沿 Ø60 clip 3mm 可接受）
DISH_GRIP_OPEN = 0.038           # 松开阈值（同 a2 洗瓶 WASH_GRIP_OPEN；GRIP 之上明显裕量
                                 #   → 合爪后不会 attach 即 release）
DISH_HELD_OFFSET_Z = DISH_ORIG_REST_Z - DISH_GRASP_Z  # 皿原点 = TCP − 0.0046（皿底在指端上方 22.4mm，
                                 #   attach 时 0.8520−0.0046 = 0.8474 = rest 零跳变）

# ---- 粉堆（程序化圆柱 /World/PowderOnDish，随皿 6-DOF 跟随 + 倒粉时 shrink「倒下」；
#      2026-08-28 用户：不用 powder.usd 静态资产粉堆，要像药匙 PowderOnSpoon 那样程序化、能倒下）----
POWDER_PATH = "/World/PowderOnDish"
POWDER_BLOB_R = 0.011            # 粉堆半径（直径 22mm，同旧 scale 0.25 半大小粉堆宽度）
POWDER_BLOB_H = 0.006            # 粉堆高（扁平，axis=Z）
POWDER_ORIG_REST_Z = 0.857       # 粉堆圆柱中心 z（贴皿顶 0.854 + h/2）
POWDER_HELD_OFFSET_Z = POWDER_ORIG_REST_Z - DISH_ORIG_REST_Z   # 粉堆中心 = 皿原点 + 0.0096

# ---- 样品烧杯（beaker.usd，内建 rotateXYZ(-135,0,0) → 直立、口朝上；2026-08-29 用户改正立烧杯）----
# 几何：pxr points-based 世界 bbox [-0.0377,-0.0435,0]..[0.0377,0.0377,0.0904]（x Ø75 / 高 90、
# 嘴朝 -y 伸 0.0435、底座 z=0）；直立无场景旋转，T(0.4120,0.0807,0.80) 后世界 bbox
# 0.374..0.450 / 0.037..0.118 / 0.80..0.890。口中心（顶 z=0.0904）世界 = (0.4120,0.0807,0.8904)（顶视图中心）。
BEAKER_MOUTH_XY = (0.412, 0.0807)           # 烧杯口顶视图中心（直立烧杯，口朝上正对上方；
                                             #   2026-08-29 用户改 beaker.usd 正立 + 再 +x 2cm，此前侧躺对齐点全部作废）
BEAKER_ABOVE_Z = H                           # 皿横移到烧杯口正上方的高位（后续倒粉前再降）
DISH_ABOVE_BEAKER = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], BEAKER_ABOVE_Z)

# ---- 倒粉（③ 倾斜皿把粉末倒入烧杯；2026-08-28 用户「机械臂只在 x 方向动，然后把玻璃皿
#      倾斜，把粉末都倒进烧杯」）----
# 倾斜 = 手腕绕世界 Y 轴（pitch）转 60°（TILT_ORIENT=euler(0,120°,0)）：tool+Z 从朝下
# (0,0,-1) 转朝 +X 斜下 (0.866,0,-0.5)，皿上法线 dish+z=-tool+z=(-0.866,0,0.5) 朝 -X 斜上
# → 皿 **-X 侧下降、粉末沿 -X 滑出**落入烧杯口（口朝 +y 斜下，x 向宽 76mm 足够；pxr 已验证）。
# 皿绕 tool_center 倾斜时皿原点仅 +x 偏 ~0.004m（DISH_HELD_OFFSET_Z=-0.0046 沿 tool+z 投影）、
# 皿中心仅 ~0.001m，相对烧杯口 x 宽 76mm 可忽略 → 原地倾斜、无需 x 补偿（「机械臂只在 x
# 方向动」= 倾斜全程无 y/z 平移，x 也基本不动）。pxr 已验证。
POUR_TCP_Z = 0.95                            # 皿下降后 TCP 高度（2026-08-29 用户「位置不用那么低，z 要高一点」0.92→0.95）
POUR_TCP = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], POUR_TCP_Z)
TILT_ORIENT = (0.5, 0.0, 0.8660254, 0.0)     # euler(0, 120°, 0)，绕 Y 轴倾斜 60°（w,x,y,z）
POUR_TILT_TCP = POUR_TCP                      # 原地倾斜（= 下降终点，无 x 补偿）
POUR_HOLD = 90                               # 倾斜后保持帧数（粉末下落动画走完 ~1.5s）
# 粉末下落终点 = 烧杯口中心下方（粉粒从皿低侧滑出，竖直坠入直立烧杯口内）
POWDER_LAND = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], 0.82)
BEAKER_POWDER_PATH = "/World/BeakerPowder"   # 烧杯内粉末（初始隐藏，倒粉后显示）
# 粉末下落动画（仿 d2s PowderDrop：父 + N 粉粒球 + 队列错帧起落）
POWDER_DROPS = 14            # 粉粒数
POWDER_STAGGER = 3           # 相邻粉粒起落间隔帧
POWDER_HANG = 4              # 每粒在皿口悬停成形帧
POWDER_FALL = 16             # 每粒坠落帧数（皿底 0.897 → 烧杯口 0.804 约 9cm）

# ---- 洗瓶（wash_bottle.usd，EQUIP (0.3536,0.3062) rotZ180 → 红嘴尖朝 +X；
#      pxr 实测 2026-08-29 世界 bbox）----
#   瓶身 Mesh_006  x 0.3216..0.3856（中心 0.3536）/ y 0.2742..0.3382（中心 0.3062）/ z 0.800..0.9683
#   吸管 Mesh_001  x 0.3502..0.4606 / y 0.3045..0.3079（中心线 y≈0.3062 在瓶身中心线上）
#   红嘴尖 Mesh     x 0.4535..0.4650（开口 +X 端 x=0.4650）/ y 0.3029..0.3095 / z 0.8314..0.8563
WASH_XY = (0.3536, 0.3062)          # 瓶身中心 x,y（= translate，Mesh_006 中心）
WASH_GRASP_Y = WASH_XY[1] - 0.005   # 抓取 y：瓶身中心偏 -y 0.5cm（2026-08-30 用户「夹得有点偏+y，
                                     #   目的坐标再靠-y0.5cm」0.3062→0.3012）
WASH_GRASP_Z = 0.90                 # 抓取高度：瓶身中上部（z 0.80..0.968；2026-08-30 用户「夹太靠下，
                                     #   靠上夹一点点」0.88→0.90，略高于瓶身中心 0.884）
WASH_APPROACH_X = 0.284            # 下探 x 偏移：避开 +X 侧红嘴尖（瓶身 -X 壁 x=0.3216 前 3.8cm；
                                     #   2026-08-30 用户「夹不起来了 → 改回原来」0.20→0.284（指端 0.311 离壁 1cm）；
                                     #   仍保留「先 -x 备好再只 x 伸过去抓」的两段结构，只把偏移量还原）
WASH_GRASP = (WASH_XY[0], WASH_GRASP_Y, WASH_GRASP_Z)   # (0.3536,0.3012,0.90)
GRIP_WASHBOT = 0.030               # 夹肚子开度（半开度）：6cm 开口压 6.4cm 软瓶身每侧 2mm
WASH_LIFT = WASH_GRASP_Z + 0.15    # ⑤ 抬升目标 z：0.90 + 15cm = 1.05（同 d2s「先抬升 15cm」）
WASH_MOVE_DX = -0.10                     # ⑦ -X 10cm（2026-08-29「12cm」→ 2026-08-30 用户「最后往-x移动有点多，减少2cm」→10cm）
WASH_MOVE_DY = -0.20                     # ⑥ -Y 20cm（2026-08-29 用户「-y方向移动20cm」）
WASH_MOVE_X = WASH_XY[0] + WASH_MOVE_DX  # ⑦ TCP 目标 x = 0.3536−0.10 = 0.2536
WASH_MOVE_Y = WASH_GRASP_Y + WASH_MOVE_DY  # ⑥ TCP 目标 y = 0.3012−0.20 = 0.1012（夹点已偏 -y 0.5cm，
                                           #   移动目标同量偏 -y → 挤水位置不变：瓶中心仍 0.1062）
# 移动顺序（2026-08-29 用户「先把最后一步交换，先向-y再向-x」）：⑤ 提起 → ⑥ -Y 20cm → ⑦ -X 10cm
# 挤水后红嘴尖世界位 = 瓶原点 (0.2536,0.1062,0.95) + SPOUT_TIP_OFFSET (0.1114,0,0.044) =
# (0.3650,0.1062,0.994)，在烧杯左壁 0.374 外侧 1cm → 水从红嘴弧线向右下落入烧杯口。

# ---- 挤水（⑥ 挤压洗瓶身，水从红嘴弧线流入烧杯，烧杯内液面上涨；2026-08-30 用户
#      「增加挤液体的动作，注意要精致，是一个水流从红嘴处出来弧线落入烧杯，烧杯里面页面上涨」）----
WASH_SQUEEZE = 0.020            # 挤水开度：夹爪 0.030→0.020（压软瓶身出水，d2s 同款）
WASH_SQUEEZE_CLOSED = 0.025     # task 挤水判定：opening < 0.025 算正在挤（持握 0.030 不误触）
WASH_SQUEEZE_DWELL = 150        # 挤水保持帧数（水流持续 ~2.5s @60Hz，d2s 同款）
# 水流抛物线（d2s 同款：x/y 线性、z t² 重力加速）：起点 = 红嘴尖（task 随瓶动态算），
# 终点 = 烧杯口顶中心（直立烧杯口朝上正对上方，z=口顶 0.8904）
BEAKER_MOUTH_TOP = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], 0.8904)   # 水落点
SPOUT_TIP_OFFSET = (0.1114, 0.0, 0.044)   # 红嘴尖相对瓶原点世界偏移（rest 实测：开口 +X 端
                                          #   x=0.4650−0.3536、z=0.844−0.80；纯平移持握瓶朝向恒定）
WATER_DROPS = 16                # 水滴池大小（round-robin 复用，同 gen_a3_scene WATER_DROPS）
WATER_STAGGER = 2               # 相邻水滴发射间隔帧（错成连续水流）
WATER_FALL = 12                 # 每滴沿抛物线坠落帧数（重力加速视觉）
WATER_LAND_FULL = 64            # 液面涨满需要的落定水滴数（挤水 150 帧≈75 滴，近尾声涨满）
# 烧杯内液面（/World/BeakerLiquid 圆柱，淡蓝半透明；挤水时液面随水流上涨）
BEAKER_LIQUID_PATH = "/World/BeakerLiquid"
BEAKER_LIQUID_R = 0.030         # 液柱半径（烧杯内径 ~Ø60，< 外 Ø76）
BEAKER_LIQUID_H0 = 0.004        # 初始液面高（≈0，几乎不可见）
BEAKER_LIQUID_H_MAX = 0.040     # 挤完最终液面高（液面顶 0.84，烧杯高 0.80..0.890 内）

# ---- 洗瓶抓取前「归位」高位（2026-08-29 修洗瓶夹不起来）----
# 烧杯 +x 2cm 改了表面皿放回末端姿态 → 进洗瓶①时 IK warm-start 翻到近奇异手腕（j5≈-0.3，
# 夹爪方向漂、反复开合夹不住）。归位必须在**侧向**（+Y 大、非正前），因为 ORIENT_FWD 手指
# 朝前在正前方（+X 主导）就是手腕近奇异点（手臂朝前伸 + 手指朝前 = 手腕伸直 j5≈0），
# 在侧向（手臂侧伸 + 手指朝前 = 手腕自然弯 j5≈-2.2）才非奇异。d2s 洗瓶在 +Y 0.475 侧向能
# 夹、A3 在 +Y 0.186 正前夹不住，正是此因。
# 归位点 = 表面皿放回终点 DISH_LIFT (0.3442,0.5550,H)（天平位，+Y 0.435 已证可达）：在那边
# **原地**转 ORIENT_FWD（位置不动只转手腕）→ 手腕落到非奇异 j5≈-2.2；再进①正前接近时
# warm-start 锁死这条好分支（不再被表面皿放回末端姿态翻到奇异分支）。
WASH_HOME = DISH_LIFT

# ---- 玻璃棒（glass_rod_6x6x261.usd，Ø6×261mm 竖直插试管架前排；2026-08-30 用户
#      「到试管架上水平横夹取出玻璃棒并移动到烧杯上方」）----
# 几何：pxr 实测世界 bbox x 0.5404..0.5464 / y 0.1964..0.2026 / z 0.80..1.061（Ø6、高 0.261、
# prim 原点=棒底 0.80=台面顶）；试管架 (0.5630,0.1989) bbox z 0.80..0.917（架顶 0.917）。
# 抓点 = 棒底上方 0.20m（同 E1 PickRod）：z=1.00 在架顶 0.917 上 8.3cm 可握段，手指 ORIENT_FWD
# 朝 +X、两指沿 ±Y 横夹 Ø6 棒身（水平横夹，同洗瓶/药匙）；棒顶 1.061 伸出指端之上无碰撞
# （E1 同款已验证）。纯平移持握：棒底 = TCP − 0.20（棒底悬 0.95 时 TCP=1.15）。
ROD_XY = (0.5434, 0.1995)            # 棒中心 x,y（= translate）
ROD_BOTTOM_Z = 0.80                  # 棒底 z（= 台面顶，棒立架内底贴台面）
ROD_TIP_TO_GRIP = 0.20               # 抓点 = 棒底上方 0.20m（同 E1）
ROD_GRASP_Z = ROD_BOTTOM_Z + ROD_TIP_TO_GRIP    # 1.00（架顶 0.917 上 8.3cm 可握段）
ROD_GRASP = (ROD_XY[0], ROD_XY[1], ROD_GRASP_Z)   # 抓点（tool_center）
ROD_REST_POS = (ROD_XY[0], ROD_XY[1], ROD_BOTTOM_Z)   # 棒静止位（= translate；reset/释放回位用）
GRIP_ROD = 0.003                     # 玻璃棒 Ø6mm → 半宽 3mm（同 E1：开度 = 2×值）

# ⑨ 搅拌（StirInBeaker）：⑧ 之后棒悬烧杯口上方（棒底 0.95）。下探把棒底沉到
# STIR_TIP_Z（液面顶 0.84 下 2cm、杯内底 0.80 上 2cm），绕烧杯口中心画半径
# STIR_RADIUS 的圆（杯内径 Ø60 → 内 r 30mm，棒径 6 + 圆半径 15 远在杯内）。
STIR_TIP_Z = 0.82                  # 搅拌时棒底 z（沉入混合液）
STIR_TCP_Z = STIR_TIP_Z + ROD_TIP_TO_GRIP    # 1.02（下探目标 TCP z；口顶 0.8904 下探段棒底穿过无碰壁）
STIR_CENTER = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], STIR_TCP_Z)   # 搅拌圆心（= 烧杯口中心，z 锁）
STIR_RADIUS = 0.015                # 搅拌圆半径（棒底画圆）
STIR_CYCLES = 15                   # 搅拌圈数（2026-08-30 用户「搅拌次数太少，至少 15 次」3→15）
STIR_PERIOD = 45                   # 每圈帧数（60Hz → 0.75s/圈，共 ~11.25s）

# ---- 导电率仪电极（/World/Meter/electrode，随 conductivity_meter.usd 内置；2026-08-30 用户
#      「竖直提起导电率仪的棒子」）----
# 几何：pxr 实测世界 bbox 0.3849..0.4049 / -0.029..-0.007 / 0.80..0.966（Ø20×20 截面、高 0.166m
# 竖直探头；blades 底贴台面 0.80、cap 顶 0.966）。cap（顶部手柄）world 中心 (0.3949,-0.018,0.9455)、
# 截面 20×20mm。电极 prim 无 xform op（mesh 烘焙在 meter 局部系），meter 仅 rotZ90 → 局部 +z = 世界 +z
# → 竖直提起 = 对电极 prim 写 (0,0,lift_z) 纯 z 平移。
# 抓点 = cap 中心 z=0.945（ORIENT_FWD 手指朝 +X、两指沿 ±Y 横夹 Ø20 cap）；指端 = TCP+0.027 沿 tool+X。
# 接近：-X 侧偏移 10cm（cap -X 壁 0.3849 前；电缆从 cap 顶向 +X/-Y 后右角走，-X 侧干净无穿模）。
ELECTRODE_XY = (0.3949, -0.018)            # cap 中心 x,y（pxr 实测）
ELECTRODE_CAP_Z = 0.9625                   # 抓点 TCP 高：竖直夹 cap（手指朝下，指端 = TCP − 0.027）→
                                           #   指端 0.9355 = cap 下部（cap z 0.925..0.966）；2026-08-30 用户
                                           #   「竖直提不是水平横夹，像 d3l 夹滴管」0.945→0.9725（指端=cap 中心）
                                           #   再「太高了再下伸 1cm」0.9725→0.9625（指端 0.9355 = cap 下段）
ELECTRODE_GRASP = (ELECTRODE_XY[0], ELECTRODE_XY[1], ELECTRODE_CAP_Z)
ELECTRODE_APPROACH_X = 0.285               # -X 侧接近偏移（cap -X 壁 0.3849 前 10cm；电缆从 cap 顶向 +X
                                           #   前下垂 → -X 侧干净，竖直下探后 +X 移入 cap 上方）
GRIP_ELECTRODE = 0.010                     # Ø20 cap → 半宽 10mm（手指朝下两指沿 ±Y 竖直夹 cap 的 ±Y 面）
ELECTRODE_PATH = "/World/Meter/electrode"  # 电极 prim（无 xform op，mesh 烘焙；持握 3-DOF 平移跟随）
ELECTRODE_CAP_TOP = (0.3949, -0.018, 0.965)  # cap 顶世界位（rest；= CAP_TOP 资产局部 (0.195,0.040,0.165)
                                           #   rotZ90+平移后）。电缆移动端锚点，task 逐帧 cable.update
CABLE_PATH = "/World/Meter/CableRoot"      # 动态电缆根（fix_conductivity_cable.py 建；task 逐帧 update）

# ---- ⑫ 电极移到烧杯上方 + 下降深入（2026-08-30 用户「加动作把仪器移到烧杯上方然后下降深入」）----
# 电极持握后（cap 中心随 TCP，blades 底 = TCP_z − (0.9625−0.80)=TCP_z−0.1625）：
#   blades 底 z = 0.80 + (TCP_z − ELECTRODE_CAP_Z)。要 blades 浸入液面 0.84 下约 2cm → blades 0.82。
ELECTRODE_LIFT_Z = 1.10                    # ⑪ 提起后 TCP 高（blades 0.9375 清烧杯口 0.8904 约 4.7cm；
                                           #   2026-08-30 用户「不用抬那么高」H=1.15→1.10，够越过烧杯口即可）
ELECTRODE_ABOVE = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], ELECTRODE_LIFT_Z)   # 烧杯口正上方（TCP）
ELECTRODE_DIP_Z = 0.98                     # ⑫ 下降深入后 TCP 高：blades 底 = 0.80 + (0.98−0.9625)
                                           #   = 0.8175，浸入液面 0.84 下 2.25cm、离杯底 0.80 约 1.75cm
ELECTRODE_DIP = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], ELECTRODE_DIP_Z)

# ---- ⑬ 松爪放电极进烧杯（2026-08-30 用户「放进去就可以松手」）----
# ⑫ 之后电极仍 attached、blades 已浸入烧杯液面（TCP 0.98）。本动作开爪松放 → task 检测
# opening > ELECTRODE_GRIP_OPEN(0.038) → released（电极 + 电缆冻结在烧杯内，不回 meter），
# 再抬空爪到 ELECTRODE_LIFT_Z 清电极 cap 顶（cap 顶 0.9825）撤离。
ELECTRODE_RELEASE_LIFT = (BEAKER_MOUTH_XY[0], BEAKER_MOUTH_XY[1], ELECTRODE_LIFT_Z)

# ---- ⑭ 按下导电率仪机顶「开始」键（2026-08-30 用户「把开始按钮放到顶部」；机顶红色矮圆柱
#      /World/Meter/start_button，fix_conductivity_button.py 建）----
# front 面板「确认」键贴竖直面板、太前+低 z，水平横向按 ORIENT_FWD 手腕近奇异（j5≈0）IK 卡住
# 一分钟、手指朝下又按不到竖直面 → 把键放机顶 deck 顶，垂直按下（手指朝下），折光仪机顶
# start_button 同款（Ø32mm×6mm、红色、UsdPreviewSurface diffuse(0.85,0.12,0.1)）。
# 按钮世界：中心 (0.3549,-0.133,0.908)、底 0.905（贴 deck 顶）、顶 0.911（pxr 实测 2026-08-30）。
# 按下 = 爪子先合爪夹按钮两侧（GRIP_BUTTON=按钮半径 0.016），再垂直下探到按钮顶 0.911（手指
# 夹着按钮下压）。按钮无碰撞，是「按」到顶即触发（读数后续步骤追加）。
START_BUTTON_XY = (0.3549, -0.133)        # 机顶按钮中心 x,y（rotZ90+平移后，pxr 实测）
START_BUTTON_TOP_Z = 0.911                # 按钮顶世界 z（底 0.905 + 高 6mm）
GRIP_BUTTON = 0.001                       # 按下时完全闭合（2026-08-30 用户「按下爪子应完全闭合，
                                          #   现在半张开」0.016→0.001）：闭合指端并拢压按钮顶，非
                                          #   半张开夹 Ø32 两侧
BTN_CONTACT_TCP_Z = START_BUTTON_TOP_Z + 0.027   # 0.938：闭合指端触按钮顶（指端=TCP−0.027，
                                          #   手指朝下时指尖在 tool_center 下方 27mm）
BTN_APPROACH = (START_BUTTON_XY[0], START_BUTTON_XY[1], H)                # 高位接近按钮上方
BTN_PREPRESS = (START_BUTTON_XY[0], START_BUTTON_XY[1], BTN_CONTACT_TCP_Z + 0.02)  # 预按位（触顶上方 2cm）
BTN_PRESS = (START_BUTTON_XY[0], START_BUTTON_XY[1], BTN_CONTACT_TCP_Z)   # 闭合指端压按钮顶（0.938）
BTN_DWELL = 30                           # 按下保持帧（~0.5s 让「按下」可见）

# ---- 按钮下沉动画（task._ButtonLifecycle 驱动，折光仪 A1 同款）：按下触发 → 按钮局部 z 下沉
#      5mm（0.108→0.103，顶 0.911→0.906 几乎压平到 deck 顶 0.905）→ 爪子抬离 > BUTTON_LIFT_Z
#      → 缓慢弹回。按钮无碰撞，是「按」到触发点即下沉（非物理下压），按钮局部 z 写在
#      /World/Meter/start_button 的 translate op 上。 ----
BUTTON_PATH = "/World/Meter/start_button"
BUTTON_REST_Z = 0.108           # 按钮静止局部 z（Meter 内；顶 0.911）
BUTTON_PRESSED_Z = 0.103        # 按下局部 z（下沉 5mm，顶 0.906 ≈ deck 顶 0.905 几乎压平）
BUTTON_LIFT_Z = 0.950           # 爪子抬离判定 z（按下停在 TCP 0.938，抬到 0.95 触发弹回）
BUTTON_SPRING_STEP = 0.0002     # 每帧上抬 0.2mm（0.103→0.108 共 25 帧 ≈ 0.4s 缓慢弹回）

# ---- 导电率读数输入（experiment_result schema 写回 cfg.conductivity，config 同源，勿单边改）----
# 屏幕读数由输入决定（A1 nD 同款——headless 运行时改材质不渲染 → gen 预烘焙每档一张贴图 +
# 一个 ScreenGlow_<key> prim，task 按 cfg.conductivity 选一个 show）。key=去小数点（1.413→1_413）。
# gen_a3_scene.py CONDUCTIVITY_OPTIONS 须与此一致。
CONDUCTIVITY_OPTIONS = ["0.012", "0.250", "1.413", "12.88"]   # mS/cm（蒸馏水/自来水/0.01M KCl/0.1M KCl）
CONDUCTIVITY_DEFAULT = "1.413"

# ---- 屏幕读数效果 prim 路径（scene 内建，task 动画驱动）----
EFFECT_SCREEN_MEASURING_TPL = "/World/ScreenMeasuring_{step:02d}"   # 测量中进度条帧（i=0..PROGRESS_STEPS-1）
PROGRESS_STEPS = 16        # 进度条帧数（每帧 ~0.25s @ MEASURE_FRAMES 240 帧 / 4s；须与 gen 一致）
EFFECT_SCREEN_RESULT_TPL = "/World/ScreenGlow_{key}"    # 完成读数屏幕（导电率读数，key=去小数点档位）
