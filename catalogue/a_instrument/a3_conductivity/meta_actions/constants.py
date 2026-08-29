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
# 嘴朝 -y 伸 0.0435、底座 z=0）；直立无场景旋转，T(0.3920,0.0807,0.80) 后世界 bbox
# 0.354..0.430 / 0.037..0.118 / 0.80..0.890。口中心（顶 z=0.0904）世界 = (0.3920,0.0807,0.8904)（顶视图中心）。
BEAKER_MOUTH_XY = (0.392, 0.0807)           # 烧杯口顶视图中心（直立烧杯，口朝上正对上方；
                                             #   2026-08-29 用户改 beaker.usd 正立，此前侧躺对齐点全部作废）
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
WASH_GRASP_Z = 0.88                 # 抓取高度：瓶身中部（z 0.80..0.968 中心 ≈0.884，同 d2s）
WASH_APPROACH_X = 0.284            # 下探 x 偏移：避开 +X 侧红嘴尖（瓶身 -X 壁 x=0.3216 前 3.8cm）
WASH_GRASP = (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z)   # (0.3536,0.3062,0.88)
GRIP_WASHBOT = 0.030               # 夹肚子开度（半开度）：6cm 开口压 6.4cm 软瓶身每侧 2mm
WASH_LIFT = WASH_GRASP_Z + 0.15    # ⑤ 抬升目标 z：0.88 + 15cm = 1.03（同 d2s「先抬升 15cm」）
WASH_MOVE_DX = -0.12                     # ⑥ -X 12cm（2026-08-29 用户「不朝-x移动15cm，改为12cm」，不再对齐烧杯口）
WASH_MOVE_DY = -0.20                     # ⑦ -Y 20cm（2026-08-29 用户「-y方向移动20cm」）
WASH_MOVE_X = WASH_XY[0] + WASH_MOVE_DX  # ⑥ TCP 目标 x = 0.3536−0.12 = 0.2336
WASH_MOVE_Y = WASH_XY[1] + WASH_MOVE_DY  # ⑦ TCP 目标 y = 0.3062−0.20 = 0.1062
# 挤水/水流动画留待后续动作（用户 2026-08-29「你先只写这个移动过程」）。
