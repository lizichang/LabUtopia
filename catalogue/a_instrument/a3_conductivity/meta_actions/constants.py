# -*- coding: utf-8 -*-
"""A3 电导率测量元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（台面顶 z=0.80）。
几何来源：2026-08-27 pxr 读 gen_a3_scene.py 世界包围盒 + sample_dish.usd 局部 bbox。

布局（2026-08-27 二改=用户 Isaac 重摆，tmp=a3_conductivity_tmp.usd 为真相，围绕机械臂
底座 (0.37,0.16) 环形摆放；站间净距 ≥0.02m）：
  分析天平 Balance    (0.3442, 0.5550)  盘顶 z=0.8475
  表面皿 SurfaceDish  (0.3442,0.5550,0.8474)  Ø60×6.5 贴盘顶，世界 bbox 底 0.8475 顶 0.854
     （bbox 中心 z=0.85075；皿 prim 原点=皿底 0.8474，mesh 中心在 origin+0.00335）
  粉堆 SamplePowder   (0.3442,0.5550,0.84985) scale 0.5 落皿内，bbox y 0.5327..0.592
     （+Y 伸出皿沿 7mm）、底贴皿顶 0.854、顶 0.869；独立 /World prim 不随皿走 → task 需跟随

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

# ---- 表面皿（浅碗：Ø6 底陡峭外翻到 Ø60 口沿，贴天平盘顶；分析天平 (0.3442,0.5550)）----
DISH_XY = (0.3442, 0.5550)
DISH_ORIG_REST_Z = 0.8474        # 皿 prim 原点（gen translate；= 皿底）
DISH_CENTER_Z = 0.85075          # 皿 bbox 中心 z（几何参考）
DISH_GRASP_Z = 0.8620            # 抓取 TCP 高度：tool_center 比指端高 0.027 → 指端 0.835（进天平机身顶
                                 #   5mm，无碰撞仅接近时短暂穿入）；皿底 0.8474 在指端上方 12.4mm、
                                 #   皿中心 0.85075 高出指腹正中 2.25mm（四改，皿偏上不再偏低）
DISH_GRASP = (DISH_XY[0], DISH_XY[1], DISH_GRASP_Z)   # 抓点（tool_center）
DISH_LIFT = (DISH_XY[0], DISH_XY[1], H)                # 竖直提出到 H
GRIP_DISH = 0.027                # Ø54/2（碗壁外翻，Ø54 处指腹压住 ~0.853；口沿 Ø60 clip 3mm 可接受）
DISH_GRIP_OPEN = 0.038           # 松开阈值（同 a2 洗瓶 WASH_GRIP_OPEN；GRIP 之上明显裕量
                                 #   → 合爪后不会 attach 即 release）
DISH_HELD_OFFSET_Z = DISH_ORIG_REST_Z - DISH_GRASP_Z  # 皿原点 = TCP − 0.0146（皿底在指端上方 12.4mm，
                                 #   attach 时 0.8620−0.0146 = 0.8474 = rest 零跳变）

# ---- 粉堆（独立 /World prim，随皿同位移跟随；2026-08-28 用户：粉末缩小到一半 scale 0.25）----
POWDER_PATH = "/World/SamplePowder"
POWDER_ORIG_REST_Z = 0.851925    # gen translate（scale 0.25 的粉堆底贴皿顶 0.854；原 scale 0.5 时 0.84985）
POWDER_HELD_OFFSET_Z = POWDER_ORIG_REST_Z - DISH_ORIG_REST_Z   # 粉原点 = 皿原点 + 0.004525
