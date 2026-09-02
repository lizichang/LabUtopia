"""B3L 水浴加热（液体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-30 gen_b3l_scene.py verify 输出（pxr 读 b3l_water_bath.usd 世界包围盒）。

  B3L = 液体实验（用户逐字「写成b3l」「不是挖粉末而是往试管里面滴加溶液（动作参考d3l），
  其他动作直接复制」）：DropperDripPass 取样滴管吸液→滴入试管（复刻 d3l sample_pass）替代
  B3S 的药匙挖粉；LightFlamePass/PickTubePass/ReturnTubePass/LampMovePass/CapLampPass
  直接复制 B3S。结果现象（用户逐字「只变色，要有两个输入最开始的颜色，和加热后变的颜色，
  变色过程是渐变」）= 双颜色输入 before_color/liquid_color + 渐变变色（照 B2 移植）。

  阶段A 样品区 = D3L 布局 + B3S 样品区坐标（用户逐字「动作参考d3l」）：
    滴管 (0.659,0.3209,0.806) 立插架近侧列第3排（原 (0.6993,0.3608) 中心孔距底座
      0.904m 手指朝下 IK 不可达，2026-08-31 挪近侧列 + 改 ORIENT_FWD 水平横夹；
      用户再挪第2排(0.281)→第3排(0.3209)，0.853m 仍在 ORIENT_FWD 可达域内），
      尖嘴贴洞底 0.806、抓点在胶头顶 z = 0.806 + TIP_OFFSET 0.13 = 0.936。
    样品瓶 (0.5365,0.20) 替代表面皿位（sample_bottle.usd 底座贴台面），瓶口 0.870、
      瓶内液面 0.840。
  加热堆叠（铁架台/酒精灯/石棉网/烧杯）= B3S 同款（2026-08-29 用户「随便放个位置后面
    我来调整」）→ 灯/网/烧杯同轴 (0.5286,-0.25)：灯芯顶 0.9005、烧杯底 0.9205 口 1.0109、
    水浴水面 0.9805。火柴 (0.3314,0.1607) 头朝灯芯、灯帽静止位 (0.42,-0.2629)。

持握约定（同 B3S）：火柴/灯帽 = 纯平移持握（不随夹爪旋转，同 B2/d3l）；试管 = 矩阵
持握（B1 同款 _T_HELD_TUBE，B3S 照抄）；滴管 = 矩阵持握（d3s/B2 同款 _T_HELD_DROPPER，
手指朝前 ORIENT_FWD 水平横夹竖直管身，尖嘴 0.13m 吊在夹爪下方）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度，同 d3l/d2s）
SETTLE = 12         # 到点 settle 帧数

# —— 朝向（引擎 [w,x,y,z] 存储，scipy [x,y,z,w] 读法，同 d2s）——
ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 手指 +X（水平横夹试管/移灯/横越）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04

# —— 取样滴管（/World/Dropper，立插架近侧列第3排 (0.659,0.3209)；尖嘴贴洞底 0.806、
#    抓点在胶头顶 z=0.936）——
# 持握 = 矩阵 _T_HELD_DROPPER（d3s/B2 同款水平横夹）：手指朝前 ORIENT_FWD 水平横着夹住
# 竖直管身，旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）+ 平移沿 tool+X 伸
# TIP_OFFSET=0.13；ORIENT_FWD 下 tool+X=世界 -Z → 尖嘴 0.13m 吊在夹爪下方（尖嘴底=原点，
# 与旧手指朝下 HELD_OFFSET 尖嘴位置相同，仅手腕朝向改 ORIENT_FWD 扩可达域 0.75→0.85m）。
# 夹爪开度（用户 2026-08-14 实测）：移动/持握全程 = GRIP_DROPPER = GRIP_ASPIRATE = 0.0055
# （≈ 指间距一半 ≈ 胶头 Ø11mm/2，正好贴胶头面）；只在排空气/滴液瞬间挤到 GRIP_SQUEEZE=0.002。
TIP_OFFSET = 0.13                  # 尖嘴到夹爪下探量（TCP z = 尖嘴 z + 0.13）
DROPPER_XY = (0.659, 0.3209)       # 滴管架近侧列第3排（2026-08-31 自中心孔 (0.6993,0.3608) 挪近
                                   #   →第2排 (0.281)；用户再挪第3排 (0.3209)，0.853m 仍可达）
DROPPER_REST_Z = 0.806             # 尖嘴（原点）贴架孔洞底 z
DROPPER_REST = (DROPPER_XY[0], DROPPER_XY[1], DROPPER_REST_Z)
DROPPER_GRASP = (DROPPER_XY[0], DROPPER_XY[1], DROPPER_REST_Z + TIP_OFFSET)  # (0.659,0.3209,0.936) 胶头顶
GRIP_DROPPER = 0.0055              # 移动/持握开度（胶头 Ø11mm 一半）
GRIP_SQUEEZE = 0.002               # 挤胶头开度（排空气/滴液瞬间）
GRIP_ASPIRATE = 0.0055             # 松胶头吸液开度（=持握宽）

# —— 样品瓶（/World/SolutionBottle (0.5365,0.20) 替代表面皿位；sample_bottle.usd 底座贴台面）——
# 瓶口 z=0.870、瓶内液面 z=0.840（pxr 实测，同 B2 样品瓶）。B3L 挤胶头排空气/浸液吸液
# 都在这瓶：挤气 TCP = 瓶口上 5mm + TIP_OFFSET、浸液 TCP = 瓶口下 DIP_INSET + TIP_OFFSET
# （尖嘴 0.830 沉入液面 0.840 下，能吸上液体）。
# 2026-08-31 修：y 0.105 → 0.20（相对底座 rel y = 0.15，D2S 判据「关键终点 rel y ≥ 0.15 才稳」；
#   旧 0.105 rel y=0.055 正前方奇异位形，下探瓶口 z=0.96 IK FAIL，运行 log 表现为 force-done）。
SOLUTION_BOTTLE_XY = (0.5365, 0.20)
BOTTLE_MOUTH_Z = 0.870             # 瓶口 z
LIQUID_TOP_Z = 0.840               # 瓶内液面 z
DIP_INSET = 0.040                  # 浸液下探量（尖嘴到瓶口下 4cm 入液）
BOTTLE_SQUEEZE_TCP = (SOLUTION_BOTTLE_XY[0], SOLUTION_BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # (0.5365,0.20,1.005)
SOLUTION_DIP_TCP = (SOLUTION_BOTTLE_XY[0], SOLUTION_BOTTLE_XY[1],
                    BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)        # (0.5365,0.20,0.960)

# —— 试管（/World/TestTube，架近侧左孔立插；滴管滴入处）——
TUBE_XY = (0.659, 0.241)             # 试管口中心（pxr 实测；管底 z 0.806、管口 z 0.9593）
TUBE_MOUTH_Z = 0.9593                # 试管口顶 z
TUBE_DROP_RAISE = 0.025              # 管口上提量（尖嘴 0.984 在管口 0.9593 上方 25mm，液滴下落可见）
TUBE_DROP_TCP = (TUBE_XY[0], TUBE_XY[1],
                 TUBE_MOUTH_Z + TUBE_DROP_RAISE + TIP_OFFSET)       # (0.659,0.241,1.114)

# —— 火柴（阶段B 点燃酒精灯；2026-08-29 用户迭代位置）——
# 初版 = 架中心 (0.6803,0.3607) 正后方 -X 10cm → 中心 (0.5803,0.3607)、原点 (0.5314,0.3607)。
# 用户「火柴放在那里还是不太好，在往-x和-y各移动20cm」→ 再 -X 20cm、-Y 20cm：
#   MATCH_XY = (0.5314−0.20, 0.3607−0.20) = (0.3314, 0.1607)。
# 火柴资产原点在 -X 端（全长 0.0978），头 +X 端朝灯芯；本位置离架/皿都远（头 0.4292 距皿
# min x 0.5065 有 7.7cm），横移轨迹（z0.90 直线到 IGNITE）过皿西侧 2.3cm 无碰撞。
# 2026-08-30 用户逐字「夹火柴的时候不要夹中间要夹末尾（-x端）」→ 抓点改到尾端（-X）10mm 处
#   MATCH_GRASP_OFFSET 0.04（杆身中间）→ 0.010；MATCH_TIP_OFFSET 由头中心导出（不手写）。
MATCH_XY = (0.3314, 0.1607)           # 火柴原点世界坐标（-X 端 = 尾部）
MATCH_REST_Z = 0.813
MATCH_GRASP_OFFSET = 0.010            # 夹末尾：距尾部 10mm（杆身中间 0.04 → 末尾 0.010）
MATCH_HEAD_FROM_TAIL = 0.0894         # 火柴头中心到尾部距离（pxr 实测：头中心 x 0.4208 − 尾 0.3314）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)  # (0.3414,0.1607,0.8145)
GRIP_MATCH = 0.0015
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)   # 纯平移持握：尾端(原点) = 夹爪 − 0.010
MATCH_TIP_OFFSET = (MATCH_HEAD_FROM_TAIL - MATCH_GRASP_OFFSET, 0.0, 0.0)  # 头中心 = 夹爪 + 0.0794
MATCH_LIFT_Z = 0.90
WICK = (0.5286, -0.25, 0.9005)       # 灯芯顶（灯随堆叠移到 y=-0.25）
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])  # (0.4492,-0.25,0.9005)
MATCH_HIGH = 1.35

# —— 酒精灯移灯（阶段E 加热结束：用户逐字「熄灭酒精灯应该先把酒精灯往+y方向移动20cm(参考b2)，
#    然后再盖上灯冒」）—— 照 B2 LampMovePass：水平横夹灯体宽处 z=0.845 → 水平 +Y 移 20cm
#    （灯 (0.5286,-0.25) → (0.5286,-0.05)，xz/朝向不变）→ 松爪。移灯期间 task 把帽钉在静止位
#    CAP_REST（帽是灯子 prim，不随灯滑走）。
LAMP_XY = (0.5286, -0.25)              # 灯原点 xy（加热堆叠中心）
LAMP_REST_Z = 0.8002                   # 灯原点 z（底座中心贴台面）
LAMP_BODY_Z = 0.845                    # 灯体宽处世界 z（Ø76.8mm，同 B2 可握）
LAMP_GRASP_OFFSET = LAMP_BODY_Z - LAMP_REST_Z      # 0.0448（_LAMP_HELD 平移）
LAMP_GRASP = (LAMP_XY[0], LAMP_XY[1], LAMP_BODY_Z) # 灯体宽处抓点 (0.5286,-0.25,0.845)
GRIP_LAMP = 0.038                      # 合爪开度 ≈ Ø76mm（同 B2）
LAMP_CLOSED_THRESHOLD = 0.039          # 灯 attach 阈值（灯体宽，同 B2）
LAMP_OPEN_THRESHOLD = 0.0395           # 灯 release 阈值（>GRIP_LAMP 才真松爪，同 B2）
LAMP_MOVE = 0.20                       # 移灯距离 20cm（水平 +Y）
LAMP_TARGET = (LAMP_XY[0], LAMP_XY[1] + LAMP_MOVE, LAMP_BODY_Z)  # 移灯终点夹爪 (0.5286,-0.05,0.845)
LAMP_APPROACH = (LAMP_XY[0] - 0.11, LAMP_XY[1], LAMP_BODY_Z)     # 灯 -X 侧准备 (0.4186,-0.25,0.845)
LAMP_HIGH = (LAMP_APPROACH[0], LAMP_APPROACH[1], 1.25)           # 高位中转 (0.4186,-0.25,1.25)

# —— 灯帽（阶段F 移灯后盖帽灭火；同 B2，先 LampMovePass 移灯 +Y 20cm 再 CapLampPass 盖帽，
#    CAP_BURNER 指移灯后灯口 (0.5286,-0.05)）——
CAP_CENTER_DZ = 0.0915            # 帽中心到灯底座 z 偏移
CAP_REST = (0.42, -0.2629, 0.8155)  # 帽静止位世界中心（盖帽动作在此夹帽；随灯 -Y 平移）
CAP_GRASP = (0.42, -0.2629, 0.824)  # 夹帽点（帽顶 0.8312 下 7mm）
CAP_HIGH = 1.00                   # 高位（取帽/运帽，高于火焰顶 0.9184 清障）
CAP_HELD_OFFSET = (0.0, 0.0, -0.0083)  # 纯平移持握：帽中心 = 夹爪 + offset
CAP_BURNER = (LAMP_TARGET[0], LAMP_TARGET[1], 0.900)  # 盖灯口夹爪（灯 +Y 移 20cm 后灯口 (0.5286,-0.05)；帽中心 0.8917 = 灯z0.8002 + 0.0915）
GRIP_CAP = 0.0185
CAP_CLOSED_THRESHOLD = 0.022
CAP_COVER_NEAR = 0.010            # 盖到位判定：夹爪距 CAP_BURNER < 1cm
CAP_EXTINGUISH_XY = 0.06          # 下落即熄火 xy 门控
CAP_EXTINGUISH_Z = 0.965          # 下落即熄火 z 门控（帽底罩过火焰顶 0.9184 才熄）

# —— 烧杯水浴（加热堆叠中心 (0.5286,-0.25)，B3L 试管浸入水浴 → 气泡环带只避烧杯壁）——
BEAKER_XY = (0.5286, -0.25)       # 烧杯水柱中心（气泡环带/试管浸入参考中心，勿与试管 TUBE_XY 混淆）

# —— 试管水浴转移（阶段A'+E：滴加完成后水平横夹试管提出架孔 → 纯平移分段转烧杯水浴浸入，
#    机械臂保持夹持不松爪；加热结束 ReturnTubePass 提起原路平移回架孔松爪放回）——
# 持握 = B1 同款 _T_HELD_TUBE 矩阵（toolX→(0,0,1)、toolY→(0,1,0)、toolZ→(-1,0,0)，平移
# +TUBE_HELD_X 沿 tool-X），试管被 ORIENT_FWD 水平横夹（手指朝前，同夹药匙）、竖直吊在夹爪下
# （管口 = 夹爪、管底 = 夹爪−0.1533，管口朝上——抓点抬到管口顶）。ORIENT_FWD 组合旋转后试管世界旋转 =
# 恒等（= 架孔竖插静置旋转）→ 抓点吸附零跳变（B1 pxr 数值验证）。
# 用户逐字（2026-08-29）：「拿试管的过程中不是平移过去的，而是中间过程有反转」→ 转移路径改
# 纯平移分段：竖直提出到 TUBE_TRANSIT_Z → 水平横移（z 恒定）→ 竖直浸入，试管全程竖直不翻转。
# 「拿试管加热的时候机械臂不能松手，直到加热结束才放回去」→ 浸入后不松爪，保持夹持加热；
# 加热结束 ReturnTubePass 把试管提回架孔松爪放回（_TubeLifecycle 释放点 = 架孔抓点）。
TUBE_REST_Z = 0.806                    # 架孔试管底 z（管口 0.9593、架顶 0.917）
# 抓点：管口顶 z（2026-08-30 修：原 0.9453=管口下 14mm 在 (0.659,0.241) 低 z 用 ORIENT_FWD
# IK 不可达（D2S 底座注释「关键终点相对底座 y≥0.15 才稳」，试管 y rel=0.191 低 z 死区），
# 运行 log 表现为下探 force-done、手指悬管口上方；抬到管口顶 = 该位可达最高点，配合
# task._near z_thresh=0.03 放宽吸附窗（机械臂到管口上方几毫米即吸附）。
TUBE_GRASP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z)   # (0.659,0.241,0.9593)
GRIP_TUBE = 0.012                     # 试管 Ø19.2mm：开度(2×=24mm)>管径，手指不贴管壁、干净闭合
                                        #（2026-08-30 用户「爪子可以再多紧一点」：0.014→0.012 仍留 4.8mm 裕量不触管壁）
                                        #（2026-08-30 修：旧 0.0096=Ø/2 开度恰等于管径零间隙，
                                        #  手指贴合管壁易卡在 attach 阈值边缘；改同药匙不触杆模式）
TUBE_HELD_X = TUBE_GRASP_TCP[2] - TUBE_REST_Z       # 0.1533：管底吊夹爪下偏移（随抓点 0.9593 自动派生）
TUBE_TRANSIT_Z = 1.174                 # 横移高度（管底 1.0207 清烧杯口 1.0109 / 架顶 0.917；
                                        #  2026-08-30 随 TUBE_HELD_X 0.1393→0.1533 抬 14mm 保管底净空）
TUBE_BOTTOM_IN_BEAKER = 0.9255         # 浸入烧杯管底 z（贴烧杯底 0.9205 上 5mm）
TUBE_TRANSIT = (BEAKER_XY[0], BEAKER_XY[1], TUBE_TRANSIT_Z)  # 横移烧杯正上方（纯水平，z 恒定）
TUBE_IMMERSE_TCP = (BEAKER_XY[0], BEAKER_XY[1], TUBE_BOTTOM_IN_BEAKER + TUBE_HELD_X)  # (0.5286,-0.25,1.0788)
TUBE_RETURN_TRANSIT = (TUBE_XY[0], TUBE_XY[1], TUBE_TRANSIT_Z)  # 放回横移架上方（纯水平，z 恒定）

# —— 变色液柱几何（照 B2 add_color_liquid / d2l-liquid-color-recipe）——
# 液柱 r=LIQUID_R 贴管壁内缘（试管 Ø19.2 外 r0.0096 壁厚~1.1mm 内径~0.0085）、变色柱粗一圈
# COLOR_R 盖在主液柱外层（必须 >LIQUID_R，否则细柱被不透明 op0.95 主液柱包住、变色透不出来）；
# 材质配方（近黑 diffuse + 单通道主导 emissive ~0.8-2.2 + op 0.95）由 gen 脚本烘焙，headless
# 下运行改材质不渲染 → 变色靠**几何**（变色柱顶贴液面向下扩散）。
LIQUID_R = 0.0070                 # 主液柱（TubeLiquidBefore_<色>）半径
COLOR_R = 0.0080                  # 变色柱（TubeLiquidColor_<色>）半径（粗一圈，盖红柱外）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_BEAKER_BUBBLES = "/World/BeakerBubbles"     # 烧杯水浴气泡组（加热时逐个 reveal）
EFFECT_TUBE_DROPS = "/World/TestTubeLiquid"        # 试管内主液柱（滴加逐滴长高，before 柱/清色）
EFFECT_DROPPER_DROP = "/World/DropperDrop"         # 滴管挤胶头滴落串（Drop_0..N 球）
