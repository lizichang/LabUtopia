"""B3 水浴加热（固体样品熔化）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-29 gen_b3_scene.py verify 输出（pxr 读 b3s_water_bath.usd 世界包围盒）：

  阶段A 样品区 = D2-S 布局复刻（用户「把 d2s 所有物品包含机械臂的位置复刻过来，这样挖粉末
    才不会出错」）→ 挖粉动作坐标与 d2s 完全一致：
    试管架 (0.6803,0.3607)、试管 (0.659,0.241,0.806) 架近侧左孔（管口 0.9593）、
    药匙 (0.6993,0.3608,0.828) 架中心孔竖插、皿 (0.5365,0.105,0.80)、粉丘 (0.5383,0.0992) scale0.4。
  加热堆叠（铁架台/酒精灯/石棉网/烧杯）= B2 相对几何整组 -Y 平移 25cm（用户「还是这样相对
    位置，随便放个位置后面我来调整」）→ 灯/网/烧杯同轴 (0.5286,-0.25)：酒精灯灯芯顶 0.9005、
    烧杯底 0.9205 口 1.0109、水浴水面 0.9805（水柱 /World/BeakerWater）。
    火柴 (0.3314,0.1607) 头朝灯芯（2026-08-29 用户迭代位置：架后 10cm 再 -X/-Y 各 20cm，
    离架/皿都远）、灯帽静止位 (0.42,-0.2629)（随灯 -Y 平移）。

持握约定（同 B2/d2s）：药匙横夹 = ORIENT_FWD 手指朝前水平横夹柄杆（d2s 同款）；火柴/灯帽
= 纯平移持握（不随夹爪旋转，同 B2）。试管先立架孔里（B3 无试管夹、无沸石）。

挖粉链（PickSpatula ①-⑬）坐标 = D2-S 逐字：抓药匙 → 法兰 -45° → 对齐粉 x → 下降 24.5cm →
-Y 16cm → 舀粉 -45°→-90° → 抬到管口上 14cm → +Y 24cm → +X 11cm → 法兰回卷 +90° + -Y 14cm。
ReturnSpatula ⑭ 原路放回架孔。任务端粉下落动画仿 D2-S（PowderOnSpoon/PowderDrop/TubeSample）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度，同 d2s）
SPAT_LIFT_Z = 1.15  # pick ④ 竖直提起高度：药匙底部(勺尖)高于架顶 0.917 后加裕量。
SETTLE = 12         # 到点 settle 帧数

# —— 朝向（引擎 [w,x,y,z] 存储，scipy [x,y,z,w] 读法，同 d2s）——
ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 手指 +X（水平横夹药匙/横越）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_SPATULA = 0.008    # 药匙柄杆 Ø8mm（mesh 实测），目标开度 = 杆径

# —— 药匙（/World/Spatula，竖插于架中心孔；勺头在下、柄杆在上，rotZ-180°）——
# mesh：勺头 z 0.806-0.830（22mm 宽扁平），柄杆 z 0.830-0.963（Ø8mm 圆杆）。
# 原点（xform translate）在柄勺交界 z=0.828；抓点 = 原点上方 0.112m = 柄杆 z 0.94。
SPAT_XY = (0.6993, 0.3608)          # 药匙原点世界坐标（柄勺交界）
SPAT_GRASP_Z = 0.94                  # 抓点 z（柄杆上，架顶 0.917 之上可握段）
SPAT_GRASP = (SPAT_XY[0], SPAT_XY[1], SPAT_GRASP_Z)
SPAT_HEAD_DIST = 0.134               # 勺头尖到夹持点距离（勺头方向 = 夹爪局部 +X = tool X 行向量第 1 行）
SPAT_HANDLE_DIST = 0.023             # 柄顶到夹持点距离（反方向）

# —— 表面皿 / 粉末（/World/SurfaceDish (0.5365,0.105,0.80)，粉丘 scale0.4 贴皿顶）——
# 粉丘实测 bbox：x 0.5188-0.5542，y 0.0814-0.1288，z 0.8021-0.8141。
# 皿沿（rim）顶 z=0.8066 → 插入 z 必须 > 0.8066（过沿）且 < 0.8141（沉入粉）。
DISH_XY = (0.5365, 0.105)
POWDER_TOP_Z = 0.8141               # 粉丘顶
POWDER_Z = 0.809                     # 插入 z：勺尖 5mm 沉入粉丘（高于皿沿 2.4mm）
POWDER_X = 0.537                     # 粉堆中心 x
DROP_DOWN = 0.245                    # 第⑦步竖直下降量（勺尖 z=0.810，高于皿沿 3.4mm、沉入粉丘 ~4mm）
Y_SHIFT_NEG = 0.16                   # 第⑧步往 -Y 平移量

# —— 舀取段参考几何（flange roll 后药匙 45° 倾斜：勺头低柄高、凹槽开口朝 +Z 上）——
# 勺尖 = TCP + 0.134·toolX（toolX=世界 -Y）；碗心 = TCP + (0,-0.123,-0.0048)。
# 舀取平面 TCP z = POWDER_Z + 0.0048 = 0.8138：刃底(勺尖)0.809=粉顶下 5mm、刃沿 0.8138
# 仅低粉顶 0.3mm → 碗槽 4.8mm 深蓄粉；刃底高于皿沿 0.8066 → 推进全程不碰皿。
ORIENT_SCOOP = (0.5, 0.5, 0.5, 0.5)       # [w,x,y,z] scalar-first：flange roll 后 tool 朝向（参考值）
SCOOP_PLANE_Z = POWDER_Z + 0.0048         # 舀取平面 TCP z（=0.8138）
SCOOP_ALIGN_X = (DISH_XY[0], SPAT_XY[1], H)     # 法兰转完第一步：水平对齐粉末（参考）
SCOOP_APPROACH = (0.5365, 0.2678, SCOOP_PLANE_Z)   # 下降目标 TCP：勺尖在粉 +Y 沿外 5mm
SCOOP_INSERT   = (0.5365, 0.2278, SCOOP_PLANE_Z)   # 水平插入终点 TCP：碗心到粉丘中心 y=0.105
SCOOP_HOLD = 25                          # 插入到位停留帧数（粉灌入碗槽 / powder_on_spoon）
SCOOP_LIFT_Z = 0.95                      # 舀取后垂直提出高度

# —— 试管（/World/TestTube，架近侧左孔立插；挖粉⑩-⑬ 勺尖落到管口倒粉）——
TUBE_XY = (0.659, 0.241)             # 试管口中心（pxr 实测；管底 z 0.806、管口 z 0.9593）
TUBE_MOUTH_Z = 0.9593                # 试管口顶 z
LIFT_TUBE_Z = TUBE_MOUTH_Z + 0.14    # ⑩ 抬升目标 z：管口上方 14cm = 1.0993
# ⑪⑫ 水平移向试管（试管在架近侧左孔 (0.659,0.241)）：⑪ 回调 24cm 对准管口 y、⑫ 调 10cm 到管口前
Y_SHIFT_POS = 0.24                   # ⑪ 往 +Y 平移量（TCP y 0.2008→0.4408，勺尖 y 0.3068）
X_SHIFT_POS = 0.11                   # ⑫ 往 +X 平移量（TCP x 0.537→0.647，勺尖 x 0.647）
# ⑬ 法兰回卷 + 往 -Y 平移（边往 -y 移动边旋转法兰到 0°，同时开始同时结束）
Y_SHIFT_NEG_LAST = 0.14              # ⑬ 往 -Y 平移量（TCP y 0.4408→0.3008）；法兰 -90°→0°（回卷 90°）
POUR_TCP = (0.7538, 0.241, 1.0541)   # 倾倒点 TCP（POUR 朝向下勺尖 → 管口，柄顶避开洗瓶；y 随试管新孔位）
POUR_HOLD = 30                       # 倾斜倒入停留帧数

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

# —— 烧杯水浴（加热堆叠中心 (0.5286,-0.25)，B3 无试管浸入 → 气泡环带只避烧杯壁）——
BEAKER_XY = (0.5286, -0.25)       # 烧杯水柱中心（气泡环带/熔区参考中心，勿与试管 TUBE_XY 混淆）

# —— 试管水浴转移（阶段A'+E：倒粉后水平横夹试管提出架孔 → 纯平移分段转烧杯水浴浸入，机械臂
#    保持夹持不松爪；加热结束 ReturnTubePass 提起原路平移回架孔松爪放回）——
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
TUBE_POWDER_OFFSET_Z = 0.004           # 管内粉末球心距管底（球半径 0.006 贴底半圆，随管刚性跟随）
TUBE_MELT_OFFSET_Z = 0.006             # 管内熔化液球心距管底（球半径 0.008 贴底半圆，随管刚性跟随）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_BEAKER_BUBBLES = "/World/BeakerBubbles"   # 烧杯水浴气泡组（加热时逐个 reveal）
EFFECT_TUBE_MELT = "/World/TubeMelt"             # 试管内熔化液柱（前缀 + <色>，sample_phase=melted 时揭示）
