# -*- coding: utf-8 -*-
"""B5 熔点测定（提勒管法）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-30 pxr 读 b5_melting_point.usd 世界包围盒（gen_b5_scene.py verify 输出）。

本批次（用户 2026-08-31「算了还是换一种方法，同时把表面皿的位置放回去」弃「矩阵持握
+ 程序化旋转 90°/180°」）改「夹毛细管**端部**拎起，重力让水平细管自动变成竖直」——夹一端
往上拎，被夹端在上、自由端因重力垂到下面，自然竖直：
  ① 夹封口端(-X)拎起 → 开口端(+X)朝下 → 直接竖直插进粉丘蘸粉
  ② 放回桌面（倒成水平）
  ③ 夹开口端(+X)拎起 → 封口端(-X)朝下 → 竖直上下快速抖把粉从开口端震到封口端
端点物理（用户 2026-08-31 确认「按物理正确」）：蘸粉要开口端朝下 → 夹封口端(-X)；抖粉要
封口端朝下（粉落封口端）→ 夹开口端(+X)。

2026-09-01 用户验收三处修正：①「夹的不够靠端」→ 夹点从离端 2cm 挪到 2mm；②「机械臂就
只是直上直下的运动不要有其他的变化」→ 弃 RotateHeldAction 边提边转，拎起改纯竖直
MovePreserveAction；③「拎起来毛细管没有完全变竖直」→ 管身摆转改 task 侧 pivot 持握
（θ=swing_sign·90°·swing_frac 精确到 90°），不再靠机械臂旋转近似。
"""
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.constants import (
    SETTLE, GRIP_OPEN,
)

# —— 毛细管（/World/CapillaryTube，Ø1.5×100mm 平放台面上方 12mm）——
# 资产 capillary_tube.usd：管轴沿局部 Z（闭口端 z=0、开口端 z=0.100），场景 rot(0,90,0)
# 后局部 +Z → 世界 +X：闭口端（xform 原点）在 x=0.1710、开口端在 x=0.2710，管身水平沿 +X。
# 中心抬高 12mm（z=0.813，同火柴「抬高 12mm 防夹爪 collider 扎桌面」）——Ø1.5mm 极细管
# 平贴台面时夹爪手指会穿桌面，必须抬高才能夹住。
CAP_Z = 0.813                    # 毛细管中心 z
CAP_REST = (0.1710, 0.2704, CAP_Z)   # 闭口端（xform 原点）静止位
CAP_XY = (0.2210, 0.2704)            # 毛细管中心 x,y（闭口端 0.1710 + 半长 0.05）
CAP_HIGH = 1.05                  # 高位接近/提起（毛细管上方无遮挡，1.05 即可）

# 夹点（端部，离所夹端部 2mm 在直管身处——封口端是圆钝尖端、开口端是平口，都夹不住，
# 须夹直管身 Ø1.5mm 段；2026-09-01 用户「夹的不够靠端」→ 从离端 2cm 挪到 2mm）：
GRASP_SEALED = (0.1730, 0.2704, CAP_Z)  # 夹封口端侧（第一次蘸粉：拎起后开口端朝下）
GRASP_OPEN = (0.2690, 0.2704, CAP_Z)    # 夹开口端侧（第二次抖粉：拎起后封口端朝下）
GRIP_CAPILLARY = 0.00075             # 合爪开度 = 管外径 Ø1.5mm / 2（半开度，总开口 1.5mm）

# —— 拎起变竖直（新方法核心：夹端部拎起，重力让自由端下垂，弃程序化旋转）——
# 夹点离所夹端部 2mm，另一端距夹点 9.8cm（管长 10cm）。拎起竖直后自由端在夹点正下方 9.8cm。
# 摆转由 task 侧 pivot 持握驱动（θ=swing_sign·90°·swing_frac，精确 90°），机械臂只纯竖直
# 拎起、不旋转夹爪（2026-09-01 用户「机械臂就只是直上直下不要其他变化」）。
END_OFFSET = 0.098               # 自由端在夹点正下方距离（0.50-0.402 = 0.498-0.40 = 0.098）
LIFT_HIGH = 1.05                 # 拎起后夹点高位（自由端 0.952，离桌面 15cm 安全横移）
CAP_LIFT_Z = 1.20                # 开局清高位：先竖直上提到此再横移（温度计泡顶 1.084 / 架顶 0.897 上方，
                                 # 用户 2026-09-02「第一步就是机械臂往上抬不要直接去拿毛细管」避斜切穿模；
                                 # (0.173,0.2704,1.20) 距底座 0.791m < 0.855m 可达）
SWING_THRESHOLD_Z = 0.85         # 摆转阈值：TCP 高于此 → 竖直（frac→1），低于 → 水平（frac→0）
SWING_FRAMES = 20                # 摆转完成帧数（≈0.33s，重力摆快速甩直）

# 粉丘（/World/SamplePowder，表面皿已放回原位 (0.4433,0.1488)）：
POWDER_XY = (0.4451, 0.1430)      # 粉丘中心 x,y（= gen_b5_scene.py POWDER_T）
POWDER_TOP_Z = 0.814              # 粉丘顶 z
DIP_OPEN_Z = 0.809                # 开口端下探目标 z（沉入粉丘 5mm，同 d2s POWDER_Z）
# 夹封口端时开口端在夹点下方 END_OFFSET=0.098，开口端 0.809 → 夹点 z = 0.907
DIP_SEALED_Z = DIP_OPEN_Z + END_OFFSET   # 0.907 = 夹封口端下探 z

# —— 震实（第二次夹开口端拎起后，封口端朝下，竖直上下快速来回）——
TAMP_CENTER_Z = LIFT_HIGH         # 震实中心 = 拎起高位（就地抖；封口端最低 0.907 不撞桌面）
TAMP_AMP = 0.045                  # 振幅（±4.5cm）
TAMP_CYCLES = 10                  # 来回次数（用户指定 10 次）
TAMP_PERIOD = 15                  # 一个来回帧数（60Hz 下 ≈0.25s，快速）

# —— 温度计 + 蘸油 + 插管（2026-09-02 用户改「倒插试管架」弃旋转竖直）——
# 主温度计 /World/MainThermometer：倒插试管架左前孔（rot(180,0,0)，泡朝上），origin（泡尖
# local z=0）world (0.3471,0.2696,1.0822)。倒插后局部 +Z→世界 −Z：
#   泡 local z[-0.002,0.014] → world z[1.068,1.084]（泡中心 1.076）；杆 local z[0.008,0.268]
#   → world z[0.814,1.074]；塞 local z[0.125,0.149] → world z[0.933,0.957]；挂环 local
#   z[0.2628,0.2762] → world z[0.806,0.819]（挂环顶 0.806 = 架底板顶）。
# 臂手指朝前 ORIENT_FWD 水平横夹竖直杆身（d2s 夹药匙同款），夹点选塞子(0.957)上方、泡(1.068)
# 下方——local z=0.082 → world (0.3471,0.2696,1.000)。竖直提出（挂环 1.25−0.276=0.974 清架顶
# 0.914）后只用法兰（panda_joint7）滚 FLANGE_ANGLE=−166°（限位 ±166°）把泡翻朝下，再 IK 校直
# 剩余 ~14°（mv orient=ORIENT_VERT）。泡底（泡尖远端 local z=−0.002）到夹点距离 = 0.082−(−0.002)
# = 0.084 = THERMO_BULB_DZ。
ORIENT_FWD = (0.0, 0.7071, 0.0, 0.7071)   # 手指 +X（d2s/b2 已验证）：横夹竖直杆身（泡朝上）
THERMO_REST = (0.3471, 0.2696, 1.0822)    # 温度计原点（泡尖）静止位（倒插左前孔，泡朝上）
THERMO_GRASP = (0.3471, 0.2696, 1.000)    # 夹杆身夹点（塞子 0.957 上方、泡 1.068 下方）
GRIP_THERMOMETER = 0.004                  # 合爪开度 = 杆 Ø8mm / 2
THERMO_BULB_DZ = 0.084                    # 泡底到夹点竖直距离（局部 z 0.082−(−0.002)=0.084）
THERMO_APPROACH_X = 0.08                  # 抓取 -X 侧接近偏移（从泡下方横移 +X 到杆身，避从泡正上方下探穿模）
THERMO_HIGH = 1.25                        # 提起/清高位（挂环 1.25−0.276=0.974 清架顶 0.914）
FLANGE_ANGLE = -166.0                     # 法兰滚动角（负向翻转泡朝下，IK 校直补 ~14°）

# 蘸油皿（/World/OilDish 培养皿 + /World/OilDishLiquid 石蜡油薄层，台面 (0.25,0.15)）：
OIL_DISH_XY = (0.25, 0.15)
OIL_DIP_BULB_Z = 0.804                  # 蘸油时泡底 z（沉入油层 0.802-0.806）
# 贴毛细管：泡底贴毛细管封口端（闭口端静止位 x=0.1710，中心 z=0.813）
ATTACH_XY = (0.1710, 0.2704)            # 毛细管封口端 x,y
ATTACH_BULB_Z = 0.813                   # 贴毛细管时泡底 z（= 封口端中心）
# 插提勒管：橡胶塞中心（局部 z=0.137）封管口 z=1.078 → 泡底 z = 1.078−0.139
TUBE_XY = (0.400, 0.0059)              # 提勒管口 x,y（= 主管轴 gen TUBE_X 0.400，见 task OIL_TUBE_XY；2026-09-02 曾误校 -0.003 到 0.397，2026-09-03 用户报「下落温度计还是没对准需向+x再移0.3cm」→ 0.397→0.400 回到主管轴，同 OIL_TUBE_XY.x）
TUBE_MOUTH_Z = 1.078                    # 提勒管口顶 z
INSERT_BULB_Z = TUBE_MOUTH_Z - 0.139    # 0.939 插管后泡底 z（塞子封口）
# 2026-09-02 用户改「高位对准后直接松爪，温度计靠重力落进管口、塞子正好对准」——弃竖直下探，
# 松爪后 task 侧落体动画（DROP_FRAMES 帧加速下落）把温度计从高位降到插管终态：
INSERT_THERMO_ORIGIN_Z = TUBE_MOUTH_Z - 0.137  # 0.941 插管后温度计 origin（泡尖 local z=0）z：塞中心 0.137 封口
DROP_FRAMES = 15                        # 松爪后温度计自由落体帧数（task 侧落体动画，grip dwell 内播完）

# —— 毛细管蘸油 + 贴温度计泡（2026-09-02 用户改流程：蘸油后放回，再夹端部拎起竖直贴泡）——
# 震实放回桌面后，从**中部**夹起毛细管保持**水平**（矩阵持握）移到油皿蘸油（封口端沉油），
# 蘸完**放回桌面**；再**夹封口端拎起**（pivot 竖直，封口端朝上、开口端垂下，同 ① 蘸粉姿势），
# 移到倒插温度计泡旁，封口端（=TCP，夹封口端时 TCP 即封口端）竖直贴泡靠油膜吸附。弃旧「中部
# 水平直接横着贴泡」——毛细管横着贴泡不符合物理（真实实验毛细管竖直平行贴温度计杆、封口端在泡位）。
CAP_MID = (0.2210, 0.2704, CAP_Z)      # 毛细管中部夹点（封口端 0.1710 + 半长 0.05）
CAP_HELD_OFFSET = 0.05                 # 持握矩阵平移：封口端(局部z=0)到夹点(局部z=0.05)距离
# 封口端在夹点 −0.05·世界+X（手指朝下 tool+X→−X）：蘸油的 TCP = 封口端目标 + (0.05,0,0)。
OIL_DIP_GRIP = (0.27, 0.15)            # 蘸油 TCP xy：封口端在油皿 (0.22,0.15)（用户 2026-09-02 再 -X 3cm）
OIL_DIP_CAP_Z = 0.804                  # 蘸油时毛细管中心 z（管底 0.803 沉入油层 0.802-0.806）
STICK_SEALED = (0.3471, 0.2706, 1.076)  # 竖贴泡 TCP 目标（夹封口端后 TCP=封口端）= 倒插泡中心 (0.3471,0.2696) +Y 侧；
                                         # 2026-09-02 用户报「毛细管贴温度计不够近偏+y 0.5cm」→ 0.2756→0.2706（-Y 0.5cm 贴紧泡）
STICK_APPROACH_Z = 1.15                # 竖贴泡横移高位（泡顶 1.084 上方，横移到泡旁再下探贴泡）
# 插管安全高度：法兰翻转后泡朝下，泡底在夹点下方 THERMO_BULB_DZ=0.084。清高位须泡底 > 管口 1.078
# 才横移 → 夹点清高 ≥ 1.078 + 0.084 = 1.162，取 1.22（2026-09-02 用户报「最后一步松爪子的高度有点
# 太高了，比当前下降 13cm」1.35→1.22；对齐 (0.397,0.0059,1.22) 距底座 [-0.048,-0.311,0.71] 0.747m <
# 0.885m 极限，泡底 1.136 > 管口 1.078 仍清空）。pick 已提至 THERMO_HIGH=1.25，insert ① 先竖直上提到
# 1.22 再横移对齐（先上移再横移，避免从温度计斜切穿提勒管/夹）。
INSERT_CLEAR_Z = 1.22                    # 插管清高位（泡底 1.136 > 管口 1.078；1.18→1.35→1.22 用户要求降 13cm）
ORIENT_VERT = (0.7071068, 0.0, 0.7071068, 0.0)  # = Rx(180°)·ORIENT_FWD：法兰翻转后泡朝下目标朝向（IK 校直）

# —— 酒精灯 + 火柴（⑬ 点火，2026-09-02 用户「拿起火柴点燃酒精灯」）——
# 酒精灯 (LAMP_X=0.355, TUBE_Y=0.0029) rot180，灯体顶 0.8897、灯芯顶 z≈0.9007（同 B2/C4
# alcohol_lamp.usd pxr 实测，取 0.9005 同 B2）。火柴 (0.4817,-0.164,0.813) 抬高 13mm（灯 +X -Y
# 侧右下），杆 Ø3mm、头朝 +X。火柴纯平移持握（头朝 +X 不随夹爪旋转，手指朝下竖直夹杆身，
# 照 B2/C4）。夹爪从抓点 x=0.5217 斜推进到 IGNITE（头中心=WICK）点火。
LAMP_XY = (0.355, 0.0029)                    # 酒精灯中心（= 侧管下弯处正下方）
WICK = (LAMP_XY[0], LAMP_XY[1], 0.9005)      # 灯芯顶（B2 pxr 0.9007，取 0.9005）
MATCH_XY = (0.4817, -0.164)                  # 火柴原点（杆 -X 端）x,y（tmp 2026-09-02 重摆）
MATCH_REST_Z = 0.813                         # 火柴原点 z（抬高 13mm 防夹爪扎桌面，同 B2）
MATCH_GRASP_OFFSET = 0.04                    # 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1],
               MATCH_REST_Z + 0.0015)        # (0.5217,-0.164,0.8145) 杆中心 z=+0.0015
GRIP_MATCH = 0.0015                          # 合爪开度 = 杆身 Ø3mm / 2
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)        # 头中心相对夹爪（头 x=0.0894 − 抓点 0.04）
MATCH_LIFT_Z = 0.90                          # 夹起后低位运移高度（高于灯体顶 0.8897、低于提勒管底 0.928）
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])  # (0.3056,0.0029,0.9005) 夹爪目标，
                                                             #   头中心=WICK；斜推进点火
MATCH_HIGH = 1.15                            # 高位接近火柴（>提勒管口 1.078 清障，同 C4 H）

# —— 火焰效果 prim 路径（⑬ 点火后 task 每帧 flicker）——
# 火焰 = /World/flame_outer_grp + /World/flame_inner_grp（水滴形组，pivot=火焰底，gen 生成
# 初始隐藏）。task 点火 reveal + 每帧 scale(高/宽)+rotateXYZ(侧摆) flicker（用户 2026-09-02
# 「火焰应该是水滴型然后在动比较逼真，模仿 C3/C4」）。
FLAME_GRPS = ("/World/flame_outer_grp", "/World/flame_inner_grp")
FLAME_PRIMS = ("/World/flame_outer_grp/flame_outer",
               "/World/flame_outer_grp/flame_outer_sphere",
               "/World/flame_inner_grp/flame_inner",
               "/World/flame_inner_grp/flame_inner_sphere")

# —— 酒精灯夹灯加热摆动 15s + 移灯 -X 5cm + 盖帽（⑭⑮⑯ 加热/移灯/熄火）——
# 用户 2026-09-03 逐字：「加热的时候要停留15s，这个期间机械臂应该夹住酒精灯在x方向上来回
# 移动，来控制升温速度，最后盖酒精灯帽前，应该先把酒精灯往-x移动5cm移出再盖酒精灯。夹酒精灯
# 参考B2、B3。」——照 B2/B3 LampMovePass（水平横夹灯体宽处 z=0.845，ORIENT_FWD 手指朝前）
# + 加热期 X 轴正弦摆动 15s（ShakeAction axis=X，控温）+ 移灯 -X 5cm + CapLampPass 盖帽熄火。
# B5 灯 (0.355,0.0029) 在提勒管侧臂下弯处正下方（加热点 x=0.355、火焰尖 0.9795），移灯 -X 5cm
# → (0.305,0.0029) 移出侧臂下方，帽可竖直下扣不穿侧臂（侧臂 x≥0.355）。
LAMP_REST_Z = 0.8002                    # 灯底座中心 z（同 B2/B3，alcohol_lamp.usd 底座贴台面）
LAMP_BODY_Z = 0.845                     # 灯体宽处世界 z（Ø76.8mm，同 B2/B3 可握）
LAMP_GRASP_OFFSET = LAMP_BODY_Z - LAMP_REST_Z   # 0.0448（_LAMP_HELD 平移，灯原点在夹爪正下方）
LAMP_GRASP = (LAMP_XY[0], LAMP_XY[1], LAMP_BODY_Z)   # 灯体宽处抓点 (0.355,0.0029,0.845)
GRIP_LAMP = 0.038                       # 合爪开度 ≈ Ø76mm（同 B2/B3）
LAMP_CLOSED_THRESHOLD = 0.039           # 灯 attach 阈值（灯体宽，同 B2/B3）
LAMP_OPEN_THRESHOLD = 0.0395            # 灯 release 阈值（>GRIP_LAMP 才真松爪，同 B2/B3）
LAMP_MOVE = 0.05                        # 移灯距离 5cm（水平 -X，用户「往-x移动5cm移出」）
LAMP_TARGET = (LAMP_XY[0] - LAMP_MOVE, LAMP_XY[1], LAMP_BODY_Z)  # 移灯终点夹爪 (0.305,0.0029,0.845)
LAMP_APPROACH = (LAMP_XY[0] - 0.11, LAMP_XY[1], LAMP_BODY_Z)     # 灯 -X 侧准备（只 +X 移入）(0.245,0.0029,0.845)
LAMP_HIGH = (LAMP_APPROACH[0], LAMP_APPROACH[1], 1.25)           # 高位中转（-X 侧上方，从上方竖直降）(0.245,0.0029,1.25)

# 加热摆动（夹住灯后在 x 方向来回移动 15s 控温）：
HEAT_SWAY_AMP = 0.012                   # 摆动振幅 ±12mm（火焰尖在加热点附近小范围往复）
HEAT_SWAY_CYCLES = 15                   # 15 个来回 = 15s（period 60 帧 ≈ 1s）
HEAT_SWAY_PERIOD = 60                   # 一个来回 60 帧（1s）

# —— 灯帽（⑯ 移灯后盖帽熄火；同 B2/B3，先移灯 -X 5cm 再 CapLampPass 盖帽）——
# 帽摘放灯 -X 侧 12cm 台面（gen CAP_DETACH：帽心 (0.235,0.0029,0.8155)）。盖灯口 = 移灯后灯口
# (0.305,0.0029)。帽中心 = 灯z0.8002 + LAMPCAP_CENTER_DZ0.0915 = 0.8917、帽底 0.8762 盖严实
# （同 B2 资产原始帽位；盖住灯芯 0.9005 与灯体顶 0.8897）。
# 前缀 LAMPCAP_ 区分毛细管常量（CAP_REST/CAP_HIGH/CAP_HELD_OFFSET 已被毛细管占用）。
LAMPCAP_CENTER_DZ = 0.0915
LAMPCAP_REST = (LAMP_XY[0] - 0.12, LAMP_XY[1], 0.8155)   # 帽静止位世界中心 (0.235,0.0029,0.8155)
LAMPCAP_GRASP = (LAMPCAP_REST[0], LAMPCAP_REST[1], 0.824)  # 夹帽点（帽顶 0.831 下 7mm，同帽心水平）
LAMPCAP_HIGH = 1.05                                      # 高位（取帽/运帽；>火焰尖 flicker 上限 ~0.996 清障）
LAMPCAP_HELD_OFFSET = (0.0, 0.0, -0.0083)                # 纯平移持握：帽中心 = 夹爪 + offset
LAMPCAP_BURNER = (LAMP_TARGET[0], LAMP_TARGET[1], 0.900)  # 盖灯口夹爪（移灯后灯口 0.305；帽中心 0.8917）
GRIP_CAP = 0.0185                                         # 帽 Ø37mm / 2
LAMPCAP_CLOSED_THRESHOLD = 0.022
LAMPCAP_COVER_NEAR = 0.010
LAMPCAP_EXTINGUISH_XY = 0.06
LAMPCAP_EXTINGUISH_Z = 1.00                               # 下落即熄火 z 门控（帽底 0.976 罩过火焰尖 0.9795 才灭）
