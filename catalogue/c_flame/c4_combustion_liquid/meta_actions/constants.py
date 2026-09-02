"""C4 燃烧试验（液体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-09-01 gen_c4_scene.py verify 输出（C3 定稿布局 + d3l 液体区）：
  TestTubeRack   (0.72,0.42)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  CombustionSpoon (0.636,0.3093,0.8068) 碗贴台、把手斜靠试管架（碗口 z=0.8068、
      碗底 0.7966、把手顶 1.097；无旋转，原厂"碗朝上把手斜向上 +X"）
  AlcoholLamp    (0.45,0.05)  rot180；火焰 /World 顶层（初始隐藏，base 0.900 apex 0.936）
  SampleBottle   (0.50,0.55)  瓶口 rim z=0.870，瓶内液面 z=0.840（半瓶）；
      瓶盖 stopper 摘下倒放瓶旁桌面（瓶 -X 侧 45mm，密封面朝上，用户 09-01）
  Dropper        (0.7385,0.5395,0.806)  右列**最后一排（第7排）**孔（用户 09-01 两改：
      第1排→第3排→定稿最后一排，y=0.5395 远离燃烧匙把手/药品瓶；右孔避开斜靠把手），
      尖嘴底=原点，胶头顶 z=0.9562
  Match          (0.62,0.05,0.813)  抬高 13mm

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，
d3l 同款）：滴管尖嘴 0.13m 吊在夹爪下方。故
  TCP z = 尖嘴 z + 0.13
抓点 = 立放位 + (0,0,0.13)：滴管尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段）。
"""
# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管）
# 胶头物理直径 Ø11mm（d3l pxr 实测）。Franka 开度 ≈ 指间距一半 → "正好贴胶头面"
# = 0.011/2 = 0.0055（用户 2026-08-14 实测 0.0055 正合适，0.011 太松）。
GRIP_DROPPER = 0.0055       # 抓滴管：移动/持握开度（贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴液）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）

# —— 滴管尖嘴到夹爪距离（持握 HELD_OFFSET 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# —— 滴管（/World/Dropper，右列最后一排[第7排]孔）——
DROP_XY = (0.7385, 0.5395)
DROP_GRASP = (0.7385, 0.5395, 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上）
DROP_REST = (0.7385, 0.5395, 0.806)   # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）

# —— 药品瓶（/World/SampleBottle (0.50,0.55)，口 rim 0.870，液面 0.840；
#    瓶盖 stopper 摘下倒放瓶旁，见 gen_c4_scene.py flip_bottle_stopper）——
SAMPLE_BOTTLE_XY = (0.50, 0.55)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去 stopper 后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840 ✓
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP：
BOTTLE_SQUEEZE_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
SAMPLE_DIP_TCP = (SAMPLE_BOTTLE_XY[0], SAMPLE_BOTTLE_XY[1],
                  BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 燃烧匙碗（/World/CombustionSpoon (0.636,0.3093)，碗口 z=0.8068）——
SPOON_XY = (0.636, 0.3093)
SPOON_MOUTH_Z = 0.8068       # 碗口世界 z（碗贴台面、把手斜靠试管架）
SPOON_BOWL_BOTTOM = 0.7966   # 碗底最低点世界 z（碗液底面 = +0.002 = 0.7986，避曲面）
# 滴加：尖嘴抬到碗口上方 25mm（液滴从尖嘴坠落入碗可见）
SPOON_DROP_RAISE = 0.025
SPOON_DROP_TCP = (SPOON_XY[0], SPOON_XY[1],
                  SPOON_MOUTH_Z + SPOON_DROP_RAISE + TIP_OFFSET)     # = 0.9618

# —— 酒精灯 + 火柴（/World/AlcoholLamp (0.7094,-0.10) rot180，灯芯顶 z=0.9007；
#    火柴 /World/Match (0.62,0.05) 抬 13mm，杆身 Ø3mm、头朝 +X）——
# 2026-09-01 用户「酒精灯和灯帽整体移动到火柴的正对 -y」：灯 (0.45,0.05)→(0.7094,-0.10)，
# 灯芯 (0.7094,-0.10,0.9007) 在火柴头 (0.7094,0.05) 正下方。夹爪从抓点 x=0.66 直推 -Y 到
# IGNITE (0.66,-0.10,0.9007)（gripper x 不变 = 纯直进），头落灯芯点火；回程纯 +Y 直退，
# 火柴头从火焰 (0.7094,-0.10) 侧向撤走，永不横向扫过火焰柱——无需再抬升绕焰。
# 火柴持握 = 纯平移 offset（照 B2：火柴水平头朝 +X，不随夹爪旋转；夹爪手指朝下竖直夹杆）。
# 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯，同 B2 match.usd）。
LAMP_XY = (0.7094, -0.10)
WICK = (LAMP_XY[0], LAMP_XY[1], 0.9007)   # 酒精灯灯芯顶（整灯 bbox max z 实测 0.9007，
                                          #  同 B2 资产 alcohol_lamp.usd；gen 火焰底 0.900）
MATCH_XY = (0.62, 0.05)
MATCH_REST_Z = 0.813            # 火柴原点（杆 -X 端）z，抬 13mm（gen MATCH_T，同 B2 火柴抬 12mm 口径）
MATCH_GRASP_OFFSET = 0.04       # 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1],
               MATCH_REST_Z + 0.0015)        # 杆中心 z=0.0015 → (0.66,0.05,0.8145)
GRIP_MATCH = 0.0015             # 合爪开度 = 杆身 Ø3mm / 2
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)   # 头中心相对夹爪（头 x=0.0894 − 抓点 0.04，z 同）
MATCH_LIFT_Z = 0.90             # 夹起后低位运移高度（高于灯体顶 0.8897 横越灯体不穿；头 z 0.8992 落灯芯）
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])   # (0.66,-0.10,0.9007) 夹爪=抓点 x，
                                                              #  头中心=WICK；直推 -Y 点火
MATCH_HIGH = H                  # 高位接近火柴（C4 无温度计/试管挡路，同滴管安全高 H）

# —— 燃烧匙入外焰（阶段 ③ 用户 09-01「水平横夹起燃烧匙，将勺子部分移动到外焰部分」）——
# 燃烧匙 = 碗贴台（碗口 z 0.8068）+ 杆斜靠试管架（无旋转）；杆 Ø3mm，中心线从碗旁
# (0.6446,0.3093,0.8064) 到杆顶 (0.7251,0.3093,1.097)。水平横夹 = 手指朝下竖直下探到
# 杆身横跨夹住（同火柴夹杆）。抓点 = z 0.98 处杆中心 (0.6927,0.3093,0.98)
# （高于试管架顶 0.917 的 63mm，下探/横夹安全）。
# 纯平移持握：勺原点（碗口平面）= 夹爪 + SPOON_HELD_OFFSET，姿态不变。
# 碗口放到外焰中心 BOWL_AT_FLAME（灯上方，z 0.925 = 外焰锥内 apex 0.936 下 11mm），
# 夹爪即 FLAME_HOLD_TCP（杆上同一点，几何自洽）。
SPOON_GRASP = (0.6927, 0.3093, 0.98)            # 杆身横夹点（z=0.98 杆中心）
GRIP_SPOON = 0.0015                              # 合爪开度 = 杆身 Ø3mm / 2
SPOON_HELD_OFFSET = (SPOON_XY[0] - SPOON_GRASP[0], 0.0,
                     SPOON_MOUTH_Z - SPOON_GRASP[2])   # 勺原点相对夹爪 = (-0.0567,0,-0.1732)
SPOON_REST = (SPOON_XY[0], SPOON_XY[1], SPOON_MOUTH_Z)  # 勺原点台面静止位
SPOON_LIQUID_OFFSET = (0.0, 0.0, SPOON_BOWL_BOTTOM + 0.004 - SPOON_MOUTH_Z)  # 碗液中心相对勺原点 (0,0,-0.0062)
BOWL_AT_FLAME = (LAMP_XY[0], LAMP_XY[1], 0.925)  # 碗口（勺原点）目标 = 外焰中心
FLAME_HOLD_TCP = (BOWL_AT_FLAME[0] - SPOON_HELD_OFFSET[0],
                  BOWL_AT_FLAME[1] - SPOON_HELD_OFFSET[1],
                  BOWL_AT_FLAME[2] - SPOON_HELD_OFFSET[2])   # = (0.7661,-0.10,1.0982)
SPOON_LIFT_Z = H                                  # 提出/运移高度（碗 z = 1.15−0.173 = 0.977 > 架顶 0.917）
FLAME_DWELL = 240                                 # 碗在外焰停留帧数（4s@60fps，用户 09-02「改 4 秒」）
# —— 观察停留（阶段 ③ 燃烧 4s 后，往 +y 移 5cm 离开酒精灯，停留 10s 观察，用户 09-02）——
OBSERVE_OFFSET_Y = 0.05                            # 离开酒精灯：往 +y 方向水平移 5cm
OBSERVE_BOWL = (BOWL_AT_FLAME[0], BOWL_AT_FLAME[1] + OBSERVE_OFFSET_Y, BOWL_AT_FLAME[2])
                                                   # 碗口（勺原点）观察位 = 外焰中心 +y 5cm（同高度）
OBSERVE_TCP = (OBSERVE_BOWL[0] - SPOON_HELD_OFFSET[0],
               OBSERVE_BOWL[1] - SPOON_HELD_OFFSET[1],
               OBSERVE_BOWL[2] - SPOON_HELD_OFFSET[2])   # = (0.7661,-0.05,1.0982)
OBSERVE_DWELL = 600                                # 观察停留帧数（10s@60fps，用户 09-02「停留不动 10s 观察」）

# —— 液体燃烧现象（阶段 ③ dwell 期间，用户 09-01「增加现象」+「火焰要动起来」）——
# config combustion 双现象：combustible=碗液面点燃淡蓝火焰+液面渐降烧尽；non_combustible=
# 沸腾冒泡蒸发（无火焰）。火焰（酒精灯+液面）每帧 flicker（scale 高/宽 + rotate 侧摆，
# pivot=火焰底，task._step_flame_anim/_apply_flame_flicker 驱动，用户 09-01「火焰要动起来
# 不要不动不然太假了」）。
SPOON_FLAME_NEAR = 0.025          # 碗在火焰判定近窗：夹爪距 FLAME_HOLD_TCP < 2.5cm（dwell）
IGNITION_DELAY = 40               # 点火延迟帧数（液体受热升温，~0.67s@60fps）
BURN_TOTAL = 0.008                # 燃烧消耗总量（碗液满高 8mm → 烧尽 0）
BURN_FRAMES = 480                 # 烧尽帧数（~8s，留 dwell 末 ~2s 空勺）
BURN_STEP = BURN_TOTAL / BURN_FRAMES   # 每帧液面下降量（燃烧）
BURN_OUT_AT = 0.0008              # 液面低于此即算烧尽（隐藏液面火焰）
BOIL_DELAY = 30                   # 沸腾冒泡延迟帧数（不可燃，受热更快冒泡）
EVAP_TOTAL = 0.0032               # 蒸发消耗总量（不可燃：10s 只蒸掉 ~40%）
EVAP_FRAMES = 600                 # 蒸发参考帧数（固定物理速率：10s 蒸掉 EVAP_TOTAL，不随 FLAME_DWELL 变）
EVAP_STEP = EVAP_TOTAL / EVAP_FRAMES   # 每帧液面下降量（蒸发，比燃烧慢）
BOIL_CYCLE = 36                   # 单泡上升周期帧数（~0.6s@60fps）
SPOON_BUBBLE_RISE = 0.020         # 气泡上升距离（液面上 20mm）
SPOON_BUBBLE_N = 6                # 气泡数（错帧循环）

# —— 酒精灯帽盖灭（阶段 ⑤ 用户 09-02「燃烧 4s → +y 5cm 观察 10s → 放回，然后盖上酒精灯帽熄灭火焰」）——
# 帽 = /World/AlcoholLamp/cap（灯子 prim），Ø37mm×3.1cm 开口朝下倒扣桌面（pxr 实测
# bbox x[0.5708,0.608] y[-0.1186,-0.0814] z[0.800,0.831]：中心 z 0.8155、顶 0.831、底 0.800）。
# 帽静止位 = 灯旁 -X 12cm 桌面 (LAMP_XY[0]−0.12, LAMP_XY[1], 0.8155)（gen CAP_DETACH translate
# (0.12,0,-0.0762) 随灯 R180 → 帽世界 x=灯x−0.12）。盖严实 = 帽中心 灯z+CAP_CENTER_DZ=0.8915
# （帽底 0.8760 低于灯体顶 0.8897 13.7mm，帽 local translate → (0,0,0) = 资产原始帽位）。
# 夹爪 CAP_BURNER 0.900 = 帽中心 + 持握偏移 0.0085（B2 六改：帽要盖严实，不能只搭灯口沿）。
# 全程默认朝向手指朝下（B2 三改：低 z 桌面夹帽 ORIENT_FWD 手指朝前 Lula 无解；纯平移持握，
# 帽竖直开口朝下不旋转）。火焰熄灭时机照 B2 十一改：帽下降罩过火焰顶才熄，不移动时早灭。
CAP_CENTER_DZ = 0.0915            # 帽中心到灯底座 z 偏移（同 B2 alcohol_lamp.usd 资产）
CAP_REST = (LAMP_XY[0] - 0.12, LAMP_XY[1], 0.8155)   # 帽静止位世界中心（灯旁 -X 12cm 桌面）
CAP_GRASP = (LAMP_XY[0] - 0.12, LAMP_XY[1], 0.824)   # 夹帽点（帽顶 0.831 下 7mm，同 CAP_REST 水平）
GRIP_CAP = 0.0185                 # 合爪开度 = 帽 Ø37mm / 2
CAP_CLOSED_THRESHOLD = 0.022      # 帽 attach 阈值（合到 0.0185 < 0.022）
CAP_HELD_OFFSET = (0.0, 0.0, -0.0085)   # 纯平移持握：帽中心 = 夹爪 + offset（夹点 0.824 − 帽中心 0.8155）
CAP_HIGH = 1.00                   # 高位（取帽/运帽，高于火焰顶 0.936 清障）
CAP_BURNER = (LAMP_XY[0], LAMP_XY[1], 0.900)   # 盖灯口夹爪（帽中心 0.8915、帽底 0.8760 < 灯体顶
                                       #   0.8897 盖严实；同 B2 CAP_BURNER 0.900）
CAP_COVER_NEAR = 0.010            # 盖到位判定：夹爪距 CAP_BURNER < 1cm（B2 四改：4cm 太松，
                                  #   下降途中离灯口 3cm 就判盖到位，视频断在"没盖完"）
CAP_EXTINGUISH_XY = 0.06          # 下落即熄火 xy 门控（夹爪 xy 距 CAP_BURNER < 6cm 才熄）
CAP_EXTINGUISH_Z = 0.963          # 下落即熄火 z 门控（帽底 = 夹爪−0.024 刚罩过火焰顶 0.936
                                  #   才灭：0.936+0.003+0.024；火焰保持到帽盖住顶部，帽继续降不穿火）

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_SPOON_LIQUID = "/World/SpoonLiquid"       # 碗内液体（首滴落定后显示，与瓶液同色）
EFFECT_DROPPER_FILL = "/World/DropperFill"       # 滴管尖内截锥液柱（吸液后显示，跟随尖嘴；
                                                 #  几何在 gen_c4_scene.py，task 只需 translate=尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"       # 挤胶头滴落时从尖嘴掉落的液滴（task 动画坠落）
EFFECT_FLAME_OUTER = "/World/flame_outer"        # 酒精灯外焰（水滴形=底半球 Sphere+上部 Cone，
                                                 #  迁到 /World 顶层；gen 初始隐藏，task 点着 reveal）
EFFECT_FLAME_INNER = "/World/flame_inner"        # 内焰（同上，apex 0.922 低于外焰 0.936）
EFFECT_LAMP_FLAME_GRPS = ("/World/flame_outer_grp", "/World/flame_inner_grp")  # 酒精灯火焰组
                                                 #   （pivot=火焰底，task 每帧 flicker）
EFFECT_SPOON_FLAME = "/World/SpoonFlame_{color}_grp"     # 液面燃烧火焰组（pivot=火焰底=液面，
                                                         #   {color}=cfg.flame_color，task 写 translate 跟随液面 + flicker）
EFFECT_SPOON_FLAME_CONE = "/World/SpoonFlame_{color}_grp/SpoonFlame_{color}"
EFFECT_SPOON_FLAME_SPHERE = "/World/SpoonFlame_{color}_grp/SpoonFlame_{color}_sphere"
FLAME_COLOR_OPTIONS = ("blue", "yellow", "orange", "red", "green", "purple")  # 液面火焰候选色
EFFECT_SPOON_BUBBLE = "/World/SpoonBubble"       # 沸腾气泡父（Bubble_0..N，task 上升循环）
