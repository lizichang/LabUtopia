"""C3 燃烧试验（固体样品）元动作共享常量。

用户 2026-09-01 指示（逐字）：「把机械臂的位置和药匙和表面皿的位置跟 d2s 保持一致」——
挖粉段（横夹药匙 + 挖粉）坐标**逐字对齐 d2s**（药匙/表面皿/粉末/试管架/机械臂底座全同），
仅 C3 专属器材（燃烧匙/酒精灯/火柴）另摆。9 个挖粉子动作（FlangeRollAction / AlignPowderX /
LowerPowder / ShiftYNeg / ScoopUpAction / LiftToTube）直接复用 d2s 元动作包，参数与 d2s 完全一致。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何来源：scripts/gen_c3_scene.py verify 输出（2026-09-01 pxr bbox）：
  TestTubeRack  (0.6803,0.3607)  z 0.800..0.917（顶板 0.917，插孔底面 z=0.806）
  Spatula       (0.6993,0.3608,0.828)  立插架中心孔（rotZ -180°，勺头扁平面沿 X，同 d2s）
  SurfaceDish   (0.5365,0.105,0.80)    皿沿顶 z=0.8066
  SamplePowder  (0.5383,0.0992)        粉丘 bbox x[0.5188,0.5542] y[0.0814,0.1288] z[0.8021,0.8141]
  CombustionSpoon (0.596,0.250)        燃烧匙碗（后续倒粉目标，本阶段不参与挖粉）
  AlcoholLamp   (0.40,0.18)            酒精灯（点火/加热；2026-09-01 挪离底座 +x/+y）
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SPAT_LIFT_Z = 1.15  # pick ④ 竖直提起高度：药匙底部(勺尖)高于架顶 0.917 后加裕量。
                    #   勺尖 = 1.15-0.134 = 1.016 > 0.917+0.05 ✓（同 d2s，架顶不变）
SETTLE = 12         # 到点 settle 帧数（同 d2s）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_SPATULA = 0.008    # 药匙柄杆 Ø8mm（mesh 实测），目标开度 = 杆径（同 d2s）

# —— 药匙朝向（四元数 = scipy [x,y,z,w] 序，存储元组直接喂 from_quat；同 d2s）——
# 用户要求模仿 level4_liquidmixing「爪子朝前，朝向camera1，夹起」：手指 tool+Z=(1,0,0)
# 朝 +X；药匙 tool+X=(0,0,-1) 头下柄上。attach 后药匙世界 = REST 零跳变。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)

# —— 药匙（/World/Spatula，竖插架中心孔；勺头在下、柄杆在上）——
# mesh：勺头 z 0.806-0.830（22mm 宽扁平），柄杆 z 0.830-0.963（Ø8mm 圆杆）。
# 原点（xform translate）在柄勺交界 z=0.828；抓点 = 原点上方 0.112m = 柄杆 z 0.94。
# 与 d2s 逐字一致（用户要求对齐 d2s）。
SPAT_XY = (0.6993, 0.3608)          # 药匙原点世界坐标（柄勺交界；gen SPATULA_T）
SPAT_GRASP_Z = 0.94                  # 抓点 z（柄杆上，架顶 0.917 之上可握段，同 d2s）
SPAT_GRASP = (SPAT_XY[0], SPAT_XY[1], SPAT_GRASP_Z)
SPAT_HEAD_DIST = 0.134               # 勺头尖到夹持点距离（勺头方向 = 夹爪局部 +X = tool X 行向量第 1 行）
SPAT_HANDLE_DIST = 0.023             # 柄顶到夹持点距离（反方向）

# —— 表面皿 / 粉末（/World/SurfaceDish (0.5365,0.105,0.80)，粉末在皿内；同 d2s）——
# 粉丘实测 bbox：x 0.5188-0.5542，y 0.0814-0.1288，z 0.8021-0.8141。
# 皿沿（rim）顶 z=0.8066 → 插入 z 必须 > 0.8066（过沿）且 < 0.8141（沉入粉）。
DISH_XY = (0.5365, 0.105)
POWDER_TOP_Z = 0.8141               # 粉丘顶
POWDER_Z = 0.809                    # 插入 z：勺尖 5mm 沉入粉丘（高于皿沿 2.4mm，同 d2s）
POWDER_X = 0.537                    # 粉堆中心 x（实测 bbox x[0.5188,0.5542] 中心 0.5365）
DROP_DOWN = 0.245                   # 第⑦步竖直下降量（同 d2s：1.15→0.905；粉顶 0.8141、
                                    #   皿沿 0.8066 与 d2s 逐字一致，下降量不重算）
Y_SHIFT_NEG = 0.16                  # 第⑧步往 -Y 平移量（同 d2s）：⑧终点 TCP y=0.3608-0.16=0.2008，
                                    #   相对底座 y=0.05 回到 0.15 脱离贴底座失效区，勺尖 y=0.1058
                                    #   仍到粉丘中心 y=0.105；x/z/朝向严格不变

# —— 倒燃烧匙前段（2026-09-01 用户新增「z下降10cm，向+y移动20cm，+x移动5cm」）——
# 挖粉后抬升到 H=1.15（TCP (0.537,0.2008,1.15)，法兰 -90° 药匙水平），三步移向燃烧匙：
#   ⑪ 竖直下降 17cm → TCP z 0.98（2026-09-01 用户追加「z再多下降7cm」→ 0.10→0.17；仍高位未触台面）
#   ⑫ 向 +y 移动 31cm → TCP y 0.2008→0.5108（2026-09-01 追加「再多移动10cm」→0.20→0.30，随后
#      「少移动5cm」→0.30→0.25，再「像+y移动那一步还是改回30cm」→0.25→0.30，最后「倒数第二步
#      往+y移动减少5cm」（⑭ 卡穿模）→ 0.30→0.25，再「倒数第二步+y移动多移动5cm」→ 0.25→0.30，
#      再「倒数第二步+再多移动1cm」→ 0.30→0.31）
#   ⑬ 向 +x 移动 5cm → TCP x 0.537→0.587
#   ⑭ 法兰 -90°→0° 转竖直 与 往 -y 平移 18cm 同步（同一进度 t 同始同终、t>=1 即冻结不强制
#      竖直）→ TCP y 0.5108→0.3308、法兰随动 → 勺尖 (0.587,0.3308,0.846)。
#      2026-09-01 用户追加「加动作根d2s一样，一个动作在药匙旋转竖直的同时往-y移动24cm」→ 0.24，
#      「最后一步多移动3cm」→ 0.27；运行反馈「没水平走+抖+会洒粉」→ 改两阶段，再被用户否定
#      「-y移动和法兰旋转不是同步的」→ 改回同步（见文件头，命令基准 + 收紧 eps + 两级退路）；
#      「⑭ 直接把机械臂卡穿模」→ ⑫ 与 ⑭ 各减 5cm（0.30→0.25 / 0.27→0.22）；「去掉最后的强制
#      竖直，不然最后一步都不是同时开始同时结束的」→ 删完成收敛等待；「最后一步-y移动再减少4cm」
#      → 0.22→0.18。
# ⑪⑫⑬ 锁其余两轴 + 保持世界朝向（复用 d2s LowerPowder/ShiftYPos/ShiftXPos，仅参数换 C3）；
# ⑭ C3 本地 FlangeRollShiftYNeg 同步版（joint7 法兰直接命令 + TCP -y 平移同一进度；
# 带朝向 IK 无解退回只解位置保证水平走；实际法兰收敛才冻结不抖），见 flange_roll_shift_y_neg.py。
SPOON_DOWN = 0.17        # ⑪ z 下降量（17cm，原 10cm + 追加 7cm）
SPOON_Y_SHIFT = 0.31     # ⑫ 向 +y 平移量（31cm，30cm 加 1cm——用户要求再多移动）
SPOON_X_SHIFT = 0.05     # ⑬ 向 +x 平移量（5cm）
SPOON_Y_NEG_LAST = 0.18  # ⑭ 同步往 -y 平移量（18cm，22cm 再减 4cm——用户要求缩短）

# —— C3 专属器材（燃烧匙 / 酒精灯 / 火柴；gen_c3_scene.py 同值）——
# 2026-09-01 用户：火柴点灯 OK 但 S4 燃烧匙入外焰 IK FAIL（把手 0.854-0.899m 手指朝下超界、
# 臂乱甩）。① 灯/帽/火柴从底座 y=0.05 线挪离 +x/+y 到 (0.40,0.18)（gen verify 灯体东缘
# 0.4436 距表面皿西缘 0.5065 留 6.3cm；帽挪灯正北 (0.40,0.30)——原 +X 侧会撞火柴）；
# ② S3/S4 全改 ORIENT_FWD 横夹（d2s 药匙同款，本日志 S1 在 1.006m 已证实可达）。
LAMP_X, LAMP_Y = 0.30, 0.38        # 酒精灯（rot180，帽摘正北 12cm；gen LAMP_X/Y）
                                    #   2026-09-02 用户：灯/帽/火柴向 -x 10cm、+y 20cm（0.40,0.18 → 0.30,0.38）
WICK = (LAMP_X, LAMP_Y, 0.9007)    # 灯芯顶（整灯 bbox max z，同 B2 alcohol_lamp；gen 火焰底 0.900）
SPOON_X, SPOON_Y = 0.596, 0.250    # 燃烧匙碗心（碗口 z=SPOON_TZ，把手竖直立起靠架旁）
SPOON_TZ = 0.8068                  # 燃烧匙原点（碗口平面）z（碗底≈0.7966）

# —— 放回药匙（ReturnSpatula，d2s lift_first 同款）——
# ⑭ 同步倒粉结束：TCP (0.587,0.3308,0.98)、法兰随动（≈竖直）、勺尖 (0.587,0.3308,0.846)
# < 架顶 0.917 → 水平回程会拖穿架 → 必须先原位竖直提到安全高位（勺头≥1.016）再高位水平
# 移回架孔正上方，最后竖直下探入孔（用户 2026-09-01「放回前先对准，然后把药匙强制旋转
# 竖直再向下放好」）。架孔 = 药匙位 SPAT_XY=(0.6993,0.3608)，下探深度 = SPAT_GRASP_Z=0.94。
# 水平/竖直段全带 orient=ORIENT_FWD（d2s：不可 FK 采样当前朝向保持——⑭ 残余倾斜会被
# 带回架里，必须显式调直）。

# —— 火柴 + 酒精灯（LightFlamePass，照 C4；S3 全段手指朝下）——
# 火柴 /World/Match (0.54,0.18,0.813) 抬 13mm、杆身 Ø3mm、头朝 +X（头中心 = 原点+0.0894）。
# 随灯挪离底座线 +y（原 0.05 → 0.18）：杆 x[0.5400,0.6378] 距灯体东缘 0.4436 留 9.6cm、距架
# y≥0.2179 留 3.5cm、距表面皿 y≤0.135 留 4.2cm。抓点 (0.58,0.18,0.8145) 3D 0.749m、高位接近
# MATCH_HIGH=0.96 → 0.783m，全在手指朝下可达范围（B3L 实测 0.841m 才 FAIL）。
# **2026-09-01 误改 ORIENT_FWD 后火柴没夹起来**——火柴水平杆、抓点贴台面 0.8145，手指 +X
# 与杆轴平行，两指从杆两侧夹不到；必须竖直手指下探、两指在杆 ±Y 两侧闭合（见 light_flame_pass）。
# 持握 = 纯平移 offset（同 C4/B2）：夹杆身 x=0.04，头在夹爪 +X 0.0494。
# 点火：夹爪推 -X 到灯上方 IGNITE (0.3506,0.18,0.9157)，火柴头落 WICK 正上方 1.5cm → task
# 检测点火。回程必须先抬到 0.96（>火焰尖 0.936）再直退——C3 灯在火柴 +X 侧、回程沿 +X
# 扫回，火焰已点着，不抬高会横穿火焰柱（C4 灯在 -Y 侧向撤无需抬）。
MATCH_XY = (0.44, 0.38)
MATCH_REST_Z = 0.813            # 火柴原点 z（抬 13mm，gen MATCH_T）
MATCH_GRASP_OFFSET = 0.04       # 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1],
               MATCH_REST_Z + 0.0015)       # (0.58,0.18,0.8145)
GRIP_MATCH = 0.0015             # 合爪开度 = 杆身 Ø3mm / 2
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)   # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)   # 头中心相对夹爪（头 x=0.0894 − 抓点 0.04，z 同）
MATCH_CARRY_Z = 0.96            # 运移高（>火焰尖 0.936，横越灯体/回程绕焰）
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2] + 0.015)   # (0.3506,0.18,0.9157)
                                #   夹爪 = WICK − 头偏 0.0494；头在芯上方 1.5cm 触芯
# 高位接近火柴：手指朝下（默认），不能到 H=1.15（(0.58,0.18,1.15)=0.862m 超手指朝下
# 死区 0.816m，B3L 实测 0.841m 已 FAIL）。0.96（=MATCH_CARRY_Z）→ 0.782m ✓，且 > 火焰尖
# 0.936/灯顶 0.9007，接近火柴时灯芯上方高位即可，无需更高。**S3 全段手指朝下**（同 B2/C4；
# ORIENT_FWD 横夹 2026-09-01 实测火柴没夹起来——手指 +X 与水平火柴杆平行，下探只压到杆上
# 夹不住；火柴必须竖直手指下探、两指在杆两侧闭合）。
MATCH_HIGH = MATCH_CARRY_Z     # 高位接近 = 运移高（0.96，手指朝下可达）

# —— 燃烧匙入外焰（SpoonToFlamePass，照 C4；全程 ORIENT_FWD 横夹）——
# 燃烧匙碗贴台 (0.596,0.250,0.8068)（碗口 z=0.8068、碗底≈0.7966），把手竖直立起靠试管架旁。
# pxr 实测把手中心线：z=0.90→x=0.632、z=1.00→x=0.658（斜度 dx/dz≈0.26），Ø3mm 杆。
# 横夹把手 z=0.93 处中心 (0.640,0.250,0.93)（高于架顶 0.917，下探/横夹安全）。
# **全程 ORIENT_FWD 横夹**（单一朝向，无切换 → 无手腕旋转）：抓/提/移灯/入焰/回勺全同朝向。
#   2026-09-02 用户报「夹燃烧匙后移动时机械臂旋转、燃烧匙跟着转、粉末会倒」——根因=旧两段
#   朝向（远端横夹 ↔ 灯区手指朝下）在 ⑤→⑥/⑨→⑩ 切换 90° 致手腕旋转。实测手指朝下（竖直夹）
#   在把手 z≥0.90（0.829m）已 IK FAIL、勺位上方提出高位也 FAIL（死区 ~0.82m）→ 竖直夹不可行；
#   ORIENT_FWD 在抓点 0.803-0.881m + 全灯区（灯移 (0.30,0.38) 后 0.677m）全程可达。持握纯平移
#   offset（夹爪朝向不影响碗位），改全程 ORIENT_FWD 消除旋转。
# 碗口放外焰中心 BOWL_AT_FLAME (0.30,0.38,0.912)（外焰锥 0.900-0.936 内、碗底 0.902 入焰）。
# 提出/运移高 SPOON_LIFT_Z=1.08：碗 z=1.08−0.1232=0.957 > 架顶 0.917 清障。
# 2026-09-02 用户「夹太靠下」→ 抓点 z0.90→0.93（杆中心线 x 0.632→0.640）；SPOON_HELD_OFFSET
#   派生为 (-0.044,0,-0.1232)，FLAME_HOLD_TCP 派生为 (0.344,0.38,1.0352)。
SPOON_GRASP = (0.640, 0.250, 0.93)      # 杆身横夹点（z=0.93 杆中心，ORIENT_FWD 两指夹杆）
                                        #   2026-09-02 用户「夹太靠下」上调 3cm（0.90→0.93，x 随杆斜度 0.26 增到 0.640）
GRIP_SPOON = 0.0015                     # 合爪开度 = 杆身 Ø3mm / 2
SPOON_HELD_OFFSET = (SPOON_X - SPOON_GRASP[0], 0.0,
                     SPOON_TZ - SPOON_GRASP[2])    # 勺原点相对夹爪 = (-0.044,0,-0.1232)
SPOON_REST = (SPOON_X, SPOON_Y, SPOON_TZ)         # 勺原点台面静止位
BOWL_AT_FLAME = (LAMP_X, LAMP_Y, 0.912)            # 碗口（勺原点）目标 = 外焰中心
FLAME_HOLD_TCP = (BOWL_AT_FLAME[0] - SPOON_HELD_OFFSET[0],
                  BOWL_AT_FLAME[1] - SPOON_HELD_OFFSET[1],
                  BOWL_AT_FLAME[2] - SPOON_HELD_OFFSET[2])   # (0.344,0.38,1.0352)
SPOON_LIFT_Z = 1.08               # 提出/运移高度（碗 z=0.957 > 架顶 0.917）
FLAME_DWELL = 240                 # 入焰静止帧数（4s@60fps）。2026-09-02 用户「静止10s」300→600，
                                  #   再「改为4s」600→240；从 HoldAction 改为 MoveAction ⑦ 的 dwell
                                  #   （freeze 后发固定关节值，真正静止）——原 hold 发「当前关节值」
                                  #   在灯区近奇异 pose 下随重力下垂漂移。
OBSERVE_SHIFT_Y = 0.05            # 离焰后往 +y 平移量（5cm，观察位；用户「离开酒精灯往+y移动5cm」）
OBSERVE_DWELL = 600               # 观察静止帧数（10s@60fps；离开火焰后停在观察位不动，观察燃烧现象）

# —— 酒精灯帽盖灭（CapLampPass，阶段 ⑤ 燃烧放回后盖帽熄火，照 C4/B2）——
# 帽 = /World/AlcoholLamp/cap（灯子 prim），Ø37mm×3.1cm 开口朝下倒扣桌面（同 B2/C4
# alcohol_lamp.usd 资产）。帽静止位 = 灯正北 +y 12cm 桌面 (LAMP_X, LAMP_Y+0.12, 0.8155)
# （gen CAP_DETACH translate(0,-0.12,-0.0762) 随灯 R180Z → 帽世界 y=灯y+0.12；pxr 实测
# 帽中心 z 0.8155、顶 0.831、底 0.800）。盖严实 = 帽中心 灯z+CAP_CENTER_DZ=0.8915
# （帽底 0.8760 低于灯体顶 0.8897 13.7mm，帽 local translate → (0,0,0) = 资产原始帽位）。
# 夹爪 CAP_BURNER 0.900 = 帽中心 + 持握偏移 0.0085（B2 六改：帽要盖严实，不能只搭灯口沿）。
# 全程默认朝向手指朝下（B2 三改：低 z 桌面夹帽 ORIENT_FWD 手指朝前 Lula 无解；纯平移持握，
# 帽竖直开口朝下不旋转）。火焰熄灭时机照 B2 十一改：帽下降罩过火焰顶才熄，不移动时早灭。
CAP_CENTER_DZ = 0.0915            # 帽中心到灯底座 z 偏移（同 B2/C4 alcohol_lamp.usd 资产）
CAP_REST = (LAMP_X, LAMP_Y + 0.12, 0.8155)   # 帽静止位世界中心（灯正北 +y 12cm 桌面）
CAP_GRASP = (LAMP_X, LAMP_Y + 0.12, 0.824)   # 夹帽点（帽顶 0.831 下 7mm，同 CAP_REST 水平）
GRIP_CAP = 0.0185                 # 合爪开度 = 帽 Ø37mm / 2
CAP_CLOSED_THRESHOLD = 0.022      # 帽 attach 阈值（合到 0.0185 < 0.022）
CAP_HELD_OFFSET = (0.0, 0.0, -0.0085)   # 纯平移持握：帽中心 = 夹爪 + offset（夹点 0.824 − 帽中心 0.8155）
CAP_HIGH = 1.00                   # 高位（取帽/运帽，高于火焰顶 0.936 清障）
CAP_BURNER = (LAMP_X, LAMP_Y, 0.900)   # 盖灯口夹爪（帽中心 0.8915、帽底 0.8760 < 灯体顶
                                       #   0.8897 盖严实；同 B2/C4 CAP_BURNER 0.900）
CAP_COVER_NEAR = 0.010            # 盖到位判定：夹爪距 CAP_BURNER < 1cm（B2 四改：4cm 太松，
                                  #   下降途中离灯口 3cm 就判盖到位，视频断在"没盖完"）
CAP_EXTINGUISH_XY = 0.06          # 下落即熄火 xy 门控（夹爪 xy 距 CAP_BURNER < 6cm 才熄）
CAP_EXTINGUISH_Z = 0.963          # 下落即熄火 z 门控（帽底 = 夹爪−0.024 刚罩过火焰顶 0.936
                                  #   才灭：0.936+0.003+0.024；火焰保持到帽盖住顶部，帽继续降不穿火）

# —— 固体燃烧现象（阶段 ④ 碗入外焰 dwell 期间，用户 09-02「火焰要动起来模仿C4」+「燃烧后
#    留一点黑色残渣」+「倒完粉末满的、燃烧完几乎空」+「不可燃轻微变黑碳化」）——
# config combustion 双现象：combustible=碗内粉末点燃火焰（焰色=flame_color 输入）烧尽留黑色
# 残渣（粉末满→几乎空）；non_combustible=轻微碳化变黑（无火焰，粉末体积基本不变）。
# 火焰（酒精灯 grp + 样品火焰 grp）每帧 flicker（scale 高/宽 + rotate 侧摆，pivot=火焰底，
# task._step_flame_anim/_apply_flame_flicker 驱动，仿 C4）。
SPOON_FLAME_NEAR = 0.025          # 碗在火焰判定近窗：夹爪距 FLAME_HOLD_TCP < 2.5cm（dwell）
IGNITION_DELAY = 40               # 点火延迟帧数（粉末受热升温，~0.67s@60fps）
BURN_FRAMES = 180                 # 烧尽帧数（~3s，在 FLAME_DWELL 4s 内烧完）
BURN_OUT_AT = 0.002               # 粉末高度低于此算烧尽（隐藏样品火焰）
POWDER_FULL_H = 0.006             # 满粉末高（gen PowderInBowl height，倒粉落定）
POWDER_FULL_R = 0.012             # 满粉末半径（gen PowderInBowl radius）
POWDER_RESIDUE_H = 0.002          # 残渣高（烧尽后几乎空）
POWDER_RESIDUE_R = 0.002          # 残渣半径（烧尽后几乎空）
POWDER_WHITE = (0.93, 0.93, 0.94)   # 白色粉末（gen PowderInBowl diffuse，倒粉落定色）
POWDER_ASH = (0.10, 0.10, 0.11)   # 烧尽黑色残渣（炭化，几乎空）
POWDER_CARBON = (0.45, 0.42, 0.40) # 不可燃轻微碳化（深灰，非纯黑）
CARBONIZE_DELAY = 60              # 不可燃碳化延迟帧数（~1s@60fps 受热）
SAMPLE_FLAME_R = 0.005            # 样品燃烧火焰底球半径（照 C4 SPOON_FLAME_R）
SAMPLE_FLAME_APEX_DZ = 0.028      # 样品火焰高（粉末顶上方 ~28mm，照 C4 SPOON_FLAME_APEX_DZ）

# 效果 prim 路径（scene 内建，task 动画驱动）
EFFECT_LAMP_FLAME_GRPS = ("/World/flame_outer_grp", "/World/flame_inner_grp")  # 酒精灯火焰组
                                  #   （pivot=火焰底，task 每帧 flicker；gen grp 包装）
EFFECT_SAMPLE_FLAME = "/World/SampleFlame_grp"          # 样品燃烧火焰组（pivot=火焰底=粉末顶，
                                  #   task 写 translate 跟随粉末顶 + flicker）
EFFECT_SAMPLE_FLAME_CONE = "/World/SampleFlame_grp/SampleFlame"
EFFECT_SAMPLE_FLAME_SPHERE = "/World/SampleFlame_grp/SampleFlame_sphere"
