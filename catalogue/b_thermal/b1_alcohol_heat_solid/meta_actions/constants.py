"""B1 酒精灯加热（固体样品）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-27 pxr 读 b1_alcohol_heat_solid.usd 世界包围盒
（gen_b1_scene.py verify 输出）：
  酒精灯 AlcoholLamp (0.50,0.0029)：玻璃罩 z[0.80,0.8897]、灯芯顶 wick_top 0.9007；
    灯帽 cap 盖在灯口（Ø37.2×31mm 钟罩，世界中心 (0.50,0.0029,0.8917)）
    （2026-08-27 用户改灯到 x=0.659 对齐试管架→「位置全部调整回去」→ 回退 x=0.50）
  火柴 Match (0.40,-0.06) 原点 z=0.813（抬高 12mm），头=asset +X 端朝灯芯（同 B2）
  药匙/皿/粉/试管/试管架 = D2S 逐字（挖粉坐标才不跑偏），挖粉复用 d2s 元动作默认值

挖粉常量直接复用 d2s（SPAT_XY/H/SETTLE/GRIP_OPEN/ORIENT_FWD + 勺/皿/粉/试管坐标）；
本文件只放 B1 专属（酒精灯 / 灯帽 / 火柴 + 点火）。
"""
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.constants import (
    H, SETTLE, GRIP_OPEN, ORIENT_FWD,
    SPAT_XY, SPAT_GRASP, GRIP_SPATULA, SPAT_HEAD_DIST,
    TUBE_XY, TUBE_MOUTH_Z, DISH_XY, POWDER_X, POWDER_TOP_Z,
    GRIP_TUBE, TUBE_GRASP_TCP,
)

# —— 酒精灯（/World/AlcoholLamp (0.50,0.0029)，玻璃罩顶 0.8897、灯芯顶 0.9007）——
# 2026-08-27 用户先「把酒精灯往前移一移让x坐标对齐试管架」（x→0.659），随后「调整完位置后
# 现在完全错了，位置全部调整回去」→ 回退 x=0.50。灯帽/火柴抓点等 LAMP_XY 派生量自动更新。
LAMP_XY = (0.50, 0.0029)
WICK = (LAMP_XY[0], LAMP_XY[1], 0.9005)   # 灯芯顶（pxr wick_top 0.9007 差 0.2mm，同 B2 取 0.9005）

# —— 灯帽（/World/AlcoholLamp/cap，盖在灯口；Ø37.2mm×31mm 钟罩）——
# 帽世界中心 (0.50,0.0029,0.8917)。lamp 无旋转、帽中心相对灯原点 +Z 0.09152 恒定
# （帽 ops = translate+rotateXYZ(90)+scale(0.01)，mesh 中心 local (0,9.152,0) → 世界 +Z）。
# 抓点 = 帽中心（火柴/沸石同款：TCP = 物体中心），手指夹帽壁（帽壁 y=±0.0186 在空心灯口
# 内、玻璃壁 y=±0.0435 之内 → 无碰撞；TCP=帽中心时手指不低到帽下，不碰灯体）。
# 持握 = 纯平移：帽世界中心 = 夹爪，task._set_cap_center 写帽 local translate =
#   center - CAP_CENTER_REST（只写现有 translate op，姿态不变）。
CAP_CENTER_REST = (0.50, 0.0029, 0.8917)   # 帽静止中心（盖在灯上，local translate=0）
CAP_GRASP = CAP_CENTER_REST                # 抓点 TCP = 帽中心
GRIP_CAP = 0.0185                          # 合爪开度 = 帽 Ø37.2mm / 2（flametest 同款，贴合帽壁）
# CAP_HIGH：2026-08-27 修「拿起/放下灯帽卡几秒」——原 1.10 无解（Lula 各 warm start
#   FAIL，(0.63,-0.06,1.10) IK 解不出 → 到位判定永不冻结 → 卡 2.3s/2s）。降为 0.95：
#   取帽位 (0.50,0.0029,0.95) 与放帽位 (0.52,-0.18,0.95，臂展 0.72m 舒适) 均在臂展内可解，
#   帽底 0.9345 仍高于灯顶 0.8897 4.5cm（横移不碰灯）。
CAP_HIGH = 0.95                            # 取帽高位（帽底 0.9345 > 灯顶 0.8897，横移安全）
# CAP_ASIDE_XY：2026-08-27 用户「放远一点」→ 先 (0.63,-0.06) → (0.62,-0.13)（距灯 18cm），
#   再 08-27 修「拿完帽还有多余的往下的动作穿模」：(0.62,-0.13) 的 +X 拉伸（x 0.50→0.62，
#   距底座 0.79m 近 Franka 0.855m 臂展上限）使关节空间插值经低臂位 → TCP 下沉到 z≈0.814
#   穿灯身（⑥b -Y 段实测全程干净，只 +X 段沉）；横移改到 (0.52,-0.18)（主要 -Y、仅 2cm +X，
#   臂展 0.72m 舒适不沉；灯在 (0.50,0.0029)，放帽位 (0.52,-0.18) 距灯 ~18cm 安全）。
#   落位 z=0.8155 帽底贴台面 0.80。
CAP_ASIDE_XY = (0.52, -0.18)               # 放一边位置（灯正前方更远，帽底贴台面）
CAP_ASIDE_CENTER = (0.52, -0.18, 0.8155)   # 帽放一边静止中心（底 0.80 + 半高 0.0155）
CAP_ASIDE_TCP = CAP_ASIDE_CENTER           # 放帽下探 TCP（= 帽中心，手指夹帽壁落位台面）

# —— 火柴（/World/Match (0.40,-0.06)，头朝灯芯；同 B2）——
# 火柴躺台面 (0.40,-0.06) 头朝灯芯（头=asset +X 端），原点 z=0.813（抬高 12mm 避免夹爪
# collider 扎桌面卡爪）。持握 = 纯平移 offset（火柴全程水平头朝 +X，不随夹爪旋转）。
# 2026-08-27 用户先让移酒精灯（x→0.659）随后回退（位置全部调整回去）→ 火柴一直不动
# （灯在 (0.50,0.0029)，火柴 (0.40,-0.06) 距灯 ~12cm，点火时夹爪从 (0.44,-0.06) 平移到
# IGNITE 携火柴 ~3cm，臂展内可解）。
MATCH_XY = (0.40, -0.06)
MATCH_REST_Z = 0.813            # 火柴原点（杆 -X 端）z
MATCH_GRASP_OFFSET = 0.04       # 抓杆身 x=0.04（杆中部，头留 0.0494 在前伸向灯芯）
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)
GRIP_MATCH = 0.0015             # 合爪开度 = 杆身 Ø3mm / 2
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)        # 头中心相对夹爪（头 x=0.0894 − 抓点 0.04）
MATCH_LIFT_Z = 0.90             # 夹起后低位运移高度（压灯体顶 0.8897 之下）
# 点火：火柴头落在灯芯顶 WICK。夹爪到 IGNITE（灯芯偏 -X 侧 4.94cm，夹爪 0.4506 在灯体
# min x 0.4564 之外），头中心 +0.0494 到 WICK (0.50,0.0029,0.9005)。
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])   # (0.4506,0.0029,0.9005)
MATCH_HIGH = 1.05               # 高位接近

# —— 试管（加热流第一步：拿试管 → 移到酒精灯火焰上方）——
# 试管几何 = d2s 同款（Ø19.2×153mm，管口 0.9593、管底 0.806、架顶 0.917），抓管口下
# 14mm（TUBE_GRASP_TCP，从 d2s 导入）。用户 2026-08-27 逐字：「先夹住试管，水平横着夹住
# （跟夹药匙的方法一样），然后移动到酒精灯火焰上方」→ ORIENT_FWD 水平横夹（手指朝前，
# 同 d2s 夹药匙/夹试管——竖直下探在试管位 (0.659,0.241) IK 不可达，D2S S6 已踩）。
# 持握 = 矩阵持握（药匙同款 _T_HELD）：试管世界位姿 = _T_HELD_TUBE · tool_world，
# 随夹爪 6-DOF 刚性跟随（2026-08-27 用户逐字「爪子抓的东西应该也倾斜了呀」）——爪子转到
# 朝下试管跟着转水平；管内白粉柱随管刚性跟随，不再悬原架。
TUBE_REST_Z = 0.806            # 管底静置 z（架内竖插，D2S 逐字）
TUBE_HIGH = 1.10               # 拿管提出高度（管底 0.961 清架顶 0.917；= 火焰上方 z，横移纯水平）
# 管底持握沿 tool-X 偏移幅值（正数；_T_HELD_TUBE 平移量 = +TUBE_HELD_X，管底吊夹爪下）——
# 由几何导出（TUBE_GRASP_TCP[2] − TUBE_REST_Z = 0.9453 − 0.806 = 0.1393，勿写反：写反会让
# 抓点处试管抬到 1.0846 而非原位 0.806）。
TUBE_HELD_X = TUBE_GRASP_TCP[2] - TUBE_REST_Z
# 管内白粉柱中心距管底（= TUBE_POWDER_REST.z 0.809 − 管底 0.806，task._TUBE_POWDER_OFFSET 同源）。
# 2026-08-27 用户「粉末只舀了一勺不可能那么多」→ 粉末坐管底（中心 0.809，r0.004 h0.006），
# 不再悬管底上方 34mm 的大粉柱。
TUBE_POWDER_OFFSET_Z = 0.003
# 火焰上方定位（加热流第二步，2026-08-27 用户「现在加动作水平往负x方向移动（yz还有朝向都
# 不变）让爪子x坐标对齐火焰的x坐标」→ ⑦ 水平 -X 移到 TUBE_AT_FLAME）：pick 抓管（① 高位
# 接近 → ② 横夹下探 → ③ 停顿 → ④ 合爪 → ⑤ 竖直提出架顶）→ ⑥ 法兰只动 joint7 转 -95°
# （试管竖直→近水平，过水平 5°）→ ⑦ 水平往 -X 移动（y,z 不变、朝向保持法兰转后姿态），
# 爪子 x 对齐火焰 x。火焰 x = 灯芯 x = LAMP_XY[0]（灯在 (0.50,0.0029)，火焰居灯芯顶上方）。
TUBE_AT_FLAME = (LAMP_XY[0], TUBE_XY[1], TUBE_HIGH)   # ⑦ 横移终点：爪子 x=火焰 x=0.50，y=0.241、z=TUBE_HIGH 不变
# 火焰几何（alcohol_lamp.usd flame_outer Cone，2026-08-28 pxr 实测）：灯基偏移 0.0002 → 世界灯
# tz=0.8002；flame_outer 局部 z=0.118 h=0.035 → 世界锥底 0.9007（=灯芯顶）、锥中心 0.9182、锥顶
# 0.9357。⑧ 竖直下降（用户逐字「再加动作，竖直z方向降低，让爪子在z坐标比火焰小2cm（过程中yx
# 还有朝向不变，只有z变）」）→ 爪子 z = 火焰 z − 2cm（火焰 z 取 flame_outer 锥中心）。
FLAME_Z = 0.9182                   # 火焰 z（flame_outer 锥中心，世界坐标）
TUBE_AT_FLAME_Z = FLAME_Z - 0.02   # ⑧ 终点爪子 z：比火焰小 2cm（= 0.8982）
# ⑨ 水平 -Y 移动 11cm（用户 2026-08-28 逐字「然后加动作，只水平-y移动15cm，xz朝向不要变」，
# 后改「最后一步水平移动改为11cm」）
# → 目标 y = TUBE_XY[1] − 0.11 = 0.241 − 0.11 = 0.131（⑧ 已锁 y=0.241 不变，-11cm 即绝对 0.131）；
# 仍在火焰 x=0.50 正下、向灯 y=0.0029 靠近（距灯中心 y 12.8cm，灯身 Ø~8cm，无碰撞）。
TUBE_SHIFT_Y_NEG = 0.11
TUBE_AT_FLAME_2 = (LAMP_XY[0], TUBE_XY[1] - TUBE_SHIFT_Y_NEG, TUBE_AT_FLAME_Z)   # ⑨ 终点 (0.50,0.131,0.8982)
# —— 预热 / 持续加热（加热流第二步，2026-08-28 用户逐字「现在需要加的动作是来回预热，
# 在y的方向上来回移动2cm，来回移动5次速度不要太快，最后持续加热持续8s，最后放回试管」）——
# 预热 = 试管停在 TUBE_AT_FLAME_2 中心做 y 向正弦往复（HeatSweepAction 内部 ShakeAction）：
#   amplitude=±2cm → y 0.131±0.02 = [0.111,0.151]，距灯中心 (0.50,0.0029) y≥10.8cm，
#   灯身 Ø~8cm 无碰撞；cycles=5 个来回（正弦相位 0→10π，首尾都回中心，起止无横移）；
#   period=150 帧/来回（60Hz 下 2.5s，5 来回 ≈ 12.5s，慢速——振幅 2cm 半周期 1.25s ≈ 1.6cm/s）。
PREHEAT_AMPLITUDE = 0.02       # y 向 ±2cm
PREHEAT_CYCLES = 5             # 来回 5 次
PREHEAT_PERIOD = 150           # 一个来回帧数（2.5s/来回，慢速）
HEAT_HOLD_FRAMES = 480         # 持续加热 8s（60fps × 8，预热结束后 hold）
# 法兰转 -95° 后的工具朝向（(x,y,z,w)，推导见 flange_roll_tube.py）：tool+Z 仍 = 世界 +X
# （手指朝前不变，joint7 绕 tool+Z 自转）；tool+X=(0,-0.996,+0.087)、tool+Y=(0,-0.087,-0.996)。
# ⑦ 横移必须显式传此朝向（mv orient=None 会用引擎默认手指朝下 → 试管又转回竖直，违背用户
# 「朝向不变」）。算法脚本验证往返误差 ~0。
FLANGE_HOLD_ORIENT = (0.521334, -0.477714, 0.521334, -0.477714)

# —— 实验结果输入（2026-08-28 用户「加一个输入是表示粉末的颜色参考d2s,d3l」+「根据输入让你有
# 什么现象就有什么现象」）——
# 粉末颜色 + 加热现象名（与 config experiment_result.options 同源，task 校验 cfg 输入用）。
# 粉末颜色照 d2s LIQUID_COLOR_NAMES（白红蓝绿紫）；加热现象 = 持续加热后固体变化三档。
POWDER_COLOR_NAMES = ("white", "red", "blue", "green", "purple")
HEAT_PHENOMENON_NAMES = ("disappear", "blacken", "unchanged")