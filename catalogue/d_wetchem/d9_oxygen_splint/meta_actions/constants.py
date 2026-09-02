"""D9 氧气检验（带火星木条复燃）元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-09-01 pxr 读 d9_oxygen_splint.usd 世界包围盒（gen_d9_oxygen_splint_scene.py
verify 输出）。

动作链（用户逐字 2026-09-01）：「摘开酒精灯帽 → 拿火柴点燃酒精灯 → 拿木条点燃 → 快速摆动
机械臂让它熄灭（甩灭明火留余烬）→ 火星现象 → 悬停氧气试管口上方（不伸进去）→ 复燃 → 取出归位
→ 盖灯帽（熄灯）」。
8 元动作（一个 v11 步骤 = 一个元动作）：
  ① CapOffPass     摘灯帽：帽从灯口 → 桌面 CAP_REST
  ② IgniteLamp     火柴点燃酒精灯（照 B2 LightFlamePass 逐字）
  ③ PickSplint     夹木条（手指朝下默认朝向横夹，同 B2 火柴持握）
  ④ LightSplint    木条端伸入灯焰点燃（task：端近灯焰 → SplintFlame 显）
  ⑤ BlowOutSplint  快速摆动熄火（shake 基元，task：明火灭 → 余烬火星点显）
  ⑥ HoverSplint    火星端悬停氧气试管口上方（不伸入，task：复燃/不复燃）
  ⑦ ReturnSplint   木条取出归位
  ⑧ CapOnPass      盖灯帽：帽从桌面 → 灯口（task：帽回灯上 + 酒精灯火焰熄）

朝向：全程默认朝向（手指朝下）——木条/火柴均水平横躺（轴 +X），手指朝下竖直夹杆身
（B2 火柴同款，已验证）。灯帽也默认朝向手指朝下（B2 盖帽同款）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.20            # 安全高位（清试管架顶板 0.917 / 灯帽顶 0.907 / 木条端效果 0.838）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_CAP = 0.0185        # 帽 Ø37mm / 2（同 B2）
GRIP_MATCH = 0.0015      # 火柴杆 Ø3mm / 2（同 B2）
GRIP_SPLINT = 0.003      # 木条 Ø6mm / 2
CAP_CLOSED_THRESHOLD = 0.022   # 帽 attach 阈值（同 B2）

# —— 朝向 ——
# 木条/火柴/灯帽全程默认朝向（手指朝下），不显式传 orient（engine 默认 = 手指朝下）。
# ORIENT_FWD 保留备用（手指朝前 +X），本实验不用于木条持握（木条轴 +X 手指朝下才横夹得住）。

# —— 酒精灯 / 灯帽（/World/AlcoholLamp R180 @ (0.5286,0.0029)，帽 = 灯子 prim，起始在灯上）——
LAMP_XY = (0.5286, 0.0029)
LAMP_REST_Z = 0.8002            # 灯底座中心 z（asset 局部 min z=-0.0002）
WICK = (0.5286, 0.0029, 0.9005) # 灯芯顶（点火触发点）
CAP_CENTER_DZ = 0.0915          # 帽中心到灯底座 z 偏移（帽 translate z + 灯 z + 0.0915 = 帽中心）
CAP_ON_GRASP = (0.5286, 0.0029, 0.900)    # 帽在灯上时的夹帽点（帽顶 0.9072 下 7mm）
CAP_HELD_OFFSET = (0.0, 0.0, -0.0083)     # 纯平移持握：帽中心 = 夹爪 + offset
CAP_REST = (0.42, -0.01, 0.8155)          # 帽静止位（桌面）中心（同 B2，-X 侧避石棉网）
CAP_REST_GRASP = (0.42, -0.01, 0.824)     # 帽静止位夹帽点（帽顶 0.8312 下 7mm）
CAP_HIGH = 1.00                           # 摘/运帽高位（高于灯帽顶 0.907）

# —— 火柴（/World/Match (0.40,-0.06,0.813)，头 +X 朝灯芯，抬高 12mm；同 B2）——
MATCH_XY = (0.40, -0.06)
MATCH_REST_Z = 0.813
MATCH_GRASP_OFFSET = 0.04        # 抓杆身 x=0.04
MATCH_GRASP = (MATCH_XY[0] + MATCH_GRASP_OFFSET, MATCH_XY[1], MATCH_REST_Z + 0.0015)
MATCH_HELD_OFFSET = (-MATCH_GRASP_OFFSET, 0.0, -0.0015)  # 火柴原点相对夹爪（纯平移持握）
MATCH_TIP_OFFSET = (0.0494, 0.0, 0.0)    # 头中心相对夹爪
MATCH_LIFT_Z = 0.90              # 低位运移高度
IGNITE = (WICK[0] - MATCH_TIP_OFFSET[0], WICK[1], WICK[2])   # (0.4792,0.0029,0.9005)
MATCH_HIGH = 1.35                # 高位接近（同 B2）

# —— 木条（/World/WoodSplint (0.27,0.25,0.813)，Ø6mm×150mm，握持端原点、点燃端 +X）——
# 木条轴沿 +X，杆中心 z=0（local），底 0.810 抬高 10mm。抓点取 local x=0.04（杆身，离握持端 4cm，
# 同火柴抓杆身 x=0.04），点燃端（local x=0.15）相对抓点 +X 0.11。
# x=0.27（原 0.30）——放回竖直下降有 +X 抖动致点燃端穿试管架（架 min x=0.4573），故整体 -X 退 30mm，
# 点燃端 0.45→0.42 留 ~37mm 净空。
SPLINT_XY = (0.27, 0.25)
SPLINT_REST_Z = 0.813
SPLINT_GRASP_OFFSET = 0.04
SPLINT_GRASP = (SPLINT_XY[0] + SPLINT_GRASP_OFFSET, SPLINT_XY[1], SPLINT_REST_Z)   # (0.31,0.25,0.813)
SPLINT_HELD_OFFSET = (-SPLINT_GRASP_OFFSET, 0.0, 0.0)   # 木条原点相对夹爪（纯平移，杆中心 z=0）
SPLINT_TIP_OFFSET = (0.11, 0.0, 0.0)                    # 点燃端相对夹爪（0.15-0.04）
SPLINT_TIP = (SPLINT_XY[0] + 0.15, SPLINT_XY[1], SPLINT_REST_Z)   # (0.42,0.25,0.813) 木条端世界坐标（静止）
SPLINT_HIGH = 1.20               # 木条安全高位（同 H）
SPLINT_LIFT_Z = 0.90             # 低位运移（同火柴，压灯体顶之下）

# —— 木条端余烬/炭黑效果 prim（与 gen add_splint_effects 一致；task 每帧钉到木条端）——
EMBER_N = 10                     # 余烬火星点数量
EMBER_PREFIX = "/World/SplintEmber_"   # 余烬火星点 prim 前缀（SplintEmber_0..9）
SPLINT_CHAR = "/World/SplintChar"      # 炭黑区圆柱 prim（包木条端 30mm）

# —— 灯焰（/World/flame_outer 等，点火后可视；木条端点燃 = 端入灯焰）——
FLAME_CENTER = (0.5286, 0.0029, 0.920)   # 灯焰中心（木条端目标，灯焰底 0.891 顶 0.936 中段）
LIGHT_GRIP = (FLAME_CENTER[0] - SPLINT_TIP_OFFSET[0], FLAME_CENTER[1], FLAME_CENTER[2])
# = (0.4186, 0.0029, 0.920)：夹爪在灯体 min x 0.485 之外，仅木条端伸进灯焰。

# —— 摆动熄火（shake 中心，灯/管之间空档）——
SHAKE_GRIP = (0.35, 0.20, 1.05)  # shake 中心（夹爪），木条端 (0.46,0.20,1.05)
SHAKE_AMPLITUDE = 0.03           # 摆动振幅 ±3cm（快速甩灭）
SHAKE_CYCLES = 4                 # 来回次数
SHAKE_PERIOD = 45                # 每来回帧数（~0.75s，快速）

# —— 氧气试管（/World/OxygenTube (0.519,0.389) 竖立插架孔，底 0.806 口 0.959，氧气预收集）——
OXY_TUBE_XY = (0.519, 0.389)
OXY_TUBE_MOUTH_Z = 0.959         # 管口世界 z
HOVER_TIP_Z = OXY_TUBE_MOUTH_Z + 0.015   # 木条端悬停 z = 0.974（管口上方 15mm，不伸入）
HOVER_GRIP = (OXY_TUBE_XY[0] - SPLINT_TIP_OFFSET[0], OXY_TUBE_XY[1], HOVER_TIP_Z)
# = (0.409, 0.389, 0.974)：夹爪在管 -X 侧，木条端悬管口上方（横越架顶，无立柱穿模）
HOVER_DWELL = 60                 # 悬停观察帧（复燃判定 + 展示结果）
