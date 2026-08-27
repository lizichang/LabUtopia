"""D7 气体鉴定元动作共享常量：坐标 / 抓取 / 高度 / 朝向。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-27 pxr 读 d7_gas_identification.usd 世界包围盒（gen_d7_scene.py verify）。
坐标系与 d2s/d3l/flametest/e2/d6 一致（复用 IkMotionEngine / BaseMetaAction，Lula IK 驱动）。

2026-08-27 用户方案：检测试剂统一表现为液体（无浑浊/变色变体），仅初始颜色由输入 liquid_color
决定（6 色变体，默认 colorless）；通气+识别合并为单个 HoldDetect。
**2026-08-27 三改：产气试管（带导气管橡皮塞）移出试管架，改用铁架台 + 试管夹固定**（用户：
抓取时从管子上方向下穿模 → 移出架外夹持，导气管桥/末端不再横跨检验试管抓点上方）。
**2026-08-27 四改：下浸路线改「从下方接近」**（用户：从导气管上方直下会穿过导气管桥）→
产气试管抬高 10cm（底 0.90，末端 1.049），下浸先降到偏移位下方、平移到末端正下方、再上移套入。
**2026-08-27 五改：末端下探段加长 + 上移缩短 + 删装饰样品瓶**（用户：「上移距离太长超出导气管
长度；试管不够透明；样品瓶多余」）→ 导气管下探 50mm→75mm（末端 1.049→1.024，下浸后管口 1.092
仍低于桥 1.099）、DIP_APPROACH_CLEAR 0.04→0.01（上移 108mm→78mm）、样品瓶删除。

动作 3 元动作（一个 v11 步骤 = 一个元动作，橡皮塞预装塞紧不夹取/拔塞）：
  ① DipGasTube    取检验试管 → 移到下浸点下放使导气管末端浸入液面下 15mm
  ② HoldDetect    保持 2.5s 通气（task 驱动气泡上升动画）
  ③ ReturnTube    检验试管归位

朝向：全程手指朝前 ORIENT_FWD（d2s 夹药匙同款）——检验试管侧面横夹（架顶有孔、不能正上方
下探），+X 侧接近/退开（避 +X 导气管桥）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.20            # 安全高位（清试管架顶板 0.917 / 管口 0.959 / 导气管桥 1.099）
SETTLE = 12         # 到点 settle 帧数
HOLD_DETECT_DWELL = 150   # 通气观察停留帧（2.5s @60Hz，task 期间气泡连续上升）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_TUBE = 0.0096       # 试管 Ø19.2mm/2（与 d3l/d6 一致）
GRIP_STOPPER = 0.0096    # 橡皮塞塞体中部 Ø~18.5mm/2（同 GRIP_TUBE，Ø20.5 顶/Ø16.6 底）

# —— 朝向 ——
# ORIENT_FWD：手指 tool+Z 朝 +X（朝向 camera1），侧面横夹用（与 d2s 夹药匙/洗瓶同款）。
# 手指水平、两指沿 ±Y 张开夹管身/塞体 ±Y 面 → 手指不朝下戳进试管架顶板/导气管竖段。
# 引擎 [x,y,z,w] 序（scipy），d2s pxr 已验证。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)
# 侧面横夹接近偏移：夹爪先到目标 x 偏 -X 侧（-X 壁外侧），再水平移 +X 入中心夹持
# （避竖直下探穿模，同 d6 TUBE_APPROACH_DX 模式）。
TUBE_APPROACH_DX = 0.05
STOPPER_APPROACH_DX = 0.05

# —— 产气试管（/World/GasTube (0.40,-0.08)，铁架台试管夹固定，底 0.90，口 1.053）——
GAS_TUBE_XY = (0.40, -0.08)
TUBE_BOTTOM_Z = 0.806                   # 检验试管底面（架孔底，产气试管已抬高用 GAS_TUBE_BOTTOM_Z）
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.153   # 0.9593（检验试管口）

# —— 检验试管（/World/TestTube (0.300,0.160)，右列第 3 排，底 0.806，口 0.9593）——
TEST_TUBE_XY = (0.300, 0.160)
TUBE_GRASP_Z = TUBE_MOUTH_Z - 0.014    # 0.9453（夹管口下 14mm，与 d3l/d6 一致）
TUBE_GRASP = (TEST_TUBE_XY[0], TEST_TUBE_XY[1], TUBE_GRASP_Z)
TUBE_HELD_OFFSET = (0.0, 0.0, -(TUBE_GRASP_Z - TUBE_BOTTOM_Z))  # 底 = tool_center - 0.139
TUBE_PRE_Z = 1.02                      # 预抓/预放中转高度（管口 0.959 上方）

# —— 下浸点（/World 导气管末端 (0.44,0.079) = 产气试管 + (ΔX40,ΔY159)，检验试管移此下浸）——
DIP_XY = (0.44, 0.079)
LIQUID_H = 0.100                       # 检测液柱高（TubeSolution 圆柱 h100mm）
IMMERSION = 0.015                      # 导气管末端沉入液面下 15mm（用户「1-2cm」按 15mm 实施）
FREE_END_Z = 1.024                     # 塞紧后导气管末端世界 z（塞底 1.044 - 局部 0.020，下探 75mm）
DIP_SURFACE_Z = FREE_END_Z + IMMERSION  # 1.039（下浸后检测液面 z）
# 下浸抓点 z：管底 = 抓点 - 0.139，液面 = 管底 + 0.100 → 液面 = 抓点 - 0.039
# → 抓点 = DIP_SURFACE_Z + 0.039 = 1.078（管口 1.092 仍低于导气管桥 1.099）
DIP_GRASP_Z = DIP_SURFACE_Z + (TUBE_GRASP_Z - TUBE_BOTTOM_Z) - LIQUID_H   # 1.078
DIP_GRASP = (DIP_XY[0], DIP_XY[1], DIP_GRASP_Z)

# —— 下浸「从下方接近」路线（2026-08-27 用户：从导气管上方直下会穿过导气管桥 → 先降到下方、
#    再平移到末端正下方、再上移套入；产气试管抬高后末端 1.024 下方有足够空间容检验试管底部
#    不碰台面 0.80）——
DIP_APPROACH_CLEAR = 0.010            # 下方接近时管口低于导气管末端的距离（1cm，缩短上移行程）
DIP_BELOW_Z = FREE_END_Z - (TUBE_MOUTH_Z - TUBE_GRASP_Z) - DIP_APPROACH_CLEAR  # 1.000 下方抓点 z
DIP_OFFSET = 0.08                     # 偏移位 +Y 偏移（避开导气管桥，桥 y≤0.079）
DIP_APPROACH_XY = (DIP_XY[0], DIP_XY[1] + DIP_OFFSET)  # (0.44,0.159) 下方接近/退开偏移位

# —— 带导气管橡皮塞（/World/Stopper (0.58,0.03) 桌面，塞底 0.80、塞顶 0.824；
#    塞顶中心伸导气管短竖段 0.824..0.855 + 桥 +X+Y + 末端 (0.623,0.192,0.805)）。
#    2026-08-27 用户：首次夹取碰撞 → 橡皮塞远离试管架（右缘 0.3227）移到 (0.58,0.03)，
#    -X 侧接近完全清空（距架 0.21m），不再与产气试管/架框挤在一起。——
STOPPER_INITIAL_XY = (0.58, 0.03)
STOPPER_BOTTOM_Z = 0.80                # 桌面塞底 z
STOPPER_GRASP_Z = 0.816                # 桌面抓点（塞体中部，塞底 +0.016）
STOPPER_INITIAL_GRASP = (STOPPER_INITIAL_XY[0], STOPPER_INITIAL_XY[1], STOPPER_GRASP_Z)
STOPPER_PLUG_BOTTOM_Z = 0.950          # 塞紧后塞底 z（沉入管口 0.9593 下 ~9mm，胶压视觉简化）
STOPPER_PLUG_GRASP = (GAS_TUBE_XY[0], GAS_TUBE_XY[1], STOPPER_PLUG_BOTTOM_Z + 0.016)  # 0.966
STOPPER_HELD_OFFSET = (0.0, 0.0, -(STOPPER_GRASP_Z - STOPPER_BOTTOM_Z))  # 塞底 = tool_center - 0.016
STOPPER_PRE_Z = 1.05                   # 预抓/预放中转高度（塞顶 0.824 / 塞紧后管口 0.959 上方）
