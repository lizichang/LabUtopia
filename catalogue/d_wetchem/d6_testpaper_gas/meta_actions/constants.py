"""D6 试纸气体检测（通用）元动作共享常量：坐标 / 抓取 / 高度 / 朝向。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-26 pxr 读 d6_testpaper_gas.usd 世界包围盒（gen_d6_scene.py verify）。
坐标系与 d2s/d3l/flametest/e2 一致（复用 IkMotionEngine / BaseMetaAction，Lula IK 驱动）。

2026-08-26 用户重新设计：专用试纸夹预夹好试纸（机械臂不碰试纸），动作减为 4 元动作。
2026-08-26 试管架离底座太近（试管 0.154m）碰撞卡住 → 操作区整体前移 -Y 远离底座 y=0.57：
试管架 (0.28,0.30)→(0.28,0.16)，试管 0.416→0.276（距底座 0.294m）、滴管 0.181→0.041（0.529m）。
试纸夹 2026-08-27 rot180°（试纸 -X 朝试管架）+ 移远 (0.55,0.20) 拉开与架距，湿润端中心 (0.4925,0.20)。
  ① WetPaper          取蒸馏水滴管 → 滴 1-2 滴润湿试纸 → 归位
  ② MoveTubeUnderPaper 取反应试管 → 移到试纸湿润端正下方（管口距试纸 2.5cm）
  ③ HoldDetect         保持 2.5s 观察试纸变色
  ④ ReturnTube         试管归位
滴管竖直夹取（手指朝下，默认朝向）；试管从侧面横夹（手指朝前 ORIENT_FWD，d2s 夹药匙/洗瓶
同款）——2026-08-26 用户：竖直下探抓试管会穿模，改侧面横夹后手指水平不戳进试管架顶板。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.20            # 安全高位（试纸 z=0.99 + 滴管尖 1.01 + 抓点 1.14 需留裕量，H 提到 1.20）
SETTLE = 12         # 到点 settle 帧数
HOLD_DETECT_DWELL = 150   # 观察变色停留帧（2.5s @60Hz）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_DROPPER = 0.0055    # 滴管胶头 Ø11mm → 半宽 5.5mm（开度 = 2×值 = 11mm）
GRIP_SQUEEZE = 0.002     # 挤胶头出水（比 GRIP_DROPPER 更紧，压出 1-2 滴）
GRIP_TUBE = 0.0096       # 试管 Ø19.2mm/2（与 d3l 一致）

# —— 朝向 ——
# ORIENT_FWD：手指 tool+Z 朝 +X（朝向 camera1），试管侧面横夹用（与 d2s 夹药匙/洗瓶同款）。
# 手指水平、两指沿 ±Y 张开夹管身 ±Y 面 → 手指不朝下戳进试管架顶板（竖直夹取穿模根因）。
# 引擎 [x,y,z,w] 序（scipy），d2s pxr 已验证。
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)
# 侧面横夹接近偏移：夹爪先到管口 x 偏 -X 侧（管身 -X 壁外侧），再水平移 +X 入管身中心夹持
# （避竖直下探穿模，同 pick_wash_bottle 的 WASH_APPROACH_X 模式）。
TUBE_APPROACH_DX = 0.05

# —— 滴管（/World/Dropper (0.300,0.041)，底 0.806，胶头 0.936..0.956；预吸好蒸馏水）——
DROPPER_XY = (0.300, 0.041)
DROPPER_BOTTOM_Z = 0.806
TIP_OFFSET = 0.13                      # 滴管尖距抓点 0.13m（抓胶头 0.936 = 底 0.806 + 0.13）
DROPPER_GRASP_Z = DROPPER_BOTTOM_Z + TIP_OFFSET   # 0.936
DROPPER_GRASP = (DROPPER_XY[0], DROPPER_XY[1], DROPPER_GRASP_Z)
DROPPER_HELD_OFFSET = (0.0, 0.0, -TIP_OFFSET)     # 滴管 translate(=底) = tool_center - 0.13
DROPPER_PRE_Z = 1.02                   # 预抓/预放中转高度（胶头 0.956 上方）

# —— 反应试管（/World/TestTube (0.260,0.276)，底 0.806，口 0.9593；含预置反应混合物）——
TUBE_XY = (0.260, 0.276)
TUBE_BOTTOM_Z = 0.806
TUBE_MOUTH_Z = TUBE_BOTTOM_Z + 0.153   # 0.9593
TUBE_GRASP_Z = TUBE_MOUTH_Z - 0.014    # 0.9453（夹管口下 14mm，与 d3l 一致）
TUBE_GRASP = (TUBE_XY[0], TUBE_XY[1], TUBE_GRASP_Z)
TUBE_HELD_OFFSET = (0.0, 0.0, -(TUBE_GRASP_Z - TUBE_BOTTOM_Z))  # 底 = tool_center - 0.139
TUBE_PRE_Z = 1.02                      # 预抓/预放中转高度（管口 0.959 上方）

# —— 试纸（夹缝 z≈0.99，湿润端=最后 15mm；2026-08-27 试纸夹 rot180°，试纸沿 -X 悬挑朝试管架）——
PAPER_WET_XY = (0.4925, 0.20)          # 湿润端中心（world，试纸沿 -X 悬挑，朝向试管架）
PAPER_Z = 0.99
PAPER_GAP = 0.025                      # 气体间隙：试管口距试纸 2.5cm（不接触）

# —— 试管检测「下潜点」：先降到试纸水平面之下的过渡点（架与试纸之间空隙），再水平滑入湿润端
# 正下方（避从试纸正上方直降穿模）。架 X 0.2373..0.3227、试纸 X 0.485..0.555，空隙中心 ≈0.40。
TUBE_DESCEND_XY = (0.40, 0.20)

# —— 润湿滴加 / 试管检测位（TCP）——
WET_TCP_Z = PAPER_Z + 0.02 + TIP_OFFSET        # 1.14（滴管尖 = 1.14-0.13 = 1.01，试纸上方 2cm）
WET_TCP = (PAPER_WET_XY[0], PAPER_WET_XY[1], WET_TCP_Z)
# 试管检测位：目标在湿润端中心 +X 侧多推 0.0075（0.50）——MoveAction 冻结裕量 1cm 会让试管
# 停在湿润端 -X 边（0.485）显得「没完全推到正下方」（2026-08-27 用户），目标略偏 +X 补偿后
# 试管落到湿润端中心附近（0.49..0.50），真正在试纸正下方。
TUBE_UNDER_PAPER_X = PAPER_WET_XY[0] + 0.0075   # 0.50
TUBE_UNDER_PAPER_TCP = (TUBE_UNDER_PAPER_X, PAPER_WET_XY[1],
                        (PAPER_Z - PAPER_GAP) - 0.014)   # 0.951（管口 0.965 = 试纸下 2.5cm）
