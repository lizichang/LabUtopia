# -*- coding: utf-8 -*-
"""A2 旋光仪测量元动作共享常量。

所有坐标 = TCP（tool_center / right_gripper）世界坐标，米，Z-up（台面顶 z=0.80）。
几何来源：2026-08-27 pxr 读 gen_a2_scene.py 世界包围盒 + polarimeter.usd 三改
（scripts/fix_polarimeter.py）+ 旋光管资产（scripts/gen_polarimeter_tube.py）：

  Polarimeter     (0.30,0.00)  机身 x 0.115..0.490 / y -0.308..0.305 / z 0.80..1.05，
                               屏幕朝 +y、lid 已掀 120°（铰链 y -0.163，盖尖升到 ~1.27）；
                               顶板前部红色启动键 Ø64（局部 (0,0.18,0.253)）→ 世界顶 1.056
  PolarimeterTube 1dm (0.70,0.30,0.811)  桌面空管横放泡朝上（轴 Y、泡 +y、加液口顶 z 0.830）
  TestTubeRack   (0.82,0.55)   架顶 z=0.917
  TestTube       (0.799,0.43,0.806)  立插架近侧孔，管口 z=0.9593、管底 0.806
  WashBottle     (-0.10,0.60)  rotZ-180（红嘴朝 +X 对试管方向，同 d2s），瓶身 z 0.80..0.97

持握约定：
  试管（/World/TestTube）    旋转跟随 _T_HELD_TUBE（管底吊夹爪下 0.1393m），抓管口下 14mm
  洗瓶（/World/WashBottle）  动态锁 _T_HELD_WASHB = 静止矩阵 · tool^-1，横夹肚子
  旋光管（/World/PolarimeterTube） 纯平移持握（set_object_position），保持横放泡朝上，
                              管中心 = TCP + PTUBE_HELD_OFFSET(0,0,-0.019)

动作横移走廊：旋光管在旋光仪右前方 (0.70,0.30)（抓点 3D 0.46m 脱离 0.33m 低 z 失效区，
倒液点 (0.70,0.3325) 贴仪右缘外侧净空）→ 试管从架到倒液点的走廊只剩 0.1m（原 0.75m
单轴横移 600 帧超时 force-done）；仍经 CORRIDOR_Y=0.40（旋光仪前缘 0.305 之后）净空绕行。
"""

# ---- 基础 ----
H = 1.15                     # 高位横移高度（被持物最低点 > 沿途最高障碍）
SETTLE = 12                  # 到位稳定帧
GRIP_OPEN = 0.04             # 夹爪满开
# 引擎四元数约定 [x,y,z,w]（scipy 序）；R_y(±90°)
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)          # 手指朝 +X（夹药匙/横夹洗瓶/试管）
ORIENT_POUR = (0.0, 0.7071068, 0.0, -0.7071068)        # 手指朝 -X（试管倒置，管口朝下）
# 横移净空走廊（绕开旋光仪，见文件头）
CORRIDOR_Y = 0.40

# ---- 试管（d2s 同款 Ø19.2×153mm，立插架近侧孔）----
TUBE_XY = (0.799, 0.43)
TUBE_MOUTH_Z = 0.9593        # 管口（架顶 0.917 之上）
TUBE_ORIG_Z = 0.806          # 管底
TUBE_GRASP_TCP = (0.799, 0.43, TUBE_MOUTH_Z - 0.014)   # 抓管口下 14mm = 0.9453
GRIP_TUBE = 0.0096           # Ø19.2/2
TUBE_HELD_OFFSET_Z = TUBE_ORIG_Z - (TUBE_MOUTH_Z - 0.014)  # 管底吊夹爪下 0.1393
SHAKE_CENTER_TCP = (0.799, 0.43, 1.09)                 # 提出架顶 17cm 震荡
SHAKE_AMPLITUDE = 0.04
SHAKE_PERIOD = 60
SHAKE_HOLD_FRAMES = 300

# ---- 洗瓶（d2s 同款 6.4×6.4×16.8cm，rotZ-180 红嘴朝 +X）----
WASH_XY = (-0.10, 0.60)      # 底座 (0.30,0.50) 后抓点 3D 0.45m 可达（原 -0.45 处 0.76m 卡死）
WASH_GRASP_Z = 0.88          # 横夹肚子高度（瓶身 z 0.80..0.97，夹中段）
WASH_GRASP = (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z)
GRIP_WASHBOT = 0.030         # 半开度：6cm 开口压 6.4cm 软瓶身每侧 2mm
WASH_LIFT = 1.03             # 提出 15cm
WASH_NOZZLE_OFF = (0.106, 0.0, -0.036)  # 红嘴相对 TCP（d2s 实测：嘴=TCP+偏移）
WASH_TO_TUBE_X = TUBE_XY[0] - WASH_NOZZLE_OFF[0]       # 嘴对管口：TCP x=0.693
WASH_TO_TUBE_Y = TUBE_XY[1]                            # =0.43
WASH_SQUEEZE = 0.020         # 挤水开合（<0.025 出水）
WASH_SQUEEZE_CLOSED = 0.025
WASH_SQUEEZE_DWELL = 150
WASH_GRIP_OPEN = 0.038       # 松开阈值（0.04=满开永不 released）
WATER_START = (TUBE_XY[0], TUBE_XY[1], WASH_LIFT + WASH_NOZZLE_OFF[2])  # 嘴尖 0.994
WATER_END = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z)     # 落管口 0.9593

# ---- 旋光管（1dm，轴 Y、泡 +y、加液口顶 0.830，桌面横放）----
PTUBE_PATH = "/World/PolarimeterTube"
PTUBE_REST = (0.70, 0.30, 0.811)       # 管中心（桌面，旋光仪右前方；底座 0.50 后抓点 3D 0.46m，
                                       # 脱离原 0.33m 低 z 失效区；倒液点贴仪右缘净空、走廊仅 0.1m）
PTUBE_GRASP_Z = 0.83                   # 手指下探到管身 0.83（管径 Ø13，指端 0.803 贴台面余 3mm）
PTUBE_GRASP = (PTUBE_REST[0], PTUBE_REST[1], PTUBE_GRASP_Z)
PTUBE_APPROACH = (PTUBE_REST[0], PTUBE_REST[1], 0.95)    # 高位接近
PTUBE_LIFT = (PTUBE_REST[0], PTUBE_REST[1], H)           # 提出 32cm 净空横移
GRIP_PTUBE = 0.0065                    # Ø13/2
PTUBE_HELD_OFFSET_Z = PTUBE_REST[2] - PTUBE_GRASP_Z   # 管中心=TCP-0.019
# 导轨：tube_rails 顶 1.001、中心 y=-0.03；管搭槽上中心 z=1.001+0.0065
PTUBE_PLACE_CENTER = (0.30, -0.03, 1.0075)
PTUBE_PLACE_TCP = (0.30, -0.03, PTUBE_PLACE_CENTER[2] - PTUBE_HELD_OFFSET_Z)  # 1.0265
PTUBE_PLACE_ABOVE = (0.30, -0.03, 1.10)   # 接近高度 1.10（管 1.081 仍清机身顶 1.05；底座 0.50 后 3D 0.66）

# ---- 倒液（试管 ORIENT_POUR 倒置，管口朝下对准加液口）----
FILL_XY = (PTUBE_REST[0], PTUBE_REST[1] + 0.0325)  # 加液口中心（管泡 +y 端顶，随 PTUBE_REST）
FILL_TOP_Z = 0.830                     # 加液口顶
POUR_APPROACH = (FILL_XY[0], FILL_XY[1], H)
POUR_TCP = (FILL_XY[0], FILL_XY[1], 0.860)   # 管口=TCP-0.014=0.846，加液口上 16mm
POUR_HOLD = 60
POUR_LIFT = (FILL_XY[0], FILL_XY[1], 1.00)   # 倒完提起（仍倒置，须 ≥1.0 才能安全转正：
                                             # 回旋最低点 TCP-0.1393 ≥0.86 > 加液口 0.830）
POUR_MOUTH = (FILL_XY[0], FILL_XY[1], POUR_TCP[2] - 0.014)  # 流束起点 0.846

# ---- 启动按钮（Ø64，机顶前部 (0.30,0.18)，顶 1.056/底 1.050）----
START_BUTTON_XY = (0.30, 0.18)
START_BUTTON_TOP_Z = 1.056
START_BUTTON_PRESS_TCP = (0.30, 0.18, START_BUTTON_TOP_Z)
BUTTON_PATH = "/World/Polarimeter/start_button"
BUTTON_CENTER_Z = 0.253       # 局部中心（Polarimeter 原点半是台面）
BUTTON_REST_Z = BUTTON_CENTER_Z
BUTTON_PRESSED_Z = BUTTON_CENTER_Z - 0.005   # 按下行程 5mm
BUTTON_HALF_H = 0.003
BUTTON_PRESSED_TOP_LOCAL = BUTTON_PRESSED_Z + BUTTON_HALF_H
BTN_APPROACH = (0.30, 0.18, 1.15)          # ①高位接近按钮上方
BTN_PREPRESS = (0.30, 0.18, 0.80 + 0.253 + 0.003 + 0.020)  # ②下探预按位 1.076
BTN_PRESS = (0.30, 0.18, 0.80 + BUTTON_PRESSED_TOP_LOCAL)  # ④按到底 1.051
BUTTON_LIFT_Z = 1.07          # ⑥抬起触发（press + 14mm）
BUTTON_SPRING_STEP = 0.0002   # 弹回速度（5mm/25 帧）
GRIP_BUTTON = 0.032           # Ø64/2

# ---- 屏幕读数（旋转角度 enum，引号字符串防 YAML 浮点）----
ROTATION_OPTIONS = ["+12.5", "+8.4", "+15.0", "-3.2"]
ROTATION_DEFAULT = "+12.5"
ROTATION_KEY = lambda v: v.replace("+", "p").replace("-", "m").replace(".", "_")

# ---- 效果 prim（gen_a2_scene.py 已预制）----
EFFECT_TUBE_POWDER = "/World/TubePowder"
EFFECT_TUBE_WATER = "/World/TestTubeWater"
EFFECT_TUBE_LIQUID = "/World/PolarimeterTube/TubeLiquid"
EFFECT_WATER_STREAM = "/World/WaterStream"
EFFECT_POUR_STREAM = "/World/PourStream"
EFFECT_SCREEN_MEASURING_TPL = "/World/ScreenMeasuring_{step:02d}"
PROGRESS_STEPS = 16
EFFECT_SCREEN_RESULT_TPL = "/World/ScreenGlow_{key}"
