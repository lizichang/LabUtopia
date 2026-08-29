# -*- coding: utf-8 -*-
"""A2 旋光仪测量元动作共享常量。

所有坐标 = TCP（tool_center / right_gripper）世界坐标，米，Z-up（台面顶 z=0.80）。
几何来源：2026-08-27 pxr 读 gen_a2_scene.py 世界包围盒 + polarimeter.usd 三改
（scripts/fix_polarimeter.py）+ 旋光管资产（scripts/gen_polarimeter_tube.py）。

布局（2026-08-27 滴管转移改造：倒液 → 胶头滴管吸3次挤进加液口）：
  d2s 同款三件（坐标与 gen_d2s_scene.py 一字不差，复用其已证可行包络）：
    TestTubeRack (0.6803,0.3607)  架顶 z=0.917
    TestTube     (0.659,0.241,0.806)  立插架近侧孔，管口 z=0.9593、管底 0.806
    WashBottle   (0.370,0.525)   rotZ-180（红嘴朝 +X 对试管方向，同 d2s），瓶身 z 0.80..0.97
  A2 专用三件（全部在机械臂 +x 侧，底座 (-0.15,0.05)）：
    Polarimeter     (0.48,-0.24)  rotZ+90°：机身 x 0.175..0.788 / y -0.425..-0.05 / z 0.80..1.05，
                                  屏幕朝 -x（相机2 从 -x 拍）、lid 已掀 120°（铰链 -x，盖尖升到 ~1.27）；
                                  顶板前部红色启动键 Ø64（局部 (0,0.18,0.253)）→ 世界 (0.30,-0.24) 顶 1.056
    Dropper         (0.6993,0.4003,0.806)  胶头滴管立插主试管架第二列第5排（尖嘴底 0.806）
    PolarimeterTube 1dm (0.5265,0.241,0.811)  桌面空管横放 rotZ-90°（轴 X、泡 +x、加液口 +x 端顶 0.830）；
                                  加液口 = 滴管滴液点（试管口 0.659 往 -x 10cm = 0.559），
                                  管身向 -x 伸到 0.4685 清开试管 6.5cm（rotZ+90 泡 -x 会顶试管）

持握约定：
  试管（/World/TestTube）    旋转跟随 _T_HELD_TUBE（管底吊夹爪下 0.1393m），抓管口下 14mm，仅震荡周期
  洗瓶（/World/WashBottle）  动态锁 _T_HELD_WASHB = 静止矩阵 · tool^-1，横夹肚子
  滴管（/World/Dropper）     _T_HELD_DROPPER 沿 tool+X 伸 0.13（同 d3s 酸滴管，尖嘴吊夹爪下），
                              吸试管内液 → 挤进旋光管加液口（3 次）
  旋光管（/World/PolarimeterTube） 纯平移持握（set_object_position），保持横放泡朝上，
                              管中心 = TCP + PTUBE_HELD_OFFSET(0,0,-0.019)
"""

# ---- 基础 ----
H = 1.15                     # 高位横移高度（被持物最低点 > 沿途最高障碍）
SETTLE = 12                  # 到位稳定帧
GRIP_OPEN = 0.04             # 夹爪满开
# 引擎四元数约定 [x,y,z,w]（scipy 序）；R_y(±90°)
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)          # 手指朝 +X（夹药匙/横夹洗瓶/试管/滴管）
# 夹旋光管（横放轴 X）**不用显式 orient**：引擎默认朝向（手指朝下、指隙沿 X）就是正确朝向
# （用户 2026-08-28 拍板：原方向对，只是点位有偏差）。曾误改 ORIENT_PTUBE=(0.7071,0.7071,0,0)
# 让指隙沿 ±Y =「从侧边夹」，用户判「完全改错、根本加不起来」，已回退。

# ---- 试管（d2s 同款 Ø19.2×153mm，立插架近侧孔；d2s 坐标）----
TUBE_XY = (0.659, 0.241)
TUBE_MOUTH_Z = 0.9593        # 管口（架顶 0.917 之上）
TUBE_ORIG_Z = 0.806          # 管底
TUBE_GRASP_TCP = (0.659, 0.241, TUBE_MOUTH_Z - 0.014)   # 抓管口下 14mm = 0.9453
GRIP_TUBE = 0.0096           # Ø19.2/2
TUBE_HELD_OFFSET_Z = TUBE_ORIG_Z - (TUBE_MOUTH_Z - 0.014)  # 管底吊夹爪下 0.1393
SHAKE_CENTER_TCP = (0.659, 0.241, 1.09)                 # 提出架顶 17cm 震荡
SHAKE_AMPLITUDE = 0.04
SHAKE_PERIOD = 60
SHAKE_HOLD_FRAMES = 30

# ---- 洗瓶（d2s 同款 6.4×6.4×16.8cm，rotZ-180 红嘴朝 +X；d2s 坐标）----
WASH_XY = (0.370, 0.525)
WASH_GRASP_Z = 0.88          # 横夹肚子高度（瓶身 z 0.80..0.97，夹中段）
WASH_APPROACH_X = 0.30       # 下探 x 偏移：避 +X 嘴尖，从瓶身 -X 外侧水平移入（d2s 同款折叠构型）
WASH_GRASP = (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z)
GRIP_WASHBOT = 0.030         # 半开度：6cm 开口压 6.4cm 软瓶身每侧 2mm
WASH_LIFT = 1.03             # 提出 15cm
WASH_NOZZLE_OFF = (0.106, 0.0, -0.036)  # 红嘴相对 TCP（d2s 实测：嘴=TCP+偏移）
# 嘴对管口 = d2s 偏移式（WASH_XY + (0.173,-0.294)），TCP x=0.543 / y=0.231（嘴落管口 1cm 偏）
WASH_TO_TUBE_X = WASH_XY[0] + 0.173
WASH_TO_TUBE_Y = WASH_XY[1] - 0.294
WASH_SQUEEZE = 0.020         # 挤水开合（<0.025 出水）
WASH_SQUEEZE_CLOSED = 0.025
WASH_SQUEEZE_DWELL = 150
WASH_GRIP_OPEN = 0.038       # 松开阈值（0.04=满开永不 released）
WATER_START = (WASH_TO_TUBE_X + WASH_NOZZLE_OFF[0], WASH_TO_TUBE_Y,
               WASH_LIFT + WASH_NOZZLE_OFF[2])          # 嘴尖 (0.649,0.231,0.994)
WATER_END = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z)     # 落管口 0.9593

# ---- 洗瓶预摆位（PrePoseWash，乱动修复）----
# d2s 的 PickWashBottle 是第 3 个动作（从 ReturnSpatula 终点 = 药匙架高位 ORIENT_FWD 进入）；
# A2 的是第 1 个动作（sim 冷启动直接单次 IK 到洗瓶上方）→ 落 IK 另一解分支，送红嘴
# linewalk 中途 FK 朝向不合格保持/跳变 = 用户看到的「乱动」。预摆到 d2s 同款入臂姿势
# 再走洗瓶，分支连续、送嘴走直线（坐标一样就能照搬 d2s 行为）。
PREPOSE_XY = (0.6993, 0.3608)   # d2s 药匙架位（A2 场景该处为空：架后方高位，3D 1.0m 同 d2s 已证）

# ---- 旋光管（1dm，rotZ-90° 轴 X、泡 +x、加液口 +x 端顶 0.830，桌面横放）----
# 2026-08-27 滴管转移改造：加液口 = 滴管滴液点（试管口 0.659 往 -x 10cm → 0.559）。
# rotZ-90 映射 局部(lx,ly)→世界(px+ly, py-lx)；加液口局部 (0,+0.0325)→世界 (px+0.0325, py)：
#   PTUBE_REST.x = 0.559 − 0.0325 = 0.5265。管身伸向 -x 到 0.4685（清开试管 6.5cm，不顶）；
#   泡 +x 端 (0.548..0.570) 朝滴液点，抓点 x=0.5265 落在泡西侧净管身上。
#   （rotZ+90 泡 -x 会把管身推到 0.5915 顶试管 0.6494，弃用。）
PTUBE_PATH = "/World/PolarimeterTube"
PTUBE_REST = (0.5265, 0.241, 0.811)     # 管中心（加液口 0.559 泡在此上方；与试管同 y=0.241）
PTUBE_GRASP_Z = 0.83                   # 手指下探到管身 0.83（管径 Ø13，指端 0.803 贴台面余 3mm）
PTUBE_GRASP = (PTUBE_REST[0], PTUBE_REST[1], PTUBE_GRASP_Z)
PTUBE_APPROACH = (PTUBE_REST[0], PTUBE_REST[1], 0.95)    # 高位接近
PTUBE_LIFT = (PTUBE_REST[0], PTUBE_REST[1], 1.20)        # 提出到 1.20（管底 1.1745 清掀开翻盖自由边 1.1475）
GRIP_PTUBE = 0.0065                    # Ø13/2
PTUBE_HELD_OFFSET_Z = PTUBE_REST[2] - PTUBE_GRASP_Z   # 管中心=TCP-0.019
# 导轨：tube_rails 顶 1.001、中心 x=+0.03（仪局部，rotZ+90 后）；管搭槽上中心 z=1.001+0.0065
PTUBE_PLACE_CENTER = (0.51, -0.24, 1.0075)   # 仪 (0.48,-0.24) rotZ+90 + 局部 (+0.03,0)
PTUBE_PLACE_TCP = (0.51, -0.24, PTUBE_PLACE_CENTER[2] - PTUBE_HELD_OFFSET_Z)  # 1.0265
PTUBE_PLACE_ABOVE = (0.51, -0.24, 1.18)   # 接近/松爪高度 1.18（-y 横移越过掀开翻盖自由边 1.1475，不穿模；
                                          #   旧 1.20 在深 -y 近奇异 Lula 解不出 → IK FAIL/force-done；1.18 可达）

# ---- 加液口（滴管滴液点：试管口 0.659 往 -x 10cm → (0.559,0.241)，加液口顶 0.830）----
FILL_XY = (PTUBE_REST[0] + 0.0325, PTUBE_REST[1])  # 加液口中心（泡 +x 端顶，随 PTUBE_REST；
                                                    # rotZ-90 后管轴沿 x，局部 (0,+0.0325)→世界 (+0.0325,0)）
FILL_TOP_Z = 0.830                     # 加液口顶

# ---- 胶头滴管（架第二列第5排 (0.6993,0.4003)，吸试管内液→挤进加液口，全程水平横夹 ORIENT_FWD）----
# 2026-08-27 用户「滴管放第四列第一排」澄清=主试管架第二列第5排 (0.6993,0.4003)；
# 持握 d3s 同款：_T_HELD_DROPPER 沿 tool+X 伸 0.13（尖嘴吊夹爪下），抓点 = 尖嘴底 + 0.13。
# 注意：该点 3D 0.946m 贴近 d2s 已证包络上限（d3s 滴管 0.910m），若 IK FAIL 回退列1第5排
# (0.659,0.4003)（d3s 已证）。全程 ORIENT_FWD，无手腕翻转。
DROP_PATH = "/World/Dropper"
DROP_XY = (0.6993, 0.4003)
GRIP_DROPPER = 0.0055      # 持握开度（Ø11 胶头/2，d3s 同款）
GRIP_SQUEEZE = 0.002       # 挤胶头（排空气/滴液）
GRIP_ASPIRATE = 0.0055     # 松胶头吸液后回持握开度（移动全程贴合胶头面）
TIP_OFFSET = 0.13          # 尖嘴 = 夹爪 + 0.13·tool+X（ORIENT_FWD → 世界 -Z，尖嘴吊夹爪下）
DROP_GRASP = (DROP_XY[0], DROP_XY[1], 0.806 + TIP_OFFSET)   # = 0.936（架顶 0.917 之上可握段，夹胶头）
DROP_REST = (DROP_XY[0], DROP_XY[1], 0.806)                 # 架内竖插静止位（尖嘴底=架孔底 0.806）
# 吸液：尖嘴从试管口（0.9593）沉入液面下 20mm（液面顶 0.8725、管底 0.806，余 46mm 不碰底）
TUBE_LIQUID_H = 0.035
TUBE_LIQUID_TOP_Z = TUBE_ORIG_Z + 0.049 + TUBE_LIQUID_H / 2.0   # 液面顶 ≈ 0.8725
DIP_INSET = 0.020
DROPPER_SQUEEZE_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z + 0.005 + TIP_OFFSET)   # 管口上 5mm → 1.0943
DROPPER_DIP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_LIQUID_TOP_Z - DIP_INSET + TIP_OFFSET)  # 尖嘴 0.8525 → 0.9825
# 滴液：尖嘴抬到加液口顶上方 25mm（液滴坠落可见，不沉入口被壁挡住）；尖嘴 0.855 → TCP 0.985
DROPPER_DROP_RAISE = 0.025
DROPPER_DROP_TCP = (FILL_XY[0], FILL_XY[1], FILL_TOP_Z + DROPPER_DROP_RAISE + TIP_OFFSET)
DROP_CYCLES = 3            # 吸3次挤3次（用户「总共从试管里面吸三次液体挤到旋光管里面」）

# ---- 启动按钮（Ø64，机顶前部 (0.30,-0.24)，顶 1.056/底 1.050；rotZ+90 后局部 (0,0.18)→世界 (-0.18,0)）----
START_BUTTON_XY = (0.30, -0.24)
START_BUTTON_TOP_Z = 1.056
START_BUTTON_PRESS_TCP = (0.30, -0.24, START_BUTTON_TOP_Z)
BUTTON_PATH = "/World/Polarimeter/start_button"
BUTTON_CENTER_Z = 0.253       # 局部中心（Polarimeter 原点半是台面）
BUTTON_REST_Z = BUTTON_CENTER_Z
BUTTON_PRESSED_Z = BUTTON_CENTER_Z - 0.005   # 按下行程 5mm
BUTTON_HALF_H = 0.003
BUTTON_PRESSED_TOP_LOCAL = BUTTON_PRESSED_Z + BUTTON_HALF_H
BTN_APPROACH = (0.30, -0.24, 1.15)          # ①高位接近按钮上方
BTN_PREPRESS = (0.30, -0.24, 0.80 + 0.253 + 0.003 + 0.020)  # ②下探预按位 1.076
BTN_PRESS = (0.30, -0.24, 0.80 + BUTTON_PRESSED_TOP_LOCAL)  # ④按到底 1.051
BUTTON_LIFT_Z = 1.07          # ⑥抬起触发（press + 14mm）
BUTTON_SPRING_STEP = 0.0002   # 弹回速度（5mm/25 帧）
GRIP_BUTTON = 0.0             # 完全闭合（指端并拢压按钮顶；Ø64 宽按钮需全闭才有按力，
                              # 0.032=指间距 64mm 零夹紧、按不动 = 用户报「按不下去」）

# ---- 翻盖（lid，⑩ 按启动键前拨回盖上；asset rotateXYZ.Y 120° 掀开 → 0° 闭合）----
# 几何（pxr 实测，Polarimeter (0.48,-0.24) rotZ+90）：lid 铰链沿世界+X 在机身近侧
#   y−0.184 / z1.0505（过 lid 局部原点，轴沿世界±X，板伸向深侧 −y）；掀 120° 板斜向上，
#   自由边 y−0.128/z1.1475，板面在 TCP z1.09 处 y−0.161；闭 0° 盖 x 0.383..0.643 /
#   y −0.296..−0.184 / z 1.0505..1.0595（盖住导轨上试管顶 1.017）。
# **可达性（底座 (-0.15,0.05,0.71)）**：翻盖在机身近侧（y≥−0.184），推点
#   (0.51,−0.15,1.10)（触板近面，3D 0.79m）→ (0.51,−0.24,1.10)（向 −y 折叠，3D 0.82m）
#   全部 ≤0.83m 可达——旧 x 向拨盖 y−0.24 深侧 z 上限 0.83 够不到翻盖顶是根因。task 按
#   夹爪 y 进度 −0.15→−0.22 联动 rotateXYZ.Y 120°→0°。x 固定 0.51（机身中线）；y −0.24
#   深侧用引擎默认 orient（PlaceOnRails 同款：显式朝向 FK 解不出 IK）。
LID_PATH = "/World/Polarimeter/lid"
LID_PUSH_X = 0.51                # 推盖 x（机身中线；task 联动 x 门控、夹爪路径都用它）
LID_PUSH_Y0 = -0.15              # task 联动：y ≤ −0.15 lid 开始转闭（推盖起点，触板近面）
LID_PUSH_Y1 = -0.22              # task 联动：y ≤ −0.22 lid 完全闭合（freeze y≤−0.23<Y1 必闭）
LID_APPROACH_HIGH = (0.51, -0.11, 1.20)   # ① 高位接近（越过掀开翻盖自由边 y−0.128/z1.1475，不再穿模）
LID_PUSH_START = (0.51, -0.11, 1.18)      # ② 下探到推盖高度（手指 1.153 清自由边 1.1475，贴板近面）
LID_PUSH_END = (0.51, -0.24, 1.18)        # ③ 水平向 −y 折叠拨回（task 按 y 过 −0.15 联动闭合；
                                           #   目标 −0.24，freeze 3D y≤−0.23<Y1 必闭；3D 0.82m）
LID_LIFT = (0.51, -0.11, 1.20)            # ⑥ 抬离（退回近侧 LID_APPROACH_HIGH，交回 PressStartPass；
                                          #   旧深 -y (0.51,-0.24,1.20) 近奇异 IK FAIL → 退浅侧同高）
GRIP_LID = 0.005                          # 推盖时爪近闭（指端贴板面作推面）
LID_OPEN_DEG = 120.0                   # 掀开角（asset 烘焙 rotateXYZ.Y）

# ---- 屏幕读数（旋转角度 enum，引号字符串防 YAML 浮点）----
ROTATION_OPTIONS = ["+12.5", "+8.4", "+15.0", "-3.2"]
ROTATION_DEFAULT = "+12.5"
ROTATION_KEY = lambda v: v.replace("+", "p").replace("-", "m").replace(".", "_")

# ---- 效果 prim（gen_a2_scene.py 已预制）----
EFFECT_TUBE_POWDER = "/World/TubePowder"
EFFECT_TUBE_WATER = "/World/TestTubeWater"
EFFECT_TUBE_LIQUID = "/World/PolarimeterTube/TubeLiquid"
EFFECT_WATER_STREAM = "/World/WaterStream"
EFFECT_DROPPER_DRIP = "/World/DropperDrip"
EFFECT_SCREEN_MEASURING_TPL = "/World/ScreenMeasuring_{step:02d}"
PROGRESS_STEPS = 16
EFFECT_SCREEN_RESULT_TPL = "/World/ScreenGlow_{key}"
