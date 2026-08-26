"""A1 折光率测量元动作共享常量。

所有坐标 = TCP（right_gripper / tool_center）世界坐标，米，Z-up（桌面 z=0.80）。
几何实测来源：2026-08-25 pxr 读 a1_refractometer.usd 世界包围盒（gen_a1_scene.py
verify 输出）：
  Refractometer (0.30,0.00)   机身 x ±0.1125 / y -0.165..+0.165 / z 0.80..0.915，
                              棱镜中心 (0.30,0.11) 顶 z=0.9175，盖内置 -50° 掀开态
                              （盖 bbox y 0.1167..0.1388，向后 +Y 掀开，棱镜前/上暴露）
  SampleBottle   (0.10,0.34)  瓶 Ø36、口 rim z=0.870、液面 z=0.840（半瓶）；
                              白塞 stopper Ø25.2（局部 z 0.068..0.079 中心 0.0735）
                              → 世界几何中心 (0.10,0.34,0.8735)
  TestTubeRack   (0.55,0.20)  架顶 z=0.917、孔底 z=0.806
  Dropper        (0.531,0.1996,0.806)  插左孔，尖嘴底=原点，胶头 Ø11（z 0.921..0.9562）

滴管持握约定（task 持握 = TCP + HELD_OFFSET(0,0,-0.13)，纯平移保竖立，flametest/
d4l 同款）：滴管尖嘴 0.13m 吊在夹爪下方。故 TCP z = 尖嘴 z + 0.13；抓点 = 立放位
+ (0,0,0.13)：滴管尖嘴 0.806，抓点 0.936（架顶 0.917 之上可握段）。

瓶塞持握约定（task 持握 = TCP + CAP_HELD_OFFSET(0,0,-0.0035)，抓近顶）：塞世界
几何中心 0.8735，抓点 0.877（近顶，同 flametest 瓶塞抓法）。
"""
import numpy as np

# —— 高度 / 停留 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
SETTLE = 12         # 到点 settle 帧数

# —— 夹爪开度（开度值 = joint[7]，≈ 实际指间距的一半）——
GRIP_OPEN = 0.04            # 松开（放回滴管/瓶塞）
GRIP_STOPPER = 0.0126       # 抓瓶塞：Ø25.2mm/2（同 flametest HCl 瓶塞）
GRIP_DROPPER = 0.0055       # 抓滴管：胶头 Ø11mm/2（同 d4l，贴合胶头面）
GRIP_SQUEEZE = 0.002        # 挤胶头（排空气 / 滴样）
GRIP_ASPIRATE = 0.0055      # 松胶头吸液后回持握开度（移动全程贴合胶头面）
GRIP_BUTTON = 0.016         # 按测量键：合爪夹住按钮两侧（按钮 Ø32mm/2，2026-08-26 加宽）

# —— 滴管尖嘴到夹爪距离（持握 _T_HELD 的 z 偏移）——
TIP_OFFSET = 0.13           # 尖嘴在夹爪下方 0.13m（dropperdrip grasp_offset，实测）

# —— 瓶塞（/World/SampleBottle/stopper，瓶口白塞）——
CAP_PATH = "/World/SampleBottle/stopper"
CAP_PARENT_PATH = "/World/SampleBottle"
CAP_PARENT_T = np.array([0.10, 0.34, 0.80])     # 瓶 Xform 世界平移（塞父级）
CAP_LOCAL_OFFSET = np.array([0.0, 0.0, 0.0735]) # 塞 mesh 局部几何中心（顶点 z 0.068..0.079）
CAP_GRASP = (0.10, 0.34, 0.8770)                # 抓近顶（塞顶 0.879 下 2mm，同 flametest）
CAP_REST = (0.10, 0.34, 0.8735)                 # 塞世界几何中心（瓶口静置）
CAP_HELD_OFFSET = np.array([0.0, 0.0, -0.0035]) # 持握：塞中心在夹爪下 3.5mm
CAP_DESK = (0.16, 0.34, 0.806)                  # 拔下后倒放桌面（瓶 +X 侧 6cm，中心离台 6mm）

# —— 滴管（/World/Dropper，插架左孔）——
DROP_PATH = "/World/Dropper"
DROP_XY = (0.531, 0.1996)
DROP_REST = (0.531, 0.1996, 0.806)              # 架内竖插静止位姿（尖嘴底=原点，立放底面 z=0.806）
DROP_GRASP = (0.531, 0.1996, 0.806 + TIP_OFFSET)  # = 0.936（架顶 0.917 之上可握段）

# —— 样品瓶（/World/SampleBottle (0.10,0.34)，去塞后口 rim 0.870，液面 0.840）——
BOTTLE_XY = (0.10, 0.34)
BOTTLE_MOUTH_Z = 0.870       # 瓶口 rim 世界 z（去塞后）
LIQUID_TOP_Z = 0.840         # 瓶内液面（SampleLiquid 顶）
DIP_INSET = 0.040            # 浸液：尖嘴沉入瓶口下 40mm → 尖嘴 z=0.830 < 液面 0.840
# 瓶口挤空气（尖嘴贴瓶口 rim 上方 5mm）与浸液吸液（尖嘴 0.830 入液面下 10mm）的 TCP：
BOTTLE_SQUEEZE_TCP = (BOTTLE_XY[0], BOTTLE_XY[1],
                      BOTTLE_MOUTH_Z + 0.005 + TIP_OFFSET)          # = 1.005
SAMPLE_DIP_TCP = (BOTTLE_XY[0], BOTTLE_XY[1],
                  BOTTLE_MOUTH_Z - DIP_INSET + TIP_OFFSET)          # = 0.960

# —— 棱镜（折光仪 /World/Refractometer (0.30,0.00)，棱镜中心 (0.30,0.11) 顶 z 0.9175）——
PRISM_XY = (0.30, 0.11)
PRISM_TOP_Z = 0.9175         # 棱镜顶世界 z（滴样落点）
PRISM_DROP_RAISE = 0.025     # 滴样：尖嘴抬到棱镜顶上方 25mm（液滴从尖嘴坠落到棱镜面可见）
PRISM_DROP_TCP = (PRISM_XY[0], PRISM_XY[1],
                  PRISM_TOP_Z + PRISM_DROP_RAISE + TIP_OFFSET)      # = 1.0725

# —— 机顶测量键（/World/Refractometer/start_button，棱镜正前方 -y 6cm、凸起 6mm）——
# 2026-08-25 资产加测量键（红色矮圆柱，局部 (0,0.05,0.115..0.121)）→ 世界顶 z 0.921。
# 机械臂滴样后：平移按钮上方 → 下探按下（手指触按钮顶 z≈0.921）触发测量。
START_BUTTON_XY = (0.30, 0.05)
START_BUTTON_TOP_Z = 0.80 + 0.121          # 0.921 按钮顶（机顶 0.915 + 高 6mm）
START_BUTTON_PRESS_TCP = (0.30, 0.05, START_BUTTON_TOP_Z)  # 按下 TCP（手指触按钮顶）

# —— 合盖圆盘（/World/Refractometer/Cover/Disc，铰链 well 后沿 (0.30,0.127,0.9215) X 轴）——
# 真实 Abbemat 棱镜盖 = 圆形磁吸样品盖（用户 2026-08-26 视频看着像方形，实为 Ø34 圆盘）。
# 掀开态 rotateX=-55：圆盘绕铰链立起、前缘抬到 z≈0.9505（verify 实测 bbox 0.1075..0.1286）。
# 合平态 rotateX=0：圆盘盖住 well（y 0.093..0.127 / z 0.9215..0.9235）。
# 合盖方法（用户 2026-08-25）：爪子先待盖子 +y 侧面，再沿 -y 推前缘 → 盖自动合平。
# 推的高度取板面中上部 z 0.94（该高度板面 y≈0.115，越过前缘即触发合盖动画）。
COVER_PATH = "/World/Refractometer/Cover"
COVER_HINGE_XY = (0.30, 0.127)          # 铰链 xy（well 后沿）
COVER_OPEN_ANGLE = -55.0                # 掀开态 rotateX（场景初值）
COVER_CLOSED_ANGLE = 0.0                # 合平态 rotateX
COVER_PUSH_Z = 0.94                     # 推的高度（板面中上部）
COVER_PUSH_TRIGGER_Y = 0.115            # 爪子 y 越过此值（前缘 0.1075 稍外）触发合盖
COVER_PUSH_Z_MIN, COVER_PUSH_Z_MAX = 0.90, 0.97   # 触发 z 窗（推高度附近）
COVER_APPROACH = (0.30, 0.16, H)        # 高位接近（铰链 +y 后方）
COVER_PUSH_START = (0.30, 0.16, COVER_PUSH_Z)     # 推起点（板面 +y 侧）
COVER_PUSH_END = (0.30, 0.10, COVER_PUSH_Z)       # 推终点（越过前缘 0.1075）

# —— 按下测量键（PressStartPass：滴样合盖后按下触发测量读数）——
# 按钮 = /World/Refractometer/start_button（机顶红色矮圆柱 Ø32mm、凸起 6mm：
# 底 0.915..顶 0.921、中心 (0.30,0.05)，见 START_BUTTON_*）。按下 = 爪子垂直下探到
# 按钮顶（手指触顶），task._ButtonLifecycle 检测爪子近按钮顶 → 触发测量：先显示
# ScreenMeasuring（红进度条「测量中」），MEASURE_FRAMES 帧后切 ScreenGlow（绿满条 + nD 读数）
# + 按钮下沉 5mm（顶 0.921→0.916，凸起 6mm 几乎完全下压，视觉"按下"效果）。
BTN_APPROACH = (START_BUTTON_XY[0], START_BUTTON_XY[1], H)          # 高位接近按钮上方
BTN_PREPRESS = (START_BUTTON_XY[0], START_BUTTON_XY[1], 0.94)       # 预按位（按钮顶上方 2cm）
BUTTON_PATH = "/World/Refractometer/start_button"
BUTTON_REST_Z = 0.118          # 按钮静止局部 z（顶 0.921，凸起 6mm）
BUTTON_PRESSED_Z = 0.113       # 按下局部 z（下沉 5mm，顶 0.916 ≈ 机顶 0.915 几乎完全下沉）
BUTTON_HALF_H = 0.003          # 按钮半高（bbox 绕 translate 原点 ±3mm → 顶 = translate z + 0.003）
BUTTON_PRESSED_TOP_LOCAL = BUTTON_PRESSED_Z + BUTTON_HALF_H   # 0.116 按下后顶面局部 z
# 按下到底（用户 2026-08-26）：夹爪降到「下沉后按钮顶面」0.916。下探路过 0.921（检测点）先
# 触发 → 按钮下沉 → 手指继续压到底。旧 0.921 = 静止顶，按钮下沉后手指悬空 5mm。
BTN_PRESS = (START_BUTTON_XY[0], START_BUTTON_XY[1], 0.80 + BUTTON_PRESSED_TOP_LOCAL)  # 0.916
# 抬手按钮弹回（用户 2026-08-26）：夹爪抬离 > BUTTON_LIFT_Z 触发缓慢上抬回 BUTTON_REST_Z。
BUTTON_LIFT_Z = 0.930          # 夹爪抬离判定 z（按下停在 0.916，抬到 0.94 触发弹回）
BUTTON_SPRING_STEP = 0.0002    # 每帧上抬 0.2mm（0.113→0.118 共 25 帧 ≈ 0.4s 缓慢弹回）

# —— 折光率读数输入（experiment_result schema 写回 cfg.n_d，config level2_A1Refractometer.yaml
# 同源，勿单边改）：屏幕 nD 读数由输入决定（d3l 同款——headless 下运行时改材质不渲染，
# 故 gen 预烘焙每档一张贴图 + 一个 ScreenGlow_<key> prim，task 按 cfg.n_d 选一个 show）。
#   key = 读数去小数点（1.4000 → 1_4000）；gen_a1_scene.py N_D_OPTIONS 须与此一致。
N_D_OPTIONS = ["1.3300", "1.3610", "1.4000", "1.4600"]   # 常见液体折射率（水/乙醇/琥珀液/高折射）
N_D_DEFAULT = "1.4000"

# —— 效果 prim 路径（scene 内建，task 动画驱动）——
EFFECT_PRISM_DROP = "/World/PrismDrop"          # 棱镜面液滴（滴样后显示）
EFFECT_DROPPER_FILL = "/World/DropperFill"      # 滴管尖内截锥液柱（吸液后显示，跟随尖嘴）
EFFECT_DROPPER_DROP = "/World/DropperDrop"      # 挤胶头滴落时从尖嘴坠落的液滴（task 动画坠落）
# 测量进度条动画（用户 2026-08-26：先显示进度条，~4s 走完最后显示结果）——headless 下
# 运行时改材质/贴图不渲染 → gen 预烘焙 PROGRESS_STEPS 帧 screen_measuring_<i>.png +
# ScreenMeasuring_<i> prim（红条 0%→100%），task 测量期每帧按进度切显一个。
#   PROGRESS_STEPS 须与 scripts/gen_a1_scene.py 一致（勿单边改）。
EFFECT_SCREEN_MEASURING_TPL = "/World/ScreenMeasuring_{step:02d}"  # 测量中进度条帧（i=0..PROGRESS_STEPS-1）
PROGRESS_STEPS = 16        # 进度条帧数（每帧 ~0.25s @ MEASURE_FRAMES 240 帧 / 4s）
EFFECT_SCREEN_RESULT_TPL = "/World/ScreenGlow_{key}"    # 完成读数屏幕（nD 读数，key = 去小数点档位）
