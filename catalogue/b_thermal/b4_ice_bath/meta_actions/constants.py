"""B4 冰浴/冷却 —— 元动作共享常量：坐标 / 抓取 / 高度 / 朝向（与 b4_ice_bath.usd 对齐）。

所有坐标 = TCP（right_gripper）世界坐标，米，Z-up（桌面 z=0.80）。
几何来源：gen_b4_scene.py 布局 + d2s wash_bottle 同款资产实测（wash_bottle.usd 瓶身
Mesh_006 = 6.4×6.4×16.8cm 方柱，世界 z 0.8001-0.9683）。B4 洗瓶 translate
(0.20, 0.10, 0.80) rot180（红嘴朝 +X 正对烧杯，gen_b4_scene EQUIP True）。
坐标系与 flametest/d2s 一致（复用 IkMotionEngine / BaseMetaAction，Lula IK）。
"""
import numpy as np

# —— 高度 / 朝向 ——
H = 1.15            # 安全高位（跨越桌面障碍的水平平移高度）
ORIENT_FWD = (0.0, 0.7071068, 0.0, 0.7071068)   # 手指朝 +X = 朝向 camera1（d2s 同款）

# —— 夹爪开度 ——
GRIP_OPEN = 0.04
GRIP_WASHBOT = 0.030    # 夹肚子开度（半开度）：6cm 开口压 6.4cm 软瓶身每侧 2mm（d2s 同款）

# —— 洗瓶（/World/WashBottle (0.20,0.10,0.80) rot180，红嘴朝 +X 正对烧杯 (0.45,0.10)）——
# 瓶身 Mesh_006 = 6.4×6.4×16.8cm 方柱（可挤压肚子），世界中心 (0.20,0.10)，
#   z 0.8001-0.9683（同 wash_bottle.usd 资产，pxr 实测 d2s）。吸管沿 X 轴（y≈0.10
#   中心线）：红嘴尖朝 +X（正对烧杯）。手指 ORIENT_FWD tool+Z=+X、两指沿 ±Y 张开
#   夹瓶身 ±Y 面，吸管在指间、前支在指端外 → 无碰撞。抓取高度取瓶身中部 z=0.88
#   （d2s 经验：低 z 远伸 IK 冻结，抬高中部给 IK 留俯仰空间）。
WASH_XY = (0.20, 0.10)           # 瓶身（肚子）中心 x,y（= translate）
WASH_GRASP_Z = 0.88              # 抓取高度：瓶身中部（z 0.80-0.97 中心 ≈0.884）
WASH_APPROACH_X = 0.13           # 下探 x 偏移：避开 +X 侧吸管与红色嘴尖（d2s 同法偏 -X 7cm：
                                 #   瓶心 0.20 - 0.07 = 0.13，指端 +X 伸出 0.0277 → 0.1577
                                 #   < 瓶身 -X 壁 0.168 净空 ~1cm）
WASH_GRASP = (WASH_XY[0], WASH_XY[1], WASH_GRASP_Z)   # (0.20,0.10,0.88)
WASH_LIFT = WASH_GRASP_Z + 0.15   # ⑤ 抬升目标 z：抓取 z 0.88 + 15cm = 1.03（用户「抬起来」）
WASH_TO_X = WASH_XY[0] + 0.15     # ⑥ 往 +X 移 15cm：TCP x 0.20→0.35（用户「向+x移动15cm」）

# ---- 挤水（② 挤压洗瓶身，水从红嘴弧线落入烧杯，烧杯内液面上涨 + 冰块浮起；
#      2026-08-30 用户「机械臂往里面挤入液体要真实（可参考a3）…冰块浮起来」）----
WASH_SQUEEZE = 0.020            # 挤水开度：夹爪 0.030→0.020（压软瓶身出水，a3 同款）
WASH_SQUEEZE_CLOSED = 0.025     # task 挤水判定：opening < 0.025 算正在挤（持握 0.030 不误触）
WASH_SQUEEZE_DWELL = 150        # 挤水保持帧数（水流持续 ~2.5s @60Hz，a3 同款）
# 水落点 = 烧杯口顶中心（直立烧杯，口朝上，z=口顶 0.8904）
BEAKER_MOUTH_TOP = (0.45, 0.10, 0.8904)
# 红嘴尖相对瓶原点世界偏移（wash_bottle.usd rot180 → 红嘴朝 +X，a3 同款 pxr 实测）：
#   开口 +X 端 x=+0.1114、z=+0.044（纯平移持握瓶朝向恒定）
SPOUT_TIP_OFFSET = (0.1114, 0.0, 0.044)
WATER_DROPS = 16                # 水滴池大小（round-robin 复用）
WATER_STAGGER = 2               # 相邻水滴发射间隔帧
WATER_FALL = 12                 # 每滴沿抛物线坠落帧数
WATER_LAND_FULL = 64            # 液面涨满需要的落定水滴数
# 烧杯内液面（/World/BeakerLiquid 圆柱，淡蓝半透明；挤水时液面随水流上涨）
BEAKER_LIQUID_PATH = "/World/BeakerLiquid"
BEAKER_LIQUID_R = 0.030         # 液柱半径（烧杯内径 ~Ø60，< 外 Ø76）
BEAKER_LIQUID_H0 = 0.004        # 初始液面高（≈0，几乎不可见）
BEAKER_LIQUID_H_MAX = 0.040     # 挤完最终液面高（液面顶 0.840，烧杯高 0.80..0.8904 内）

# ---- 冰块浮起（挤水后烧杯内 6 块冰块随液面上涨浮到水面，符合物理：冰密度 < 水）----
# 冰块 /World/Ice_0..5（fix_ice_cube 缩到 ~1.5cm，底 z=0 顶 z=0.0121），随液面上涨浮到
# 水面：冰底 = 台面 + 液面高 − 冰高（冰浮于水，顶贴水面）。task 逐帧写 translate z。
ICE_PATHS = [f"/World/Ice_{i}" for i in range(6)]
ICE_HEIGHT = 0.0121              # 冰块高（gen_b4_scene 实测底 0 顶 0.0121）
ICE_FLOAT_SUBMERGE = 0.90        # 冰浮出水面的淹没比：冰密度 ~0.90 水 1.0 → ~10% 露出水面
                                 #   （用户「冰块应该有一部分露出水面」；原顶贴水面=全浸没）

# ---- 试管（test_tube.usd，Ø19.2×153mm，立试管架近侧左孔 (0.279,0.241,0.806)，口 0.9593；
#      管内预装药品液柱 /World/TubeDrug (r0.0086 h0.040 中心 0.826)）----
# 近侧孔=最靠 -Y（机器人侧），2026-08-30 用户「够不到 → 放 -y 侧」（原 +y 远孔 0.476 抓不到）。
# 竖直提取（2026-08-30 用户「试管不要水平横夹还是竖直提取出来吧（参考d3l）」）：手指朝下
# （orient=None 默认），抓管口下 14mm，纯平移持握（管底吊夹爪下方，保竖立）。
TUBE_XY = (0.279, 0.241)
TUBE_REST_Z = 0.806              # 管底静置 z（架孔底）
TUBE_GRASP_Z = 0.9453            # 抓点 z = 管口 0.9593 − 14mm（d3l 同款）
TUBE_GRASP_TCP = (TUBE_XY[0], TUBE_XY[1], TUBE_GRASP_Z)
GRIP_TUBE = 0.0096               # 试管 Ø19.2/2（b1/d2s 同款）
TUBE_HIGH = 1.10                 # 拿管提出高度（管底 0.961 清架顶 0.917）
TUBE_HELD_Z = TUBE_GRASP_Z - TUBE_REST_Z   # 0.1393：管底在夹爪下方 z 偏移（纯平移持握）
TUBE_DRUG_OFFSET_Z = 0.020       # 药品液柱中心距管底（0.826 − 0.806）

# ---- 浸冰（2026-08-30 用户「继续加动作把试管移动到烧杯上方并浸入冰水」）----
# 试管竖直夹持从 (TUBE_XY, TUBE_HIGH) 水平移到烧杯口上方 (0.45,0.10)，再竖直下探把管底浸入
# 冰水（水面 = 烧杯内底 0.802 + 液面高 0.040 = 0.842）。管底目标 0.828（水面下 1.4cm，
# 内底 0.802 上 2.6cm，避内底）。TCP z = 管底 + TUBE_HELD_Z。
TUBE_ABOVE_BEAKER = (0.45, 0.10, TUBE_HIGH)   # 试管移到烧杯口上方（管底 0.961 清杯口 0.8904）
TUBE_IMMERSE_BOTTOM_Z = 0.828                 # 管底浸入目标 z（水面 0.842 下 1.4cm）
TUBE_IMMERSE_TCP = (0.45, 0.10, TUBE_IMMERSE_BOTTOM_Z + TUBE_HELD_Z)   # ≈0.9673
IMMERSE_DWELL = 900                           # 浸冰保持帧数（冷却 15s @60Hz，用户「冰浴时间再增加10s」）
OBSERVE_DWELL = 480                           # 冰浴提出后看现象停留 8s @60Hz（浑浊渐褪回澄清 + 起雾）
TUBE_ABOVE_RACK = (TUBE_XY[0], TUBE_XY[1], TUBE_HIGH)   # (0.279,0.241,1.10) 试管架上方（放回，够得到）

SETTLE = 12                      # 到位稳定帧（a3/b1 同款）

# ---- 液面/水流效果（gen add_effects 烘焙，task 运行时驱动；值须与 gen_b4_scene.py 一致）----
BEAKER_INNER_BOTTOM_Z = 0.802    # 烧杯内底（杯外底 0.80 + 壁厚 2mm）：液面/冰块坐于此
WATER_DROP_R = 0.004             # 水流水滴半径（4mm，红嘴尖坠入烧杯口）
WATER_DROP_COLOR = (0.85, 0.88, 0.92)   # 水无色（近白微冷，透亮）
LIQUID_COLOR = (0.85, 0.88, 0.92)       # 烧杯内液面无色（近白，透亮）
# 试管内药品液柱变体路径 = /World/TubeDrug_<liquid_color>（颜色化后，task.py 动态拼接；
# 浑浊 /World/TubeCloud_<色>_<1..3>、晶体 /World/TubeCrystal_<色> 同法，见 gen_b4_scene.py）
