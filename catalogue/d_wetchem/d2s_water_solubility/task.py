"""D2-S 水溶性任务：药匙随夹爪 6-DOF 持握 + 粉末/倒入效果。

与 flametest 的关键差异：药匙必须随夹爪**旋转**（竖直提起 → 过架顶后放平），所以不是
flametest 的 set_object_position 平移跟随，而是每帧把药匙世界位姿写为
  药匙世界 = T_held · tool_center 世界矩阵
T_held = 平移(0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）——
药匙相对夹爪沿"局部 X = 手指侧面"伸出 0.112m、长轴与手指垂直。用户 2026-08-14 把药匙
在架内绕竖轴多转 90°（rotZ -90°→-180°，勺头扁平面沿 X 为后续旋转铺路），T_held 同步
绕工具 X（勺头方向）多转 90°：手指水平(+X)时药匙竖直挂在手指下、与架内新姿态零跳变；
竖直提起过架顶后手腕转成 ORIENT_FLAT（手指朝上），药匙才变水平（勺头朝 +X 远离机械臂）。
行向量约定：T_held 必须先作用在夹爪局部系（右乘 tool_world）。写反成
tool_center · T_held 会把旋转作用到世界系 → 药匙原点被翻到桌面下不可见（夹住瞬间
"消失"，与朝向无关；flametest 只平移不合成旋转所以免疫，2026-08-14 pxr 数值验证）。

药匙持握 = 关碰撞 + transform-op 覆写（药匙是静态碰撞体，逐帧传送会让物理干扰
手指闭合；关掉后与 flametest 铂丝同模式，手指按 grip_target=0.008 闭合、视觉贴杆）。
"""
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_SPATULA, SPAT_GRASP, SPAT_HEAD_DIST,
    POWDER_TOP_Z, POWDER_X, POWDER_Z, SCOOP_INSERT, POUR_TCP, TUBE_XY,
    TUBE_MOUTH_Z, DISH_XY, WASH_GRASP, GRIP_WASHBOT, WASH_SQUEEZE_CLOSED,
    GRIP_TUBE, TUBE_GRASP_TCP, TUBE_ORIG_Z, TUBE_HELD_OFFSET_Z,
    LIQUID_COLOR_NAMES,
)

# 药匙相对夹爪：平移 (0.112,0,0) + 旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）。
# 药匙长轴 = 夹爪局部 X（手指侧面）、与手指垂直 → 手指水平时药匙竖直挂下、与架内姿态
# （用户已转 rotZ -180°）零跳变；手腕转 ORIENT_FLAT 后药匙变水平。
# 平移必须在最后一行（USD 行向量约定）。
_T_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.112, 0.0, 0.0, 1.0)


class D2SWaterSolubilityTask(BaseTask):
    """D2-S 水溶性测试任务：药匙横向夹取 → 舀粉 → 倾倒 + 洗瓶夹取 的持握与效果驱动。

    试管留在架上（不放操作位），本阶段驱动药匙 + 粉末效果 prim + 洗瓶持握
    （S3 夹肚子：attach 时动态锁定 _T_HELD_WASHB 使瓶子随夹爪走、零跳变）。
    """

    TABLE_Z = 0.80

    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array(SPAT_GRASP)
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量
    GRIP_OPEN_THRESH = 0.03                    # 松开阈值（与 flametest 一致）

    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array(WASH_GRASP)
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004   # 夹紧阈值：grip 0.030 + 4mm 裕量
    # 松开阈值 = 0.038（略低于完全开爪 0.04，保证 S5 开爪能真正越过它触发 released），
    # 不用共享 GRIP_OPEN_THRESH(0.03)（2026-08-25 用户「现在有夹住动作但是没有夹起来啊」
    # 根因）：瓶腹半宽 0.032 > 0.03，持握开度 0.030 本就贴着 0.03——闭爪途中 opening 先过
    # attach 阈值 0.034 触发 attached，此时 opening≈0.033 仍 > 0.03 → 下一帧立即判
    # released → 瓶子弹回表位、臂空抬。药匙 attach 阈值 0.012 ≪ 0.03 无此竞态。
    # 洗瓶必须 attach 阈值(0.034) < 松开阈值(0.038) < 完全开爪(0.04)。
    WASH_GRIP_OPEN = 0.038
    # 挤水判定阈值：opening < 0.025 算正在挤（介于持握 0.030 与挤压 0.020 之间，
    # pick 闭合只到 0.030 不误触）。来自 constants.WASH_SQUEEZE_CLOSED。
    WASH_SQUEEZE_CLOSED = WASH_SQUEEZE_CLOSED

    # 粉丘实测 bbox：x 0.5188-0.5542，y 0.0814-0.1288，z 0.8021-0.8141（详见 meta_actions/constants.py）
    TUBE_MOUTH = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_MOUTH_Z])

    # 试管震荡（S6 拿起试管震荡使粉末溶于水，参考 d3l TubeShakePass）：试管 Ø19.2×153mm
    # 立插架近侧左孔 (0.659,0.241)，管底 z=0.806（gen BUILTIN translate）。持握 = 纯平移
    # （只写 translate 保竖立，不写 4x4），管底吊夹爪下方 0.1393m；管内粉/水随管平移。
    TUBE = "/World/TestTube"
    TUBE_ORIG = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z])   # 管底架内竖插位
    TUBE_GRASP = np.array(TUBE_GRASP_TCP)                          # 抓点（管口下 14mm）
    TUBE_GRIP_CLOSED = GRIP_TUBE + 0.004   # 夹紧阈值：grip 0.0096 + 4mm 裕量（同药匙）
    TUBE_SAMPLE_REST = np.array([TUBE_XY[0], TUBE_XY[1], 0.84])    # 管内粉 rest 位（gen BUILTIN）
    TUBE_WATER_REST = np.array([TUBE_XY[0], TUBE_XY[1], 0.855])    # 管内水 rest 位（gen BUILTIN）
    TUBE_SAMPLE_H = 0.012      # 粉末柱高（与 gen TUBE_SAMPLE_H 对齐）
    WATER_HEIGHT = 0.035       # 液体柱高（洗瓶注水一次，与 gen TUBE_LIQUID_H 对齐）

    # 效果 prim（初始 invisible，task 动画驱动）
    POWDER_EFFECT = "/World/PowderOnSpoon"
    TUBE_WATER = "/World/TubeWater"    # 溶剂水柱（淡蓝，S4 注水显示；不溶最终态液体）
    # 现象色变体（2026-08-25 用户：终端输入溶解度+液体颜色，颜色=粉末色兼溶解后液体色；
    # headless 运行时改材质不渲染 → gen 预烘焙各色，task 按 cfg.liquid_color 拼路径驱动对应的一个）
    CLOUD_TMPL = "/World/Cloud_{}"     # 浑浊云（带粉末色，震荡升起/停震褪去；三档都先浑浊）
    TUBE_SAMPLE_TMPL = "/World/TubeSample_{}"           # 粉末柱（输入色）
    TUBE_SOLUTION_TMPL = "/World/TubeSolution_{}"       # 溶解液柱（输入色全，可溶最终态）
    TUBE_SOLUTION_LIGHT_TMPL = "/World/TubeSolutionLight_{}"   # 溶解液柱（输入色浅，微溶最终态）
    # 挤水水流（S4）：父 WaterStream + N 颗小水滴球沿抛物线从红嘴坠入试管口（用户
    # 「水流太粗/草率就是一个圆柱体」「从红嘴出来一个抛物线落进试管口」
    #  「x坐标再增大1cm」「要有水流感觉」）。task 挤水时逐颗错帧发射。
    WATER_STREAM = "/World/WaterStream"
    WATER_DROPS = 16           # 水滴池大小（与 gen_d2s_scene 的 WATER_DROPS 对齐）
    WATER_STAGGER = 2          # 相邻水滴发射间隔帧（错成连续水流）
    WATER_FALL = 12            # 每滴沿抛物线坠落帧数（重力加速视觉）
    WATER_START = np.array([0.649, 0.231, 0.994])   # 红嘴尖（水平射出起点），终点用 TUBE_MOUTH
    # 药粉下落动画（仿 D2L/D3L DropperDrop：父 PowderDrop + N 颗粉粒 + 队列错帧起落）
    POWDER_DROP = "/World/PowderDrop"
    POWDER_DROPS = 14          # 粉粒数（连续细粉流观感，与 gen_d2s_scene 的 POWDER_DROPS 对齐）
    POWDER_STAGGER = 3         # 相邻粉粒起落间隔帧（错落成细流）
    POWDER_HANG = 4            # 每粒在勺尖悬停成形帧
    POWDER_FALL = 14           # 每粒加速坠落帧数（~0.12m，重力加速视觉）
    POWDER_LAND_Z = 0.84       # 落定 z：管内样品位（TubeSample 中心）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.spatula_path = self.SPATULA_PATH
        # 药匙是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理
        # 干扰），与 flametest 铂丝/滴管同模式。
        self._disable_collision(self.spatula_path)

        # 洗瓶同模式（S3 夹肚子）：静态碰撞体，持握期关碰撞；attach 时动态锁定
        # _T_HELD_WASHB（= 静止矩阵 · tool_world^-1），瓶子保持静止朝向、随夹爪平移。
        self.washbottle_path = self.WASH_PATH
        self._disable_collision(self.washbottle_path)
        self._wb_near_frames = 0
        self.washbottle_state = "rest"   # rest / attached / released
        self._T_HELD_WASHB = None

        self.GRASP_NEAR_FRAMES = 3
        self._near_frames = 0
        self.spatula_state = "rest"     # rest / attached / released
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False     # 倒粉下落动画进行中
        self._powder_queue = []         # 下落动画队列（delay/t/hang/fall/start/target）
        self._prev_flange = None        # 上一帧法兰角（joint7，索引 6），用于判定⑨挖粉旋转开始
        self.squeezing = False          # 挤水进行中（持续发射水滴）
        self.water_in_tube = False      # 已挤入水（管内水显示，只触发一次）
        self._water_queue = []          # 在飞水滴队列（prim/t）
        self._water_next_prim = 0       # 下一颗水滴用哪个 Drop_i（round-robin 复用池）
        self._water_spawn = 0           # 距下次发射水滴的倒计时帧

        # 试管震荡（S6）：静态碰撞体持握期关碰撞（同药匙/洗瓶）；纯平移持握保竖立。
        self._disable_collision(self.TUBE)
        self.tube_state = "rest"        # rest / attached / released
        self._tube_near_frames = 0
        self._tube_pos = self.TUBE_ORIG.copy()   # 试管底当前世界位（现象渲染定位用）

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

        # 现象参数（2026-08-25 用户：终端输入溶解度+液体颜色）——main.py 已按
        # experiment_result schema 把 CLI/交互结果写回 cfg.solubility / cfg.liquid_color。
        self.solubility = getattr(cfg, "solubility", "soluble")
        if self.solubility not in ("soluble", "insoluble", "slightly_soluble"):
            self.solubility = "soluble"
        self.liquid_color = getattr(cfg, "liquid_color", "white")
        if self.liquid_color not in LIQUID_COLOR_NAMES:
            self.liquid_color = "white"
        # 预烘焙色变体路径（粉末/溶解液/浅溶解液/浑浊云 = 输入色）
        self.TUBE_SAMPLE = self.TUBE_SAMPLE_TMPL.format(self.liquid_color)
        self.TUBE_SOLUTION = self.TUBE_SOLUTION_TMPL.format(self.liquid_color)
        self.TUBE_SOLUTION_LIGHT = self.TUBE_SOLUTION_LIGHT_TMPL.format(self.liquid_color)
        self.CLOUD = self.CLOUD_TMPL.format(self.liquid_color)
        # 浑浊→分化状态机
        self._cloud_frac = 0.0       # 浑浊云高度占比 0..1（震荡升起）
        self._settle_frac = 0.0      # 停震分化进度 0..1（澄清/沉淀/变浅）
        self._settling = False       # 已进入分化（浑浊起够 ≥0.5 才允许）
        self._prev_gripper = None    # 震荡判定：上一帧夹爪位
        self._shake_stop_frames = 0  # 连续"静止"帧数（≥SHAKE_STILL_FRAMES 判停震）
        self._seen_oscillation = False  # 高位区是否已观察到真正水平位移（防提起竖直帧提前冒浑浊云）

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.spatula_state = "rest"
        self._near_frames = 0
        self.powder_on_spoon = False
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._prev_flange = None
        self._set_spatula_world(_rest_matrix())
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self._set_washbottle_world(_washbottle_rest_matrix())
        self.squeezing = False
        self.water_in_tube = False
        self._water_queue = []
        self._water_next_prim = 0
        self._water_spawn = 0
        self._set_visibility(self.POWDER_EFFECT, False)
        self._set_visibility(self.TUBE_SAMPLE, False)
        self._set_visibility(self.TUBE_WATER, False)
        self._set_visibility(self.WATER_STREAM, False)
        for i in range(self.WATER_DROPS):
            self._set_visibility(f"{self.WATER_STREAM}/Drop_{i}", False)
        self._set_visibility(self.POWDER_DROP, False)
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        # 还原粉堆尺寸：上一集 _shrink_powder_blob 缩到 12%，不还原下集⑨挖粉粉堆显示成小点
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005)
            cyl.GetHeightAttr().Set(0.005)
        # 试管震荡复位：回架 + 管内粉/水回 rest 位 + 尺寸还原 + 现象状态清零
        self.tube_state = "rest"
        self._tube_near_frames = 0
        self._tube_pos = self.TUBE_ORIG.copy()
        self._cloud_frac = 0.0
        self._settle_frac = 0.0
        self._settling = False
        self._prev_gripper = None
        self._shake_stop_frames = 0
        self._seen_oscillation = False
        self._set_tube_world(self.TUBE_ORIG)
        for path in (self.TUBE_SAMPLE, self.TUBE_WATER,
                     self.TUBE_SOLUTION, self.TUBE_SOLUTION_LIGHT, self.CLOUD):
            self._set_visibility(path, False)
        self._set_tube_column(self.TUBE_SAMPLE, self.TUBE_SAMPLE_H, self.TUBE_SAMPLE_REST, r=0.006)
        self._set_tube_column(self.TUBE_WATER, self.WATER_HEIGHT, self.TUBE_WATER_REST)
        self._set_tube_column(self.TUBE_SOLUTION, self.WATER_HEIGHT, self.TUBE_WATER_REST)
        self._set_tube_column(self.TUBE_SOLUTION_LIGHT, self.WATER_HEIGHT, self.TUBE_WATER_REST)
        self._set_tube_column(self.CLOUD, 0.0, np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z]))

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()   # 试管现象震荡判定用（各子方法自取）
        self._update_spatula()
        self._update_washbottle()   # S3 洗瓶持握（rest/attached/released）
        self._update_tube()         # S6 试管震荡持握（rest/attached/released）
        self._step_powder_anim()    # 药粉下落动画独立推进（倒入完成 / 药匙释放后仍收尾）
        self._step_water_anim()     # 挤水水流动画（挤水时发射水滴、松爪后收尾）
        self._step_phenomenon(gripper_pos)   # 试管内现象：先浑浊 → 按溶解度分化
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "washbottle_state": self.washbottle_state,
            "tube_state": self.tube_state,
            "powder_on_spoon": self.powder_on_spoon,
            "poured": self.poured,
        })

    def on_task_complete(self, success):
        print(f"[d2s] episode done success={success} "
              f"spatula={self.spatula_state} washbottle={self.washbottle_state} "
              f"tube={self.tube_state} powder_on_spoon={self.powder_on_spoon} poured={self.poured}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧药匙持握 / 效果
    # ------------------------------------------------------------------
    def _update_spatula(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        # 法兰（joint7，索引 6）是否在旋转：⑨ 挖粉起判定信号。⑥⑦⑧ 保持世界朝向（法兰恒定），
        # ⑤ 法兰旋转但勺尖在架位高位，⑨ 法兰旋转且勺尖在粉丘 → 仅⑨首帧满足全部条件。
        flange_rotating = (self._prev_flange is not None
                           and abs(joints[6] - self._prev_flange) > 0.005)
        self._prev_flange = float(joints[6])

        if self.spatula_state == "rest":
            if self._near_grasp(gripper_pos, self.SPAT_GRASP):
                self._near_frames += 1
            else:
                self._near_frames = 0
            # 夹爪开始合拢且够近：药匙平滑拉向夹爪持握位（消除闪现吸附）
            if self._near_grasp(gripper_pos, self.SPAT_GRASP) and opening < self.gripper_open_threshold:
                self._ease_spatula_to_gripper(gripper_pos)
            if (self._near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.SPAT_GRIP_CLOSED):
                self.spatula_state = "attached"
                self._set_spatula_from_gripper()
                print(f"[d2s] spatula attached (grip={opening:.4f})")

        elif self.spatula_state == "attached":
            self._set_spatula_from_gripper()
            tip = self._spoon_tip_pos(gripper_pos)
            # 粉末：⑨ 法兰开始旋转（挖粉）→ 显示粉末，跟随勺尖；倒入 → 粉末入管
            if not self.powder_on_spoon and self._scoop_starting(tip, flange_rotating):
                self.powder_on_spoon = True
                self._set_visibility(self.POWDER_EFFECT, True)
                print(f"[d2s] powder on spoon (tip={np.round(tip, 3)})")
            if self.powder_on_spoon and not self.poured:
                # 勺上粉堆跟随勺尖（下落动画中也跟，tip 静止；同时 _shrink_powder_blob 缩小）
                self.object_utils.set_object_position(
                    self.POWDER_EFFECT, tip + np.array([0.0, 0.0, 0.003]))
                # 药匙竖直（法兰≈0）且勺尖近管口 → 开始倒粉下落（只触发一次，_powder_queue 驱动）
                if not self.powder_falling and self._vertical_over_mouth(tip, joints):
                    self.powder_falling = True
                    self._start_powder_fall(tip)
            # 松开：回到架内竖插位姿
            if opening > self.gripper_open_threshold:
                self.spatula_state = "released"
                self._set_spatula_world(_rest_matrix())
                self._set_visibility(self.POWDER_EFFECT, False)
                print("[d2s] spatula released to rack")

    # ------------------------------------------------------------------
    # 每帧洗瓶持握（S3 夹肚子）：rest → 近抓点+合拢 → attached（跟随夹爪）→ released（回表位）
    # ------------------------------------------------------------------
    def _update_washbottle(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]

        if self.washbottle_state == "rest":
            if self._near_grasp(gripper_pos, self.WASH_GRASP):
                self._wb_near_frames += 1
            else:
                self._wb_near_frames = 0
            if (self._wb_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.WASH_GRIP_CLOSED):
                self.washbottle_state = "attached"
                # 动态持握变换：抓取时刻瓶子正好在静止位 → 锁 (静止 · tool^-1)，
                # 瓶子保持静止朝向、随夹爪平移，attach 瞬间零跳变。
                self._T_HELD_WASHB = _washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[d2s] washbottle attached (grip={opening:.4f})")

        elif self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            # 挤水（S4）：夹爪从持握 0.030 进一步合到 0.020 挤压瓶身 → 水流显示；
            # 松回 0.030 → 水流结束、管内显水（只触发一次）。
            if not self.water_in_tube:
                if not self.squeezing and opening < self.WASH_SQUEEZE_CLOSED:
                    self.squeezing = True
                    self._water_spawn = self.WATER_STAGGER   # 下一帧立即发射首滴
                    self._set_visibility(self.WATER_STREAM, True)
                    print(f"[d2s] washbottle squeezing (grip={opening:.4f}) water stream")
                elif self.squeezing and opening >= self.WASH_SQUEEZE_CLOSED:
                    # 松爪：停止发射，让在飞水滴由 _step_water_anim 收尾坠落，再显管内水
                    self.squeezing = False
                    self.water_in_tube = True
                    self._set_visibility(self.TUBE_WATER, True)
                    print("[d2s] water in tube")
            if opening > self.WASH_GRIP_OPEN:   # 完全开爪才算松开（见类常量注释）
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(_washbottle_rest_matrix())
                print("[d2s] washbottle released to table")

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 _set_spatula_from_gripper，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        """把洗瓶写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.washbottle_path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    # ------------------------------------------------------------------
    # 药匙位姿
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_spatula_from_gripper(self):
        # 行向量约定：先 _T_HELD（药匙局部→夹爪局部）再 tool_world（局部→世界）。
        # 写反成 tool_world * _T_HELD 会把 R_y(π) 作用到世界系，药匙原点算到
        # (-0.70,0.36,-0.83)（桌面下）→ 夹住瞬间"消失"（已 pxr 数值验证）。
        self._set_spatula_world(_T_HELD * self._tool_world())

    def _set_spatula_world(self, world_matrix):
        """把药匙写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.spatula_path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        # 本版本 USD 无 RemoveXformOp/SetWorldTransform：清空 op 表 + 单 transform op 写矩阵
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _ease_spatula_to_gripper(self, gripper_pos, k=0.18):
        """夹爪合拢期间药匙逐帧平滑移向持握位（消除闪现吸附）。"""
        # 目标 = _T_HELD * tool_world（顺序同 _set_spatula_from_gripper，不能反）。
        # 插值用 _blend_world（平移线性 + 旋转 slerp）：rest(竖插) 与 held(横持)
        # 旋转差 ~90°，逐分量矩阵 lerp 会产生剪切/缩放（药匙看起来变形）。
        target = _T_HELD * self._tool_world()
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.spatula_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_spatula_world(_blend_world(cur, target, k))

    def _spoon_tip_pos(self, gripper_pos):
        """勺尖 = 夹爪 + 0.134 × 夹爪局部 +X（勺头方向）世界方向。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]   # 行向量约定：tool +X = 旋转部分第 1 行 = 勺头方向（新 _T_HELD）
        return np.asarray(gripper_pos, dtype=float) + SPAT_HEAD_DIST * x_dir

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    def _scoop_starting(self, tip, flange_rotating):
        """⑨ 法兰 -45°→-90° 开始旋转（挖粉起）判定：法兰正在旋转 且 勺尖在粉丘附近（松带）。

        排除误触发：⑤ 法兰也旋转但勺尖在架位高位（x=0.6993 不在粉堆 x 带、z≈1.03 高于高度带）；
        ⑥⑦⑧ 法兰保持朝向恒定不旋转（⑦ 勺尖 z 递减、⑧ 平移 z 恒定）；④ 提勺过架顶 x=0.6993 远离。
        只在 ⑨ 旋转首帧触发（勺尖 (0.537,0.106,0.810) 在松带内）→ 粉末随旋转从粉丘带起。"""
        near = (abs(tip[0] - POWDER_X) < 0.04
                and abs(tip[1] - DISH_XY[1]) < 0.08
                and tip[2] < POWDER_TOP_Z + 0.02)
        return flange_rotating and near

    def _vertical_over_mouth(self, tip, joints):
        """勺尖水平近管口且已降到回卷中段高度 → 开始倒粉（⑬ 进行到约一半掉入试管）。

        2026-08-25 改（用户逐字）：「粉末下掉应该是第13步进行到一半的时候开始掉入试管」——
        删掉「法兰≈0」（vertical，用 joints[6]）约束。旧判定要求法兰转满才触发，只在 ⑬
        末段；⑬ 缩短到 14cm 后终点勺尖 (0.647,0.3008,0.9653) 水平距管口 ≈0.061 > near 0.06，
        末段近不满足 → 粉一直不掉。现回卷中段（法兰约 -45°→0°、勺尖半斜）勺尖水平对准管口
        （水平距最小 ≈0.037，视频视角对齐），勺尖降到 管口顶+5cm 内即触发（t≈0.47，动画
        delay/hang 约 7 帧后粉粒肉眼可见落入试管）。
        防误触发：⑤ 法兰转但勺尖在架位、⑨ 勺尖在粉堆（near 失败）；⑪⑫ 勺尖水平距 ≥0.067
        （near 失败）；⑩⑪⑫ 勺尖 z=1.0993 高于带（above 失败）。仅 ⑬ 中段 near+above 齐备 → 触发一次。
        """
        above = tip[2] < TUBE_MOUTH_Z + 0.05                # 勺尖在管口上方 5cm 内（回卷中段 ~47%）
        near = np.linalg.norm(tip[:2] - np.array([TUBE_XY[0], TUBE_XY[1]])) < 0.06
        return above and near

    def _start_powder_fall(self, tip):
        """倒粉起：勺尖正下方生成一串粉粒（PowderDrop 父 Xform 的 Drop_0..N 球），
        delay 错帧起落成连续细粉流、斜向坠入试管落定在管内样品位（仿 D2L/D3L DropperDrop）。"""
        # 先清掉上一集残留的可见粉粒（父隐藏 ≠ 单粒 visibility 复位）
        for i in range(self.POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{i}", False)
        start = tip + np.array([0.0, 0.0, -0.004])   # 勺尖正下方（略离勺面，不穿勺）
        for i in range(self.POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * self.POWDER_STAGGER,     # 错帧起落 → 连续成流
                "t": 0,
                "start": start.copy(),
                "target": np.array([TUBE_XY[0], TUBE_XY[1], self.POWDER_LAND_Z]),  # 落定在管内样品位
                "hang": self.POWDER_HANG, "fall": self.POWDER_FALL,
            })
        self._set_visibility(self.POWDER_DROP, True)
        print(f"[d2s] powder fall started from {np.round(start, 3)}")

    def _step_powder_anim(self):
        """推进下落串：每粒 delay 错帧起落，悬停→加速坠落→落定（隐藏该粒 + 勺上粉堆缩小），
        全部落定 → 勺上粉消失、试管显示样品（poured）。"""
        if not self._powder_queue:
            return
        remaining = []
        landed = self.POWDER_DROPS - len(self._powder_queue)   # 已落定粒数（本帧循环内递增）
        for d in self._powder_queue:
            if d["delay"] > 0:
                d["delay"] -= 1
                remaining.append(d)
                continue
            d["t"] += 1
            if d["t"] <= d["hang"]:
                pos = d["start"]                         # 悬停：看得见粉粒挂在勺尖
            elif d["t"] <= d["hang"] + d["fall"]:
                frac = (d["t"] - d["hang"]) / d["fall"]  # 重力加速（t² 缓入）
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", False)
                landed += 1
                continue
            # 该粒上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visibility(f"{self.POWDER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.POWDER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._powder_queue = remaining
        self._shrink_powder_blob(landed / self.POWDER_DROPS)
        if not remaining:
            self._set_visibility(self.POWDER_DROP, False)
            if self.powder_falling:
                self.powder_falling = False
                if not self.poured:
                    self.poured = True
                    self._set_visibility(self.POWDER_EFFECT, False)
                    self._set_visibility(self.TUBE_SAMPLE, True)
                    print("[d2s] powder poured into tube")

    def _step_water_anim(self):
        """挤水水流：挤水期间每 WATER_STAGGER 帧发射一颗水滴，沿抛物线（x/y 线性、z t²
        重力加速）从红嘴尖坠入试管口中心；松爪后停止发射、让在飞水滴落完再隐藏父节点。

        抛物线（水平初速、只受重力）：x = x0+(x1-x0)·t，z = z0-(z0-z1)·t²（t∈[0,1]），
        起点 WATER_START=红嘴尖 (0.649,0.231,0.994)、终点 TUBE_MOUTH=管口中心 (0.659,0.241,0.9593)。
        水滴池 WATER_DROPS 颗 round-robin 复用：复用周期=池大小×发射间隔 ≫ 单滴坠落帧数，无同 prim 碰撞。
        """
        if self.squeezing:
            self._water_spawn += 1
            if self._water_spawn >= self.WATER_STAGGER:
                self._water_spawn = 0
                idx = self._water_next_prim % self.WATER_DROPS
                self._water_next_prim += 1
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{idx}", True)
                self.object_utils.set_object_position(
                    f"{self.WATER_STREAM}/Drop_{idx}", self.WATER_START.copy())
                self._water_queue.append({"prim": idx, "t": 0})
        if not self._water_queue:
            return
        remaining = []
        for d in self._water_queue:
            d["t"] += 1
            if d["t"] >= self.WATER_FALL:
                self._set_visibility(f"{self.WATER_STREAM}/Drop_{d['prim']}", False)
                continue
            frac = d["t"] / self.WATER_FALL
            x = self.WATER_START[0] + (self.TUBE_MOUTH[0] - self.WATER_START[0]) * frac
            y = self.WATER_START[1] + (self.TUBE_MOUTH[1] - self.WATER_START[1]) * frac
            z = self.WATER_START[2] - (self.WATER_START[2] - self.TUBE_MOUTH[2]) * frac * frac
            self.object_utils.set_object_position(
                f"{self.WATER_STREAM}/Drop_{d['prim']}", np.array([x, y, z]))
            remaining.append(d)
        self._water_queue = remaining
        if not remaining and not self.squeezing:
            self._set_visibility(self.WATER_STREAM, False)

    def _shrink_powder_blob(self, landed_frac):
        """勺上粉堆随下落进度缩小（粉粒落定越多、勺上剩得越少，避免整块粉堆闪现消失）。"""
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(self.POWDER_EFFECT)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(0.005 * remain)
            cyl.GetHeightAttr().Set(0.005 * remain)

    # ------------------------------------------------------------------
    # 试管震荡持握（S6 拿起试管震荡使粉末溶于水，参考 d3l TubeShakePass）
    # ------------------------------------------------------------------
    def _update_tube(self):
        """每帧试管持握：rest → 近抓点+合拢 → attached（跟随 + 粉/水随管平移）→ released（回架）。

        与药匙/洗瓶不同：试管只做纯平移（不旋转），set_object_position 写首 op（translate），
        姿态恒竖立；持握 = TCP + (0,0,TUBE_HELD_OFFSET_Z)（管底吊夹爪下方 0.1393m）。
        """
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        held = np.asarray(gripper_pos, dtype=float) + np.array([0.0, 0.0, TUBE_HELD_OFFSET_Z])

        if self.tube_state == "rest":
            if self._near_grasp(gripper_pos, self.TUBE_GRASP):
                self._tube_near_frames += 1
            else:
                self._tube_near_frames = 0
            if (self._tube_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.TUBE_GRIP_CLOSED):
                self.tube_state = "attached"
                self._set_tube_world(held)
                print(f"[d2s] tube attached (grip={opening:.4f})")
        elif self.tube_state == "attached":
            self._set_tube_world(held)
            self._follow_tube_effects(held)
            if opening > self.gripper_open_threshold:
                self.tube_state = "released"
                self._set_tube_world(self.TUBE_ORIG)
                self._follow_tube_effects(self.TUBE_ORIG)
                print("[d2s] tube released to rack")

    def _set_tube_world(self, pos):
        """把试管写到给定世界位置（只写 translate，保竖立姿态，同 d3l _set_obj_world）。"""
        self._tube_pos = np.asarray(pos, dtype=float)   # 现象渲染定位（底中心）
        self.object_utils.set_object_position(self.TUBE, self._tube_pos)

    def _follow_tube_effects(self, tube_pos):
        """试管被拿起时管内粉/水/溶液/浑浊云随管平移（保持相对管底的偏移 delta）。"""
        delta = np.asarray(tube_pos, dtype=float) - self.TUBE_ORIG
        cloud_rest = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_ORIG_Z])
        for path, rest in ((self.TUBE_SAMPLE, self.TUBE_SAMPLE_REST),
                           (self.TUBE_WATER, self.TUBE_WATER_REST),
                           (self.TUBE_SOLUTION, self.TUBE_WATER_REST),
                           (self.TUBE_SOLUTION_LIGHT, self.TUBE_WATER_REST),
                           (self.CLOUD, cloud_rest)):
            self.object_utils.set_object_position(path, rest + delta)

    # —— 试管内现象（2026-08-25 用户：终端输入溶解度+液体颜色，三档都先浑浊再分化）——
    SHAKE_TOP_Z = 1.07           # 进入震荡判定区的高度（SHAKE_CENTER_TCP z=1.09 往下 2cm）
    SHAKE_STOP_EPS = 0.0005      # 水平单帧位移 < 此值算"静止"（d3l 同款）
    SHAKE_STILL_FRAMES = 20      # 连续静止这么多帧判停震（震荡相邻峰值间仅 2 帧不会累计到）
    CLOUD_RISE_RATE = 0.012      # 震荡时浑浊云升起速率/帧（~83 帧盖满 0.047m 液柱+粉）
    CLOUD_FADE_RATE = 0.02       # 停震后浑浊云褪去速率/帧（浑浊渐沉淀/澄清）
    SETTLE_FRAMES = 240          # 停震后分化动画帧数（可溶澄清/不溶沉淀/微溶变浅）

    def _detect_oscillating(self, gripper_pos):
        """当前是否在震荡：试管已抓起 + 夹爪在震荡高度区 + 已出现过真正水平位移（_seen_oscillation）
        + 未连续静止。2026-08-25 用户「刚滴入还没震荡拿起来就是灰色的液体」→ 加 _seen_oscillation 门控：
        提起试管竖直停在顶端那 20 帧也算"静止"，旧判据会提前冒浑浊云；现在必须先观察到水平位移
        （真正的震荡来回）才允许云升起，纯竖直提起/放下不冒云。"""
        if self.tube_state != "attached" or gripper_pos is None:
            self._prev_gripper = None
            self._shake_stop_frames = 0
            self._seen_oscillation = False
            return False
        gp = np.asarray(gripper_pos, dtype=float)
        if gp[2] < self.SHAKE_TOP_Z:      # 高位区外（下降/提起初段）不判震荡
            self._prev_gripper = None
            self._shake_stop_frames = 0
            self._seen_oscillation = False
            return False
        move = 0.0
        if self._prev_gripper is not None:
            move = float(np.linalg.norm(gp[:2] - self._prev_gripper[:2]))
        self._prev_gripper = gp
        if move >= self.SHAKE_STOP_EPS:
            self._seen_oscillation = True
            self._shake_stop_frames = 0
        else:
            self._shake_stop_frames += 1
        return self._seen_oscillation and self._shake_stop_frames < self.SHAKE_STILL_FRAMES

    def _step_phenomenon(self, gripper_pos):
        """三档现象（cfg.solubility + cfg.liquid_color）：
        三档都先「逐渐变浑浊」——震荡中 Cloud 云从管底盖满液柱+粉末（_cloud_frac 升起）；
        停震（夹爪静止且浑浊已起够 ≥0.5）才进入分化 _settle_frac，浑浊云同步褪去：
          soluble           浑浊渐澄清 + 粉末溶尽 + 液体变成输入色（全）
          insoluble         浑浊渐沉淀 + 粉末留管底 + 液体回溶剂水色
          slightly_soluble  浑浊渐沉淀 + 粉末留管底 + 液体渐渐变输入色（浅）
        """
        if not self.water_in_tube:
            return
        oscillating = self._detect_oscillating(gripper_pos)
        if oscillating and self._cloud_frac < 1.0:
            self._cloud_frac = min(1.0, self._cloud_frac + self.CLOUD_RISE_RATE)
        elif not oscillating:
            if self._cloud_frac >= 0.5 or self._settling:
                self._settling = True
                if self._settle_frac < 1.0:
                    self._settle_frac = min(1.0, self._settle_frac + 1.0 / self.SETTLE_FRAMES)
                    self._cloud_frac = max(0.0, self._cloud_frac - self.CLOUD_FADE_RATE)
        self._render_phenomenon()

    def _render_phenomenon(self):
        """按当前 _cloud_frac/_settle_frac 渲染试管内现象（粉末/水/溶解液/浑浊云几何+可见性）。"""
        bx, by, bz = self._tube_pos
        hw = self.WATER_HEIGHT
        hp = self.TUBE_SAMPLE_H
        # 浑浊云：从管底盖满（水+粉），_cloud_frac 驱动
        cloud_h = (hw + hp) * self._cloud_frac
        if cloud_h > 0.0005:
            self._set_tube_column(self.CLOUD, cloud_h, (bx, by, bz + cloud_h / 2))
            self._set_visibility(self.CLOUD, True)
        else:
            self._set_visibility(self.CLOUD, False)
        # 粉末：可溶 → 随分化溶解缩小（溶尽隐藏）；不溶/微溶 → 保持全高留管底
        if self.solubility == "soluble":
            ph = hp * (1.0 - self._settle_frac)
            pr = 0.006 * (1.0 - self._settle_frac)
        else:
            ph, pr = hp, 0.006
        if ph > 0.0005:
            self._set_tube_column(self.TUBE_SAMPLE, ph, (bx, by, bz + ph / 2), r=pr)
            self._set_visibility(self.TUBE_SAMPLE, True)
        else:
            self._set_visibility(self.TUBE_SAMPLE, False)
        # 液体：
        if self.solubility == "insoluble":
            # 不溶：浑浊沉淀后液体回溶剂水色（水柱恒显，粉末留底）
            self._set_tube_column(self.TUBE_WATER, hw, (bx, by, bz + hw / 2))
            self._set_visibility(self.TUBE_WATER, True)
            self._set_visibility(self.TUBE_SOLUTION, False)
            self._set_visibility(self.TUBE_SOLUTION_LIGHT, False)
        else:
            # 可溶/微溶：分化中溶解液柱从底渐长，完成后水柱隐藏
            sol_path = (self.TUBE_SOLUTION if self.solubility == "soluble"
                        else self.TUBE_SOLUTION_LIGHT)
            sh = hw * self._settle_frac
            if sh > 0.0005:
                self._set_tube_column(sol_path, sh, (bx, by, bz + sh / 2))
                self._set_visibility(sol_path, True)
            else:
                self._set_visibility(sol_path, False)
            if self._settle_frac >= 1.0:
                self._set_visibility(self.TUBE_WATER, False)
            else:
                self._set_tube_column(self.TUBE_WATER, hw, (bx, by, bz + hw / 2))
                self._set_visibility(self.TUBE_WATER, True)

    def _set_tube_column(self, path, h, center, r=None):
        """设置试管内圆柱效果：高度 + 底部中心位置（r 可选，粉末溶解时同步收细）。"""
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetHeightAttr().Set(h)
            if r is not None:
                cyl.GetRadiusAttr().Set(r)
        self.object_utils.set_object_position(path, np.asarray(center, dtype=float))

    # ------------------------------------------------------------------
    def _disable_collision(self, root):
        prim = self.stage.GetPrimAtPath(root)
        if not prim.IsValid():
            return
        stack = [prim]
        while stack:
            p = stack.pop()
            if UsdPhysics.CollisionAPI(p):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
            for c in p.GetChildren():
                stack.append(c)

    def _set_visibility(self, path, visible):
        try:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass


def _rest_matrix():
    """药匙架内竖插位姿（用户 temp_d2s.usd 2026-08-14 更新坐标+旋转）：与场景 /World/Spatula
    世界矩阵一致 (translate (0.6993,0.3608,0.828)，rotateXYZ(0,0,-180) 烘平后即下行序)。

    重要：Gf.Matrix4d 构造是行主序、USD 变换矩阵平移在最后一行（row-vector）。
    若把平移写在每行第 4 个参数（第 4 列），AddTransformOp 读出的世界平移是
    (0,0,0)——药匙被 reset 到世界原点 = 桌面下 = 不可见（长期报"看不到药匙"根因）。
    """
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.6993, 0.3608, 0.828, 1.0)


def _washbottle_rest_matrix():
    """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵，pxr 实测 2026-08-25）：
    rotateXYZ(0,0,-180) + translate (0.370,0.525,0.80) 烘平后即下行序。
    行 0 = (-1,0,0,0) → 局部 +X 朝世界 -X；行 1 = (0,-1,0,0) → +Y 朝 -Y（嘴尖朝 +X）。
    """
    return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.370, 0.525, 0.80, 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。

    用 Gf.Slerp（pybind 签名 Slerp(t, q1, q2)，t 在前）替换组件级矩阵 lerp；
    已 pxr 验证输出为正交（orth_err≈2e-16）。
    """
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
