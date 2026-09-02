# -*- coding: utf-8 -*-
"""B3L 水浴加热任务（液体样品加热变色：酒精灯加热烧杯水 → 水浴加热试管内溶液 → 渐变变色）。

B3L 拆自 B3（用户 2026-08-30 指令「写成b3l」「不是挖粉末而是往试管里面滴加溶液（动作参考
d3l），其他动作直接复制」；原固体实验改名为 b3s）：
  - 无温度计、无沸石、无药匙/表面皿/粉丘；试管先立在试管架里（D3L 样品区）。
  - 阶段A 滴加：DropperDripPass 取样滴管（d3l sample_pass 复刻）→ 瓶口挤胶头排空气 → 浸液
    吸液 → 管口挤胶头滴液（一次持握内循环 sample_cycles 遍）→ 放回滴管架。task 液滴动画
    （DropperDrop 串 + 管内液面逐滴长高）。
  - 阶段B 点燃酒精灯（LightFlamePass 火柴点火，火焰立即 reveal）。
  - 阶段A' 拿试管入水浴：PickTubePass 水平横夹试管（ORIENT_FWD）→ 纯平移分段转移（竖直提出
    → 水平横移 → 竖直浸入，不反转不洒液）→ 浸入烧杯水浴后**不松爪**，机械臂保持夹持直到
    加热结束；ReturnTubePass 加热后把试管放回架孔（用户逐字「拿试管加热的时候机械臂不能松手，
    直到加热结束才放回去」）。
  - 加热容器 = 烧杯盛水坐石棉网上（水浴）。试管放回架孔后 → LampMovePass 把酒精灯 +Y 移 20cm
    （参考 B2）→ CapLampPass 盖帽熄灭。

结果现象（用户 2026-08-30 逐字「只变色，要有两个输入最开始的颜色，和加热后变的颜色，变色
过程是渐变」）= 双颜色输入 before_color/liquid_color + **渐变变色**（照 B2 移植 top-down
dye）：水到沸点（试管充分加热）后，变色柱 TubeLiquidColor_<liquid_color> 顶贴液面向下扩散，
加热前色柱同步自顶向下缩（未染段红 + 已染段蓝边界相接，两柱不重叠——避免红柱轮廓从半透明
蓝柱透出成竖线），变色柱粗一圈 COLOR_R > 主液柱 LIQUID_R 盖外层。
**不沸腾**（试管内液体只变色不起泡；烧杯水浴照常沸腾出泡）。

温度模型（同 B2 简化）：T 从 room_temp 按 heat_rate 升温到 boiling_point（水 100.0）。
相态机：
    idle（等滴加完成 + 火柴点燃 + 试管浸入水浴）→ ignited
    → heating（烧杯水按 heat_rate 升温；气泡随温度进度逐个 reveal，机械臂夹着试管不动）
    → boiling（水到沸点，气泡全亮 30 泡；触发变色渐变 _color_transition，保持 boil_dwell 5s）
    → tube_return（ReturnTubePass 试管放回架孔，气泡继续沸腾）
    → move_lamp（LampMovePass 把酒精灯 +Y 移 20cm，移灯期间气泡继续沸腾；灯移走松爪后气泡
      逐个渐熄）→ cap_lamp（CapLampPass 盖帽灭火，帽盖到位停留后完成）→ done。

驱动 prim（b3l_water_bath.usd，由 scripts/gen_b3l_scene.py 生成）：
    /World/flame_outer|flame_inner(±_sphere)   火焰（水滴形，迁 /World 顶层，同 B2/B3S）
    /World/BeakerWater                         烧杯内水柱（水浴，可见 r0.031 h0.060）
    /World/BeakerBubbles/bubble_{0..29}        烧杯水浴气泡（球组 r2mm 亮白，初始隐藏，
                                                环带避烧杯壁，上升动画）
    /World/TestTubeLiquid                       管内清色水柱（before_color=clear 回退，隐藏 h0）
    /World/TubeLiquidBefore_<色> ×4            管内加热前色液柱（before_color≠clear，隐藏 h0）
    /World/TubeLiquidColor_<色> ×4             管内加热后变色柱（liquid_color≠clear，隐藏 h0）
    /World/SolutionLiquid                      样品瓶内溶液（静态可见）
    /World/DropperDrop / Drop_0..3             滴管挤胶头滴落串（隐藏，task 液滴动画驱动）
    /World/Dropper                             取样滴管（立插架中心孔，task 纯平移持握跟随）
    /World/Match                               火柴（抬高 12mm，头朝灯芯）
    /World/AlcoholLamp/cap                     灯帽（灯子 prim，静止位 CAP_REST）
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    TIP_OFFSET, TUBE_XY, BEAKER_XY,
    DROPPER_REST, DROPPER_GRASP, SOLUTION_BOTTLE_XY,
    BOTTLE_MOUTH_Z, TUBE_MOUTH_Z,
    MATCH_XY, MATCH_REST_Z, MATCH_GRASP, MATCH_HELD_OFFSET, MATCH_TIP_OFFSET, WICK,
    CAP_CENTER_DZ, CAP_GRASP, CAP_HELD_OFFSET, CAP_BURNER, CAP_REST,
    CAP_CLOSED_THRESHOLD, CAP_COVER_NEAR, CAP_EXTINGUISH_XY, CAP_EXTINGUISH_Z,
    EFFECT_BEAKER_BUBBLES, EFFECT_TUBE_DROPS, EFFECT_DROPPER_DROP,
    GRIP_TUBE, TUBE_REST_Z, TUBE_GRASP_TCP, TUBE_HELD_X,
    TUBE_IMMERSE_TCP,
    LAMP_XY, LAMP_REST_Z, LAMP_GRASP, LAMP_GRASP_OFFSET, GRIP_LAMP,
    LAMP_CLOSED_THRESHOLD, LAMP_OPEN_THRESHOLD, LAMP_TARGET,
)

# 试管相对夹爪的持握矩阵 _T_HELD_TUBE（2026-08-29 照 B1）：平移 +TUBE_HELD_X 沿 tool-X +
# 旋转（toolX→(0,0,1)、toolY→(0,1,0)、toolZ→(-1,0,0)）。试管长轴 = 夹爪局部 Z（手指端面
# 朝向），与药匙一样水平横夹；ORIENT_FWD（toolZ→+X）组合后试管世界旋转 = 恒等（= 架孔竖插
# 静置旋转）→ 抓点吸附零跳变（B1 pxr 数值验证）。管内液柱随管刚性跟随（_set_tube_world 写
# TUBE + 记录 _tube_bottom 供液柱锚定）。
_T_HELD_TUBE = Gf.Matrix4d(0.0, 0.0, 1.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           -1.0, 0.0, 0.0, 0.0,
                           TUBE_HELD_X, 0.0, 0.0, 1.0)

# 酒精灯相对夹爪的持握矩阵 _LAMP_HELD（阶段F 加热后移灯，2026-08-30 照 B2 抄）。
# 抓灯体宽处（z=0.845）时灯底座正好回原位（0.8002），附着手腕已回正、与场景 R180 世界旋转
# 一致 → 零跳变。行向量：灯世界 = _LAMP_HELD · tool_world。Orient_FWD 下灯保持竖直 R180，
# 只跟夹爪水平平移（xz/朝向不变）。平移写在最后一行（row-vector）。
_LAMP_HELD = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                         0.0, -1.0, 0.0, 0.0,
                         -1.0, 0.0, 0.0, 0.0,
                         LAMP_GRASP_OFFSET, 0.0, 0.0, 1.0)

# 滴管相对夹爪持握矩阵 _T_HELD_DROPPER（d3s/B2 同款水平横夹，2026-08-31 自手指朝下
# HELD_OFFSET 纯平移改）：旋转（toolX→(0,0,-1)、toolY→(0,-1,0)、toolZ→(-1,0,0)）+ 平移
# 沿 tool+X 伸 TIP_OFFSET=0.13。手指朝前 ORIENT_FWD（tool+X=世界 -Z）→ 滴管竖直挂夹爪下、
# 尖嘴朝下（尖嘴底=原点，与旧 HELD_OFFSET 尖嘴位置同，仅手腕朝向改 ORIENT_FWD 扩可达域
# 0.75→0.85m）。合成 = _T_HELD_DROPPER · tool_world（右乘=作用局部系，同 _T_HELD_TUBE）。
# 滴管对称，ORIENT_FWD 组合后世界旋转 = rotZ(180°)（视觉不可见）→ 架内吸附零跳变。
_T_HELD_DROPPER = Gf.Matrix4d(0.0, 0.0, -1.0, 0.0,
                              0.0, -1.0, 0.0, 0.0,
                              -1.0, 0.0, 0.0, 0.0,
                              TIP_OFFSET, 0.0, 0.0, 1.0)


class _DropperLifecycle:
    """取样滴管状态机（rest/attached/squeezed/filled/dropped/released，d3l 复刻）。

    持握 = 矩阵 _T_HELD_DROPPER（d3s/B2 同款水平横夹）：手指朝前 ORIENT_FWD 水平横夹竖直
    管身，滴管随夹爪旋转、尖嘴 0.13m 吊在夹爪下方。ORIENT_FWD 组合后滴管世界旋转 =
    rotZ(180°)（滴管对称、视觉不可见）→ 架内吸附零跳变。判定纯关节+TCP，无碰撞依赖。
    参考点（gripper/TCP 世界坐标；瓶口/试管口判定用尖嘴 tip 而非 TCP，同 d3s）：
      grasp        架内立放抓点（夹爪 z = 立放位 + TIP_OFFSET）
      bottle_xy    样品瓶口 xy（排空气/浸液区，z 不区分——瓶口挤与浸液都在同区）
      tube_xy      试管口 xy（滴液区）
    """

    def __init__(self, task, name, path, orig, grasp, bottle_xy, tube_xy,
                 fill_path=None):
        self.task = task
        self.name = name
        self.path = path
        self.orig = np.array(orig)
        self.grasp = np.array(grasp)
        self.bottle_xy = np.array(bottle_xy)
        self.tube_xy = np.array(tube_xy)
        self.fill_path = fill_path
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.squeezed = False
        self.filled = False
        self.dropped = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = self.squeezed = self.filled = self.dropped = self.released = False
        self.task._set_obj_world_matrix(self.path, _dropper_rest_matrix())
        if self.fill_path:
            self.task._set_visibility(self.fill_path, False)

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._dropper_held_matrix()
            # 夹爪开始合拢且已进近窗：先把滴管平滑拉向持握位（d2s 同款全矩阵 ease；
            # 附着手腕已回正→旋转差≈0，只 eases 平移，消除闭合瞬间闪现吸附）。
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_dropper_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_obj_world_matrix(self.path, held)
                print(f"[b3l] {self.name} attached (grip={opening:.4f})")
            return

        # 吸附期：逐帧跟随夹爪（矩阵持握：滴管随夹爪旋转，全程 ORIENT_FWD 竖直挂下）
        self.task._set_obj_world_matrix(self.path, self.task._dropper_held_matrix())
        tip = self.task._dropper_tip_pos()

        # z 守卫（2026-08-31）：瓶口/管口动作要求尖嘴真降到瓶口/管口高度以下，否则 move
        # force-done 卡在高位时 _near_xy（只看 xy）会误触发 squeezed/filled/dropped →
        # 假吸液假滴液（运行 log 已见 filled 假阳性）。挤气尖嘴 0.875 / 浸液 0.83 均
        # < BOTTLE_MOUTH_Z+0.03=0.90；滴液尖嘴 0.984 < TUBE_MOUTH_Z+0.035=0.994。
        at_bottle = self.task._near_xy(self.bottle_xy, tip) and tip[2] < BOTTLE_MOUTH_Z + 0.03
        at_tube = self.task._near_xy(self.tube_xy, tip) and tip[2] < TUBE_MOUTH_Z + 0.035
        if self.state == "attached":
            # 瓶口区挤胶头排空气
            if (opening < self.task.gripper_squeezed_threshold and at_bottle):
                self.state = "squeezed"
                self.squeezed = True
                print(f"[b3l] {self.name} squeezed-air at bottle")
        elif self.state == "squeezed":
            # 瓶口区松胶头吸液
            if (self.task.gripper_squeezed_threshold <= opening < self.task.gripper_closed_threshold
                    and at_bottle):
                self.state = "filled"
                self.filled = True
                print(f"[b3l] {self.name} filled (aspirated)")
        elif self.state == "filled":
            # 试管口区挤胶头滴液
            if (opening < self.task.gripper_squeezed_threshold and at_tube):
                self.state = "dropped"
                self.dropped = True
                self.task._on_drop(self)
                print(f"[b3l] {self.name} dropped into tube")
        elif self.state == "dropped":
            # 一次持握内循环：末遍滴完前回到瓶口再挤胶头 → 再吸再滴（controller 的
            # cycle 未结束，不松开滴管；判定=瓶口区挤胶头，与 attached 首次排空气同）
            if (opening < self.task.gripper_squeezed_threshold and at_bottle):
                self.state = "squeezed"
                print(f"[b3l] {self.name} re-squeeze at bottle (cycle)")
            # 末遍滴完回架松开：写回架内竖插位姿并复位 rest（released 后不再逐帧跟手）
            elif (opening > self.task.gripper_open_threshold
                    and self.task._near(self.grasp, gripper_pos)):
                self.released = True
                self.task._set_obj_world_matrix(self.path, _dropper_rest_matrix())
                self.state = "rest"
                print(f"[b3l] {self.name} released to rack -> rest")


class _MatchLifecycle:
    """单根火柴状态机（rest → attached → released → rest，阶段B 点燃酒精灯）。

    持握 = 纯平移 offset（MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
    （火柴杆横躺，夹爪手指朝下竖直夹其杆身）。释放时写回台面静止位。
    参考点（gripper/TCP 世界坐标）：
      grasp   杆身末尾抓点（MATCH_GRASP）
      rest    火柴原点台面静止位（MATCH_XY + MATCH_REST_Z）
    """

    def __init__(self, task, name, path, rest, grasp):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp = np.array(grasp)
        self.state = "rest"
        self._near_frames = 0
        self._open_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self._open_frames = 0
        self.attached = False
        self.released = False
        self.task._set_match_world(self.task._match_rest_pos())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos, z_thresh=0.03)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_match_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.gripper_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_match_world(held)
                print(f"[b3l] match attached (grip={opening:.4f})")
            return

        # 吸附期：火柴跟随夹爪（纯平移），头 = 夹爪 + MATCH_TIP_OFFSET
        self.task._set_match_world(np.asarray(gripper_pos) + np.array(MATCH_HELD_OFFSET))
        # 松爪（高位）：写回台面静止位，复位 rest。连续 N 帧张开才松（防横移穿越 y=0 奇异点
        # 时夹爪开度瞬态抖动一帧误判松爪 → 火柴瞬移回原位；仿 GRASP_NEAR_FRAMES 连续窗）。
        if opening > self.task.gripper_open_threshold:
            self._open_frames += 1
        else:
            self._open_frames = 0
        if self._open_frames >= self.task.GRASP_NEAR_FRAMES:
            self.released = True
            self.task._set_match_world(self.task._match_rest_pos())
            self.state = "rest"
            print(f"[b3l] match released to rest")


class _CapLifecycle:
    """单支灯帽状态机（rest → attached → settled，阶段F 移灯后盖帽灭火）。

    持握 = 纯平移 offset（CAP_HELD_OFFSET）：帽全程竖直开口朝下，不随夹爪旋转。帽是灯的
    子 prim，吸附期逐帧把帽写到夹爪持握位（帽中心 = 夹爪 + CAP_HELD_OFFSET，经 _set_cap_world
    换算成帽相对灯的 local translate）。盖到位（夹爪近 CAP_BURNER 连续帧）→ settled：
    火焰熄灭、帽锁灯口。B3L 灯先由 LampMovePass 往 +Y 移 20cm，CAP_BURNER 直指移灯后的灯位
    （LAMP_TARGET 正上方 0.900）。
    参考点（gripper/TCP 世界坐标）：
      grasp   帽静止位夹点（CAP_GRASP=CAP_REST 同水平，帽顶下 7mm）
      cover   盖灯口夹爪（CAP_BURNER=0.900，帽中心 0.8917 盖严实）
    """

    def __init__(self, task, name, path, grasp, cover):
        self.task = task
        self.name = name
        self.path = path
        self.grasp = np.array(grasp)
        self.cover = np.array(cover)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.settled = False
        self.extinguish_counter = 0

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.settled = False
        self.extinguish_counter = 0
        self.task._set_cap_rest()

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            near = self.task._near(self.grasp, gripper_pos, z_thresh=0.03)
            self._near_frames = self._near_frames + 1 if near else 0
            held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_cap_world(held)
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.cap_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_cap_world(held)
                print(f"[b3l] cap attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：帽跟随夹爪（纯平移），帽中心 = 夹爪 + CAP_HELD_OFFSET
            held = np.asarray(gripper_pos, dtype=float) + np.array(CAP_HELD_OFFSET)
            self.task._set_cap_world(held)
            # 下落即熄火（B2 同款）：帽底罩过火焰顶才灭，xy 门控防误触
            if (not self.task.flame_extinguished
                    and np.linalg.norm(np.asarray(gripper_pos[:2]) - self.cover[:2]) < CAP_EXTINGUISH_XY
                    and gripper_pos[2] < CAP_EXTINGUISH_Z):
                self.task._extinguish_flame()
            # 盖到位：夹爪近盖灯口位 CAP_BURNER 连续帧 → settled → 火焰熄灭、帽锁灯口
            if np.linalg.norm(np.asarray(gripper_pos) - self.cover) < self.task.cap_cover_near:
                self.extinguish_counter += 1
                if self.extinguish_counter >= self.task.cap_dwell_frames:
                    self.state = "settled"
                    self.settled = True
                    self.task._on_cap_settled(held)
                    print(f"[b3l] cap settled, flame extinguished")
            else:
                self.extinguish_counter = 0
            return

        # settled：帽锁灯口（不再跟随夹爪，帽已停在盖灭位），火焰已熄（_on_cap_settled 处理）


class _TubeLifecycle:
    """试管状态机（rest → attached → immersed → released，阶段A'：滴加后入水浴 → 加热后回架孔）。

    持握 = 矩阵持握（_T_HELD_TUBE）：试管竖直吊在夹爪下（管口朝上、管底吊夹爪下
    TUBE_HELD_X=0.1533，随抓点抬管口顶派生），随夹爪 6-DOF 刚性跟随（照 B1）。管内液柱
    随管刚性跟随（_set_tube_world 写 TUBE + 记录 _tube_bottom，液柱锚定管底）。
    用户 2026-08-30 逐字「拿试管加热的时候机械臂不能松手，直到加热结束才放回去」：
      - immersed（近浸入点连续帧，**无开爪要求**）→ tube_immersed 解除 idle 门控开始加热；
        试管仍跟随夹爪（机械臂不松爪，加热全程保持夹持）。
      - released（ReturnTubePass 回架孔：近架孔抓点 TUBE_GRASP_TCP + 开爪双条件）→ 写
        _tube_rest_matrix()（架孔竖插位，零跳变）+ tube_returned（加热结束才放回 → tube_return 相）。
    参考点（gripper/TCP 世界坐标）：
      grasp   抓点（TUBE_GRASP_TCP=(0.659,0.241,0.9593)，管口顶=低 z IK 死区可达极限最高点，
               2026-08-30 修，near z 窗 ±0.03）
      immerse 浸入点（TUBE_IMMERSE_TCP=(0.5286,-0.25,1.0788)，管底浸水浴 0.9255，不松爪）
    """

    def __init__(self, task, name, path, grasp, immerse):
        self.task = task
        self.name = name
        self.path = path
        self.grasp = np.array(grasp)
        self.immerse = np.array(immerse)
        self.state = "rest"
        self._near_frames = 0
        self._immerse_frames = 0
        self.attached = False
        self.immersed = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self._immerse_frames = 0
        self.attached = False
        self.immersed = False
        self.released = False
        self.task._set_tube_world(_tube_rest_matrix())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            # 2026-08-30：z 窗 ±0.015→±0.03——试管位 (0.659,0.241) 低 z ORIENT_FWD IK 死区，
            # 下探停在管口上方几毫米（近不可达），放宽后机械臂到位即可吸附（TUBE_GRASP_TCP 已抬管口顶）。
            near = self.task._near(self.grasp, gripper_pos, z_thresh=0.03)
            self._near_frames = self._near_frames + 1 if near else 0
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_tube_to_gripper()
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.TUBE_GRIP_CLOSED):
                self.state = "attached"
                self.attached = True
                self.task._set_tube_from_gripper()
                print(f"[b3l] tube attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：试管跟随夹爪（矩阵持握）
            self.task._set_tube_from_gripper()
            # 浸入判定：近浸入点连续帧（无开爪要求，机械臂不松爪）→ immersed（开始加热）
            if self.task._near(self.immerse, gripper_pos):
                self._immerse_frames += 1
                if self._immerse_frames >= self.task.GRASP_NEAR_FRAMES:
                    self.state = "immersed"
                    self.immersed = True
                    self.task._on_tube_immersed()
                    print(f"[b3l] tube immersed in beaker water bath (arm holding)")
            else:
                self._immerse_frames = 0
            return

        if self.state == "immersed":
            # 加热全程：试管**不松爪**，仍跟随夹爪（机械臂保持夹持在浸入位）
            self.task._set_tube_from_gripper()
            # 加热结束 ReturnTubePass 回架孔：近架孔抓点 + 开爪双条件 → released（写回架孔）
            # 2026-08-30：放回下探同样停在管口上方几毫米，z 窗放宽 ±0.03 保释放触发。
            if (self.task._near(self.grasp, gripper_pos, z_thresh=0.03)
                    and opening > self.task.gripper_open_threshold):
                self.state = "released"
                self.released = True
                self.task._on_tube_returned()
                print(f"[b3l] tube returned to rack after heating")
            return


class _LampLifecycle:
    """单支酒精灯状态机（rest → attached → released，阶段F 加热结束后移灯）。

    持握 = 矩阵 _LAMP_HELD · tool_world（水平横夹灯体宽处，ORIENT_FWD 手指朝前，
    d2s 夹药匙同款矩阵持握）：灯随夹爪，垂直姿态保持（xz/朝向不变），只跟夹爪水平平移。
    用户 2026-08-30 逐字「熄灭酒精灯应该先把酒精灯往+y方向移动20cm(参考b2)，然后再盖上灯冒」：
    移灯终点（LAMP_TARGET，+Y 20cm）松爪 → released：灯锁移灯位（不再跟随夹爪），火焰跟随
    灯锁定；task 读 lamp.released → 气泡逐个渐熄 → phase cap_lamp（盖帽）。reset 写回原位。
    参考点（gripper/TCP 世界坐标）：
      grasp   灯体宽处抓点（LAMP_GRASP=(0.5286,-0.25,0.845)，Ø76.8mm 可握）
      rest    灯底座中心静止位（LAMP_XY + LAMP_REST_Z）
      target  移灯终点夹爪（LAMP_TARGET=(0.5286,-0.05,0.845)；松爪判定）
    """

    def __init__(self, task, name, path, rest, grasp, target):
        self.task = task
        self.name = name
        self.path = path
        self.rest = np.array(rest)
        self.grasp = np.array(grasp)
        self.target = np.array(target)
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False

    def reset(self):
        self.state = "rest"
        self._near_frames = 0
        self.attached = False
        self.released = False
        self.task._set_lamp_world(self.task._lamp_rest_matrix())

    def step(self, gripper_pos, opening):
        """每帧推进。gripper_pos = TCP，opening = joint[7]（m）。"""
        if self.state == "rest":
            # 2026-09-01 回退 z 窗 ±0.03→±0.015。此前放宽到 0.03 同步 LampMovePass ④
            # freeze_dist=0.03，但 3cm 即 freeze 让 attach 时 x 停在灯体中心 2.5cm 外 →
            # ⑦ 变两轴单次 IK 在近奇异区漂移 force-done（用户「有动作卡住」）。回 0.015 与
            # ④ freeze_dist=0.010 一致：④ 冻在 1cm 内 → x 误差 <AXIS_EPS(0.015) → ⑦ 纯单轴
            # y linewalk 稳。对齐灯体中心 (0.5286,-0.25,0.845) IK 收敛慢是灯位 rel y=-0.30
            # 近奇异所致（「卡几秒」属物理可达极限，移加热堆叠近底座才能根治）。
            near = self.task._near(self.grasp, gripper_pos, z_thresh=0.015)
            self._near_frames = self._near_frames + 1 if near else 0
            held = self.task._lamp_held_matrix()
            # 夹爪开始合拢且已进近窗：先把灯平滑拉向持握位（消除闭合瞬间闪现吸附）
            if near and opening < self.task.gripper_open_threshold:
                self.task._ease_lamp_world(held)
            # 灯专用 attach 阈值 lamp_closed_threshold（灯体 Ø76.8 宽，合不到常规 0.025）
            if (near and self._near_frames >= self.task.GRASP_NEAR_FRAMES
                    and opening < self.task.lamp_closed_threshold):
                self.state = "attached"
                self.attached = True
                self.task._set_lamp_world(held)
                print(f"[b3l] lamp attached (grip={opening:.4f})")
            return

        if self.state == "attached":
            # 吸附期：灯逐帧跟随夹爪（矩阵持握，灯保持竖直只跟平移）；火焰跟随灯（y=夹爪 y）
            self.task._set_lamp_world(self.task._lamp_held_matrix())
            self.task._set_flame_lamp_y(gripper_pos[1])
            # 松爪判定：夹爪到移灯终点且 grip 真打开（>lamp_open_threshold）→ released。
            # 用灯专用 open 阈值（>GRIP_LAMP 0.038 才真松爪），否则移灯中开度 0.038 已超标
            # 提前 released（灯瞬跳终点，移灯看不全）。
            if (opening > self.task.lamp_open_threshold
                    and self.task._near(self.target, gripper_pos)):
                self.released = True
                self.state = "released"
                self.task._set_lamp_world(self.task._lamp_target_matrix())
                self.task._set_flame_lamp_y(self.target[1])
                print(f"[b3l] lamp released at target (grip={opening:.4f})")
            return

        # released：灯锁移灯位（不再跟随夹爪），火焰锁定不移（夹爪已退走）。
        # 帽是灯子 prim：task 在 move_lamp 相逐帧钉帽在静止位 CAP_REST（不随灯滑走）。


class B3LWaterBathTask(BaseTask):
    """B3L 水浴加热任务：滴管滴加溶液入试管 → 点燃酒精灯 → 拿试管入水浴加热（不松爪）→ 液体渐变变色（只变色不沸腾）→ 试管放回架孔 → 移灯 +Y → 盖帽灭火。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3

    # 水浴几何（pxr 实测 b3l_water_bath.usd 世界包围盒；加热堆叠中心 (0.5286,-0.25)）
    BEAKER_BOTTOM_Z = 0.9205      # 烧杯底（坐石棉网顶）
    WATER_TOP_Z = 0.9805          # 烧杯水面（水浴）
    TUBE_BOTTOM_Z = 0.806         # 试管底（架孔里，= constants.TUBE_REST_Z）

    # 气泡上升动画（照搬 B2/d3l 动态池：连续生成 + 速度差异 + 蛇形 + 破灭复用）。
    # B3L 试管浸入水浴 → 气泡环带只避烧杯壁（中心 BEAKER_XY），见 gen _gen_beaker_bubbles。
    BUBBLE_RISE = 0.0020          # 每帧上升量（m，@60Hz ≈ 0.12m/s；2026-08-30 0.0010→0.0020 沸腾更剧烈）
    BUBBLE_SPAWN_INTERVAL = 1     # 全速生成间隔帧（实际 = 本值 / _bubble_vigor；2026-08-30 4→1 更密集，
                                  #   稳态活跃≈上升寿命/间隔=24/1≈24颗，占满30池，升温期vigor低仍稀疏→渐进沸腾）
    BUBBLE_WOBBLE_AMP = 0.0012    # 上升蛇形摆动振幅（±1.2mm）
    BUBBLE_MIN_RADIUS = 0.012     # 气泡中心离轴最小半径（水柱中心余量）
    BUBBLE_MAX_RADIUS = 0.029     # 气泡中心离轴最大半径（水柱 0.031 − 泡 0.002）
    BUBBLE_SPAWN_Z = BEAKER_BOTTOM_Z + 0.01   # 生成高度（烧杯底上方 10mm，网加热区）
    BUBBLE_R_MIN = 0.0008          # 气泡球体半径：底部生成时 0.8mm（2026-08-30 从下到上逐渐变大）
    BUBBLE_R_MAX = 0.0040          # 气泡球体半径：顶部破灭前 4mm（用户「到最上面的时候不够大」）

    # prim 路径
    DROPPER_PATH = "/World/Dropper"
    MATCH = "/World/Match"
    MATCH_IGNITE_NEAR_FRAMES = 15  # 火柴头近灯芯连续帧数阈值（仿 B2/flametest）
    MATCH_IGNITE_DIST = 0.035      # 火柴头距灯芯 < 3.5cm 判定点火接近
    CAP = "/World/AlcoholLamp/cap"

    # 滴加效果（d3l 复刻）：管内液面逐滴生长（底面贴管底，半径贴 Ø19.2 管壁内缘）
    DROP_LEVEL_STEP = 0.008        # 每滴落定后液面升高 8mm（视觉夸张，真实单滴 <1mm）
    DROP_LEVEL_MAX = 0.100         # 上限 100mm（3 遍 × 4 滴 = 96mm，浸入水浴后液面高出水面
                                   #   0.9805 约 41mm，整管液柱可见，非「只底部红像固体」）
    # 滴落动画（task._step_drop_anim）：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴
    # （滴管内液柱很满，一挤该是一串滴不是一滴——d3l 用户 2026-08-14）。每滴 delay 错帧
    # 起落 → 悬停成形 → 加速坠落，落定才长液面（4 滴/挤=16mm/挤）。
    DROPS_PER_SQUEEZE = 4
    DROP_HANG = 5        # 每滴在尖嘴悬停成形帧数（成串时挂短点整体才连贯）
    DROP_FALL = 16       # 每滴加速坠落帧数（~0.13m，重力加速视觉）
    DROP_STAGGER = 6     # 相邻两滴起落间隔帧数（错落成串）
    # 加热前/后候选色液柱（2026-08-28 用户参考 d3l，B3L 照 B2）：
    #   cfg.before_color → TubeLiquidBefore_<色>（主液柱，滴加逐滴长高）
    #   cfg.liquid_color → TubeLiquidColor_<色>（变色柱，加热时顶贴液面向下扩散，
    #     "clear" 回退到 TestTubeLiquid 水柱，无色时无变色）
    TUBE_LIQUID_BEFORE = "/World/TubeLiquidBefore_{name}"
    TUBE_LIQUID_COLOR = "/World/TubeLiquidColor_{name}"
    COLOR_FADE_RATE = 0.012        # 加热变色每帧渐进度（b2 同款，~83 帧 ≈ 1.4s 换完）
    TUBE_DROPS = EFFECT_TUBE_DROPS
    DROPPER_DROP = EFFECT_DROPPER_DROP

    # 试管转移（阶段A' 滴加后入水浴）：静态碰撞体，吸附期关碰撞；grasp/release 两点位姿
    TUBE_PATH = "/World/TestTube"
    TUBE_GRIP_CLOSED = GRIP_TUBE + 0.005      # 试管 attach 阈值：grip 0.014 + 5mm 裕量 = 0.019。
                                              # 2026-08-30 修：旧 0.0096+4mm=0.0136 只比指令开度宽 4mm，
                                              # 手指贴合管壁时读回略高于阈值即永不吸附；放宽到 0.019
                                              # 手指一开始合拢（<0.03 已 ease）即吸附，不依赖合到精确开度。

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 温度模型参数（config 顶层可调，同 B2）
        self.room_temp = float(getattr(cfg, "room_temp", 25.0))
        self.heat_rate = float(getattr(cfg, "heat_rate", 0.15625))
        self.boiling_point = float(getattr(cfg, "boiling_point", 100.0))
        self.idle_dwell_frames = int(getattr(cfg, "idle_dwell_frames", 20))
        self.ignite_dwell_frames = int(getattr(cfg, "ignite_dwell_frames", 30))
        self.boil_dwell_frames = int(getattr(cfg, "boil_dwell_frames", 300))
        self.bubble_fade_frames = int(getattr(cfg, "bubble_fade_frames", 240))
        # 灯帽：阈值 + 结果停留（同 B2）
        self.cap_closed_threshold = CAP_CLOSED_THRESHOLD     # 帽 attach 阈值（帽 Ø37mm）
        self.cap_dwell_frames = int(getattr(cfg, "cap_dwell_frames", 15))  # 盖到位连续帧
        self.cap_cover_near = CAP_COVER_NEAR                 # 夹爪距 CAP_BURNER 盖到位近窗
        self.cap_result_hold_frames = int(getattr(cfg, "cap_result_hold_frames", 120))
        self._cap_done_frames = 0
        self.flame_extinguished = False
        # 加热前/后液体颜色（用户逐字「要有两个输入最开始的颜色，和加热后变的颜色，变色过程
        # 是渐变」；参数名与渲染方式同 B2）：
        #   before_color=加热前色（滴加逐滴长高）/ liquid_color=加热后色（变色柱）。
        # "clear" 回退到 TestTubeLiquid 水柱（无色水）；liquid_color clear 时无变色柱。
        self.before_color = str(getattr(cfg, "before_color", "clear")).strip().lower()
        self.liquid_color = str(getattr(cfg, "liquid_color", "clear")).strip().lower()
        self._before_path = (self.TUBE_LIQUID_BEFORE.format(name=self.before_color)
                             if self.before_color != "clear" else self.TUBE_DROPS)
        self._color_path = (self.TUBE_LIQUID_COLOR.format(name=self.liquid_color)
                            if self.liquid_color != "clear" else None)   # 同 b2/d3l：clear 无变色柱
        self._color_transition = False   # 加热到沸点锁 true，此后每帧扩散直到换完
        self._color_frac = 0.0

        # 夹爪阈值（同 B2/d3l）
        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_squeezed_threshold = getattr(cfg, "squeeze_close_threshold", 0.005)
        # 2026-08-31 调试（用户「把现在卡住后面的先隐藏，先看前面的对不对」）：true = 相态机停在
        # idle 不推加热/沸腾/放回/移灯/盖帽，只验证前端 S1-S3（滴加+点火+试管浸入水浴）。
        self.debug_s3_only = bool(getattr(cfg, "debug_s3_only", False))

        # 气泡球组（骨架 Sphere，排除 bubble_mat 材质 prim）
        self.bubble_prims = self._children(EFFECT_BEAKER_BUBBLES)
        self.bubble_base = [self._read_translate(p) for p in self.bubble_prims]
        self._bubble_bases = [(float(b[0]), float(b[1])) for b in self.bubble_base]
        self._bubble_z = [self.BUBBLE_SPAWN_Z] * len(self.bubble_prims)
        self._bubble_active = [False] * len(self.bubble_prims)
        self._bubble_age = [0] * len(self.bubble_prims)
        self._bubble_speed = [0.85 + 0.3 * ((i * 37) % 100) / 100.0
                              for i in range(len(self.bubble_prims))]
        self._bubble_phase = [(i * 0.7) % (2.0 * np.pi) for i in range(len(self.bubble_prims))]
        self._bubble_spawn_timer = 0
        self._bubble_vigor = 0.0
        # 火焰 prim（水滴形两焰×2=4，迁 /World 顶层）
        self.flame_prims = self._flame_paths()
        self.flame_base = [self._read_translate(p) for p in self.flame_prims]

        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._bubble_fade = 0
        self._ignited_frame = 0   # 进入 ignited 相时的全局帧号（ignited→heating 相对计时，见 _update_experiment）

        # 滴加（阶段A，d3l 复刻）：滴管静态碰撞体，吸附期关碰撞；生命周期 + 液滴动画状态
        self.sample_cycles = max(1, int(getattr(cfg, "sample_cycles", 3)))
        self._disable_collision(self.DROPPER_PATH)
        self.dropper = _DropperLifecycle(
            self, "dropper", self.DROPPER_PATH, DROPPER_REST, DROPPER_GRASP,
            SOLUTION_BOTTLE_XY, TUBE_XY)
        self._drop_count = 0           # 已生成的液滴总数（每 +DROPS_PER_SQUEEZE）
        self._drop_queue = []          # 滴落动画队列（当前在飞的滴，含 delay/t/hang/fall）
        self._liquid_added = False     # 滴加完成（idle 门控：全 cycles 滴完才允许点火）
        self.solution_added = False    # = self._liquid_added 的镜像（idle 门控）
        self._liquid_level = 0.0       # 当前管内液面高（液柱高度）
        # 试管当前世界位姿 _tube_world（含旋转，供液柱随管刚性跟随——含倾斜）+ 管底世界坐标
        # _tube_bottom（液柱锚定基准：滴加时在架孔 (0.659,0.241,0.806)，浸入水浴/放回时随管
        # ——_set_tube_world 每帧更新）
        self._tube_world = _tube_rest_matrix()
        self._tube_bottom = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_REST_Z])

        # 火柴（阶段B 点燃酒精灯）：静态碰撞体，吸附期关碰撞；rest/grasp 两点位姿
        self._disable_collision(self.MATCH)
        match_rest = (MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z)
        self.match = _MatchLifecycle(self, "match", self.MATCH, match_rest, MATCH_GRASP)
        self.flame_lit = False        # 火柴触灯芯点燃（idle 门控：火焰 reveal）
        self.match_ignite_counter = 0
        # 灯帽（阶段F 盖帽灭火）：帽是灯子 prim（碰撞已随灯 disable）
        self.cap = _CapLifecycle(self, "cap", self.CAP, CAP_GRASP, CAP_BURNER)
        self.cap_rest_translate = self._read_translate(self.CAP)
        # 酒精灯（阶段F 移灯 +Y 20cm）：静态碰撞体，吸附期关碰撞；rest/grasp/target 三点位姿
        self.LAMP = "/World/AlcoholLamp"
        self._disable_collision(self.LAMP)
        self.lamp_closed_threshold = LAMP_CLOSED_THRESHOLD   # 灯专用 attach 阈值（灯体宽）
        self.lamp_open_threshold = LAMP_OPEN_THRESHOLD       # 灯专用 release 阈值（>GRIP_LAMP 才真松爪）
        lamp_rest = (LAMP_XY[0], LAMP_XY[1], LAMP_REST_Z)
        self.lamp = _LampLifecycle(self, "lamp", self.LAMP, lamp_rest, LAMP_GRASP, LAMP_TARGET)
        # 试管（阶段A' 滴加后转移水浴 + 加热后回架孔）：静态碰撞体，吸附期关碰撞
        self.TUBE = self.TUBE_PATH
        self._disable_collision(self.TUBE)
        self.tube = _TubeLifecycle(self, "tube", self.TUBE, TUBE_GRASP_TCP, TUBE_IMMERSE_TCP)
        self.tube_immersed = False   # 试管已浸入水浴（idle 门控：点火后拿试管入水浴）
        self.tube_returned = False   # 试管已放回架孔（加热结束才放回 → tube_return 相）

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.phase = "idle"
        self.temperature = self.room_temp
        self._boil_frames = 0
        self._bubble_fade = 0
        self._ignited_frame = 0
        self._bubble_vigor = 0.0
        self._bubble_spawn_timer = 0
        self._drop_count = 0
        self._drop_queue = []
        self._liquid_added = False
        self.solution_added = False
        self.tube_immersed = False
        self.tube_returned = False
        self.flame_lit = False
        self.flame_extinguished = False
        self._cap_done_frames = 0
        self.match_ignite_counter = 0
        self._liquid_level = 0.0
        self._color_transition = False
        self._color_frac = 0.0
        self._tube_world = _tube_rest_matrix()
        self._tube_bottom = np.array([TUBE_XY[0], TUBE_XY[1], TUBE_REST_Z])
        self._set_visible(self._flame_paths(), False)
        self._set_visible(self.bubble_prims, False)
        for i in range(len(self.bubble_prims)):
            self._bubble_active[i] = False
            self._bubble_z[i] = self.BUBBLE_SPAWN_Z
            self._bubble_age[i] = 0
            self._set_translate(self.bubble_prims[i], self.bubble_base[i])
        # 滴加复位：滴管回架内竖插位 + 液滴/管柱效果隐藏 + 高度归零 + 变色状态清零
        self.dropper.reset()
        self._set_visible(self.DROPPER_DROP, False)
        for i in range(self.DROPS_PER_SQUEEZE):
            self._set_visible(f"{self.DROPPER_DROP}/Drop_{i}", False)
        # 试管复位：回架孔竖插位（+管内液柱随管回位）
        self.tube.reset()
        # 火柴复位：回台面静止位
        self.match.reset()
        # 灯帽复位：回静止位（帽相对灯 local translate 写回）
        self.cap.reset()
        # 酒精灯复位：回原位（移灯前的底座中心静止位）
        self.lamp.reset()
        for p, base in zip(self.flame_prims, self.flame_base):
            self._set_translate(p, base)
        # 液柱复位：全隐藏 + 高度归零（加热前色柱/变色柱 + 清色回退水柱）
        for p in {self._before_path, *((self._color_path,) if self._color_path else ())}:
            self._set_visible(p, False)
            lp = self.stage.GetPrimAtPath(p)
            if lp.IsValid():
                UsdGeom.Cylinder(lp).GetHeightAttr().Set(0.0)
        self._set_visible(self.TUBE_DROPS, False)
        lt = self.stage.GetPrimAtPath(self.TUBE_DROPS)
        if lt.IsValid():
            UsdGeom.Cylinder(lt).GetHeightAttr().Set(0.0)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._step_drop_anim()              # 滴落动画独立推进（与抓取/滴加并行）
        self.dropper.step(gripper_pos, opening)   # 滴管吸液→滴液生命周期
        self.tube.step(gripper_pos, opening)      # 试管转移（rest→attached→immersed→released，入水浴+回架孔）
        self.match.step(gripper_pos, opening)
        self.cap.step(gripper_pos, opening)
        self.lamp.step(gripper_pos, opening)      # 酒精灯移灯（rest→attached→released，+Y 20cm）
        self._step_match_ignite(gripper_pos)      # 点火检测（火柴头触灯芯 → flame_lit + 火焰 reveal）
        self.solution_added = (self._drop_count >= self.sample_cycles * self.DROPS_PER_SQUEEZE)
        self._update_experiment()
        return self.get_basic_state_info(additional_info={
            "phase": self.phase,
            "temperature": round(self.temperature, 1),
            "boiling_point": self.boiling_point,
            "flame_on": self.flame_lit,
            "flame_lit": self.flame_lit,
            "solution_added": self.solution_added,
            "dropper_attached": self.dropper.attached,
            "dropper_filled": self.dropper.filled,
            "dropper_dropped": self.dropper.dropped,
            "tube_immersed": self.tube_immersed,
            "tube_returned": self.tube_returned,
            "match_attached": self.match.attached,
            "cap_attached": self.cap.attached,
            "cap_settled": self.cap.settled,
            "lamp_attached": self.lamp.attached,
            "lamp_released": self.lamp.released,
            "flame_extinguished": self.flame_extinguished,
            "before_color": self.before_color,
            "liquid_color": self.liquid_color,
        })

    # ------------------------------------------------------------------
    # 相态机：idle（等滴加完成+点火+试管浸入水浴）→ ignited → heating → boiling
    #   → tube_return（试管放回架孔）→ move_lamp（灯 +Y 移 20cm）→ cap_lamp（盖帽）→ done
    # ------------------------------------------------------------------
    def _update_experiment(self):
        if self.debug_s3_only:
            return   # 只验证前端 S1-S3：相态机停在 idle，不推加热/沸腾/放回/移灯/盖帽
        if self.phase == "idle":
            # 门控 = 滴加完成 + 火柴点燃酒精灯 + 试管浸入水浴（用户「先点燃酒精灯，然后再拿起
            # 试管过去加热」）。火焰在火柴触灯芯时已 reveal（_step_match_ignite），此处不再亮。
            if self.solution_added and self.tube_immersed and self.flame_lit:
                self.phase = "ignited"
                self._ignited_frame = self.frame_idx
                print(f"[b3l] ignite: solution + tube immersed + match lit @ frame {self.frame_idx}")

        elif self.phase == "ignited":
            # 2026-08-30 修：旧代码用全局 self.frame_idx（此时已数千帧）判定，ignited→heating 在
            # 浸入动作（⑦ mv）仍在下探时就立刻触发 → 加热阶段机械臂还在往下动。改相对计时：
            # 进入 ignited 起 idle_dwell+ignite_dwell 帧后才升温，留足 ⑦ 冻结+dwell 收尾。
            if self.frame_idx - self._ignited_frame >= self.idle_dwell_frames + self.ignite_dwell_frames:
                self.phase = "heating"
                print(f"[b3l] heating start T={self.temperature:.1f}")

        elif self.phase == "heating":
            self.temperature = min(self.boiling_point, self.temperature + self.heat_rate)
            progress = (self.temperature - self.room_temp) / max(
                1e-6, self.boiling_point - self.room_temp)
            self._bubble_vigor = progress
            if progress > 0:
                self._step_bubble_anim()
            if self.temperature >= self.boiling_point:
                self.phase = "boiling"
                # 水到沸点（试管充分加热）→ 触发变色渐变（只变色不沸腾：试管内液体不起泡，
                # 变色柱逐帧盖满）。有目标变色柱才触发扩散（liquid_color clear 跳过）。
                self._color_transition = (self._color_path is not None)
                print(f"[b3l] boiling at T={self.temperature:.1f}"
                      f"{' -> liquid color transition' if self._color_transition else ''}")

        elif self.phase == "boiling":
            self._boil_frames += 1
            self._bubble_vigor = 1.0
            self._step_bubble_anim()
            # 沸腾保持 boil_dwell_frames（config 300 = 5s @60fps）→ 试管放回架孔相（气泡不熄）。
            # 用户「直到加热结束才放回去」：ReturnTubePass 此刻把试管（仍被机械臂夹持）放回架孔。
            if self._boil_frames >= self.boil_dwell_frames:
                self.phase = "tube_return"
                print(f"[b3l] boiling hold done -> return tube to rack (arm still holding)")

        elif self.phase == "tube_return":
            # 机械臂 ReturnTubePass 把试管放回架孔（加热结束才放回）。试管开爪回架孔后
            # tube_returned → 进入移灯相。气泡保持沸腾。
            self._bubble_vigor = 1.0
            self._step_bubble_anim()
            if self.tube_returned:
                self.phase = "move_lamp"
                print(f"[b3l] tube returned to rack -> move lamp phase (lamp +Y 20cm)")

        elif self.phase == "move_lamp":
            # 机械臂 LampMovePass 移灯中（用户「先把酒精灯往+y方向移动20cm(参考b2)，然后再盖上
            # 灯冒」）：灯被夹走 → 火焰跟随灯；移灯期间气泡继续沸腾。松爪（灯移到位 released）
            # 后 → 气泡逐个渐熄（bubble_fade_frames，B2 同款「只有把酒精灯移走之后气泡才慢慢减少
            # 消失」）→ 渐熄完成 → cap_lamp 盖帽。帽是灯子 prim：移灯时逐帧钉在静止位 CAP_REST
            # （不随灯滑走，否则盖帽 IK 卡死；盖帽就在 CAP_REST 夹帽）。
            self._set_cap_world(CAP_REST)
            if self.lamp.released:
                self._bubble_fade += 1
                fade_progress = self._bubble_fade / max(1e-6, self.bubble_fade_frames)
                self._bubble_vigor = max(0.0, 1.0 - fade_progress)
                self._step_bubble_anim()
                if self._bubble_fade >= self.bubble_fade_frames:
                    self._set_visible(self.bubble_prims, False)
                    self.phase = "cap_lamp"
                    print(f"[b3l] bubbles gone -> cap lamp phase (cover moved lamp)")
            else:
                # 灯仍在移灯动作中（未松爪）：气泡保持沸腾
                self._bubble_vigor = 1.0
                self._step_bubble_anim()

        elif self.phase == "cap_lamp":
            # 机械臂 CapLampPass 盖帽中：帽被夹走 → 火焰仍亮（跟随灯）；帽盖到位 settled →
            # 火焰熄灭（_on_cap_settled）。熄火后再停留 cap_result_hold_frames 帧才 done
            # （视频能看到盖好+火灭的结果，否则 done 一触发视频立刻断）。
            if self.cap.settled:
                self._cap_done_frames += 1
                if self._cap_done_frames >= self.cap_result_hold_frames:
                    self.phase = "done"
                    print(f"[b3l] done: cap covers lamp, flame extinguished, "
                          f"before_color={self.before_color} liquid_color={self.liquid_color}")

        # 加热变色 crossfade（未触发时 no-op；触发后每帧变色柱顶贴液面向下扩散）
        self._step_color_transition()

    def _step_bubble_anim(self):
        """气泡动画（照搬 B2/d3l 动态池，改烧杯水浴环带约束）：小球池按「间隔 =
        BUBBLE_SPAWN_INTERVAL / _bubble_vigor」从烧杯底生成一颗、逐帧上升（速度差异 + 蛇形
        摆动）、到水面隐藏（破灭）复用。只写子球 translate；离轴半径钳到 [MIN,MAX] 防穿壁。"""
        pop_z = self.WATER_TOP_Z - 0.002
        bx0, by0 = BEAKER_XY     # 烧杯水柱中心（B3L 试管浸入，气泡只避壁不避试管）
        if self._bubble_vigor > 0:
            if self._bubble_spawn_timer <= 0:
                for i, active in enumerate(self._bubble_active):
                    if not active:
                        self._bubble_active[i] = True
                        self._bubble_z[i] = self.BUBBLE_SPAWN_Z
                        self._bubble_age[i] = 0
                        self._set_visible(self.bubble_prims[i], True)
                        self._set_bubble_radius(i, self.BUBBLE_R_MIN)
                        break
                self._bubble_spawn_timer = max(1, round(self.BUBBLE_SPAWN_INTERVAL / self._bubble_vigor))
            else:
                self._bubble_spawn_timer -= 1
        for i, (bx, by) in enumerate(self._bubble_bases):
            if not self._bubble_active[i]:
                continue
            age = self._bubble_age[i]
            z = self._bubble_z[i] + self.BUBBLE_RISE * self._bubble_speed[i]
            if z >= pop_z:
                self._bubble_active[i] = False
                self._set_visible(self.bubble_prims[i], False)
                continue
            self._bubble_z[i] = z
            ph = self._bubble_phase[i]
            wob = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.15 + ph)
            woy = self.BUBBLE_WOBBLE_AMP * np.sin(age * 0.13 + ph + 1.7)
            cx, cy = bx + wob, by + woy
            dx, dy = cx - bx0, cy - by0
            r = np.hypot(dx, dy)
            if r < self.BUBBLE_MIN_RADIUS:      # 避轴心
                s = self.BUBBLE_MIN_RADIUS / r
                cx, cy = bx0 + dx * s, by0 + dy * s
            elif r > self.BUBBLE_MAX_RADIUS:    # 避烧杯壁（水柱 r0.031）
                s = self.BUBBLE_MAX_RADIUS / r
                cx, cy = bx0 + dx * s, by0 + dy * s
            self._bubble_age[i] = age + 1
            self._set_translate(self.bubble_prims[i], (cx, cy, z))
            # 从下到上逐渐变大：半径随高度线性增长（用户「从下到上逐渐变大，到最上面不够大」）
            prog = (z - self.BUBBLE_SPAWN_Z) / max(1e-6, pop_z - self.BUBBLE_SPAWN_Z)
            self._set_bubble_radius(i, self.BUBBLE_R_MIN
                                    + (self.BUBBLE_R_MAX - self.BUBBLE_R_MIN) * prog)

    # ------------------------------------------------------------------
    # 滴管滴加（阶段A，2026-08-30 复刻 d3l）：rest → attached → squeezed → filled → dropped
    #   → released；液滴动画 DropperDrop 串 + 管内液面逐滴长高
    # ------------------------------------------------------------------
    def _dropper_held_matrix(self):
        """滴管当前持握世界矩阵 = _T_HELD_DROPPER · tool_world（d3s 同款）。"""
        return _T_HELD_DROPPER * self._tool_world()

    def _dropper_tip_pos(self):
        """滴管尖嘴（局部原点）世界坐标 = 物理夹爪 + TIP_OFFSET × tool+X 方向（d3s/B2 同款
        混合数据源；ORIENT_FWD 下 tool+X=世界 -Z → 尖嘴 0.13m 吊夹爪正下方）。"""
        wm = self._tool_world()
        wm_np = np.array([[wm[i][j] for j in range(4)] for i in range(4)])
        x_dir = wm_np[0, :3]
        gripper_pos = self.robot.get_gripper_position()
        return np.asarray(gripper_pos, dtype=float) + TIP_OFFSET * x_dir

    def _ease_dropper_world(self, target, k=0.18):
        """夹爪合拢期间滴管逐帧平滑移向持握位（全矩阵刚性插值，消除闪现吸附）。"""
        cur = self._get_obj_world_matrix(self.DROPPER_PATH)
        self._set_obj_world_matrix(self.DROPPER_PATH, _blend_world(cur, target, k))

    def _on_drop(self, dropper):
        """任一滴加：一次挤胶头成串滴落 DROPS_PER_SQUEEZE 滴（尖嘴下逐滴错落坠落，d3l 同款）。

        挤胶头瞬间在尖嘴正下方生成一串亮色液滴（DropperDrop 父 Xform 的 Drop_0..N 球），
        delay 错帧起落形成连续"滴-滴-滴"。每滴落定后管内液面长高 DROP_LEVEL_STEP（液柱
        锚定试管当前管底 _tube_bottom——滴加时试管在架孔）。"""
        tip = self._dropper_tip_pos()
        start = tip + np.array([0.0, 0.0, -0.005])   # 尖嘴正下方（管口上方 25mm，液滴可见坠落）
        tb = self._tube_bottom
        for i in range(self.DROPS_PER_SQUEEZE):
            m = self._drop_count + i + 1
            level = min(self.DROP_LEVEL_STEP * m, self.DROP_LEVEL_MAX)
            target = np.array([tb[0], tb[1], tb[2] + level - 0.003])  # 落定在管内液面
            self._drop_queue.append({
                "idx": i,
                "delay": i * self.DROP_STAGGER,      # 错帧起落 → 连续成串
                "t": 0,
                "start": start.copy(), "target": target,
                "level": level, "name": dropper.name,
                "hang": self.DROP_HANG, "fall": self.DROP_FALL,
            })
        self._drop_count += self.DROPS_PER_SQUEEZE
        self._set_visible(self.DROPPER_DROP, True)
        print(f"[b3l] squeeze -> {self.DROPS_PER_SQUEEZE} drops spawned")

    def _step_drop_anim(self):
        """推进滴落串：每滴 delay 错帧起落，悬停→加速坠落→落定（隐藏该球+长液面）。"""
        if not self._drop_queue:
            return
        remaining = []
        for d in self._drop_queue:
            if d["delay"] > 0:
                d["delay"] -= 1
                remaining.append(d)
                continue
            d["t"] += 1
            if d["t"] <= d["hang"]:
                pos = d["start"]                         # 悬停：看得见滴挂在尖嘴
            elif d["t"] <= d["hang"] + d["fall"]:
                frac = (d["t"] - d["hang"]) / d["fall"]  # 重力加速（t² 缓入）
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                # 落定：隐藏这颗、长液面，移出队列
                self._set_visible(f"{self.DROPPER_DROP}/Drop_{d['idx']}", False)
                self._grow_tube_level(d["level"], d["name"])
                continue
            # 该滴上场才显示（delay 期间保持隐藏，不在 home 位闪现）
            self._set_visible(f"{self.DROPPER_DROP}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(
                f"{self.DROPPER_DROP}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._drop_queue = remaining
        if not remaining:
            self._set_visible(self.DROPPER_DROP, False)

    def _set_liquid_column_pos(self, path, h, anchor="bottom"):
        """写一根液柱（高度 h），位姿随试管刚性跟随（含倾斜，2026-09-01 修「液体不随管倾斜」）：
        液柱世界矩阵 = 沿试管局部 +Z 偏移 offset 左乘试管世界矩阵 _tube_world（右乘 = 作用局部
        系），柱中心落在管局部 z=offset、柱轴对齐试管轴 → 试管倾斜时液柱跟着倾斜。anchor="bottom"：
        底面贴管底向上长（offset=h/2，滴加长加热前色柱）；anchor="top"：顶贴当前液面向下缩
        （offset=liquid_level-h/2，变色柱顶贴液面向下扩散，b2/d3l 同款）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        UsdGeom.Cylinder(prim).GetHeightAttr().Set(h)
        if anchor == "top":
            offset = self._liquid_level - h / 2.0
        else:
            offset = h / 2.0
        local_offset = Gf.Matrix4d(1.0)
        local_offset.SetTranslate(Gf.Vec3d(0.0, 0.0, offset))
        self._set_obj_world_matrix(path, local_offset * self._tube_world)
        self._set_visible(path, True)

    def _grow_tube_level(self, h, name):
        """液滴落定：管内液面长到高度 h（写加热前色柱，底面贴管底），并记液面高。"""
        self._liquid_level = h
        self._render_liquid_columns()
        print(f"[b3l] tube liquid level h={h:.3f} (before={self.before_color})")

    def _render_liquid_columns(self):
        """渲染管内两液柱（锚定试管当前管底 _tube_bottom）：加热前色柱底面贴管底向上长；
        变色柱顶贴液面、向下扩散。两柱**不重叠**——变色柱自顶向下染、加热前色柱同步自顶向下
        缩（未染段红 + 已染段蓝，边界相接）——否则红柱伸进蓝柱里，红柱两条轮廓竖线从半透明
        蓝柱透出（用户「变蓝后中间两道红色竖线」，2026-09-01 修）。滴加时在架孔、浸入水浴/
        放回时随管走（_set_tube_world 每帧更新 _tube_world/_tube_bottom 后调用）。"""
        # 加热前色柱：未染段（自顶向下缩，随 color_frac 增长而变短）
        h_before = self._liquid_level * (1.0 - self._color_frac)
        if h_before > 0.0005:
            self._set_liquid_column_pos(self._before_path, h_before, anchor="bottom")
        else:
            self._set_visibility(self._before_path, False)
        # 变色柱：已染段（自顶向下扩，随 color_frac 增长而变长）
        if self._color_path is not None and self._color_frac > 0:
            h_color = self._liquid_level * self._color_frac
            if h_color > 0.0005:
                self._set_liquid_column_pos(self._color_path, h_color, anchor="top")
            else:
                self._set_visibility(self._color_path, False)

    def _step_color_transition(self):
        """加热变色（b2 同款 top-down dye）：变色柱 TubeLiquidColor_<色> 顶贴液面、向下扩散，
        加热前色柱同步自顶向下缩（未染段红 + 已染段蓝，边界相接不重叠）——两柱不重叠避免红柱
        轮廓从半透明蓝柱透出。柱体锚定试管当前管底（_render_liquid_columns 随 _set_tube_world
        每帧写）——浸入水浴时在烧杯内试管中变色，放回时变完的颜色留在管里随管回架孔。"""
        if not self._color_transition or not self._color_path:
            return
        self._color_frac = min(1.0, self._color_frac + self.COLOR_FADE_RATE)
        self._render_liquid_columns()
        if self._color_frac >= 1.0:
            self._color_transition = False
            print(f"[b3l] liquid color transition complete: "
                  f"{self.before_color} -> {self.liquid_color}")

    # ------------------------------------------------------------------
    # 试管转移持握（阶段A' 滴加后入水浴，2026-08-29 照 B1 矩阵持握）
    # ------------------------------------------------------------------
    def _set_tube_from_gripper(self):
        # 行向量约定：先 _T_HELD_TUBE（试管局部→夹爪局部）再 tool_world（局部→世界）。
        # ORIENT_FWD 组合旋转 = 恒等 → 抓点吸附零跳变（B1 pxr 数值验证）。
        self._set_tube_world(_T_HELD_TUBE * self._tool_world())

    def _set_tube_world(self, world_matrix):
        """把试管写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op），
        记录试管当前世界位姿 self._tube_world（含旋转，供液柱随管刚性跟随——含倾斜）与管底
        世界坐标 self._tube_bottom（ExtractTranslation），液柱随管刚性跟随（_render_liquid_columns：
        加热前色柱底贴管底、变色柱顶贴液面）→ 浸入水浴时在烧杯内试管中显示，试管放回架孔时
        液体留在管里随管回架孔（不会浮在烧杯水里）。"""
        self._set_obj_world_matrix(self.TUBE, world_matrix)
        self._tube_world = Gf.Matrix4d(world_matrix)
        self._tube_bottom = np.array(world_matrix.ExtractTranslation())
        self._render_liquid_columns()

    def _ease_tube_to_gripper(self, k=0.18):
        """夹爪合拢期间试管逐帧平滑移向持握位（消除闪现吸附，同药匙）。"""
        target = _T_HELD_TUBE * self._tool_world()
        cur = self._get_obj_world_matrix(self.TUBE)
        self._set_tube_world(_blend_world(cur, target, k))

    # ------------------------------------------------------------------
    # 酒精灯移灯持握（阶段F，2026-08-30 照 B2 抄）：灯水平横夹 + 竖直保持只跟平移
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka，同 b2/b3s）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _lamp_held_matrix(self):
        """灯当前持握世界矩阵 = _LAMP_HELD · tool_world（Orient_FWD 下灯保持竖直 R180，
        与场景世界旋转一致 → 附着手腕已回正零跳变；只跟夹爪水平平移）。"""
        return _LAMP_HELD * self._tool_world()

    def _set_lamp_world(self, world_matrix):
        """把灯写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(self.LAMP)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _lamp_rest_matrix(self):
        """灯原位静止位姿（底座中心 LAMP_XY + LAMP_REST_Z，世界旋转 R180 = 场景 EQUIP
        rot180 一致）。平移写在最后一行（行向量），否则 AddTransformOp 读出世界平移 (0,0,0)。"""
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           LAMP_XY[0], LAMP_XY[1], LAMP_REST_Z, 1.0)

    def _lamp_target_matrix(self):
        """灯移灯位静止位姿（底座中心 = LAMP_TARGET 夹爪正下方 LAMP_GRASP_OFFSET，xz 不变、
        y +20cm）。"""
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           LAMP_TARGET[0], LAMP_TARGET[1], LAMP_TARGET[2] - LAMP_GRASP_OFFSET, 1.0)

    def _ease_lamp_world(self, target, k=0.18):
        """夹爪合拢期间灯逐帧平滑移向持握位（消除闪现吸附）。"""
        cur = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.LAMP)) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._set_lamp_world(_blend_world(cur, target, k))

    def _set_flame_lamp_y(self, y):
        """把火焰全部 prim 的 translate y 设为 y（移灯时火焰跟随灯原点 y；x/z 不变）。"""
        for p, base in zip(self.flame_prims, self.flame_base):
            self._set_translate(p, (base[0], y, base[2]))

    def _get_obj_world_matrix(self, path):
        """prim 世界 4x4 矩阵（行主序，行向量约定）；prim 缺失返回恒等。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return Gf.Matrix4d(1.0)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _set_obj_world_matrix(self, path, world_matrix):
        """把 prim 写到给定世界位姿（局部 = 父世界逆 · 世界，清 op 表 + 单 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _on_tube_immersed(self):
        """试管浸入水浴（不松爪，机械臂保持夹持）：置 tube_immersed（解除 idle 门控开始加热）。"""
        self.tube_immersed = True

    def _on_tube_returned(self):
        """加热结束试管放回架孔：写 _tube_rest_matrix()（架孔竖插位，零跳变）+ 置 tube_returned
        （触发 tube_return 相前进 → move_lamp）。"""
        self.tube_returned = True
        self._set_tube_world(_tube_rest_matrix())

    # ------------------------------------------------------------------
    # 火柴纯平移持握（阶段B：火柴横躺水平头朝 +X，只跟夹爪平移不随旋转）
    # ------------------------------------------------------------------
    def _match_rest_pos(self):
        return np.array([MATCH_XY[0], MATCH_XY[1], MATCH_REST_Z])

    def _set_match_world(self, position):
        self._set_obj_world(self.MATCH, position)

    def _ease_match_world(self, target, k=0.18):
        self._ease_obj_world(self.MATCH, target, k)

    def _match_tip(self, gripper_pos):
        """火柴头中心世界坐标 = 夹爪 + MATCH_TIP_OFFSET（头在夹爪 +X 0.0494）。"""
        return np.asarray(gripper_pos, dtype=float) + np.array(MATCH_TIP_OFFSET)

    def _step_match_ignite(self, gripper_pos):
        """点火检测（仿 B2/flametest）：火柴 attached 期间头近灯芯连续帧 → flame_lit=True。

        火焰在点火瞬间立即 reveal（不 gate 在 idle 相）——用户「先点燃酒精灯，然后再拿起试管
        过去加热」：点燃阶段试管还在架孔，火焰须亮在拿试管之前。"""
        if self.flame_lit:
            return
        if self.match.attached:
            tip = self._match_tip(gripper_pos)
            if np.linalg.norm(tip - np.array(WICK)) < self.MATCH_IGNITE_DIST:
                self.match_ignite_counter += 1
                if self.match_ignite_counter >= self.MATCH_IGNITE_NEAR_FRAMES:
                    self.flame_lit = True
                    self._set_visible(self._flame_paths(), True)
                    print(f"[b3l] flame lit by match @ frame {self.frame_idx}")
            else:
                self.match_ignite_counter = 0
        else:
            self.match_ignite_counter = 0

    # ------------------------------------------------------------------
    # 灯帽持握（阶段C：纯平移持握，帽中心 = 夹爪 + CAP_HELD_OFFSET，帽是灯子 prim）
    # ------------------------------------------------------------------
    def _set_cap_translate(self, t):
        """只写帽的 translate op（帽 xform op 序 translate→rotateXYZ→scale，不能动其它 op）。"""
        prim = self.stage.GetPrimAtPath(self.CAP)
        if not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(*t))
                return
        xf.AddTranslateOp().Set(Gf.Vec3d(*t))

    def _set_cap_world(self, center):
        """把帽写到给定帽中心世界坐标（帽 = 灯子 prim，只写帽 translate op，保留 rotateXYZ
        +scale 形状）。换算（pxr 实测，灯 R180Z）：cx = 灯x−tx、cy = 灯y−ty、cz = 灯z+tz+CAP_CENTER_DZ
        → tx = 灯x−cx、ty = 灯y−cy、tz = cz−灯z−CAP_CENTER_DZ。"""
        lamp_pos = self._get_obj_world("/World/AlcoholLamp")
        if lamp_pos is None:
            return
        cx, cy, cz = center
        tx = lamp_pos[0] - cx
        ty = lamp_pos[1] - cy
        tz = cz - lamp_pos[2] - CAP_CENTER_DZ
        self._set_cap_translate((tx, ty, tz))

    def _set_cap_rest(self):
        """帽回静止位（写回读自场景的帽 local translate，帽底贴台面、开口朝下）。"""
        if self.cap_rest_translate is not None:
            self._set_cap_translate(self.cap_rest_translate)

    def _ease_cap_world(self, target, k=0.18):
        """夹爪合拢期间帽逐帧平滑移向持握位（消除闪现吸附）。"""
        cur_origin = self._get_obj_world(self.CAP)
        if cur_origin is None:
            return
        cur_center = np.asarray(cur_origin, dtype=float) + np.array([0.0, 0.0, CAP_CENTER_DZ])
        nxt = cur_center + (np.asarray(target, dtype=float) - cur_center) * k
        self._set_cap_world(nxt)

    def _extinguish_flame(self):
        """熄灭火焰（幂等：下落即熄/盖到位都调，只灭一次）。"""
        if self.flame_extinguished:
            return
        self.flame_extinguished = True
        self._set_visible(self._flame_paths(), False)
        print(f"[b3l] flame extinguished @ frame {self.frame_idx}")

    def _on_cap_settled(self, center):
        """帽盖到位：火焰熄灭、帽锁灯口（settled 态不再跟随夹爪，帽停在盖灭位）。"""
        self._extinguish_flame()

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _flame_paths(self):
        return ["/World/flame_outer", "/World/flame_outer_sphere",
                "/World/flame_inner", "/World/flame_inner_sphere"]

    def _get_obj_world(self, path):
        """物体原点世界坐标；prim 缺失返回 None。"""
        return self.object_utils.get_object_xform_position(path)

    def _set_obj_world(self, path, position):
        prim = self.stage.GetPrimAtPath(path)
        if prim.IsValid():
            self.object_utils.set_object_position(path, np.asarray(position, dtype=float))

    def _ease_obj_world(self, path, target, k=0.18):
        cur = self._get_obj_world(path)
        if cur is None:
            return
        nxt = cur + (target - cur) * k
        self._set_obj_world(path, nxt)

    def _children(self, root):
        prim = self.stage.GetPrimAtPath(root)
        if not prim.IsValid():
            return []
        return [str(c.GetPath()) for c in prim.GetChildren()
                if c.GetTypeName() in ("Sphere", "Cone", "Mesh", "Cylinder")]

    def _read_translate(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return [0.0, 0.0, 0.0]
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                return [float(v[0]), float(v[1]), float(v[2])]
        return [0.0, 0.0, 0.0]

    def _set_translate(self, path, t):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(*t))
        else:
            xf.AddTranslateOp().Set(Gf.Vec3d(*t))

    def _set_bubble_radius(self, i, r):
        """写气泡 i 的球体半径（从下到上逐渐变大：底部生成小、顶部破灭前大）。"""
        prim = self.stage.GetPrimAtPath(self.bubble_prims[i])
        if prim.IsValid():
            UsdGeom.Sphere(prim).GetRadiusAttr().Set(float(r))

    def _set_visible(self, paths, visible):
        if isinstance(paths, str):
            paths = [paths]
        for path in paths:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)

    def _set_visibility(self, path, visible):
        """单路径写法别名（液滴/液柱动画移植代码用 _set_visibility，同 d2s/d3l）。"""
        self._set_visible(path, visible)

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

    def _near(self, pos, gripper_pos, z_thresh=0.015):
        return (np.linalg.norm(gripper_pos[:2] - pos[:2]) < self.grasp_xy_threshold
                and abs(gripper_pos[2] - pos[2]) < z_thresh)

    def _near_xy(self, center_xy, gripper_pos):
        """只比水平 xy 近（滴管瓶口/试管口区判定，z 不区分——挤胶头与浸液/滴液都在同区）。"""
        return np.linalg.norm(gripper_pos[:2] - center_xy) < self.grasp_xy_threshold

    def on_task_complete(self, success):
        print(f"[b3l] episode done success={success} phase={self.phase} "
              f"solution_added={self.solution_added} "
              f"tube_immersed={self.tube_immersed} "
              f"tube_returned={self.tube_returned} "
              f"flame_lit={self.flame_lit} "
              f"lamp_released={self.lamp.released} "
              f"cap_settled={self.cap.settled} "
              f"flame_extinguished={self.flame_extinguished} "
              f"before_color={self.before_color} liquid_color={self.liquid_color} "
              f"temp={self.temperature:.1f}°C")
        super().on_task_complete(success)


def _tube_rest_matrix():
    """试管架孔竖插位姿（恒等旋转 + 平移 TUBE_XY/TUBE_REST_Z）。

    恒等旋转 = 架孔静置旋转 = ORIENT_FWD + _T_HELD_TUBE 组合旋转 → 抓点吸附零跳变
    （B1 pxr 数值验证）。平移写在最后一行（row-vector）。
    """
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       TUBE_XY[0], TUBE_XY[1], TUBE_REST_Z, 1.0)


def _dropper_rest_matrix():
    """滴管架孔竖插静止位姿（恒等旋转 + 平移 DROPPER_REST；滴管对称、尖嘴=原点 DROPPER_REST）。

    恒等旋转 = 架孔静置旋转（ORIENT_FWD + _T_HELD_DROPPER 组合后世界旋转 = rotZ(180°)，
    滴管对称视觉同恒等 → 抓点吸附零跳变）。平移写在最后一行（row-vector），否则
    AddTransformOp 读出的世界平移是 (0,0,0) → 滴管被 reset 到原点=桌面下不可见。
    """
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       DROPPER_REST[0], DROPPER_REST[1], DROPPER_REST[2], 1.0)


def _blend_world(a, b, k):
    """两个世界位姿的刚性插值：平移线性 + 旋转 slerp（避免逐分量矩阵 lerp 剪切）。"""
    qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
    qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
    m = Gf.Matrix4d()
    m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
    m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
    return m
