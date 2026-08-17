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
    POWDER_TOP_Z, POWDER_Z, SCOOP_INSERT, POUR_TCP, TUBE_XY,
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
    """D2-S 水溶性测试任务：药匙横向夹取 → 舀粉 → 倾倒 的持握与效果驱动。

    试管留在架上（不放操作位），本阶段只驱动药匙 + 粉末效果 prim。
    """

    TABLE_Z = 0.80

    SPATULA_PATH = "/World/Spatula"
    SPAT_GRASP = np.array(SPAT_GRASP)
    SPAT_GRIP_CLOSED = GRIP_SPATULA + 0.004   # 夹紧阈值：grip 0.008 + 4mm 裕量
    GRIP_OPEN_THRESH = 0.03                    # 松开阈值（与 flametest 一致）

    # 粉丘实测 bbox：x 0.5188-0.5542（2026-08-14 晚皿+粉 -X 移 15cm 后同步），y 0.0166-0.064，z 0.8021-0.8141
    POWDER_BBOX = (0.5188, 0.5542, 0.0166, 0.064, 0.8021, 0.8141)
    TUBE_MOUTH = np.array([TUBE_XY[0], TUBE_XY[1], 0.9593])

    # 效果 prim（初始 invisible，task 动画驱动）
    POWDER_EFFECT = "/World/PowderOnSpoon"
    TUBE_SAMPLE = "/World/TubeSample"
    TUBE_WATER = "/World/TubeWater"

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.spatula_path = self.SPATULA_PATH
        # 药匙是静态碰撞体：持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理
        # 干扰），与 flametest 铂丝/滴管同模式。
        self._disable_collision(self.spatula_path)

        self.GRASP_NEAR_FRAMES = 3
        self._near_frames = 0
        self.spatula_state = "rest"     # rest / attached / released
        self.powder_on_spoon = False
        self.poured = False

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.spatula_state = "rest"
        self._near_frames = 0
        self.powder_on_spoon = False
        self.poured = False
        self._set_spatula_world(_rest_matrix())
        self._set_visibility(self.POWDER_EFFECT, False)
        self._set_visibility(self.TUBE_SAMPLE, False)
        self._set_visibility(self.TUBE_WATER, False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_spatula()
        return self.get_basic_state_info(additional_info={
            "spatula_state": self.spatula_state,
            "powder_on_spoon": self.powder_on_spoon,
            "poured": self.poured,
        })

    def on_task_complete(self, success):
        print(f"[d2s] episode done success={success} "
              f"spatula={self.spatula_state} "
              f"powder_on_spoon={self.powder_on_spoon} poured={self.poured}")
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
            # 粉末：勺尖沉入粉丘 → 显示粉末，跟随勺尖；倒入 → 粉末入管
            if not self.powder_on_spoon and self._spoon_in_powder(tip):
                self.powder_on_spoon = True
                self._set_visibility(self.POWDER_EFFECT, True)
                print(f"[d2s] powder on spoon (tip={np.round(tip, 3)})")
            if self.powder_on_spoon and not self.poured:
                self.object_utils.set_object_position(
                    self.POWDER_EFFECT, tip + np.array([0.0, 0.0, 0.003]))
            if self.powder_on_spoon and not self.poured and self._at_pour(tip):
                self.poured = True
                self._set_visibility(self.POWDER_EFFECT, False)
                self._set_visibility(self.TUBE_SAMPLE, True)
                print("[d2s] poured into tube")
            # 松开：回到架内竖插位姿
            if opening > self.gripper_open_threshold:
                self.spatula_state = "released"
                self._set_spatula_world(_rest_matrix())
                self._set_visibility(self.POWDER_EFFECT, False)
                print("[d2s] spatula released to rack")

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

    def _spoon_in_powder(self, tip):
        x0, x1, y0, y1, z0, z1 = self.POWDER_BBOX
        return (x0 < tip[0] < x1 and y0 < tip[1] < y1
                and z0 < tip[2] < z1)

    def _at_pour(self, tip):
        return np.linalg.norm(tip - self.TUBE_MOUTH) < 0.03

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
