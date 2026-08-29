# -*- coding: utf-8 -*-
"""A3 电导率测量任务（v5：夹皿提起 → 移烧杯上方 → 倾斜倒粉 → 放回空皿 → 夹洗瓶移烧杯上方）。

表面皿生命周期：rest → 近抓点+合爪 → attached（皿随夹爪 6-DOF 持握，含旋转——倒粉需倾斜）
→ 开爪 → released（皿+粉回 rest）。洗瓶生命周期：rest → 近抓点+合爪 → attached（纯平移持握）
→ 开爪 → released（回表位）。后续步骤（挤水配液 / 电极浸入 / 读数）逐步追加。

皿持握 = 6-DOF（d2s 药匙同款）：皿世界 = _T_HELD_DISH · tool_center 世界矩阵。_T_HELD_DISH
旋转 = R_y(π)（皿 +z 朝上、tool +z 朝下翻 180°）、平移 = 皿原点在 tool 局部 +z 0.0046m
（皿 prim 原点在皿底；TCP=tool_center 比指端高 0.027，指端 0.825 进天平机身顶 15mm——无碰撞仅
接近时短暂穿入；皿底 0.8474 在指端上方 22.4mm——五改再往下伸 1cm；attach 时皿原点 0.8520−0.0046
= 0.8474 = rest，零跳变）。手腕绕 Y 轴倾斜时皿随 tool 旋转 → 皿 -X 侧下降、粉末沿 -X 滑出。
粉堆 = 程序化圆柱 /World/PowderOnDish（Ø22×6，可 shrink「倒下」），随皿 6-DOF（粉堆中心 = 皿原点
+ 0.0096 在皿局部 +z），皿倾斜时粉堆同倾斜，倒粉时随粉粒落定逐渐缩小。
"""
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    DISH_XY, DISH_ORIG_REST_Z, DISH_GRASP_Z, GRIP_DISH,
    DISH_GRIP_OPEN, DISH_HELD_OFFSET_Z,
    POWDER_PATH, POWDER_ORIG_REST_Z, POWDER_HELD_OFFSET_Z,
    POWDER_BLOB_R, POWDER_BLOB_H,
    POWDER_LAND, BEAKER_POWDER_PATH,
    POWDER_DROPS, POWDER_STAGGER, POWDER_HANG, POWDER_FALL,
    WASH_XY, WASH_GRASP_Z, GRIP_WASHBOT,
)

# 皿相对夹爪的固定变换（6-DOF 持握）：旋转 R_y(π)（皿 +z 朝上 ↔ tool +z 朝下）+ 平移
# (0,0,-DISH_HELD_OFFSET_Z)=(0,0,0.0046)（皿原点在 tool 局部 +z）。行向量约定：先
# _T_HELD_DISH（皿局部→夹爪局部）再 tool_world（局部→世界），写反会把旋转作用到世界系。
# pxr 已验证：朝下时 dish 世界旋转=单位（+z 朝上）、平移=(0,0,-0.0046) 与纯平移一致。
_T_HELD_DISH = Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                            0.0, 1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0, 0.0,
                            0.0, 0.0, -DISH_HELD_OFFSET_Z, 1.0)


class A3ConductivityTask(BaseTask):
    """A3 电导率测量任务（v3：夹皿提起 → 移烧杯上方 → 倾斜倒粉）。"""

    TABLE_Z = 0.80
    GRASP_NEAR_FRAMES = 3
    # 倒粉判定：tool+Z（手指方向）从朝下 (0,0,-1) 转朝 +X 斜下，x 分量超过此阈值判已倾斜
    TILT_X_THRESH = 0.5

    DISH = "/World/SurfaceDish"
    DISH_ORIG = np.array([DISH_XY[0], DISH_XY[1], DISH_ORIG_REST_Z])   # 皿 prim 原点 rest
    DISH_GRASP = np.array([DISH_XY[0], DISH_XY[1], DISH_GRASP_Z])      # 抓点（tool_center，指端 0.825 进机身顶 15mm）
    DISH_GRIP_CLOSED = GRIP_DISH + 0.004                                # 夹紧阈值 0.031
    DISH_HELD_OFFSET = np.array([0.0, 0.0, DISH_HELD_OFFSET_Z])         # 皿原点 = TCP + 偏移（纯平移参考）
    POWDER_ORIG = np.array([DISH_XY[0], DISH_XY[1], POWDER_ORIG_REST_Z])
    POWDER_OFFSET = np.array([0.0, 0.0, POWDER_HELD_OFFSET_Z])          # 粉原点 = 皿原点 + 偏移（皿局部 +z）

    WASH_PATH = "/World/WashBottle"
    WASH_GRASP = np.array([WASH_XY[0], WASH_XY[1], WASH_GRASP_Z])       # 抓点（tool_center，瓶身中部）
    WASH_GRIP_CLOSED = GRIP_WASHBOT + 0.004                             # 夹紧阈值：grip 0.030 + 4mm 裕量
    WASH_GRIP_OPEN = 0.038                                              # 松开阈值（同 d2s；>0.038 才算松开）

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        # 持握期关碰撞（逐帧 transform 传送 + 手指闭合会被物理干扰，d2s 同款）
        self._disable_collision(self.DISH)
        self._disable_collision(POWDER_PATH)
        self.washbottle_path = self.WASH_PATH
        self._disable_collision(self.washbottle_path)

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)
        self.gripper_closed_threshold = getattr(cfg, "gripper_closed_threshold", 0.025)

        self.dish_state = "rest"   # rest / attached / released
        self._dish_near_frames = 0
        self.poured = False            # 已倒粉（粉末落入烧杯）
        self.powder_falling = False    # 粉末下落动画进行中
        self._powder_queue = []        # 下落动画队列

        self.washbottle_state = "rest"   # rest / attached / released
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None

    def reset(self):
        super().reset()
        self.robot.initialize()
        self.dish_state = "rest"
        self._dish_near_frames = 0
        self.poured = False
        self.powder_falling = False
        self._powder_queue = []
        self._set_dish_world(self._dish_rest_matrix())
        self._set_powder_from_dish()
        self.washbottle_state = "rest"
        self._wb_near_frames = 0
        self._T_HELD_WASHB = None
        self._set_washbottle_world(self._washbottle_rest_matrix())
        # 现象复位：皿内粉显、烧杯粉隐、下落父+粉粒隐；粉堆尺寸还原（上集 shrink 缩到 12%）
        prim = self.stage.GetPrimAtPath(POWDER_PATH)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(POWDER_BLOB_R)
            cyl.GetHeightAttr().Set(POWDER_BLOB_H)
        self._set_visibility(POWDER_PATH, True)
        self._set_visibility(BEAKER_POWDER_PATH, False)
        self._set_visibility(self.POWDER_DROP_PATH, False)
        for i in range(POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{i}", False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return None
        opening = joints[7]
        self._update_dish(gripper_pos, opening)
        self._update_washbottle(gripper_pos, opening)
        self._step_powder_anim()
        return self.get_basic_state_info(additional_info={
            "dish_state": self.dish_state,
            "poured": self.poured,
            "washbottle_state": self.washbottle_state,
        })

    def on_task_complete(self, success):
        print(f"[a3] episode done success={success} dish={self.dish_state} poured={self.poured}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 每帧皿持握（6-DOF）：rest → 近抓点+合拢 → attached（皿随 tool 6-DOF、粉堆随皿）
    #   → 倾斜倒粉（tool+Z 朝 +X 斜下）→ 松开（>0.038）→ released（皿+粉回 rest）
    # ------------------------------------------------------------------
    def _update_dish(self, gripper_pos, opening):
        if self.dish_state == "rest":
            near = self._near_grasp(gripper_pos, self.DISH_GRASP)
            self._dish_near_frames = self._dish_near_frames + 1 if near else 0
            if near and opening < self.gripper_open_threshold:
                self._ease_dish_to_gripper()
            if (near and self._dish_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.DISH_GRIP_CLOSED):
                self.dish_state = "attached"
                self._set_dish_from_gripper()
                print(f"[a3] dish attached (grip={opening:.4f})")
            return

        if self.dish_state == "attached":
            self._set_dish_from_gripper()
            # 倒粉：皿倾斜到位（tool+Z 朝 +X 斜下）且尚未倒 → 触发粉末下落（只一次）
            if not self.poured and not self.powder_falling and self._is_tilted():
                self.powder_falling = True
                self._start_powder_fall()
            if opening > DISH_GRIP_OPEN:   # 0.038：皿刚性触壁 opening≈0.030 不会误判松开
                self.dish_state = "released"
                self._set_dish_world(self._dish_rest_matrix())
                self._set_powder_from_dish()
                print(f"[a3] dish released to balance (grip={opening:.4f})")
        # released：已回 rest，不再跟随

    def _ease_dish_to_gripper(self, k=0.18):
        """夹爪合拢期间皿逐帧平滑拉向持握位（6-DOF：平移线性 + 旋转 slerp）。"""
        cur = self._dish_world_matrix()
        target = _T_HELD_DISH * self._tool_world()
        self._set_dish_world(self._blend_world(cur, target, k))
        self._set_powder_from_dish()

    def _set_dish_from_gripper(self):
        """皿跟随夹爪 6-DOF（皿世界 = _T_HELD_DISH · tool_world），粉堆随皿。"""
        self._set_dish_world(_T_HELD_DISH * self._tool_world())
        self._set_powder_from_dish()

    def _set_powder_from_dish(self):
        """粉堆随皿 6-DOF：粉世界 = 皿世界 · 平移(0,0,POWDER_HELD_OFFSET_Z)（皿局部 +z）。"""
        dish_world = self._dish_world_matrix()
        offset = Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, POWDER_HELD_OFFSET_Z, 1.0)
        self._set_powder_world(dish_world * offset)

    # ------------------------------------------------------------------
    # 每帧洗瓶持握（纯平移，d2s 同款）：rest → 近抓点+合拢 → attached（随夹爪平移）→ released（回表位）
    # ------------------------------------------------------------------
    def _update_washbottle(self, gripper_pos, opening):
        if self.washbottle_state == "rest":
            if self._near_grasp(gripper_pos, self.WASH_GRASP):
                self._wb_near_frames += 1
            else:
                self._wb_near_frames = 0
            if (self._wb_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.WASH_GRIP_CLOSED):
                self.washbottle_state = "attached"
                # 动态持握变换：抓取时刻瓶子正好在静止位 → 锁 (静止 · tool^-1)，
                # 瓶子保持静止朝向、随夹爪平移，attach 瞬间零跳变（d2s 同款）。
                self._T_HELD_WASHB = self._washbottle_rest_matrix() * self._tool_world().GetInverse()
                self._set_washbottle_from_gripper()
                print(f"[a3] washbottle attached (grip={opening:.4f})")

        elif self.washbottle_state == "attached":
            self._set_washbottle_from_gripper()
            if opening > self.WASH_GRIP_OPEN:   # 完全开爪才算松开（>0.038）
                self.washbottle_state = "released"
                self._T_HELD_WASHB = None
                self._set_washbottle_world(self._washbottle_rest_matrix())
                print(f"[a3] washbottle released to table (grip={opening:.4f})")
        # released：已回表位，不再跟随

    def _set_washbottle_from_gripper(self):
        # 行向量约定：先 _T_HELD_WASHB（洗瓶局部→夹爪局部）再 tool_world（局部→世界），
        # 顺序同 _set_dish_from_gripper，不能反（反了旋转作用到世界系 → 瓶子翻走）。
        self._set_washbottle_world(self._T_HELD_WASHB * self._tool_world())

    def _set_washbottle_world(self, world_matrix):
        """把洗瓶写到给定世界位姿（局部 = 父世界逆 · 世界，写单个 transform op）。"""
        self._write_world(self.washbottle_path, world_matrix)

    def _washbottle_rest_matrix(self):
        """洗瓶静止位姿（场景 /World/WashBottle 世界矩阵，pxr 实测 2026-08-29）：
        rotateXYZ(0,0,180) + translate (0.3536,0.3062,0.80) 烘平后即下行序。
        行 0 = (-1,0,0,0) → 局部 +X 朝世界 -X；行 1 = (0,-1,0,0) → +Y 朝 -Y（红嘴尖朝 +X）。
        """
        return Gf.Matrix4d(-1.0, 0.0, 0.0, 0.0,
                           0.0, -1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           WASH_XY[0], WASH_XY[1], 0.80, 1.0)

    # ------------------------------------------------------------------
    # 位姿工具（6-DOF：读 tool_world / 写 world 矩阵到 prim，d2s 同款）
    # ------------------------------------------------------------------
    def _tool_world(self):
        """tool_center 世界 4x4 矩阵（运行时 Franka 在 /World/Franka）。"""
        prim = self.stage.GetPrimAtPath(self.robot.prim_path_str + "/panda_hand/tool_center")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dish_world_matrix(self):
        prim = self.stage.GetPrimAtPath(self.DISH)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _dish_rest_matrix(self):
        """皿静止位姿（秤盘上平放，+z 朝上，无旋转）。"""
        return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0,
                           DISH_XY[0], DISH_XY[1], DISH_ORIG_REST_Z, 1.0)

    def _set_dish_world(self, world_matrix):
        self._write_world(self.DISH, world_matrix)

    def _set_powder_world(self, world_matrix):
        self._write_world(POWDER_PATH, world_matrix)

    def _write_world(self, path, world_matrix):
        """把 prim 写到给定世界位姿（局部 = 世界 · 父世界逆，单 transform op）。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        parent = self.stage.GetPrimAtPath("/World")
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = world_matrix * parent_xf.GetInverse()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local)

    def _blend_world(self, a, b, k):
        """两个世界位姿的刚性插值（平移线性 + 旋转 slerp）。"""
        qa = Gf.Rotation(a.ExtractRotation()).GetQuat()
        qb = Gf.Rotation(b.ExtractRotation()).GetQuat()
        m = Gf.Matrix4d()
        m.SetRotateOnly(Gf.Rotation(Gf.Slerp(float(k), qa, qb)))
        m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - k) + b.ExtractTranslation() * k)
        return m

    # ------------------------------------------------------------------
    # 倒粉判定 + 粉末下落动画（仿 d2s PowderDrop：粉粒从皿低侧滑出坠入烧杯口）
    # ------------------------------------------------------------------
    def _is_tilted(self):
        """皿是否已倾斜到位：tool+Z（手指方向）从朝下转到朝 +X 斜下，x 分量超阈值。"""
        wm = self._tool_world()
        z_dir = np.array([wm[2][0], wm[2][1], wm[2][2]])   # 行向量约定 tool+Z = 第 2 行
        return z_dir[0] > self.TILT_X_THRESH

    def _dish_mouth_low_pos(self):
        """皿倾斜后低侧（-X）口沿世界位置 = 皿中心 + tool+X 方向 × 皿半径（粉末滑出起点）。

        pxr 已验证：TILT_ORIENT 让 tool+Z→(0.866,0,-0.5)，皿上法线 dish+z = -tool+z =
        (-0.866,0,0.5) 朝 -X 斜上 → 皿 -X 侧下降、粉末沿 -X 滑出。tool+x（行 0）=(-0.5,0,-0.866)
        恰好指向皿 -x 低侧（dish+x = -tool+x），故用 tool+x 方向 × 皿半径取低侧口沿。
        """
        wm = self._tool_world()
        x_dir = np.array([wm[0][0], wm[0][1], wm[0][2]])   # tool+X 世界方向（行向量第 0 行）= 皿 -x 低侧
        dish_center = np.array(self._dish_world_matrix().ExtractTranslation())
        # 皿中心 = 皿原点 + 皿 +z（= tool -z）方向 × 0.00335
        z_dir = np.array([wm[2][0], wm[2][1], wm[2][2]])
        center = dish_center + (-z_dir) * 0.00335
        return center + x_dir * 0.03   # 皿半径 30mm

    def _start_powder_fall(self):
        """倒粉起：皿低侧口沿正下方生成一串粉粒，delay 错帧坠入烧杯口（仿 d2s）。"""
        # 先清掉上一集残留的可见粉粒（父隐藏 ≠ 单粒 visibility 复位）
        for i in range(POWDER_DROPS):
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{i}", False)
        start = self._dish_mouth_low_pos() + np.array([0.0, 0.0, -0.005])
        target = np.array(POWDER_LAND, dtype=float)
        for i in range(POWDER_DROPS):
            self._powder_queue.append({
                "idx": i,
                "delay": i * POWDER_STAGGER,
                "t": 0,
                "start": start.copy(),
                "target": target.copy(),
            })
        self._set_visibility(self.POWDER_DROP_PATH, True)   # 父显，单粒才渲染（visibility 与父 AND）
        print(f"[a3] powder fall started from {np.round(start, 3)}")

    def _step_powder_anim(self):
        """推进下落串：delay 错帧 → 悬停 → 加速坠落 → 落定隐藏（皿上粉堆随落定缩小）；
        全部落定 → 烧杯显粉。"""
        if not self._powder_queue:
            return
        remaining = []
        landed = POWDER_DROPS - len(self._powder_queue)   # 已落定粒数（本帧循环内递增）
        for d in self._powder_queue:
            if d["delay"] > 0:
                d["delay"] -= 1
                remaining.append(d)
                continue
            d["t"] += 1
            if d["t"] <= POWDER_HANG:
                pos = d["start"]
            elif d["t"] <= POWDER_HANG + POWDER_FALL:
                frac = (d["t"] - POWDER_HANG) / POWDER_FALL
                pos = d["start"] + (d["target"] - d["start"]) * (frac * frac)
            else:
                self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", False)
                landed += 1
                continue
            self._set_visibility(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", True)
            self.object_utils.set_object_position(f"{self.POWDER_DROP_PATH}/Drop_{d['idx']}", pos)
            remaining.append(d)
        self._powder_queue = remaining
        self._shrink_powder_blob(landed / POWDER_DROPS)
        if not remaining:
            self._set_visibility(self.POWDER_DROP_PATH, False)   # 落完收父隐藏
            if self.powder_falling:
                self.powder_falling = False
                self.poured = True
                # 皿里粉堆隐藏、烧杯内显粉
                self._set_visibility(POWDER_PATH, False)
                self._set_visibility(BEAKER_POWDER_PATH, True)
                print("[a3] powder poured into beaker")

    def _shrink_powder_blob(self, landed_frac):
        """皿上粉堆随下落进度缩小（粉粒落定越多、皿上剩得越少，避免整块粉堆闪现消失）。"""
        if landed_frac <= 0:
            return
        remain = max(0.12, 1.0 - landed_frac)
        prim = self.stage.GetPrimAtPath(POWDER_PATH)
        if prim.IsValid():
            cyl = UsdGeom.Cylinder(prim)
            cyl.GetRadiusAttr().Set(POWDER_BLOB_R * remain)
            cyl.GetHeightAttr().Set(POWDER_BLOB_H * remain)

    POWDER_DROP_PATH = "/World/PowderDrop"

    # ------------------------------------------------------------------
    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

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
            from isaacsim.core.utils.prims import set_prim_visibility
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                set_prim_visibility(prim, visible)
        except Exception:
            pass
