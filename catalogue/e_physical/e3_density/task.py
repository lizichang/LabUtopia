"""E3「密度测定」任务：移液管平移跟随 + 液柱显色 + 天平屏密度读数。

与 e1 的 set_object_position 平移跟随同构。本实验无手腕翻转，task 只对移液管做平移跟随：
每帧把被持握移液管写到 TCP 相对位：
  移液管尖端 = 抓点 - (0,0,PIPE_TIP_TO_GRIP)（抓点在尖端上方 0.09m）

liquid_color 变色接口（预留）：样品瓶内液柱、移液管内吸液柱、量筒内液柱，各有 6 色
变体初始全隐藏，task 按 cfg.liquid_color 显示对应变体（坑 27：RTX 下改 Shader 不刷新，
须预制变体 + visibility 切换）。
  - 样品瓶液柱：开场即显示（待测液体源）
  - 移液管吸液柱：吸液（尖端入瓶口）后显示
  - 量筒液柱：放液（尖端入筒口）后显示

密度读数（v2 加天平完整测密度 ρ=Δm/5mL）：天平前面板屏贴图（gen_e3_scene.py 预生成）
  BalanceM1（空量筒 35.00g，开场可见）→ 放液后切 BalanceResult（m2+ρ）。density 为
  任意数值（type:number），task 运行时 PIL 烘焙 balance_result.png 覆写场景贴图同
  路径（headless 下运行时改贴图路径不渲染，故覆写同名文件而非换路径）。
"""
import os
import numpy as np
from pxr import UsdPhysics
from isaacsim.core.utils.prims import set_prim_visibility

from tasks.base_task import BaseTask
from .meta_actions.constants import (
    GRIP_PIPETTE,
    PIPE_GRASP, PIPE_TIP_TO_GRIP, PIPE_REST_POS,
    BOTTLE_XY, BOTTLE_MOUTH_Z,
    CYL_XY, CYL_MOUTH_Z,
    M1_GRAMS, TRANSFER_ML, DENSITY_DEFAULT,
)

# 仓库根（task.py 上溯 4 级），用于运行时覆写场景贴图 balance_result.png
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))


class E3DensityTask(BaseTask):
    """E3 密度测定：移液管吸样 → 放液到天平上量筒 → 天平屏读 ρ=Δm/5mL。"""

    PIPE_PATH = "/World/Pipette"

    PIPE_GRASP = np.array(PIPE_GRASP)
    PIPE_GRIP_CLOSED = GRIP_PIPETTE + 0.004     # 夹紧阈值：grip 0.0035 + 4mm 裕量
    GRIP_OPEN_THRESH = 0.03

    # 液柱变体父 prim 路径（gen_e3_scene.py 建的 6 色隐藏变体 liq_<color>）
    LIQ_PATHS = {
        "bottle": "/World/BottleLiquid",
        "pipe": "/World/Pipette",
        "cylinder": "/World/CylinderLiquid",
    }
    LIQ_KEYS = ("colorless", "blue", "red", "green", "yellow", "purple")

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)
        self.pipe_path = self.PIPE_PATH
        # 静态几何：移液管持握期关碰撞（逐帧 transform 覆写 + 手指闭合，避免物理干扰）
        self._disable_collision(self.pipe_path)

        self.GRASP_NEAR_FRAMES = 3
        self._pipe_near_frames = 0
        self.pipe_state = "rest"      # rest / attached / released
        self.drawn = False
        self.dispensed = False

        # liquid_color 变色接口：读 cfg.liquid_color（experiment_result 框架 --result 写回）
        self.liquid_color = getattr(cfg, "liquid_color", "colorless")
        if self.liquid_color not in self.LIQ_KEYS:
            self.liquid_color = "colorless"

        # density 接口：读 cfg.density（experiment_result 框架 --result 写回，type:number
        # 任意数值），驱动天平屏 m2+ρ 读数。运行时用 PIL 烘焙 balance_result.png 覆写
        # 场景贴图（headless 下改贴图路径不渲染，须覆写同名文件）。
        try:
            self.density = float(getattr(cfg, "density", DENSITY_DEFAULT))
        except (TypeError, ValueError):
            self.density = float(DENSITY_DEFAULT)
        self.m2_grams = M1_GRAMS + self.density * TRANSFER_ML
        self._bake_result_texture()

        self.grasp_xy_threshold = getattr(cfg, "grasp_xy_threshold", 0.03)
        self.gripper_open_threshold = getattr(cfg, "gripper_open_threshold", 0.03)

    # ------------------------------------------------------------------
    def reset(self):
        super().reset()
        self.robot.initialize()
        self.pipe_state = "rest"
        self._pipe_near_frames = 0
        self.drawn = False
        self.dispensed = False
        # 移液管回架孔；样品瓶液柱开场可见，移液管/量筒液柱隐藏；天平屏显示空量筒 m1
        self.object_utils.set_object_position(self.PIPE_PATH, np.array(PIPE_REST_POS))
        self._show_liquid("bottle", True)
        self._show_liquid("pipe", False)
        self._show_liquid("cylinder", False)
        self._show_balance(False)

    def step(self):
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None
        self._update_pipette()
        return self.get_basic_state_info(additional_info={
            "pipe_state": self.pipe_state,
            "liquid_color": self.liquid_color,
            "density": self.density,
            "drawn": self.drawn,
            "dispensed": self.dispensed,
        })

    def on_task_complete(self, success):
        print(f"[e3] episode done success={success} pipe={self.pipe_state} "
              f"drawn={self.drawn} dispensed={self.dispensed} "
              f"color={self.liquid_color} density={self.density}")
        super().on_task_complete(success)

    # ------------------------------------------------------------------
    # 移液管持握（rest → attached → released）+ 吸液/放液显色 + 天平屏密度读数
    # ------------------------------------------------------------------
    def _update_pipette(self):
        gripper_pos = self.robot.get_gripper_position()
        joints = self.robot.get_joint_positions()
        if gripper_pos is None or joints is None:
            return
        opening = joints[7]
        tip = np.asarray(gripper_pos, dtype=float) - np.array(
            [0.0, 0.0, PIPE_TIP_TO_GRIP])

        if self.pipe_state == "rest":
            if self._near_grasp(gripper_pos, self.PIPE_GRASP):
                self._pipe_near_frames += 1
            else:
                self._pipe_near_frames = 0
            if (self._pipe_near_frames >= self.GRASP_NEAR_FRAMES
                    and opening < self.PIPE_GRIP_CLOSED):
                self.pipe_state = "attached"
                print(f"[e3] pipette attached (grip={opening:.4f})")

        elif self.pipe_state == "attached":
            # 平移跟随：尖端 = 抓点 - (0,0,PIPE_TIP_TO_GRIP)
            self.object_utils.set_object_position(self.PIPE_PATH, tip)
            # 吸液：尖端入瓶口（下探到液面下）→ 显示移液管吸液柱
            if not self.drawn and self._tip_near(tip, BOTTLE_XY, BOTTLE_MOUTH_Z):
                self.drawn = True
                self._show_liquid("pipe", True)
                print(f"[e3] pipette drawn (color={self.liquid_color})")
            # 放液：尖端入筒口 → 显示量筒液柱 + 隐藏移液管吸液柱 + 天平屏 m1→m2+ρ
            if self.drawn and not self.dispensed and self._tip_near(tip, CYL_XY, CYL_MOUTH_Z):
                self.dispensed = True
                self._show_liquid("pipe", False)
                self._show_liquid("cylinder", True)
                self._show_balance(True)
                print(f"[e3] transferred to cylinder (color={self.liquid_color} "
                      f"density={self.density})")
            if opening > self.gripper_open_threshold:
                self.pipe_state = "released"
                self.object_utils.set_object_position(self.PIPE_PATH, np.array(PIPE_REST_POS))
                print("[e3] pipette released to stand")

    # ------------------------------------------------------------------
    # 判定
    # ------------------------------------------------------------------
    def _tip_near(self, tip, xy, mouth_z):
        """移液管尖端贴近某容器口并降入口内 → 判定到位。

        尖端 xy 在容器中心 2cm 内、且尖端 z 低于口沿（口沿远高于尖端最终下探位，故
        触发在尖端过口沿时，可靠且不依赖冻结裕量；吸液/放液目标位都在口沿下方 >3cm）。
        """
        near_xy = np.linalg.norm(tip[:2] - np.array(xy)) < 0.02
        near_z = tip[2] < mouth_z
        return near_xy and near_z

    def _near_grasp(self, gripper_pos, grasp_pos, xy_thresh=None, z_thresh=0.015):
        if xy_thresh is None:
            xy_thresh = self.grasp_xy_threshold
        return (np.linalg.norm(gripper_pos[:2] - grasp_pos[:2]) < xy_thresh
                and abs(gripper_pos[2] - grasp_pos[2]) < z_thresh)

    # ------------------------------------------------------------------
    def _show_liquid(self, which, visible):
        """显示/隐藏某容器中 liquid_color 对应色的液柱变体（其余色保持隐藏）。"""
        parent = self.LIQ_PATHS[which]
        for key in self.LIQ_KEYS:
            path = f"{parent}/liq_{key}"
            self._set_visibility(path, visible and (key == self.liquid_color))

    def _show_balance(self, show_result):
        """天平屏读数：show_result=False → 空量筒质量 m1；True → m2+ρ（单张动态贴图）。"""
        self._set_visibility("/World/BalanceM1", not show_result)
        self._set_visibility("/World/BalanceResult", show_result)

    def _bake_result_texture(self):
        """运行时用 PIL 烘焙天平屏结果贴图 balance_result.png（m2+ρ），覆写场景同路径
        同名文件（gen_e3_scene.py 预生成 ρ=1.0 占位版，这里按实际 density 覆写）。
        headless 下运行时改贴图路径不渲染，故写死同名文件 + 覆写内容而非换路径。"""
        tex_path = os.path.join(_REPO_ROOT, "assets", "scenes", "e_physical",
                                "e3_density", "textures", "balance_result.png")
        try:
            from PIL import Image, ImageDraw, ImageFont
            W, H = 800, 128
            BG, TXT, SUB = (8, 12, 20), (170, 240, 200), (150, 200, 185)
            img = Image.new("RGB", (W, H), BG)
            d = ImageDraw.Draw(img)
            f_main = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 44)
            t = f"{self.m2_grams:.2f} g"
            bb = d.textbbox((0, 0), t, font=f_main)
            d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 10), t, font=f_main, fill=TXT)
            f_sub = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 28)
            t = f"ρ = {self.density:.3f} g/mL"
            bb = d.textbbox((0, 0), t, font=f_sub)
            d.text(((W - (bb[2] - bb[0])) / 2 - bb[0], 82), t, font=f_sub, fill=SUB)
            # 原子写入（先写临时文件再 os.replace 替换）：渲染器在后台 fiber 线程异步
            # 加载贴图，若直接 img.save 就地截断重写，会读到半写入 PNG（头损坏 → 解码
            # 读垃圾尺寸 → bad_array_new_length 崩溃，08-28 报错）。os.replace 原子替换
            # 保证读者只看到旧的完整文件或新的完整文件，绝无半写入态。
            tmp_path = tex_path + ".tmp"
            img.save(tmp_path)
            os.replace(tmp_path, tex_path)
            print(f"[e3] baked balance_result.png m2={self.m2_grams:.2f}g "
                  f"ρ={self.density:.3f}")
        except Exception as e:
            print(f"[e3] WARN bake result texture failed (fallback ρ=1.0 default): {e}")

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
