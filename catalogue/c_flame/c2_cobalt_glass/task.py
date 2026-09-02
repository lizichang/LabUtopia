"""C2 焰色反应（隔钴玻璃观察）任务。

= C1 完整 13 步灼烧流程（铂丝蘸取固体样品）+ 两步判读的"隔钴玻璃观察"段。

C1 任务逐字复用（C1FlameWireSolidTask），仅在受染阶段后追加"隔玻璃观察"：
  ① 直接观察：铂丝尖端入外焰 → 火焰受染成本色（flame_color，Na 黄焰）。
  ② 隔钴玻璃观察：直接观察停留 through_glass_dwell_frames 帧后，切换火焰为
     through_glass 结果——紫色=K（钾，透过钴玻璃滤掉 Na 黄线后露紫焰）、
     无色=仅 Na（黄线被钴玻璃吸收，火焰看似无色）。

钴玻璃（/World/CobaltGlass）是**固定静态器材**，机械臂不抓它（铂丝受染时机械臂
已占用），故 C2 动作序列与 C1 完全一致，camera2 始终透过玻璃看火焰。

时序（已核对 BurnStain 元动作）：受染灼烧在 FLAME_HOLD 停留 400 帧，直接观察
stain_dwell_frames=150 帧 + 隔玻璃 through_glass_dwell_frames=150 帧 = 300 帧 < 400，
无需扩展元动作。切换观察色用预制变体 + visibility（RTX 不刷新运行时 shader 编辑）。
"""
import numpy as np
from pxr import UsdGeom, Gf
from isaacsim.core.utils.prims import set_prim_visibility

from catalogue.c_flame.c1_flame_wire_solid.task import C1FlameWireSolidTask


class C2CobaltGlassTask(C1FlameWireSolidTask):
    """Task definition for the flame test observed through a fixed cobalt glass."""

    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

        # 固定钴玻璃（静态，不抓取）——仅记录路径供参考，无运行时操作
        self.cobalt_glass_path = getattr(cfg, "cobalt_glass_path", "/World/CobaltGlass")
        # 隔玻璃观察结果：purple=K（钾） / colorless=仅 Na（钠）
        self.through_glass = getattr(cfg, "through_glass", "purple")
        self.through_glass_dwell_frames = int(
            getattr(cfg, "through_glass_dwell_frames", 150))
        self.through_glass_counter = 0
        self._through_glass_fired = False

    def reset(self):
        super().reset()
        self.through_glass_counter = 0
        self._through_glass_fired = False

    # ------------------------------------------------------------------
    # 每帧更新：先走 C1 全部逻辑（点燃/滴液/受染/灭焰），再追加隔玻璃观察段
    # ------------------------------------------------------------------
    def _update_effects(self, gripper_pos):
        super()._update_effects(gripper_pos)
        self._update_through_glass(gripper_pos)

    def _update_through_glass(self, gripper_pos):
        """受染（黄焰）揭示后：停留 through_glass_dwell_frames 帧 → 切隔玻璃结果。

        门控与 C1 受染段一致：火焰点燃 + 铂丝 attached + 已受染 + 尖端在外焰内。
        尖端离开火焰则清计数器（但 _through_glass_fired 一旦置位即为粘性里程碑）。
        """
        if not self.flame_on or self.wire_state != "attached":
            return
        if not self.stain_on:
            self.through_glass_counter = 0
            return

        tip = gripper_pos + self.WIRE_TIP_OFFSET
        in_flame = (np.linalg.norm(tip[:2] - self.LAMP_POS[:2]) < 0.05
                    and self.FLAME_Z[0] < tip[2] < self.FLAME_Z[1])
        if not in_flame:
            self.through_glass_counter = 0
            return

        if self._through_glass_fired:
            # 已进入隔玻璃观察：持续断言观察色（紫跟随尖端 / 无色全隐藏），
            # 覆盖"尖端短暂离焰再入"时 C1 重显黄焰的边角情况。
            self._apply_through_glass(tip)
            return

        self.through_glass_counter += 1
        if self.through_glass_counter >= self.through_glass_dwell_frames:
            self._through_glass_fired = True
            self._apply_through_glass(tip)
            print(f"[c2] through-glass observation: {self.through_glass}")

    def _apply_through_glass(self, tip_world):
        """把火焰切到隔玻璃观察结果：紫→flame_stain_purple / 无色→全隐藏。"""
        if self.through_glass == "purple":
            self._set_stain_color("purple")
            self._position_stain_color_at_tip("purple", tip_world)
        else:  # colorless：吸掉 Na 589nm 黄线 → 火焰看似无色（全隐藏）
            self._set_stain_color(None)

    # ------------------------------------------------------------------
    # 染色锥辅助（C1 只切换/定位 flame_color 本色，这里需任意色）
    # ------------------------------------------------------------------
    def _set_stain_color(self, color):
        """显示指定颜色的染色锥并隐藏其余；color=None 全隐藏。"""
        for c in self.FLAME_COLORS:
            prim = self.stage.GetPrimAtPath(f"{self.STAIN_ROOT}/flame_stain_{c}")
            if prim.IsValid():
                set_prim_visibility(prim, False)
        if color is not None:
            prim = self.stage.GetPrimAtPath(f"{self.STAIN_ROOT}/flame_stain_{color}")
            if prim.IsValid():
                set_prim_visibility(prim, True)

    def _position_stain_color_at_tip(self, color, tip_world):
        """将指定颜色染色锥中心定位到铂丝尖端（世界坐标）。"""
        prim = self.stage.GetPrimAtPath(f"{self.STAIN_ROOT}/flame_stain_{color}")
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(float(tip_world[0]), float(tip_world[1]),
                                    float(tip_world[2])))
                    break

    def on_task_complete(self, success: bool) -> None:
        """成功 = C1 三里程碑（点燃+受染+灭焰）+ 隔玻璃观察里程碑全发生。

        绕过 C1.on_task_complete（它只算 3 里程碑、会忽略 _through_glass_fired），
        直接调 BaseTask.on_task_complete。
        """
        real = (getattr(self, '_ignite_fired', False)
                and getattr(self, '_stain_fired', False)
                and getattr(self, '_through_glass_fired', False)
                and getattr(self, '_extinguish_fired', False))
        print(f"[c2] episode done success={real} (controller_said={success}) "
              f"ignite={getattr(self, '_ignite_fired', False)} "
              f"stain={getattr(self, '_stain_fired', False)} "
              f"through_glass={getattr(self, '_through_glass_fired', False)} "
              f"extinguish={getattr(self, '_extinguish_fired', False)} "
              f"min_gripper_z={getattr(self, '_min_gripper_z', float('nan')):.4f}")
        # super(C1FlameWireSolidTask, self) = BaseTask（跳过 C1 的 3 里程碑重算）
        super(C1FlameWireSolidTask, self).on_task_complete(real)
