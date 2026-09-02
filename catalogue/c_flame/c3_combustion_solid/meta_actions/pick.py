"""元动作 ①：C3 横夹药匙 → 挖起粉末 → 移向燃烧匙（pick → scoop → lift → 倒燃烧匙前段）。

完全复刻 d2s PickSpatula 的挖粉前 10 步（① 高位 → ② 下探 → ③ 夹起 → ④ 提起 → ⑤ 法兰-45°
→ ⑥ 对齐粉堆 x → ⑦ 竖直降 → ⑧ 平移-y → ⑨ 挖粉 → ⑩ 抬升），坐标**逐字对齐 d2s**
（用户 2026-09-01 要求，改前自定义坐标致臂卡死）：
  - 药匙家用 (0.6993,0.3608)（= d2s）
  - 粉堆中心 x=0.537（= d2s）
  - 挖粉 -Y 平移 0.16（= d2s）
其余 7 个子动作几何逐字同构（药匙/表面皿/粉末相对尺寸一致，粉顶 z=0.8141、皿沿 0.8066、
下降量 0.245 全部与 d2s 相同）。

用户动作要求（逐字，2026-09-01）：「现在开始写动作，先模仿d2s横夹药匙然后挖起药粉」——
横夹 = ORIENT_FWD 手指朝前（朝向 camera1），挖起 = 法兰 -45°→-90°（只动最后一关节），
勺尖从粉丘挖起、凹槽朝上蓄粉。倒燃烧匙前段（2026-09-01 追加「z下降10cm，向+y移动20cm，
+x移动5cm」，随后「正y方向那一步再多移动10cm」「z再多下降7cm」「+y少移动5cm」「加动作根d2s一样，
一个动作在药匙旋转竖直的同时往-y移动24cm」）⑪⑫⑬⑭ 已接续：
  终值 z 降 17cm / +y 31cm / +x 5cm + ⑭ 法兰转竖直 与 往 -y 平移 18cm 同步（同一进度 t）
  （⑭ C3 本地 FlangeRollShiftYNeg 同步版：joint7 法兰 -90°→0° 直接命令 与 TCP -y 平移
  同一进度同始同终；带朝向 IK 无解时退路只解位置保证水平走；实际法兰收敛才冻结不抖；
  2026-09-01 反馈「没水平走+抖」后两阶段版被否定「-y移动和法兰旋转不是同步的」→ 改回同步）。

轨迹（TCP 世界坐标）：
  ① 高位       mv((sx,sy,H),          orient=ORIENT_FWD)   药匙柄杆正上方，手指朝前
  ② 垂直下探   mv(SPAT_GRASP,         orient=ORIENT_FWD)   竖柄杆喂进指缝（z=0.94）
  ③ 夹起       grip(GRIP_SPATULA)                          合爪夹柄杆（level4 横夹模式）
  ④ 竖直提起   mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_FWD)  勺尖过架顶 0.917
  ⑤ 法兰转-45° FlangeRollAction()                          只动最后一关节：药匙竖直→45° 倾斜
  ⑥ 对齐粉堆x  AlignPowderX(x=0.537)                       首帧自采样世界朝向并保持，水平移到粉堆中心 x=0.537
  ⑦ 竖直降     LowerPowder(drop=0.245)                     锁 x/y + 保持朝向，仅 z 降 0.245（同 d2s）
  ⑧ 平移-y     ShiftYNeg(shift=0.16)                       锁 x/z + 保持朝向，仅 y 减 0.16
  ⑨ 挖粉       ScoopUpAction()                             法兰 -45°→-90°（只动 joint7）：勺尖从粉丘挖起
  ⑩ 抬升       LiftToTube(z=H)                             保持朝向、x/y 锁当前，仅 z 升到安全高位 1.15
  ⑪ 下降17cm   LowerPowder(drop=0.17)                      锁 x/y + 保持朝向，仅 z 降 0.17（1.15→0.98）
  ⑫ 平移+y31cm ShiftYPos(shift=0.31)                       锁 x/z + 保持朝向，仅 y 增 0.31（0.2008→0.5108）
  ⑬ 平移+x5cm  ShiftXPos(shift=0.05)                       锁 y/z + 保持朝向，仅 x 增 0.05（0.537→0.587）
  ⑭ 转竖直+移-y18cm FlangeRollShiftYNeg(shift=0.18)        法兰 -90°→0° 与 -y 18cm 同一进度 t 同步（t>=1 即冻结，同步结束）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_SPATULA, ORIENT_FWD,
                        SPAT_LIFT_Z, SPAT_XY, SPAT_GRASP_Z,
                        POWDER_X, DROP_DOWN, Y_SHIFT_NEG,
                        SPOON_DOWN, SPOON_Y_SHIFT, SPOON_X_SHIFT, SPOON_Y_NEG_LAST)
# 挖粉子动作直接复用 d2s 元动作包（机制已踩坑验证，仅参数换 C3 坐标）
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.align_powder_x import AlignPowderX
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.flange_roll import FlangeRollAction
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.lower_powder import LowerPowder
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.shift_y_neg import ShiftYNeg
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.shift_y_pos import ShiftYPos
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.shift_x_pos import ShiftXPos
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.scoop_up import ScoopUpAction
from catalogue.d_wetchem.d2s_water_solubility.meta_actions.lift_to_tube import LiftToTube
from .flange_roll_shift_y_neg import FlangeRollShiftYNeg   # C3 本地版：法兰转竖直 + -y 平移同步（无解退路 + 实际法兰收敛冻结）


class PickSpatula(BaseMetaAction):
    """C3 横夹药匙 → 挖起粉末 → 抬升到安全高位 → 移向燃烧匙（⑪⑫⑬ 倒燃烧匙前段）。

    home（可选）：药匙家用 (x,y)。None 用 C3 SPAT_XY (0.6993,0.3608)。
    """

    def __init__(self, engine, home=None):
        self.spatula_home = SPAT_XY if home is None else tuple(float(v) for v in home)
        super().__init__(engine)

    def _build_actions(self):
        e = self.engine
        sx, sy = self.spatula_home
        grasp = (sx, sy, SPAT_GRASP_Z)
        return [
            mv(e, (sx, sy, H), orient=ORIENT_FWD),            # ① 高位：手指朝前（+X）朝向 camera1
            mv(e, grasp, orient=ORIENT_FWD),                  # ② 垂直下探：竖柄杆进指缝
            grip(e, GRIP_SPATULA, 60),                       # ③ 夹起（level4 模式：爪子朝前夹起）
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_FWD),  # ④ 竖直提起
            FlangeRollAction(),                               # ⑤ 只动最后一个关节转-45°：药匙竖直→45° 倾斜
            AlignPowderX(e, x=POWDER_X),                      # ⑥ 保持世界朝向、y 锁当前，水平移到粉堆 x=0.537
            LowerPowder(e, drop=DROP_DOWN),                   # ⑦ 保持世界朝向、x/y 锁当前，竖直下降 0.245
            ShiftYNeg(e, shift=Y_SHIFT_NEG),                  # ⑧ 保持世界朝向、x/z 锁当前，往 -y 平移 0.16
            ScoopUpAction(),                                  # ⑨ 法兰 -45°→-90°：勺尖从粉丘挖起、凹槽朝上蓄粉
            LiftToTube(e, z=H),                              # ⑩ 保持世界朝向、x/y 锁当前，仅 z 升到安全高位 1.15
            LowerPowder(e, drop=SPOON_DOWN),                 # ⑪ 保持世界朝向、x/y 锁当前，竖直下降 17cm（1.15→0.98）
            ShiftYPos(e, shift=SPOON_Y_SHIFT),               # ⑫ 保持世界朝向、x/z 锁当前，往 +y 平移 31cm（0.2008→0.5108）
            ShiftXPos(e, shift=SPOON_X_SHIFT),               # ⑬ 保持世界朝向、y/z 锁当前，往 +x 平移 5cm（0.537→0.587）
            FlangeRollShiftYNeg(e, shift=SPOON_Y_NEG_LAST),  # ⑭ 法兰 -90°→0° 转竖直 与 往 -y 平移 18cm 同步（同一进度 t，同步结束）
        ]
