"""元动作 ①：手指朝前（朝向 camera1）夹取药匙 → 竖直提起 → 最后一个关节（法兰）转 90° 放平（第 2 微动作）。

用户动作要求（逐字，2026-08-14）：「动作还是不对，首先你爪子夹起的方式跟
level4_liquidmixing不一样，你要模仿：爪子朝前，朝向camera1，夹起。就只做这两步
先不用旋转」；「我说朝前的意思是你现在的样子正好是朝反的」。

根因：旧 ORIENT_HORIZ=(0.7071,0,-0.7071,0) 在引擎真实约定（存储元组直接喂
from_quat）下让手指 tool+Z=(-1,0,0) 朝反（背离 camera1，用户亲眼确认）。改为
ORIENT_FWD=(0,0.7071,0,0.7071)：手指 tool+Z=(1,0,0) 朝 +X = 朝向 camera1；药匙
tool+X=(0,0,-1) 头下柄上；attach 后药匙世界 = REST 零跳变（pxr 数值验证）。

轨迹（TCP 世界坐标）：用户 2026-08-14 晚追加第 2 微动作——「机械臂的夹爪的那个最后一个关节
旋转90度，只加这一个变化」，并纠正我最初错用腕关节簇贪心的写法「写的不对，只动最后一个关节！」
→ ⑤ FlangeRollAction 只命令最后一个关节（panda_joint7 法兰自转，索引6）转 +90°（见 flange_roll.py）：
  ① 高位       mv((sx,sy,H),         orient=ORIENT_FWD)   # 药匙柄杆正上方，手指朝前朝向 camera1
  ② 垂直下探   mv(SPAT_GRASP,        orient=ORIENT_FWD)   # 竖柄杆喂进指缝（手指朝前，抓点 z=0.94）
  ③ 夹起       grip(GRIP_SPATULA)                          # 合爪夹住柄杆（模仿 level4：爪子朝前夹起）
  ④ 竖直提起   mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_FWD)  # 药匙竖直提起，勺尖过架顶 0.917
  ⑤ 法兰转90°  FlangeRollAction()                         # 只动最后一个关节：药匙竖直→水平（勺头朝 -Y，用户确认方向）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, DISH_XY, GRIP_SPATULA, ORIENT_FWD, ORIENT_SCOOP,
                        SPAT_LIFT_Z, SPAT_XY, SPAT_GRASP, SCOOP_ALIGN)
from .flange_roll import FlangeRollAction


class PickSpatula(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = SPAT_XY
        return [
            mv(e, (sx, sy, H), orient=ORIENT_FWD),            # ① 高位：手指朝前（+X）朝向 camera1
            mv(e, SPAT_GRASP, orient=ORIENT_FWD),             # ② 垂直下探：竖柄杆进指缝
            grip(e, GRIP_SPATULA, 60),                       # ③ 夹起（level4 模式：爪子朝前夹起）
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_FWD),  # ④ 竖直提起
            FlangeRollAction(),                               # ⑤ 只动最后一个关节转90°：药匙竖直→水平
            # ⑥ 法兰转后第一步：水平往 -X、y 对齐表面皿（用户 2026-08-16 重写，z 保持高位）。
            #    拆成两段连续水平移动（每段仅 ~0.16m，warm start 连续，joint7 保持 -90°，
            #    药匙始终水平；一段 0.3m 长距离 IK 易解出 joint7→0 的分支＝药匙竖直"挖掘"）。
            mv(e, (DISH_XY[0], sy, H), orient=ORIENT_SCOOP),  # ⑥a 水平往-X：x 对齐表面皿 x、y 保持
            mv(e, SCOOP_ALIGN, orient=ORIENT_SCOOP),          # ⑥b 再 y 对齐表面皿 y（z 保持高位）—— 本步结束
        ]
