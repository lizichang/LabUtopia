"""元动作 ②：点燃酒精灯（B3 水浴加热 阶段B，复刻 B2 LightFlamePass）。

用户逐字（B2 2026-08-27）：「现在增加点燃火焰的动作，参考焰色反应」——照 flametest
IgniteLamp（controllers/flametest_meta_actions/ignite_lamp.py）的「取火柴 → 触灯芯
→ 放回火柴」结构。B3 的酒精灯/石棉网/火柴坐标与 B2 完全一致（同一加热堆叠），故本
元动作逐字照搬 B2，仅换 import 路径。

流程（一次持握，无循环）：
  ① 取火柴：默认朝向（手指朝下）高位接近 → 竖直下探到杆身末尾抓点（-X 端 10mm，
     2026-08-30 用户逐字「夹火柴的时候不要夹中间要夹末尾（-x端）」），合爪夹住火柴杆。
  ② 低位运移：竖直提起火柴到 MATCH_LIFT_Z=0.90（压灯体顶 0.8897、钩支臂 1.246 之下），
     水平运到灯芯偏 -X 侧（IGNITE，夹爪 x=0.4492 在灯体 min x=0.485 之外）。
  ③ 触灯芯：火柴头（夹爪 +X 0.0794 处）落在灯芯顶 WICK (0.5286,-0.25,0.9005)，
     task 检测火柴头近灯芯 → 点火（flame reveal，见 task._step_match_ignite）。
  ④ 放回：原路退回火柴上方 → 松爪 → 高位归位（task 火柴生命周期写回 rest）。

持握 = 纯平移 offset（task.MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((mx,my,MATCH_HIGH))
  ② 竖直下探   mv(MATCH_GRASP)
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_MATCH, 60)
  ⑤ 竖直提出   mv((mx,my,MATCH_LIFT_Z), 5)
  ⑥ 触灯芯     mv(IGNITE, 20)
  ⑦ 原路退回   mv((mx,my,MATCH_LIFT_Z))
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)
  ⑨ 高位归位   mv((mx,my,MATCH_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP,
                        MATCH_LIFT_Z, IGNITE, MATCH_HIGH)


class LightFlamePass(BaseMetaAction):
    """取火柴 → 低位运移 → 触灯芯点燃 → 放回火柴。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        return [
            mv(e, (mx, my, MATCH_HIGH)),            # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),  # ② 竖直下探夹杆身末尾（纯竖直线走 0.536m 需 ~357 帧；勿加短 timeout，否则 force-done 在 z≈1.05 火柴没夹起来）
            hold(e, SETTLE),                        # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_LIFT_Z), 5),       # ⑤ 竖直提出（低位 0.90）
            mv(e, IGNITE, 20, timeout=240),         # ⑥ 触灯芯点燃（头落灯芯，dwell 20；横移IK卡顿→4s兜底）
            mv(e, (mx, my, MATCH_LIFT_Z)),          # ⑦ 原路退回火柴上方
            grip(e, GRIP_OPEN, 25),                 # ⑧ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),            # ⑨ 高位归位
        ]
