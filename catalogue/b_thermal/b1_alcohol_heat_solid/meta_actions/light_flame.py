"""元动作：取火柴点燃酒精灯（B1 三过程之三，2026-08-27 用户指定）。

照 B2 LightFlamePass 逐字复用（b2_alcohol_heat_liquid/meta_actions/light_flame.py，
已验证），仅换 B1 的点火坐标：B1 灯在 (0.50,0.0029)（无铁架台，灯芯顶 0.9007），
火柴头落 WICK=(0.50,0.0029,0.9005)，夹爪到 IGNITE=(0.4506,0.0029,0.9005)（夹爪在
灯体 min x 0.4564 之外，火柴头 +X 0.0494 到灯芯）。

流程（9 步，一次持握）：
  ① 取火柴：高位接近 → 竖直下探到杆身中部抓点，合爪夹住火柴杆
  ② 低位运移：竖直提起至 MATCH_LIFT_Z=0.90（压灯体顶 0.8897 之下），水平运到
     灯芯偏 -X 侧（IGNITE）
  ③ 触灯芯：火柴头落在 WICK，task 检测火柴头近灯芯连续 15 帧 → 点火
     （B1 无温度模型，flame_lit 置位即 reveal 火焰，见 task._step_match_ignite）
  ④ 放回：原路退回火柴上方 → 松爪 → 高位归位

持握 = 纯平移 offset（task.MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转。
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
            mv(e, MATCH_GRASP),                     # ② 竖直下探到杆身中心（两指竖直夹杆）
            hold(e, SETTLE),                        # ③ 停顿稳定
            grip(e, GRIP_MATCH, 60),                # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_LIFT_Z), 5),       # ⑤ 竖直提出（低位 0.90）
            mv(e, IGNITE, 20),                      # ⑥ 触灯芯点燃（头落灯芯，dwell 20）
            mv(e, (mx, my, MATCH_LIFT_Z)),          # ⑦ 原路退回火柴上方
            grip(e, GRIP_OPEN, 25),                 # ⑧ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),            # ⑨ 高位归位
        ]
