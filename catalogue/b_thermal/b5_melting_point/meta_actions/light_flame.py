# -*- coding: utf-8 -*-
"""元动作 ⑬：点燃酒精灯（B5 熔点测定 2026-09-02 用户「拿起火柴点燃酒精灯」，装样+插管之后）。

照 B2 LightFlamePass / C4 LightFlamePass（flametest IgniteLamp 结构：取火柴 → 触灯芯 → 放回
火柴）逐字移植，仅换 B5 坐标常量。B5 布局（用户 2026-09-02 tmp 重摆）：
  酒精灯 (0.355,0.0029) rot180（侧管下弯处正下方），灯芯顶 WICK=(0.355,0.0029,0.9005)；
  火柴 (0.4817,-0.164,0.813) 抬 13mm 头朝 +X（灯 +X -Y 侧右下）。
火柴在灯 +X -Y 侧，与灯芯不同轴 → 夹爪从抓点 x=0.5217 两段直角推进到 IGNITE=(0.3056,0.0029,0.9005)
（先 -X 缩到 0.3056、再 +Y 进 0.167；2026-09-03 用户弃直斜线防刮灯体），火柴头（夹爪 +X
0.0494 处）落灯芯 WICK；回程原路返回。全程默认朝向手指朝下（火柴杆横躺，夹爪竖直夹杆身）。

流程（一次持握，无循环）：
  ① 取火柴：高位接近 → 竖直下探到杆身中部抓点，合爪夹住火柴杆（GRIP_MATCH 贴合 Ø3mm 杆，
     火柴已抬高 13mm 避免夹爪扎桌面）。
  ② 低位运移：竖直提起火柴到 MATCH_LIFT_Z=0.90（高于灯体顶 0.8897、低于提勒管底 0.928）。
     2026-09-03 用户「拿起火柴移动还是会穿模」：弃直线斜推（y 从 -0.164 斜扫到 0.0029 时
     x 0.35-0.52 段会刮灯体/灯座）→ 改两段直角路径（先 -X 后 +Y），放回原路返回：
       a. 先 -X：y 锁火柴行 -0.164（灯下方空档），x 0.5217→0.3056；
       b. 再 +Y：x 锁 IGNITE.x 0.3056，纯 +Y 推进，火柴头（夹爪 +X 0.0494→x0.355）落灯芯。
  ③ 触灯芯：火柴头（夹爪 +X 0.0494 处）落在灯芯顶 WICK，task 检测头近灯芯连续 15 帧 → 点火
     （火焰 reveal + flicker，见 task._step_match_ignite / _step_flame_anim）。
  ④ 放回：原路返回（先 -Y 退到拐点、再 +X 回火柴原位上方）→ 松爪 → 高位归位
     （task 火柴生命周期写回 rest）。

持握 = 纯平移 offset（task.MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转。火柴头
相对夹爪 = MATCH_TIP_OFFSET=(0.0494,0,0)。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((mx,my,MATCH_HIGH))          # 手指朝下
  ② 竖直下探   mv(MATCH_GRASP)                 # 夹爪降到杆身中心，两指竖直夹杆
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_MATCH, 60)            # task 检测 attached
  ⑤ 竖直提出   mv((mx,my,MATCH_LIFT_Z), 5)     # 火柴随夹爪提起（低位 0.90）
  ⑤b 先 -X     mv((ix,my,MATCH_LIFT_Z), 8)     # y 锁火柴行 -0.164，x 0.5217→0.3056（避开灯体）
  ⑥ 再 +Y      mv(IGNITE, 20)                  # x 锁 0.3056，纯 +Y 推进落灯芯（头近 WICK，dwell 20）
  ⑦a 原路 -Y   mv((ix,my,MATCH_LIFT_Z))        # 从 IGNITE 退回拐点（先 -Y）
  ⑦b 原路 +X   mv((mx,my,MATCH_LIFT_Z))        # 拐点退回火柴原位上方（再 +X）
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)             # task: released → 火柴写回 rest
  ⑨ 高位归位   mv((mx,my,MATCH_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP,
                        MATCH_LIFT_Z, IGNITE, MATCH_HIGH)


class LightFlamePass(BaseMetaAction):
    """取火柴 → 低位两段运移（先 -X 再 +Y）→ 触灯芯点燃 → 原路返回放回。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        ix = IGNITE[0]                      # 拐点 x = 灯芯正下方（-X 平移终点）
        return [
            mv(e, (mx, my, MATCH_HIGH)),                # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),                         # ② 竖直下探到杆身中心（两指竖直夹杆）
            hold(e, SETTLE),                            # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                    # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_LIFT_Z), 5),           # ⑤ 竖直提出（低位 0.90）
            mv(e, (ix, my, MATCH_LIFT_Z), 8),           # ⑤b 先 -X：y 锁火柴行，x→IGNITE.x（避直斜线刮灯）
            mv(e, IGNITE, 20),                          # ⑥ 再 +Y：纯 Y 推进，头落灯芯（dwell 20 点火）
            mv(e, (ix, my, MATCH_LIFT_Z)),              # ⑦a 原路 -Y：从灯芯退回拐点
            mv(e, (mx, my, MATCH_LIFT_Z)),              # ⑦b 原路 +X：拐点退回火柴原位上方
            grip(e, GRIP_OPEN, 25),                     # ⑧ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),                # ⑨ 高位归位
        ]
