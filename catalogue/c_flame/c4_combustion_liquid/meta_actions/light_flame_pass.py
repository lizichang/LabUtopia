"""元动作 ②：点燃酒精灯（C4 燃烧试验液体样品 V2 新增，2026-09-01 用户「拿起火柴点燃酒精灯」）。

照 B2 LightFlamePass（flametest IgniteLamp 结构：取火柴 → 触灯芯 → 放回火柴）逐字移植，
仅换 C4 坐标常量。C4 布局（用户 09-01「酒精灯和灯帽整体移动到火柴的正对 -y」）：
酒精灯 (0.7094,-0.10) rot180，火柴 (0.62,0.05) 抬 13mm 头朝 +X。灯芯 (0.7094,-0.10,0.9007)
在火柴头 (0.7094,0.05) 正下方 → 夹爪从抓点 x=0.66 直推 -Y 到 IGNITE (0.66,-0.10,0.9007)
（gripper x 不变 = 纯直进），火柴头落灯芯；回程纯 +Y 直退，火柴头从火焰侧向撤走，
永不横向扫过火焰柱（无需再抬升绕焰）。

流程（一次持握，无循环）：
  ① 取火柴：默认朝向（手指朝下）高位接近 → 竖直下探到杆身中部抓点，合爪夹住
     火柴杆（GRIP_MATCH 贴合 Ø3mm 杆，火柴已抬高 13mm 避免夹爪扎桌面）。
  ② 低位运移：竖直提起火柴到 MATCH_LIFT_Z=0.90（高于灯体顶 0.8897，横越灯体不穿；
     火柴杆 z=0.899 在灯体顶上方 9mm），直推 -Y 到灯芯上方（IGNITE，夹爪 x=0.66）。
  ③ 触灯芯：火柴头（夹爪 +X 0.0494 处）落在灯芯顶 WICK (0.7094,-0.10,0.9007)，
     task 检测火柴头近灯芯 → 点火（火焰 reveal，见 task._step_match_ignite）。
  ④ 放回：直退 +Y 回火柴原位上方 → 松爪 → 高位归位（task 火柴生命周期写回 rest）。

持握 = 纯平移 offset（task.MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
（与滴管的矩阵持握不同——火柴杆横躺，夹爪手指朝下竖直夹其杆身，火柴姿态恒等旋转
只跟夹爪平移）。火柴头相对夹爪 = MATCH_TIP_OFFSET=(0.0494,0,0)（抓点 x=0.04、
头中心 x=0.0894 → 头在夹爪 +X 0.0494）。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((mx,my,MATCH_HIGH))           # 手指朝下
  ② 竖直下探   mv(MATCH_GRASP)                  # 夹爪降到杆身中心，两指竖直夹杆
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_MATCH, 60)             # task 检测 attached
  ⑤ 竖直提出   mv((mx,my,MATCH_LIFT_Z), 5)      # 火柴随夹爪提起（低位 0.90）
  ⑥ 触灯芯     mv(IGNITE, 20)                   # 直推 -Y 落灯芯（头近 WICK，dwell 点火）
  ⑦ 直退放回   mv((mx,my,MATCH_LIFT_Z))         # 纯 +Y 直退回火柴原位上方（头侧向撤出火焰）
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)              # task: released → 火柴写回 rest
  ⑨ 高位归位   mv((mx,my,MATCH_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP,
                        MATCH_LIFT_Z, IGNITE, MATCH_HIGH)


class LightFlamePass(BaseMetaAction):
    """取火柴 → 低位运移 → 触灯芯点燃 → 直退放回。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        return [
            mv(e, (mx, my, MATCH_HIGH)),                # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),                         # ② 竖直下探到杆身中心（两指竖直夹杆）
            hold(e, SETTLE),                            # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                    # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_LIFT_Z), 5),           # ⑤ 竖直提出（低位 0.90）
            mv(e, IGNITE, 20),                          # ⑥ 直推 -Y 触灯芯点燃（头落灯芯，dwell 20）
            mv(e, (mx, my, MATCH_LIFT_Z)),              # ⑦ 纯 +Y 直退回火柴原位上方（头侧向撤出火焰）
            grip(e, GRIP_OPEN, 25),                     # ⑧ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),                # ⑨ 高位归位
        ]
