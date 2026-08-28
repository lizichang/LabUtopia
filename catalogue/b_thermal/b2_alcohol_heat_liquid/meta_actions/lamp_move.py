"""元动作 ⑤：沸腾 5s 后抓灯移灯（B2 沸点测定 V8，2026-08-27 用户修订 + 08-28 高位中转）。

用户逐字（2026-08-27）：「先把机械臂张开爪子 在酒精灯的-X方向位置准备，然后只往
水平正x移动，然后夹爪闭合夹住酒精灯。而且你现在夹住酒精灯的脖子太靠上了，能不能
往下夹，夹住最宽的地方。然后之前说移动5厘米太少了改为移动十厘米」——水平横夹
酒精灯灯体宽处（非顶部脖子），-X 侧准备只水平 +X 移入，合爪后 -y 移 10cm。
用户再确认（2026-08-28）：「那就抓稍微粗一点的地方单证不能抓火焰下方最细的部分」→
保持灯体宽处 z=0.845。

抓取可行性（2026-08-27 pxr 实测场景世界坐标）：
  灯体最宽 z=0.825（Ø87.2mm）超夹爪最大开口 80mm → 抓其上 20mm 的 z=0.845
  （Ø76.8mm，仍明显「宽身体」）可握（GRIP_LAMP≈Ø76 贴体近零嵌）；爪子下段陷进
  最宽处那 ~5mm 被不透明灯体遮挡不可见。铁架台柱在灯 +X 侧（x 0.6226-0.6346）、
  环/钩在 z≥0.93 → -X 接近路径（x 0.4186→0.5286，z 0.845）全清空。吸附期关灯碰撞。

高位中转（2026-08-28 用户「动作还是从y的方向夹住…动作有点乱」）：调试起点在 ik_home，
旧 ② 直线飞向 (0.4486,0.0029,0.845) 会从正前方（-Y）甩进来 → ② 先抬到 LAMP_HIGH
(0.4186,0.0029,1.25)（-X 侧上方，高于火焰顶 0.938/试管口 1.0939、避开钩），③ 再从
上方竖直降到 -X 侧 LAMP_APPROACH → ④ 只水平 +X 移入 → ⑤ 合爪 → ⑥ -y 移 10cm。
无论起点在哪儿（调试 ik_home 或正常流程火柴归位）都从上方 → 灯 -X 侧 → 只 +X 移入。
接近点 x=0.4186（灯 x−0.11）：铁环 -X 边 x 0.4746（y[-0.024,0.030] 实心弧段），手掌
世界 X 宽 ~6.3cm（pxr hand.stl 实测）→ 下探线须 x<0.42 才不碰环，取 0.4186 净空 5.6cm。

流程（一次持握，无循环）：
  ① 张开爪子：grip(GRIP_OPEN)（用户：先把机械臂张开爪子）。
  ② 高位中转：mv(LAMP_HIGH)（-X 侧上方，清障，不碰钩/柱/温度计/铁环）。
  ③ -X 侧准备：从高位竖直降到 LAMP_APPROACH (0.4186,0.0029,0.845)（11cm 在灯体宽处 -X 前）。
  ④ 只水平 +X 移入：mv(LAMP_GRASP) 到灯体中心 (0.5286,0.0029,0.845)。
  ⑤ 停顿稳定：hold(SETTLE)。
  ⑥ 合爪夹住灯体：grip(GRIP_LAMP, 60) → task 检测 attached（灯写
     _LAMP_HELD·tool_world，灯保持竖直 R180 与场景一致零跳变；火焰 y 跟随灯原点 y）。
  ⑦ 水平移灯：-y 移 10cm 到 LAMP_TARGET=(0.5286,-0.0971,0.845)（xz/朝向不变）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: released → 灯锁移灯位、火焰锁移灯位、
     气泡逐个渐熄 → phase → done。
  ⑨ 退回：+y/+x 退回 LAMP_APPROACH（灯已停在前方，清空）。

轨迹（TCP 世界坐标，全程 ORIENT_FWD 手指朝前）：
  ① 张开爪子   grip(GRIP_OPEN, 25)
  ② 高位中转   mv(LAMP_HIGH)
  ③ 竖直降下   mv(LAMP_APPROACH)
  ④ +X 移入    mv(LAMP_GRASP)
  ⑤ 停顿稳定   hold(SETTLE)
  ⑥ 合爪夹紧   grip(GRIP_LAMP, 60)
  ⑦ 水平移灯   mv(LAMP_TARGET, 40)
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)
  ⑨ 退回       mv(LAMP_APPROACH)
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_LAMP, ORIENT_FWD,
                        LAMP_APPROACH, LAMP_HIGH, LAMP_GRASP, LAMP_TARGET)


class LampMovePass(BaseMetaAction):
    """水平横夹酒精灯灯体宽处 → 水平 -y 移 10cm（xz/朝向不变）→ 松爪。"""

    def _build_actions(self):
        e = self.engine
        return [
            grip(e, GRIP_OPEN, 25),              # ① 张开爪子（用户：先把机械臂张开爪子）
            mv(e, LAMP_HIGH, orient=ORIENT_FWD),       # ② 高位中转（-X 侧上方，不从正前方甩进来）
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ③ 竖直降到灯 -X 侧准备（灯体宽处高度）
            mv(e, LAMP_GRASP, orient=ORIENT_FWD),      # ④ 只水平 +X 移入 灯体中心
            hold(e, SETTLE),                     # ⑤ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_LAMP, 60),              # ⑥ 合爪夹住灯体最宽处（task 检测 attached）
            mv(e, LAMP_TARGET, 40, orient=ORIENT_FWD),  # ⑦ 水平 -y 移 10cm（xz/朝向不变）
            grip(e, GRIP_OPEN, 25),              # ⑧ 松爪：task 灯锁移灯位 + 气泡渐熄 + phase→done
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ⑨ 退回（灯停在前方）
        ]
