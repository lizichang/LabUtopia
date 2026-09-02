"""元动作：加热结束后移灯（B3 水浴 阶段E，照 B2 LampMovePass 但方向 +Y）。

用户逐字（2026-08-29）：「熄灭酒精灯应该先把酒精灯往+y方向移动20cm(参考b2)，然后再盖上灯冒」
→ 加热结束 + 试管放回架孔后，机械臂水平横夹酒精灯灯体宽处 → 水平 +Y 移 20cm（xz/朝向不变）
→ 松爪。参考 B2 LampMovePass（同资产：灯体最宽 z=0.825 Ø87.2 超夹爪 80mm → 抓其上 20mm 的
z=0.845 Ø76.8 可握，GRIP_LAMP≈Ø76 贴体近零嵌）。

几何（照 B2，灯随加热堆叠移到 (0.5286,-0.25)）：接近点 x=0.4186（灯 x−0.11）在铁架台柱
（x≈0.62）西侧、石棉网底 0.9184 上方路径清空（抓取 z=0.845 < 烧杯底 0.9205，不碰烧杯）。
移灯终点 LAMP_TARGET=(0.5286,-0.05,0.845)（+Y 20cm）。移灯期间 task 把帽钉在静止位 CAP_REST
（帽是灯子 prim，不随灯滑走，否则盖帽 IK 卡死）；松爪后 task 灯锁移灯位 + 火焰跟随灯锁定。

流程（9 步，一次持握，无循环）：
  ① 张开爪子：grip(GRIP_OPEN)。
  ② 高位中转：mv(LAMP_HIGH)（-X 侧上方，清障，不从正前方甩进来）。
  ③ 竖直降下：mv(LAMP_APPROACH)（到灯体宽处 -X 前）。
  ④ +X 移入：mv(LAMP_GRASP) 到灯体中心。
  ⑤ 停顿稳定：hold(SETTLE)。
  ⑥ 合爪夹住灯体：grip(GRIP_LAMP, 60) → task 检测 attached（灯写 _LAMP_HELD·tool_world）。
  ⑦ 水平移灯：+y 移 20cm 到 LAMP_TARGET（xz/朝向不变）。
  ⑧ 松爪释放：grip(GRIP_OPEN, 25) → task: lamp released → 灯锁移灯位 + 气泡渐熄 → phase 前进。
  ⑨ 退回：mv(LAMP_APPROACH)（灯已停在前方，清空）。

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
    """水平横夹酒精灯灯体宽处 → 水平 +y 移 20cm（xz/朝向不变）→ 松爪。"""

    def _build_actions(self):
        e = self.engine
        return [
            grip(e, GRIP_OPEN, 25),              # ① 张开爪子（用户：先把机械臂张开爪子）
            mv(e, LAMP_HIGH, orient=ORIENT_FWD),       # ② 高位中转（-X 侧上方，不从正前方甩进来）
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ③ 竖直降到灯 -X 侧准备（灯体宽处高度）
            mv(e, LAMP_GRASP, orient=ORIENT_FWD),      # ④ 只水平 +X 移入 灯体中心
            hold(e, SETTLE),                     # ⑤ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_LAMP, 60),              # ⑥ 合爪夹住灯体最宽处（task 检测 attached）
            mv(e, LAMP_TARGET, 40, orient=ORIENT_FWD),  # ⑦ 水平 +y 移 20cm（xz/朝向不变）
            grip(e, GRIP_OPEN, 25),              # ⑧ 松爪：task 灯锁移灯位 + 气泡渐熄 + phase 前进
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ⑨ 退回（灯停在前方）
        ]
