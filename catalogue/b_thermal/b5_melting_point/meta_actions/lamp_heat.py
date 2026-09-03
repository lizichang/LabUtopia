"""元动作 ⑭：加热摆动 15s 控温 + 移灯 -X 5cm（B5 熔点测定）。

用户 2026-09-03 逐字：「加热的时候要停留15s，这个期间机械臂应该夹住酒精灯在x方向上来回
移动，来控制升温速度，最后盖酒精灯帽前，应该先把酒精灯往-x移动5cm移出再盖酒精灯。夹酒精灯
参考B2、B3。」——照 B2/B3 LampMovePass 水平横夹灯体宽处（z=0.845，ORIENT_FWD 手指朝前），
中间插入 ShakeAction X 轴正弦摆动 15s（15 来回 × 1s）控温，再 -X 移 5cm 移出侧臂下方，松爪。

抓取/高位中转照 B2/B3（同款 alcohol_lamp.usd，灯体最宽 Ø87.2mm 超夹爪 80mm → 抓其上方
z=0.845 Ø76.8mm 可握）：先高位中转 LAMP_HIGH(-X 侧上方 1.25，从上方竖直降，不从正前方甩
进来)，再竖直降到 LAMP_APPROACH(-X 侧 0.245)，只水平 +X 移入灯体中心 LAMP_GRASP，合爪。

B5 灯 (0.355,0.0029) 在提勒管侧臂下弯处正下方（加热点 x=0.355、火焰尖 0.9795）。加热摆动
±12mm 让火焰尖在加热点附近小范围往复控温；移灯 -X 5cm → (0.305,0.0029) 移出侧臂正下方，
帽可竖直下扣不穿侧臂（侧臂 x≥0.355）。

流程（一次持握，无循环，全程 ORIENT_FWD 手指朝前）：
  ① 张开爪子   grip(GRIP_OPEN, 25)
  ② 高位中转   mv(LAMP_HIGH)（-X 侧上方清障，不从正前方甩进来）
  ③ 竖直降下   mv(LAMP_APPROACH)（灯 -X 侧准备，灯体宽处高度）
  ④ +X 移入    mv(LAMP_GRASP)（灯体中心）
  ⑤ 停顿稳定   hold(SETTLE)
  ⑥ 合爪夹灯   grip(GRIP_LAMP, 60) → task attached（灯 _LAMP_HELD·tool_world、火焰 x 跟随）
  ⑦ 加热摆动   shake(LAMP_GRASP, axis=X, 15s)（X 轴正弦往复，火焰尖控温）
  ⑧ 移灯 -X5cm mv(LAMP_TARGET, 40)（移出提勒管侧臂正下方）
  ⑨ 松爪释放   grip(GRIP_OPEN, 25) → task released → 灯锁 LAMP_TARGET
  ⑩ 退回       mv(LAMP_APPROACH)（灯停在前方 -X，退回清空）
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .constants import (SETTLE, GRIP_OPEN, GRIP_LAMP, ORIENT_FWD,
                        LAMP_APPROACH, LAMP_HIGH, LAMP_GRASP, LAMP_TARGET,
                        HEAT_SWAY_AMP, HEAT_SWAY_CYCLES, HEAT_SWAY_PERIOD)


class LampHeatMovePass(BaseMetaAction):
    """水平横夹酒精灯灯体宽处 → X 轴摆动 15s 控温 → -X 移 5cm → 松爪。"""

    def _build_actions(self):
        e = self.engine
        return [
            grip(e, GRIP_OPEN, 25),                    # ① 张开爪子（用户：先把机械臂张开爪子）
            mv(e, LAMP_HIGH, orient=ORIENT_FWD),       # ② 高位中转（-X 侧上方，不从正前方甩进来）
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ③ 竖直降到灯 -X 侧准备
            mv(e, LAMP_GRASP, orient=ORIENT_FWD),      # ④ 只水平 +X 移入 灯体中心
            hold(e, SETTLE),                           # ⑤ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_LAMP, 60),                    # ⑥ 合爪夹住灯体最宽处（task 检测 attached）
            shake(e, LAMP_GRASP, axis=(1, 0, 0),       # ⑦ X 轴正弦摆动 15s 控温（火焰尖小范围往复）
                  amplitude=HEAT_SWAY_AMP, cycles=HEAT_SWAY_CYCLES,
                  period=HEAT_SWAY_PERIOD, orient=ORIENT_FWD),
            mv(e, LAMP_TARGET, 40, orient=ORIENT_FWD),  # ⑧ 水平 -X 移 5cm 移出侧臂正下方
            grip(e, GRIP_OPEN, 25),                    # ⑨ 松爪：task 灯锁 LAMP_TARGET
            mv(e, LAMP_APPROACH, orient=ORIENT_FWD),   # ⑩ 退回（灯停在前方 -X）
        ]
