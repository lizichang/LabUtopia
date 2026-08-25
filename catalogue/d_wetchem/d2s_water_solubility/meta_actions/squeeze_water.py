"""元动作 S4：挤水（夹爪进一步合拢挤瓶身肚子，水从红嘴流入试管）。

用户 2026-08-25（逐字）：「很好现在加入挤水动作，同时要有效果，水流从红嘴出来进入试管」。

S3 结束夹爪持瓶肚子停在管口上方（持握开度 GRIP_WASHBOT=0.030，红嘴终位
(0.649,0.231,0.994) 在管口 (0.659,0.241,0.9593) 上方），本动作只开合夹爪、不移动臂：
  ① 合爪到 WASH_SQUEEZE（0.020）挤压瓶身 → task._update_washbottle 检测
     opening < WASH_SQUEEZE_CLOSED → 显示水流（WaterStream），水从红嘴流入试管
  ② 松回 GRIP_WASHBOT（0.030）结束挤压 → task 检测 opening 回升 → 隐藏水流、
     显示管内水（TubeWater）

水流/管内水效果由 task 驱动（夹爪开度作触发信号，仿 D2-S 药粉下落/D3-L 滴管），
本动作只负责夹爪开合。
"""
from ._base import BaseMetaAction, grip
from .constants import WASH_SQUEEZE, WASH_SQUEEZE_DWELL, GRIP_WASHBOT


class SqueezeWater(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            grip(e, WASH_SQUEEZE, WASH_SQUEEZE_DWELL),   # ① 合爪挤压肚子出水（水流效果触发）
            grip(e, GRIP_WASHBOT, 20),                    # ② 松回持握开度（水流结束、管内显水）
        ]
