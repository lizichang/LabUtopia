"""元动作 ②：挤压洗瓶身出水（水从红嘴弧线落入烧杯，烧杯内液面上涨 + 冰块浮起）。

用户逐字（2026-08-30）：「机械臂往里面挤入液体要真实（可参考a3），然后烧杯里面的
冰块浮起来（符合物理现象）」。

PickWashBottle（①）结束夹爪持瓶肚子停在挤水位（持握开度 GRIP_WASHBOT=0.030，红嘴尖
在烧杯口上方），本动作只开合夹爪、不移动臂（a3 SqueezeWashBottle 同款）：
  ① 合爪到 WASH_SQUEEZE（0.020）挤压瓶身 → task._update_washbottle 检测
     opening < WASH_SQUEEZE_CLOSED（0.025）→ 发射水流（WaterStream 水滴沿抛物线
     从红嘴尖坠入烧杯口）+ 烧杯内液面（BeakerLiquid）随水流上涨 + 冰块浮起
  ② 松回 GRIP_WASHBOT（0.030）结束挤压 → task 检测 opening 回升 → 停止发射、
     液面定到最终高度（water_added）、冰块浮定

水流/液面/冰块效果由 task 驱动（夹爪开度作触发信号，仿 a3 SqueezeWashBottle），
本动作只负责夹爪开合。
"""
from ._base import BaseMetaAction, grip
from .constants import WASH_SQUEEZE, WASH_SQUEEZE_DWELL, GRIP_WASHBOT


class SqueezeWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            grip(e, WASH_SQUEEZE, WASH_SQUEEZE_DWELL),   # ① 合爪挤压肚子出水（水流/液面/冰块触发）
            grip(e, GRIP_WASHBOT, 20),                   # ② 松回持握开度（水流结束、液面定高）
        ]
