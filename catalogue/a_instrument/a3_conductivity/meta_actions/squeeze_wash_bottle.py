# -*- coding: utf-8 -*-
"""A3 ⑥ 挤压洗瓶身出水（水从红嘴弧线落入烧杯，烧杯内液面上涨）。

2026-08-30 用户（逐字）：「增加挤液体的动作，注意要精致，是一个水流从红嘴处出来弧线
落入烧杯，烧杯里面页面上涨」。

PickWashBottle（⑤）结束夹爪持瓶肚子停在挤水位（持握开度 GRIP_WASHBOT=0.030，红嘴尖
(0.3650,0.1062,0.994) 在烧杯左壁外侧 1cm、烧杯口 (0.4120,0.0807,0.8904) 左上方），
本动作只开合夹爪、不移动臂（d2s SqueezeWater 同款）：
  ① 合爪到 WASH_SQUEEZE（0.020）挤压瓶身 → task._update_washbottle 检测
     opening < WASH_SQUEEZE_CLOSED（0.025）→ 发射水流（WaterStream 水滴沿抛物线
     从红嘴尖坠入烧杯口）+ 烧杯内液面（BeakerLiquid）随水流上涨
  ② 松回 GRIP_WASHBOT（0.030）结束挤压 → task 检测 opening 回升 → 停止发射、
     液面定到最终高度（water_added）

水流/液面效果由 task 驱动（夹爪开度作触发信号，仿 d2s SqueezeWater），本动作只负责夹爪开合。
"""
from ._base import BaseMetaAction, grip
from .constants import WASH_SQUEEZE, WASH_SQUEEZE_DWELL, GRIP_WASHBOT


class SqueezeWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        return [
            grip(e, WASH_SQUEEZE, WASH_SQUEEZE_DWELL),   # ① 合爪挤压肚子出水（水流/液面效果触发）
            grip(e, GRIP_WASHBOT, 20),                    # ② 松回持握开度（水流结束、液面定高）
        ]
