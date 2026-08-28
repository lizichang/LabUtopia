# -*- coding: utf-8 -*-
"""A2 ① 预摆位：把臂移到 d2s 洗瓶入口姿势，再进洗瓶流程（乱动修复）。

根因（2026-08-27 用户「虽然没有旋转一大圈但是还是乱动，坐标都一样你就不能直接照搬过来吗」）：
A2 的 PickWashBottle 是 controller 第 1 个动作（sim 冷启动单次 IK 直接到洗瓶上方），
d2s 的是第 3 个动作（从 ReturnSpatula 终点 = 药匙架高位 (0.6993,0.3608,1.15) + ORIENT_FWD
进入）。冷启动直接到洗瓶上方会落 IK 另一解分支 → 送红嘴 -Y linewalk 中途 FK 朝向不合格
保持/跳变 = 用户看到的乱动。先预摆到 d2s 同款入臂姿势（该点 d2s 已证可达 + 朝向），
再按 d2s 顺序走洗瓶，IK 分支连续、送嘴走直线。
"""
from ._base import BaseMetaAction, mv
from .constants import H, ORIENT_FWD, PREPOSE_XY


class PrePoseWash(BaseMetaAction):
    def _build_actions(self):
        return [
            mv(self.engine, (PREPOSE_XY[0], PREPOSE_XY[1], H), orient=ORIENT_FWD),
        ]
