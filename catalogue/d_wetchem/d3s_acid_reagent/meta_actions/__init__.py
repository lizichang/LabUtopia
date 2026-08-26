"""D3-S 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

D3-S = D2-S 把「洗瓶蒸馏水」换成「胶头滴管滴加酸性试剂」。挖粉动作（①PickSpatula →
②ReturnSpatula）与试管震荡（④TubeShakePass）**直接复用 d2s 元动作包**（坐标逐字一致，
用户要求机械臂/粉末/试管相对位置都不变，挖粉轨迹才不跑偏）；酸滴管（③AcidPass）照
d3l acid_pass 单段滴加 + B2 水平横夹（ORIENT_FWD）新写。已实现：
  ① PickSpatula —— 复用 d2s（横向夹药匙 → 竖直提 → 法兰转 → 对齐粉堆 → 下探挖粉
     → 抬升 → 平移 → 回卷倒粉入管），坐标不动
  ② ReturnSpatula —— 复用 d2s（药匙放回试管架）
  ③ AcidPass —— 抓酸滴管（水平横夹）→ 吸酸 → 滴入试管（本包新写）
  ④ TubeShakePass —— 复用 d2s（拿起试管震荡混合，现象消退）
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .acid_pass import AcidPass
# 挖粉/放回药匙/试管震荡 直接复用 d2s 元动作包（药匙/皿/粉/试管/试管架坐标逐字一致）
from catalogue.d_wetchem.d2s_water_solubility.meta_actions import (PickSpatula, ReturnSpatula,
                                                                   TubeShakePass)

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake",
           "PickSpatula", "ReturnSpatula", "AcidPass", "TubeShakePass"]
