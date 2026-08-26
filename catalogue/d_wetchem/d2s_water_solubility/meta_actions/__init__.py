"""D2-S 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。已实现：
  ① PickSpatula —— 横向（水平）夹取药匙（仿 level4_LiquidMixing 横夹烧杯）→
     竖直提起 → 法兰转 -45° → 水平对齐粉堆 x=0.537（⑥ AlignPowderX）→
     竖直降 24.5cm（⑦ LowerPowder，2026-08-24 逐步调至 24.5cm）→ 往 -y 平移 16cm（⑧ ShiftYNeg，2026-08-24 皿+粉 +Y 6.5cm 后改回 16）→
     法兰 -45°→-90° 挖粉（⑨ ScoopUpAction，2026-08-24 新增：只动 joint7 再转 -45°，勺尖从粉丘挖起）→
     竖直抬升到管口上方 14cm（⑩ LiftToTube，锁 x/y、保持朝向，仅 z 升到 1.0993；2026-08-24「第十步抬升再抬升4cm」）→
     往 +y 平移 24cm（⑪ ShiftYPos，锁 x/z、保持朝向；2026-08-24 试管移近侧孔 y=0.241 后回调 32→17cm，再「倒数第二步y再增加1cm」→18cm，再「再多移动6cm」→24cm，再「第11步移动y那一步，减少移动2cm」→22cm，再「第十一步多移动2cm」→24cm）→
     往 +x 平移 11cm（⑫ ShiftXPos，锁 y/z、保持朝向；2026-08-24「最后一步还需要再往前伸到试管口」5cm→12cm，再「最后一步减少2厘米深得有点太靠前」12cm→10cm，2026-08-25「第12步先加1cm」→11cm）→
     法兰回卷 -90°→0°（+90°，2026-08-24「改为法兰旋转到0」）同时往 -y 平移 14cm（⑬ FlangeRollShiftYNeg，2026-08-24 新增：边往-y移动边旋转法兰到-45度，同时开始同时结束；同日旋转量 45→60→90、-Y 5→9→13→15→17cm，2026-08-25「现在只调整最后一步，17cm减少到14cm」→14cm）→
     ⑭ 药匙放回试管架（ReturnSpatula，2026-08-25 用户「现在加动作，把药匙放回试管架」：水平移回架孔上（同时 ORIENT_FWD 调竖直）→降回抓点→松爪→撤离）
     （用户 2026-08-20 重给对齐步骤、
     2026-08-22 追加下降步骤；2026-08-17 曾加"水平往 -X 对齐粉末"、2026-08-20 曾试
     DipToPowder 碰粉，均已删/弃）。
后续按 v11 步骤逐个补：② 舀取粉末 → ③ 加蒸馏水 → ④ 振荡 → （试管留在架上，不放操作位）。
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .pick import PickSpatula
from .return_spatula import ReturnSpatula
from .pick_wash_bottle import PickWashBottle
from .squeeze_water import SqueezeWater
from .return_wash_bottle import ReturnWashBottle
from .tube_shake_pass import TubeShakePass

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake", "PickSpatula", "ReturnSpatula",
           "PickWashBottle", "SqueezeWater", "ReturnWashBottle", "TubeShakePass"]
