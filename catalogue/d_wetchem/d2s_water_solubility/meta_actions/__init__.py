"""D2-S 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

与 flametest 同构：controller 实例化这些元动作按序执行，每个元动作组合
atomic_actions/flametest 的 IK 原子动作。已实现：
  ① PickSpatula —— 横向（水平）夹取药匙（仿 level4_LiquidMixing 横夹烧杯）→
     竖直提起 → 法兰转 -45° → 水平对齐粉堆 x=0.537（⑥ AlignPowderX）→
     竖直降 24.5cm（⑦ LowerPowder，2026-08-24 逐步调至 24.5cm）→ 往 -y 平移 16cm（⑧ ShiftYNeg，2026-08-24 皿+粉 +Y 6.5cm 后改回 16）→
     法兰 -45°→-90° 挖粉（⑨ ScoopUpAction，2026-08-24 新增：只动 joint7 再转 -45°，勺尖从粉丘挖起）→
     竖直抬升到管口上方 10cm（⑩ LiftToTube，锁 x/y、保持朝向，仅 z 升到 1.0593）→
     往 +y 平移 18cm（⑪ ShiftYPos，锁 x/z、保持朝向；2026-08-24 试管移近侧孔 y=0.241 后回调 32→17cm，再「倒数第二步y再增加1cm」→18cm）→
     往 +x 平移 10cm（⑫ ShiftXPos，锁 y/z、保持朝向；2026-08-24「最后一步还需要再往前伸到试管口」5cm→12cm，再「最后一步减少2厘米深得有点太靠前」12cm→10cm）——本阶段到此结束
     （用户 2026-08-20 重给对齐步骤、
     2026-08-22 追加下降步骤；2026-08-17 曾加"水平往 -X 对齐粉末"、2026-08-20 曾试
     DipToPowder 碰粉，均已删/弃）。
后续按 v11 步骤逐个补：② 舀取粉末 → ③ 加蒸馏水 → ④ 振荡 → （试管留在架上，不放操作位）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .pick import PickSpatula

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "PickSpatula"]
