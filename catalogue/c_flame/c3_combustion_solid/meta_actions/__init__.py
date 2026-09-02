"""C3 燃烧试验（固体样品）元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

C3 = 挖粉动作完全复刻 D2-S（横夹药匙 + 挖粉轨迹几何逐字同构），坐标**逐字对齐 d2s**
（用户 2026-09-01 要求，改前自定义坐标致臂卡死）；
6 个挖粉子动作（法兰转/对齐粉堆/下降/平移/挖粉/抬升）直接复用 d2s 元动作包。
已实现：
  ① PickSpatula —— 横夹药匙 → 竖直提起 → 法兰转 -45° → 对齐粉堆 x=0.537 → 竖直降
     → 往 -y 平移 0.16 → 法兰 -45°→-90° 挖粉 → 抬升到安全高位 → 倒燃烧匙前段
     （⑪ 下降 17cm → ⑫ +y 31cm → ⑬ +x 5cm → ⑭ 法兰转竖直 与 -y 18cm 同步【同步结束】，
     2026-09-01 用户追加，同步倒粉+粉末下落动画已接续）
  ② ReturnSpatula —— 倒粉后放回药匙（先原位提 H 调直 → 高位移回架孔 → 竖直下探入孔
     → 松爪 → 撤离；勺尖 0.846 < 架顶 0.917 必须 lift_first，2026-09-01 用户「放回前先
     对准，然后把药匙强制旋转竖直再向下放好」）
  ③ LightFlamePass —— 取火柴 → 高位运移（0.96 绕焰）→ 下探触灯芯点燃酒精灯 → 抬离
     绕焰 → 直退放回（C3 灯在火柴 +X 侧，回程必须抬高，2026-09-01 用户「拿起火柴点燃
     酒精灯」，仿照 C4）
  ④ SpoonToFlamePass —— 横夹燃烧匙把手 → 提出 → 移灯上方 → 碗口下探入外焰停留 →
     放回原位（2026-09-01 用户「拿起燃烧匙移动到外焰上」，仿照 C4；燃烧现象待接续）
  ⑤ CapLampPass —— 取灯帽（灯正北桌面）→ 高位运到灯口上方 → 下扣盖灭火焰（2026-09-02
     用户「拿起酒精灯帽盖上酒精灯熄灭火焰」，仿照 C4/B2）
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .pick import PickSpatula
from .return_spatula import ReturnSpatula
from .light_flame_pass import LightFlamePass
from .spoon_to_flame_pass import SpoonToFlamePass
from .cap_lamp_pass import CapLampPass

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake",
           "PickSpatula", "ReturnSpatula", "LightFlamePass", "SpoonToFlamePass",
           "CapLampPass"]
