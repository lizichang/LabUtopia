"""C2 焰色反应（隔钴玻璃观察）控制器。

C1 控制器逐字复用（10 元动作完全相同：开瓶塞→滴盐酸→点燃→蘸酸灼烧→反复→冷却→
蘸粉末→受染灼烧→盖灭）。钴玻璃是固定静态器材（机械臂不抓），故动作序列与 C1 完全
一致，无需新元动作。唯一差异是语言指令改为提示透过钴玻璃观察火焰颜色。
"""
from catalogue.c_flame.c1_flame_wire_solid.controller import C1FlameWireSolidTaskController


class C2CobaltGlassTaskController(C1FlameWireSolidTaskController):
    """Composite controller: C2 = C1 的 10 元动作 + 静态钴玻璃（无抓取）。"""

    def get_language_instruction(self):
        return ("Open the dilute hydrochloric acid bottle, drip 2-3 drops with the "
                "dropper onto the watch glass, ignite the alcohol lamp with a match, "
                "dip the platinum wire in the acid, burn it in the lamp flame 3-4 "
                "times until no characteristic color, cool for 5 s, dip the solid "
                "sample powder, burn for 2-5 s to observe the flame color directly, "
                "then observe the flame through the cobalt glass and extinguish the "
                "flame with the cap, rinse the wire and return it")
