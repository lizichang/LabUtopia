"""B5 元动作共享基础：复用 flametest 原子动作工厂（mv/grip/hold/shake）。"""
from controllers.flametest_meta_actions._base import BaseMetaAction, mv, grip, hold, shake

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake"]
