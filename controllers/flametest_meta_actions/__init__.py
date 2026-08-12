"""焰色反应 10 个元动作：每个 = 一类一文件的独立模块。

控制器 flametest_controller.py 实例化这些元动作并按序执行（整个实验）。
每个元动作组合 atomic_actions/flametest 的原子动作（IK 驱动），
贴合物理规律：无瞬移、无空抓、无悬空、无乱动。
"""
from .open_hcl_stopper import OpenHclStopper
from .drip_hcl_acid import DripHclAcid
from .ignite_lamp import IgniteLamp
from .dip_wire_acid import DipWireAcid
from .burn_clean import BurnClean
from .repeat_dip_burn import RepeatDipBurn
from .cool import Cool
from .dip_powder import DipPowder
from .burn_stain import BurnStain
from .extinguish import Extinguish

__all__ = [
    "OpenHclStopper", "DripHclAcid", "IgniteLamp", "DipWireAcid",
    "BurnClean", "RepeatDipBurn", "Cool", "DipPowder", "BurnStain",
    "Extinguish",
]
