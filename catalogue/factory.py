"""catalogue 动作库的统一注册入口。

39 个 v10 动作的 task/controller 都从这里注册进全局 factory
（factories/task_factory.py、factories/controller_factory.py）。

注册键 = snake_case，必须与对应 config.yaml 里的 `task_type` / `controller_type` 一致
（现有约定：两键同名，如 dissolve / flametest / dropperdrip）。

用法：在 main.py（或测试入口）启动时调用一次 register_catalogue_actions()。
实现到哪个动作，就在 register_catalogue_actions() 里加一条注册调用。
"""

from factories.task_factory import register_task
from factories.controller_factory import register_controller
from catalogue.c_flame.c1_flame_wire_solid.task import FlameTestTask
from catalogue.c_flame.c1_flame_wire_solid.controller import FlameTestTaskController
from catalogue.d_wetchem.d2s_water_solubility.task import D2SWaterSolubilityTask
from catalogue.d_wetchem.d2s_water_solubility.controller import D2SWaterSolubilityTaskController
from catalogue.d_wetchem.d3l_acid_reagent.task import D3LAcidReagentTask
from catalogue.d_wetchem.d3l_acid_reagent.controller import D3LAcidReagentTaskController


def register_catalogue_actions() -> None:
    """注册 catalogue 下所有已实现的动作。

    新增动作时，按以下模板添加（替换 <action> 为动作目录名 / snake_case 注册键）：

    ```python
    # from catalogue.<category>.<action>.task import <Action>Task
    # from catalogue.<category>.<action>.controller import <Action>TaskController
    #
    # register_task("<action>", <Action>Task)
    # register_controller("<action>", <Action>TaskController)
    ```

    注意：flametest→C1/C2、dissolve→D2-S、dropperdrip→D3-D9 模板、heatlamp→B 类、
    ignitelamp→点火前置等**现有实现**已在 factories/*.py 直接注册（键如 "flametest"）。
    catalogue 内为每个动作注册**独立的 snake_case 键**（如 "c1_flame_wire_solid"、
    "d2s_water_solubility"），类复用/新增对应实现，不迁移、不复制逻辑。
    """
    # C1 焰色反应（复用现有 flametest 实现）
    register_task("c1_flame_wire_solid", FlameTestTask)
    register_controller("c1_flame_wire_solid", FlameTestTaskController)

    # D2-S 固体水溶性测试（catalogue 内原生实现：Lula IK + ScoopSample 元动作）
    register_task("d2s_water_solubility", D2SWaterSolubilityTask)
    register_controller("d2s_water_solubility", D2SWaterSolubilityTaskController)

    # D3-L 酸性试剂滴加反应（液体样品，catalogue 内原生：Lula IK + 滴管元动作）
    register_task("d3l_acid_reagent", D3LAcidReagentTask)
    register_controller("d3l_acid_reagent", D3LAcidReagentTaskController)
