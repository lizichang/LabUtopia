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

    例如 D2-S 水溶性若在 catalogue 内重实现（当前映射到现有 dissolve，不迁移）：
    ```python
    # from catalogue.d_wetchem.d2s_water_solubility.task import D2SWaterSolubilityTask
    # from catalogue.d_wetchem.d2s_water_solubility.controller import D2SWaterSolubilityTaskController
    # register_task("d2s_water_solubility", D2SWaterSolubilityTask)
    # register_controller("d2s_water_solubility", D2SWaterSolubilityTaskController)
    ```

    注意：现有已实现动作（flametest→C1/C2、dissolve→D2-S、dropperdrip→D3-D9 模板、
    heatlamp→B 类、ignitelamp→点火前置）已在 factories/*.py 直接注册，catalogue 内
    不做重复注册、不迁移。
    """
    # —— 已实现动作的注册集中在这里，逐条追加 ——
    pass
