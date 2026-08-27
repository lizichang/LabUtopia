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
from catalogue.d_wetchem.d3s_acid_reagent.task import D3SAcidReagentTask
from catalogue.d_wetchem.d3s_acid_reagent.controller import D3SAcidReagentTaskController
from catalogue.d_wetchem.d4s_alkali_reagent.task import D4SAlkaliReagentTask
from catalogue.d_wetchem.d4s_alkali_reagent.controller import D4SAlkaliReagentTaskController
from catalogue.d_wetchem.d4l_alkali_reagent.task import D4LAlkaliReagentTask
from catalogue.d_wetchem.d4l_alkali_reagent.controller import D4LAlkaliReagentTaskController
from catalogue.d_wetchem.d2l_water_solubility.task import D2LWaterSolubilityTask
from catalogue.d_wetchem.d2l_water_solubility.controller import D2LWaterSolubilityTaskController
from catalogue.b_thermal.b1_alcohol_heat_solid.task import B1AlcoholHeatSolidTask
from catalogue.b_thermal.b1_alcohol_heat_solid.controller import B1AlcoholHeatSolidTaskController
from catalogue.b_thermal.b2_alcohol_heat_liquid.task import B2AlcoholHeatLiquidTask
from catalogue.b_thermal.b2_alcohol_heat_liquid.controller import B2AlcoholHeatLiquidTaskController
from catalogue.e_physical.e1_ph_testpaper.task import E1PhTestpaperTask
from catalogue.e_physical.e1_ph_testpaper.controller import E1PhTestpaperTaskController
from catalogue.e_physical.e2_magnetic.task import E2MagneticTask
from catalogue.e_physical.e2_magnetic.controller import E2MagneticTaskController
from catalogue.a_instrument.a1_refractometer.task import A1RefractometerTask
from catalogue.a_instrument.a1_refractometer.controller import A1RefractometerTaskController
from catalogue.a_instrument.a2_polarimeter.task import A2PolarimeterTask
from catalogue.a_instrument.a2_polarimeter.controller import A2PolarimeterTaskController
from catalogue.d_wetchem.d6_testpaper_gas.task import D6TestpaperGasTask
from catalogue.d_wetchem.d6_testpaper_gas.controller import D6TestpaperGasTaskController
from catalogue.d_wetchem.d7_gas_identification.task import D7GasIdentificationTask
from catalogue.d_wetchem.d7_gas_identification.controller import D7GasIdentificationTaskController


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

    # D2-S 固体水溶性测试（catalogue 内原生实现：Lula IK + PickSpatula 元动作）
    register_task("d2s_water_solubility", D2SWaterSolubilityTask)
    register_controller("d2s_water_solubility", D2SWaterSolubilityTaskController)

    # D3-L 酸性试剂滴加反应（液体样品，catalogue 内原生：Lula IK + 滴管元动作）
    register_task("d3l_acid_reagent", D3LAcidReagentTask)
    register_controller("d3l_acid_reagent", D3LAcidReagentTaskController)

    # D3-S 固体样品 + 酸性试剂滴加反应（D2-S 挖粉 + 酸滴管滴酸 + 试管震荡，药匙/粉/管坐标同 d2s）
    register_task("d3s_acid_reagent", D3SAcidReagentTask)
    register_controller("d3s_acid_reagent", D3SAcidReagentTaskController)

    # D4-S 固体样品 + 碱性试剂滴加反应（d3s 骨架逐字复制，仅碱瓶替换酸瓶：alkaline_bottle.usd
    # 塑料瓶 + 橡胶塞翻放桌面；碱滴管第一列第5排，碱瓶 (0.370,0.30)）
    register_task("d4s_alkali_reagent", D4SAlkaliReagentTask)
    register_controller("d4s_alkali_reagent", D4SAlkaliReagentTaskController)

    # D4-L 碱性试剂滴加反应（碱瓶替换酸瓶 + 橡胶塞动态拔/盖倒放 + 两支滴管滴加）
    register_task("d4l_alkali_reagent", D4LAlkaliReagentTask)
    register_controller("d4l_alkali_reagent", D4LAlkaliReagentTaskController)

    # D2-L 液体样品水溶性测试（取样滴管吸样→滴入试管；v1 先做第一步，注水/震荡后续补）
    register_task("d2l_water_solubility", D2LWaterSolubilityTask)
    register_controller("d2l_water_solubility", D2LWaterSolubilityTaskController)

    # B1 酒精灯加热（固体样品）：本批次三个过程（药匙挖粉倒粉 → 开灯帽放一边 → 取火柴点燃）；
    # 拿试管→外焰预热→集中加热→熄灭→归位留待验收后接续
    register_task("b1_alcohol_heat_solid", B1AlcoholHeatSolidTask)
    register_controller("b1_alcohol_heat_solid", B1AlcoholHeatSolidTaskController)

    # B2 沸点测定（酒精灯加热试管液体 → 温度计读数 → 沸腾 → 记录沸点；v1 自动观测，无机械臂元动作）
    register_task("b2_alcohol_heat_liquid", B2AlcoholHeatLiquidTask)
    register_controller("b2_alcohol_heat_liquid", B2AlcoholHeatLiquidTaskController)

    # E1 pH 试纸检测（catalogue 内原生：Lula IK + 平移跟随元动作 + pH 色斑变色接口）
    register_task("e1_ph_testpaper", E1PhTestpaperTask)
    register_controller("e1_ph_testpaper", E1PhTestpaperTaskController)

    # E2 磁性检测（catalogue 内原生：Lula IK + 药匙舀粉倒粉 + 磁铁检测 + 磁性颗粒吸起动画）
    register_task("e2_magnetic", E2MagneticTask)
    register_controller("e2_magnetic", E2MagneticTaskController)

    # A1 折光率测量（catalogue 内原生：Lula IK + 取瓶塞直拔 + 滴管吸样滴样到棱镜）
    register_task("a1_refractometer", A1RefractometerTask)
    register_controller("a1_refractometer", A1RefractometerTaskController)

    # A2 旋光仪测量（catalogue 内原生：Lula IK + 洗瓶注水 + 试管震荡溶解 + 倒液进旋光管
    # + 放导轨 + 按启动键读旋光角；10 元动作，屏幕读数按档位预烘焙切显）
    register_task("a2_polarimeter", A2PolarimeterTask)
    register_controller("a2_polarimeter", A2PolarimeterTaskController)

    # D6 试纸气体检测（通用；catalogue 内原生：试纸夹预夹 + 滴管润湿 + 移试管观察）
    register_task("d6_testpaper_gas", D6TestpaperGasTask)
    register_controller("d6_testpaper_gas", D6TestpaperGasTaskController)

    # D7 气体鉴定（catalogue 内原生：导气管橡皮塞 + 检验试管下浸通气，检测液仅初始颜色入口）
    register_task("d7_gas_identification", D7GasIdentificationTask)
    register_controller("d7_gas_identification", D7GasIdentificationTaskController)
