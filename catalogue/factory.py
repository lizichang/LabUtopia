"""catalogue 动作库的统一注册入口。

39 个 v10 动作的 task/controller 都从这里注册进全局 factory
（factories/task_factory.py、factories/controller_factory.py）。

注册键 = snake_case，必须与对应 config.yaml 里的 `task_type` / `controller_type` 一致
（现有约定：两键同名，如 dropperdrip）。

用法：在 main.py（或测试入口）启动时调用一次 register_catalogue_actions()。
实现到哪个动作，就在 register_catalogue_actions() 里加一条注册调用。
"""

from factories.task_factory import register_task
from factories.controller_factory import register_controller
from catalogue.c_flame.c1_flame_wire_solid.task import C1FlameWireSolidTask
from catalogue.c_flame.c1_flame_wire_solid.controller import C1FlameWireSolidTaskController
from catalogue.c_flame.c2_cobalt_glass.task import C2CobaltGlassTask
from catalogue.c_flame.c2_cobalt_glass.controller import C2CobaltGlassTaskController
from catalogue.c_flame.c3_combustion_solid.task import C3CombustionSolidTask
from catalogue.c_flame.c3_combustion_solid.controller import C3CombustionSolidTaskController
from catalogue.c_flame.c4_combustion_liquid.task import C4CombustionLiquidTask
from catalogue.c_flame.c4_combustion_liquid.controller import C4CombustionLiquidTaskController
from catalogue.d_wetchem.d2s_water_solubility.task import D2SWaterSolubilityTask
from catalogue.d_wetchem.d2s_water_solubility.controller import D2SWaterSolubilityTaskController
from catalogue.d_wetchem.d1_acid_base_titration.task import D1AcidBaseTitrationTask
from catalogue.d_wetchem.d1_acid_base_titration.controller import D1AcidBaseTitrationTaskController
from catalogue.d_wetchem.d3l_acid_reagent.task import D3LAcidReagentTask
from catalogue.d_wetchem.d3l_acid_reagent.controller import D3LAcidReagentTaskController
from catalogue.d_wetchem.d3s_acid_reagent.task import D3SAcidReagentTask
from catalogue.d_wetchem.d3s_acid_reagent.controller import D3SAcidReagentTaskController
from catalogue.d_wetchem.d4s_alkali_reagent.task import D4SAlkaliReagentTask
from catalogue.d_wetchem.d4s_alkali_reagent.controller import D4SAlkaliReagentTaskController
from catalogue.d_wetchem.d4l_alkali_reagent.task import D4LAlkaliReagentTask
from catalogue.d_wetchem.d4l_alkali_reagent.controller import D4LAlkaliReagentTaskController
from catalogue.d_wetchem.d8l_complex_color.task import D8LComplexColorTask
from catalogue.d_wetchem.d8l_complex_color.controller import D8LComplexColorTaskController
from catalogue.d_wetchem.d2l_water_solubility.task import D2LWaterSolubilityTask
from catalogue.d_wetchem.d2l_water_solubility.controller import D2LWaterSolubilityTaskController
from catalogue.b_thermal.b1_alcohol_heat_solid.task import B1AlcoholHeatSolidTask
from catalogue.b_thermal.b1_alcohol_heat_solid.controller import B1AlcoholHeatSolidTaskController
from catalogue.b_thermal.b2_alcohol_heat_liquid.task import B2AlcoholHeatLiquidTask
from catalogue.b_thermal.b2_alcohol_heat_liquid.controller import B2AlcoholHeatLiquidTaskController
from catalogue.b_thermal.b3s_water_bath.task import B3SWaterBathTask
from catalogue.b_thermal.b3s_water_bath.controller import B3SWaterBathTaskController
from catalogue.b_thermal.b3l_water_bath.task import B3LWaterBathTask
from catalogue.b_thermal.b3l_water_bath.controller import B3LWaterBathTaskController
from catalogue.b_thermal.b4_ice_bath.task import B4IceBathTask
from catalogue.b_thermal.b4_ice_bath.controller import B4IceBathTaskController
from catalogue.b_thermal.b5_melting_point.task import B5MeltingPointTask
from catalogue.b_thermal.b5_melting_point.controller import B5MeltingPointTaskController
from catalogue.e_physical.e1_ph_testpaper.task import E1PhTestpaperTask
from catalogue.e_physical.e1_ph_testpaper.controller import E1PhTestpaperTaskController
from catalogue.e_physical.e2_magnetic.task import E2MagneticTask
from catalogue.e_physical.e2_magnetic.controller import E2MagneticTaskController
from catalogue.e_physical.e3_density.task import E3DensityTask
from catalogue.e_physical.e3_density.controller import E3DensityTaskController
from catalogue.a_instrument.a1_refractometer.task import A1RefractometerTask
from catalogue.a_instrument.a1_refractometer.controller import A1RefractometerTaskController
from catalogue.a_instrument.a2_polarimeter.task import A2PolarimeterTask
from catalogue.a_instrument.a2_polarimeter.controller import A2PolarimeterTaskController
from catalogue.a_instrument.a3_conductivity.task import A3ConductivityTask
from catalogue.a_instrument.a3_conductivity.controller import A3ConductivityTaskController
from catalogue.d_wetchem.d6_testpaper_gas.task import D6TestpaperGasTask
from catalogue.d_wetchem.d6_testpaper_gas.controller import D6TestpaperGasTaskController
from catalogue.d_wetchem.d7_gas_identification.task import D7GasIdentificationTask
from catalogue.d_wetchem.d7_gas_identification.controller import D7GasIdentificationTaskController
from catalogue.d_wetchem.d9_oxygen_splint.task import D9OxygenSplintTask
from catalogue.d_wetchem.d9_oxygen_splint.controller import D9OxygenSplintTaskController
from catalogue.d_wetchem.d5_distillation.task import D5DistillationTask
from catalogue.d_wetchem.d5_distillation.controller import D5DistillationTaskController


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

    注意：dissolve→D2-S、dropperdrip→D3-D9 模板、heatlamp→B 类、ignitelamp→点火前置
    等**现有实现**仍在 factories/*.py 直接注册（键如 "dropperdrip"）。C1 焰色反应已从
    factories/*.py 的 "flametest" 键迁入本文件（键 "c1_flame_wire_solid"，
    类 C1FlameWireSolidTask）。catalogue 内为每个动作注册**独立的 snake_case 键**
    （如 "c1_flame_wire_solid"、"d2s_water_solubility"），类复用/新增对应实现，
    不迁移、不复制逻辑。
    """
    # C1 焰色反应（铂丝蘸取固体，已迁入 catalogue，类名 C1FlameWireSolidTask）
    register_task("c1_flame_wire_solid", C1FlameWireSolidTask)
    register_controller("c1_flame_wire_solid", C1FlameWireSolidTaskController)

    # C2 焰色反应（隔钴玻璃观察）：C1 完整灼烧流程 + 固定钴玻璃（机械臂不抓），
    # camera2 从 +Y 朝 -Y 透过玻璃看火焰；task 受染后追加两步判读（直接黄焰→隔玻璃紫/无色）
    register_task("c2_cobalt_glass", C2CobaltGlassTask)
    register_controller("c2_cobalt_glass", C2CobaltGlassTaskController)

    # C3 燃烧试验（固体样品）：药匙挖粉倒燃烧匙 → 夹燃烧匙入焰 → 火柴点燃 → 燃烧现象 → 熄火；
    # 当前为最小骨架（仅供 --snapshot 查看场景布局，验证燃烧匙 assets 渲染可见性）
    register_task("c3_combustion_solid", C3CombustionSolidTask)
    register_controller("c3_combustion_solid", C3CombustionSolidTaskController)

    # C4 燃烧试验（液体样品）：滴管吸药品瓶液 → 滴入燃烧匙碗（d3l 同款滴加生命周期）；
    # 点火/燃烧/熄火留待验收滴加动作后接续
    register_task("c4_combustion_liquid", C4CombustionLiquidTask)
    register_controller("c4_combustion_liquid", C4CombustionLiquidTaskController)

    # D2-S 固体水溶性测试（catalogue 内原生实现：Lula IK + PickSpatula 元动作）
    register_task("d2s_water_solubility", D2SWaterSolubilityTask)
    register_controller("d2s_water_solubility", D2SWaterSolubilityTaskController)

    # D1 酸碱滴定（单臂顺序改编）：P1 加指示剂——滴管吸酚酞→滴入锥形瓶 W 无色→粉→放回；
    # P2 搬瓶 W→酸式滴定管下、P3 滴定循环、P4 终点读数后续补
    register_task("d1_acid_base_titration", D1AcidBaseTitrationTask)
    register_controller("d1_acid_base_titration", D1AcidBaseTitrationTaskController)

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

    # D8-L 络合/显色试剂滴加反应（复刻 d3l 模板扩为 3 滴管 + 3 瓶；3 段变色 + 沉淀 + 分层 + 气泡）
    register_task("d8l_complex_color", D8LComplexColorTask)
    register_controller("d8l_complex_color", D8LComplexColorTaskController)

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

    # B3S 水浴加热（固体）：酒精灯加热烧杯水浴 → 试管内固体熔化/不熔化；机械臂只做三件事：
    # 药匙挖粉倒粉入试管→点燃→拿试管浸水浴加热→放回→移灯→盖帽灭火
    register_task("b3s_water_bath", B3SWaterBathTask)
    register_controller("b3s_water_bath", B3SWaterBathTaskController)

    # B3L 水浴加热（液体）：B3S 换液体样品（动作参考 d3l）——滴管滴加溶液入试管（替代挖粉）
    # → 点燃→拿试管浸水浴加热（不松爪）→ 液体渐变变色（before_color/liquid_color 双输入，
    # 只变色不沸腾）→ 放回→移灯 +Y→盖帽灭火
    register_task("b3l_water_bath", B3LWaterBathTask)
    register_controller("b3l_water_bath", B3LWaterBathTaskController)

    # B4 冰浴/冷却（v1 场景预览骨架：烧杯内装 6 冰块 + 洗瓶正-X + 试管架带药品试管；
    # 机械臂动作【取试管→插入冰浴】后续接续，现仅供 --snapshot 查看场景布局）
    register_task("b4_ice_bath", B4IceBathTask)
    register_controller("b4_ice_bath", B4IceBathTaskController)

    # B5 熔点测定（提勒管法）：第一个元动作「拿起毛细管」（照 b2 火柴 LightFlamePass 同款）；
    # 装样/挂温度计/入提勒管加热/观察熔点留待验收后接续
    register_task("b5_melting_point", B5MeltingPointTask)
    register_controller("b5_melting_point", B5MeltingPointTaskController)

    # E1 pH 试纸检测（catalogue 内原生：Lula IK + 平移跟随元动作 + pH 色斑变色接口）
    register_task("e1_ph_testpaper", E1PhTestpaperTask)
    register_controller("e1_ph_testpaper", E1PhTestpaperTaskController)

    # E2 磁性检测（catalogue 内原生：Lula IK + 药匙舀粉倒粉 + 磁铁检测 + 磁性颗粒吸起动画）
    register_task("e2_magnetic", E2MagneticTask)
    register_controller("e2_magnetic", E2MagneticTaskController)

    # E3 密度测定（catalogue 内原生：Lula IK + 移液管竖直平移跟随；v2 加天平完整测密度
    # ρ=Δm/5mL——量筒预置天平称盘上不动、样品瓶预开盖；4 元动作
    # PickPipette→DrawPipette→TransferPipette→ReturnPipette；液色变色接口 liquid_color
    # 6 色变体 + 密度接口 density 5 档（天平屏 m2+ρ 预烘焙贴图切显））
    register_task("e3_density", E3DensityTask)
    register_controller("e3_density", E3DensityTaskController)

    # A1 折光率测量（catalogue 内原生：Lula IK + 取瓶塞直拔 + 滴管吸样滴样到棱镜）
    register_task("a1_refractometer", A1RefractometerTask)
    register_controller("a1_refractometer", A1RefractometerTaskController)

    # A2 旋光仪测量（catalogue 内原生：Lula IK + 洗瓶注水 + 试管震荡溶解 + 倒液进旋光管
    # + 放导轨 + 按启动键读旋光角；10 元动作，屏幕读数按档位预烘焙切显）
    register_task("a2_polarimeter", A2PolarimeterTask)
    register_controller("a2_polarimeter", A2PolarimeterTaskController)

    # A3 电导率测量（catalogue 内原生：Lula IK + 竖直夹皿元动作；v1 第一步=竖直夹住
    # 玻璃皿提起来，称量配液/电极浸入/读数后续追加）
    register_task("a3_conductivity", A3ConductivityTask)
    register_controller("a3_conductivity", A3ConductivityTaskController)

    # D6 试纸气体检测（通用；catalogue 内原生：试纸夹预夹 + 滴管润湿 + 移试管观察）
    register_task("d6_testpaper_gas", D6TestpaperGasTask)
    register_controller("d6_testpaper_gas", D6TestpaperGasTaskController)

    # D7 气体鉴定（catalogue 内原生：导气管橡皮塞 + 检验试管下浸通气，检测液仅初始颜色入口）
    register_task("d7_gas_identification", D7GasIdentificationTask)
    register_controller("d7_gas_identification", D7GasIdentificationTaskController)

    # D9 氧气检验（catalogue 内原生：带火星木条悬停氧气试管口上方复燃；7 元动作
    # 摘帽→火柴点灯→夹木条→木条点燃→摆动熄火留余烬→悬停管口复燃→归位）
    register_task("d9_oxygen_splint", D9OxygenSplintTask)
    register_controller("d9_oxygen_splint", D9OxygenSplintTaskController)

    # D5 蒸馏分离（catalogue 内原生：预组装蒸馏装置，机械臂仅 LightFlamePass 点火，
    # 蒸馏现象 加热→沸腾→冷凝→馏出液收集 由 task 现象状态机驱动，phase=="done" 报成功）
    register_task("d5_distillation", D5DistillationTask)
    register_controller("d5_distillation", D5DistillationTaskController)
