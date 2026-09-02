# LabUtopia 动作实现进度（v12 目录 · 27 动作）

> 依据 `LabUtopia_Action_Catalogue_v12.docx`。勾选 = 已完成（`catalogue/` 内有 `task.py` + `controller.py` 且已在 `factory.py` 注册可运行）。
> 「运行参数」= 运行时通过 `--result 参数=值` 传入的实验输入（config 的 `experiment_result` 块）；详解见文末「参数详解」。
> 更新日期：2026-08-26

**完成情况：15 / 27**（含 D9 氧气检验，原 D12 改名补位）

## A. 仪器测量（3）

| 完成 | 编号 | 动作 | 代码目录 | 运行参数（--result） | 备注 |
|---|---|---|---|---|---|
| [x] | A1 | 折光率测量（折光仪） | `a_instrument/a1_refractometer/` | `n_d` | 全自动流程，含屏幕读数动画；缺归瓶/合盖/擦镜纸后段 |
| [ ] | A2 | 旋光仪测量（旋光仪） | `a_instrument/a2_polarimeter/` | — | 复合宏：称量配液+测量；需天平、药匙抖落 |
| [x] | A3 | 电导率测量（电导率仪） | `a_instrument/a3_conductivity/` | `conductivity` | 夹皿倒粉→洗瓶注水→搅拌→提电极；py_compile 绿，运行验证待用户 |

## B. 热操作（5）

| 完成 | 编号 | 动作 | 代码目录 | 运行参数（--result） | 备注 |
|---|---|---|---|---|---|
| [ ] | B1 | 酒精灯加热（固体样品） | `b_thermal/b1_alcohol_heat_solid/` | — | 前缀复用 D2-S 药匙挖粉 |
| [x] | B2 | 酒精灯加热（液体/沸点测定） | `b_thermal/b2_alcohol_heat_liquid/` | `boiling_point` | 挂温度计分两段，段1完成，段2待办 |
| [x] | B3S | 水浴加热（固体熔化） | `b_thermal/b3s_water_bath/` | `sample_phase`, `melt_color` | B3 改名；挖粉→水浴加热→熔化；py_compile+gen verify 绿，运行验证待用户 |
| [x] | B3L | 水浴加热（液体变色） | `b_thermal/b3l_water_bath/` | `before_color`, `liquid_color` | B3 改液体；滴加溶液（动作参考 d3l）→水浴加热→渐变变色（只变色不沸腾）；py_compile+gen verify 绿，运行验证待用户 |
| [x] | B4 | 冰浴/冷却 | `b_thermal/b4_ice_bath/` | `liquid_color`, `crystal_color` | 取试管→浸冰浴→归管+洗瓶注水；py_compile 绿，运行验证待用户 |
| [ ] | B5 | 熔点测定（毛细管法-油浴） | `b_thermal/b5_melting_point/` | — | 最难：研钵+毛细管+橡皮圈，全新器材 |

## C. 焰色反应与燃烧（4）

| 完成 | 编号 | 动作 | 代码目录 | 运行参数（--result） | 备注 |
|---|---|---|---|---|---|
| [x] | C1 | 焰色反应（铂丝蘸取固体） | `c_flame/c1_flame_wire_solid/` | `flame_color` | 已迁入 catalogue（类名 C1FlameWireSolidTask） |
| [ ] | C2 | 焰色反应（隔钴玻璃观察） | `c_flame/c2_cobalt_glass/` | — | 需钴玻璃资产 |
| [ ] | C3 | 燃烧试验（固体样品） | `c_flame/c3_combustion_solid/` | — | 粉入燃烧匙；燃烧匙已有 |
| [ ] | C4 | 燃烧试验（液体样品） | `c_flame/c4_combustion_liquid/` | — | |

## D. 湿化学操作（11）

| 完成 | 编号 | 动作 | 代码目录 | 运行参数（--result） | 备注 |
|---|---|---|---|---|---|
| [x] | D2-S | 固体样品水溶性测试 | `d_wetchem/d2s_water_solubility/` | `solubility`, `liquid_color` | 药匙挖粉完整；三档溶解现象 |
| [x] | D2-L | 液体样品水溶性测试 | `d_wetchem/d2l_water_solubility/` | `mixing`, `sample_color` | 洗瓶滴加水；互溶终点混液颜色 |
| [x] | D3 | 试剂滴加反应（固体样品） | `d_wetchem/d3s_acid_reagent/` | `has_bubbles`, `has_precipitate`, `input_color`, `liquid_color` | 酸滴加已做（D3-S）；**待办：输入液颜色→震荡渐变未在 task 实现，且整目录未提交 git** |
| [x] | D4 | 试剂滴加反应（液体样品） | `d_wetchem/d3l_acid_reagent/` + `d4l_alkali_reagent/` | 酸：`has_bubbles`,`has_precipitate`,`liquid_color`；碱：同 | 酸(D3-L)+碱(D4-L)已做，骨架一致 |
| [x] | D6 | 试纸气体检测（通用） | `d_wetchem/d6_testpaper_gas/` | `gas_result`, `liquid_color` | 4 元动作：润纸/移管/检测/归管 |
| [ ] | D7 | 气体鉴定（导气管通入检测试剂） | `d_wetchem/d7l_organic_qual/` + `d7s_organic_qual/` | — | 碳酸盐产气鉴定为主，频次约30 |
| [ ] | D8 | 多步试剂连续滴加反应 | `d_wetchem/d8l_complex_color/` + `d8s_complex_color/` | — | 原 D9 链式宏（VirtualTube）并入 |
| [x] | D9 | 氧气检验（带火星木条复燃） | `d_wetchem/d9_oxygen_splint/` | `oxygen_result` | 原 D12 改名：摘帽→火柴点灯→木条点燃→甩灭留余烬→悬停管口复燃→归位；py_compile 绿，运行验证待用户 |
| [ ] | D1 | 酸碱滴定 | `d_wetchem/d1_acid_base_titration/` | — | 双臂协同 |
| [ ] | D5 | 蒸馏分离 | `d_wetchem/d5_distillation/` | — | 低优先级 |

> 注：仓库 `d_wetchem/` 内另有 `d12_testpaper_gas2`、`d13_multi_drip`、`d14_virtual_tube` 等目录，为 v10 39 动作时代的占位编号，与 v12 对应关系见下表：

| v12 编号 | v10/仓库目录 |
|---|---|
| D5 蒸馏分离 | `d5_distillation/` |
| （v12 已并入 D6） | `d12_testpaper_gas2/` |
| （v12 已并入 D8） | `d13_multi_drip/`、`d14_virtual_tube/` |

## E. 简单物理检测（4）

| 完成 | 编号 | 动作 | 代码目录 | 运行参数（--result） | 备注 |
|---|---|---|---|---|---|
| [x] | E1 | pH 试纸检测 | `e_physical/e1_ph_testpaper/` | `ph_result` | 试纸预铺白瓷板，4 元动作；三色 pH 变色 |
| [x] | E2 | 磁性检测 | `e_physical/e2_magnetic/` | `magnetic` | 磁铁靠近+药匙铺样（复用挖粉/勺铺） |
| [ ] | E3 | 密度测定 | `e_physical/e3_density/` | — | |
| [ ] | E4 | 称量（独立操作） | `e_physical/e4_weighing/` | — | 分析天平已建；需抖落称量原语 |

---

# 参数详解（已实现实验）

> 运行方式：`python main.py --config-dir catalogue/<动作目录> --config-name config --backend gpu` 或原 `--config-name <键>`，
> 传入结果：`--result 参数=值`（值写回 cfg，task 据此驱动现象/读数；不传则用 default）。
> 另附「行为调参」= 非实验结果、但可调动作节奏的 config 旋钮（`sample_cycles`/`shake_cycles` 等）。

## A1 折光率测量 — `a1_refractometer`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `n_d` | enum | **折光率读数**（nD，折光仪显示屏上显示的值）。运行后屏幕按此档位显示对应数字+进度条动画 | `"1.3300"`/`"1.3610"`/`"1.4000"`/`"1.4600"`，默认 `"1.4000"`（**必须带引号字符串**，YAML 浮点 1.4 会对不上档位前缀） |

行为调参：`grasp_xy_threshold`/`gripper_closed_threshold` 等夹爪检测阈值（一般不用改）。

## B2 酒精灯加热液体（沸点测定）— `b2_alcohol_heat_liquid`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `boiling_point` | number | **测得液体沸点（°C）**。点火后温度按 `heat_rate` 从 `room_temp` 上升，到达该值后保持并沸腾（冒泡/蒸汽），随后记录完成 | 任意数字，默认 `100.0` |

行为调参：`room_temp`(室温起点)/`heat_rate`(升温速度 °C/帧，调大加速演示)/`idle_dwell_frames`(点火前停留)/`ignite_dwell_frames`(点火后到升温)/`boil_dwell_frames`(沸腾保持)/`sample_cycles`(滴液循环遍数)。

## C1 焰色反应 — `c1_flame_wire_solid`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `flame_color` | enum | **焰色反应火焰颜色**（P9 铂丝入外焰灼烧显色时火焰的颜色） | `yellow`(Na黄)/`purple`(K紫)/`green`(Cu绿)/`red`/`orange`(Ca,Sr红)/`blue`，默认 `yellow` |

行为调参：`n_drops`(滴酸滴数)、`ignite/stain/extinguish/drop_dwell_frames`(各事件停留帧数)。

## D2-S 固体样品水溶性 — `d2s_water_solubility`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `solubility` | enum | **溶解情况三档**，决定震荡停后现象分化（先整管浑浊→停震分化） | `soluble`(可溶：浑浊渐澄清+液体变输入色、粉末溶尽)/`insoluble`(不溶：浑浊渐沉淀+粉末留底、液体回水色)/`slightly_soluble`(微溶：粉末留底+液体渐变浅色)，默认 `soluble` |
| `liquid_color` | enum | **粉末/溶解液颜色**（可溶时溶后液体的颜色，兼粉末色） | `white`/`red`/`blue`/`green`/`purple`，默认 `white` |

行为调参：`shake_cycles`(S6 试管震荡来回次数)。

## D2-L 液体样品水溶性 — `d2l_water_solubility`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `mixing` | enum | **震荡后混合情况**，决定分层/互溶/浑浊现象 | `miscible`(互溶：先分层→震荡中层扩散长满全管)/`layered`(不溶分层：震荡后仍上下两层)/`cloudy`(浑浊：震荡中整管白浊→归架后澄清露分层)，默认 `miscible` |
| `sample_color` | enum | **样品液颜色** | `clear`/`red`/`blue`/`green`/`purple`，默认 `blue` |

行为调参：`sample_cycles`(取样滴管滴入遍数)/`wash_cycles`(洗瓶注水挤压次数)/`shake_cycles`(震荡来回次数)。

## D3 试剂滴加反应（固体样品）— `d3s_acid_reagent`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `has_bubbles` | bool | **滴加酸后是否产生气泡** | `true`/`false`，默认 `true` |
| `has_precipitate` | bool | **滴加酸后是否产生沉淀** | `true`/`false`，默认 `true` |
| `input_color` | enum | **输入酸溶液颜色**（滴入试管后、震荡反应前的颜色） | `clear`/`red`/`blue`/`green`/`purple`，默认 `clear`（**待办：task 尚未实现"震荡后液体慢慢变成输出色"的渐变，目前只支持档位直接显示**） |
| `liquid_color` | enum | **震荡反应后液体颜色**（输出色/目标色） | `clear`/`red`/`blue`/`green`/`purple`，默认 `clear` |

行为调参：`acid_cycles`(吸酸→滴入试管循环遍数)/`shake_cycles`(震荡来回次数)。

## D4 试剂滴加反应（液体样品）— `d3l_acid_reagent` + `d4l_alkali_reagent`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `has_bubbles` | bool | **滴加试剂后是否产生气泡** | `true`/`false`，默认 `true` |
| `has_precipitate` | bool | **滴加试剂后是否产生沉淀** | `true`/`false`，默认 `true` |
| `liquid_color` | enum | **滴加试剂后液体变色**（目标色；酸=d3l，碱=d4l 各自读同名字段） | `clear`/`red`/`blue`/`green`/`purple`，默认 `clear` |

行为调参：`sample_cycles`(取样滴管滴入遍数)/`acid_cycles` 或 `alkali_cycles`(加试剂滴入遍数)/`shake_cycles`(震荡来回次数)。

## D6 试纸气体检测（通用）— `d6_testpaper_gas`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `gas_result` | enum | **试纸检测结果**（试纸类型 × 是否变蓝），task 检测时切换湿润端颜色 | `oxidative_blue`(淀粉碘化钾试纸→变蓝，检出氧化性气体)/`oxidative_negative`(不变色)/`alkaline_blue`(红色石蕊试纸→变蓝，检出碱性气体)/`alkaline_negative`(不变色)，默认 `oxidative_blue` |
| `liquid_color` | enum | **试管内预置反应液颜色**（液柱变体，预留接口） | `colorless`/`blue`/`red`/`green`/`yellow`/`purple`，默认 `blue` |

## D9 氧气检验 — `d9_oxygen_splint`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `oxygen_result` | enum | **木条是否复燃（氧气检验结果）**。余烬木条悬停氧气试管口上方，`reignite` 余烬复燃（明火显），`negative` 余烬渐熄（无复燃） | `reignite`/`negative`，默认 `reignite` |

行为调参：`grasp_xy_threshold`/`gripper_closed_threshold` 等夹爪检测阈值（一般不用改）。

## E1 pH 试纸检测 — `e1_ph_testpaper`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `ph_result` | enum | **pH 试纸检测结果（变色档）**，显示试纸中央对应色斑 | `acidic`(酸→红)/`neutral`(中→黄绿)/`alkaline`(碱→蓝)，默认 `neutral` |

## E2 磁性检测 — `e2_magnetic`

| 参数 | 类型 | 含义 | 选项 / 默认 |
|---|---|---|---|
| `magnetic` | enum | **磁性检测结果（是否被磁铁吸引）** | `magnetic`(磁铁靠近→颗粒被吸起)/`non_magnetic`(无动画)，默认 `magnetic` |

---

## 参数类型速查

| 类型 | 说明 | 传入方式示例 |
|---|---|---|
| `bool` | 是否发生 | `--result has_bubbles=yes`（`yes/no`/`true/false`） |
| `enum` | 从固定档位选一 | `--result flame_color=green` |
| `number` | 任意数值 | `--result boiling_point=97` |
