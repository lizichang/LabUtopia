"""C4 燃烧试验（液体样品）。

滴管从药品瓶吸液体样品 → 滴入燃烧匙碗（d3l 同款滴加生命周期）→（后续）火柴点燃
酒精灯 → 燃烧匙碗伸入火焰 → 液体燃烧现象 → 移灯/盖帽熄火。

当前进度：场景已定稿（C3 燃烧骨架无铁架台 + d3l 液体样品区）+ 滴加动作已实现
（DripSpoonPass：抓滴管 → 药品瓶吸液 → 燃烧匙碗口上方挤胶头 → 成串滴入碗内，
碗液 SpoonLiquid 与瓶液同色逐滴生长）。点火/燃烧/熄火留待验收后接续。

task.py / controller.py / config/level2_C4CombustionLiquid.yaml 已实现并注册
（factory 键 c4_combustion_liquid）；场景 assets/scenes/c_flame/c4_combustion_liquid/。

运行：python main.py --config-dir config --config-name level2_C4CombustionLiquid --backend gpu
"""
