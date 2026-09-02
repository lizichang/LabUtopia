"""C2 焰色反应（隔钴玻璃观察）。

C1 完整灼烧流程（铂丝蘸取固体样品）+ 固定钴玻璃（机械臂不抓）。task/controller
逐字复用 C1 实现，task 仅在受染后追加两步判读：直接黄焰 → 隔钴玻璃切紫/无色。

task.py / controller.py / config/level2_C2CobaltGlass.yaml 已实现并注册
（factory 键 c2_cobalt_glass）；场景 assets/scenes/c_flame/c2_cobalt_glass/。

运行：python main.py --config-dir config --config-name level2_C2CobaltGlass --backend gpu
"""
