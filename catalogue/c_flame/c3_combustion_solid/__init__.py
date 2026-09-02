"""C3 燃烧试验（固体样品）。

药匙挖粉 → 倒燃烧匙 → 夹燃烧匙（柄） → 火柴点燃酒精灯 → 燃烧匙碗入外焰加热
→ 固体燃烧现象（发光/冒烟/灰化） → 移灯 → 盖帽熄火 → 归位。

当前进度：场景 + 最小 task/controller 骨架（仅供 --snapshot 查看场景布局，
验证燃烧匙 assets 渲染可见性）。动作序列（挖粉/倒粉/夹燃烧匙/点火/燃烧现象）
留待验收后接续。

task.py / controller.py / config/level2_C3CombustionSolid.yaml 已实现并注册
（factory 键 c3_combustion_solid）；场景 assets/scenes/c_flame/c3_combustion_solid/。

运行：python main.py --config-dir config --config-name level2_C3CombustionSolid --backend gpu
"""
