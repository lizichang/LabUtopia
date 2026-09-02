"""C1 焰色反应（铂丝蘸取固体样品）。

task.py / controller.py / meta_actions/ 已从旧 flametest 实现迁入本目录，
类名 C1FlameWireSolidTask / C1FlameWireSolidTaskController；
共享 IK 基础设施仍在 controllers/atomic_actions/flametest/ 与
controllers/flametest_meta_actions/_base.py（约 20 个实验共用，未迁移）。

运行：python main.py --config-dir config --config-name level2_C1FlameWireSolid --backend gpu
"""
