"""A2 旋光仪测量（复合宏）：试管预装粉 → 洗瓶加水 → 震荡溶解 → 倒进旋光管 → 放导轨
→ 按启动键读旋光角。

- task.py: A2PolarimeterTask（洗瓶注水 + 试管两周期持握（震荡/倒液）+ 旋光管平移 + 屏幕读数）
- controller.py: A2PolarimeterTaskController（10 元动作顺序编排 + 按钮读数 gate）
- meta_actions/: 10 个元动作（PickWashBottle…PressStartPass）
"""
