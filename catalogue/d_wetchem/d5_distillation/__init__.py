"""D5 蒸馏分离 —— 预组装蒸馏装置，机械臂仅执行加热收集。

文档 D11 注：「装置组装建议人工完成，机械臂仅加热收集」。本包实现：
  task.py / controller.py / meta_actions/（LightFlamePass 点火元动作）。
蒸馏现象（加热→沸腾→冷凝→馏出液收集）由 task 现象状态机驱动。
"""
