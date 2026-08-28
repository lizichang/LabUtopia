"""元动作：拿起酒精灯盖放到一边（B1 点火前开盖，2026-08-27 用户指定三过程之二）。

用户逐字：「拿起酒精灯盖儿放到一边儿」——从酒精灯口取下钟罩灯帽，横移到台面
右前区放一边（帽底贴台面，方便稍后机械臂拿试管时不会碰倒）。

灯帽几何（2026-08-27 pxr 实测）：Ø37.2mm×31mm 钟罩盖在灯口，世界中心
(0.50,0.0029,0.8917)；lamp 恒等旋转，帽 local translate=0 即在灯口。帽 ops =
translate + rotateXYZ(90 绕 X) + scale(0.01)（gen_b1_scene 烘焙），mesh 中心 local
(0,9.152,0) → 世界帽中心 = 灯原心 + (0,0,0.09152)。故帽相对灯原点纯 +Z 平移恒定，
**持握 = 纯平移**（帽中心 = 夹爪，task._set_cap_center 写帽 local translate =
center − CAP_CENTER_REST，只动 translate op，姿态恒等）。这与滴管/药匙的矩阵持握
不同——帽中心就是夹爪，无需 _T_HELD 偏移。

抓取三问：① 抓帽壁（夹爪落帽中心 TCP，两指在 y=±0.0186 夹 Ø37.2 帽壁）——
② 只有夹住帽壁才能整体端走；帽壁在空心灯口内、玻璃壁 y=±0.0435 之内，手指
开 ±0.04 不碰玻璃。③ GRIP_CAP=Ø37.2/2=0.0185（flametest 取帽同款）。

流程（9 步，照焰色反应 extinguish.py 写法，一次持握）：
  ① 高位接近   mv((LAMP_XY, CAP_HIGH))        # 帽底 0.9345 之上的高位，横移不碰灯
  ② 竖直下探   mv(CAP_GRASP)                   # 夹爪落到帽中心，两指夹帽壁
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_CAP, 60)              # task: 近帽+闭爪 → cap attached
  ⑤ 竖直提起   mv((LAMP_XY, CAP_HIGH), 5)      # 帽随夹爪竖直提出灯口
  ⑥ 单段横移   mv((CAP_ASIDE_XY, CAP_HIGH))    # flametest 写法：单个 mv 横移到放一边上方
  ⑦ 下探落位   mv(CAP_ASIDE_TCP, 25)           # 帽底落台面 0.80（帽中心 0.8155）+ 停顿
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)             # task: 近 aside+开爪 → cap 写回 aside rest
  ⑨ 高位归位   mv((CAP_ASIDE_XY, CAP_HIGH))

  2026-08-27 修「拿完灯帽还有多余的往下的动作穿模」：先前把横移拆成 ⑥a（只变 x）+
  ⑥b（只变 y）两段单轴线走——但 +X 拉伸段（x 0.50→0.62，距底座 0.79m 近臂展上限）
  逐帧重解分支翻转，TCP 仍下沉到 z≈0.814 穿灯身（h5 轨迹实证）。照焰色反应
  extinguish.py 写法改回单段 mv，并把放帽位 (0.62,-0.13)→(0.52,-0.18)：横移主要 -Y
  （-Y 段实测全程干净）、仅 2cm +X，臂展 0.72m 舒适不再下沉。


task 侧：cap 释放须同时满足「夹爪近 CAP_ASIDE_TCP」+「开爪 > open」（类沸石生命周期，
非裸火柴判据）——防止下探未到位提前释放导致帽悬空。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, LAMP_XY, CAP_HIGH, CAP_GRASP, GRIP_CAP,
                        CAP_ASIDE_XY, CAP_ASIDE_TCP)


class OpenCapPass(BaseMetaAction):
    """取灯帽 → 提起 → 放一边 → 松爪归位。"""

    def _build_actions(self):
        e = self.engine
        lx, ly = LAMP_XY
        ax, ay = CAP_ASIDE_XY
        return [
            mv(e, (lx, ly, CAP_HIGH)),            # ① 高位接近灯帽上方
            mv(e, CAP_GRASP),                     # ② 竖直下探到帽中心（两指夹帽壁）
            hold(e, SETTLE),                      # ③ 停顿稳定
            grip(e, GRIP_CAP, 60),                # ④ 合爪夹住帽壁（task 检测 attached）
            mv(e, (lx, ly, CAP_HIGH), 5),         # ⑤ 竖直提出灯口（帽底 0.9345）
            mv(e, (ax, ay, CAP_HIGH)),            # ⑥ 单段横移到放一边上方（flametest 写法）
            mv(e, CAP_ASIDE_TCP, 25),             # ⑦ 下探到帽落位（帽底贴台面）+ 停顿
            grip(e, GRIP_OPEN, 25),               # ⑧ 松爪：task 帽写回 aside rest
            mv(e, (ax, ay, CAP_HIGH)),            # ⑨ 高位归位
        ]
