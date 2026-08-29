# -*- coding: utf-8 -*-
"""元动作：盖上灯帽熄灭火焰（B1 加热完成后熄火收尾，2026-08-28 用户逐字「现在加动作盖上
灯冒，熄灭火焰」）。

OpenCapPass 的逆过程（同一对点反向走，9 步一次持握）：从放一边的台面端起灯帽 → 提起 →
横移回灯口 → 落回灯口 → 松爪 → 归位。task 侧在帽盖回灯口的瞬间熄灭火焰（_CapLifecycle
对称化：吸附期夹爪近 CAP_GRASP 即 _extinguish_flame，B1 无温度模型直接隐藏
flame_outer/flame_inner；松爪写回 CAP_CENTER_REST local_t=0）。

灯帽几何（OpenCapPass 同款）：Ø37.2mm×31mm 钟罩，帽世界中心 = 夹爪（纯平移持握，
task._set_cap_center 写帽 local translate = center − CAP_CENTER_REST，只动 translate op
姿态不变）。放一边中心 CAP_ASIDE_CENTER=(0.52,-0.18,0.8155)（帽底贴台面 0.80）；
灯口中心 CAP_CENTER_REST=(0.50,0.0029,0.8917)（local_t=0）。GRIP_CAP=Ø37.2/2=0.0185
（两指夹帽壁，帽壁在空心灯口内玻璃壁 y=±0.0435 之内无碰撞）。CAP_HIGH=0.95 帽底 0.9345
高于灯顶 0.8897，横移不碰灯。

流程（9 步，OpenCapPass 镜像）：
  ① 高位接近   mv((CAP_ASIDE_XY, CAP_HIGH))        # 放一边帽上方的高位
  ② 竖直下探   mv(CAP_ASIDE_TCP)                   # 夹爪落到帽中心（从台面端起帽）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_CAP, 60)                  # task: 近 aside+闭爪 → cap attached
  ⑤ 竖直提起   mv((CAP_ASIDE_XY, CAP_HIGH), 5)     # 帽随夹爪竖直提起（帽底离台面）
  ⑥ 单段横移   mv((LAMP_XY, CAP_HIGH))             # 横移到灯帽上方（flametest 单段写法）
  ⑦ 下探落位   mv(CAP_GRASP, 25)                   # 帽落回灯口（垂直 linewalk）+ 停顿
  ⑧ 松爪释放   grip(GRIP_OPEN, 25)                 # task: 近灯口+开爪 → cap 写回灯口 rest
  ⑨ 高位归位   mv((LAMP_XY, CAP_HIGH))             # 空爪离开（帽已盖回灯口，火焰已熄）

task 侧：_CapLifecycle 对称化——rest 检测「近灯口或近放一边」任一吸附（开盖从灯口端起、
合盖从放一边端起）；吸附期帽中心近 CAP_GRASP 即熄火（盖住即隔氧）；松爪时近灯口 → 写回
CAP_CENTER_REST + 熄火；近放一边 → 写回放一边中心（OpenCapPass 原行为）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, LAMP_XY, CAP_HIGH, CAP_GRASP, GRIP_CAP,
                        CAP_ASIDE_XY, CAP_ASIDE_TCP)


class CloseCapPass(BaseMetaAction):
    """端起放一边的灯帽 → 横移回灯口 → 落回 → 松爪（task 帽盖回灯口即熄火）。"""

    def _build_actions(self):
        e = self.engine
        lx, ly = LAMP_XY
        ax, ay = CAP_ASIDE_XY
        return [
            mv(e, (ax, ay, CAP_HIGH)),            # ① 高位接近放一边的帽上方
            mv(e, CAP_ASIDE_TCP),                 # ② 竖直下探到帽中心（从台面端起帽）
            hold(e, SETTLE),                      # ③ 停顿稳定
            grip(e, GRIP_CAP, 60),                # ④ 合爪夹住帽壁（task 检测 attached）
            mv(e, (ax, ay, CAP_HIGH), 5),         # ⑤ 竖直提起（帽底离台面）
            mv(e, (lx, ly, CAP_HIGH)),            # ⑥ 单段横移到灯帽上方（flametest 写法）
            mv(e, CAP_GRASP, 25),                 # ⑦ 下探落回灯口 + 停顿
            grip(e, GRIP_OPEN, 25),               # ⑧ 松爪：task 帽写回灯口 rest + 熄火
            mv(e, (lx, ly, CAP_HIGH)),            # ⑨ 高位归位
        ]
