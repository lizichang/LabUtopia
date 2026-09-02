"""元动作 ③：水平横夹燃烧匙 → 碗入外焰（2026-09-01 用户「水平横夹起燃烧匙，将勺子部分
移动到外焰部分」）。

C4 布局：燃烧匙 (0.636,0.3093) 碗贴台、杆斜靠试管架（无旋转）；酒精灯 (0.7094,-0.10)，
火焰 /World 顶层初始隐藏（base 0.900 apex 0.936，阶段 ② 火柴点燃后 visible）。
③ 在 ② 点灯后执行：水平横夹杆身把碗口放进外焰燃烧 4s（FLAME_DWELL），往 +y 移 5cm
离开酒精灯停留观察 10s（OBSERVE_DWELL），然后提出放回原位（用户 09-02「燃烧 4s → +y 5cm
观察 10s → 放回」）——阶段 ④ 移灯/盖帽熄灭在其后。

持握 = 纯平移 offset（task.SPOON_HELD_OFFSET）：勺原点（碗口平面）= 夹爪 + offset，
姿态不变（杆仍斜，碗在夹爪下方 0.173m 略偏 -X）。碗液（/World/SpoonLiquid 或 <色> 变体）
随碗平移（task._SpoonLifecycle）。

轨迹（TCP 世界坐标，全程默认朝向手指朝下）：
  ① 高位接近   mv((gx,gy,SPOON_LIFT_Z))     # 手指朝下
  ② 竖直下探   mv(SPOON_GRASP)              # 下探到杆身（横夹 Ø3mm 杆）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_SPOON, 60)         # task 检测 attached（勺写回持握位）
  ⑤ 竖直提出   mv((gx,gy,SPOON_LIFT_Z), 5)  # 勺随夹爪提出（碗口 z = LIFT−0.173 = 0.977）
  ⑥ 高位移灯   mv((fx,fy,SPOON_LIFT_Z))     # 水平移到灯上方（碗口 0.977 > 架顶 0.917 清障）
  ⑦ 下探入焰   mv(FLAME_HOLD_TCP, 10)       # 碗口降到外焰中心（dwell 燃烧观察）
  ⑧ 停在外焰   hold(FLAME_DWELL)            # 碗在火焰中停留 4s（用户 09-02「改 4 秒」）
  ⑨ 离焰观察   mv(OBSERVE_TCP, 10)          # 往 +y 移 5cm 离开酒精灯（碗口同高度）
  ⑩ 停留观察   hold(OBSERVE_DWELL)          # 停留不动 10s 观察（用户 09-02）
  ⑪ 竖直提出   mv((ox,oy,SPOON_LIFT_Z), 5)  # 观察完提出离焰（放回原位）
  ⑫ 高位移回   mv((gx,gy,SPOON_LIFT_Z))     # 水平移回勺位上方
  ⑬ 下探回杆   mv(SPOON_GRASP, 10)          # 竖直下探回杆身（勺原点落回原位）
  ⑭ 松爪释放   grip(GRIP_OPEN, 60)          # 开爪（task 检测 released → 勺写回 rest）
  ⑮ 抬离让路   mv((gx,gy,SPOON_LIFT_Z), 5)  # 抬离（为阶段 ⑤ 盖帽让路）
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_SPOON, SPOON_GRASP, SPOON_LIFT_Z,
                        FLAME_HOLD_TCP, FLAME_DWELL, OBSERVE_TCP, OBSERVE_DWELL)


class SpoonToFlamePass(BaseMetaAction):
    """水平横夹燃烧匙杆身 → 提出 → 碗口放入外焰。"""

    def _build_actions(self):
        e = self.engine
        gx, gy, _ = SPOON_GRASP
        fx, fy, _ = FLAME_HOLD_TCP
        ox, oy, _ = OBSERVE_TCP
        return [
            mv(e, (gx, gy, SPOON_LIFT_Z)),             # ① 高位接近（手指朝下）
            mv(e, SPOON_GRASP),                        # ② 竖直下探到杆身（横夹）
            hold(e, SETTLE),                           # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_SPOON, 60),                   # ④ 合爪夹住杆（task 检测 attached）
            mv(e, (gx, gy, SPOON_LIFT_Z), 5),          # ⑤ 竖直提出
            mv(e, (fx, fy, SPOON_LIFT_Z)),             # ⑥ 高位水平移到灯上方
            mv(e, FLAME_HOLD_TCP, 10),                 # ⑦ 下探入外焰（dwell 燃烧观察）
            hold(e, FLAME_DWELL),                      # ⑧ 碗停在外焰中（4s）
            mv(e, OBSERVE_TCP, 10),                    # ⑨ 往 +y 移 5cm 离开酒精灯
            hold(e, OBSERVE_DWELL),                    # ⑩ 停留不动 10s 观察
            mv(e, (ox, oy, SPOON_LIFT_Z), 5),          # ⑪ 观察完竖直提出离焰
            mv(e, (gx, gy, SPOON_LIFT_Z)),             # ⑫ 高位水平移回勺位上方
            mv(e, SPOON_GRASP, 10),                    # ⑬ 竖直下探回杆身（勺原点落回原位）
            grip(e, GRIP_OPEN, 60),                    # ⑭ 松爪（task 检测 released → 勺写回 rest）
            mv(e, (gx, gy, SPOON_LIFT_Z), 5),          # ⑮ 抬离（为阶段 ⑤ 盖帽让路）
        ]
