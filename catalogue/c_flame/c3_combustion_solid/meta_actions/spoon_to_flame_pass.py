"""元动作 ③：横夹燃烧匙把手 → 碗入外焰（2026-09-01 用户「再拿起燃烧匙移动到外焰上。（仿照C4）」）。

C3 布局：燃烧匙 (0.596,0.250) 碗贴台、把手竖直立起靠试管架旁（无旋转，原厂朝向）。
酒精灯 (0.30,0.38)（2026-09-02 用户挪离底座 -x 10cm/+y 20cm，见 constants），火焰 /World 顶层
初始隐藏（base 0.900 apex 0.936，阶段 ② 火柴点燃后 visible）。③ 在 ② 点灯后执行：横夹把手
把碗口放进外焰停留（FLAME_DWELL，待燃烧现象接续该窗口），然后提出放回原位（C4 全流程）。
阶段 ④ 燃烧现象/熄火留待后续接续。

持握 = 纯平移 offset（task.SPOON_HELD_OFFSET）：勺原点（碗口平面）= 夹爪 + offset，
姿态不变（把手仍竖直，碗在夹爪下方 0.123m 略偏 -X）。碗内样品（/World/PowderInBowl）
随碗平移（task._SpoonLifecycle 写 translate 跟随勺原点）。**夹爪朝向不影响碗位**。

朝向：**全程 ORIENT_FWD 横夹**（单一朝向，无切换 → 无手腕旋转）。
  2026-09-02 用户报「夹燃烧匙后移动时机械臂旋转、燃烧匙跟着转、粉末会倒」——根因=旧两段
  朝向（远端横夹 ↔ 灯区手指朝下）在 ⑤→⑥ / ⑨→⑩ 切换 90°，手腕随行旋转，视觉上勺跟着转。
  实测（verify_c3_spoon_reach.py）：手指朝下（「竖直夹」）在把手 z≥0.90（0.829m）已 IK FAIL、
  勺位上方提出高位 (0.632,0.25,1.08) 也 FAIL（死区 ~0.82m，B3L 实测 0.841m 已 FAIL）→「竖直夹」
  不可行；ORIENT_FWD 在抓点 0.803-0.881m + 全灯区（灯移 (0.30,0.38) 后 0.677m）全程可达。
  故改全程 ORIENT_FWD，消除旋转；持握仍纯平移 offset，夹爪朝向不影响碗位。
  2026-09-02 用户「夹太靠下」→ 抓点 z0.90→0.93（杆中心线 x 0.632→0.640，随斜度 0.26）。

轨迹（TCP 世界坐标；全程 orient=ORIENT_FWD）：
  ① 高位接近   mv((gx,gy,SPOON_LIFT_Z), orient=ORIENT_FWD)
  ② 竖直下探   mv(SPOON_GRASP, orient=ORIENT_FWD)   # 下探到把手（横夹 Ø3mm 杆，高于架顶 0.917）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_SPOON, 60)                 # task 检测 attached（勺写回持握位）
  ⑤ 竖直提出   mv((gx,gy,SPOON_LIFT_Z), 5, orient=ORIENT_FWD)   # 勺随夹爪提出（碗口 z = LIFT−0.123 = 0.957）
  ⑥ 高位移灯   mv((fx,fy,SPOON_LIFT_Z), orient=ORIENT_FWD)      # 横移灯上方（碗口 0.957 > 架顶 0.917 清障）
  ⑦ 下探入焰   mv(FLAME_HOLD_TCP, FLAME_DWELL, orient=ORIENT_FWD)   # 碗口降到外焰中心 → freeze 静止 4s（MoveAction freeze 发固定关节值，不漂移）
  ⑧ 提出离焰   mv((fx,fy,SPOON_LIFT_Z), 5, orient=ORIENT_FWD)   # 提出离焰（碗口 0.957 > 火焰顶 0.936 清障）
  ⑨ 移观察位   mv((fx,fy+OBSERVE_SHIFT_Y,SPOON_LIFT_Z), OBSERVE_DWELL, orient=ORIENT_FWD)   # 往+y 5cm → freeze 静止 10s（观察现象）
  ⑩ 高位移回   mv((gx,gy,SPOON_LIFT_Z), orient=ORIENT_FWD)      # 回勺位上方
  ⑪ 下探回杆   mv(SPOON_GRASP, 10, orient=ORIENT_FWD)           # 竖直下探回把手（勺原点落回原位）
  ⑫ 松爪释放   grip(GRIP_OPEN, 60)                   # 开爪（task 检测 released → 勺写回 rest）
  ⑬ 抬离让路   mv((gx,gy,SPOON_LIFT_Z), 5, orient=ORIENT_FWD)   # 抬离
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_SPOON, SPOON_GRASP, SPOON_LIFT_Z,
                        FLAME_HOLD_TCP, FLAME_DWELL, OBSERVE_SHIFT_Y, OBSERVE_DWELL,
                        ORIENT_FWD)


class SpoonToFlamePass(BaseMetaAction):
    """横夹燃烧匙把手 → 提出 → 碗口放入外焰（全程 ORIENT_FWD）。"""

    def _build_actions(self):
        e = self.engine
        gx, gy, _ = SPOON_GRASP
        fx, fy, _ = FLAME_HOLD_TCP
        return [
            mv(e, (gx, gy, SPOON_LIFT_Z), orient=ORIENT_FWD),        # ① 高位接近（ORIENT_FWD 横夹）
            mv(e, SPOON_GRASP, orient=ORIENT_FWD),                   # ② 竖直下探到把手（两指横夹杆）
            hold(e, SETTLE),                                         # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_SPOON, 60),                                 # ④ 合爪夹住把手（task 检测 attached）
            mv(e, (gx, gy, SPOON_LIFT_Z), 5, orient=ORIENT_FWD),     # ⑤ 竖直提出
            mv(e, (fx, fy, SPOON_LIFT_Z), orient=ORIENT_FWD),        # ⑥ 高位移到灯上方（全程 ORIENT_FWD）
            mv(e, FLAME_HOLD_TCP, FLAME_DWELL, orient=ORIENT_FWD),   # ⑦ 下探入外焰 + freeze 静止 4s（发固定关节值不漂移）
            mv(e, (fx, fy, SPOON_LIFT_Z), 5, orient=ORIENT_FWD),     # ⑧ 提出离焰
            mv(e, (fx, fy + OBSERVE_SHIFT_Y, SPOON_LIFT_Z), OBSERVE_DWELL, orient=ORIENT_FWD),  # ⑨ 往+y 5cm + freeze 静止 10s（观察）
            mv(e, (gx, gy, SPOON_LIFT_Z), orient=ORIENT_FWD),        # ⑩ 高位移回勺位上方（无朝向切换）
            mv(e, SPOON_GRASP, 10, orient=ORIENT_FWD),               # ⑪ 竖直下探回把手
            grip(e, GRIP_OPEN, 60),                                  # ⑫ 松爪（task 检测 released → 勺写回 rest）
            mv(e, (gx, gy, SPOON_LIFT_Z), 5, orient=ORIENT_FWD),     # ⑬ 抬离让路
        ]
