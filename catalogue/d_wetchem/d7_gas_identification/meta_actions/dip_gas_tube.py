"""元动作 ②：取检验试管 → 移到下浸孔下放使末端浸入检测液面下 15mm（D7 气体鉴定）。

检验试管 Ø19.2×153mm 竖立在试管架右列第 3 排 (0.300,0.160)。侧面横夹（手指朝前 ORIENT_FWD，
同 d6 夹试管）：**2026-08-27 用户：右列试管从 -X 接近会插进架框/左列碰撞报错 → 改从 +X 侧接近**
（右列右侧架外清空），夹爪先到管口 +X 侧再水平移 -X 入管身中心。

移出试管架后**从导气管末端下方接近**（2026-08-27 用户：从导气管上方直下会穿过导气管桥穿模）：
先水平移到偏移位 (0.44,0.159) 高位 → 垂直降到下方抓点（管口低于末端 1cm）→ 平移到末端正下方
(0.44,0.079) → 上移套入到 DIP_GRASP——管口 (1.092) 套住导气管末端 (1.024)，末端沉入检测液面
(1.039) 下 15mm（产气试管已移出架外固定并抬高，下浸点不再占架孔；下探段加长后管口 1.092 仍
低于导气管桥 1.099）。

持握 = 纯平移跟随（试管 translate=底中心 = tool_center + (0,0,-0.139)，抓点处 = rest 0.806
零跳变）。task 期间管内检测液（TubeSolution 父 prim）随管平移。

轨迹（10 段，全程 orient=ORIENT_FWD）：
  ① 管口 +X 侧高位 → ② 降抓点高度 +X 侧 → ③ 水平移 -X 入管身中心 → ④ 合爪横夹
  → ⑤ 提预抓点(1.02) → ⑥ 提安全高位 H（清架顶板 0.917）→ ⑦ 移偏移位高位 H
  → ⑧ 垂直降到下方抓点(1.000，管口 1.014 低于末端 1.024) → ⑨ 平移入末端正下方
  → ⑩ 上移套入 DIP_GRASP(1.078，管底 0.939、液面 1.039、末端 1.024 浸 15mm)。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_TUBE, ORIENT_FWD, TEST_TUBE_XY, TUBE_GRASP,
                        TUBE_APPROACH_DX, TUBE_PRE_Z, DIP_XY, DIP_GRASP,
                        DIP_BELOW_Z, DIP_APPROACH_XY)


class DipGasTube(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TEST_TUBE_XY
        dx, dy = DIP_XY
        ax = tx + TUBE_APPROACH_DX          # 管口 +X 侧接近（右列右侧架外清空，避架框/左列碰撞）
        ox, oy = DIP_APPROACH_XY            # 下方接近偏移位（导气管桥 +Y 侧）
        t_high = (tx, ty, H)
        t_pre = (tx, ty, TUBE_PRE_Z)
        o_high = (ox, oy, H)
        o_below = (ox, oy, DIP_BELOW_Z)
        d_below = (dx, dy, DIP_BELOW_Z)
        return [
            mv(e, (ax, ty, H), orient=ORIENT_FWD),          # ① 管口 +X 侧高位
            mv(e, (ax, ty, TUBE_GRASP[2]), orient=ORIENT_FWD),  # ② 降抓点高度 +X 侧
            mv(e, TUBE_GRASP, orient=ORIENT_FWD),           # ③ 水平移 -X 入管身中心
            grip(e, GRIP_TUBE, 60),                         # ④ 合爪横夹
            mv(e, t_pre, orient=ORIENT_FWD),                # ⑤ 提预抓点
            mv(e, t_high, orient=ORIENT_FWD),               # ⑥ 提安全高位（清架顶板）
            mv(e, o_high, orient=ORIENT_FWD),               # ⑦ 移偏移位高位 H（桥 +Y 侧）
            mv(e, o_below, orient=ORIENT_FWD),              # ⑧ 垂直降到下方抓点（管口低于末端）
            mv(e, d_below, orient=ORIENT_FWD),              # ⑨ 平移入末端正下方（低于末端，不穿桥）
            mv(e, DIP_GRASP, orient=ORIENT_FWD),            # ⑩ 上移套入（末端浸入液面 15mm）
        ]
