"""元动作 ②c：放回洗瓶（②b 挤水后，把洗瓶平移回台面静止位并松爪放回）。

用户 2026-08-25（逐字，对齐 D2-S）：「然后加动作放回wash bottle」。

②b 结束夹爪持瓶停在管口上方 (WASH_TO_TUBE_X, WASH_TO_TUBE_Y, WASH_LIFT)，本动作逆 ②a 的 ⑦⑥
轨迹（单轴 linewalk，避免对角 2 轴单次 IK 的曲线 TCP 路径在高位扫弧）：
  ① 逆⑦ +Y 移回瓶位 y（锁 x/z）：(WASH_TO_TUBE_X, WASH_TO_TUBE_Y) → (WASH_TO_TUBE_X, WASH_XY.y)
  ② 逆⑥ -X 移回瓶位 x（锁 y/z）：→ (WASH_XY.x, WASH_XY.y)
  ③ 竖直降到抓点 z=WASH_GRASP_Z（此时瓶底已落回台面 z=0.80 = 静止位）
  ④ 开爪到 GRIP_OPEN 放回 —— task._update_washbottle 检测 opening > WASH_GRIP_OPEN →
     released、洗瓶 _washbottle_rest_matrix() 归位（瓶已在静止位，零跳变）
  ⑤ 竖直抬回安全高位 H（夹爪张开撤离，洗瓶留在台面）

朝向 = ORIENT_FWD 全程（与 pick 一致，瓶保持静止朝向，水平段边移边不转）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, ORIENT_FWD, GRIP_OPEN, WASH_XY, WASH_GRASP_Z, WASH_LIFT,
                        WASH_TO_TUBE_X, WASH_TO_TUBE_Y)


class ReturnWashBottle(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        wx, wy = WASH_XY
        tx, ty = WASH_TO_TUBE_X, WASH_TO_TUBE_Y
        return [
            mv(e, (tx, wy, WASH_LIFT), orient=ORIENT_FWD),   # ① 逆⑦ +Y 移回瓶位 y（锁 x/z）
            mv(e, (wx, wy, WASH_LIFT), orient=ORIENT_FWD),    # ② 逆⑥ -X 移回瓶位 x（锁 y/z）
            mv(e, (wx, wy, WASH_GRASP_Z), orient=ORIENT_FWD), # ③ 降到抓点（瓶落回台面）
            grip(e, GRIP_OPEN, 25),                            # ④ 开爪放回
            mv(e, (wx, wy, H), orient=ORIENT_FWD),             # ⑤ 撤离高位
        ]
