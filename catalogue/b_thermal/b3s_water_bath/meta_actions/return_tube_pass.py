"""元动作：加热结束后把试管从烧杯水浴提起 → 原路水平平移回架孔上方 → 竖直降回架孔 → 松爪放回 → 抬离。

用户逐字（2026-08-29）：「拿试管加热的时候机械臂不能松手，直到加热结束才放回去」→ 加热结束
（task 相态机 boiling 保持完成进入 tube_return 相）后，机械臂仍夹着试管（_T_HELD_TUBE 矩阵
持握），由本动作把试管放回架孔。放回 = PickTubePass 纯平移分段路径的反向：① 竖直提出 → ② 水平
横移回架 → ③ 竖直降回架孔（管底回 0.806）→ ④ 松爪（task 检测近架孔抓点+开爪 → released → 写
_tube_rest_matrix + tube_returned）→ ⑤ 空爪抬离。

流程（5 步，全程 ORIENT_FWD、无法兰翻转）：
  ① 竖直提出   mv(TUBE_TRANSIT)                     # 从浸入位竖直提到横移高度（管底 1.0207 清烧杯口）
  ② 水平横移   mv(TUBE_RETURN_TRANSIT)              # 纯水平移回架孔上方（z 恒定 1.174，不反转）
  ③ 竖直降回   mv(TUBE_GRASP_TCP, dwell=20)         # 降回架孔抓点（管底 0.806 回架孔洞底），dwell 准备释放
  ④ 松爪放回   grip(GRIP_OPEN, 25)                  # task: 近架孔+开爪 → released → 写架孔静止位 + tube_returned
  ⑤ 空爪抬离   mv((TUBE_XY, H))                     # 抬离（试管留在架孔）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, ORIENT_FWD,
                        TUBE_XY, TUBE_GRASP_TCP,
                        TUBE_TRANSIT, TUBE_RETURN_TRANSIT)


class ReturnTubePass(BaseMetaAction):
    """加热结束后把试管从烧杯提起 → 原路平移回架孔 → 松爪放回 → 抬离。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, TUBE_TRANSIT, orient=ORIENT_FWD),              # ① 竖直提出到横移高度（管底清烧杯口）
            mv(e, TUBE_RETURN_TRANSIT, orient=ORIENT_FWD),       # ② 水平横移回架孔上方（z恒定1.174，纯水平）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD, dwell=20, freeze_dist=0.03, timeout=200),  # ③ 降回架孔抓点；低z死区→near即freeze+4s兜底
            grip(e, GRIP_OPEN, 25),                              # ④ 松爪：task 近架孔+开爪 → released → 写架孔静止位
            mv(e, (tx, ty, H), orient=ORIENT_FWD),               # ⑤ 空爪抬离（试管留架孔）
        ]
