"""元动作 ④：竖直提取试管提出架顶（2026-08-30 用户「试管不要水平横夹还是竖直提取出来吧
（参考d3l）」）。

与 d3l TubeShakePass 抓管前缀同款（orient=None 手指朝下竖直下探，纯平移持握保竖立），
但 B4 试管在 (0.279,0.241)（架近侧左孔），只做「夹起提出」、不接震荡（后续浸冰另写）。
管内药品液柱 /World/TubeDrug 随管平移跟随（task 纯平移持握）。

  ① 高位接近   mv((tx, ty, TUBE_HIGH))      # 高位接近试管上方（手指朝下；TUBE_HIGH=1.10 够得到，
                                            #   原 H=1.15 超 Franka 行程 0.855m → IK FAIL 卡 25s）
  ② 垂直下探   mv(TUBE_GRASP_TCP)           # 抓管身中段（管口下 14mm，手指朝下）
  ③ 合爪夹紧   grip(GRIP_TUBE, 60)          # task: 近抓点+闭爪 → tube attached
  ④ 垂直提出   mv((tx, ty, TUBE_HIGH), 5)   # 管底 0.961 清架顶 0.917

持握 = TCP + (0,0,-TUBE_HELD_Z)（管底 0.1393m 吊在夹爪下方，task 纯平移保竖立），
管内药品液柱随管平移。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (GRIP_TUBE, TUBE_XY, TUBE_GRASP_TCP, TUBE_HIGH)


class PickTube(BaseMetaAction):
    """竖直提取试管（手指朝下，d3l 同款）→ 提出架顶。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, (tx, ty, TUBE_HIGH)),        # ① 高位接近试管上方（手指朝下；1.10 够得到）
            mv(e, TUBE_GRASP_TCP),             # ② 垂直下探抓管身中段（管口下14mm）
            grip(e, GRIP_TUBE, 60),            # ③ 合爪夹紧（task: tube attached）
            mv(e, (tx, ty, TUBE_HIGH), 5),     # ④ 垂直提出架顶（管底清架顶）
        ]
