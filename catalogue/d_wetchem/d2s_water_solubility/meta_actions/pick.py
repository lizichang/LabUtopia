"""元动作 ①：水平夹取药匙 → 竖直提起过架顶 → 手腕旋转放平。

用户动作要求（逐字）：「不是说夹的时候立刻把药匙旋转90度 我的意思是首先水平夹住
然后直接竖直向上提起来，直到药匙底部高于试管架的时候，通过机械臂的旋转把药匙变成
水平的（这也说明爪子从水平现在变成竖直了）」

新握持（task._T_HELD = R_z90·R_y90+t(0.112,0,0)）让药匙长轴 = 夹爪局部 X、与手指
垂直：手指水平(+X)时药匙竖直挂在下、与架内竖插姿态零跳变——夹住**不旋转**，竖直
提起也**不旋转**；提到勺尖过架顶 0.917（SPAT_LIFT_Z=1.10，勺尖 = 1.10-0.134 =
0.966 > 0.917）后才把腕转成 ORIENT_FLAT（手指朝上），药匙才变水平（勺头朝 +X
远离机械臂）。

轨迹（TCP 世界坐标，x 均取 +X 侧）：
  ① 高位接近架后   mv((SPAT_APPROACH[0],SPAT_APPROACH[1],H))      # 手指朝下先到 +X 侧上方
  ② 下探 + 转横    mv(SPAT_APPROACH, orient=ORIENT_HORIZ)         # 垂直下降同时转水平（手指横）
  ③ 水平扫到柄杆   mv(SPAT_GRASP,   orient=ORIENT_HORIZ)          # 竖柄杆喂进水平指缝（-X 扫 5cm）
  ④ 合爪横夹       grip(GRIP_SPATULA)                             # 药匙保持竖直 attach（不旋转）
  ⑤ 竖直提起       mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_HORIZ)   # 药匙仍竖直，勺尖过架顶 0.966>0.917
  ⑥ 手腕旋转放平   mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_FLAT)    # 手指水平→竖直，药匙变水平
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_SPATULA, ORIENT_HORIZ, ORIENT_FLAT,
                        SPAT_LIFT_Z, SPAT_XY, SPAT_APPROACH, SPAT_GRASP)


class PickSpatula(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = SPAT_XY
        return [
            mv(e, (SPAT_APPROACH[0], SPAT_APPROACH[1], H)),           # ① 高位接近架后（+X 侧）
            mv(e, SPAT_APPROACH, orient=ORIENT_HORIZ),                # ② 下探 + 转横（手指水平）
            mv(e, SPAT_GRASP, orient=ORIENT_HORIZ),                   # ③ 水平扫到柄杆（竖杆进指缝）
            grip(e, GRIP_SPATULA, 60),                               # ④ 合爪横夹：药匙保持竖直 attach
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_HORIZ),        # ⑤ 竖直提起：药匙竖直，勺尖过架顶
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_FLAT),         # ⑥ 手腕旋转：药匙放平（爪子水平→竖直）
        ]
