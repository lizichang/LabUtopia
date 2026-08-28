"""元动作：水平横夹试管提出架顶 + 法兰转 -95° + 水平移到火焰上方 + 竖直下降到火焰下 2cm（B1 加热流第一步）。

用户逐字（2026-08-27）：「现在最后动作就拿起试管，不要再做其他移动动作了，就先夹起试管」
→ 本元动作先做抓管：① 高位接近 → ② 水平横夹下探 → ③ 停顿 → ④ 合爪 → ⑤ 竖直提出架顶。
用户再批：「试管提出之后法兰旋转-100度」→ ⑥ 法兰只动 joint7 转 -95°（先 -100°，用户又改
「法兰改为旋转-95度吧」）——试管由竖直转成近水平（过水平 5°）。用户再批：「现在加动作水平
往负x方向移动（yz还有朝向都不变）让爪子x坐标对齐火焰的x坐标」→ ⑦ 水平 -X 移到
TUBE_AT_FLAME（爪子 x=火焰 x=0.50，y/z/朝向不变）。初版 mv(orient=FLANGE_HOLD_ORIENT) 被用户
证伪（「你为什么最后又加了一个旋转法兰的动作？最后把试管又旋转到正的角度去了」）——FLANGE_HOLD_ORIENT
在 tool_center 系手工推导、未计与 Lula "right_gripper" 帧的固定偏移 R_off，喂给 Lula 时工具朝向
被转偏 → 试管被甩回竖直。改 MovePreserveTubeAction（仿 d2s 移动药匙：首帧 fk_pose
采样当前**实际**朝向 [w,x,y,z] 作 target_orientation，R_off 自动抵消；MoveAction 单轴 linewalk，
x 0.659→0.50、y/z 锁当前、TCP 走严格直线、每帧重解 IK 保持朝向 → joint7 全程 ≈-95°，不会转回竖直）。
用户再批（2026-08-28）：「再加动作，竖直z方向降低，让爪子在z坐标比火焰小2cm（过程中yx还有朝向不变，
只有z变）」→ ⑧ 竖直下降 TUBE_AT_FLAME_Z（=火焰 z 0.9182 − 2cm = 0.8982），x/y/朝向不变
（同一 MovePreserveTubeAction，仅 z 变 → 单轴 linewalk）。用户再批：「然后加动作，只水平-y移动15cm，
xz朝向不要变」→ ⑨ 水平 -Y 移动（后改「最后一步水平移动改为11cm」）到 TUBE_AT_FLAME_2
（=(0.50,0.131,0.8982)，仅 y 变 → 单轴 linewalk）。
试管沿 tool+X=(0,-0.996,+0.087) 伸向 -Y（火焰方向），⑨ 后管身横跨火焰 y=0.0029、z≈0.898 高于灯身
玻璃顶 0.8897 不碰灯。

试管几何 = d2s 同款（Ø19.2×153mm，架内竖插，管口 0.9593、管底 0.806、架顶 0.917），
抓管口下 14mm（TUBE_GRASP_TCP = (0.659,0.241,0.9453)，从 d2s 导入）。

为什么 ORIENT_FWD 水平横夹：药匙/试管在 (0.659,0.241) 处竖直下探（手指朝下）IK 不可达
——D2S S6 已踩（日志 `IK FAIL target=[0.659 0.241 ...]`，Lula 解不出手指朝下的低 z 远点）。
夹药匙的 ORIENT_FWD（手指朝 +X，tool+Z=+X）在 (0.659,0.241) 处可达，B1 场景复刻 D2S
坐标逐字，故同一朝向即可夹试管管身。

持握 = 矩阵持握（药匙同款 _T_HELD）：试管世界位姿 = _T_HELD_TUBE · tool_world，随夹爪
6-DOF 刚性跟随（2026-08-27 用户逐字「爪子抓的东西应该也倾斜了呀」）；管内白粉柱随管
刚性跟随（task._set_tube_world 清 op 序写单一 transform op）。⑥ 法兰转 -95° 后试管
由竖直转成近水平（过水平 5°），白粉柱随管转、最低点 z≥0.9607 不碰架；⑦ 全程保持此
倾斜姿态横移（爪子 x 0.659→0.50 对齐火焰），试管 z≈1.08-1.12 高于灯顶 0.8897 无碰撞；
⑧ 竖直下降到爪子 z=0.8982（火焰下 2cm，仅 z 变），试管身落火焰高度（火焰锥底 0.9007、
中心 0.9182、顶 0.9357），爪子 x=0.50/y=0.241 距灯中心 y 0.238m 无碰撞。

流程（9 步，①-⑤ 全程 ORIENT_FWD；⑥ 法兰转只动 joint7；⑦⑧⑨ 采样当前朝向 + MoveAction 单轴 linewalk）：
  ① 高位接近   mv((TUBE_XY, H))               # 高位接近试管上方（水平横夹朝向）
  ② 水平横夹下探 mv(TUBE_GRASP_TCP)            # 抓管身（管口下 14mm，手指朝 +X 夹管壁）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_TUBE, 60)             # task: 近抓点+闭爪 → tube attached
  ⑤ 竖直提出   mv((TUBE_XY, TUBE_HIGH))        # 管底 0.961 清架顶 0.917（拿管出架不拖底）
  ⑥ 法兰转-95° FlangeRollTubeAction()          # 只动最后一个关节：试管竖直→近水平（文档倾斜姿态）
  ⑦ 水平移火焰上 MovePreserveTubeAction(TUBE_AT_FLAME)  # 水平 -X 0.659→0.50，y/z/朝向不变
  ⑧ 竖直降火焰下 MovePreserveTubeAction((TUBE_AT_FLAME[0],TUBE_AT_FLAME[1],TUBE_AT_FLAME_Z))  # 爪子 z=火焰 z−2cm=0.8982，x/y/朝向不变
  ⑨ 水平移火焰下2 MovePreserveTubeAction(TUBE_AT_FLAME_2)  # 水平 -Y 11cm → y 0.241→0.131，x/z/朝向不变

task 侧：tube 释放 = 近抓点+开爪双条件（本批次没有放回动作，高位开爪不误释放）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, ORIENT_FWD,
                        TUBE_XY, GRIP_TUBE, TUBE_GRASP_TCP,
                        TUBE_HIGH, TUBE_AT_FLAME, TUBE_AT_FLAME_Z, TUBE_AT_FLAME_2)
from .flange_roll_tube import FlangeRollTubeAction
from .move_x_preserve import MovePreserveTubeAction


class PickTubePass(BaseMetaAction):
    """水平横夹试管（跟夹药匙一样，ORIENT_FWD）→ 提出架顶 → 法兰转 -95° → 水平移到火焰上方 → 竖直下降到火焰下 2cm。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, (tx, ty, H), orient=ORIENT_FWD),                # ① 高位接近试管上方（水平横夹朝向）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD),             # ② 水平横夹下探抓管身（管口下14mm）
            hold(e, SETTLE),                                      # ③ 停顿稳定
            grip(e, GRIP_TUBE, 60),                               # ④ 合爪夹紧（task: tube attached）
            mv(e, (tx, ty, TUBE_HIGH), orient=ORIENT_FWD),        # ⑤ 竖直提出架顶（管底0.961清架顶0.917）
            FlangeRollTubeAction(),                               # ⑥ 法兰只动 joint7 转 -95°（试管竖直→近水平）
            MovePreserveTubeAction(e, TUBE_AT_FLAME),             # ⑦ 水平 -X 移火焰上方（保持 -95° 倾斜姿态）
            MovePreserveTubeAction(e, (TUBE_AT_FLAME[0], TUBE_AT_FLAME[1], TUBE_AT_FLAME_Z),
                                   dwell=20),                      # ⑧ 竖直下降：爪子 z=火焰 z−2cm，x/y/朝向不变
            MovePreserveTubeAction(e, TUBE_AT_FLAME_2, dwell=20),  # ⑨ 水平 -Y 11cm：y 0.241→0.131，x/z/朝向不变
        ]
