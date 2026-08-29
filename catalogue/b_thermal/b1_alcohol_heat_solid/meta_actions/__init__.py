"""B1 元动作（一个 v11 步骤 = 一个元动作，一类一文件）。

B1 本批次三个过程（用户 2026-08-27：「先写咬粉末咬进试管里面，然后拿起酒精灯盖儿放到
一边儿，再拿起火柴点燃酒精灯，这几个过程先只写这些我来验收」）：
  ① PickSpatula —— 复用 d2s（横向夹药匙 → 竖直提 → 法兰转 → 对齐粉堆 → 下探挖粉
     → 抬升 → 平移 → 回卷倒粉入管），坐标不动（home=None → d2s SPAT_XY）
  ② ReturnSpatula —— 复用 d2s（药匙放回试管架），坐标不动
  ③ OpenCapPass —— 拿起酒精灯盖放到一边（本包新写：取帽 → 提起 → 横移 → 落台面 →
     松爪归位；纯平移持握，帽中心 = 夹爪）
  ④ LightFlamePass —— 取火柴点燃酒精灯（B2 同款逐字：抓杆 → 触灯芯 → 点燃 → 放回火柴；
     B1 无温度模型，flame_lit 置位即 reveal 火焰）
  ⑤ PickTubePass —— 水平横夹试管（ORIENT_FWD，跟夹药匙一样）→ 提出架顶 → 法兰转 -95°
     （先 -100° 后用户改 -95°；再批「现在加动作水平往负x方向移动（yz还有朝向都不变）让爪子
       x坐标对齐火焰的x坐标」→ ⑦ 水平 -X 移火焰上方，MovePreserveTubeAction 首帧 fk_pose
       采样当前实际朝向、MoveAction 单轴 linewalk（仿 d2s 移动药匙，y/z 锁当前），
       joint7 保持 ≈-95° 倾斜姿态横移（mv+FLANGE_HOLD_ORIENT 会把试管甩回竖直，已弃）；
       再批「再加动作，竖直z方向降低，让爪子在z坐标比火焰小2cm（过程中yx还有朝向不变，
       只有z变）」→ ⑧ 竖直下降 TUBE_AT_FLAME_Z（火焰 z 0.9182 − 2cm = 0.8982），x/y/朝向不变；
       再批「然后加动作，只水平-y移动15cm，xz朝向不要变」→ ⑨ 水平 -Y 15cm（后改「最后一步水平
       移动改为11cm」）到 TUBE_AT_FLAME_2
       （y 0.241→0.131，x/z/朝向不变）；矩阵持握 _T_HELD_TUBE，试管随夹爪 6-DOF 转、白粉柱随管跟随）
  ⑥ PreheatTubePass —— 预热（用户 2026-08-28 逐字「现在需要加的动作是来回预热，现在加动作，
     在y的方向上来回移动2cm，来回移动5次速度不要太快」）：HeatSweepAction（首帧采样当前实际朝向
     + ShakeAction 正弦振荡）在 TUBE_AT_FLAME_2 附近 y 向 ±2cm 往复 5 次（period 150 = 2.5s/来回，
     慢速），全程保持法兰 -95° 倾斜姿态
  ⑦ HeatHoldPass —— 持续加热 8s（用户逐字「最后持续加热持续8s」）：纯 hold HEAT_HOLD_FRAMES=480
     帧，试管停在火焰上方外焰集中加热；B1 无温度模型 → 无加热现象
  ⑧ ReturnTubePass —— 放回试管（用户逐字「最后放回试管」；2026-08-28 修「现在放回有问题，没有
     对准试管架的孔，应该对准再竖直插下来」+「你就不能先抬高对准再下降放吗」）：PickTubePass 逆
     过程 = +Y 退开火焰 → 升 z → +X 回架上方 → 法兰转回竖直（FlangeRollTubeAction(angle=+95°)，
     joint7 → 抓取值）→ **精确对准孔心 + 强制竖直**（mv((TUBE_XY,TUBE_HIGH), orient=ORIENT_FWD)：
     不再采样当前朝向——会继承法兰回滚残余倾斜（ORIENT_EPS 8.6°，管底摆幅 2.1cm>孔半径 1.1cm →
     斜插穿模），直接喂验证过的竖直朝向 ORIENT_FWD（拾管②同款），高 z 安全位先转正对准）→
     竖直下降放回抓点（mv(TUBE_GRASP_TCP, orient=ORIENT_FWD)，仿 d3l 放回 mv()，linewalk 锁 x-y
     孔心）→ 松爪（task 近抓点+开爪 → released → 写回静置矩阵）→ 抬走
"""
from ._base import BaseMetaAction, mv, grip, hold, shake
from .open_cap import OpenCapPass
from .light_flame import LightFlamePass
from .close_cap_pass import CloseCapPass
from .pick_tube_pass import PickTubePass
from .preheat_tube import PreheatTubePass
from .heat_hold import HeatHoldPass
from .return_tube_pass import ReturnTubePass
# 挖粉/放回药匙 直接复用 d2s 元动作包（药匙/皿/粉/试管/试管架坐标逐字一致）
from catalogue.d_wetchem.d2s_water_solubility.meta_actions import (PickSpatula, ReturnSpatula)

__all__ = ["BaseMetaAction", "mv", "grip", "hold", "shake",
           "PickSpatula", "ReturnSpatula", "OpenCapPass", "LightFlamePass", "PickTubePass",
           "PreheatTubePass", "HeatHoldPass", "ReturnTubePass", "CloseCapPass"]
