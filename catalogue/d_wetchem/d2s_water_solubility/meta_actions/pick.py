"""元动作 ①：手指朝前（朝向 camera1）夹取药匙 → 竖直提起 → 最后一个关节（法兰）转 90° 放平（第 2 微动作）。

用户动作要求（逐字，2026-08-14）：「动作还是不对，首先你爪子夹起的方式跟
level4_liquidmixing不一样，你要模仿：爪子朝前，朝向camera1，夹起。就只做这两步
先不用旋转」；「我说朝前的意思是你现在的样子正好是朝反的」。

根因：旧 ORIENT_HORIZ=(0.7071,0,-0.7071,0) 在引擎真实约定（存储元组直接喂
from_quat）下让手指 tool+Z=(-1,0,0) 朝反（背离 camera1，用户亲眼确认）。改为
ORIENT_FWD=(0,0.7071,0,0.7071)：手指 tool+Z=(1,0,0) 朝 +X = 朝向 camera1；药匙
tool+X=(0,0,-1) 头下柄上；attach 后药匙世界 = REST 零跳变（pxr 数值验证）。

轨迹（TCP 世界坐标）：用户 2026-08-14 晚追加第 2 微动作——「机械臂的夹爪的那个最后一个关节
旋转90度，只加这一个变化」，并纠正我最初错用腕关节簇贪心的写法「写的不对，只动最后一个关节！」
→ ⑤ FlangeRollAction 只命令最后一个关节（panda_joint7 法兰自转，索引6）转 -45°（见 flange_roll.py）；
用户 2026-08-20「不对，删了重写法兰旋转后的动作，按我的步骤来！！你先删」删掉了之前的水平对齐/
DipToPowder；随后给出新步骤「法兰旋转后机械臂移动到粉堆的x绝对位置（0.537），然后机械臂的yz值
都不要变，然后整个夹爪在视频里面（整个世界）里面的绝对朝向也不要变，就是药匙还是要-60度这样子
夹着」→ ⑥ AlignPowderX：
  ① 高位       mv((sx,sy,H),          orient=ORIENT_FWD)   # 药匙柄杆正上方，手指朝前朝向 camera1
  ② 垂直下探   mv(SPAT_GRASP,         orient=ORIENT_FWD)   # 竖柄杆喂进指缝（手指朝前，抓点 z=0.94）
  ③ 夹起       grip(GRIP_SPATULA)                           # 合爪夹住柄杆（模仿 level4：爪子朝前夹起）
  ④ 竖直提起   mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_FWD)   # 药匙竖直提起，勺尖过架顶 0.917
  ⑤ 法兰转-45° FlangeRollAction()                          # 只动最后一个关节：药匙竖直→45° 倾斜
  ⑥ 对齐粉堆x  AlignPowderX(e)                              # 首帧自采样世界朝向并保持（药匙仍 -45° 夹着），
                                                            #   y/z 锁当前值，水平移到粉堆中心 x=0.537
  ⑦ 竖直降24.5cm LowerPowder(e)                           # 锁 x/y + 保持世界朝向，仅 z 降 24.5cm（2026-08-22 定 22cm、
                                                            #   08-23 改回 20cm；08-24 再降 3cm→23、2cm→25 但 25cm 会穿皿沿，
                                                            #   用户改选 24.5cm：勺尖沉粉 4mm 触发舀粉、不碰皿沿；x/y/朝向不变）
  ⑧ 平移-y16cm ShiftYNeg(e)                                # 锁 x/z + 保持世界朝向，仅 y 减 16cm（用户
                                                            #   2026-08-22 定 15cm、08-23 先后改 18cm→23cm；2026-08-24
                                                            #   皿+粉 +Y 6.5cm 后改回 16cm：终点 y=0.2008 脱离贴底座失效区，
                                                            #   勺尖仍到粉丘正上方，x/z/朝向严格不变）
  ⑨ 挖粉 ScoopUpAction()                                   # 法兰 -45°→-90°（只动最后一关节再转 -45°，用户
                                                            #   2026-08-24「要挖起来，法兰从-45旋转到-90」）：勺尖
                                                            #   随旋转上升 9.5cm 从粉丘挖起、凹槽朝上蓄粉，TCP 不动、不重解 IK
  ⑩ 抬升 LiftToTube(e)                                     # 保持世界朝向、x/y 锁当前，仅 z 升到管口上方 10cm
                                                            #   （1.0593 = 管口顶 0.9593+0.10，用户 2026-08-24「将爪子
                                                            #   抬升到试管管口高2cm」→「把第10步的2cm改成10cm」）：
                                                            #   为下一步水平移到管口倾倒留净空
  ⑪ 平移+y18cm ShiftYPos(e)                                # 保持世界朝向、x/z 锁当前，仅 y 增 18cm（2026-08-24 试管移到
                                                            #   架最近侧孔 y=0.241 后回调 17cm，再「倒数第二步y再增加1cm」
                                                            #   →18cm：TCP y 0.2008→0.3808，勺尖 y 0.2468（管口中心后
                                                            #   5.8mm），从粉丘上方水平移到试管口近前
  ⑫ 平移+x10cm ShiftXPos(e)                                # 保持世界朝向、y/z 锁当前，仅 x 增 10cm（用户 2026-08-24
                                                            #   「然后再往+x移动5cm」→12cm 伸到管口→「最后一步减少2厘米
                                                            #   深得有点太靠前」→10cm：TCP x 0.537→0.637，勺尖 x 0.637
                                                            #   （管口中心 0.659 前 2.2cm）
"""
from ._base import BaseMetaAction, mv, grip
from .align_powder_x import AlignPowderX
from .constants import (H, GRIP_SPATULA, ORIENT_FWD,
                        SPAT_LIFT_Z, SPAT_XY, SPAT_GRASP)
from .flange_roll import FlangeRollAction
from .lift_to_tube import LiftToTube
from .lower_powder import LowerPowder
from .scoop_up import ScoopUpAction
from .shift_x_pos import ShiftXPos
from .shift_y_neg import ShiftYNeg
from .shift_y_pos import ShiftYPos


class PickSpatula(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = SPAT_XY
        return [
            mv(e, (sx, sy, H), orient=ORIENT_FWD),            # ① 高位：手指朝前（+X）朝向 camera1
            mv(e, SPAT_GRASP, orient=ORIENT_FWD),             # ② 垂直下探：竖柄杆进指缝
            grip(e, GRIP_SPATULA, 60),                       # ③ 夹起（level4 模式：爪子朝前夹起）
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_FWD),  # ④ 竖直提起
            FlangeRollAction(),                               # ⑤ 只动最后一个关节转-45°：药匙竖直→45° 倾斜
            AlignPowderX(e),                                  # ⑥ 保持世界朝向、y/z 锁当前，水平移到粉堆 x=0.537
            LowerPowder(e),                                   # ⑦ 保持世界朝向、x/y 锁当前，竖直下降 24.5cm
            ShiftYNeg(e),                                     # ⑧ 保持世界朝向、x/z 锁当前，往 -y 平移 16cm
            ScoopUpAction(),                                  # ⑨ 法兰 -45°→-90°（只动 joint7 再转 -45°）：勺尖从粉丘挖起、凹槽朝上蓄粉
            LiftToTube(e),                                    # ⑩ 保持世界朝向、x/y 锁当前，仅 z 升到管口上方 10cm（1.0593）
            ShiftYPos(e),                                     # ⑪ 保持世界朝向、x/z 锁当前，往 +y 平移 18cm（0.2008→0.3808）
            ShiftXPos(e),                                     # ⑫ 保持世界朝向、y/z 锁当前，往 +x 平移 10cm（0.537→0.637，12cm 太靠前减 2cm）
        ]
