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
→ ⑤ FlangeRollAction 只命令最后一个关节（panda_joint7 法兰自转，索引6）转 -90°（见 flange_roll.py）；
用户 2026-08-17 重新加入第 3 步——法兰转后机械臂水平往 -X、直到对齐粉末：
  ① 高位       mv((sx,sy,H),          orient=ORIENT_FWD)   # 药匙柄杆正上方，手指朝前朝向 camera1
  ② 垂直下探   mv(SPAT_GRASP,         orient=ORIENT_FWD)   # 竖柄杆喂进指缝（手指朝前，抓点 z=0.94）
  ③ 夹起       grip(GRIP_SPATULA)                           # 合爪夹住柄杆（模仿 level4：爪子朝前夹起）
  ④ 竖直提起   mv((sx,sy,SPAT_LIFT_Z), orient=ORIENT_FWD)   # 药匙竖直提起，勺尖过架顶 0.917
  ⑤ 法兰转-90° FlangeRollAction()                          # 只动最后一个关节：药匙竖直→水平
  ⑥ 对齐粉末   mv(SCOOP_ALIGN_X, orient=ORIENT_SCOOP, dwell=120) # 水平往 -X 到粉丘中心 x=0.5365
                                                            #   （v47 单轴直线：y/z 锁目标值、x 逐帧推进，TCP 不升高）
                                                            #   ORIENT_SCOOP=实测 post-roll tool 朝向（2026-08-17 修
                                                            #   正：旧值 180° 反会让 solve_verified 拒绝当前分支、
                                                            #   joint7 翻到 +83° 药匙面滚歪）
                                                            #   到点停 120 帧≈2s（60fps）看最终状态——到此结束
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_SPATULA, ORIENT_FWD, ORIENT_SCOOP,
                        SPAT_LIFT_Z, SPAT_XY, SPAT_GRASP, SCOOP_ALIGN_X)
from .flange_roll import FlangeRollAction


class PickSpatula(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = SPAT_XY
        return [
            mv(e, (sx, sy, H), orient=ORIENT_FWD),            # ① 高位：手指朝前（+X）朝向 camera1
            mv(e, SPAT_GRASP, orient=ORIENT_FWD),             # ② 垂直下探：竖柄杆进指缝
            grip(e, GRIP_SPATULA, 60),                       # ③ 夹起（level4 模式：爪子朝前夹起）
            mv(e, (sx, sy, SPAT_LIFT_Z), orient=ORIENT_FWD),  # ④ 竖直提起
            FlangeRollAction(),                               # ⑤ 只动最后一个关节转-90°：药匙竖直→水平
            mv(e, SCOOP_ALIGN_X, orient=ORIENT_SCOOP,
               dwell=120),                                    # ⑥ 水平往 -X 对齐粉末（x=0.5365），到点
                                                              #    停 120 帧≈2s 看最终状态—— 到此结束
        ]
