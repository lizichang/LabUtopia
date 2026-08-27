"""元动作 ②：放沸石（B2 沸点测定 V7 步骤 4，2026-08-27 用户新增，滴加与挂温度计之间）。

用户逐字（2026-08-27）：「机械臂直接从玻璃皿竖直夹起一块沸石，然后旋转后，像滴管
滴加液体一样水平地伸到试管上方松开爪子放入沸石」。2026-08-27 追加「放两个沸石 1 个
太少了」→ 皿上并排两颗（ZEO_GRASP/ZEO2_GRASP），本元动作依次各抓放一颗（无内部循环，
靠 _build_actions 顺序展开两遍）。

流程（每颗一次持握，共两遍）：
  ① 竖直夹起：默认朝向（手指朝下）从皿上方竖直下探，两指竖直夹住沸石颗粒（抓点 =
     沸石中心 z=0.810，开度 GRIP_ZEO 贴合 Ø10.8mm 颗粒）。
  ② 旋转：竖直提起后，原地旋转到 ORIENT_FWD（手指朝前水平，像滴管横夹那样）。
  ③ 水平伸到试管上方：低空横越（TRANSIT_Z=1.15 压在挂钩支臂 z 1.216 之下）到管口
     偏 -X 侧 → 偏侧升到放下高度 ZEO_DROP_Z → 纯水平移 x 到管口正上方。
  ④ 松爪：沸石从夹爪坠落进试管口（Ø16.1mm > 沸石 Ø10.8mm），task 侧做坠落+沉底动画
     （比重大沉到管底防暴沸）。

持握矩阵（task._ZEO_HELD）：沸石局部原点=底 z=0、+Z 朝上；旋转与滴管/温度计同款
（沸石 Z→tool -X、X→tool -Z、Y→tool -Y），平移沿 tool+X 偏移 ZEO_CENTER_OFFSET=0.0037
（沸石中心落在夹爪处）。竖直夹（tool+X 朝下）→ 沸石在夹爪正下方（中心对齐夹爪）；
旋转后沸石随夹爪、位置不变（颗粒旋转对称无朝向差别）。

避穿模：横越走 TRANSIT_Z（沸石+夹爪全程在挂钩支臂底 1.216 之下）；松爪夹爪 z=1.10
也在支臂之下、试管夹（1.024..1.053）之上；沸石坠落在管内（x=管中心），不碰管壁/夹。

轨迹（TCP 世界坐标，每个 grasp 一遍）：
  ① 高位接近   mv((zx,zy,H))                    # 默认朝下（竖直夹姿态）
  ② 竖直下探   mv(grasp)                        # 夹爪降到沸石中心，两指竖直夹颗粒
  ③ 合爪夹紧   grip(GRIP_ZEO, 60)               # task 检测 attached
  ④ 竖直提出   mv((zx,zy,H), 5)                 # 沸石随夹爪提起（中心对齐夹爪）
  ⑤ 原地旋转   mv((zx,zy,H), orient=ORIENT_FWD, linewalk=False)  # 手指朝前水平
  ⑥ 低空横越   mv((tx-dx,ty,TRANSIT_Z), orient=ORIENT_FWD)       # 到管口偏 -X 侧
  ⑦ 偏侧升放   mv((tx-dx,ty,ZEO_DROP_Z), orient=ORIENT_FWD)      # 升到放下高度
  ⑧ 水平入管   mv((tx,ty,ZEO_DROP_Z), orient=ORIENT_FWD)         # 纯水平移到管口正上方
  ⑨ 松爪释放   grip(GRIP_OPEN, 60)              # task: released → 沸石坠落沉底
  ⑩ 水平退侧   mv((tx-dx,ty,ZEO_DROP_Z), orient=ORIENT_FWD)      # 原路退偏 -X 侧
  ⑪ 降回低空   mv((tx-dx,ty,TRANSIT_Z), orient=ORIENT_FWD)
  ⑫ 高位归位   mv((zx,zy,H), orient=ORIENT_FWD) # 回皿上方（下一颗或直接回架区）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_ZEO, ORIENT_FWD, TRANSIT_Z,
                        ZEO_GRASP, ZEO2_GRASP, ZEO_DROP_Z, ZEO_DROP_APPROACH_DX,
                        TUBE_XY)


class AddZeolitePass(BaseMetaAction):
    """竖直夹沸石 → 旋转手指朝前 → 水平伸到试管上方 → 松爪放入，两颗依次各一遍。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        dx = ZEO_DROP_APPROACH_DX
        actions = []
        for grasp in (ZEO_GRASP, ZEO2_GRASP):
            zx, zy, _ = grasp
            actions += [
                # —— 竖直夹起（默认朝向手指朝下，从皿上方竖直下探夹颗粒）——
                mv(e, (zx, zy, H)),                      # ① 高位接近（手指朝下）
                mv(e, grasp),                            # ② 竖直下探到沸石中心（两指竖直夹颗粒）
                grip(e, GRIP_ZEO, 60),                   # ③ 合爪夹住沸石（task 检测 attached）
                mv(e, (zx, zy, H), 5),                   # ④ 竖直提出（沸石中心对齐夹爪）
                # —— 旋转手指朝前（像滴管横夹那样水平操作），沸石随夹爪、位置不变 ——
                mv(e, (zx, zy, H), orient=ORIENT_FWD, linewalk=False),  # ⑤ 原地旋转（位置已到，只解朝向）
                # —— 水平伸到试管上方（低空横越压挂钩支臂之下，两段式放下像滴管滴加）——
                mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),   # ⑥ 低空横越到管口偏 -X 侧
                mv(e, (tx - dx, ty, ZEO_DROP_Z), orient=ORIENT_FWD),  # ⑦ 偏侧升到放下高度
                mv(e, (tx, ty, ZEO_DROP_Z), orient=ORIENT_FWD),       # ⑧ 纯水平移 x 到管口正上方
                grip(e, GRIP_OPEN, 60),                 # ⑨ 松爪：沸石坠落进试管口沉底（task 动画）
                mv(e, (tx - dx, ty, ZEO_DROP_Z), orient=ORIENT_FWD),  # ⑩ 原路退偏 -X 侧（避让）
                mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),   # ⑪ 降回低空
                mv(e, (zx, zy, H), orient=ORIENT_FWD),  # ⑫ 高位归位（回皿上方，下一颗）
            ]
        return actions
