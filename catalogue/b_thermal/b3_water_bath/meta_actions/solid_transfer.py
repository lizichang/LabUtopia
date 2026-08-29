"""元动作 ①：放固体样品（B3 水浴加热 阶段A，复刻 B2 AddZeolitePass 放沸石）。

用户逐字（2026-08-29）：「初始样品也还是先放在玻璃皿上如果是固体的话」——固体样品
（白颗粒，复用 zeolite.usd）预置于玻璃皿上并排两颗，本元动作依次各抓放一颗入试管
（水浴加热熔化前的进样步骤，结构完全照 B2 放沸石：竖直夹 → 旋转手指朝前 → 水平伸
到试管上方 → 松爪坠落进试管口）。

与 B2 放沸石唯一差别：试管口更高（1.1088 vs 1.0939，因试管浸在烧杯水里），故放下
高度 SOLID_DROP_Z=1.135（口 +0.026）。持握矩阵 / 避穿模 / 两段式放下全部同 B2。

流程（每颗一次持握，共两遍，靠 _build_actions 顺序展开）：
  ① 竖直夹起：默认朝向（手指朝下）从皿上方竖直下探，两指竖直夹住固体颗粒（抓点 =
     固体中心 z=0.810，开度 GRIP_SOLID 贴合 Ø10.8mm 颗粒）。
  ② 旋转：竖直提起后，原地旋转到 ORIENT_FWD（手指朝前水平，像 d2s 夹药匙）。
  ③ 水平伸到试管上方：低空横越（TRANSIT_Z=1.15 压在挂钩支臂底 1.246 之下）到管口
     偏 -X 侧 → 偏侧升到放下高度 SOLID_DROP_Z → 纯水平移 x 到管口正上方。
  ④ 松爪：固体从夹爪坠落进试管口（Ø16.1mm > 固体 Ø10.8mm），task 侧做坠落 + 沉底动画。

持握矩阵（task._SOLID_HELD）：固体局部原点=底 z=0、+Z 朝上；旋转同滴管/温度计
（固体 Z→tool -X、X→tool -Z、Y→tool -Y），平移沿 tool+X 偏移 SOLID_CENTER_OFFSET=0.0037
（固体中心落在夹爪处）。竖直夹（tool+X 朝下）→ 固体在夹爪正下方（中心对齐夹爪）；
旋转后固体随夹爪、位置不变（颗粒旋转对称无朝向差别）。

避穿模：横越走 TRANSIT_Z（固体+夹爪全程在挂钩支臂底 1.246 之下）；松爪夹爪 z=1.135
也在支臂之下、试管夹（1.0589..1.0876）之上；固体坠落在管内（x=管中心），不碰管壁/夹。

轨迹（TCP 世界坐标，每个 grasp 一遍）：
  ① 高位接近   mv((sx,sy,H))                    # 默认朝下（竖直夹姿态）
  ② 竖直下探   mv(grasp)                        # 夹爪降到固体中心，两指竖直夹颗粒
  ③ 合爪夹紧   grip(GRIP_SOLID, 60)             # task 检测 attached
  ④ 竖直提出   mv((sx,sy,H), 5)                 # 固体随夹爪提起（中心对齐夹爪）
  ⑤ 原地旋转   mv((sx,sy,H), orient=ORIENT_FWD, linewalk=False)  # 手指朝前水平
  ⑥ 低空横越   mv((tx-dx,ty,TRANSIT_Z), orient=ORIENT_FWD)       # 到管口偏 -X 侧
  ⑦ 偏侧升放   mv((tx-dx,ty,SOLID_DROP_Z), orient=ORIENT_FWD)    # 升到放下高度
  ⑧ 水平入管   mv((tx,ty,SOLID_DROP_Z), orient=ORIENT_FWD)       # 纯水平移到管口正上方
  ⑨ 松爪释放   grip(GRIP_OPEN, 60)              # task: released → 固体坠落沉底
  ⑩ 水平退侧   mv((tx-dx,ty,SOLID_DROP_Z), orient=ORIENT_FWD)    # 原路退偏 -X 侧
  ⑪ 降回低空   mv((tx-dx,ty,TRANSIT_Z), orient=ORIENT_FWD)
  ⑫ 高位归位   mv((sx,sy,H), orient=ORIENT_FWD) # 回皿上方（下一颗或直接回架区）
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_SOLID, ORIENT_FWD, TRANSIT_Z,
                        SOLID_GRASP, SOLID2_GRASP, SOLID_DROP_Z,
                        SOLID_DROP_APPROACH_DX, TUBE_XY)


class SolidTransferPass(BaseMetaAction):
    """竖直夹固体 → 旋转手指朝前 → 水平伸到试管上方 → 松爪放入，两颗依次各一遍。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        dx = SOLID_DROP_APPROACH_DX
        actions = []
        for grasp in (SOLID_GRASP, SOLID2_GRASP):
            sx, sy, _ = grasp
            actions += [
                # —— 竖直夹起（默认朝向手指朝下，从皿上方竖直下探夹颗粒）——
                mv(e, (sx, sy, H)),                      # ① 高位接近（手指朝下）
                mv(e, grasp),                            # ② 竖直下探到固体中心（两指竖直夹颗粒）
                grip(e, GRIP_SOLID, 60),                 # ③ 合爪夹住固体（task 检测 attached）
                mv(e, (sx, sy, H), 5),                   # ④ 竖直提出（固体中心对齐夹爪）
                # —— 旋转手指朝前（像 d2s 夹药匙那样水平操作），固体随夹爪、位置不变 ——
                mv(e, (sx, sy, H), orient=ORIENT_FWD, linewalk=False),  # ⑤ 原地旋转（位置已到，只解朝向）
                # —— 水平伸到试管上方（低空横越压挂钩支臂之下，两段式放下像滴管滴加）——
                mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),   # ⑥ 低空横越到管口偏 -X 侧
                mv(e, (tx - dx, ty, SOLID_DROP_Z), orient=ORIENT_FWD),  # ⑦ 偏侧升到放下高度
                mv(e, (tx, ty, SOLID_DROP_Z), orient=ORIENT_FWD),       # ⑧ 纯水平移 x 到管口正上方
                grip(e, GRIP_OPEN, 60),                 # ⑨ 松爪：固体坠落进试管口沉底（task 动画）
                mv(e, (tx - dx, ty, SOLID_DROP_Z), orient=ORIENT_FWD),  # ⑩ 原路退偏 -X 侧（避让）
                mv(e, (tx - dx, ty, TRANSIT_Z), orient=ORIENT_FWD),     # ⑪ 降回低空
                mv(e, (sx, sy, H), orient=ORIENT_FWD),  # ⑫ 高位归位（回皿上方，下一颗）
            ]
        return actions
