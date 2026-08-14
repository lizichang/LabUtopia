"""元动作 ①：横向夹取药匙 → 抬起 → 转水平 → 插入粉末舀取 → 移到试管口 → 倾斜倒入。

用户动作要求（逐字）：「机械臂不能是从上到下拿起药匙，而是先横着夹住钥匙 向上
 抬起 然后再转向 水平的将药匙插入药粉末中 再移动到试管口 然后倾斜倒入」

几何（2026-08-14 pxr 实测 d2s_water_solubility.usd）：药匙竖插架前排右孔，勺头在
下（z 0.806-0.830，22mm 宽扁平）、柄杆在上（z 0.830-0.963，Ø8mm）。抓点在柄杆
z=0.94（原点上方 0.112m）：勺头尖 0.134m 在夹持点下方、柄顶 0.023m 在上方。
- DOWN 持握：勺头吊在夹持点下 134mm
- 转 HORIZ：勺头朝 -X（水平插入粉末）
- 转 POUR：勺头朝 -X 下 45°（倾倒时勺尖正好落试管口，见 POUR_TCP 注释）

轨迹（TCP 世界坐标，手指默认朝下；HORIZ/POUR 段显式传朝向）：
  ① 高位接近 +X 侧   mv((SPAT_APPROACH[0],SPAT_APPROACH[1],H))
  ② 下探到柄高       mv(SPAT_APPROACH)        # 手指朝下，从 +X 侧横着夹住柄杆
  ③ 水平扫到柄杆     mv(SPAT_GRASP)           # 柄杆进指缝
  ④ 合爪夹紧         grip(GRIP_SPATULA)
  ⑤ 垂直提出         mv(SPAT_GRASP_XY, H)     # 勺头吊下方，管架顶 0.914 之上
  ⑥ 原地转水平       mv(SPAT_GRASP_XY, H, orient=HORIZ)
  ⑦ 高位平移至皿上   mv((SCOOP_APPROACH, H))
  ⑧ 垂直下探到插入 z mv(SCOOP_APPROACH + POWDER_Z)
  ⑨ 水平插入粉丘     mv(SCOOP_INSERT + POWDER_Z) + hold(SETTLE)   # 勺尖 5mm 沉入粉
  ⑩ 垂直提出         mv(SCOOP_INSERT + SCOOP_LIFT_Z)
  ⑪ 平移到倾倒点     mv(POUR_TCP, orient=HORIZ)   # 勺尖在管口上方
  ⑫ 原地倾斜倒入     mv(POUR_TCP, orient=POUR) + hold(POUR_HOLD)
  ⑬ 转回水平         mv(POUR_TCP, orient=HORIZ)
  ⑭ 平移回架上方     mv(SPAT_GRASP_XY, H, orient=HORIZ)
  ⑮ 原地转回竖直     mv(SPAT_GRASP_XY, H, orient=ORIENT_DOWN)
  ⑯ 垂直下探放回     mv(SPAT_GRASP)
  ⑰ 松开释放         grip(GRIP_OPEN)           # task 侧把药匙落回架孔
  ⑱ 垂直归位         mv(SPAT_GRASP_XY, H)
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, GRIP_OPEN, GRIP_SPATULA,
                        ORIENT_DOWN, ORIENT_HORIZ, ORIENT_POUR,
                        SPAT_XY, SPAT_GRASP_Z, SPAT_APPROACH, SPAT_GRASP,
                        SCOOP_APPROACH, SCOOP_INSERT, SCOOP_LIFT_Z, POWDER_Z,
                        POUR_TCP, POUR_HOLD)


class ScoopSample(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        sx, sy = SPAT_XY
        ax, ay = SCOOP_APPROACH
        ix, iy = SCOOP_INSERT
        return [
            # —— 横向夹取药匙（不是从上到下拿起）——
            mv(e, (SPAT_APPROACH[0], SPAT_APPROACH[1], H)),   # ① 高位接近 +X 侧（随 SPAT_APPROACH 新坐标，勿硬编码旧值）
            mv(e, SPAT_APPROACH),                       # ② 下探到柄高
            mv(e, SPAT_GRASP),                          # ③ 水平扫到柄杆（进指缝）
            grip(e, GRIP_SPATULA, 60),                  # ④ 合爪夹紧
            mv(e, (sx, sy, H), 5),                      # ⑤ 垂直提出
            mv(e, (sx, sy, H), orient=ORIENT_HORIZ),    # ⑥ 原地转水平（勺头 -X）
            # —— 水平插入粉末舀取 ——
            mv(e, (ax, ay, H)),                         # ⑦ 高位平移至皿上
            mv(e, (ax, ay, POWDER_Z)),                  # ⑧ 垂直下探到插入 z
            mv(e, (ix, iy, POWDER_Z)),                  # ⑨ 水平插入粉丘
            hold(e, SETTLE),                            # ⑩ 停稳（沉入粉，等 task 显粉末）
            mv(e, (ix, iy, SCOOP_LIFT_Z)),              # ⑪ 垂直提出（勺上带粉）
            # —— 移到试管口、倾斜倒入 ——
            mv(e, POUR_TCP, orient=ORIENT_HORIZ),       # ⑫ 平移至倾倒点
            mv(e, POUR_TCP, orient=ORIENT_POUR),        # ⑬ 原地倾斜（勺尖落管口）
            hold(e, POUR_HOLD),                         # ⑭ 停留（粉末倒出）
            mv(e, POUR_TCP, orient=ORIENT_HORIZ),       # ⑮ 转回水平
            # —— 放回药匙架 ——
            mv(e, (sx, sy, H), orient=ORIENT_HORIZ),    # ⑯ 平移回架上方
            mv(e, (sx, sy, H), orient=ORIENT_DOWN),     # ⑰ 原地转回竖直
            mv(e, SPAT_GRASP),                          # ⑱ 垂直下探放回
            grip(e, GRIP_OPEN, 25),                     # ⑲ 松开释放
            mv(e, (sx, sy, H)),                         # ⑳ 垂直归位
        ]
