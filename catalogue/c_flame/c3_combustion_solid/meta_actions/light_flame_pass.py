"""元动作 ②：点燃酒精灯（C3 燃烧试验固体样品，2026-09-01 用户「然后拿起火柴点燃酒精灯」）。

照 C4 LightFlamePass 结构移植，仅换 C3 坐标。C3 布局（用户定稿，与 d2s 骨架并存）：
酒精灯 (0.40,0.18) rot180，火柴 (0.54,0.18,0.813) 抬 13mm 头朝 +X。2026-09-01 用户指示
「酒精灯/灯帽/火柴离机械臂太近 → 挪 +x/+y 且不与现有器材重合」→ 从底座 y=0.05 线挪到
y=0.18（灯体东缘 0.491 距表面皿西缘 0.507 留 1.6cm、火柴杆 x[0.54,0.638] 距灯 4.9cm/距架
3.8cm/距皿 4.2cm），灯帽摘灯正北 12cm (0.40,0.30)（原 +X 侧会被火柴撞）。灯芯
WICK=(0.40,0.18,0.9007) 在火柴头 (+X 端) 推进方向正前方 → 夹爪从抓点 x=0.58 直推 -X
到 IGNITE (0.3506,0.18,0.9157)（gripper 头偏 0.0494 → 火柴头落 WICK 正上方 1.5cm），
task 检测近芯 → 点火（火焰 reveal）。

与 C4 的关键差异（回程绕焰）：C4 灯在火柴 -Y 侧，回程纯 +Y 直退（头侧向撤走火焰）；
**C3 灯在火柴 +X 侧**，回程沿 +X 扫回——火焰已点着（外焰锥 0.900-0.936），若按 C4 的低位
运移（z≈0.92 横越灯体），回程火柴头会横穿火焰柱。故 C3 全程在 MATCH_CARRY_Z=0.96
（>火焰尖 0.936）高位运移，只在点火瞬间下探 4.5cm 到 IGNITE（头近芯），点着后抬回 0.96
再直退 +X——头始终在火焰尖上方绕行。

**朝向 = 默认（手指朝下）**。2026-09-01 用户两次运行确认：手指朝下火柴夹得起、点灯 OK
（「拿起火柴点燃酒精灯这个步骤是没问题的」）；本日志误改成 ORIENT_FWD（手指 +X）后
**火柴没夹起来**——火柴是水平杆、抓点贴台面 0.8145，ORIENT_FWD 手指 +X 与杆轴平行、
下探时两指从杆两侧/杆上方落不到杆上（且 C4 常量注释早有预警：「低 z 桌面夹帽 ORIENT_FWD
手指朝前 Lula 无解」）。抓点 (0.58,0.18,0.8145) 3D=0.749m、高位 MATCH_HIGH=0.96
（=MATCH_CARRY_Z）=0.782m，均在手指朝下可达范围（B3L 实测 0.841m 才 FAIL）。
**必须手指朝下**。

流程（一次持握）：
  ① 取火柴：默认朝向（手指朝下）高位接近 MATCH_HIGH=0.96 → 竖直下探到杆身中部抓点
     （两指从 ±Y 夹杆，杆 Ø3mm 水平朝 +X），合爪夹住（GRIP_MATCH 贴合杆径）。
  ② 高位运移：竖直提起火柴到 MATCH_CARRY_Z=0.96（高于火焰尖 0.936 / 灯体顶 0.8897，
     横越灯体不穿），直推 -X 到灯芯上方（IGNITE 同高）。
  ③ 下探触芯：竖直下探 4.5cm 到 IGNITE（头中心 = WICK 正上方 1.5cm，dwell 20 帧点火检测）。
  ④ 抬离绕焰：竖直抬回 0.96（>火焰尖，避开刚点着的火焰柱）。
  ⑤ 放回：直退 +X 回火柴原位上方 → 松爪 → 高位归位（task 火柴生命周期写回 rest）。

持握 = 纯平移 offset（task.MATCH_HELD_OFFSET）：火柴全程水平头朝 +X，不随夹爪旋转
（夹爪朝下夹其杆身，火柴姿态恒等旋转只跟夹爪平移）。火柴头相对夹爪 =
MATCH_TIP_OFFSET=(0.0494,0,0)（抓点 x=0.04、头中心 x=0.0894 → 头在夹爪 +X 0.0494）。

轨迹（TCP 世界坐标，全段默认手指朝下）：
  ① 高位接近   mv((mx,my,MATCH_HIGH))
  ② 竖直下探   mv(MATCH_GRASP)
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_MATCH, 60)      # task 检测 attached
  ⑤ 竖直提出   mv((mx,my,MATCH_CARRY_Z), 5)
  ⑥ 直推-X     mv((IGNITE[0],my,MATCH_CARRY_Z))   # 灯芯正上方（高位，绕焰）
  ⑦ 下探触芯   mv(IGNITE, 20)             # 头落芯上方 1.5cm，dwell 点火
  ⑧ 抬离绕焰   mv((IGNITE[0],my,MATCH_CARRY_Z), 5)
  ⑨ 直退放回   mv((mx,my,MATCH_CARRY_Z))  # 纯 +X 直退（头在火焰尖上方绕走）
  ⑩ 松爪释放   grip(GRIP_OPEN, 25)        # task: released → 火柴写回 rest
  ⑪ 高位归位   mv((mx,my,MATCH_HIGH))
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (SETTLE, GRIP_OPEN, GRIP_MATCH, MATCH_GRASP,
                        MATCH_CARRY_Z, IGNITE, MATCH_HIGH)


class LightFlamePass(BaseMetaAction):
    """取火柴 → 高位运移 → 下探触灯芯点燃 → 抬离绕焰 → 直退放回。全段手指朝下。"""

    def _build_actions(self):
        e = self.engine
        mx, my, _ = MATCH_GRASP
        ix, _, _ = IGNITE
        return [
            mv(e, (mx, my, MATCH_HIGH)),                    # ① 高位接近（手指朝下）
            mv(e, MATCH_GRASP),                             # ② 竖直下探到杆身中部（两指夹杆）
            hold(e, SETTLE),                                # ③ 停顿稳定（合爪前多停稳）
            grip(e, GRIP_MATCH, 60),                        # ④ 合爪夹住火柴（task 检测 attached）
            mv(e, (mx, my, MATCH_CARRY_Z), 5),              # ⑤ 竖直提出（高位 0.96，绕焰）
            mv(e, (ix, my, MATCH_CARRY_Z)),                 # ⑥ 直推 -X 到灯芯正上方（高位）
            mv(e, IGNITE, 20),                              # ⑦ 下探触芯点燃（头落芯上方 1.5cm，dwell 20）
            mv(e, (ix, my, MATCH_CARRY_Z), 5),              # ⑧ 抬离绕焰（>火焰尖 0.936）
            mv(e, (mx, my, MATCH_CARRY_Z)),                 # ⑨ 纯 +X 直退放回（头在火焰上方绕走）
            grip(e, GRIP_OPEN, 25),                         # ⑩ 松爪：task 火柴写回 rest
            mv(e, (mx, my, MATCH_HIGH)),                    # ⑪ 高位归位
        ]
