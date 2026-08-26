"""元动作 ②：取反应试管 → 移到试纸湿润端正下方（管口距试纸 2.5cm，不接触）。

试管 Ø19.2×153mm 竖直立在试管架前排左孔。**侧面横夹**（2026-08-26 用户：竖直下探夹试管
会穿模，改手指朝前 ORIENT_FWD 横着夹管身，d2s 夹药匙/洗瓶同款）：夹爪先到管口 x 偏 -X
侧（管身 -X 壁外侧），再水平移 +X 入管身中心夹持，手指水平不戳进试管架顶板（0.917）。

持握 = 纯平移跟随（试管 translate=底中心 = tool_center + (0,0,-0.139)，抓点处 = rest 0.806
零跳变；手指水平横夹，管身竖直、管底仍吊在夹爪正下方，纯平移保竖立）。task 期间试管内
液体（TubeSolution 父 prim）跟随平移。检测位 = 试管口升到试纸湿润端下方 2.5cm（管口 0.965，
抓点 0.951），气体上升使试纸变色（task 驱动）。

轨迹（8 段，含中间节点防穿模，全程 orient=ORIENT_FWD）：
  ① 高位试管上方 → ② 降到抓点高度但 x 偏 -X 侧 → ③ 水平移 +X 入管身中心夹持 → ④ 合爪夹试管
  → ⑤ 提预抓点(1.02) → ⑥ 提安全高位 H → ⑦ 水平移试纸湿润端正上方 → ⑧ 垂直下降到检测位
  （管口 0.965）。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_TUBE, ORIENT_FWD, TUBE_XY, TUBE_GRASP,
                        TUBE_APPROACH_DX, TUBE_PRE_Z, PAPER_WET_XY,
                        TUBE_UNDER_PAPER_TCP)


class MoveTubeUnderPaper(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        wx, wy = PAPER_WET_XY
        ax = tx - TUBE_APPROACH_DX          # 侧面接近 x（管身 -X 壁外侧）
        t_high = (tx, ty, H)
        t_pre = (tx, ty, TUBE_PRE_Z)
        w_high = (wx, wy, H)
        return [
            mv(e, t_high, orient=ORIENT_FWD),                   # ① 试管上方高位
            mv(e, (ax, ty, TUBE_GRASP[2]), orient=ORIENT_FWD),  # ② 降到抓点高度但 x 偏 -X 侧
            mv(e, TUBE_GRASP, orient=ORIENT_FWD),               # ③ 水平移 +X 入管身中心（锁 y/z）
            grip(e, GRIP_TUBE, 60),                             # ④ 合爪横夹试管
            mv(e, t_pre, orient=ORIENT_FWD),                    # ⑤ 提回预抓点（先垂直提离）
            mv(e, t_high, orient=ORIENT_FWD),                   # ⑥ 提到安全高位
            mv(e, w_high, orient=ORIENT_FWD),                   # ⑦ 水平移试纸湿润端正上方
            mv(e, TUBE_UNDER_PAPER_TCP, orient=ORIENT_FWD),     # ⑧ 垂直下降到检测位（管口 0.965）
        ]
