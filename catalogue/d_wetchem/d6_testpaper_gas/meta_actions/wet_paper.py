"""元动作 ①：取蒸馏水滴管 → 滴 1-2 滴润湿试纸湿润端 → 归位（D6 润湿）。

滴管 Ø11mm 竖直立在试管架后排右孔，手指朝下（默认朝向）竖直下探夹胶头（抓点 0.936）。
持握 = 纯平移跟随（滴管 translate=底中心 = tool_center + (0,0,-0.13)，抓点处=rest 0.806
零跳变）。润湿 = 滴管尖降到试纸湿润端上方 2cm（抓点 1.14），挤胶头 GRIP_SQUEEZE 出 1-2 滴
（task 期间驱动 DropperDrop 水滴坠落动画），再松胶头归位。

轨迹（17 段，含中间节点防穿模）：
  ① 高位滴管上方 → ② 降预抓点(1.02) → ③ 垂直下探抓点(0.936) → ④ 合爪夹滴管 → ⑤ 提预抓点
  → ⑥ 提安全高位 H → ⑦ 水平移试纸湿润端正上方 → ⑧ 垂直下降到润湿点(1.14) → ⑨ 挤胶头出水
  → ⑩ 松胶头 → ⑪ 提回 H → ⑫ 水平移回滴管上方 → ⑬ 降预放点 → ⑭ 垂直降回抓点 → ⑮ 松爪放回
  → ⑯ 提预放点 → ⑰ 撤离回 H。
"""
from ._base import BaseMetaAction, mv, grip
from .constants import (H, GRIP_OPEN, GRIP_DROPPER, GRIP_SQUEEZE,
                        DROPPER_XY, DROPPER_GRASP, DROPPER_PRE_Z,
                        PAPER_WET_XY, WET_TCP)


class WetPaper(BaseMetaAction):
    def _build_actions(self):
        e = self.engine
        dx, dy = DROPPER_XY
        wx, wy = PAPER_WET_XY
        d_high = (dx, dy, H)
        d_pre = (dx, dy, DROPPER_PRE_Z)
        w_high = (wx, wy, H)
        return [
            mv(e, d_high),                  # ① 滴管上方高位
            mv(e, d_pre),                   # ② 降到预抓点（胶头上方中间节点）
            mv(e, DROPPER_GRASP),           # ③ 垂直下探抓胶头
            grip(e, GRIP_DROPPER, 60),      # ④ 合爪夹滴管
            mv(e, d_pre),                   # ⑤ 提回预抓点（先垂直提离）
            mv(e, d_high),                  # ⑥ 提到安全高位
            mv(e, w_high),                  # ⑦ 水平移试纸湿润端正上方
            mv(e, WET_TCP),                 # ⑧ 垂直下降到润湿点（滴管尖 1.01）
            grip(e, GRIP_SQUEEZE, 40),      # ⑨ 挤胶头出 1-2 滴蒸馏水
            grip(e, GRIP_DROPPER, 20),      # ⑩ 松胶头
            mv(e, w_high),                  # ⑪ 提回安全高位
            mv(e, d_high),                  # ⑫ 水平移回滴管上方
            mv(e, d_pre),                   # ⑬ 降到预放点
            mv(e, DROPPER_GRASP),           # ⑭ 垂直降回抓点（滴管已回 rest 位）
            grip(e, GRIP_OPEN, 25),         # ⑮ 松爪放回
            mv(e, d_pre),                   # ⑯ 提回预放点
            mv(e, d_high),                  # ⑰ 撤离回 H
        ]
