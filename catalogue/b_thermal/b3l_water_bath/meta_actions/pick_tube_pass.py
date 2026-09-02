"""元动作：倒粉后水平横夹试管（跟夹药匙一样 ORIENT_FWD）→ 纯平移分段转烧杯水浴 → 竖直浸入
保持夹持（不松爪），加热结束 ReturnTubePass 提回架孔放回。

用户逐字（2026-08-29）：
  ①「倒完粉末后，没有水平夹住试管你仔细看看b1怎么写的！」→ 持握/夹取完全照 B1 PickTubePass
    （ORIENT_FWD 水平横夹 + _T_HELD_TUBE 矩阵持握），但 **B3 无法兰翻转**：试管由架孔提出后
    竖直吊在夹爪下（管口朝上），直接横移到烧杯正上方、竖直浸入水浴。
  ②「现在动作是先点燃酒精灯，然后再拿起试管过去加热」→ 段1 顺序 = PickSpatula → ReturnSpatula
    → LightFlamePass（先点火）→ PickTubePass（再拿试管）。
  ③「拿试管的过程中不是平移过去的，而是中间过程有反转，这样试剂不都掉出来了吗」→ 转移路径改
    纯平移分段：⑤ 竖直提出到 TUBE_TRANSIT_Z=1.174 → ⑥ 水平横移（z 恒定，纯水平）→ ⑦ 竖直浸入，
    试管全程竖直（ORIENT_FWD + _T_HELD_TUBE 组合旋转 = 恒等）不翻转、管内试剂不洒。
  ④「拿试管加热的时候机械臂不能松手，直到加热结束才放回去」→ ⑦ 浸入后**不松爪**，机械臂保持
    夹持在浸入位直到加热结束；task 检测近浸入点连续帧置 tube_immersed（解除 idle 门控开始加热）。

为什么 ORIENT_FWD 水平横夹：试管在 (0.659,0.241) 处竖直下探（手指朝下）IK 不可达——D2S S6
已踩（日志 `IK FAIL target=[0.659 0.241 ...]`，Lula 解不出手指朝下的低 z 远点）。夹药匙的
ORIENT_FWD（手指朝 +X，tool+Z=+X）在 (0.659,0.241) 处**高 z 可达**（① 1.15 冻住），但**低 z
死区**（2026-08-30 运行 log：下探 0.9453 force-done、手指悬管口上方几毫米；同底座药匙
(0.699,0.361) 低 z 0.94 却可达——试管 y rel=0.191 < 药匙 0.311，缺侧向肘部空间）。
修：抓点抬到管口顶 TUBE_GRASP_TCP=(0.659,0.241,0.9593)（可达极限最高点）+ task near z 窗放宽
±0.03（机械臂停在管口上方几毫米即吸附）。

持握 = 矩阵持握（_T_HELD_TUBE，见 task.py）：试管世界位姿 = _T_HELD_TUBE · tool_world，
随夹爪 6-DOF 刚性跟随；管内粉末柱随管刚性跟随（task._set_tube_world 清 op 序写单一
transform op）。_T_HELD_TUBE 旋转 = toolX→(0,0,1)、toolY→(0,1,0)、toolZ→(-1,0,0)（管口朝上、
管底吊夹爪下 TUBE_HELD_X=0.1533）+ 平移 +TUBE_HELD_X 沿 tool-X；ORIENT_FWD 组合后试管世界
旋转 = 恒等（= 架孔竖插静置旋转）→ 抓点吸附零跳变（B1 pxr 数值验证）。

流程（7 步，全程 ORIENT_FWD、无法兰翻转、不松爪）：
  ① 高位接近   mv((TUBE_XY, H))                    # 高位接近试管上方（水平横夹朝向）
  ② 水平横夹下探 mv(TUBE_GRASP_TCP)                 # 抓管口顶 (0.9593)（低 z IK 死区，见上；手指朝 +X）
  ③ 停顿稳定   hold(SETTLE)
  ④ 合爪夹紧   grip(GRIP_TUBE, 60)                  # task: 近抓点+闭爪 → tube attached
  ⑤ 竖直提出   mv((TUBE_XY, TUBE_TRANSIT_Z))        # 到横移高度（管底 1.0207 清架顶 0.917/烧杯口 1.0109）
  ⑥ 水平横移   mv(TUBE_TRANSIT)                     # 纯水平移烧杯正上方（z 恒定 1.174，不反转）
  ⑦ 竖直浸入   mv(TUBE_IMMERSE_TCP, dwell=30)       # 管底 0.9255 贴烧杯底 0.9205 上 5mm；dwell 让
                                                     # task 近浸入点判定 tube_immersed；不松爪保持夹持

task 侧：tube immersed（近浸入点连续帧，不要求开爪）→ tube_immersed 解除 idle 门控开始加热；
加热结束后 ReturnTubePass 把试管提回架孔，_TubeLifecycle 检测近架孔抓点+开爪 → released →
写 _tube_rest_matrix() + tube_returned（见 return_tube_pass.py）。
"""
from ._base import BaseMetaAction, mv, grip, hold
from .constants import (H, SETTLE, ORIENT_FWD, GRIP_TUBE,
                        TUBE_XY, TUBE_GRASP_TCP,
                        TUBE_TRANSIT_Z, TUBE_TRANSIT, TUBE_IMMERSE_TCP)


class PickTubePass(BaseMetaAction):
    """水平横夹试管（跟夹药匙一样，ORIENT_FWD）→ 纯平移分段转烧杯水浴 → 竖直浸入保持夹持（不松爪）。"""

    def _build_actions(self):
        e = self.engine
        tx, ty = TUBE_XY
        return [
            mv(e, (tx, ty, H), orient=ORIENT_FWD),                  # ① 高位接近试管上方（水平横夹朝向）
            mv(e, TUBE_GRASP_TCP, orient=ORIENT_FWD, freeze_dist=0.03, timeout=200),  # ② 下探抓管口顶（0.9593）；低z IK死区→near即freeze+4s兜底
            hold(e, SETTLE),                                        # ③ 停顿稳定
            grip(e, GRIP_TUBE, 60),                                 # ④ 合爪夹紧（task: tube attached）
            mv(e, (tx, ty, TUBE_TRANSIT_Z), orient=ORIENT_FWD),     # ⑤ 竖直提出到横移高度（管底1.0207清架顶0.917）
            mv(e, TUBE_TRANSIT, orient=ORIENT_FWD),                 # ⑥ 水平横移烧杯正上方（z恒定1.174，纯水平不反转）
            mv(e, TUBE_IMMERSE_TCP, orient=ORIENT_FWD, dwell=30),   # ⑦ 竖直浸入水浴（管底0.9255贴烧杯底上5mm），
                                                                    #    dwell 让 task 判 tube_immersed；不松爪保持夹持
        ]
