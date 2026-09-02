"""元动作基类：把一串原子动作（flametest/ 子包）顺序推进，夹爪目标跨动作持久。

BaseMetaAction 组合 IkMotionEngine + MoveAction/GripAction/HoldAction：
  - _build_actions() 返回原子动作实例列表（子类实现 = 该元动作的轨迹）。
  - forward(state)：推当前原子动作；is_done() 后进下一个；grip_target 持久化
    （GripAction 完成时记下 width，后续 Move/Hold 每帧发送，v41 教训）。
  - is_done()：整个列表跑完。
reset()：从头再跑（子类必须调用 super().reset()）。

工厂（供 _build_actions 拼序列）：
  mv(pos, dwell=0)        移动到 pos（到达冻结后 dwell 帧）
  grip(width, dwell=25)   原地合爪/开爪到 width
  hold(n)                 停在当前位置 n 帧
  shake(center, ...)      在 center 附近水平正弦振荡（"震荡来回 N 下"，抓试管用）
  stir(center, ...)       绕 center 水平圆周搅拌（"搅拌 N 圈"，棒插烧杯画圆用）
"""
from ..atomic_actions.flametest.grip_action import GripAction
from ..atomic_actions.flametest.hold_action import HoldAction
from ..atomic_actions.flametest.move_action import MoveAction
from ..atomic_actions.flametest.shake_action import ShakeAction
from ..atomic_actions.flametest.stir_action import StirAction
GRIP_OPEN = 0.04  # 夹爪开度（constants.py 已迁入 catalogue，此值同 catalogue/_shared/constants.py）


class BaseMetaAction:
    """顺序执行一串原子动作的元动作。子类实现 _build_actions()。"""

    def __init__(self, engine):
        self.engine = engine
        self._actions = self._build_actions()
        self._idx = 0
        self.grip_target = GRIP_OPEN
        self._cur = self._actions[0] if self._actions else None
        self._done = False

    def _build_actions(self):
        raise NotImplementedError

    def reset(self):
        self._idx = 0
        self.grip_target = GRIP_OPEN
        self._done = False
        for a in self._actions:
            a.reset()
        self._cur = self._actions[0] if self._actions else None

    def forward(self, state):
        if self._cur is None:
            return None
        action = self._cur.forward(
            state["joint_positions"], state["gripper_position"], self.grip_target)
        if self._cur.is_done():
            if isinstance(self._cur, GripAction):
                self.grip_target = self._cur.width
            self._idx += 1
            if self._idx < len(self._actions):
                self._cur = self._actions[self._idx]
                self._cur.reset()
            else:
                self._done = True
        return action

    def is_done(self):
        return self._done


def mv(engine, pos, dwell=0, orient=None, linewalk=True, orient_eps=None,
       freeze_dist=0.010, timeout=None):
    """移动到 pos（可带停留/朝向）。orient=None 沿用引擎默认朝向（手指朝下）。
    linewalk=False 强制单次 IK（近奇异短距下降用，见 MoveAction 注释）。
    orient_eps：朝向收敛阈值覆盖（None=全局 ORIENT_EPS；插管入孔段传紧值如
    0.01 rad ≈0.57°，见 B1 试管放回）。
    freeze_dist：冻结距离阈值（吸附式抓取低 z IK 死区传 0.03 与 near 一致，
    机械臂到 near 范围即 freeze，不 force-done 25 秒）。
    timeout：超时预算帧（近奇异低 z 段传更小值快速放弃，None=全局 1500）。"""
    return MoveAction(engine, pos, dwell, orient=orient, linewalk=linewalk,
                      orient_eps=orient_eps, freeze_dist=freeze_dist,
                      timeout=timeout)


def grip(engine, width, dwell=25):
    return GripAction(engine, width, dwell)


def hold(engine, n):
    return HoldAction(engine, n)


def shake(engine, center, axis=(1, 0, 0), amplitude=0.02, cycles=3, period=60,
          orient=None):
    """在 center 附近沿 axis 正弦振荡 cycles 个来回（默认 x 轴 ±20mm、1s/来回）。

    orient=None 沿用引擎默认朝向（手指朝下）；显式传时震荡全程保持该朝向
    （D2-S 试管远在 +X，手指朝前 ORIENT_FWD 才够得着，见 ShakeAction）。
    """
    return ShakeAction(engine, center, axis=axis, amplitude=amplitude,
                       cycles=cycles, period=period, orient=orient)


def stir(engine, center, radius=0.015, cycles=3, period=45, orient=None):
    """绕 center 水平圆周搅拌 cycles 圈（z 锁 center[2]，棒插烧杯后画圆搅拌）。

    orient=None 沿用引擎默认朝向（手指朝下）；显式传时搅拌全程保持该朝向
    （A3 棒横夹持握用 ORIENT_FWD，见 StirAction）。
    """
    return StirAction(engine, center, radius=radius, cycles=cycles,
                      period=period, orient=orient)
