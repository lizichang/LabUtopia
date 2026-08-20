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
"""
from ..atomic_actions.flametest.grip_action import GripAction
from ..atomic_actions.flametest.hold_action import HoldAction
from ..atomic_actions.flametest.move_action import MoveAction
from ..atomic_actions.flametest.shake_action import ShakeAction
from .constants import GRIP_OPEN


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


def mv(engine, pos, dwell=0, orient=None, linewalk=True):
    """移动到 pos（可带停留/朝向）。orient=None 沿用引擎默认朝向（手指朝下）。
    linewalk=False 强制单次 IK（近奇异短距下降用，见 MoveAction 注释）。"""
    return MoveAction(engine, pos, dwell, orient=orient, linewalk=linewalk)


def grip(engine, width, dwell=25):
    return GripAction(engine, width, dwell)


def hold(engine, n):
    return HoldAction(engine, n)


def shake(engine, center, axis=(1, 0, 0), amplitude=0.02, cycles=3, period=60):
    """在 center 附近沿 axis 正弦振荡 cycles 个来回（默认 x 轴 ±20mm、1s/来回）。"""
    return ShakeAction(engine, center, axis=axis, amplitude=amplitude,
                       cycles=cycles, period=period)
