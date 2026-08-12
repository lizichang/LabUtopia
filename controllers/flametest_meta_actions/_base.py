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
"""
from ..atomic_actions.flametest.grip_action import GripAction
from ..atomic_actions.flametest.hold_action import HoldAction
from ..atomic_actions.flametest.move_action import MoveAction
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


def mv(engine, pos, dwell=0):
    return MoveAction(engine, pos, dwell)


def grip(engine, width, dwell=25):
    return GripAction(engine, width, dwell)


def hold(engine, n):
    return HoldAction(engine, n)
