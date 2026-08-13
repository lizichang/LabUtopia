"""catalogue 动作库共享控制器基类。

39 个 v10 动作的 controller 统一从这里派生，继承现有
`controllers/base_controller.py` 的 BaseController（RMP 控制器、数据采集、
推理引擎、episode 统计等能力全部复用）。
"""

from typing import Any, Dict, Tuple

from controllers.base_controller import BaseController


class CatalogueBaseController(BaseController):
    """catalogue 复合实验控制器骨架。

    封装 AGENTS.md §3.7 的 Phase 样板（原范本 `controllers/dissolve_controller.py`
    已废弃删除，D2-S 重写后以 `catalogue/d_wetchem/d2s_water_solubility/` 为准）：

    **Controller 侧四件套**
    - `Phase` 枚举（如 SCOOP_POWDER → POUR_WATER → SHAKE → OBSERVE → FINISHED）
    - `_check_phase_success(state)`：读 task state 里的状态机标志/粘性 flag，
      成功才推进（上一个 phase 的 object state == 'released'）
    - `_phase_action(phase, state, ...)`：把 state 里的世界参考点映射到原子动作
      controller 的 `forward()` 参数
    - `_advance_phase()`：切 phase，进入新 phase 前用 `set_initial_position(...)`
      做参数注入（如 `ShakeController.set_initial_position(state['shake_position'])`）

    **子类需实现**
    - `_init_collect_mode(cfg, robot)`：实例化复用的原子动作 controller
      （直接 `from controllers.atomic_actions.xxx import XxxController`，原子动作
      不注册 factory）
    - `_step_collect(state)`：主循环，见下方参考实现

    **原子动作复用**
    - 通用原子动作：`controllers/atomic_actions/`（pick/place/pour/scoop/shake/
      dropper/uncap/cap/ignite/match_ignite/press/move 等 15 个，统一签名
      `__init__(name, cspace_controller, events_dt=None, position_threshold=0.01)`，
      几何参数全部走 `forward()`）
    - 焰色 IK 原子动作：`controllers/atomic_actions/flametest/`
      （IkMotionEngine + MoveAction/GripAction/HoldAction，供 C 类复用）
    """

    def step(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """BaseController 抽象方法实现：按 mode 分派（Phase 样板写法）。"""
        if self.mode == "collect":
            return self._step_collect(state)
        elif self.mode == "infer":
            return self._step_infer(state)
        else:
            raise ValueError(f"Invalid mode: {self.mode}")

    def _step_collect(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """Phase 主循环参考实现（子类按需覆写）。骨架（原 dissolve 实现）：

            1. self._check_phase_success(state)      —— 先判当前 phase 是否成功
            2. active_controller 未完成 → _phase_action 产动作
            3. controller 完成后再判 phase 成功，成功则 _advance_phase
            4. FINISHED 时返回 (action, done=True, is_success)
        """
        raise NotImplementedError(
            f"{type(self).__name__} 未实现 _step_collect：参考 "
            "catalogue/d_wetchem/d2s_water_solubility/ 的 Phase 主循环"
        )

    def _step_infer(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """推理模式：走 inference engine（BaseController._init_infer_mode 已建）。"""
        return self.inference_engine.step_inference(state)
