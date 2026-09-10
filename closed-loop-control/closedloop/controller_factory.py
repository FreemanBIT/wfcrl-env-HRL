"""controller_factory.py — 控制器统一构建（示例与 demos 共用）。

把 schemes/greedy 的实例化逻辑集中在此，避免 launcher/bridge/wfcrl/demos
四处重复（REFACTOR_PLAN 第 2 部分）。
"""
from __future__ import annotations

from typing import Optional

from .case_config import FarmCase
from .control_mode import ControlMode
from .surrogate import SurrogateModel


def build_controller(name: str, mode: int, case: FarmCase, surrogate: SurrogateModel,
                     dt: float, seed: int = 0, mpc_t_ctrl: Optional[float] = None):
    """实例化指定控制器。

    name: greedy / A / B / C（大小写不敏感）
    mode: ControlMode（1=YAW, 2=INDUCTION, 3=YAW_INDUCTION；4/5 已删除）
    mpc_t_ctrl: ControllerB 的 MPC 重规划周期（s），None 用默认 20s
    """
    m = ControlMode(mode)
    name = name.lower()
    if name == 'greedy':
        from .demos.greedy_baseline import GreedyController
        return GreedyController(case.n_turbines, dt)
    if name == 'a':
        from .scheme_a import ControllerA
        return ControllerA(m, surrogate, case.n_turbines)
    if name == 'b':
        from .scheme_b.controller_b import ControllerB, SchemeBConfig
        cfg = SchemeBConfig(t_ctrl=mpc_t_ctrl) if mpc_t_ctrl else None
        return ControllerB(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, config=cfg, seed=seed)
    if name == 'c':
        from .scheme_c import ControllerC
        return ControllerC(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=seed)
    raise ValueError(f"unknown controller: {name}")


__all__ = ['build_controller']