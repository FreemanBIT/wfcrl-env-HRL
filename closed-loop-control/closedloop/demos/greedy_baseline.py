"""
D1 — Greedy baseline controller.

The reference "do-nothing" strategy: every turbine aligns with the inflow
(yaw = 0) and runs at the Betz-optimal induction (a = 1/3). No wake steering,
no derating. This is the baseline every scheme is measured against (the
denominator of dP%), and it equals mode-1 with zero action (cross-checks M9).
"""
from __future__ import annotations

from ..types import FlowEstimate, TurbineMeas, Cmd
from ..base_controller import BaseController
from ..control_mode import ControlMode, greedy_cmd


class GreedyController(BaseController):
    """Aligns all turbines to the wind and runs greedy (a = 1/3)."""

    def __init__(self, n_turbines: int, dt: float = 2.0):
        super().__init__(ControlMode.YAW, n_turbines, dt)

    def step(
        self, flow_est: FlowEstimate, meas: dict[int, TurbineMeas]
    ) -> dict[int, Cmd]:
        cmds: dict[int, Cmd] = {}
        for tid in range(1, self.n_turbines + 1):
            U_eff = flow_est.U_eff(tid)
            cmds[tid] = greedy_cmd(tid, U_eff)
        return cmds
