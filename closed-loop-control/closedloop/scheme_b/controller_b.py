"""
B5 — Scheme B controller (FLORIDyn + EnKF + MPC).

Per control step:
    sensing  -> EnKF.update(measured power) corrects FLORIDyn ambient + params
             -> MPC.solve(posterior) optimizes yaw/induction over a horizon that
                covers the wake-travel delay
             -> emit mode-appropriate commands.

Supports all 5 control modes (M9): the MPC's decision variables are selected by
``ControlModeSpec``.

To keep per-step cost bounded, the EnKF runs every step (cheap) while the MPC
re-plan can run every ``t_ctrl`` seconds (configurable); between re-plans the
last plan is held.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..types import FlowEstimate, TurbineMeas, Cmd
from ..base_controller import BaseController
from ..control_mode import ControlMode
from ..surrogate import SurrogateModel
from .floridyn_model import FLORIDyn, FLORIDynConfig
from .enkf import EnKF, EnKFConfig
from .mpc import MPCController, MPCConfig


@dataclass
class SchemeBConfig:
    dt: float = 2.0
    t_ctrl: float = 20.0             # MPC re-plan interval (s)
    enable_enkf: bool = True


class ControllerB(BaseController):
    """Scheme B controller."""

    def __init__(
        self,
        mode: ControlMode,
        surrogate: SurrogateModel,
        layout_x: list[float],
        layout_y: list[float],
        n_turbines: int,
        config: Optional[SchemeBConfig] = None,
        floridyn_config: Optional[FLORIDynConfig] = None,
        enkf_config: Optional[EnKFConfig] = None,
        mpc_config: Optional[MPCConfig] = None,
        seed: int = 0,
    ):
        cfg = config or SchemeBConfig()
        super().__init__(mode, n_turbines, cfg.dt)
        self.cfg = cfg
        self.model = FLORIDyn(surrogate, layout_x, layout_y,
                              floridyn_config or FLORIDynConfig(dt=cfg.dt))
        self.enkf = EnKF(self.model, enkf_config or EnKFConfig(), seed=seed)
        self.mpc = MPCController(self.model, mpc_config or MPCConfig(dt=cfg.dt))

        self._initialized = False
        self._last_ctrl_t = -1e9
        self._yaw_sol = np.zeros(n_turbines)
        self._a_sol = np.full(n_turbines, 1.0 / 3.0)

    def reset(self) -> None:
        self._initialized = False
        self._last_ctrl_t = -1e9
        self._yaw_sol[:] = 0.0
        self._a_sol[:] = 1.0 / 3.0
        self.mpc.reset()

    def step(self, flow_est: FlowEstimate, meas: dict[int, TurbineMeas]) -> dict[int, Cmd]:
        t = flow_est.t
        ids = sorted(meas.keys())
        p_meas = np.array([meas[i].genpwr_kw for i in ids]) if ids else np.zeros(self.n_turbines)
        wind_meas = np.array([meas[i].wind_x for i in ids]) if ids else None

        # initialize model + filter on first call
        if not self._initialized:
            self.model.set_conditions(flow_est.U_inf, flow_est.phi, flow_est.TI_est)
            self.model.set_controls(self._yaw_sol, self._a_sol)
            self.model.step()
            if self.cfg.enable_enkf:
                self.enkf.initialize(flow_est.U_inf, flow_est.phi, flow_est.TI_est)
            self._initialized = True

        # --- observer: EnKF correction (cheap, every step) ---
        if self.cfg.enable_enkf and len(p_meas) == self.n_turbines:
            self.model.set_controls(self._yaw_sol, self._a_sol)
            self.enkf.update(p_meas, self._yaw_sol, self._a_sol, meas_wind=wind_meas)
        else:
            # no filter: just track sensed ambient
            self.model.set_conditions(flow_est.U_inf, flow_est.phi, flow_est.TI_est)

        # --- controller: MPC re-plan on schedule ---
        if (t - self._last_ctrl_t) >= self.cfg.t_ctrl:
            U = self.model._U_inf
            phi = self.model._phi
            TI = self.model._TI
            self._yaw_sol, self._a_sol = self.mpc.solve(self.spec, U, phi, TI)
            self._last_ctrl_t = t

        # advance the model's OP registers with the applied control
        self.model.set_controls(self._yaw_sol, self._a_sol)
        self.model.step()

        # --- assemble commands (mode-aware) ---
        cmds: dict[int, Cmd] = {}
        for tid in range(1, self.n_turbines + 1):
            i = tid - 1
            U_eff = flow_est.U_eff(tid)
            cmds[tid] = self.spec.to_controls(
                turbine_id=tid,
                yaw_deg=float(self._yaw_sol[i]),
                a=float(self._a_sol[i]),
                U_eff=U_eff,
            )
        return cmds
