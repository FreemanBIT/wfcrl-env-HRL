"""
C5 — Scheme C controller (grey-box augmentation).

Two routes, selectable at construction:

  * ``route="greybox_mpc"`` (default): reuse Scheme B's EnKF + MPC, but with the
    surrogate wrapped by the learned residual (:class:`GreyBoxSurrogate`). This
    is the low-risk, interpretable path; the residual is trained offline from
    A/B run data via :class:`JointEstimator` and passed in.

  * ``route="rl_safety"``: use a (pre-trained) policy for real-time action, then
    project onto the physics-certified safe set (:class:`SafetyProjector`). The
    safety projection guarantees no worse than (1-eps) x physical baseline.

Both routes support all 5 control modes (M9).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..types import FlowEstimate, TurbineMeas, Cmd
from ..base_controller import BaseController
from ..control_mode import ControlMode, A_GREEDY
from ..surrogate import SurrogateModel
from ..scheme_b.controller_b import ControllerB, SchemeBConfig
from ..scheme_b.enkf import EnKFConfig
from ..scheme_b.mpc import MPCConfig
from ..scheme_b.floridyn_model import FLORIDynConfig
from .residual_model import GreyBoxSurrogate, ResidualModel
from .rl_policy import SafetyProjector, LinearPolicy


@dataclass
class SchemeCConfig:
    dt: float = 2.0
    route: str = "greybox_mpc"     # "greybox_mpc" | "rl_safety"
    t_ctrl: float = 20.0
    safety_epsilon: float = 0.02


class ControllerC(BaseController):
    """Scheme C controller."""

    def __init__(
        self,
        mode: ControlMode,
        surrogate: SurrogateModel,
        layout_x: list[float],
        layout_y: list[float],
        n_turbines: int,
        residual: Optional[ResidualModel] = None,
        policy: Optional[object] = None,
        config: Optional[SchemeCConfig] = None,
        seed: int = 0,
    ):
        cfg = config or SchemeCConfig()
        super().__init__(mode, n_turbines, cfg.dt)
        self.cfg = cfg
        self.phys = surrogate
        self.greybox = GreyBoxSurrogate(surrogate, residual)

        if cfg.route == "greybox_mpc":
            # reuse Scheme B, but hand it the grey-box surrogate
            self._inner = ControllerB(
                mode, self.greybox, layout_x, layout_y, n_turbines,
                SchemeBConfig(dt=cfg.dt, t_ctrl=cfg.t_ctrl, enable_enkf=True),
                FLORIDynConfig(dt=cfg.dt),
                EnKFConfig(n_ensemble=20, estimate_params=True),
                MPCConfig(dt=cfg.dt),
                seed=seed,
            )
            self._projector = None
            self._policy = None
        elif cfg.route == "rl_safety":
            self._inner = None
            self._policy = policy or LinearPolicy(n_turbines)
            self._projector = SafetyProjector(surrogate, epsilon=cfg.safety_epsilon)
        else:
            raise ValueError(f"unknown route: {cfg.route}")

    def reset(self) -> None:
        if self._inner is not None:
            self._inner.reset()

    def step(self, flow_est: FlowEstimate, meas: dict[int, TurbineMeas]) -> dict[int, Cmd]:
        if self.cfg.route == "greybox_mpc":
            # Scheme B controller already emits mode-aware commands using the
            # grey-box surrogate for both EnKF and MPC.
            return self._inner.step(flow_est, meas)

        # --- rl_safety route ---
        yaw_prop, a_prop = self._policy.act(flow_est, self.spec)
        yaw, a, accepted = self._projector.project(
            yaw_prop, a_prop, flow_est.U_inf, flow_est.phi, flow_est.TI_est,
        )
        cmds: dict[int, Cmd] = {}
        for tid in range(1, self.n_turbines + 1):
            i = tid - 1
            U_eff = flow_est.U_eff(tid)
            cmds[tid] = self.spec.to_controls(
                turbine_id=tid, yaw_deg=float(yaw[i]), a=float(a[i]), U_eff=U_eff
            )
        return cmds
