"""
A3 — Scheme A controller (calibration + steady re-optimization).

Orchestrates, per control step:
    if T_cal reached : calibrate FLORIS params from the recent window
    if T_ctrl reached or wind bin changed : re-solve steady optimum (mode-aware)
    assemble mode-appropriate commands and emit.

Supports all 5 control modes (M9): the optimizer's decision variables are
selected by ``ControlModeSpec``.
"""
from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..types import FlowEstimate, TurbineMeas, Cmd
from ..base_controller import BaseController
from ..control_mode import ControlMode
from ..surrogate import SurrogateModel
from .calibration import Calibrator, CalibrationConfig
from .steady_opt import SteadyOptimizer, OptConfig


@dataclass
class SchemeAConfig:
    dt: float = 2.0
    t_ctrl: float = 30.0             # re-optimize interval (s)
    t_cal: float = 600.0             # calibration interval (s)
    cal_window_s: float = 300.0      # window of data used for calibration
    bin_tol_dir: float = 3.0         # wind-bin change threshold (deg)
    bin_tol_speed: float = 1.0       # (m/s)
    enable_calibration: bool = True


class ControllerA(BaseController):
    """Scheme A controller."""

    def __init__(
        self,
        mode: ControlMode,
        surrogate: SurrogateModel,
        n_turbines: int,
        config: Optional[SchemeAConfig] = None,
        cal_config: Optional[CalibrationConfig] = None,
        opt_config: Optional[OptConfig] = None,
    ):
        cfg = config or SchemeAConfig()
        super().__init__(mode, n_turbines, cfg.dt)
        self.cfg = cfg
        self.sur = surrogate
        self.calibrator = Calibrator(surrogate, cal_config)
        self.optimizer = SteadyOptimizer(surrogate, opt_config)

        self._last_ctrl_t = -1e9
        self._last_cal_t = -1e9
        self._last_bin: Optional[tuple[float, float]] = None
        self._yaw_sol = np.zeros(n_turbines)
        self._a_sol = np.full(n_turbines, 1.0 / 3.0)
        # calibration window buffers
        win = max(1, int(cfg.cal_window_s / max(cfg.dt, 1e-6)))
        self._pwin: dict[int, deque] = defaultdict(lambda: deque(maxlen=win))

    def reset(self) -> None:
        self._last_ctrl_t = -1e9
        self._last_cal_t = -1e9
        self._last_bin = None
        self._yaw_sol[:] = 0.0
        self._a_sol[:] = 1.0 / 3.0
        self._pwin.clear()

    def step(self, flow_est: FlowEstimate, meas: dict[int, TurbineMeas]) -> dict[int, Cmd]:
        t = flow_est.t
        # accumulate window power for calibration
        for tid, m in meas.items():
            self._pwin[tid].append(m.genpwr_kw)

        # --- calibration (slow loop) ---
        if (self.cfg.enable_calibration and
                (t - self._last_cal_t) >= self.cfg.t_cal and
                self._window_ready()):
            self._run_calibration(flow_est)
            self._last_cal_t = t

        # --- re-optimization (control loop) ---
        bin_now = (round(flow_est.U_inf, 1), round(flow_est.phi, 1))
        bin_changed = self._bin_changed(bin_now)
        if (t - self._last_ctrl_t) >= self.cfg.t_ctrl or bin_changed:
            self._yaw_sol, self._a_sol = self.optimizer.solve(
                self.spec, flow_est.U_inf, flow_est.phi, flow_est.TI_est
            )
            self._last_ctrl_t = t
            self._last_bin = bin_now

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

    # -- helpers ---------------------------------------------------------------
    def _window_ready(self) -> bool:
        return all(len(self._pwin[t]) >= 3 for t in range(1, self.n_turbines + 1)
                   if t in self._pwin) and len(self._pwin) == self.n_turbines

    def _run_calibration(self, flow_est: FlowEstimate) -> dict:
        p_meas = np.array([
            np.mean(self._pwin[t]) if self._pwin[t] else 0.0
            for t in range(1, self.n_turbines + 1)
        ])
        result = self.calibrator.run(
            p_meas_kw=p_meas,
            U_inf=flow_est.U_inf,
            phi=flow_est.phi,
            TI=flow_est.TI_est,
            yaw=self._yaw_sol,
            induction=self._a_sol,
        )
        return result

    def _bin_changed(self, bin_now) -> bool:
        if self._last_bin is None:
            return True
        du = abs(bin_now[0] - self._last_bin[0])
        dphi = abs(bin_now[1] - self._last_bin[1])
        return du >= self.cfg.bin_tol_speed or dphi >= self.cfg.bin_tol_dir
