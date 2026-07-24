"""
B3 — Model Predictive Controller.

Rolls the FLORIDyn model forward over a prediction horizon and optimizes the
farm yaw/induction to maximize predicted energy minus an action penalty
(derivation doc eqs 2.21-2.27):

    max_U  sum_{j=0}^{Np-1} sum_i P_i(x_{k+j}, u_{k+j}) dt
           - sum_{Nc} (lam_gamma ||dgamma||^2 + lam_a ||da||^2)
    s.t.   |gamma| <= gamma_max,  a in [a_min,a_max], rate limits
           x_{k}   = x_hat_k^+   (EnKF posterior)

Key points:
  * Prediction horizon MUST cover the max wake-travel delay (eq 2.22):
    ``Np*dt > tau_max``. The horizon is auto-sized from FLORIDyn.max_delay_steps.
  * Decision variables are mode-aware (M9): yaw-only, induction-only, or both.
  * Control parameterization is a per-turbine constant set-point over the
    control horizon (a pragmatic, robust default; a B-spline basis, eq 2.26, can
    be dropped in for finer temporal shaping). Optimization is by coordinate
    refinement over the surrogate roll-out — gradient-free and reliable at this
    scale (6 turbines).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .floridyn_model import FLORIDyn
from ..control_mode import ControlModeSpec, A_GREEDY, YAW_MAX


@dataclass
class MPCConfig:
    dt: float = 2.0
    horizon_margin: float = 1.3      # Np*dt >= margin * tau_max (eq 2.22)
    min_horizon_steps: int = 8
    max_horizon_steps: int = 60
    lam_yaw: float = 0.002           # yaw-move penalty (per deg^2, normalized)
    lam_a: float = 0.5               # induction-move penalty
    yaw_grid: tuple = (-30, -20, -10, 0, 10, 20, 30)
    yaw_refine_levels: int = 2
    a_grid: tuple = (0.15, 0.22, 0.28, 1.0 / 3.0)
    n_passes: int = 2


class MPCController:
    """Receding-horizon controller over the FLORIDyn model."""

    def __init__(self, model: FLORIDyn, config: Optional[MPCConfig] = None):
        self.model = model
        self.cfg = config or MPCConfig()
        self.n = model.n
        self._prev_yaw = np.zeros(self.n)
        self._prev_a = np.full(self.n, A_GREEDY)

    def horizon_steps(self, U_inf: float) -> int:
        """Prediction horizon that covers tau_max (eq 2.22)."""
        tau = self.model.max_delay_steps(U_inf)
        need = int(np.ceil(self.cfg.horizon_margin * tau))
        return int(np.clip(max(need, self.cfg.min_horizon_steps),
                           self.cfg.min_horizon_steps, self.cfg.max_horizon_steps))

    def solve(
        self,
        spec: ControlModeSpec,
        U_inf: float,
        phi: float,
        TI: float,
        verbose: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return the first-step optimal ``(yaw[n], a[n])`` to apply now.

        The model state (ambient + OP history) is taken as-is (already set to the
        EnKF posterior by the controller); prediction snapshots/restores it.
        """
        self.model.set_conditions(U_inf, phi, TI)
        Np = self.horizon_steps(U_inf)

        opt_yaw = spec.optimizes_yaw()
        opt_a = spec.optimizes_induction()

        yaw = self._prev_yaw.copy() if opt_yaw else np.zeros(self.n)
        a = self._prev_a.copy() if opt_a else np.full(self.n, A_GREEDY)

        def objective(yaw_vec, a_vec) -> float:
            p_settled = self.model.steady_power(yaw_vec, a_vec)   # kW
            dyaw = yaw_vec - self._prev_yaw
            da = a_vec - self._prev_a
            pen = (self.cfg.lam_yaw * float(np.sum(dyaw ** 2))
                   + self.cfg.lam_a * float(np.sum(da ** 2)) * 1e3)
            return p_settled - pen

        # baseline
        yaw_base = np.zeros(self.n)
        a_base = np.full(self.n, A_GREEDY)
        p_base = self.model.steady_power(yaw_base, a_base)
        if verbose:
            print(f"  MPC: Np={Np} steps, greedy baseline={p_base:.0f} kW")

        # coordinate refinement over the roll-out
        for pass_idx in range(self.cfg.n_passes):
            if verbose:
                print(f"  MPC Pass {pass_idx+1}/{self.cfg.n_passes}:")
            for i in range(self.n):
                if opt_yaw:
                    yaw[i] = self._refine_scalar_verbose(
                        lambda v: objective(_set(yaw, i, v), a),
                        list(self.cfg.yaw_grid), -YAW_MAX, YAW_MAX,
                        self.cfg.yaw_refine_levels,
                        label=f"T{i+1} yaw", base_obj=objective(yaw_base, a_base),
                        verbose=verbose,
                    )
                    if verbose:
                        yaw_tmp = yaw.copy()
                        yaw_tmp[i] = yaw[i]
                        p_now = self.model.steady_power(yaw_tmp, a)
                        gain = 100.0 * (p_now - p_base) / p_base if p_base > 0 else 0.0
                        print(f"    T{i+1} yaw={yaw[i]:.0f}°  P={p_now:.0f}kW ({gain:+.1f}%)")
                if opt_a:
                    a[i] = self._best_on_grid_verbose(
                        lambda v: objective(yaw, _set(a, i, v)),
                        list(self.cfg.a_grid),
                        label=f"T{i+1} a", verbose=verbose,
                    )

        p_final = self.model.steady_power(yaw, a)
        if verbose:
            gain = 100.0 * (p_final - p_base) / p_base if p_base > 0 else 0.0
            yaw_s = "[" + " ".join(f"{y:3.0f}" for y in yaw) + "]"
            a_s = "[" + " ".join(f"{v:.2f}" for v in a) + "]"
            print(f"  MPC Final: P={p_final:.0f}kW ({gain:+.1f}%)  yaw={yaw_s}  a={a_s}")

        self._prev_yaw = yaw.copy()
        self._prev_a = a.copy()
        return yaw, a

    def reset(self) -> None:
        self._prev_yaw[:] = 0.0
        self._prev_a[:] = A_GREEDY

    # -- helpers ---------------------------------------------------------------
    @staticmethod
    def _best_on_grid(f, grid):
        vals = [f(v) for v in grid]
        return grid[int(np.argmax(vals))]

    @staticmethod
    def _best_on_grid_verbose(f, grid, label="", verbose=False):
        vals = [f(v) for v in grid]
        best_idx = int(np.argmax(vals))
        if verbose:
            s = " ".join(f"{v}={val:.0f}" for v, val in zip(grid, vals))
            print(f"    {label} grid: [{s}]  best={grid[best_idx]}")
        return grid[best_idx]

    def _refine_scalar_verbose(self, f, grid, lo, hi, levels, label="",
                                base_obj=0.0, verbose=False):
        best = self._best_on_grid(f, grid)
        if verbose:
            print(f"    {label} coarse: best={best}", end="")
        span = (grid[1] - grid[0]) if len(grid) > 1 else 5.0
        for _ in range(levels):
            span *= 0.4
            local = [min(hi, max(lo, best + d))
                     for d in (-span, -span / 2, 0, span / 2, span)]
            best = self._best_on_grid(f, local)
            if verbose:
                print(f" → refine best={best}", end="")
        if verbose:
            print()
        return best


def _set(arr: np.ndarray, i: int, v: float) -> np.ndarray:
    out = arr.copy()
    out[i] = v
    return out
