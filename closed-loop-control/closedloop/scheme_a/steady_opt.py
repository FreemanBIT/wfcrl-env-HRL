"""
A2 — Steady re-optimization (Scheme A controller core).

On the calibrated FLORIS, re-solve the farm yaw + induction optimum for the
current wind bin (eq 1.11):

    max_{gamma,a}  P_farm(gamma, a; theta_hat)
    s.t.  |gamma_i| <= gamma_max,  a_i in [a_min, a_max]

Decision variables are chosen by the control mode (M9): mode 1 optimizes yaw
only (a fixed at greedy), modes 2/3 optimize induction only (yaw fixed 0), modes
4/5 optimize both.

Two solvers:
  * ``serial_refine`` — deterministic per-turbine grid refinement (default,
    robust to the multi-modal yaw landscape).
  * ``slsqp`` — gradient-based (scipy), used when available and requested.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..surrogate import SurrogateModel
from ..control_mode import ControlModeSpec, A_GREEDY, YAW_MAX


@dataclass
class OptConfig:
    method: str = "serial_refine"    # "serial_refine" | "slsqp"
    yaw_grid: tuple = (-30, -20, -10, 0, 10, 20, 30)  # coarse grid (deg)
    yaw_refine_levels: int = 2       # refinement passes around best
    a_grid: tuple = (0.10, 0.20, 0.25, 0.30, 1.0 / 3.0)
    n_passes: int = 2                # sweeps over turbines


class SteadyOptimizer:
    """Solve the steady farm optimum given a surrogate and control mode."""

    def __init__(self, surrogate: SurrogateModel, config: Optional[OptConfig] = None):
        self.sur = surrogate
        self.cfg = config or OptConfig()

    def solve(
        self,
        spec: ControlModeSpec,
        U_inf: float,
        phi: float,
        TI: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(yaw_opt_deg[n], induction_opt[n])`` for the current bin."""
        self.sur.set_conditions(U_inf, phi, TI)
        n = self.sur.n

        # initialize at defaults implied by the mode
        yaw = np.zeros(n)
        a = np.full(n, A_GREEDY)

        if self.cfg.method == "slsqp":
            got = self._solve_slsqp(spec, yaw, a)
            if got is not None:
                return got
            # fall through to serial-refine if scipy unavailable

        return self._solve_serial_refine(spec, yaw, a)

    # -- objective -------------------------------------------------------------
    def _farm_power(self, yaw: np.ndarray, a: np.ndarray) -> float:
        self.sur.set_controls(yaw, a)
        return float(self.sur.predict()["P_farm"])

    # -- serial refine ---------------------------------------------------------
    def _solve_serial_refine(self, spec: ControlModeSpec, yaw, a):
        opt_yaw = spec.optimizes_yaw()
        opt_a = spec.optimizes_induction()
        yaw_lo, yaw_hi = (-YAW_MAX, YAW_MAX)

        for _ in range(self.cfg.n_passes):
            for i in range(self.sur.n):
                if opt_yaw:
                    yaw[i] = self._refine_scalar(
                        lambda v: self._farm_power(_set(yaw, i, v), a),
                        list(self.cfg.yaw_grid), yaw_lo, yaw_hi,
                        self.cfg.yaw_refine_levels,
                    )
                if opt_a:
                    a[i] = self._best_on_grid(
                        lambda v: self._farm_power(yaw, _set(a, i, v)),
                        list(self.cfg.a_grid),
                    )
        return yaw, a

    @staticmethod
    def _best_on_grid(f, grid):
        vals = [f(v) for v in grid]
        return grid[int(np.argmax(vals))]

    def _refine_scalar(self, f, grid, lo, hi, levels):
        best = self._best_on_grid(f, grid)
        span = (grid[1] - grid[0]) if len(grid) > 1 else 5.0
        for _ in range(levels):
            span *= 0.4
            local = [best + d for d in (-span, -span / 2, 0, span / 2, span)]
            local = [min(hi, max(lo, v)) for v in local]
            best = self._best_on_grid(f, local)
        return best

    # -- slsqp -----------------------------------------------------------------
    def _solve_slsqp(self, spec: ControlModeSpec, yaw0, a0):
        try:
            from scipy.optimize import minimize
        except Exception:
            return None

        n = self.sur.n
        opt_yaw = spec.optimizes_yaw()
        opt_a = spec.optimizes_induction()

        x0, bounds, unpack = [], [], []
        if opt_yaw:
            x0 += list(yaw0); bounds += [(-YAW_MAX, YAW_MAX)] * n; unpack.append("yaw")
        if opt_a:
            x0 += list(a0); bounds += [(0.0, A_GREEDY)] * n; unpack.append("a")

        def split(x):
            x = np.asarray(x)
            yaw = yaw0.copy(); a = a0.copy()
            off = 0
            if opt_yaw:
                yaw = x[off:off + n]; off += n
            if opt_a:
                a = x[off:off + n]; off += n
            return yaw, a

        def neg_power(x):
            yaw, a = split(x)
            return -self._farm_power(yaw, a)

        res = minimize(neg_power, np.array(x0), method="SLSQP", bounds=bounds,
                       options=dict(maxiter=100, ftol=1e-4))
        return split(res.x)


def _set(arr: np.ndarray, i: int, v: float) -> np.ndarray:
    out = arr.copy()
    out[i] = v
    return out
