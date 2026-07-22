"""
A1 — Periodic FLORIS parameter calibration (Scheme A observer).

Every ``T_cal`` (~10-15 min sim time), uses the window's measured per-turbine
mean power to re-fit the FLORIS wake parameters so the model matches the current
conditions. Implements the weighted least-squares with regularization (eq 1.8)
and SVD-truncated update to handle identifiability (eq 1.10).

    theta = [ka, kb, alpha, beta, {phi_i_bias}, U_inf]        (eq 1.7)
    min_theta  sum_i w_i (Pbar_i^meas - P_i^FLORIS(theta))^2  + lambda||theta-theta0||^2
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..surrogate import SurrogateModel


@dataclass
class CalibrationConfig:
    param_names: tuple[str, ...] = ("ka", "kb")   # subset actually calibrated
    reg_lambda: float = 1e-3                       # regularization to prior
    svd_rcond: float = 1e-3                        # SVD truncation threshold (eq 1.10)
    max_iter: int = 30
    step_damping: float = 0.5                      # Gauss-Newton damping
    fd_eps: float = 1e-3                           # finite-difference step for Jacobian
    bounds: Optional[dict] = None                  # {name: (lo, hi)}


class Calibrator:
    """Calibrates a subset of FLORIS wake parameters against measured power.

    Parameters
    ----------
    surrogate:
        The shared SurrogateModel (M5) whose parameters are updated in place.
    config:
        Calibration configuration.
    """

    def __init__(self, surrogate: SurrogateModel, config: Optional[CalibrationConfig] = None):
        self.sur = surrogate
        self.cfg = config or CalibrationConfig()
        self._default_bounds = {
            "ka": (0.1, 0.8), "kb": (0.0, 0.05),
            "alpha": (0.3, 0.9), "beta": (0.03, 0.2),
        }

    def _get_theta(self) -> np.ndarray:
        p = self.sur.get_params()
        return np.array([p[n] for n in self.cfg.param_names], float)

    def _set_theta(self, theta: np.ndarray) -> None:
        self.sur.set_params(
            {n: float(v) for n, v in zip(self.cfg.param_names, theta)}
        )

    def _clip_theta(self, theta: np.ndarray) -> np.ndarray:
        bnds = self.cfg.bounds or self._default_bounds
        out = theta.copy()
        for i, n in enumerate(self.cfg.param_names):
            lo, hi = bnds.get(n, (-np.inf, np.inf))
            out[i] = min(hi, max(lo, out[i]))
        return out

    def _predict_power(self, theta: np.ndarray, U_inf, phi, TI, yaw, induction) -> np.ndarray:
        self._set_theta(theta)
        self.sur.set_conditions(U_inf, phi, TI)
        self.sur.set_controls(yaw, induction)
        return np.asarray(self.sur.predict()["P_i"], float)  # kW

    def run(
        self,
        p_meas_kw: np.ndarray,
        U_inf: float,
        phi: float,
        TI: float,
        yaw: np.ndarray,
        induction: np.ndarray,
        weights: Optional[np.ndarray] = None,
    ) -> dict:
        """Run one calibration and write the result back into the surrogate.

        Parameters
        ----------
        p_meas_kw:
            Window-averaged measured per-turbine power (kW).
        U_inf, phi, TI, yaw, induction:
            Conditions/controls that were active during the window.
        weights:
            Optional per-turbine weights ``w_i`` (default: normalized by power).

        Returns
        -------
        dict with the fitted parameters and the pre/post RMS error.
        """
        p_meas = np.asarray(p_meas_kw, float)
        n = len(p_meas)
        w = weights if weights is not None else _default_weights(p_meas)
        w = np.asarray(w, float)

        theta0 = self._get_theta()
        theta = theta0.copy()

        def residual(th):
            p_pred = self._predict_power(th, U_inf, phi, TI, yaw, induction)
            return np.sqrt(w) * (p_meas - p_pred)

        r0 = residual(theta0)
        rms0 = float(np.sqrt(np.mean(r0 ** 2)))

        for _ in range(self.cfg.max_iter):
            r = residual(theta)
            J = self._jacobian(theta, U_inf, phi, TI, yaw, induction, w)
            # regularized normal equations solved via SVD truncation (eq 1.10)
            # augment with regularization toward prior theta0
            JtJ = J.T @ J + self.cfg.reg_lambda * np.eye(len(theta))
            Jtr = J.T @ r - self.cfg.reg_lambda * (theta - theta0)
            dtheta = _svd_solve(JtJ, Jtr, self.cfg.svd_rcond)
            theta = self._clip_theta(theta + self.cfg.step_damping * dtheta)
            if np.linalg.norm(dtheta) < 1e-6:
                break

        rms1 = float(np.sqrt(np.mean(residual(theta) ** 2)))
        self._set_theta(theta)  # persist best estimate into surrogate

        return dict(
            params={n: float(v) for n, v in zip(self.cfg.param_names, theta)},
            rms_before_kw=rms0,
            rms_after_kw=rms1,
            improved=rms1 <= rms0,
        )

    def _jacobian(self, theta, U_inf, phi, TI, yaw, induction, w) -> np.ndarray:
        """Finite-difference Jacobian d(residual)/d(theta)."""
        eps = self.cfg.fd_eps
        base = np.sqrt(w) * (
            -self._predict_power(theta, U_inf, phi, TI, yaw, induction)
        )
        n_res = len(base)
        J = np.zeros((n_res, len(theta)))
        for j in range(len(theta)):
            tp = theta.copy()
            tp[j] += eps
            pj = np.sqrt(w) * (
                -self._predict_power(tp, U_inf, phi, TI, yaw, induction)
            )
            J[:, j] = (pj - base) / eps
        return J


def _default_weights(p_meas: np.ndarray) -> np.ndarray:
    """Weight turbines by inverse power scale so waked (low-power) turbines are
    not drowned out; falls back to uniform."""
    scale = np.maximum(p_meas, 1.0)
    w = 1.0 / scale
    return w / w.mean()


def _svd_solve(A: np.ndarray, b: np.ndarray, rcond: float) -> np.ndarray:
    """Solve A x = b keeping only singular values > rcond*max (eq 1.10)."""
    U, s, Vt = np.linalg.svd(A, full_matrices=False)
    smax = s.max() if s.size else 0.0
    keep = s > (rcond * smax) if smax > 0 else np.zeros_like(s, bool)
    s_inv = np.where(keep, 1.0 / np.where(keep, s, 1.0), 0.0)
    return (Vt.T * s_inv) @ (U.T @ b)
