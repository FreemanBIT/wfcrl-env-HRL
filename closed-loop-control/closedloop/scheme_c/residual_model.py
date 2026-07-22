"""
C1 / C3 — Grey-box residual model.

Learns the systematic residual between the physical surrogate's prediction and
the measured per-turbine power/velocity, and adds it back (derivation doc eqs
3.1-3.2, 3.4-3.5):

    U_eff^corr = U_eff^phys(theta) + g_psi(xi)         (additive residual, eq 3.1)

Two residual back-ends are provided:
  * :class:`ResidualModel` (default) — a lightweight ridge-regression on simple
    features (per-turbine physical prediction, waked/upstream flag, local
    conditions). Dependency-free, robust, and interpretable.
  * A Gaussian-process option (:class:`GPResidual`) when ``scikit-learn`` is
    available, which additionally yields a predictive variance for the safety
    layer (eq 3.5).

:class:`GreyBoxSurrogate` wraps a physical :class:`SurrogateModel` so it exposes
the same ``predict()`` interface with the residual applied — it can be dropped
into Scheme B's MPC unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..surrogate import SurrogateModel


def _features(P_phys_kw: np.ndarray, U_eff: np.ndarray, U_inf: float, TI: float) -> np.ndarray:
    """Feature matrix (n_turb, n_feat) for the residual model."""
    n = len(P_phys_kw)
    waked = (U_eff < 0.95 * U_inf).astype(float)   # is this turbine waked?
    return np.column_stack([
        np.ones(n),                    # bias
        P_phys_kw / 1000.0,            # physical power (scaled)
        U_eff / 10.0,                  # local speed (scaled)
        waked,                         # waked flag
        np.full(n, TI),                # turbulence
    ])


class ResidualModel:
    """Ridge-regression residual on power (kW).

    Fit on collected (features -> measured_power - physical_power) pairs; predict
    the correction to add to the physical power.
    """

    def __init__(self, ridge: float = 1.0):
        self.ridge = ridge
        self.w: Optional[np.ndarray] = None
        self.n_feat = 5

    def fit(self, X: np.ndarray, residual_kw: np.ndarray) -> "ResidualModel":
        X = np.asarray(X, float)
        y = np.asarray(residual_kw, float)
        A = X.T @ X + self.ridge * np.eye(X.shape[1])
        b = X.T @ y
        self.w = np.linalg.solve(A, b)
        return self

    def predict(self, X: np.ndarray) -> np.ndarray:
        if self.w is None:
            return np.zeros(len(X))
        return np.asarray(X, float) @ self.w

    # convenience: accumulate data and refit
    def fit_from_records(self, records: list[tuple[np.ndarray, np.ndarray]]) -> "ResidualModel":
        """records: list of (feature_row, residual_scalar)."""
        X = np.array([r[0] for r in records])
        y = np.array([r[1] for r in records])
        return self.fit(X, y)


class GPResidual:
    """Gaussian-process residual (optional; needs scikit-learn). Provides mean
    and variance (eq 3.4-3.5)."""

    def __init__(self, length_scale: float = 1.0, noise: float = 1e-2):
        self.length_scale = length_scale
        self.noise = noise
        self._gp = None

    def available(self) -> bool:
        try:
            import sklearn  # noqa
            return True
        except Exception:
            return False

    def fit(self, X, residual_kw):
        from sklearn.gaussian_process import GaussianProcessRegressor
        from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel
        kern = ConstantKernel() * RBF(length_scale=self.length_scale) + WhiteKernel(self.noise)
        self._gp = GaussianProcessRegressor(kernel=kern, normalize_y=True)
        self._gp.fit(np.asarray(X, float), np.asarray(residual_kw, float))
        return self

    def predict(self, X, return_std: bool = False):
        if self._gp is None:
            z = np.zeros(len(X))
            return (z, z) if return_std else z
        return self._gp.predict(np.asarray(X, float), return_std=return_std)


class GreyBoxSurrogate:
    """Physical surrogate + additive power residual, exposing the SurrogateModel
    ``predict`` interface so it plugs into Scheme B's MPC/FLORIDyn unchanged.

    Parameters
    ----------
    physical:
        The underlying physical :class:`SurrogateModel`.
    residual:
        A fitted residual model (ResidualModel or GPResidual). If None, behaves
        exactly like the physical surrogate.
    """

    def __init__(self, physical: SurrogateModel, residual: Optional[object] = None):
        self.phys = physical
        self.residual = residual
        # expose the same public attributes the MPC/FLORIDyn read
        self.n = physical.n
        self.layout_x = physical.layout_x
        self.layout_y = physical.layout_y

    # delegate configuration to the physical model
    def set_conditions(self, U_inf, phi, TI):
        self._U_inf, self._phi, self._TI = U_inf, phi, TI
        self.phys.set_conditions(U_inf, phi, TI)

    def set_controls(self, yaw, induction):
        self.phys.set_controls(yaw, induction)

    def set_params(self, theta):
        self.phys.set_params(theta)

    def get_params(self):
        return self.phys.get_params()

    def predict(self) -> dict:
        base = self.phys.predict()
        if self.residual is None:
            return base
        X = _features(np.asarray(base["P_i"]), np.asarray(base["U_eff_i"]),
                      self._U_inf, self._TI)
        corr = self.residual.predict(X)
        p_corr = np.maximum(0.0, np.asarray(base["P_i"]) + np.asarray(corr))
        return dict(P_i=p_corr, U_eff_i=base["U_eff_i"], P_farm=float(np.sum(p_corr)))


def build_features(P_phys_kw, U_eff, U_inf, TI):
    """Public helper to build residual features (used by the joint estimator)."""
    return _features(np.asarray(P_phys_kw, float), np.asarray(U_eff, float), U_inf, TI)
