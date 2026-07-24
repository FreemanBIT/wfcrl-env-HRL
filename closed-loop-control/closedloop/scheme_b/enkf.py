"""
B2 — Ensemble Kalman Filter observer.

Corrects the FLORIDyn ambient state (and, optionally, wake parameters via joint
state augmentation) each control step using measured per-turbine power (and
optionally nacelle wind speed). Implements the standard stochastic EnKF
(derivation doc eqs 2.11-2.20):

    forecast:  x_k^(e)- = f(x_{k-1}^(e)+, u) + w        (eq 2.12)
    analysis:  P^xy, P^yy from the ensemble               (eq 2.15-2.16)
               K = P^xy (P^yy)^-1                          (eq 2.17)
               x_k^(e)+ = x_k^(e)- + K (y + v^(e) - h(x)) (eq 2.18)

State (minimal): [U_inf, phi, TI]. With ``estimate_params=True`` the state is
augmented with wake parameters [ka, kb] as a random walk (eq 2.20), so the same
filter corrects model bias.

The forecast operator uses the FLORIDyn model as h(): given an ambient state
(and params), it predicts per-turbine power. Because a single measurement step
is quasi-static from the ambient-state viewpoint, the "forecast" here maps
ambient state -> predicted measurement through the wake model; the ensemble
spread captures uncertainty and yields the Kalman update.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .floridyn_model import FLORIDyn


@dataclass
class EnKFConfig:
    n_ensemble: int = 40                     # ensemble size N_e
    estimate_params: bool = True             # joint ambient + wake-param estimation
    # process noise std (random-walk drift) per state dim
    q_U: float = 0.05                        # m/s per step
    q_phi: float = 0.2                       # deg per step
    q_TI: float = 0.002
    q_ka: float = 0.002
    q_kb: float = 0.0002
    # measurement noise std
    r_power_frac: float = 0.03               # fraction of measured power
    r_power_floor: float = 20.0              # kW floor
    use_wind_meas: bool = False              # also assimilate nacelle wind speed
    r_wind: float = 0.3                      # m/s
    inflation: float = 1.02                  # covariance inflation factor
    # prior bounds (states are clipped to stay physical)
    U_bounds: tuple = (2.0, 20.0)
    phi_bounds: tuple = (180.0, 360.0)
    TI_bounds: tuple = (0.01, 0.30)
    ka_bounds: tuple = (0.1, 0.8)
    kb_bounds: tuple = (0.0, 0.05)


class EnKF:
    """Ensemble Kalman Filter over FLORIDyn ambient state (+ optional params).

    Parameters
    ----------
    model:
        The FLORIDyn model used as the forecast/observation operator. The filter
        writes its posterior back into this model each update.
    config:
        EnKF configuration.
    seed:
        RNG seed.
    """

    def __init__(self, model: FLORIDyn, config: Optional[EnKFConfig] = None, seed: int = 0):
        self.model = model
        self.cfg = config or EnKFConfig()
        self.rng = np.random.default_rng(seed)
        self.n_turb = model.n

        self._names = ["U", "phi", "TI"]
        if self.cfg.estimate_params:
            self._names += ["ka", "kb"]
        self.dim = len(self._names)
        self._ens: Optional[np.ndarray] = None  # (dim, N_e)

    # -- lifecycle -------------------------------------------------------------
    def initialize(self, U_inf: float, phi: float, TI: float) -> None:
        """Seed the ensemble around an initial ambient guess (+ current params)."""
        Ne = self.cfg.n_ensemble
        params = self.model.sur.get_params()
        mean = [U_inf, phi, TI]
        std = [0.5, 2.0, 0.01]
        if self.cfg.estimate_params:
            mean += [params["ka"], params["kb"]]
            std += [0.05, 0.005]
        mean = np.array(mean, float)
        std = np.array(std, float)
        self._ens = mean[:, None] + std[:, None] * self.rng.standard_normal((self.dim, Ne))
        self._clip_ensemble()

    def _q_vector(self) -> np.ndarray:
        q = [self.cfg.q_U, self.cfg.q_phi, self.cfg.q_TI]
        if self.cfg.estimate_params:
            q += [self.cfg.q_ka, self.cfg.q_kb]
        return np.array(q, float)

    # -- forecast / observation operator --------------------------------------
    def _predict_measurement(self, member: np.ndarray, yaw, a) -> np.ndarray:
        """h(x): map an ambient/param state to predicted measurement vector."""
        U, phi, TI = member[0], member[1], member[2]
        if self.cfg.estimate_params:
            self.model.sur.set_params({"ka": member[3], "kb": member[4]})
        self.model.set_conditions(U, phi, TI)
        self.model.set_controls(yaw, a)
        self.model._recompute_effective()
        meas = self.model.get_measurements()
        y = list(meas["P_i"])
        if self.cfg.use_wind_meas:
            y += list(meas["U_eff_i"])
        return np.array(y, float)

    def _measurement_vector(self, meas_power_kw, meas_wind=None) -> np.ndarray:
        y = list(meas_power_kw)
        if self.cfg.use_wind_meas and meas_wind is not None:
            y += list(meas_wind)
        return np.array(y, float)

    def _R_diag(self, y_meas: np.ndarray) -> np.ndarray:
        n = self.n_turb
        r = np.maximum(self.cfg.r_power_frac * np.abs(y_meas[:n]), self.cfg.r_power_floor) ** 2
        if self.cfg.use_wind_meas:
            r = np.concatenate([r, np.full(n, self.cfg.r_wind ** 2)])
        return r

    # -- update ----------------------------------------------------------------
    def update(
        self,
        meas_power_kw: np.ndarray,
        yaw: np.ndarray,
        a: np.ndarray,
        meas_wind: Optional[np.ndarray] = None,
        verbose: bool = False,
    ) -> dict:
        """One EnKF forecast+analysis step. Returns the posterior mean state and
        writes it into the model."""
        if self._ens is None:
            self.initialize(self.model._U_inf, self.model._phi, self.model._TI)

        Ne = self.cfg.n_ensemble
        # --- forecast: random-walk drift + inflation (eq 2.12) ---
        q = self._q_vector()
        self._ens = self._ens + q[:, None] * self.rng.standard_normal((self.dim, Ne))
        # inflation around mean
        xbar = self._ens.mean(axis=1, keepdims=True)
        self._ens = xbar + self.cfg.inflation * (self._ens - xbar)
        self._clip_ensemble()

        if verbose:
            xbar_post_fc = self._ens.mean(axis=1)
            std_post_fc = self._ens.std(axis=1)
            names = ["U", "φ", "TI"] + (["ka", "kb"] if self.cfg.estimate_params else [])
            s_mean = " ".join(f"{n}={xbar_post_fc[i]:.3f}" for i, n in enumerate(names))
            s_std  = " ".join(f"σ{n}={std_post_fc[i]:.3f}" for i, n in enumerate(names))
            print(f"  EnKF Forecast:  mean [{s_mean}]")
            print(f"                  std  [{s_std}]")

        # --- predicted measurements for each member (eq 2.13 mean of h) ---
        y_meas = self._measurement_vector(meas_power_kw, meas_wind)
        m = y_meas.size
        Y = np.zeros((m, Ne))
        for e in range(Ne):
            Y[:, e] = self._predict_measurement(self._ens[:, e], yaw, a)

        xbar = self._ens.mean(axis=1, keepdims=True)     # (dim,1)
        ybar = Y.mean(axis=1, keepdims=True)             # (m,1)
        Ex = self._ens - xbar                            # (dim,Ne)
        Ey = Y - ybar                                    # (m,Ne)

        # --- covariances (eq 2.15-2.16) ---
        Pxy = (Ex @ Ey.T) / (Ne - 1)                     # (dim,m)
        Pyy = (Ey @ Ey.T) / (Ne - 1)                     # (m,m)
        R = np.diag(self._R_diag(y_meas))
        S = Pyy + R

        # --- gain and update (eq 2.17-2.18) ---
        K = Pxy @ np.linalg.pinv(S)                      # (dim,m)

        if verbose:
            y_meas_vec = self._measurement_vector(meas_power_kw, meas_wind)
            innov_per_turb = y_meas_vec[:self.n_turb] - ybar.ravel()[:self.n_turb]
            s_innov = " ".join(f"{v:+.0f}" for v in innov_per_turb)
            print(f"  EnKF Predict:   P_farm ensemble range "
                  f"[{Y[:self.n_turb].sum(axis=0).min():.0f} .. "
                  f"{Y[:self.n_turb].sum(axis=0).mean():.0f} .. "
                  f"{Y[:self.n_turb].sum(axis=0).max():.0f}] kW")
            print(f"  EnKF Measure:   P_farm_meas={meas_power_kw.sum():.0f} kW")
            print(f"  EnKF Innov:     per-turb [{s_innov}] kW, |K|₂={np.linalg.norm(K):.3f}")

        # perturbed observations
        Rsqrt = np.sqrt(np.diag(R))
        D = y_meas[:, None] + Rsqrt[:, None] * self.rng.standard_normal((m, Ne))
        self._ens = self._ens + K @ (D - Y)
        self._clip_ensemble()

        if verbose:
            delta = self._ens.mean(axis=1) - xbar_post_fc
            names = ["U", "φ", "TI"] + (["ka", "kb"] if self.cfg.estimate_params else [])
            s_delta = " ".join(f"Δ{n}={delta[i]:+.4f}" for i, n in enumerate(names))
            post_mean = self._ens.mean(axis=1)
            s_post = " ".join(f"{n}={post_mean[i]:.3f}" for i, n in enumerate(names))
            print(f"  EnKF Analysis:  Δ  [{s_delta}]")
            print(f"                  post [{s_post}]")

        # posterior mean -> write into model
        post = self._ens.mean(axis=1)
        self._write_state(post)
        return dict(
            state={n_: float(v) for n_, v in zip(self._names, post)},
            innovation_norm=float(np.linalg.norm(y_meas - ybar.ravel())),
        )

    # -- helpers ---------------------------------------------------------------
    def _write_state(self, post: np.ndarray) -> None:
        self.model.set_conditions(post[0], post[1], post[2])
        if self.cfg.estimate_params:
            self.model.sur.set_params({"ka": post[3], "kb": post[4]})

    def _clip_ensemble(self) -> None:
        c = self.cfg
        self._ens[0] = np.clip(self._ens[0], *c.U_bounds)
        self._ens[1] = np.clip(self._ens[1], *c.phi_bounds)
        self._ens[2] = np.clip(self._ens[2], *c.TI_bounds)
        if self.cfg.estimate_params:
            self._ens[3] = np.clip(self._ens[3], *c.ka_bounds)
            self._ens[4] = np.clip(self._ens[4], *c.kb_bounds)

    @property
    def state_mean(self) -> dict:
        if self._ens is None:
            return {}
        post = self._ens.mean(axis=1)
        return {n_: float(v) for n_, v in zip(self._names, post)}

    @property
    def state_std(self) -> dict:
        if self._ens is None:
            return {}
        s = self._ens.std(axis=1)
        return {n_: float(v) for n_, v in zip(self._names, s)}
