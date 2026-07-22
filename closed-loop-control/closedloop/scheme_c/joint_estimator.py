"""
C2 — Joint state / parameter / residual estimation.

Extends Scheme A's parameter calibration to also fit the data-driven residual
(derivation doc eq 3.3):

    {theta_hat, psi_hat} = argmin sum_m || y_m - h(x_m; theta, psi) ||_R^-1^2 + R(.)

Practically this is done in an alternating scheme on a batch of collected
(conditions, controls, measured power) records:
  1. Fit the physical wake parameters (ka, kb) to minimize the *pre-residual*
     power mismatch (reuse Scheme A's Calibrator).
  2. With the physical model fixed, fit the residual model to the remaining
     mismatch (measured - physical).
Iterating a couple of times gives a consistent grey-box fit.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..surrogate import SurrogateModel
from ..scheme_a.calibration import Calibrator, CalibrationConfig
from .residual_model import ResidualModel, build_features


@dataclass
class Record:
    """One data point for joint estimation."""
    U_inf: float
    phi: float
    TI: float
    yaw: np.ndarray
    induction: np.ndarray
    power_meas_kw: np.ndarray


@dataclass
class JointConfig:
    n_alternations: int = 2
    residual_ridge: float = 5.0
    calibrate_params: bool = True


class JointEstimator:
    """Alternating physical-parameter + residual estimation on a batch."""

    def __init__(
        self,
        surrogate: SurrogateModel,
        config: Optional[JointConfig] = None,
        cal_config: Optional[CalibrationConfig] = None,
    ):
        self.sur = surrogate
        self.cfg = config or JointConfig()
        self.calibrator = Calibrator(surrogate, cal_config)
        self.residual = ResidualModel(ridge=self.cfg.residual_ridge)

    def _phys_power(self, rec: Record) -> tuple[np.ndarray, np.ndarray]:
        self.sur.set_conditions(rec.U_inf, rec.phi, rec.TI)
        self.sur.set_controls(rec.yaw, rec.induction)
        pred = self.sur.predict()
        return np.asarray(pred["P_i"], float), np.asarray(pred["U_eff_i"], float)

    def fit(self, records: list[Record]) -> dict:
        """Run alternating estimation; returns fitted params + residual RMS."""
        rms_history = []
        for _ in range(self.cfg.n_alternations):
            # --- step 1: calibrate physical params on averaged record ---
            if self.cfg.calibrate_params and records:
                # use the mean conditions/power as a representative calibration
                self._calibrate_batch(records)

            # --- step 2: fit residual on the post-physical mismatch ---
            feats, resid = [], []
            for rec in records:
                p_phys, u_eff = self._phys_power(rec)
                X = build_features(p_phys, u_eff, rec.U_inf, rec.TI)
                r = rec.power_meas_kw - p_phys
                for i in range(len(r)):
                    feats.append(X[i])
                    resid.append(r[i])
            if feats:
                self.residual.fit(np.array(feats), np.array(resid))
                # residual RMS after fit
                pred = self.residual.predict(np.array(feats))
                rms = float(np.sqrt(np.mean((np.array(resid) - pred) ** 2)))
                rms_history.append(rms)

        return dict(
            params=self.sur.get_params(),
            residual_rms_kw=rms_history[-1] if rms_history else float("nan"),
            rms_history=rms_history,
        )

    def _calibrate_batch(self, records: list[Record]) -> None:
        """Calibrate physical params against the batch (uses the record whose
        conditions are closest to the batch mean as the calibration anchor)."""
        U_mean = np.mean([r.U_inf for r in records])
        phi_mean = np.mean([r.phi for r in records])
        # anchor = record nearest the mean condition
        anchor = min(records, key=lambda r: abs(r.U_inf - U_mean) + abs(r.phi - phi_mean))
        self.calibrator.run(
            p_meas_kw=anchor.power_meas_kw,
            U_inf=anchor.U_inf,
            phi=anchor.phi,
            TI=anchor.TI,
            yaw=anchor.yaw,
            induction=anchor.induction,
        )

    def get_residual(self) -> ResidualModel:
        return self.residual
