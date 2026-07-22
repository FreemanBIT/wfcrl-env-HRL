"""
M5 — Surrogate wake-model wrapper (FLORIS), shared by A / B / C.

Provides a uniform interface for predicting per-turbine effective wind speed and
power given (U_inf, phi, TI) and controls (yaw, induction).

  * Primary backend: the FLORIS library (``floris``), loaded from the project's
    ``case.yaml`` template.
  * Fallback backend: a lightweight analytical Gaussian-wake model
    (:class:`_AnalyticalGaussianWake`) so this package runs and is unit-testable
    even where FLORIS is not installed. It implements the same equations used in
    the derivation document (Bastankhah-Porte-Agel Gaussian deficit + sum-of-
    squares superposition + a simple deflection), with the tunable parameters
    ``ka, kb, alpha, beta`` that Scheme A / C calibrate.

Scheme A uses steady prediction directly; Scheme B wraps this as the local
steady solver inside FLORIDyn's Temporary Wind Farm; Scheme C adds an ML
residual on top of the prediction.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .induction import cp_from_induction, ct_from_induction, D_ROTOR, A_ROTOR, RHO_AIR, CP_YAW_EXP


@dataclass
class SurrogateParams:
    """Tunable wake parameters (subset that A/C calibrate). Defaults match the
    project ``case.yaml`` (GCH)."""
    ka: float = 0.38      # wake expansion vs TI slope
    kb: float = 0.004     # wake expansion offset
    alpha: float = 0.58   # near-wake length coeff
    beta: float = 0.077   # near-wake length coeff


class SurrogateModel:
    """Uniform surrogate interface.

    Parameters
    ----------
    layout_x, layout_y:
        Turbine coordinates (m). x is streamwise for westerly (270 deg).
    case_yaml:
        Optional path to a FLORIS case.yaml. If provided and FLORIS is available,
        the FLORIS backend is used; otherwise the analytical fallback is used.
    prefer_floris:
        If False, always use the analytical fallback (useful for fast tests).
    """

    def __init__(
        self,
        layout_x: list[float],
        layout_y: list[float],
        case_yaml: Optional[str] = None,
        prefer_floris: bool = True,
    ):
        self.layout_x = np.asarray(layout_x, dtype=float)
        self.layout_y = np.asarray(layout_y, dtype=float)
        self.n = len(layout_x)
        self.params = SurrogateParams()

        self._U_inf = 8.0
        self._phi = 270.0
        self._TI = 0.06
        self._yaw = np.zeros(self.n)
        self._induction = np.full(self.n, 1.0 / 3.0)

        self._floris = None
        if prefer_floris and case_yaml is not None:
            self._floris = _try_load_floris(case_yaml, self.layout_x, self.layout_y)
        self._fallback = _AnalyticalGaussianWake(self.layout_x, self.layout_y)

    @property
    def backend(self) -> str:
        return "floris" if self._floris is not None else "analytical"

    # -- configuration ---------------------------------------------------------
    def set_conditions(self, U_inf: float, phi: float, TI: float) -> None:
        self._U_inf, self._phi, self._TI = float(U_inf), float(phi), float(TI)

    def set_controls(self, yaw: np.ndarray, induction: np.ndarray) -> None:
        self._yaw = np.asarray(yaw, dtype=float).reshape(self.n)
        self._induction = np.asarray(induction, dtype=float).reshape(self.n)

    def set_params(self, theta: dict) -> None:
        """Write calibrated wake parameters (ka, kb, alpha, beta). Used by
        Scheme A calibration and Scheme C joint estimation."""
        for k, v in theta.items():
            if hasattr(self.params, k):
                setattr(self.params, k, float(v))
        if self._floris is not None:
            _apply_floris_params(self._floris, self.params)
        self._fallback.set_params(self.params)

    def get_params(self) -> dict:
        return dict(ka=self.params.ka, kb=self.params.kb,
                    alpha=self.params.alpha, beta=self.params.beta)

    # -- prediction ------------------------------------------------------------
    def predict(self) -> dict:
        """Return ``{'P_i': array(kW), 'U_eff_i': array(m/s), 'P_farm': kW}``."""
        if self._floris is not None:
            try:
                return self._predict_floris()
            except Exception:
                pass  # fall through to analytical on any FLORIS hiccup
        return self._predict_fallback()

    def _predict_floris(self) -> dict:
        fm = self._floris
        # yaw sign convention: FLORIS yaw is measured such that positive steers
        # the wake; keep the same sign the controller uses.
        fm.set(
            wind_speeds=[self._U_inf],
            wind_directions=[self._phi],
            turbulence_intensities=[self._TI],
            yaw_angles=np.array([[float(y) for y in self._yaw]]),
        )
        # power set-points via axial induction are approximated by de-rating in
        # the fallback; in FLORIS we run greedy Cp and scale by Cp(a)/Cp_greedy
        fm.run()
        p_greedy_w = np.asarray(fm.get_turbine_powers()).reshape(self.n)
        cp_ratio = np.array(
            [cp_from_induction(a) / (16.0 / 27.0) for a in self._induction]
        )
        p_w = p_greedy_w * cp_ratio
        u_eff = np.asarray(fm.turbine_average_velocities).reshape(self.n)
        return dict(
            P_i=p_w / 1e3,
            U_eff_i=u_eff,
            P_farm=float(np.sum(p_w)) / 1e3,
        )

    def _predict_fallback(self) -> dict:
        u_eff = self._fallback.effective_speeds(
            self._U_inf, self._phi, self._TI, self._yaw, self._induction
        )
        p_w = np.array([
            0.5 * RHO_AIR * A_ROTOR * cp_from_induction(self._induction[i])
            * (u_eff[i] ** 3)
            * (max(0.0, math.cos(math.radians(self._yaw[i]))) ** CP_YAW_EXP)
            for i in range(self.n)
        ])
        return dict(P_i=p_w / 1e3, U_eff_i=u_eff, P_farm=float(np.sum(p_w)) / 1e3)


# ============================================================================
# Analytical fallback: Bastankhah-Porte-Agel Gaussian wake + SOS superposition
# ============================================================================
class _AnalyticalGaussianWake:
    """Minimal, dependency-free Gaussian wake model matching the derivation doc.

    Deficit (eq 1.1):  dU/U = (1 - sqrt(1 - C_T cos g)) * exp(-(y-δ)^2/(2σy^2))
    Expansion (1.2/1.3): σ/D = (ka*TI + kb) * (x-x0)/D + σ0/D
    Deflection (1.4, simplified): δ ≈ tan(θ)·x with θ ∝ C_T·γ
    Superposition (1.5): U_eff,i = U_inf * (1 - sqrt(Σ (dU_j→i/U)^2))
    """

    def __init__(self, layout_x: np.ndarray, layout_y: np.ndarray):
        self.x = layout_x
        self.y = layout_y
        self.n = len(layout_x)
        self.D = D_ROTOR
        self.p = SurrogateParams()
        self.sigma0_over_D = 0.25  # initial wake width

    def set_params(self, params: SurrogateParams) -> None:
        self.p = params

    def _rotate_to_flow(self, phi_deg: float):
        """Rotate layout so +s is the streamwise (downwind) direction.

        Meteorological convention: ``phi`` is the direction the wind comes FROM,
        measured clockwise from North (0=N, 90=E, 180=S, 270=W). The wind blows
        TOWARD ``phi + 180``. With x = East and y = North, the unit vector of the
        flow (downwind) direction is::

            dirx = sin((phi+180) deg)   # east component
            diry = cos((phi+180) deg)   # north component

        For phi=270 (west), this gives (dirx, diry)=(+1, 0): wind blows toward
        +x, so higher-x (higher column index) turbines are downstream. Good.
        """
        toward = math.radians((phi_deg + 180.0) % 360.0)
        dirx, diry = math.sin(toward), math.cos(toward)
        # streamwise coordinate = projection on flow dir; cross = perpendicular
        s = self.x * dirx + self.y * diry
        c = -self.x * diry + self.y * dirx
        return s, c

    def effective_speeds(
        self,
        U_inf: float,
        phi: float,
        TI: float,
        yaw: np.ndarray,
        induction: np.ndarray,
    ) -> np.ndarray:
        s, c = self._rotate_to_flow(phi)
        ct = np.array([ct_from_induction(a) for a in induction])
        kstar = self.p.ka * TI + self.p.kb  # expansion rate

        u_eff = np.full(self.n, float(U_inf))
        for i in range(self.n):
            deficit_sq = 0.0
            for j in range(self.n):
                if j == i:
                    continue
                dx = s[i] - s[j]  # downstream distance from j to i
                if dx <= 1e-3:
                    continue  # i not downstream of j
                g = math.radians(float(yaw[j]))
                # wake width at i
                sigma = (kstar * dx + self.sigma0_over_D * self.D)
                # deflection of j's wake at i (simplified)
                theta0 = 0.3 * g / max(math.cos(g), 1e-3) * (
                    1.0 - math.sqrt(max(1e-6, 1.0 - ct[j] * math.cos(g)))
                )
                delta = theta0 * dx
                dy = c[i] - c[j] - delta
                # centreline deficit
                core = 1.0 - math.sqrt(max(1e-6, 1.0 - ct[j] * math.cos(g)))
                defc = core * math.exp(-(dy ** 2) / (2.0 * sigma ** 2))
                deficit_sq += defc ** 2
            u_eff[i] = U_inf * (1.0 - math.sqrt(deficit_sq))
        return np.maximum(u_eff, 0.1)


# ============================================================================
# Optional FLORIS loading helpers (best-effort; silent fallback if unavailable)
# ============================================================================
def _try_load_floris(case_yaml: str, layout_x, layout_y):
    try:
        from floris import FlorisModel  # FLORIS v4
    except Exception:
        try:
            from floris.tools import FlorisInterface as FlorisModel  # older
        except Exception:
            return None
    try:
        fm = FlorisModel(case_yaml)
        fm.set(layout_x=list(layout_x), layout_y=list(layout_y))
        return fm
    except Exception:
        return None


def _apply_floris_params(fm, params: SurrogateParams) -> None:
    """Best-effort write of wake params into a FLORIS model (velocity + deflection)."""
    try:
        core = fm.core
        for section in ["wake_velocity_parameters", "wake_deflection_parameters"]:
            d = getattr(core.wake, section, None)
            if isinstance(d, dict):
                target = d.get("gauss", d)  # some FLORIS versions nest under "gauss"
                for k in ("ka", "kb", "alpha", "beta"):
                    if hasattr(params, k):
                        target[k] = getattr(params, k)
    except Exception:
        pass
