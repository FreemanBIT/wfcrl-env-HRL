"""
C4 — RL policy + safety projection (optional, aggressive route).

A pre-trained policy maps state -> control for online real-time use (derivation
doc eq 3.7), with a mandatory safety projection onto the set of controls the
physical model certifies as no worse than a fraction of the baseline (eq 3.8):

    u = Pi_{U_safe}( pi_phi(s) ),
    U_safe = { u : P_farm^phys(u) >= (1-eps) P_farm^base }.

This module provides:
  * :class:`SafetyProjector` — the physics guardrail (usable standalone with any
    policy or even with Scheme B's MPC output).
  * :class:`LinearPolicy` — a minimal, dependency-free policy placeholder so the
    pipeline runs; replace ``act`` with a trained network (e.g. from the WFCRL
    RL environment) in production.

The safety projector is the important, non-optional part: it guarantees the
learned controller never underperforms the physical baseline by more than eps.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from ..surrogate import SurrogateModel
from ..control_mode import ControlModeSpec, A_GREEDY, YAW_MAX


class SafetyProjector:
    """Project a proposed control onto the physics-certified safe set.

    Parameters
    ----------
    surrogate:
        Physical surrogate used to certify safety.
    epsilon:
        Allowed shortfall vs baseline (e.g. 0.02 = accept if >= 98% of baseline).
    """

    def __init__(self, surrogate: SurrogateModel, epsilon: float = 0.02):
        self.sur = surrogate
        self.eps = epsilon

    def _farm_power(self, yaw, a, U_inf, phi, TI) -> float:
        self.sur.set_conditions(U_inf, phi, TI)
        self.sur.set_controls(yaw, a)
        return float(self.sur.predict()["P_farm"])

    def project(
        self,
        yaw_prop: np.ndarray,
        a_prop: np.ndarray,
        U_inf: float,
        phi: float,
        TI: float,
        yaw_fallback: Optional[np.ndarray] = None,
        a_fallback: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, np.ndarray, bool]:
        """Return ``(yaw, a, accepted)``. If the proposal fails the safety test,
        fall back to the physical baseline (greedy or provided fallback)."""
        n = len(yaw_prop)
        yaw_base = yaw_fallback if yaw_fallback is not None else np.zeros(n)
        a_base = a_fallback if a_fallback is not None else np.full(n, A_GREEDY)

        p_base = self._farm_power(yaw_base, a_base, U_inf, phi, TI)
        p_prop = self._farm_power(yaw_prop, a_prop, U_inf, phi, TI)

        if p_prop >= (1.0 - self.eps) * p_base:
            return yaw_prop, a_prop, True
        return yaw_base, a_base, False


@dataclass
class LinearPolicy:
    """Minimal placeholder policy: yaw proportional to a learned gain on the
    (normalized) waked-ness of upstream turbines. Stands in for a trained
    network so the pipeline is runnable; swap ``act`` for a real policy.
    """
    n_turbines: int
    gain_deg: float = 25.0

    def act(self, flow_est, spec: ControlModeSpec) -> tuple[np.ndarray, np.ndarray]:
        n = self.n_turbines
        yaw = np.zeros(n)
        a = np.full(n, A_GREEDY)
        if spec.optimizes_yaw():
            # steer upstream turbines (those seeing near-free-stream) toward +gain
            for tid, tf in flow_est.per_turbine.items():
                i = tid - 1
                waked = tf.U < 0.95 * flow_est.U_inf
                yaw[i] = 0.0 if waked else self.gain_deg
            yaw = np.clip(yaw, -YAW_MAX, YAW_MAX)
        return yaw, a
