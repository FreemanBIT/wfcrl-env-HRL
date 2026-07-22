"""
M4 — Axial-induction <-> power/torque/pitch set-point conversion.

The optimizers work in axial-induction ``a`` (and yaw ``gamma``); the FAST.Farm
bridge accepts ``power=`` / ``torque=`` / ``pitch=``. This module converts
between them using actuator-disc theory, consistent with the derivation
document:

    C_P = 4 a (1 - a)^2                         (Betz / actuator disc)
    C_T = 4 a (1 - a)
    P_i = 1/2 * rho * A * C_P(a) * U^3 * cos^p(gamma)     with p = 1.88

The greedy (Betz) optimum is a = 1/3 -> C_P = 16/27.

Pitch mapping is a monotone engineering approximation: for a target C_P below
greedy, we find the collective pitch that reduces power by the same factor,
using a small calibrated power-vs-pitch sensitivity for the NREL 5MW rotor.
A project-specific lookup can replace :func:`induction_to_pitch` later.
"""
from __future__ import annotations

import math

# --- NREL 5MW geometry / constants -------------------------------------------
D_ROTOR = 126.0                      # rotor diameter (m)
A_ROTOR = math.pi * (D_ROTOR / 2.0) ** 2
RHO_AIR = 1.225                      # air density (kg/m^3)
CP_YAW_EXP = 1.88                    # cosine-loss exponent (matches FLORIS case.yaml)
A_GREEDY = 1.0 / 3.0
CP_MAX = 4.0 * A_GREEDY * (1.0 - A_GREEDY) ** 2   # = 16/27


def cp_from_induction(a: float) -> float:
    """Power coefficient from axial induction (actuator-disc)."""
    a = _clip(a, 0.0, A_GREEDY)     # a ∈ [0, 1/3] per development plan §M4
    return 4.0 * a * (1.0 - a) ** 2


def ct_from_induction(a: float) -> float:
    """Thrust coefficient from axial induction (actuator-disc)."""
    a = _clip(a, 0.0, A_GREEDY)
    return 4.0 * a * (1.0 - a)


def induction_to_power(
    a: float,
    U: float,
    yaw_deg: float = 0.0,
    rho: float = RHO_AIR,
    area: float = A_ROTOR,
) -> float:
    """Target aerodynamic power [W] for a given axial induction, inflow speed
    and yaw misalignment.

        P = 1/2 rho A C_P(a) U^3 cos^p(gamma)
    """
    cp = cp_from_induction(a)
    cos_loss = max(0.0, math.cos(math.radians(yaw_deg))) ** CP_YAW_EXP
    return 0.5 * rho * area * cp * (U ** 3) * cos_loss


def power_to_induction(
    power_w: float,
    U: float,
    yaw_deg: float = 0.0,
    rho: float = RHO_AIR,
    area: float = A_ROTOR,
) -> float:
    """Inverse of :func:`induction_to_power`: recover the *derating* axial
    induction (the root a<=1/3) that yields ``power_w`` at inflow ``U``.

    Solves 4 a (1-a)^2 = C_P_target on [0, 1/3] by bisection.
    """
    if U <= 0.1:
        return A_GREEDY
    cos_loss = max(1e-6, math.cos(math.radians(yaw_deg))) ** CP_YAW_EXP
    cp_target = power_w / (0.5 * rho * area * (U ** 3) * cos_loss)
    cp_target = _clip(cp_target, 0.0, CP_MAX)
    # bisection on the de-rating branch a in [0, 1/3] where C_P is monotone-increasing
    lo, hi = 0.0, A_GREEDY
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if cp_from_induction(mid) < cp_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def induction_to_pitch(a: float, U: float) -> float:
    """Approximate collective pitch [deg] that de-rates the rotor to axial
    induction ``a`` (relative to greedy).

    Engineering approximation: below-rated, power scales ~ with the pitch-driven
    C_P reduction. We map the required power ratio to a pitch offset using a
    small sensitivity ``dP/dpitch`` typical of the NREL 5MW below rated. Replace
    with a calibrated lookup table if higher fidelity is needed.
    """
    a = _clip(a, 0.0, A_GREEDY)
    ratio = cp_from_induction(a) / CP_MAX          # in (0, 1]
    if ratio >= 0.999:
        return 0.0
    # empirical: ~1.0 deg pitch reduces C_P by ~4% near fine pitch for NREL 5MW
    # invert: pitch = (1 - ratio) / sensitivity, clamped to a safe range
    sensitivity_per_deg = 0.04
    pitch = (1.0 - ratio) / sensitivity_per_deg
    return _clip(pitch, 0.0, 20.0)


def induction_to_torque(a: float, U: float, gen_speed_rads: float) -> float:
    """Approximate generator torque [Nm] for a target axial induction, given the
    current generator speed.

        T = P_target / omega
    (feed-forward; the bridge additionally closes the loop on power if power= is
    used instead.)
    """
    p = induction_to_power(a, U)
    return p / max(gen_speed_rads, 0.1)


# --- helpers -----------------------------------------------------------------
def _clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
