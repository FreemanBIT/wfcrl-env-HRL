"""
Wind schedules for demos / experiments.

Each schedule is a callable ``t -> (U_inf, phi, TI)`` used by the mock plant and
(conceptually) mirrored by the FAST.Farm inflow. The names match the demo CLI
(``--wind steady_8ms | ramp | step``).
"""
from __future__ import annotations

from typing import Callable


def steady(U: float = 8.0, phi: float = 270.0, TI: float = 0.06):
    """Constant conditions."""
    def fn(t: float):
        return U, phi, TI
    return fn


def direction_step(
    U: float = 8.0,
    phi0: float = 270.0,
    phi1: float = 260.0,
    t_switch: float = 300.0,
    TI: float = 0.06,
):
    """Wind direction steps from phi0 to phi1 at ``t_switch`` — the transient
    case where dynamic schemes (B/C) shine."""
    def fn(t: float):
        return U, (phi0 if t < t_switch else phi1), TI
    return fn


def direction_ramp(
    U: float = 8.0,
    phi0: float = 270.0,
    phi1: float = 260.0,
    t_start: float = 200.0,
    t_end: float = 400.0,
    TI: float = 0.06,
):
    """Wind direction ramps linearly from phi0 to phi1 over [t_start, t_end]."""
    def fn(t: float):
        if t <= t_start:
            phi = phi0
        elif t >= t_end:
            phi = phi1
        else:
            frac = (t - t_start) / (t_end - t_start)
            phi = phi0 + frac * (phi1 - phi0)
        return U, phi, TI
    return fn


def speed_ramp(
    U0: float = 6.0,
    U1: float = 10.0,
    phi: float = 270.0,
    t_start: float = 200.0,
    t_end: float = 400.0,
    TI: float = 0.06,
):
    """Wind speed ramps from U0 to U1 over [t_start, t_end]."""
    def fn(t: float):
        if t <= t_start:
            U = U0
        elif t >= t_end:
            U = U1
        else:
            U = U0 + (t - t_start) / (t_end - t_start) * (U1 - U0)
        return U, phi, TI
    return fn


def from_name(name: str) -> Callable[[float], tuple[float, float, float]]:
    """Resolve a schedule from the demo CLI name."""
    name = name.lower()
    if name in ("steady", "steady_8ms"):
        return steady()
    if name in ("step", "dir_step"):
        return direction_step()
    if name in ("ramp", "dir_ramp"):
        return direction_ramp()
    if name in ("speed_ramp", "uramp"):
        return speed_ramp()
    raise ValueError(f"unknown wind schedule: {name}")
