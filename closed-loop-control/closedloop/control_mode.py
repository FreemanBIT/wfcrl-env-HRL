"""
M9 — Control-mode interface (5 modes), shared by schemes A / B / C.

The 5 modes align with the actuation channels the project actually supports:
the ZeroMQ ``setpoints(5)`` (torque=1, yaw=2, pitch=3..5) in
``ZeroMQInterface.f90`` and the ``yaw / pitch / torque / power`` command fields
of the file bridge ``DISCON_bridge.f90``.

    mode 1  YAW          optimize gamma;  induction fixed at greedy a=1/3   -> yaw=
    mode 2  TORQUE       optimize a (->torque); yaw fixed at 0             -> torque= (or power=)
    mode 3  PITCH        optimize a (->pitch);  yaw fixed at 0             -> pitch=
    mode 4  YAW_TORQUE   optimize gamma + a                                -> yaw= + torque=/power=
    mode 5  YAW_PITCH    optimize gamma + a                                -> yaw= + pitch=

Every controller (A2 / B3 / C5) queries ``ControlModeSpec`` to decide which
variables to optimize and which command fields to emit. Defining it once here
and reusing it in three places avoids divergence.
"""
from __future__ import annotations

from enum import IntEnum
from typing import Optional

from .types import Cmd
from . import induction as ind


# greedy (Betz-optimal) axial induction used when a turbine is not derating
A_GREEDY = 1.0 / 3.0
A_MIN = 0.0
A_MAX = 1.0 / 3.0
YAW_MAX = 30.0  # deg, absolute yaw magnitude limit


class ControlMode(IntEnum):
    YAW = 1
    TORQUE = 2
    PITCH = 3
    YAW_TORQUE = 4
    YAW_PITCH = 5


# which optimization variables are active in each mode
_DECISION_VARS = {
    ControlMode.YAW:         ["yaw"],
    ControlMode.TORQUE:      ["a"],
    ControlMode.PITCH:       ["a"],
    ControlMode.YAW_TORQUE:  ["yaw", "a"],
    ControlMode.YAW_PITCH:   ["yaw", "a"],
}

# how the induction/derating channel is delivered in each mode
#   "power"  -> power= (bridge closed-loop power tracking, preferred/most stable)
#   "torque" -> torque= (direct torque override)
#   "pitch"  -> pitch= (collective pitch override)
#   None     -> no derating channel (yaw-only)
_DERATE_CHANNEL = {
    ControlMode.YAW:         None,
    ControlMode.TORQUE:      "power",   # torque-side derating, delivered via power set-point
    ControlMode.PITCH:       "pitch",
    ControlMode.YAW_TORQUE:  "power",
    ControlMode.YAW_PITCH:   "pitch",
}


class ControlModeSpec:
    """Derives decision variables, fixed defaults, bounds, and command assembly
    from a :class:`ControlMode`.

    Parameters
    ----------
    mode:
        The selected control mode.
    use_power_for_torque:
        If True (default), torque-side derating (modes 2/4) is delivered via the
        bridge ``power=`` channel (closed-loop, more stable). If False, uses a
        direct ``torque=`` override (requires a torque model; see note in
        :meth:`to_controls`).
    """

    def __init__(self, mode: ControlMode, use_power_for_torque: bool = True):
        self.mode = ControlMode(mode)
        self.use_power_for_torque = use_power_for_torque

    # -- introspection used by optimizers --------------------------------------
    def decision_vars(self) -> list[str]:
        """Return the active optimization variables, e.g. ``['yaw']`` or ``['yaw','a']``."""
        return list(_DECISION_VARS[self.mode])

    def optimizes_yaw(self) -> bool:
        return "yaw" in _DECISION_VARS[self.mode]

    def optimizes_induction(self) -> bool:
        return "a" in _DECISION_VARS[self.mode]

    def fixed_defaults(self) -> dict:
        """Return default values for variables NOT optimized in this mode."""
        d: dict[str, float] = {}
        if not self.optimizes_yaw():
            d["yaw"] = 0.0
        if not self.optimizes_induction():
            d["a"] = A_GREEDY
        return d

    def bounds(self) -> dict:
        """Return (lo, hi) bounds for each active decision variable."""
        b: dict[str, tuple[float, float]] = {}
        if self.optimizes_yaw():
            b["yaw"] = (-YAW_MAX, YAW_MAX)
        if self.optimizes_induction():
            b["a"] = (A_MIN, A_MAX)
        return b

    # -- command assembly ------------------------------------------------------
    def to_controls(
        self,
        turbine_id: int,
        yaw_deg: float,
        a: float,
        U_eff: float,
        rho: float = 1.225,
    ) -> Cmd:
        """Assemble a :class:`Cmd` for one turbine given the (already-clamped)
        optimizer solution ``(yaw_deg, a)`` and its effective inflow ``U_eff``.

        The derating channel is chosen by the mode. Yaw is emitted whenever the
        mode steers.
        """
        cmd = Cmd(turbine_id=turbine_id)

        # yaw channel
        if self.optimizes_yaw():
            cmd.yaw_deg = float(yaw_deg)
        else:
            cmd.yaw_deg = 0.0  # explicitly hold aligned in non-steering modes

        # derating / induction channel
        channel = _DERATE_CHANNEL[self.mode]
        if channel == "power":
            # induction_to_power returns W; bridge expects MW
            p_mw = ind.induction_to_power(a, U_eff, yaw_deg=cmd.yaw_deg, rho=rho) / 1.0e6
            cmd.power_mw = max(0.0, p_mw)
        elif channel == "pitch":
            # convert axial induction -> equivalent collective pitch (deg)
            cmd.pitch_deg = ind.induction_to_pitch(a, U_eff)
        elif channel is None:
            # yaw-only: let the turbine run greedy internally (no derating field)
            pass

        return cmd


def greedy_cmd(turbine_id: int, U_eff: float, rho: float = 1.225) -> Cmd:
    """Convenience: the greedy baseline command for one turbine (yaw=0, a=1/3).

    Equivalent to ControlModeSpec(YAW).to_controls with zero yaw and greedy a,
    but emitted through the power channel so the turbine tracks its Betz-optimal
    power at the measured inflow.
    """
    p_mw = ind.induction_to_power(A_GREEDY, U_eff, yaw_deg=0.0, rho=rho) / 1.0e6
    return Cmd(turbine_id=turbine_id, yaw_deg=0.0, power_mw=max(0.0, p_mw))
