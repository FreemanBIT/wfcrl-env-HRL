"""
M6 — Control base class + command arbiter (shared execution layer).

``BaseController`` defines the uniform interface every scheme implements. Each
controller holds a :class:`ControlMode` (M9) and decides internally which
channels to optimize/emit.

``CommandArbiter`` applies engineering limits (yaw magnitude/rate, hysteresis /
dead-band, induction/power range) before commands go to the bridge.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from .types import FlowEstimate, TurbineMeas, Cmd
from .control_mode import ControlMode, ControlModeSpec, YAW_MAX


class BaseController(ABC):
    """Abstract controller. Subclasses: GreedyController, ControllerA/B/C.

    Parameters
    ----------
    mode:
        Control mode (M9). Determines which variables are optimized and which
        command fields are emitted.
    n_turbines:
        Number of turbines.
    dt:
        Control step (s).
    """

    def __init__(self, mode: ControlMode, n_turbines: int, dt: float = 2.0):
        self.mode = ControlMode(mode)
        self.spec = ControlModeSpec(self.mode)
        self.n_turbines = n_turbines
        self.dt = dt

    @abstractmethod
    def step(
        self, flow_est: FlowEstimate, meas: dict[int, TurbineMeas]
    ) -> dict[int, Cmd]:
        """Compute the command for every turbine for this control step."""
        raise NotImplementedError

    def reset(self) -> None:
        """Optional per-episode reset hook."""
        pass


class CommandArbiter:
    """Apply engineering limits to raw controller commands.

    Parameters
    ----------
    yaw_max:
        Absolute yaw magnitude limit (deg).
    yaw_rate_max:
        Max yaw change per control step (deg). None disables per-step limiting
        (the bridge already rate-limits via its 0.2 proportional gain).
    yaw_deadband:
        Ignore yaw changes smaller than this (deg) to avoid chattering.
    power_min_mw, power_max_mw:
        Power set-point clamp (MW).
    """

    def __init__(
        self,
        yaw_max: float = YAW_MAX,
        yaw_rate_max: Optional[float] = None,
        yaw_deadband: float = 0.5,
        power_min_mw: float = 0.0,
        power_max_mw: float = 5.3,
    ):
        self.yaw_max = yaw_max
        self.yaw_rate_max = yaw_rate_max
        self.yaw_deadband = yaw_deadband
        self.power_min_mw = power_min_mw
        self.power_max_mw = power_max_mw
        self._prev_yaw: dict[int, float] = {}

    def apply_limits(self, cmds: dict[int, Cmd]) -> dict[int, Cmd]:
        for tid, cmd in cmds.items():
            if cmd.yaw_deg is not None:
                y = max(-self.yaw_max, min(self.yaw_max, cmd.yaw_deg))
                prev = self._prev_yaw.get(tid, y)
                # dead-band: hold previous if change too small
                if abs(y - prev) < self.yaw_deadband:
                    y = prev
                # per-step rate limit (optional)
                if self.yaw_rate_max is not None:
                    dy = max(-self.yaw_rate_max, min(self.yaw_rate_max, y - prev))
                    y = prev + dy
                cmd.yaw_deg = y
                self._prev_yaw[tid] = y
            if cmd.power_mw is not None:
                cmd.power_mw = max(self.power_min_mw, min(self.power_max_mw, cmd.power_mw))
        return cmds

    def reset(self) -> None:
        self._prev_yaw.clear()
