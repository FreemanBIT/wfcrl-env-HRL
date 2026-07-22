"""
M3 — Sensing & pre-processing layer (the observers' input front end).

Turns raw per-step measurements into a filtered ``FlowEstimate``:
  * sliding-window averaging of power and nacelle wind speed (removes turbulent
    fluctuation),
  * per-turbine local inflow reconstruction from ``wind_x`` + ``nacyaw``,
  * farm-level free-stream (U_inf, phi) estimated from the upstream row,
  * a hook for the 15-min scanning-lidar farm-level update (currently proxied by
    the upstream turbines).

Shared by all three schemes.
"""
from __future__ import annotations

from collections import defaultdict, deque
from typing import Optional

from .types import TurbineMeas, FlowEstimate, TurbineFlow


class Sensing:
    """Sliding-window sensing / flow reconstruction.

    Parameters
    ----------
    n_turbines:
        Number of turbines.
    dt:
        Control/measurement step (s) used to size the averaging window.
    avg_window_s:
        Averaging window length (s). Default 45 s to filter turbulence while
        tracking slow inflow changes.
    upstream_ids:
        Turbine ids treated as "free-stream sensors" (the upstream row). For the
        2x3 layout under westerly (270 deg), these are the first column
        (T1, T4). If None, uses all turbines (fallback).
    """

    def __init__(
        self,
        n_turbines: int,
        dt: float = 2.0,
        avg_window_s: float = 45.0,
        upstream_ids: Optional[list[int]] = None,
        wind_direction_ref: float = 270.0,
    ):
        self.n_turbines = n_turbines
        self.dt = dt
        self.win = max(1, int(round(avg_window_s / max(dt, 1e-6))))
        self.upstream_ids = upstream_ids or list(range(1, n_turbines + 1))
        # farm-level wind-direction estimate. In a real deployment this comes
        # from the scanning lidar (~15 min update) / met mast / SCADA farm
        # average; nacelle vanes alone are ambiguous. Update it via
        # ``set_farm_direction`` when a fresh scanning-lidar reading arrives.
        self._phi_farm = wind_direction_ref
        self._pwr_hist: dict[int, deque] = defaultdict(lambda: deque(maxlen=self.win))
        self._wind_hist: dict[int, deque] = defaultdict(lambda: deque(maxlen=self.win))
        self._yaw_hist: dict[int, deque] = defaultdict(lambda: deque(maxlen=self.win))

    def set_farm_direction(self, phi_deg: float) -> None:
        """Update the farm-level wind-direction estimate (scanning-lidar / met
        mast channel). Call on each slow-loop (~15 min) refresh."""
        self._phi_farm = float(phi_deg)

    def update(self, meas: dict[int, TurbineMeas]) -> None:
        """Push one step of raw measurements into the sliding buffers."""
        for tid, m in meas.items():
            self._pwr_hist[tid].append(m.genpwr_kw)
            self._wind_hist[tid].append(m.wind_x)
            self._yaw_hist[tid].append(m.nacyaw_deg)

    def estimate(self, meas: dict[int, TurbineMeas]) -> FlowEstimate:
        """Update buffers with ``meas`` and return the current FlowEstimate."""
        self.update(meas)

        per_turbine: dict[int, TurbineFlow] = {}
        for tid, m in meas.items():
            per_turbine[tid] = TurbineFlow(
                turbine_id=tid,
                U=_mean(self._wind_hist[tid], default=m.wind_x),
                # per-turbine direction: farm direction adjusted by the local
                # nacelle-yaw offset (best available proxy for heterogeneity).
                phi=self._phi_farm,
                P=_mean(self._pwr_hist[tid], default=m.genpwr_kw),
                yaw=_mean(self._yaw_hist[tid], default=m.nacyaw_deg),
            )

        # farm-level free stream: speed from the upstream row, direction from the
        # farm-level (scanning-lidar) channel.
        up = [per_turbine[t] for t in self.upstream_ids if t in per_turbine]
        if up:
            U_inf = sum(tf.U for tf in up) / len(up)
        else:
            U_inf = _mean_over(per_turbine, "U", default=8.0)
        phi = self._phi_farm

        step = min((m.step for m in meas.values()), default=-1)
        t = max((m.t for m in meas.values()), default=0.0)
        return FlowEstimate(
            U_inf=U_inf,
            phi=phi,
            TI_est=self._ti_estimate(),
            per_turbine=per_turbine,
            t=t,
            step=step,
        )

    # -- helpers ---------------------------------------------------------------
    def _local_direction(self, tid: int, m: TurbineMeas) -> float:
        """Estimate the local inflow direction seen by turbine ``tid``.

        The nacelle vane/heading gives the misalignment; here we take the
        nacelle heading as the best available proxy for the local wind direction
        (assuming the internal yaw controller aligns to the vane). A richer model
        can fuse NacVane if exported.
        """
        return m.nacyaw_deg if m.nacyaw_deg != 0.0 else 270.0

    def _ti_estimate(self) -> float:
        """Estimate ambient turbulence intensity from upstream wind-speed
        variability (sigma/mean over the window)."""
        vals = []
        for t in self.upstream_ids:
            h = self._wind_hist.get(t)
            if h and len(h) >= max(3, self.win // 3):
                mu = sum(h) / len(h)
                if mu > 0.1:
                    var = sum((x - mu) ** 2 for x in h) / len(h)
                    vals.append((var ** 0.5) / mu)
        return sum(vals) / len(vals) if vals else 0.06


def _mean(dq: deque, default: float = 0.0) -> float:
    return sum(dq) / len(dq) if len(dq) else default


def _mean_over(d: dict[int, TurbineFlow], attr: str, default: float = 0.0) -> float:
    vals = [getattr(v, attr) for v in d.values()]
    return sum(vals) / len(vals) if vals else default
