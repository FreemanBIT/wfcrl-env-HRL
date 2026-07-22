"""
WFCRL interface adapter.

Lets the closed-loop controllers drive FAST.Farm through the project's own
``wfcrl.interface.FastFarmInterface`` (which handles the MPI launch, .fstf
generation and the ZeroMQ measurement/command exchange), instead of the raw
file bridge.

Use this if you launch FAST.Farm the WFCRL way::

    from wfcrl.interface import FastFarmInterface
    from wfcrl.environments import data_cases as cases
    iface = FastFarmInterface(cases.fastfarm_6t)          # 6-turbine case
    plant = WFCRLPlant(iface, n_turbines=6)

and then hand ``plant`` to :class:`closedloop.runner.ClosedLoopRunner`.

IMPORTANT — API contract
-------------------------
The WFCRL FastFarmInterface exposes (per its docs/notebooks) a per-step exchange
that returns 12 measures per turbine (2 wind, power, yaw, pitch, torque, 6 blade
loads) and accepts yaw/pitch/torque commands. The exact method names differ
slightly between WFCRL versions, so this adapter probes for the common ones and
raises a clear error if it cannot bind. If your installed WFCRL uses different
names, set the four callables in :meth:`__init__` (see ``_bind`` below) — it is
a 4-line change, documented inline.

Because WFCRL's native path is ZeroMQ (not the custom file bridge), prefer
:class:`closedloop.runner.FastFarmPlant` when you are using the project's
``DISCON_bridge.f90`` file protocol, and this adapter when you are using the
stock WFCRL ZeroMQ interface.
"""
from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from .runner import Plant
from .types import TurbineMeas, Cmd


class WFCRLPlant(Plant):
    """Adapter: drive FAST.Farm via a WFCRL ``FastFarmInterface`` instance.

    Parameters
    ----------
    interface:
        A constructed ``wfcrl.interface.FastFarmInterface`` (or compatible).
    n_turbines:
        Number of turbines.
    measure_map:
        Optional custom function ``(interface) -> dict[int, TurbineMeas]``. If
        given, it overrides the built-in measurement reader (use this if your
        WFCRL version exposes a different measurement API).
    command_map:
        Optional custom function ``(interface, step, cmds) -> None`` that applies
        commands. If given, overrides the built-in writer.
    """

    def __init__(
        self,
        interface,
        n_turbines: int,
        measure_map: Optional[Callable] = None,
        command_map: Optional[Callable] = None,
    ):
        self.iface = interface
        self.n = n_turbines
        self._measure_map = measure_map
        self._command_map = command_map
        self._bind()

    # -- API binding -----------------------------------------------------------
    def _bind(self) -> None:
        """Probe the interface for the measurement/command methods.

        If your WFCRL version differs, replace the two lookups below with the
        correct method references, e.g.::

            self._get = self.iface.get_measurements
            self._send = self.iface.update_command
        """
        if self._measure_map is None:
            self._get = _first_method(
                self.iface,
                ["get_measurements", "get_measures", "measure", "get_turbine_measures", "read"],
            )
        else:
            self._get = None
        if self._command_map is None:
            self._send = _first_method(
                self.iface,
                ["update_command", "set_command", "apply_command", "step", "send_command"],
            )
        else:
            self._send = None

        if self._measure_map is None and self._get is None:
            raise RuntimeError(
                "WFCRLPlant could not find a measurement method on the interface. "
                "Pass measure_map=... or edit _bind() with the correct method name."
            )
        if self._command_map is None and self._send is None:
            raise RuntimeError(
                "WFCRLPlant could not find a command method on the interface. "
                "Pass command_map=... or edit _bind() with the correct method name."
            )

    # -- Plant API -------------------------------------------------------------
    def read(self, step: int) -> dict[int, TurbineMeas]:
        if self._measure_map is not None:
            return self._measure_map(self.iface)
        raw = self._get()
        return _coerce_measurements(raw, self.n, step)

    def write(self, step: int, cmds: dict[int, Cmd]) -> None:
        if self._command_map is not None:
            self._command_map(self.iface, step, cmds)
            return
        # Build per-turbine yaw/pitch/torque arrays for the WFCRL command API.
        yaw = np.zeros(self.n)
        pitch = np.full(self.n, np.nan)
        torque = np.full(self.n, np.nan)
        for tid, c in cmds.items():
            i = tid - 1
            if c.yaw_deg is not None:
                yaw[i] = c.yaw_deg
            if c.pitch_deg is not None:
                pitch[i] = c.pitch_deg
            if c.torque_nm is not None:
                torque[i] = c.torque_nm
        # WFCRL command APIs vary; the most common accepts a dict of arrays.
        try:
            self._send({"yaw": yaw, "pitch": pitch, "torque": torque})
        except TypeError:
            # some versions take positional yaw only
            self._send(yaw)

    def current_direction(self) -> Optional[float]:
        # WFCRL exposes farm-inlet wind direction in the measurements; if you
        # want to feed it to sensing, return it here. Default None keeps the
        # sensing reference.
        return None

    def reset(self) -> None:
        for name in ["reset", "restart"]:
            fn = getattr(self.iface, name, None)
            if callable(fn):
                try:
                    fn()
                except Exception:
                    pass
                return


# ---------------------------------------------------------------------------
def _first_method(obj, names):
    for n in names:
        fn = getattr(obj, n, None)
        if callable(fn):
            return fn
    return None


def _coerce_measurements(raw, n: int, step: int) -> dict[int, TurbineMeas]:
    """Best-effort conversion of a WFCRL measurement payload into TurbineMeas.

    Accepts a few shapes:
      * dict with keys like 'power'/'yaw'/'wind_speed'/'wind_dir'/... -> arrays
      * a 2D array (n_turbines, n_channels) in the documented WFCRL order:
        [wind_speed, wind_dir, power, yaw, pitch, torque, load1..load6]
    """
    out: dict[int, TurbineMeas] = {}

    if isinstance(raw, dict):
        def col(keys, default=0.0):
            for k in keys:
                if k in raw:
                    return np.asarray(raw[k], float).reshape(-1)
            return np.full(n, default)
        power = col(["power", "genpwr", "P"])
        yaw = col(["yaw", "nacyaw", "gamma"])
        wind = col(["wind_speed", "wind_velocity", "wind", "U"])
        pitch = col(["pitch", "blpitch"])
        torque = col(["torque", "gentq"])
        load = col(["load", "moop", "blade_load"], default=0.0)
        for i in range(n):
            out[i + 1] = TurbineMeas(
                turbine_id=i + 1, step=step, t=step,
                genpwr_kw=_get(power, i) / (1e3 if _get(power, i) > 1e5 else 1.0),
                wind_x=_get(wind, i), nacyaw_deg=_get(yaw, i),
                blpitch_deg=_get(pitch, i), gentq_nm=_get(torque, i),
                moop1_knm=_get(load, i),
            )
        return out

    arr = np.asarray(raw, float)
    if arr.ndim == 2 and arr.shape[0] == n:
        for i in range(n):
            row = arr[i]
            out[i + 1] = TurbineMeas(
                turbine_id=i + 1, step=step, t=step,
                wind_x=row[0] if row.size > 0 else 0.0,
                genpwr_kw=(row[2] if row.size > 2 else 0.0),
                nacyaw_deg=(row[3] if row.size > 3 else 0.0),
                blpitch_deg=(row[4] if row.size > 4 else 0.0),
                gentq_nm=(row[5] if row.size > 5 else 0.0),
                moop1_knm=(row[7] if row.size > 7 else 0.0),
            )
        return out

    raise ValueError(
        "Unrecognized WFCRL measurement payload shape; pass a custom "
        "measure_map to WFCRLPlant."
    )


def _get(a: np.ndarray, i: int, default: float = 0.0) -> float:
    return float(a[i]) if i < a.size else default
