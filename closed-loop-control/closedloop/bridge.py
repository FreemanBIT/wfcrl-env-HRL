"""
M2 — FAST.Farm file-bridge interface (the core integration layer).

Wraps the file protocol implemented by ``DISCON_bridge.f90``:

    write controls.txt              controller -> FAST.Farm
        step=<N>
        T1 yaw=<deg> pitch=<deg> torque=<Nm> power=<MW>
        ...
        END

    read measurements_T<id>.txt     FAST.Farm -> controller
        step=<N> t=<s> genpwr=<kW> genspd=<rpm> gentq=<Nm> rotspd=<rpm>
        wind_x=<m/s>
        blpitch=<deg> nacyaw=<deg>
        mip1=<kNm> moop1=<kNm> mzb1=<kNm>

Key correctness requirements (see plan §M2):
  * controls.txt must be written atomically (temp file + os.replace) so the
    Fortran bridge never reads a half-written line.
  * ``step`` must be strictly increasing; the bridge only applies a new file
    when read_step > applied_step.
  * measurement parsing must tolerate the instant the bridge is overwriting a
    file (retry on short/partial reads).
"""
from __future__ import annotations

import os
import re
import time
from pathlib import Path
from typing import Iterable, Optional

from .types import TurbineMeas, Cmd

# tokens like "genpwr=123.45" possibly run together without spaces; parse robustly
_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
_FIELD_RE = {
    "step": re.compile(r"step=(" + r"[-+]?\d+" + r")"),
    "t": re.compile(r"\bt=(" + _NUM + r")"),
    "genpwr": re.compile(r"genpwr=(" + _NUM + r")"),
    "genspd": re.compile(r"genspd=(" + _NUM + r")"),
    "gentq": re.compile(r"gentq=(" + _NUM + r")"),
    "rotspd": re.compile(r"rotspd=(" + _NUM + r")"),
    "wind_x": re.compile(r"wind_x=(" + _NUM + r")"),
    "blpitch": re.compile(r"blpitch=(" + _NUM + r")"),
    "nacyaw": re.compile(r"nacyaw=(" + _NUM + r")"),
    "mip1": re.compile(r"mip1=(" + _NUM + r")"),
    "moop1": re.compile(r"moop1=(" + _NUM + r")"),
    "mzb1": re.compile(r"mzb1=(" + _NUM + r")"),
}


class FarmBridge:
    """File-based bridge to a running FAST.Farm simulation.

    Parameters
    ----------
    run_dir:
        Directory in which FAST.Farm runs (where controls.txt and
        measurements_T*.txt live).
    n_turbines:
        Number of turbines (6 for the 2x3 layout).
    """

    def __init__(self, run_dir: os.PathLike | str, n_turbines: int):
        self.run_dir = Path(run_dir)
        self.n_turbines = int(n_turbines)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._last_step_written = -1

    # -- controls ---------------------------------------------------------------
    def write_controls(self, step: int, cmds: dict[int, Cmd]) -> None:
        """Atomically write ``controls.txt`` for the given step.

        ``step`` must be strictly greater than any previously written step, or
        the bridge will ignore the file.
        """
        if step <= self._last_step_written:
            raise ValueError(
                f"step must be strictly increasing; got {step} "
                f"after {self._last_step_written}"
            )
        lines = [f"step={step}"]
        for tid in range(1, self.n_turbines + 1):
            cmd = cmds.get(tid)
            if cmd is None:
                cmd = Cmd(turbine_id=tid)  # empty -> bridge keeps previous values
            lines.append(cmd.to_line())
        lines.append("END")
        text = "\n".join(lines) + "\n"

        target = self.run_dir / "controls.txt"
        tmp = self.run_dir / f".controls.txt.tmp.{os.getpid()}"
        tmp.write_text(text)
        os.replace(tmp, target)  # atomic on POSIX and Windows
        self._last_step_written = step

    # -- measurements -----------------------------------------------------------
    def read_measurements(
        self, retries: int = 5, retry_wait: float = 0.02
    ) -> dict[int, TurbineMeas]:
        """Read and parse ``measurements_T<id>.txt`` for every turbine.

        Returns a dict ``{turbine_id: TurbineMeas}``. Turbines whose file is not
        yet present are omitted. Short/partial reads (the bridge overwriting the
        file) are retried.
        """
        out: dict[int, TurbineMeas] = {}
        for tid in range(1, self.n_turbines + 1):
            meas = self._read_one(tid, retries, retry_wait)
            if meas is not None:
                out[tid] = meas
        return out

    def _read_one(
        self, tid: int, retries: int, retry_wait: float
    ) -> Optional[TurbineMeas]:
        path = self.run_dir / f"measurements_T{tid}.txt"
        for _ in range(retries):
            if not path.exists():
                time.sleep(retry_wait)
                continue
            try:
                text = path.read_text()
            except (OSError, UnicodeDecodeError):
                time.sleep(retry_wait)
                continue
            meas = self._parse(tid, text)
            if meas is not None:
                return meas
            time.sleep(retry_wait)
        return None

    @staticmethod
    def _parse(tid: int, text: str) -> Optional[TurbineMeas]:
        """Parse one measurement file's text into a TurbineMeas (or None if
        essential fields are missing / file is mid-write)."""
        vals: dict[str, float] = {}
        for key, rgx in _FIELD_RE.items():
            m = rgx.search(text)
            if m:
                try:
                    vals[key] = float(m.group(1))
                except ValueError:
                    return None
        # require at least step + power to consider the record usable
        if "step" not in vals or "genpwr" not in vals:
            return None
        return TurbineMeas(
            turbine_id=tid,
            step=int(vals.get("step", -1)),
            t=vals.get("t", 0.0),
            genpwr_kw=vals.get("genpwr", 0.0),
            genspd_rpm=vals.get("genspd", 0.0),
            gentq_nm=vals.get("gentq", 0.0),
            rotspd_rpm=vals.get("rotspd", 0.0),
            wind_x=vals.get("wind_x", 0.0),
            blpitch_deg=vals.get("blpitch", 0.0),
            nacyaw_deg=vals.get("nacyaw", 0.0),
            mip1_knm=vals.get("mip1", 0.0),
            moop1_knm=vals.get("moop1", 0.0),
            mzb1_knm=vals.get("mzb1", 0.0),
        )

    # -- synchronization --------------------------------------------------------
    def wait_step(
        self, step: int, timeout: float = 60.0, poll: float = 0.05
    ) -> dict[int, TurbineMeas]:
        """Block until every turbine reports measurements at ``>= step``.

        Returns the measurements once all turbines have advanced. Raises
        TimeoutError if the deadline passes first.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            meas = self.read_measurements()
            if len(meas) == self.n_turbines and all(
                m.step >= step for m in meas.values()
            ):
                return meas
            time.sleep(poll)
        raise TimeoutError(
            f"Timed out waiting for step {step} from {self.n_turbines} turbines"
        )

    def latest_step(self) -> int:
        """Return the minimum step currently reported across turbines (-1 if
        none)."""
        meas = self.read_measurements()
        if len(meas) < self.n_turbines:
            return -1
        return min(m.step for m in meas.values())

    # -- housekeeping -----------------------------------------------------------
    def reset(self) -> None:
        """Remove stale controls/measurement files from the run directory."""
        for p in [self.run_dir / "controls.txt"] + [
            self.run_dir / f"measurements_T{tid}.txt"
            for tid in range(1, self.n_turbines + 1)
        ]:
            try:
                p.unlink()
            except FileNotFoundError:
                pass
        self._last_step_written = -1
