"""
M7 — Evaluation & metrics (shared measure of closed-loop benefit).

Computes the metrics defined in the derivation document (§4.4):

    farm power gain   dP% = (E_ctrl - E_greedy) / E_greedy * 100
    power pulsation   CoV_i = std(P_i) / mean(P_i)     (after transient t0)
    window energy     E = sum P_farm * dt
    load proxy        blade-root moment equivalent (from moop1/mip1)

Works on trajectories collected by the runner (M8) / demo runner (D2).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# numpy 2.x renamed trapz -> trapezoid; support both
_trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))
if _trapz is None:
    def _trapz(y, x):  # fallback
        dx = np.diff(x)
        return float(np.sum(0.5 * (y[:-1] + y[1:]) * dx))


@dataclass
class Trajectory:
    """Time series collected during a run.

    Arrays are indexed by control step. ``power[k]`` is a length-n_turbines
    array of per-turbine power (kW) at step k.
    """
    t: list[float] = field(default_factory=list)
    step: list[int] = field(default_factory=list)
    power: list[np.ndarray] = field(default_factory=list)      # per-turbine kW
    yaw: list[np.ndarray] = field(default_factory=list)        # per-turbine deg
    wind: list[np.ndarray] = field(default_factory=list)       # per-turbine m/s
    moop: list[np.ndarray] = field(default_factory=list)       # per-turbine kNm
    controller: str = ""
    mode: int = 0

    def add(self, t, step, power, yaw, wind, moop):
        self.t.append(float(t))
        self.step.append(int(step))
        self.power.append(np.asarray(power, float))
        self.yaw.append(np.asarray(yaw, float))
        self.wind.append(np.asarray(wind, float))
        self.moop.append(np.asarray(moop, float))

    # -- array views -----------------------------------------------------------
    @property
    def P(self) -> np.ndarray:
        """(T, n) per-turbine power array (kW)."""
        return np.array(self.power) if self.power else np.zeros((0, 0))

    @property
    def P_farm(self) -> np.ndarray:
        """(T,) farm total power (kW)."""
        return self.P.sum(axis=1) if self.P.size else np.zeros(0)

    @property
    def time(self) -> np.ndarray:
        return np.asarray(self.t)

    # -- persistence -----------------------------------------------------------
    def save_csv(self, path) -> None:
        """Write the trajectory to a CSV (one row per step, per-turbine columns).

        Columns: t, step, P_T1..P_Tn (kW), yaw_T1..yaw_Tn (deg),
        wind_T1..wind_Tn (m/s), moop_T1..moop_Tn (kNm), P_farm (kW).
        """
        from pathlib import Path
        P = self.P
        n = P.shape[1] if P.size else 0
        header = (["t", "step"]
                  + [f"P_T{i+1}" for i in range(n)]
                  + [f"yaw_T{i+1}" for i in range(n)]
                  + [f"wind_T{i+1}" for i in range(n)]
                  + [f"moop_T{i+1}" for i in range(n)]
                  + ["P_farm"])
        rows = []
        for k in range(len(self.t)):
            row = [self.t[k], self.step[k]]
            row += list(self.power[k])
            row += list(self.yaw[k])
            row += list(self.wind[k])
            row += list(self.moop[k])
            row.append(float(np.sum(self.power[k])))
            rows.append(row)
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w") as f:
            f.write(",".join(header) + "\n")
            for row in rows:
                f.write(",".join(f"{v:.6g}" for v in row) + "\n")

    def save(self, path) -> None:
        """Pickle the full trajectory object."""
        import pickle
        from pathlib import Path
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "wb") as f:
            pickle.dump(self, f)

    @staticmethod
    def load(path) -> "Trajectory":
        """Load a pickled trajectory."""
        import pickle
        with open(path, "rb") as f:
            return pickle.load(f)

    @staticmethod
    def from_csv(path) -> "Trajectory":
        """Reconstruct a Trajectory from a CSV written by :meth:`save_csv`."""
        import csv
        traj = Trajectory()
        with open(path) as f:
            reader = csv.reader(f)
            header = next(reader)
            n = sum(1 for h in header if h.startswith("P_T") and h != "P_farm")
            for row in reader:
                vals = [float(x) for x in row]
                t, step = vals[0], int(vals[1])
                off = 2
                P = vals[off:off + n]; off += n
                yaw = vals[off:off + n]; off += n
                wind = vals[off:off + n]; off += n
                moop = vals[off:off + n]; off += n
                traj.add(t, step, P, yaw, wind, moop)
        return traj


class Evaluator:
    """Compute metrics for a single trajectory and compare against a baseline.

    Parameters
    ----------
    transient_s:
        Initial time (s) discarded before statistics (skip FAST.Farm startup
        transient). Aligns with the energy-calculation methodology.
    """

    def __init__(self, transient_s: float = 120.0):
        self.transient_s = transient_s

    def _mask(self, traj: Trajectory) -> np.ndarray:
        t = traj.time
        return t >= (t.min() + self.transient_s) if t.size else np.zeros(0, bool)

    def window_energy(self, traj: Trajectory) -> float:
        """Trapezoidal window energy of farm power (kW·s) after the transient."""
        m = self._mask(traj)
        if m.sum() < 2:
            return 0.0
        t = traj.time[m]
        pf = traj.P_farm[m]
        return float(_trapz(pf, t))

    def mean_farm_power(self, traj: Trajectory) -> float:
        m = self._mask(traj)
        return float(traj.P_farm[m].mean()) if m.sum() else 0.0

    def cov(self, traj: Trajectory) -> np.ndarray:
        """Per-turbine coefficient of variation (after transient)."""
        m = self._mask(traj)
        if m.sum() < 2:
            return np.zeros(traj.P.shape[1] if traj.P.size else 0)
        P = traj.P[m]
        mu = P.mean(axis=0)
        sd = P.std(axis=0)
        return np.where(mu > 1e-6, sd / mu, 0.0)

    def load_proxy(self, traj: Trajectory) -> np.ndarray:
        """Per-turbine blade-root out-of-plane moment RMS (kNm) — fatigue proxy."""
        m = self._mask(traj)
        if m.sum() < 1 or not traj.moop:
            return np.zeros(traj.P.shape[1] if traj.P.size else 0)
        M = np.array(traj.moop)[m]
        return np.sqrt((M ** 2).mean(axis=0))

    def gain_vs(self, traj: Trajectory, baseline: Trajectory) -> dict:
        """Compare a controlled trajectory against a baseline (greedy).

        Returns a dict with net gain %, mean powers, per-turbine power ratios,
        CoV, and load proxy for both.
        """
        e_ctrl = self.window_energy(traj)
        e_base = self.window_energy(baseline)
        gain_pct = 100.0 * (e_ctrl - e_base) / e_base if e_base > 0 else float("nan")

        m_c = self._mask(traj)
        m_b = self._mask(baseline)
        P_ctrl = traj.P[m_c].mean(axis=0) if m_c.sum() else np.zeros(0)
        P_base = baseline.P[m_b].mean(axis=0) if m_b.sum() else np.zeros(0)
        per_turbine_ratio = np.where(P_base > 1e-6, P_ctrl / P_base, np.nan)

        return dict(
            gain_pct=gain_pct,
            energy_ctrl=e_ctrl,
            energy_base=e_base,
            mean_power_ctrl=self.mean_farm_power(traj),
            mean_power_base=self.mean_farm_power(baseline),
            per_turbine_power_ctrl=P_ctrl,
            per_turbine_power_base=P_base,
            per_turbine_ratio=per_turbine_ratio,
            cov_ctrl=self.cov(traj),
            cov_base=self.cov(baseline),
            load_ctrl=self.load_proxy(traj),
            load_base=self.load_proxy(baseline),
        )

    @staticmethod
    def summary_text(gain: dict, controller: str = "controller") -> str:
        """One-paragraph human-readable summary."""
        g = gain["gain_pct"]
        lines = [
            f"[{controller} vs greedy]",
            f"  net farm-power gain : {g:+.2f} %",
            f"  mean farm power     : {gain['mean_power_ctrl']:.1f} kW "
            f"(baseline {gain['mean_power_base']:.1f} kW)",
        ]
        pr = gain["per_turbine_ratio"]
        if pr.size:
            ratios = ", ".join(f"T{i+1}:{r:.2f}" for i, r in enumerate(pr))
            lines.append(f"  per-turbine P ratio : {ratios}")
        return "\n".join(lines)
