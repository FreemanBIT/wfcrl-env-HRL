"""
M8 — Experiment runner (closed-loop orchestration).

Drives one closed-loop episode:

    for each control step k:
        meas   = plant.read()                # measurements_T*.txt (or mock)
        flow   = sensing.estimate(meas)
        cmds   = controller.step(flow, meas)
        cmds   = arbiter.apply_limits(cmds)
        plant.write(step, cmds)              # controls.txt (or mock)
        record trajectory

The "plant" is either:
  * :class:`FastFarmPlant` — talks to a live FAST.Farm through the file bridge
    (M2). You start FAST.Farm separately (e.g. via the project's runner), then
    point this at the same run directory.
  * :class:`MockPlant` — a lightweight surrogate-driven stand-in used for
    development, unit tests, and the demo smoke tests when no FAST.Farm binary
    is available. It responds to yaw/derating so the whole pipeline (sensing ->
    controller -> arbiter -> evaluate) can be exercised end-to-end.

The wind schedule (steady / ramp / step) is provided by :mod:`wind_schedule`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from .types import TurbineMeas, Cmd, FlowEstimate
from .bridge import FarmBridge
from .sensing import Sensing
from .base_controller import BaseController, CommandArbiter
from .surrogate import SurrogateModel
from .evaluate import Trajectory
from .induction import power_to_induction, cp_from_induction, A_ROTOR, RHO_AIR, CP_YAW_EXP
from .case_config import FarmCase
import math


# ---------------------------------------------------------------------------
# Plant abstractions
# ---------------------------------------------------------------------------
class Plant:
    """Interface a runner needs from a plant."""

    def read(self, step: int) -> dict[int, TurbineMeas]:
        raise NotImplementedError

    def write(self, step: int, cmds: dict[int, Cmd]) -> None:
        raise NotImplementedError

    def current_direction(self) -> Optional[float]:
        """Farm-level wind direction (deg) if the plant can provide it (e.g. a
        scanning-lidar file for FAST.Farm, or ground truth for the mock).
        Returns None when unavailable, in which case sensing keeps its last
        estimate."""
        return None

    def reset(self) -> None:
        pass


class FastFarmPlant(Plant):
    """Live FAST.Farm plant via the file bridge (M2).

    Parameters
    ----------
    bridge:
        The file bridge (M2).
    step_timeout:
        Max seconds to wait for each step's measurements.
    direction_file:
        Optional path to a farm-level wind-direction file written by an external
        source (the scanning-lidar channel, ~15 min update). If present, its
        single float (deg) is returned by :meth:`current_direction`. When
        absent, direction falls back to sensing's reference.
    """

    def __init__(
        self,
        bridge: FarmBridge,
        step_timeout: float = 60.0,
        direction_file: Optional[str] = None,
    ):
        self.bridge = bridge
        self.step_timeout = step_timeout
        self.direction_file = direction_file

    def read(self, step: int) -> dict[int, TurbineMeas]:
        return self.bridge.wait_step(step, timeout=self.step_timeout)

    def write(self, step: int, cmds: dict[int, Cmd]) -> None:
        self.bridge.write_controls(step, cmds)

    def current_direction(self) -> Optional[float]:
        if not self.direction_file:
            return None
        try:
            from pathlib import Path
            txt = Path(self.direction_file).read_text().strip()
            return float(txt.split()[0])
        except Exception:
            return None

    def reset(self) -> None:
        self.bridge.reset()


class MockPlant(Plant):
    """Surrogate-driven mock plant for testing without a FAST.Farm binary.

    Uses the analytical wake model to compute per-turbine effective wind and
    power in response to the last commands, adds optional turbulence noise, and
    returns them in the same TurbineMeas structure the bridge would.

    This is intentionally simple: it lets the full software pipeline run and be
    asserted on, and it reproduces the qualitative wake behaviour (downstream
    turbines lose power; upstream yaw recovers some downstream power).
    """

    def __init__(
        self,
        case: FarmCase,
        wind_fn: Callable[[float], tuple[float, float, float]],
        dt: float = 2.0,
        dt_low: float = 0.05,      # FAST.Farm internal timestep for actuator dynamics
        ti_noise: bool = True,
        seed: int = 0,
    ):
        self.case = case
        self.wind_fn = wind_fn
        self.dt = dt
        self.dt_low = dt_low  # sub-step for realistic first-order actuator response
        self.ti_noise = ti_noise
        self.rng = np.random.default_rng(seed)
        self.n = case.n_turbines
        self._model = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
        # actuator state
        self._yaw = np.zeros(self.n)          # current nacelle yaw (deg)
        self._induction = np.full(self.n, 1.0 / 3.0)
        self._t = 0.0

    def reset(self) -> None:
        self._yaw[:] = 0.0
        self._induction[:] = 1.0 / 3.0
        self._t = 0.0

    def current_direction(self) -> Optional[float]:
        _, phi, _ = self.wind_fn(self._t)
        return phi

    def write(self, step: int, cmds: dict[int, Cmd]) -> None:
        """Apply commands to the mock actuators with first-order yaw dynamics."""
        for tid in range(1, self.n + 1):
            i = tid - 1
            cmd = cmds.get(tid)
            if cmd is None:
                continue
            # yaw: first-order dynamics matching DISCON bridge (0.2 gain per DT_low)
            if cmd.yaw_deg is not None:
                target = cmd.yaw_deg
                n_sub = max(1, int(round(self.dt / self.dt_low)))
                for _ in range(n_sub):
                    self._yaw[i] += 0.2 * (target - self._yaw[i]) * self.dt_low
            # induction: from power set-point or explicit; else greedy
            U = self._last_ueff[i] if hasattr(self, "_last_ueff") else 8.0
            if cmd.power_mw is not None:
                self._induction[i] = power_to_induction(
                    cmd.power_mw * 1e6, max(U, 0.5), yaw_deg=self._yaw[i]
                )
            elif cmd.pitch_deg is not None:
                # crude: map pitch back to a Cp reduction -> induction
                ratio = max(0.0, 1.0 - cmd.pitch_deg * 0.04)
                self._induction[i] = _induction_from_cp_ratio(ratio)
            # torque handled implicitly via power path in this mock

    def read(self, step: int) -> dict[int, TurbineMeas]:
        """Advance the mock plant one step and return measurements."""
        self._t += self.dt
        U_inf, phi, TI = self.wind_fn(self._t)
        self._model.set_conditions(U_inf, phi, TI)
        self._model.set_controls(self._yaw, self._induction)
        pred = self._model.predict()
        u_eff = np.asarray(pred["U_eff_i"], float)
        self._last_ueff = u_eff
        p_kw = np.asarray(pred["P_i"], float)

        # turbulence noise on power and wind
        if self.ti_noise:
            noise = self.rng.normal(0.0, TI, size=self.n)
            p_kw = p_kw * (1.0 + 0.5 * noise)
            u_meas = u_eff * (1.0 + noise)
        else:
            u_meas = u_eff

        out: dict[int, TurbineMeas] = {}
        for tid in range(1, self.n + 1):
            i = tid - 1
            # crude blade-root load proxy grows with thrust and turbulence
            ct = 4.0 * self._induction[i] * (1.0 - self._induction[i])
            moop = 8000.0 * ct * (u_eff[i] / 8.0) ** 2 * (1.0 + 2.0 * abs(
                self.rng.normal(0.0, TI)))
            out[tid] = TurbineMeas(
                turbine_id=tid,
                step=step,
                t=self._t,
                genpwr_kw=max(0.0, p_kw[i]),
                genspd_rpm=1173.0 * min(u_eff[i] / 11.4, 1.0),  # proportional to rated speed
                gentq_nm=0.0,
                rotspd_rpm=12.1,
                wind_x=max(0.1, u_meas[i]),
                blpitch_deg=0.0,
                nacyaw_deg=float(self._yaw[i]),
                mip1_knm=0.0,
                moop1_knm=float(moop),
                mzb1_knm=0.0,
            )
        return out


def _induction_from_cp_ratio(ratio: float) -> float:
    """Invert Cp(a)/Cp_greedy = ratio on the derating branch a in [0,1/3]."""
    cp_target = ratio * (16.0 / 27.0)
    lo, hi = 0.0, 1.0 / 3.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if cp_from_induction(mid) < cp_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------
@dataclass
class RunResult:
    trajectory: Trajectory
    n_steps: int


class ClosedLoopRunner:
    """Runs a closed-loop episode against a plant."""

    def __init__(
        self,
        plant: Plant,
        controller: BaseController,
        sensing: Sensing,
        arbiter: Optional[CommandArbiter] = None,
    ):
        self.plant = plant
        self.controller = controller
        self.sensing = sensing
        self.arbiter = arbiter or CommandArbiter()

    def run(
        self,
        n_steps: int,
        controller_name: str = "",
        mode: int = 0,
        warmup_steps: int = 0,
        verbose: bool = False,
    ) -> RunResult:
        """Run ``n_steps`` control steps and return the recorded trajectory.

        ``warmup_steps`` lets the plant settle (commands applied but not scored)
        before recording — useful to skip startup transients in mock runs.
        """
        self.plant.reset()
        self.controller.reset()
        self.arbiter.reset()

        traj = Trajectory(controller=controller_name, mode=mode)
        n = self.controller.n_turbines

        for k in range(n_steps):
            step = k + 1
            meas = self.plant.read(step)
            # update farm-level wind direction (scanning-lidar / ground-truth)
            phi = self.plant.current_direction()
            if phi is not None:
                self.sensing.set_farm_direction(phi)
            flow = self.sensing.estimate(meas)
            cmds = self.controller.step(flow, meas)
            cmds = self.arbiter.apply_limits(cmds)
            self.plant.write(step, cmds)

            if k >= warmup_steps and meas:
                ids = sorted(meas.keys())
                traj.add(
                    t=max(m.t for m in meas.values()),
                    step=step,
                    power=[meas[i].genpwr_kw for i in ids],
                    yaw=[meas[i].nacyaw_deg for i in ids],
                    wind=[meas[i].wind_x for i in ids],
                    moop=[meas[i].moop1_knm for i in ids],
                )
            if verbose and k % max(1, n_steps // 10) == 0:
                pf = sum(m.genpwr_kw for m in meas.values()) if meas else 0.0
                print(f"  step {step:4d}/{n_steps}  farm_power={pf:8.1f} kW")

        return RunResult(trajectory=traj, n_steps=n_steps)
