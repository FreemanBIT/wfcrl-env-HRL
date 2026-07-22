"""
D2 — Demo runner (single entry point).

Runs one closed-loop episode for a chosen controller/mode/wind and stores the
trajectory. Greedy is a first-class controller so every scheme can be compared
against it under identical conditions.

Usage (CLI)::

    python -m closedloop.demos.run_demo --controller A --mode 1 --wind steady_8ms \\
        --duration 400 --seed 0 --out results/

    python -m closedloop.demos.run_demo --controller greedy --wind step --duration 600

By default it uses the MockPlant (surrogate-driven) so it runs without a
FAST.Farm binary. Pass ``--fastfarm RUN_DIR`` to drive a live FAST.Farm through
the file bridge instead (you start FAST.Farm separately, pointing at RUN_DIR).
"""
from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from typing import Optional

import numpy as np

from ..case_config import default_case
from ..surrogate import SurrogateModel
from ..sensing import Sensing
from ..base_controller import CommandArbiter
from ..bridge import FarmBridge
from ..runner import ClosedLoopRunner, MockPlant, FastFarmPlant
from ..evaluate import Trajectory
from .. import wind_schedule as ws
from ..control_mode import ControlMode

from .greedy_baseline import GreedyController


def build_controller(name: str, mode: int, case, surrogate, dt, seed):
    """Instantiate the requested controller."""
    m = ControlMode(mode)
    name = name.lower()
    if name == "greedy":
        return GreedyController(case.n_turbines, dt)
    if name == "a":
        from ..scheme_a import ControllerA
        return ControllerA(m, surrogate, case.n_turbines)
    if name == "b":
        from ..scheme_b import ControllerB
        return ControllerB(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=seed)
    if name == "c":
        from ..scheme_c import ControllerC
        return ControllerC(m, surrogate, case.layout_x, case.layout_y,
                          case.n_turbines, seed=seed)
    raise ValueError(f"unknown controller: {name}")


def run_demo(
    controller: str = "greedy",
    mode: int = 1,
    wind: str = "steady_8ms",
    duration: float = 400.0,
    dt: float = 2.0,
    seed: int = 0,
    out: Optional[str] = None,
    fastfarm_dir: Optional[str] = None,
    warmup_s: float = 60.0,
    verbose: bool = True,
) -> Trajectory:
    """Run one demo episode and return (and optionally save) the trajectory."""
    case = default_case()
    wind_fn = ws.from_name(wind)
    n_steps = int(duration / dt)
    warmup_steps = int(warmup_s / dt)

    surrogate = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    sensing = Sensing(case.n_turbines, dt=dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    ctrl = build_controller(controller, mode, case, surrogate, dt, seed)

    if fastfarm_dir:
        bridge = FarmBridge(fastfarm_dir, case.n_turbines)
        plant = FastFarmPlant(bridge)
        case.write_discon_inputs(Path(fastfarm_dir))
    else:
        plant = MockPlant(case, wind_fn, dt=dt, ti_noise=True, seed=seed)

    runner = ClosedLoopRunner(plant, ctrl, sensing, CommandArbiter(yaw_deadband=0.5))
    if verbose:
        print(f"Running demo: controller={controller} mode={mode} wind={wind} "
              f"duration={duration}s ({n_steps} steps)")
    result = runner.run(n_steps, controller_name=controller, mode=mode,
                        warmup_steps=warmup_steps, verbose=verbose)
    traj = result.trajectory

    if out:
        outdir = Path(out)
        outdir.mkdir(parents=True, exist_ok=True)
        stem = f"trajectory_{controller}_mode{mode}_{wind}"
        with open(outdir / f"{stem}.pkl", "wb") as f:
            pickle.dump(traj, f)
        traj.save_csv(outdir / f"{stem}.csv")
        if verbose:
            print(f"Saved trajectory -> {outdir / stem}.pkl / .csv")
    return traj


def main(argv=None):
    p = argparse.ArgumentParser(description="Closed-loop control demo runner")
    p.add_argument("--controller", default="greedy",
                   choices=["greedy", "A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument("--wind", default="steady_8ms",
                   help="steady_8ms | step | ramp | speed_ramp")
    p.add_argument("--duration", type=float, default=400.0)
    p.add_argument("--dt", type=float, default=2.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None, help="output directory for trajectory")
    p.add_argument("--fastfarm", default=None,
                   help="FAST.Farm run directory (live bridge); omit for mock")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args(argv)

    run_demo(
        controller=args.controller, mode=args.mode, wind=args.wind,
        duration=args.duration, dt=args.dt, seed=args.seed, out=args.out,
        fastfarm_dir=args.fastfarm, verbose=not args.quiet,
    )


if __name__ == "__main__":
    main()
