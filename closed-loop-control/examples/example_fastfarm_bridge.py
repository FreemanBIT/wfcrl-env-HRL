"""
Example 2 — Drive a live FAST.Farm through the custom file bridge.

This is the PRIMARY path for the project's ``DISCON_bridge.f90`` file protocol
(the one your uploaded project uses). It:

  1. builds the closed-loop controller (Scheme A / B / C),
  2. connects to a FAST.Farm run directory via the file bridge,
  3. runs the loop: read measurements_T*.txt -> control -> write controls.txt.

You start FAST.Farm SEPARATELY (see the integration guide), pointing it at the
same ``RUN_DIR``. This script then attaches to it.

Prerequisites
-------------
  * The bridge DLL ``DISCON_WT<i>.dll`` compiled from the project
    ``wfcrl/simulators/fastfarm/src`` (use its compile.bat / _compile.py).
  * Per-turbine ``DISCON_T<i>.IN`` (first line = turbine id) in RUN_DIR — these
    are generated for you under ``cases/farm_2x3_4D/`` by case_config, or by
    this script if you pass --write-inputs.
  * FLORIS installed (recommended) so the surrogate is high-fidelity; otherwise
    the analytical fallback is used.

Usage
-----
    # Option A: one-step launcher (recommended — generates case + starts FAST.Farm + runs loop)
    python examples/example_fastfarm_launcher.py --duration 300 --controller A --mode 1

    # Option B: two-step (you start FAST.Farm manually via WFCRL, then attach)
    python examples/example_fastfarm_bridge.py --run-dir <ff_output_dir>/FarmInputs/ \\
        --controller A --mode 1 --steps 150 --dt 2.0 --write-inputs

    --run-dir MUST be the FAST.Farm working directory (where controls.txt and
    measurements_T*.txt live). When using ContinuousFastFarmInterface, this is
    ``<output_dir>/FarmInputs/`` (the ``_farm_base`` attribute).

    --dt MUST equal the FAST.Farm low-resolution time step (DT_low, typically 2.0s).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from closedloop.case_config import default_case
from closedloop.surrogate import SurrogateModel
from closedloop.sensing import Sensing
from closedloop.base_controller import CommandArbiter
from closedloop.bridge import FarmBridge
from closedloop.runner import ClosedLoopRunner, FastFarmPlant
from closedloop.evaluate import Evaluator
from closedloop.control_mode import ControlMode


def build_controller(name, mode, case, surrogate, dt, seed):
    m = ControlMode(mode)
    name = name.lower()
    if name == "greedy":
        from closedloop.demos.greedy_baseline import GreedyController
        return GreedyController(case.n_turbines, dt)
    if name == "a":
        from closedloop.scheme_a import ControllerA
        return ControllerA(m, surrogate, case.n_turbines)
    if name == "b":
        from closedloop.scheme_b import ControllerB
        return ControllerB(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=seed)
    if name == "c":
        from closedloop.scheme_c import ControllerC
        return ControllerC(m, surrogate, case.layout_x, case.layout_y,
                          case.n_turbines, seed=seed)
    raise ValueError(name)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", required=True, help="FAST.Farm run directory")
    p.add_argument("--controller", default="B",
                   choices=["greedy", "A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument("--steps", type=int, default=300)
    p.add_argument("--dt", type=float, default=2.0,
                   help="MUST equal FAST.Farm DT_low")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--floris-yaml", default="cases/farm_2x3_4D/floris_case.yaml")
    p.add_argument("--direction-file", default=None,
                   help="optional scanning-lidar wind-direction file")
    p.add_argument("--write-inputs", action="store_true",
                   help="write DISCON_T<i>.IN into run-dir before starting")
    p.add_argument("--step-timeout", type=float, default=120.0)
    args = p.parse_args()

    case = default_case()
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    if args.write_inputs:
        paths = case.write_discon_inputs(run_dir)
        print(f"Wrote {len(paths)} DISCON_T*.IN files to {run_dir}")

    # surrogate: try FLORIS, else analytical fallback
    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=args.floris_yaml, prefer_floris=True)
    print(f"Surrogate backend: {surrogate.backend}")

    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    controller = build_controller(args.controller, args.mode, case, surrogate,
                                  args.dt, args.seed)

    bridge = FarmBridge(run_dir, case.n_turbines)
    bridge.reset()
    plant = FastFarmPlant(bridge, step_timeout=args.step_timeout,
                          direction_file=args.direction_file)

    runner = ClosedLoopRunner(plant, controller, sensing,
                              CommandArbiter(yaw_deadband=0.5))
    print(f"Attaching to FAST.Farm in {run_dir} — start FAST.Farm now if not "
          f"already running.\nController={args.controller} mode={args.mode} "
          f"steps={args.steps} dt={args.dt}s")

    result = runner.run(args.steps, controller_name=args.controller,
                        mode=args.mode, warmup_steps=int(60 / args.dt),
                        verbose=True)

    ev = Evaluator(transient_s=60.0)
    traj = result.trajectory
    print(f"\nDone. Mean farm power (post-transient): "
          f"{ev.mean_farm_power(traj):.1f} kW over {result.n_steps} steps.")
    print("To compare against greedy, run this script again with "
          "--controller greedy and compare mean farm power / energy.")


if __name__ == "__main__":
    main()
