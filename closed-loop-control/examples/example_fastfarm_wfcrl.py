"""
Example 3 — Drive FAST.Farm through the project's WFCRL FastFarmInterface.

Use this if you launch FAST.Farm the stock WFCRL way (MPI + ZeroMQ) rather than
the custom file bridge. It reuses your installed ``wfcrl`` package to start and
step FAST.Farm, and plugs the closed-loop controllers on top via WFCRLPlant.

    # UNIX:
    python examples/example_fastfarm_wfcrl.py --controller B --mode 1 --steps 300
    # Windows (WFCRL requires mpiexec):
    mpiexec -n 1 python examples/example_fastfarm_wfcrl.py --controller B --steps 300

Prerequisites
-------------
  * ``wfcrl`` installed and its FAST.Farm binaries set up (see the project README:
    `wfcrl-simulator fastfarm` on UNIX, or the Windows MPI instructions).
  * The 6-turbine case (WFCRL ships ``Turb6_Row2`` = 2 rows of 3 turbines, and
    ``cases.fastfarm_6t``). Adjust the layout to 4D spacing to match this project
    (see the integration guide).

NOTE on API: WFCRL's measurement/command method names vary by version. If this
script errors on binding, open ``closedloop/wfcrl_plant.py`` and set the two
method references in ``_bind`` (a 4-line change), or pass measure_map/command_map.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from closedloop.case_config import default_case
from closedloop.surrogate import SurrogateModel
from closedloop.sensing import Sensing
from closedloop.base_controller import CommandArbiter
from closedloop.runner import ClosedLoopRunner
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
    p.add_argument("--controller", default="B",
                   choices=["greedy", "A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument("--steps", type=int, default=300)
    p.add_argument("--dt", type=float, default=2.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--fstf", default=None, help="path to an existing .fstf file")
    p.add_argument("--floris-yaml", default="cases/farm_2x3_4D/floris_case.yaml")
    args = p.parse_args()

    # --- build the WFCRL interface (their FAST.Farm launcher) ---
    try:
        from wfcrl.interface import FastFarmInterface
    except Exception as e:
        print("ERROR: wfcrl is not installed / importable:", e)
        print("Install and set up wfcrl first (see project README).")
        sys.exit(1)

    if args.fstf:
        iface = FastFarmInterface(fstf_file=args.fstf)
    else:
        try:
            from wfcrl.environments import data_cases as cases
            iface = FastFarmInterface(cases.fastfarm_6t)
        except Exception as e:
            print("Could not build the default 6-turbine WFCRL case:", e)
            print("Pass --fstf pointing at your .fstf file instead.")
            sys.exit(1)

    from closedloop.wfcrl_plant import WFCRLPlant
    case = default_case()
    plant = WFCRLPlant(iface, case.n_turbines)

    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=args.floris_yaml, prefer_floris=True)
    print(f"Surrogate backend: {surrogate.backend}")

    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    controller = build_controller(args.controller, args.mode, case, surrogate,
                                  args.dt, args.seed)

    runner = ClosedLoopRunner(plant, controller, sensing,
                              CommandArbiter(yaw_deadband=0.5))
    result = runner.run(args.steps, controller_name=args.controller,
                        mode=args.mode, warmup_steps=int(60 / args.dt),
                        verbose=True)

    ev = Evaluator(transient_s=60.0)
    print(f"\nDone. Mean farm power (post-transient): "
          f"{ev.mean_farm_power(result.trajectory):.1f} kW.")


if __name__ == "__main__":
    main()
