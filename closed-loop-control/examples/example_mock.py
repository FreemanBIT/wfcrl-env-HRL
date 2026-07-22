"""
Example 1 — Run the three schemes against the greedy baseline on the MOCK plant.

This runs immediately with no FAST.Farm and no FLORIS (uses the built-in
analytical wake). It is the fastest way to confirm the whole pipeline works and
to see the relative behaviour of the schemes.

    python examples/example_mock.py
"""
import sys
from pathlib import Path

# allow running from the repo root without installing
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from closedloop.case_config import default_case
from closedloop.surrogate import SurrogateModel
from closedloop.sensing import Sensing
from closedloop.base_controller import CommandArbiter
from closedloop.runner import ClosedLoopRunner, MockPlant
from closedloop.evaluate import Evaluator
from closedloop import wind_schedule as ws
from closedloop.demos.greedy_baseline import GreedyController
from closedloop.control_mode import ControlMode


def run(controller_factory, name, wind_fn, dt=2.0, n=250, seed=1, mode=1):
    case = default_case()
    plant = MockPlant(case, wind_fn, dt=dt, ti_noise=True, seed=seed)
    sensing = Sensing(case.n_turbines, dt=dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    ctrl = controller_factory(case)
    runner = ClosedLoopRunner(plant, ctrl, sensing, CommandArbiter(yaw_deadband=0.5))
    return runner.run(n, controller_name=name, mode=mode, warmup_steps=40).trajectory


def main():
    case = default_case()
    dt = 2.0
    wind = ws.steady(8.0, 270.0, 0.06)
    ev = Evaluator(transient_s=60.0)

    print(f"Farm: {case.name}, {case.n_turbines} turbines (2x3 @ 4D), NREL 5MW")
    print(f"Upstream turbines (270 deg): {case.upstream_ids}\n")

    # baseline
    greedy = run(lambda c: GreedyController(c.n_turbines, dt), "greedy", wind)

    # Scheme A
    from closedloop.scheme_a import ControllerA
    surA = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    a = run(lambda c: ControllerA(ControlMode.YAW, surA, c.n_turbines), "A", wind)
    print(ev.summary_text(ev.gain_vs(a, greedy), "Scheme A (yaw)"))
    print()

    # Scheme B
    from closedloop.scheme_b import ControllerB
    surB = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    b = run(lambda c: ControllerB(ControlMode.YAW, surB, c.layout_x, c.layout_y,
                                  c.n_turbines, seed=2), "B", wind)
    print(ev.summary_text(ev.gain_vs(b, greedy), "Scheme B (yaw)"))
    print()

    # Scheme C
    from closedloop.scheme_c import ControllerC
    surC = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    c = run(lambda cc: ControllerC(ControlMode.YAW, surC, cc.layout_x, cc.layout_y,
                                   cc.n_turbines, seed=3), "C", wind)
    print(ev.summary_text(ev.gain_vs(c, greedy), "Scheme C (greybox_mpc, yaw)"))


if __name__ == "__main__":
    main()
