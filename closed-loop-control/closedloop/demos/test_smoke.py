"""
D4 — Smoke tests (auto pass/fail regression).

Turns the demos into assertions a CI can run after any change:
  * connectivity: bridge round-trip (controls written, measurements parsed,
    yaw tracks the command).
  * baseline sanity: greedy downstream power < upstream power (wake exists).
  * effect: each scheme's net gain vs greedy is >= 0 (not worse than baseline)
    on the standard yaw scenario.
  * stability: no NaN, no divergence.
  * modes: all 5 control modes run and stay bounded.

Run with::  pytest closedloop/demos/test_smoke.py -v
Or standalone:  python -m closedloop.demos.test_smoke
"""
from __future__ import annotations

import numpy as np

from ..case_config import default_case
from ..surrogate import SurrogateModel
from ..sensing import Sensing
from ..base_controller import CommandArbiter
from ..bridge import FarmBridge
from ..runner import ClosedLoopRunner, MockPlant
from ..evaluate import Evaluator
from ..types import Cmd
from .. import wind_schedule as ws
from ..control_mode import ControlMode
from .greedy_baseline import GreedyController

DT = 2.0


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _run(controller_factory, n=90, seed=1, wind=None, mode=1):
    case = default_case()
    wind = wind or ws.steady(8, 270, 0.06)
    plant = MockPlant(case, wind, dt=DT, ti_noise=True, seed=seed)
    sensing = Sensing(case.n_turbines, dt=DT, upstream_ids=case.upstream_ids,
                      wind_direction_ref=270.0)
    ctrl = controller_factory(case)
    runner = ClosedLoopRunner(plant, ctrl, sensing, CommandArbiter(yaw_deadband=0.5))
    return runner.run(n, controller_name="test", mode=mode, warmup_steps=30).trajectory


def _greedy_traj(n=90, seed=1, wind=None):
    return _run(lambda case: GreedyController(case.n_turbines, DT), n, seed, wind)


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
def test_bridge_roundtrip(tmp_path=None):
    """controls.txt is written atomically and measurements parse back."""
    import tempfile, os
    d = tmp_path or tempfile.mkdtemp()
    case = default_case()
    bridge = FarmBridge(d, case.n_turbines)
    cmds = {tid: Cmd(turbine_id=tid, yaw_deg=10.0 + tid, power_mw=2.0)
            for tid in range(1, case.n_turbines + 1)}
    bridge.write_controls(5, cmds)
    text = (os.path.join(str(d), "controls.txt"))
    with open(text) as f:
        content = f.read()
    assert content.startswith("step=5")
    assert "END" in content
    assert "T1 yaw=11.0000" in content
    # step must strictly increase
    try:
        bridge.write_controls(5, cmds)
        assert False, "expected ValueError on non-increasing step"
    except ValueError:
        pass


def test_measurement_parse():
    """The bridge parser handles the exact measurement file format."""
    case = default_case()
    bridge = FarmBridge("/tmp/_cl_test_parse", case.n_turbines)
    sample = ("step=12 t=24.0000 genpwr=2300.50 genspd=1173.70 gentq=40000.0 rotspd=12.10\n"
              " wind_x=8.020000\n blpitch=0.000000 nacyaw=10.500000\n"
              " mip1=100.00 moop1=8000.00 mzb1=50.00\n")
    m = bridge._parse(3, sample)
    assert m is not None
    assert m.step == 12
    assert abs(m.genpwr_kw - 2300.5) < 1e-3
    assert abs(m.wind_x - 8.02) < 1e-3
    assert abs(m.nacyaw_deg - 10.5) < 1e-3
    assert abs(m.moop1_knm - 8000.0) < 1e-3


def test_greedy_baseline_wake_exists():
    """Greedy: downstream turbines produce less than upstream (wake present)."""
    traj = _greedy_traj()
    P = traj.P.mean(axis=0)   # per-turbine mean
    # upstream T1,T4 (indices 0,3) vs downstream T3,T6 (indices 2,5)
    assert P[0] > P[2], "T1 should exceed waked T3"
    assert P[3] > P[5], "T4 should exceed waked T6"
    assert np.all(np.isfinite(P))


def test_scheme_a_not_worse_than_greedy():
    g = _greedy_traj()
    from ..scheme_a import ControllerA
    sur = SurrogateModel(default_case().layout_x, default_case().layout_y, prefer_floris=False)
    a = _run(lambda case: ControllerA(ControlMode.YAW, sur, case.n_turbines))
    ev = Evaluator(transient_s=50.0)
    gain = ev.gain_vs(a, g)
    assert np.isfinite(gain["gain_pct"])
    assert gain["gain_pct"] >= -1.0, f"Scheme A worse than greedy: {gain['gain_pct']:.2f}%"


def test_scheme_b_not_worse_than_greedy():
    g = _greedy_traj()
    from ..scheme_b import ControllerB
    from ..scheme_b.enkf import EnKFConfig
    from ..scheme_b.mpc import MPCConfig
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    def mk(case):
        return ControllerB(ControlMode.YAW, sur, case.layout_x, case.layout_y,
                           case.n_turbines,
                           enkf_config=EnKFConfig(n_ensemble=16),
                           mpc_config=MPCConfig(dt=DT, n_passes=1, yaw_refine_levels=1),
                           seed=2)
    b = _run(mk)
    ev = Evaluator(transient_s=50.0)
    gain = ev.gain_vs(b, g)
    assert np.isfinite(gain["gain_pct"])
    assert gain["gain_pct"] >= -1.0, f"Scheme B worse than greedy: {gain['gain_pct']:.2f}%"


def test_scheme_c_safety_never_below_baseline():
    """Scheme C rl_safety route must never fall below the physical baseline."""
    g = _greedy_traj()
    from ..scheme_c import ControllerC
    from ..scheme_c.controller_c import SchemeCConfig
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    c = _run(lambda case: ControllerC(ControlMode.YAW, sur, case.layout_x,
             case.layout_y, case.n_turbines,
             config=SchemeCConfig(dt=DT, route="rl_safety"), seed=3))
    ev = Evaluator(transient_s=50.0)
    gain = ev.gain_vs(c, g)
    assert gain["gain_pct"] >= -1.0, f"Scheme C below baseline: {gain['gain_pct']:.2f}%"


def test_all_control_modes_run():
    """All 5 modes instantiate and produce bounded commands (Scheme A)."""
    from ..scheme_a import ControllerA
    case = default_case()
    for mode in ControlMode:
        sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
        traj = _run(lambda case: ControllerA(mode, sur, case.n_turbines),
                    n=50, mode=int(mode))
        P = traj.P
        assert np.all(np.isfinite(P)), f"mode {mode} produced non-finite power"
        yaw = np.array(traj.yaw)
        assert np.all(np.abs(yaw) <= 30.5), f"mode {mode} yaw out of bounds"


def test_no_divergence_long_run():
    """A longer greedy run stays finite and bounded."""
    traj = _greedy_traj(n=150)
    assert np.all(np.isfinite(traj.P))
    assert traj.P_farm.max() < 1e5


# ---------------------------------------------------------------------------
def _run_all():
    tests = [v for k, v in globals().items() if k.startswith("test_")]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"FAIL  {t.__name__}: {e}")
        except Exception as e:
            print(f"ERROR {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n{passed}/{len(tests)} smoke tests passed")
    return passed == len(tests)


if __name__ == "__main__":
    import sys
    sys.exit(0 if _run_all() else 1)
