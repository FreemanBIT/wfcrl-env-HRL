"""
Module-level unit tests (per development plan §5.2 / test requirements).

Focused, fast assertions on individual modules:
  * M2 bridge  : controls.txt format + measurement round-trip
  * M4 induction: power<->induction inversion consistency
  * M5 surrogate: wake direction correctness, param monotonicity
  * M9 modes   : decision-variable selection per mode
  * B1 FLORIDyn: delay and steady-state consistency
  * B2 EnKF    : ambient-state convergence on synthetic data

Run:  pytest tests/test_units.py -v
"""
from __future__ import annotations

import numpy as np
import pytest

from closedloop.case_config import default_case
from closedloop.bridge import FarmBridge
from closedloop.types import Cmd
from closedloop import induction as ind
from closedloop.surrogate import SurrogateModel
from closedloop.control_mode import ControlMode, ControlModeSpec


# --- M4 induction -----------------------------------------------------------
def test_induction_power_inversion():
    for a in [0.05, 0.1, 0.2, 0.25, 1 / 3]:
        for U in [6.0, 8.0, 10.0]:
            p = ind.induction_to_power(a, U)
            a_rec = ind.power_to_induction(p, U)
            assert abs(a_rec - a) < 1e-2, f"a={a} U={U} recovered {a_rec}"


def test_cp_betz_optimum():
    # Cp is maximized at a=1/3 with Cp=16/27
    assert abs(ind.cp_from_induction(1 / 3) - 16 / 27) < 1e-6
    assert ind.cp_from_induction(1 / 3) >= ind.cp_from_induction(0.2)
    assert ind.cp_from_induction(1 / 3) >= ind.cp_from_induction(0.45)


def test_yaw_reduces_power():
    p0 = ind.induction_to_power(1 / 3, 8.0, yaw_deg=0.0)
    p30 = ind.induction_to_power(1 / 3, 8.0, yaw_deg=30.0)
    assert p30 < p0
    # cos^1.88(30 deg)
    assert abs(p30 / p0 - np.cos(np.radians(30)) ** 1.88) < 1e-6


# --- M2 bridge --------------------------------------------------------------
def test_controls_format(tmp_path):
    case = default_case()
    b = FarmBridge(tmp_path, case.n_turbines)
    cmds = {1: Cmd(1, yaw_deg=15.0, power_mw=2.5)}
    b.write_controls(1, cmds)
    content = (tmp_path / "controls.txt").read_text()
    lines = content.strip().split("\n")
    assert lines[0] == "step=1"
    assert lines[-1] == "END"
    assert "T1 yaw=15.0000 power=2.500000" in content


def test_bridge_increasing_step(tmp_path):
    b = FarmBridge(tmp_path, 6)
    b.write_controls(1, {})
    b.write_controls(2, {})
    with pytest.raises(ValueError):
        b.write_controls(2, {})  # non-increasing


# --- M5 surrogate -----------------------------------------------------------
def test_wake_direction_westerly():
    """Under 270 deg, higher-x (downstream column) turbines are waked."""
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    sur.set_conditions(8, 270, 0.06)
    sur.set_controls(np.zeros(6), np.full(6, 1 / 3))
    u = sur.predict()["U_eff_i"]
    # T1 (x=0) upstream full speed; T3 (x=1008) downstream waked
    assert u[0] > u[1] > u[2] - 1e-9   # column 0 > 1 > 2
    assert u[0] > 7.5
    assert u[2] < u[0]


def test_param_monotonicity():
    """Increasing ka (wake expansion vs TI) changes downstream recovery.

    ka acts through kstar = ka*TI + kb, so its effect is probed at elevated TI
    with a lateral offset (yawed upstream) where the Gaussian width matters.
    """
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    sur.set_conditions(8, 270, 0.12)   # higher TI so ka has leverage
    # give the upstream a yaw so the wake is deflected and width controls overlap
    sur.set_controls(np.array([20., 0, 0, 20, 0, 0]), np.full(6, 1 / 3))
    sur.set_params({"ka": 0.2, "kb": 0.004})
    u_low = sur.predict()["U_eff_i"][1]   # first downstream (partial wake)
    sur.set_params({"ka": 0.8, "kb": 0.004})
    u_high = sur.predict()["U_eff_i"][1]
    assert abs(u_low - u_high) > 1e-3, "ka should move the partially-waked speed"


def test_yaw_steering_gain():
    """Yawing upstream turbines increases farm power (wake steering)."""
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    sur.set_conditions(8, 270, 0.06)
    sur.set_controls(np.zeros(6), np.full(6, 1 / 3))
    p0 = sur.predict()["P_farm"]
    sur.set_controls(np.array([25., 25, 0, 25, 25, 0]), np.full(6, 1 / 3))
    p1 = sur.predict()["P_farm"]
    assert p1 > p0


# --- M9 control modes -------------------------------------------------------
def test_mode_decision_vars():
    assert ControlModeSpec(ControlMode.YAW).decision_vars() == ["yaw"]
    assert ControlModeSpec(ControlMode.TORQUE).decision_vars() == ["a"]
    assert ControlModeSpec(ControlMode.PITCH).decision_vars() == ["a"]
    assert set(ControlModeSpec(ControlMode.YAW_TORQUE).decision_vars()) == {"yaw", "a"}
    assert set(ControlModeSpec(ControlMode.YAW_PITCH).decision_vars()) == {"yaw", "a"}


def test_mode_command_channels():
    spec_yaw = ControlModeSpec(ControlMode.YAW)
    c = spec_yaw.to_controls(1, yaw_deg=20.0, a=1 / 3, U_eff=8.0)
    assert c.yaw_deg == 20.0 and c.power_mw is None  # yaw-only, no derating

    spec_pitch = ControlModeSpec(ControlMode.PITCH)
    c = spec_pitch.to_controls(1, yaw_deg=0.0, a=0.2, U_eff=8.0)
    assert c.pitch_deg is not None and c.yaw_deg == 0.0

    spec_yt = ControlModeSpec(ControlMode.YAW_TORQUE)
    c = spec_yt.to_controls(1, yaw_deg=15.0, a=0.25, U_eff=8.0)
    assert c.yaw_deg == 15.0 and c.power_mw is not None


# --- B1 FLORIDyn ------------------------------------------------------------
def test_floridyn_steady_matches_surrogate():
    from closedloop.scheme_b.floridyn_model import FLORIDyn, FLORIDynConfig
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    fd = FLORIDyn(sur, case.layout_x, case.layout_y, FLORIDynConfig(dt=2.0))
    fd.set_conditions(8, 270, 0.06)
    yaw = np.array([20., 0, 0, 20, 0, 0])
    a = np.full(6, 1 / 3)
    # prime to steady state
    for _ in range(80):
        fd.set_controls(yaw, a)
        fd.step()
    p_fd = fd.get_measurements()["P_farm"]
    sur.set_conditions(8, 270, 0.06)
    sur.set_controls(yaw, a)
    p_sur = sur.predict()["P_farm"]
    assert abs(p_fd - p_sur) / p_sur < 0.02


def test_floridyn_delay_positive():
    from closedloop.scheme_b.floridyn_model import FLORIDyn, FLORIDynConfig
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    fd = FLORIDyn(sur, case.layout_x, case.layout_y, FLORIDynConfig(dt=2.0))
    fd.set_conditions(8, 270, 0.06)
    tau = fd.max_delay_steps(8.0)
    assert tau > 0  # there is a finite wake-travel delay across the farm


# --- B2 EnKF ----------------------------------------------------------------
def test_enkf_converges_ambient():
    """EnKF recovers a biased ambient wind speed from power measurements."""
    from closedloop.scheme_b.floridyn_model import FLORIDyn, FLORIDynConfig
    from closedloop.scheme_b.enkf import EnKF, EnKFConfig
    case = default_case()
    sur = SurrogateModel(case.layout_x, case.layout_y, prefer_floris=False)
    fd = FLORIDyn(sur, case.layout_x, case.layout_y, FLORIDynConfig(dt=2.0))

    # ground truth ambient
    U_true, phi_true, TI_true = 9.0, 270.0, 0.06
    yaw = np.zeros(6)
    a = np.full(6, 1 / 3)
    fd.set_conditions(U_true, phi_true, TI_true)
    fd.set_controls(yaw, a)
    fd._recompute_effective()
    p_true = fd.get_measurements()["P_i"]

    # filter starts from a wrong guess
    fd.set_conditions(7.0, 268.0, 0.08)
    enkf = EnKF(fd, EnKFConfig(n_ensemble=40, estimate_params=False), seed=0)
    enkf.initialize(7.0, 268.0, 0.08)
    for _ in range(15):
        enkf.update(p_true, yaw, a)
    est = enkf.state_mean
    assert abs(est["U"] - U_true) < 0.5, f"U not recovered: {est['U']}"
    assert abs(est["phi"] - phi_true) < 2.0, f"phi not recovered: {est['phi']}"
