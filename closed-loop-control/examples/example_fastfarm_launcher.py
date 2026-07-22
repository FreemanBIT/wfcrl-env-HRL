"""
Launch FAST.Farm + closed-loop controller in one step (WFCRL native).

Drives FAST.Farm directly through ContinuousFastFarmInterface — no file-bridge
middleware needed. The WFCRL interface handles setup, DISCON DLL deployment,
controls.txt I/O, and measurement polling.

Usage:
    python examples/example_fastfarm_launcher.py --duration 300 --controller A --mode 1
"""
import argparse, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
_wfcrl_root = Path(__file__).resolve().parents[2]
if (_wfcrl_root / "wfcrl").exists():
    sys.path.insert(0, str(_wfcrl_root))

import numpy as np
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.config.types import WindConfig, WindType
from wfcrl.config.control import ControlInput
from wfcrl.engine.fastfarm_continuous import ContinuousFastFarmInterface

from closedloop.case_config import FarmCase, default_case
from closedloop.surrogate import SurrogateModel
from closedloop.sensing import Sensing
from closedloop.base_controller import CommandArbiter
from closedloop.evaluate import Evaluator, Trajectory
from closedloop.control_mode import ControlMode
from closedloop.types import TurbineMeas


def _cmds_to_control_input(cmds, n_turbines, wind_speed, wind_dir) -> ControlInput:
    """Convert closedloop Cmd dict -> wfcrl ControlInput.

    Cmd objects already carry power_mw / pitch_deg / yaw_deg as set by
    ControlModeSpec.to_controls(). Just copy them into WFCRL arrays.
    """
    yaw = np.zeros(n_turbines)
    pitch = np.zeros(n_turbines)
    power = np.zeros(n_turbines)
    mode_arr = np.zeros(n_turbines, dtype=np.int32)
    for tid, c in cmds.items():
        i = tid - 1
        if 0 <= i < n_turbines:
            yaw[i] = c.yaw_deg if c.yaw_deg is not None else 0.0
            pitch[i] = c.pitch_deg if c.pitch_deg is not None else 0.0
            power[i] = c.power_mw if c.power_mw is not None else 0.0
            # determine mode from which channels are active
            if c.yaw_deg is not None and c.power_mw is not None:
                mode_arr[i] = 4
            elif c.yaw_deg is not None:
                mode_arr[i] = 0  # yaw only
            elif c.power_mw is not None:
                mode_arr[i] = 1  # power only
    return ControlInput(mode=mode_arr.astype(np.int32), yaw=yaw, pitch=pitch, power=power)


def _output_to_meas(output) -> dict[int, TurbineMeas]:
    """Convert wfcrl SimulationOutput -> dict of closedloop TurbineMeas."""
    meas = {}
    pw = output.power_mw
    ws = output.wind_speed
    yw = output.yaw_deg
    pitch = output.pitch_deg
    torque = output.torque_nm
    rotor = output.rotor_speed_rpm
    blades = output.blade_loads
    t = output.time[0] if output.time is not None and len(output.time) > 0 else 0.0
    for i in range(output.n_turbines):
        tid = i + 1
        meas[tid] = TurbineMeas(
            turbine_id=tid, step=-1, t=t,
            genpwr_kw=float(pw[0, i] * 1e3) if pw is not None and pw.size > i else 0.0,
            genspd_rpm=0.0, gentq_nm=float(torque[0, i]) if torque is not None else 0.0,
            rotspd_rpm=float(rotor[0, i]) if rotor is not None else 0.0,
            wind_x=float(ws[0, i]) if ws is not None else 8.0,
            blpitch_deg=float(pitch[0, i]) if pitch is not None else 0.0,
            nacyaw_deg=float(yw[0, i]) if yw is not None else 0.0,
            mip1_knm=float(blades[0, i, 0]) if blades is not None and blades.size > i * 3 else 0.0,
            moop1_knm=float(blades[0, i, 1]) if blades is not None and blades.size > i * 3 + 1 else 0.0,
            mzb1_knm=float(blades[0, i, 2]) if blades is not None and blades.size > i * 3 + 2 else 0.0,
        )
    return meas


def _parse_discon_file(fpath: str) -> dict:
    """Parse a DISCON measurement file (key=value with multi-token values)."""
    vals: dict = {}
    last_label = None
    try:
        with open(fpath, 'r') as f:
            for line in f:
                for part in line.strip().split():
                    if '=' in part:
                        k, v = part.split('=', 1)
                        v = v.strip()
                        if v:
                            try: vals[k] = float(v)
                            except ValueError: pass
                            last_label = None
                        else:
                            last_label = k  # value on next token
                    elif last_label is not None:
                        try: vals[last_label] = float(part)
                        except ValueError: pass
                        last_label = None
    except OSError:
        pass
    return vals


def _ensure_all_turbines_measured(output, ff, n_turbines: int, timeout: float = 0.5):
    """Re-read measurement files until all N turbines have fresh wind speed.

    WFCRL's _read_all() exits on first match; DISCON in parallel OpenFAST
    threads writes at different wall-clock times.  This polls until all N
    files report wind_x > 0.1 OR timeout expires (accepts partial data then).
    """
    import time as _time, os as _os
    n = n_turbines
    ws = output.wind_speed
    if ws is not None and np.all(ws[0, :n] > 0.1):
        return

    farm_base = getattr(ff, '_farm_base', None)
    if not farm_base:
        return

    # Accept any step >= the one wait_step just collected — DISCON advances
    # between wait_step's poll and our re-read, so files may be ahead.
    min_step = getattr(ff, '_step_idx', 1) - 1
    wdir = (ff._current_wind.direction if getattr(ff, '_current_wind', None)
            else ff.config.wind.direction)

    start = _time.time()
    while _time.time() - start < timeout:
        count = 0
        for t_id in range(1, n + 1):
            fpath = _os.path.join(farm_base, f'measurements_T{t_id}.txt')
            vals = _parse_discon_file(fpath)
            if not vals:
                continue
            step = int(vals.get('step', -1))
            if step < min_step:          # stale — skip
                continue
            wind_x = vals.get('wind_x', 0.0)
            if wind_x > 0.1 and ws is not None and t_id <= ws.shape[1]:
                i = t_id - 1
                ws[0, i] = wind_x
                if output.power_mw is not None and t_id <= output.power_mw.shape[1]:
                    output.power_mw[0, i] = vals.get('genpwr', 0.0) / 1000.0
                if output.yaw_deg is not None and t_id <= output.yaw_deg.shape[1]:
                    from wfcrl.engine.angle_utils import misalignment_from_nacyaw
                    output.yaw_deg[0, i] = misalignment_from_nacyaw(
                        wdir, vals.get('nacyaw', 0.0))
                count += 1
        if count >= n:
            return
        _time.sleep(0.02)

    missing = n - (int(np.sum(ws[0, :n] > 0.1)) if ws is not None else n)
    if missing > 0:
        print(f"  [warn] step {min_step}: {missing}/{n} turbines still missing"
              f" after {timeout}s")


def main():
    p = argparse.ArgumentParser(description="FAST.Farm + closed-loop launcher (WFCRL native)")
    p.add_argument("--duration", type=float, default=300)
    p.add_argument("--dt", type=float, default=2.0, help="FAST.Farm DT_low")
    p.add_argument("--controller", default="greedy",
                   choices=["greedy", "A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1,2,3,4,5])
    p.add_argument("--wind-speed", type=float, default=8.0)
    p.add_argument("--wind-dir", type=float, default=270.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-floris", action="store_true")
    args = p.parse_args()

    # Use WFCRL built-in "6T" layout — y coords centred at 0, compatible with
    # _ff_case.py grid generation which assumes symmetry around y=0.
    # default_case() uses y=[0,504] (offset) which puts row-1 wakes at the
    # low-res domain boundary, causing Grid4D abort.
    case = FarmCase.from_wfcrl_layout("6T", wind_speed=args.wind_speed)
    n_steps = max(1, int(args.duration / args.dt))
    warmup_steps = int(60 / args.dt)

    # ── 1. Generate FAST.Farm case via WFCRL ────────────────────────────
    ts = int(time.time())
    output_dir = str(Path(__file__).resolve().parents[1] /
                     f"__simul__/launcher/{args.controller}_mode{args.mode}_{ts}")

    wind = WindConfig(wind_type=WindType.STEADY, speed=args.wind_speed,
                      direction=args.wind_dir, turbulence_intensity=0.06)

    config = FastFarmConfig(
        case_name=f"launcher_{args.controller}",
        num_turbines=case.n_turbines,
        xcoords=list(case.layout_x), ycoords=list(case.layout_y),
        dt=args.dt, max_iter=n_steps, wind=wind,
        output_dir=output_dir,
    )

    ff = ContinuousFastFarmInterface(config)
    ff.setup()       # generates case + deploys DISCON DLL + writes ROSCO params
    ff.reset(wind)   # configures InflowWind
    print(f"Case: {ff._farm_base}")

    # ── 2. Build controller ────────────────────────────────────────────
    floris_yaml = str(Path(__file__).resolve().parents[1] /
                      "cases/farm_2x3_4D/floris_case.yaml")
    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=floris_yaml,
                               prefer_floris=not args.no_floris)
    print(f"Surrogate backend: {surrogate.backend}")

    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    arbiter = CommandArbiter(yaw_deadband=0.5)

    mode = ControlMode(args.mode)
    if args.controller.lower() == "greedy":
        from closedloop.demos.greedy_baseline import GreedyController
        ctrl = GreedyController(case.n_turbines, args.dt)
    elif args.controller.lower() == "a":
        from closedloop.scheme_a import ControllerA
        ctrl = ControllerA(mode, surrogate, case.n_turbines)
    elif args.controller.lower() == "b":
        from closedloop.scheme_b import ControllerB
        ctrl = ControllerB(mode, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=args.seed)
    elif args.controller.lower() == "c":
        from closedloop.scheme_c import ControllerC
        ctrl = ControllerC(mode, surrogate, case.layout_x, case.layout_y,
                          case.n_turbines, seed=args.seed)

    # ── 3. Start FAST.Farm ─────────────────────────────────────────────
    print(f"Starting FAST.Farm ({args.duration}s, {n_steps} steps)...")
    ff.start()

    # ── 4. Closed-loop via WFCRL native interface ───────────────────────
    traj = Trajectory(controller=args.controller, mode=args.mode)
    prev_cmds = None

    # First-iteration state: no measurements yet; controller starts from defaults.
    meas: dict[int, TurbineMeas] = {}
    flow = sensing.estimate({})
    aborted = False

    try:
        for k in range(n_steps):
            step = k + 1
            # Check if FAST.Farm is still alive before sending commands
            if ff._process is not None and ff._process.poll() is not None:
                print(f"\nFAST.Farm exited early (rc={ff._process.returncode})"
                      f" — stopping at step {step}/{n_steps}")
                aborted = True
                break

            # Compute controller commands using LAST step's measurements
            cmds = ctrl.step(flow, meas)
            cmds = arbiter.apply_limits(cmds)

            # Convert to WFCRL ControlInput
            cin = _cmds_to_control_input(
                cmds, case.n_turbines, args.wind_speed, args.wind_dir)

            # Send to FAST.Farm via WFCRL interface
            try:
                output = ff.wait_step(cin)
            except Exception as e:
                # FAST.Farm finished or aborted — capture what we can
                print(f"\nFAST.Farm communication lost at step {step}: {e}")
                aborted = True
                break

            # Re-read until all N turbines report (WFCRL's poll exits on first match)
            _ensure_all_turbines_measured(output, ff, case.n_turbines, timeout=0.5)

            # Parse response — feeds NEXT iteration's controller step
            meas = _output_to_meas(output)
            sensing.update(meas)
            flow = sensing.estimate(meas)

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
            prev_cmds = cmds

            if k % max(1, n_steps // 10) == 0:
                pf = sum(m.genpwr_kw for m in meas.values()) if meas else 0.0
                print(f"  step {step:4d}/{n_steps}  P_farm={pf:8.1f} kW")

    finally:
        ff.stop()
        ff.close()

    # ── 5. Report ──────────────────────────────────────────────────────
    ev = Evaluator(transient_s=60.0)
    print(f"\nDone. Mean farm power (post-transient): {ev.mean_farm_power(traj):.1f} kW")
    print(f"To compare vs greedy, rerun with --controller greedy")

    out = Path(output_dir) / "trajectory.csv"
    traj.save_csv(out)
    print(f"Trajectory saved: {out}  ({'partial — FAST.Farm exited early' if aborted else 'complete'})")


if __name__ == "__main__":
    main()
