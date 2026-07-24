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
from wfcrl.engine._outlist import _set_fast_scalar

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


def _ensure_all_turbines_measured(output, ff, n_turbines: int, timeout: float = 2.0):
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
            # Retry once if file was empty (mid-write race)
            if not vals:
                _time.sleep(0.01)
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


def _read_sim_time(ff) -> float:
    """Read current FAST.Farm simulation time from T1's measurement file."""
    import os as _os
    farm_base = getattr(ff, '_farm_base', None)
    if not farm_base:
        return -1.0
    vals = _parse_discon_file(_os.path.join(farm_base, 'measurements_T1.txt'))
    return vals.get('t', -1.0) if vals else -1.0


def _poll_sim_time_until(ff, target_t: float, timeout: float = 0.0) -> float:
    """Wait until FAST.Farm simulation time >= target_t. Returns actual time."""
    import time as _time
    t = _read_sim_time(ff)
    if t >= target_t:
        return t
    deadline = _time.time() + timeout if timeout > 0 else float('inf')
    while _time.time() < deadline:
        if getattr(ff, '_process', None) is not None and ff._process.poll() is not None:
            return t  # FAST.Farm died
        _time.sleep(0.05)
        t = _read_sim_time(ff)
        if t >= target_t:
            return t
    return t


def _unlock_yaw_dof(ff):
    """Revert _fix_initial_yaw: set YawDOF=True + YCMode=5 so DISCON yaw
    commands take effect. YCMode=0 (template default) ignores DLL yaw rate.
    NacYaw=0 is correct — wind-field rotation means 0° is aligned with +X flow."""
    import os as _os
    farm_base = getattr(ff, '_farm_base', None)
    fstf_file = getattr(ff, '_fstf_file', None)
    if not farm_base or not fstf_file:
        return
    try:
        from openfast_toolbox.io.fast_input_file import FASTInputFile
        fstf = FASTInputFile(fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            fst = FASTInputFile(str(_os.path.join(farm_base, wt_ref)))
            # YawDOF in ElastoDyn
            ed_rel = fst["EDFile"].replace('"', "")
            ed_path = _os.path.join(farm_base, ed_rel)
            if _os.path.exists(ed_path):
                _set_fast_scalar(ed_path, "YawDOF", "True")
            # YCMode in ServoDyn (must NOT be 0, or DLL yaw rate is ignored)
            servo_rel = fst["ServoFile"].replace('"', "")
            servo_path = _os.path.join(farm_base, servo_rel)
            if _os.path.exists(servo_path):
                _set_fast_scalar(servo_path, "YCMode", "5")
                _set_fast_scalar(servo_path, "TYCOn", "0.0")  # enable from t=0
    except Exception as e:
        print(f"  [warn] Could not unlock yaw DOF: {e}")


def _enable_per_turbine_out(ff):
    """Set OutFileFmt=3 in each turbine's .fst so per-turbine .out files are written."""
    import os as _os
    farm_base = getattr(ff, '_farm_base', None)
    fstf_file = getattr(ff, '_fstf_file', None)
    if not farm_base or not fstf_file:
        return
    try:
        from openfast_toolbox.io.fast_input_file import FASTInputFile
        fstf = FASTInputFile(fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            fst_path = _os.path.join(farm_base, wt_ref)
            if _os.path.exists(fst_path):
                _set_fast_scalar(fst_path, "OutFileFmt", "3")
    except Exception as e:
        print(f"  [warn] Could not enable per-turbine .out: {e}")


def _save_dtl_from_outb(output_dir, ff, n_turbines, dt_sample=1.0):
    """Parse FAST.Farm per-turbine .out files and save resampled data to CSV."""
    import os as _os, csv as _csv
    from pathlib import Path as _Path
    from wfcrl.engine._outb import _parse_all_outb, CHANNEL_TO_OUTPUT

    farm_base = getattr(ff, '_farm_base', None)
    fstf_file = getattr(ff, '_fstf_file', None)
    if not farm_base or not fstf_file:
        print("  [warn] No farm_base, cannot parse .out")
        return

    prefix = _os.path.splitext(_os.path.basename(fstf_file))[0]  # "Case"
    try:
        data = _parse_all_outb(farm_base, prefix, n_turbines)
    except Exception as e:
        print(f"  [warn] .out parse failed: {e}")
        return

    time_vec = data.get("time")
    if time_vec is None or len(time_vec) == 0:
        print("  [warn] No .out data found")
        return

    # Channels to extract and their CSV column prefixes
    channel_csv_map = {
        "power": "genpwr", "generator_torque": "gentq",
        "rotor_speed": "rotspd", "pitch": "blpitch",
        "yaw": "nacyaw", "wind_x": "wind_x",
        "blade_load_1": "mip1", "blade_load_2": "moop1",
        "blade_load_3": "mzb1",
    }

    # Build header
    header = ["t"]
    for col in channel_csv_map.values():
        for t_id in range(1, n_turbines + 1):
            header.append(f"{col}_T{t_id}")

    csv_path = _Path(output_dir) / "measurements_dtl.csv"
    t_start = time_vec[0]
    t_end = time_vec[-1]

    with open(csv_path, "w", newline="") as f:
        writer = _csv.writer(f)
        writer.writerow(header)

        t_sample = max(t_start, dt_sample * ((t_start + 0.001) // dt_sample + 1))
        while t_sample <= t_end + 1e-6:
            idx = int(np.searchsorted(time_vec, t_sample))
            idx = min(idx, len(time_vec) - 1)
            row = [t_sample]
            for ch_key, _col in channel_csv_map.items():
                arr = data.get(ch_key)
                if arr is not None and arr.shape[1] >= n_turbines:
                    for t_id in range(n_turbines):
                        row.append(arr[idx, t_id])
                else:
                    row.extend([0.0] * n_turbines)
            writer.writerow(row)
            t_sample += dt_sample

    print(f"DTL measurements saved: {csv_path}  ({time_vec[0]:.1f}s–{time_vec[-1]:.1f}s, "
          f"resampled to {dt_sample}s)")


def main():
    p = argparse.ArgumentParser(description="FAST.Farm + closed-loop launcher (WFCRL native)")
    p.add_argument("--duration", type=float, default=300)
    p.add_argument("--dt", type=float, default=10.0, help="FAST.Farm control step (s)")
    p.add_argument("--controller", default="greedy",
                   choices=["greedy", "A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1,2,3,4,5])
    p.add_argument("--wind-speed", type=float, default=10.0)
    p.add_argument("--wind-dir", type=float, default=270.0)
    p.add_argument("--bts", type=str,
                   default=r"D:\HR_Project\wfcrl-env-HRL\FarmInputs\inflow_10ms_TI05_s8D.bts",
                   help="TurbSim .bts wind file (absolute path avoids copy to FarmInputs/)")
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

    wind = WindConfig(wind_type=WindType.TURBSIM_BTS, speed=args.wind_speed,
                      direction=args.wind_dir, turbulence_intensity=0.06,
                      wind_file=args.bts)

    config = FastFarmConfig(
        case_name=f"launcher_{args.controller}",
        num_turbines=case.n_turbines,
        xcoords=list(case.layout_x), ycoords=list(case.layout_y),
        dt=args.dt, max_iter=n_steps, wind=wind,
        output_dir=output_dir,
    )

    ff = ContinuousFastFarmInterface(config)
    ff.setup()       # generates case + deploys DISCON DLL + writes ROSCO params
    _enable_per_turbine_out(ff)  # OutFileFmt=3 on each turbine for per-turbine .out
    _unlock_yaw_dof(ff)  # _fix_initial_yaw sets YawDOF=False for fixed-yaw LUT tests
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

    # Controls CSV: record yaw/power/pitch commands at each control step
    controls_csv = Path(output_dir) / "controls.csv"
    with open(controls_csv, 'w', newline='') as f:
        header = ['step', 't']
        for tid in range(1, case.n_turbines + 1):
            header.extend([f'yaw_{tid}', f'power_{tid}', f'pitch_{tid}'])
        import csv as _csv2
        _csv2.writer(f).writerow(header)

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

            # Record commands to controls.csv (at intended control time)
            ctrl_t_target = (k + 1) * args.dt
            with open(controls_csv, 'a', newline='') as f:
                row = [step, ctrl_t_target]
                for tid in range(case.n_turbines):
                    row.extend([
                        cin.yaw[tid] if tid < len(cin.yaw) else 0.0,
                        cin.power[tid] if cin.power is not None and tid < len(cin.power) else 0.0,
                        cin.pitch[tid] if tid < len(cin.pitch) else 0.0,
                    ])
                import csv as _csv3
                _csv3.writer(f).writerow(row)

            # Wait until FAST.Farm simulation time reaches the control boundary,
            # then dispatch commands exactly on schedule.  If computation
            # took too long and we missed the boundary, skip to the next one.
            sim_t = _read_sim_time(ff)
            if sim_t > ctrl_t_target + args.dt * 0.5:
                skipped = int((sim_t - ctrl_t_target) / args.dt) + 1
                new_target = ctrl_t_target + skipped * args.dt
                print(f"  [warn] computation late (sim t={sim_t:.1f}s, "
                      f"target {ctrl_t_target:.0f}s) — resetting to t={new_target:.0f}s")
                ctrl_t_target = new_target
                # Update controls.csv row
                import csv as _csv4
                with open(controls_csv, 'r') as f:
                    rows = list(_csv4.reader(f))
                if rows:
                    rows[-1][1] = str(int(ctrl_t_target))
                    with open(controls_csv, 'w', newline='') as f:
                        _csv4.writer(f).writerows(rows)

            sim_t = _poll_sim_time_until(ff, ctrl_t_target)
            if sim_t < ctrl_t_target - 0.5:
                print(f"\nFAST.Farm stalled at t={sim_t:.1f}s (target {ctrl_t_target})")
                aborted = True
                break

            # Send to FAST.Farm via WFCRL interface
            try:
                output = ff.wait_step(cin)
            except Exception as e:
                # FAST.Farm finished or aborted — capture what we can
                print(f"\nFAST.Farm communication lost at step {step}: {e}")
                aborted = True
                break

            # Re-read until all N turbines report (WFCRL's poll exits on first match)
            _ensure_all_turbines_measured(output, ff, case.n_turbines, timeout=2.0)

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

    # ── Extract DT_low measurements from .out files → measurements_dtl.csv ──
    _save_dtl_from_outb(output_dir, ff, case.n_turbines)

    # ── 5. Report ──────────────────────────────────────────────────────
    ev = Evaluator(transient_s=60.0)
    print(f"\nDone. Mean farm power (post-transient): {ev.mean_farm_power(traj):.1f} kW")
    print(f"To compare vs greedy, rerun with --controller greedy")

    out = Path(output_dir) / "trajectory.csv"
    traj.save_csv(out)
    print(f"Trajectory saved: {out}  ({'partial — FAST.Farm exited early' if aborted else 'complete'})")
    print(f"Controls saved: {controls_csv}")


if __name__ == "__main__":
    main()
