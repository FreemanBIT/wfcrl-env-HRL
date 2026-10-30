
"""Comprehensive diagnostic: runs all test scenarios and collects per-step data"""
import sys, os, math, time, json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import induction_vs_yaw_study.run.verify_multiturbine_isolation as iso_mod
from pathlib import Path
_STUDY_ROOT = Path(iso_mod.__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

import numpy as np
from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds, _CMD_TTL_S
from wfcrl.config import WindConfig
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge


def run_scenario(name, cmds, env_extra=None):
    """Run one scenario and return per-step log."""
    print(f"\n=== {name} ===")
    
    case = Case(wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
                turbulence_intensity=0.10, spacing_D=6.0, seed_id=1,
                control_dt_s=3.0, t_sim_s=300.0, t_settle_s=0.0)
    out_dir = _STUDY_ROOT / "results" / (f"_full_diag_{name}_{os.getpid() % 10000}")
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = make_fastfarm_config(case, output_dir=str(out_dir))
    cfg.wind = WindConfig(speed=cfg.wind.speed, direction=cfg.wind.direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file="coarse/inflow_U08_TI10_seed01.bts")
    
    _base = 29000 + (os.getpid() % 1500) * 2
    engine = FastFarmZmqEngine(cfg, cmd_port=_base, state_port=_base + 1)
    engine.setup()
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception:
        pass
    
    if env_extra:
        os.environ.update(env_extra)
    
    t0 = time.time()
    engine.start()
    bridge = ZmqFarmBridge(engine, 3, wind_direction=float(cfg.wind.direction))
    
    while time.time() - t0 < 90:
        st = bridge.get_state(timeout_s=5.0)
        if st is not None and st.get("fast_step", 0) > 0:
            break
    
    seq_ctr = {"yaw": {}, "ind": {}}
    n_total = 100
    log = []  # (t, [(T1_h, T1_p), (T2_h, T2_p), (T3_h, T3_p)], [(T1_ys,T1_is), ...])
    
    for k in range(1, n_total + 1):
        bridge.wait_step(k, timeout=180.0)
        if k == 1 and cmds:
            _send_cmds(engine, cmds, seq_ctr)
        try:
            st = engine.get_state(timeout_s=5.0)
        except Exception:
            st = None
        if st:
            t = k * 3
            turbines = []
            seqs = []
            for i in range(3):
                ti = st["turbines"][i]
                h = ti.get("nacelle_heading_rad", 0)
                pw = ti.get("gen_power_w", 0)
                p = (pw if isinstance(pw, (int, float)) else pw.get("instant", 0)) / 1000.0
                ys = ti.get("yaw_seq_applied", 0)
                ins = ti.get("induction_seq_applied", 0)
                turbines.append((h, p))
                seqs.append((ys, ins))
            log.append((t, turbines, seqs))
    
    proc = getattr(engine, "_proc", None)
    if proc:
        try: proc.wait(timeout=600)
        except Exception: pass
    try: engine.stop()
    except Exception: pass
    try: engine.close()
    except Exception: pass
    
    # Summary
    t1_p = [p for t, tb, sq in log for i, (h, p) in enumerate(tb) if i == 0]
    t2_p = [p for t, tb, sq in log for i, (h, p) in enumerate(tb) if i == 1]
    t3_p = [p for t, tb, sq in log for i, (h, p) in enumerate(tb) if i == 2]
    print(f"  T1 avg={np.mean(t1_p):.0f} kW  std={np.std(t1_p):.0f}")
    print(f"  T2 avg={np.mean(t2_p):.0f} kW  std={np.std(t2_p):.0f}")
    print(f"  T3 avg={np.mean(t3_p):.0f} kW  std={np.std(t3_p):.0f}")
    
    # Final headings
    final_h = [math.degrees(tb[i][0]) for i in range(3) for t, tb, sq in log[-1:]]
    final_seqs = [sq[i] for i in range(3) for t, tb, sq in log[-1:]]
    print(f"  Final headings: T1={final_h[0]:+.1f}deg T2={final_h[1]:+.1f}deg T3={final_h[2]:+.1f}deg")
    print(f"  Final seqs: T1 yaw={final_seqs[0][0]} ind={final_seqs[0][1]}  "
          f"T2 yaw={final_seqs[1][0]} ind={final_seqs[1][1]}  "
          f"T3 yaw={final_seqs[2][0]} ind={final_seqs[2][1]}")
    
    return log


# Run all scenarios
results = {}

# 1. Baseline (no commands)
results["baseline"] = run_scenario("baseline", {})

# 2. Induction-only (T1 a=0.20)
results["induction"] = run_scenario("induction", {
    1: Cmd(turbine_id=1, yaw_delta_deg=None, induction_a=0.20),
})

# 3. Yaw-only (T1 +10deg)
results["yaw"] = run_scenario("yaw", {
    1: Cmd(turbine_id=1, yaw_delta_deg=10.0, induction_a=None),
})

# 4. Multi-turbine (T1 ind + T2 yaw)
results["multi"] = run_scenario("multi", {
    1: Cmd(turbine_id=1, yaw_delta_deg=None, induction_a=0.20),
    2: Cmd(turbine_id=2, yaw_delta_deg=10.0, induction_a=None),
})

# Save raw data
out_file = _STUDY_ROOT / "results" / f"_full_diag_data_{os.getpid() % 10000}.json"
with open(out_file, "w") as f:
    json.dump({k: [(t, tb, sq) for t, tb, sq in v] for k, v in results.items()}, f)
print(f"\nData saved to {out_file}")

# Same-run comparison analysis
print("\n" + "="*70)
print("SAME-RUN COMPARISON ANALYSIS")
print("="*70)

base = results["baseline"]
ind = results["induction"]
yaw = results["yaw"]
multi = results["multi"]

for scenario_name, scenario_data in [("induction", ind), ("yaw", yaw), ("multi", multi)]:
    print(f"\n--- {scenario_name.upper()} vs BASELINE ---")
    print(f"{'Time':>6} {'Base_T1':>8} {'Test_T1':>8} {'Diff':>8} {'%':>7} | {'Base_T2':>8} {'Test_T2':>8} {'Diff':>8} | {'Base_T3':>8} {'Test_T3':>8} {'Diff':>8}")
    
    for i in range(min(len(base), len(scenario_data))):
        t, base_tb, _ = base[i]
        _, test_tb, _ = scenario_data[i]
        
        bt1, bp1 = base_tb[0]
        tt1, tp1 = test_tb[0]
        bt2, bp2 = base_tb[1]
        tt2, tp2 = test_tb[1]
        bt3, bp3 = base_tb[2]
        tt3, tp3 = test_tb[2]
        
        d1 = tp1 - bp1
        pct1 = (tp1/bp1 - 1) * 100 if bp1 > 0 else 0
        d2 = tp2 - bp2
        d3 = tp3 - bp3
        
        if t % 30 == 0 or t <= 15:  # Print every 30s + first few
            print(f"{t:>5}s {bp1:>8.0f} {tp1:>8.0f} {d1:>+8.0f} {pct1:>+6.1f}% | {bp2:>8.0f} {tp2:>8.0f} {d2:>+8.0f} | {bp3:>8.0f} {tp3:>8.0f} {d3:>+8.0f}")
    
    # Average deltas
    d1_avg = np.mean([test_tb[0][1] - base_tb[0][1] for _, base_tb, _ in base for _, test_tb, _ in scenario_data])
    d2_avg = np.mean([test_tb[1][1] - base_tb[1][1] for _, base_tb, _ in base for _, test_tb, _ in scenario_data])
    d3_avg = np.mean([test_tb[2][1] - base_tb[2][1] for _, base_tb, _ in base for _, test_tb, _ in scenario_data])
    print(f"  Avg delta: T1={d1_avg:+.0f} kW  T2={d2_avg:+.0f} kW  T3={d3_avg:+.0f} kW")

print("\nDone.")
