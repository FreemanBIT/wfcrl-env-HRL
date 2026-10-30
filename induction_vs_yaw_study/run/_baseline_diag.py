
"""Baseline diagnostic: T1 NO command, log power/heading at each step"""
import sys, os, math, time
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

case = Case(wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
            turbulence_intensity=0.10, spacing_D=6.0, seed_id=1,
            control_dt_s=3.0, t_sim_s=300.0, t_settle_s=0.0)
out_dir = _STUDY_ROOT / "results" / ("_baseline_diag_" + str(os.getpid() % 10000))
out_dir.mkdir(parents=True, exist_ok=True)
cfg = make_fastfarm_config(case, output_dir=str(out_dir))
cfg.wind = WindConfig(speed=cfg.wind.speed, direction=cfg.wind.direction,
                      wind_type=WindType.TURBSIM_BTS, wind_file="coarse/inflow_U08_TI10_seed01.bts")

# NO COMMANDS - pure baseline
cmds = {}
print("Baseline diagnostic: NO commands, logging T1 power/heading")

_base = 28000 + (os.getpid() % 1500) * 2
engine = FastFarmZmqEngine(cfg, cmd_port=_base, state_port=_base + 1)
engine.setup()
try:
    from induction_vs_yaw_study.run.add_load_channels import add_load_channels
    _ff_base = getattr(engine._ff, "_farm_base", None)
    if _ff_base:
        add_load_channels(_ff_base, verbose=False)
except Exception:
    pass

t0 = time.time()
engine.start()
bridge = ZmqFarmBridge(engine, 3, wind_direction=float(cfg.wind.direction))

while time.time() - t0 < 90:
    st = bridge.get_state(timeout_s=5.0)
    if st is not None and st.get("fast_step", 0) > 0:
        break

n_total = 100
log = []
for k in range(1, n_total + 1):
    bridge.wait_step(k, timeout=180.0)
    try:
        st = engine.get_state(timeout_s=5.0)
    except Exception:
        st = None
    if st:
        t1 = st["turbines"][0]
        h = t1.get("nacelle_heading_rad", 0)
        pw = t1.get("gen_power_w", 0)
        p = (pw if isinstance(pw, (int, float)) else pw.get("instant", 0)) / 1000.0
        log.append((k * 3, h, p))
        if k % 10 == 0 or k <= 5:
            print(f"  t={k*3:3d}s heading={h:+.4f} rad ({math.degrees(h):+6.1f} deg)  P={p:.0f} kW")

proc = getattr(engine, "_proc", None)
if proc:
    try: proc.wait(timeout=600)
    except Exception: pass
try: engine.stop()
except Exception: pass
try: engine.close()
except Exception: pass

print(f"\nBaseline summary:")
if log:
    early = [p for t,h,p in log if t <= 30]
    mid = [p for t,h,p in log if 120 <= t <= 180]
    late = [p for t,h,p in log if t >= 270]
    all_p = [p for t,h,p in log]
    print(f"  Avg P t=0-30s:    {np.mean(early):.0f} kW")
    print(f"  Avg P t=120-180s: {np.mean(mid):.0f} kW")
    print(f"  Avg P t=270-300s: {np.mean(late):.0f} kW")
    print(f"  Avg P full run:   {np.mean(all_p):.0f} kW")
    print(f"  Std P:            {np.std(all_p):.0f} kW")
    print(f"  Min/Max P:        {min(all_p):.0f} / {max(all_p):.0f} kW")
