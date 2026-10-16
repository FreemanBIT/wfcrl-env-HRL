
"""Functional test: Yaw channel only (T1=+10deg, T2/T3=no command)"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from induction_vs_yaw_study.run.verify_multiturbine_isolation import main as iso_main

if __name__ == "__main__":
    import argparse
    import math as _math
    import os as _os
    import time as _time
    from pathlib import Path
    
    import induction_vs_yaw_study.run.verify_multiturbine_isolation as iso_mod
    _STUDY_ROOT = Path(iso_mod.__file__).resolve().parents[1]
    _os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))
    
    import numpy as np
    from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
    from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds, _CMD_TTL_S
    from wfcrl.config import WindConfig
    from wfcrl.config.types import WindType
    from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
    from closedloop.types import Cmd
    from closedloop.bridge import ZmqFarmBridge
    
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--yaw-deg", type=float, default=10.0)
    ap.add_argument("--out-root", type=Path, default=_STUDY_ROOT / "results" / "_yaw_test")
    ap.add_argument("--bts", default="coarse/inflow_U08_TI10_seed01.bts")
    args = ap.parse_args()
    
    from induction_vs_yaw_study.run._proc_utils import cleanup_stale_fastfarm
    cleanup_stale_fastfarm(min_age_s=300.0)
    
    case = Case(wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
                turbulence_intensity=0.10, spacing_D=6.0, seed_id=1,
                control_dt_s=args.dt, t_sim_s=args.duration, t_settle_s=0.0)
    out_dir = args.out_root / _time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = make_fastfarm_config(case, output_dir=str(out_dir))
    cfg.wind = WindConfig(speed=cfg.wind.speed, direction=cfg.wind.direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=args.bts)
    n_turbines = 3
    n_total = max(1, int(getattr(case, "n_total_control_steps", cfg.max_iter)))
    
    # Yaw-only: T1=yaw_deg, T2/T3=no command
    cmds = {1: Cmd(turbine_id=1, yaw_delta_deg=args.yaw_deg, induction_a=None)}
    print(f"== YAW TEST: T1 yaw=+{args.yaw_deg}deg, T2/T3 no command")
    
    _base = 26000 + (_os.getpid() % 1500) * 2
    engine = FastFarmZmqEngine(cfg, cmd_port=_base, state_port=_base + 1)
    engine.setup()
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception:
        pass
    
    t0 = _time.time()
    engine.start()
    bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=float(cfg.wind.direction))
    state_rows = []
    try:
        br_ok = False
        while _time.time() - t0 < 90:
            st = bridge.get_state(timeout_s=5.0)
            if st is not None and st.get("fast_step", 0) > 0:
                br_ok = True
                break
        if not br_ok:
            raise TimeoutError("FAST.Farm no state frame in 90s")
        
        seq_ctr = {"yaw": {}, "ind": {}}
        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=180.0)
            if k == 1:
                _send_cmds(engine, cmds, seq_ctr)
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            state_rows.append(st if st else {"turbines": []})
        
        _proc = getattr(engine, "_proc", None)
        if _proc is not None:
            try:
                _proc.wait(timeout=600)
            except Exception:
                pass
    finally:
        try: engine.stop()
        except Exception: pass
        try: engine.close()
        except Exception: pass
    
    wall = _time.time() - t0
    pw = iso_mod._per_turbine_power(state_rows, n_turbines)
    stt = iso_mod._per_turbine_state(state_rows, n_turbines)
    print(f"wall={wall:.0f}s  T-power(kW): T1={pw.get(1):.0f} T2={pw.get(2):.0f} T3={pw.get(3):.0f}")
    for tid in range(1, n_turbines + 1):
        s = stt.get(tid, {})
        print(f"  T{tid}: heading={s.get('nacelle_heading_rad')} yaw_seq={s.get('yaw_seq_applied')} ind_seq={s.get('induction_seq_applied')}")
    
    import json
    (out_dir / "yaw_summary.json").write_text(json.dumps({
        "case": case.id, "yaw_deg": args.yaw_deg, "wall_s": wall,
        "turbine_power_kw": {str(k): v for k, v in pw.items()},
        "turbine_state": {str(k): v for k, v in stt.items()},
    }, indent=2, ensure_ascii=False), encoding="utf-8")
