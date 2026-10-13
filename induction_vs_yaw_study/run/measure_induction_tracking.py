"""measure_induction_tracking.py — M6 前置：诱导因子 1s 跟踪测量（DR-9）

在 v2.10.6 链路上测诱导命令阶跃的动态响应：
  阶段1（t=0..120s）：T1 无命令（greedy）
  阶段2（t=120s 起）：下发 induction a=0.20（seq=1）
测量：功率/转速从阶跃时刻到新稳态的沉降时间、超调、以及状态帧观测到的
      induction_seq_applied 确认时间（DR-9：不得仅凭命令下发成功认定达成）。

用法：
    python -m induction_vs_yaw_study.run.measure_induction_tracking [--duration 300] [--step-at 120]
"""
from __future__ import annotations

import argparse
import math as _math
import os as _os
import sys as _sys
import time as _time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
_os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))
_ccc = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..", "..", "closed-loop-control"))
if _ccc not in _sys.path:
    _sys.path.insert(0, _ccc)

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds, _CMD_TTL_S

from wfcrl.config import WindConfig
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge


def main():
    ap = argparse.ArgumentParser(description="诱导因子 1s 跟踪测量")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--step-at", type=float, default=120.0, help="诱导阶跃时刻 (s)")
    ap.add_argument("--a", type=float, default=0.20, help="诱导目标 a")
    ap.add_argument("--out-root", type=Path, default=_STUDY_ROOT / "results" / "_tracking")
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
    n_turbines = int(getattr(case, "n_turbines", 3))
    n_total = max(1, int(getattr(case, "n_total_control_steps", cfg.max_iter)))
    step_step = max(1, int(round(args.step_at / args.dt)))

    print(f"== induction tracking: a={args.a} step@{args.step_at}s "
          f"(step {step_step}/{n_total}), dur={args.duration}s")

    base = 24000 + (_os.getpid() % 1500) * 2
    engine = FastFarmZmqEngine(cfg, cmd_port=base, state_port=base + 1)
    engine.setup()
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception as _e:
        print(f"[warn] load channels: {_e}")
    _omp = _os.environ.get("WFCRL_STUDY_OMP_NUM_THREADS", "1")
    if _omp not in ("", "0"):
        _os.environ["OMP_NUM_THREADS"] = _omp

    t0 = _time.time()
    engine.start()
    bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=float(cfg.wind.direction))
    rows = []
    try:
        br_ok = False
        while _time.time() - t0 < 90:
            st = bridge.get_state(timeout_s=5.0)
            if st is not None and st.get("fast_step", 0) > 0:
                br_ok = True
                break
        if not br_ok:
            raise TimeoutError("FAST.Farm 未在 90s 内发出状态帧")
        seq_ctr = {"yaw": {}, "ind": {}}
        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=180.0)
            if k == step_step:
                # 阶跃：T1 下发诱导 a
                cmds = {1: Cmd(turbine_id=1, yaw_delta_deg=None, induction_a=args.a)}
                _send_cmds(engine, cmds, seq_ctr)
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            rows.append(st if st else {"turbines": []})
        _proc = getattr(engine, "_proc", None)
        if _proc is not None:
            try:
                _proc.wait(timeout=600)
            except Exception:
                pass
    finally:
        try:
            engine.stop()
        except Exception:
            pass
        try:
            engine.close()
        except Exception:
            pass

    # ---- 提取 T1 时序 ----
    ts = []
    pw = []
    spd = []
    seq = []
    for st in rows:
        tt = (st or {}).get("sim_time_s")
        t1 = next((t for t in (st or {}).get("turbines", []) if t.get("turbine_id") == 1), None)
        if tt is None or t1 is None:
            continue
        ts.append(float(tt))
        pw.append(float(t1.get("gen_power_w", float("nan")) or float("nan")) / 1e3)
        spd.append(float(t1.get("rotor_speed_rad_s", float("nan")) or float("nan")))
        seq.append(int(t1.get("induction_seq_applied", 0) or 0))
    ts = np.array(ts); pw = np.array(pw); spd = np.array(spd); seq = np.array(seq)

    # ---- 稳态窗计算 ----
    def mean_window(a, t0s, t1s):
        m = (ts >= t0s) & (ts <= t1s) & np.isfinite(a)
        return float(np.mean(a[m])) if m.sum() > 0 else float("nan")

    p_before = mean_window(pw, 30.0, args.step_at - 15)
    p_after = mean_window(pw, args.step_at + 60, args.duration - 10)
    spd_before = mean_window(spd, 30.0, args.step_at - 15)
    spd_after = mean_window(spd, args.step_at + 60, args.duration - 10)
    derate_pct = 100.0 * (p_after - p_before) / p_before if p_before and p_before > 0 else float("nan")

    # ---- 沉降时间：功率进入新稳态 ±2% 带且不再离开 ----
    settle_s = float("nan")
    if p_after == p_after:
        band = 0.02 * p_after
        first = None
        for i in range(len(ts)):
            if ts[i] < args.step_at:
                continue
            if abs(pw[i] - p_after) <= band:
                if first is None:
                    first = i
            elif first is not None:
                first = None
        if first is not None:
            settle_s = ts[first] - args.step_at

    # ---- seq 确认时间 ----
    seq_confirm_s = float("nan")
    for i in range(len(ts)):
        if seq[i] >= 1:
            seq_confirm_s = ts[i] - args.step_at
            break

    print(f"  T1 before: P={p_before:.0f} kW  spd={spd_before:.3f} rad/s")
    print(f"  T1 after : P={p_after:.0f} kW  spd={spd_after:.3f} rad/s  derate={derate_pct:.1f}%")
    print(f"  seq confirm: {seq_confirm_s:.1f}s after step  |  power settle: {settle_s:.1f}s")

    import json
    (out_dir / "tracking_summary.json").write_text(json.dumps({
        "case": case.id, "a": args.a, "step_at_s": args.step_at,
        "p_before_kw": p_before, "p_after_kw": p_after, "derate_pct": derate_pct,
        "spd_before": spd_before, "spd_after": spd_after,
        "seq_confirm_s": seq_confirm_s, "power_settle_s": settle_s,
    }, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---- 判定（DR-9：以实际状态测量为准）----
    fails = []
    if not (p_before == p_before and p_after == p_after):
        fails.append("功率数据缺失")
    else:
        if derate_pct > -6.0:
            fails.append(f"诱导降额不足: {derate_pct:.1f}% (期望 <= -6%)")
        if settle_s == settle_s and settle_s > 90.0:
            fails.append(f"功率沉降过慢: {settle_s:.0f}s (期望 <= 90s)")
    if fails:
        print("[FAIL] " + " | ".join(fails))
        return 1
    print("[PASS] 诱导跟踪测量完成（降额生效 + 沉降可接受）")
    return 0


if __name__ == "__main__":
    _sys.exit(main())
