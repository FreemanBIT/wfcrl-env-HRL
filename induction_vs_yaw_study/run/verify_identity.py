"""verify_identity.py — 控制 identity 校验（方案 V2 §21.2/§20.2/§31.6-7）

在 (U=8, TI=0.10, seed=1, WD=0, S=4D) 上跑：
  * baseline 两次（可重复性）
  * yaw=[0,0,0] vs baseline（zero-yaw identity）
  * induction r=1（a=power_to_induction(贪婪功率)≈1/3）vs baseline
验收：farm 均值功率差 < 0.5%（默认）。

用法：
    python -m induction_vs_yaw_study.run.verify_identity
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

import numpy as np

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq

TOL_PCT = 0.5  # 验收阈值


def _greedy_mw(wind_speed_ms: float) -> float:
    """Betz 贪婪电功率 (MW)，用于 r=1 的 induction 目标（a≈1/3）。"""
    p_w = C.power_from_cp(C.cp_max_betz(), wind_speed_ms, electrical=True)
    return p_w / 1e6


def _mean_farm_power(out) -> float:
    if out is None or out.power_mw is None:
        return float("nan")
    pmw = np.asarray(out.power_mw, dtype=float)
    farm = pmw.sum(axis=1) if pmw.ndim > 1 else pmw
    return float(np.nanmean(farm)) * 1e3  # MW -> kW


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind-time-series", default="coarse/inflow_U08_TI10_seed01.bts")
    ap.add_argument("--out", type=Path, default=_STUDY_ROOT / "results" / "_identity")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    steps = int(round(args.duration / args.dt))
    base_case = Case(8.0, 0.0, 0.10, 4.0, seed_id=1,
                     fastfarm_dt=args.dt, fastfarm_t_settle_s=0.0,
                     fastfarm_n_control_steps=steps)

    greedy_mw = _greedy_mw(8.0)

    def run(tag, control_type, settings):
        t0 = time.time()
        out = replay_fastfarm_zmq(base_case, control_type, settings,
                                  output_dir=str(args.out / tag),
                                  wind_time_series=args.wind_time_series,
                                  yaw_mode="static")
        p = _mean_farm_power(out)
        print(f"  {tag:10s} farm={p:8.1f} kW  wall={time.time()-t0:.0f}s", flush=True)
        return p

    print("identity checks @ U8/TI0.10/seed1/WD0/S4D ...")
    p_base0 = run("baseline_0", "baseline", {})
    p_base1 = run("baseline_1", "baseline", {})
    p_yaw0 = run("yaw0", "yaw",
                 {1: {"yaw_deg": 0.0}, 2: {"yaw_deg": 0.0}, 3: {"yaw_deg": 0.0}})
    p_ind1 = run("induction_r1", "derating",
                 {1: {"power_target_mw": greedy_mw},
                  2: {"power_target_mw": greedy_mw},
                  3: {"power_target_mw": greedy_mw}})

    def diff(a, b):
        return float("nan") if not b else 100.0 * (a - b) / b

    rows = [
        ("baseline_repeat", p_base1, diff(p_base1, p_base0)),
        ("zero_yaw", p_yaw0, diff(p_yaw0, p_base0)),
        ("induction_r1", p_ind1, diff(p_ind1, p_base0)),
    ]
    import csv as _csv
    with open(args.out / "identity_results.csv", "w", newline="", encoding="utf-8") as f:
        w = _csv.writer(f)
        w.writerow(["check", "farm_kw", "diff_vs_baseline_pct", "pass"])
        ok_all = True
        for name, p, d in rows:
            ok = abs(d) < TOL_PCT
            ok_all &= ok
            print(f"  {name:18s} diff={d:+.3f}%  [{'PASS' if ok else 'FAIL'}]")
            w.writerow([name, f"{p:.1f}" if p == p else "", f"{d:.4f}" if d == d else "", ok])
    print(f"IDENTITY: {'ALL PASS' if ok_all else 'SOME FAILED'} -> {args.out / 'identity_results.csv'}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
