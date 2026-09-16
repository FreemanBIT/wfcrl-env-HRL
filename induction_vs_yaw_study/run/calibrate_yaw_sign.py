"""calibrate_yaw_sign.py — 偏航符号标定（方案 V2 §20.1 / 开发计划 §7.3）

固定 (U=8, TI=0.05 最低湍流盒, S=4D, WD=10°)，对上游机 T1/T2 下发
yaw ∈ {-20, 0, +20}（T3 固定 0），测 farm/T1..T3 功率与 WT2 入流，
明确 positive yaw 在 FAST.Farm 中把尾流偏向哪一侧（与 FLORIS convention 对齐）。

注意：方案 V2 的"确定性"测试因 v5 稳态风硬墙改用 TI=0.05 盒近似（开发计划 §13.1）。

用法：
    python -m induction_vs_yaw_study.run.calibrate_yaw_sign
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq


def _agg(out):
    """返回 dict(farm_kw, t1..t3_kw, wt2_wind)。"""
    if out is None or out.power_mw is None:
        return None
    pmw = np.asarray(out.power_mw, dtype=float)
    farm = pmw.sum(axis=1) if pmw.ndim > 1 else pmw
    r = {"farm_kw": float(np.nanmean(farm)) * 1e3}
    for i in range(3):
        r[f"t{i+1}_kw"] = float(np.nanmean(pmw[:, i])) * 1e3 if pmw.ndim > 1 else float("nan")
    if out.wind_speed is not None and np.asarray(out.wind_speed).ndim == 2 and             np.asarray(out.wind_speed).shape[1] >= 2:
        r["wt2_wind"] = float(np.nanmean(np.asarray(out.wind_speed)[:, 1]))
    else:
        r["wt2_wind"] = float("nan")
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind-time-series", default="coarse/inflow_U08_TI05_seed01.bts")
    ap.add_argument("--wd", type=float, default=10.0)
    ap.add_argument("--spacing", type=float, default=4.0)
    ap.add_argument("--out", type=Path, default=_STUDY_ROOT / "results" / "_calib")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--yaws", nargs="+", type=float, default=[-20.0, 0.0, 20.0])
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    steps = int(round(args.duration / args.dt))
    case = Case(8.0, args.wd, 0.05, args.spacing, seed_id=1,
                control_dt_s=args.dt, t_sim_s=args.duration, t_settle_s=0.0)
    print(f"yaw sign calibration @ U8/TI0.05/seed1/WD+{args.wd:g}/S{args.spacing:g}D")

    rows = []
    for y in args.yaws:
        t0 = time.time()
        settings = {1: {"yaw_deg": y}, 2: {"yaw_deg": y}, 3: {"yaw_deg": 0.0}}
        out = replay_fastfarm_zmq(case, "yaw", settings,
                                  output_dir=str(args.out / f"yaw{int(y):+d}"),
                                  wind_time_series=args.wind_time_series,
                                  yaw_mode="static")
        a = _agg(out) or {}
        rows.append({"yaw_deg": y, **a, "wall_s": round(time.time() - t0)})
        print(f"  yaw {y:+5.1f}° -> " + ", ".join(f"{k}={v:.2f}" for k, v in a.items())
              + f"  wall={time.time()-t0:.0f}s", flush=True)

    # 按 WD 分文件（多 WD 标定时不得互相覆盖）
    wd_tag = str(args.wd).replace("-", "m").replace(".0", "")
    out_csv = args.out / f"yaw_sign_calibration_wd{wd_tag}.csv"
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["wd", "yaw_deg", "farm_kw", "t1_kw",
                                          "t2_kw", "t3_kw", "wt2_wind", "wall_s"])
        w.writeheader()
        for r_ in rows:
            w.writerow({"wd": args.wd, **r_})
    print(f"-> {out_csv}")


if __name__ == "__main__":
    sys.exit(main())
