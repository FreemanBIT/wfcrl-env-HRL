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


def _mean_farm_power(out) -> tuple:
    """返回 (farm_kw, [t1_kw, t2_kw, t3_kw])。"""
    if out is None or out.power_mw is None:
        return float("nan"), [float("nan")] * 3
    pmw = np.asarray(out.power_mw, dtype=float)
    if pmw.ndim == 1:
        pmw = pmw.reshape(-1, 1)
    farm = float(np.nanmean(pmw.sum(axis=1))) * 1e3
    per_t = [float(np.nanmean(pmw[:, i])) * 1e3 for i in range(min(3, pmw.shape[1]))]
    return farm, per_t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind-time-series", default="coarse/inflow_U08_TI10_seed01.bts")
    ap.add_argument("--out", type=Path, default=_STUDY_ROOT / "results" / "_identity")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--wd", type=float, default=0.0,
                    help="WD offset（sanity/mirror 用 -10/0/10；盒为 (U,TI,seed) 键共享）")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    steps = int(round(args.duration / args.dt))
    base_case = Case(8.0, args.wd, 0.10, 4.0, seed_id=1,
                     control_dt_s=args.dt, t_sim_s=args.duration, t_settle_s=0.0)

    greedy_mw = _greedy_mw(8.0)

    def run(tag, control_type, settings):
        t0 = time.time()
        out = replay_fastfarm_zmq(base_case, control_type, settings,
                                  output_dir=str(args.out / (tag + f"_wd{args.wd:g}")),
                                  wind_time_series=args.wind_time_series,
                                  yaw_mode="static")
        farm, per_t = _mean_farm_power(out)
        print(f"  {tag:10s} farm={farm:8.1f} T={['%.1f' % v for v in per_t]}"
              f"  wall={time.time()-t0:.0f}s", flush=True)
        return farm, per_t

    print(f"identity checks @ U8/TI0.10/seed1/WD{args.wd:g}/S4D ...")
    p_base0, t_base0 = run("baseline_0", "baseline", {})
    p_base1, t_base1 = run("baseline_1", "baseline", {})
    p_yaw0, t_yaw0 = run("yaw0", "yaw",
                         {1: {"yaw_deg": 0.0}, 2: {"yaw_deg": 0.0}, 3: {"yaw_deg": 0.0}})
    p_ind1, t_ind1 = run("induction_r1", "derating",
                         {1: {"power_target_mw": greedy_mw},
                          2: {"power_target_mw": greedy_mw},
                          3: {"power_target_mw": greedy_mw}})

    def diff(a, b):
        return float("nan") if not b else 100.0 * (a - b) / b

    # farm + T1..3 分别校验（审查 §6.3）
    rows = [
        ("baseline_repeat", p_base1, t_base1, diff(p_base1, p_base0),
         [diff(t_base1[i], t_base0[i]) for i in range(3)]),
        ("zero_yaw", p_yaw0, t_yaw0, diff(p_yaw0, p_base0),
         [diff(t_yaw0[i], t_base0[i]) for i in range(3)]),
        ("induction_r1", p_ind1, t_ind1, diff(p_ind1, p_base0),
         [diff(t_ind1[i], t_base0[i]) for i in range(3)]),
    ]
    import csv as _csv
    with open(args.out / ("identity_results_wd%s.csv" % str(args.wd).lstrip("-").replace(".0", "")),
              "w", newline="", encoding="utf-8") as f:
        w = _csv.writer(f)
        w.writerow(["wd", "check", "farm_kw", "farm_diff_pct", "t1_diff_pct",
                    "t2_diff_pct", "t3_diff_pct", "pass"])
        ok_all = True
        for name, farm, per_t, d_farm, d_t in rows:
            ok = abs(d_farm) < TOL_PCT and all(abs(x) < TOL_PCT for x in d_t)
            ok_all &= ok
            print(f"  {name:16s} farm_diff={d_farm:+.3f}% T1/2/3=["
                  + ",".join(f"{x:+.2f}" for x in d_t) + f"]  [{'PASS' if ok else 'FAIL'}]")
            w.writerow([args.wd, name, f"{farm:.1f}", f"{d_farm:.4f}",
                        ",".join(f"{x:.4f}" if x == x else "" for x in d_t), ok])
    print(f"IDENTITY: {'ALL PASS' if ok_all else 'SOME FAILED'}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
