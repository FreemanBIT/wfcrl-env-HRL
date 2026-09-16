"""timestep_test.py — Timestep Compatibility Test（审查 P0-1，full-DOE gate）

背景：FAST.Farm DT_High=0.01 由集成 DLL 约束（E6a 不可改 0.05）；
本测试聚焦 **bts dt** 与入流组合的兼容性/结果稳定性：
  A: bts dt=0.10（当前默认，已验证）
  B: bts dt=0.05（需先生成短盒，见 --gen-box）
  C: bts dt=0.01（TurbSim 成本过高，标记 N/A，记录结论）

方法：1T / U=8 / TI=0.10 / 300s / baseline greedy；
记录 wall、mean P、T1 统计稳定性；与 A 档对比（期望几 % 内一致，wake 采样差异）。

用法：
  1) 生成 B 档短盒：
       python -m induction_vs_yaw_study.turbsim_inp.generate_wind_boxes \
           --seeds 1 --speeds 8 --tis 0.10 --out <study>/FarmInputs/timestep_test \
           --analysis-time 600 --time-step 0.05 --run --turbsim <exe>
  2) 跑测试：
       python -m induction_vs_yaw_study.run.timestep_test [--wind-time-series timestep_test/inflow_U08_TI10_seed01.bts]
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

from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind-time-series", required=True,
                    help="bts 相对路径（如 timestep_test/inflow_U08_TI10_seed01.bts）")
    ap.add_argument("--tag", default="ts")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--out", type=Path, default=_STUDY_ROOT / "results" / "_timestep")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    case = Case(8.0, 0.0, 0.10, 500.0, seed_id=1, n_turbines=1,
                control_dt_s=3.0, t_sim_s=args.duration, t_settle_s=0.0)
    t0 = time.time()
    out = replay_fastfarm_zmq(case, "baseline", {},
                              output_dir=str(args.out / args.tag),
                              wind_time_series=args.wind_time_series,
                              yaw_mode="static")
    wall = time.time() - t0
    if out is None or out.power_mw is None:
        print(f"[{args.tag}] FAIL: 无输出")
        return 1
    pw = np.asarray(out.power_mw, dtype=float)
    mean_kw = float(np.nanmean(pw.sum(axis=1))) * 1e3
    std_kw = float(np.nanstd(pw.sum(axis=1))) * 1e3
    print(f"[{args.tag}] PASS wall={wall:.0f}s steps={len(out.time)} "
          f"mean={mean_kw:.1f} kW std={std_kw:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
