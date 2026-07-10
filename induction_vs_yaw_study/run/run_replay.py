"""
run_replay.py — 回放 LUT (Stage 5/6 入口)
=========================================
对每个工况 × {yaw, derating, baseline}，在指定仿真器中回放 LUT，存时序到
results/timeseries/{sim}_{control}_{case_id}.csv。

支持：断点续跑（已存在产物跳过）、子集运行、失败日志。

用法：
    python -m induction_vs_yaw_study.run.run_replay --sim floris
    python -m induction_vs_yaw_study.run.run_replay --sim fastfarm --subset
    python -m induction_vs_yaw_study.run.run_replay --sim fastfarm --controls derating baseline
"""

from __future__ import annotations

import argparse
import os
import time
import traceback
from pathlib import Path

import pandas as pd

from induction_vs_yaw_study.cases.three_nrel5mw import iter_cases, load_grid
from induction_vs_yaw_study.lut.schema import load_lut
from induction_vs_yaw_study.lut.interpolate import _rows_to_dict
from induction_vs_yaw_study.replay.floris_replay import replay_floris
from induction_vs_yaw_study.replay.fastfarm_replay import replay_fastfarm


def _dirs():
    root = Path(__file__).resolve().parents[2]
    ts = root / "results" / "timeseries"
    ts.mkdir(parents=True, exist_ok=True)
    return root, ts


def _settings_for(lut: pd.DataFrame, case_id: str, control_type: str):
    """从 LUT 取某工况某控制类型的三机设定 dict。"""
    if control_type == "baseline":
        # baseline 不需要 LUT 行；返回空 → schedule 用全零
        return {i: {"yaw_deg": 0.0, "power_target_mw": float("nan"),
                    "ratio": 1.0, "min_pitch_deg": 0.0,
                    "greedy_power_w": float("nan")} for i in (1, 2, 3)}
    rows = lut[(lut["case_id"] == case_id) & (lut["control_type"] == control_type)]
    if rows.empty:
        raise ValueError(f"No LUT rows for {case_id}/{control_type}")
    return _rows_to_dict(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True, choices=["floris", "fastfarm"])
    ap.add_argument("--subset", action="store_true")
    ap.add_argument("--controls", nargs="+",
                    default=["baseline", "yaw", "derating"],
                    choices=["baseline", "yaw", "derating"])
    ap.add_argument("--grid", default=None)
    ap.add_argument("--lut", default=None, help="LUT 路径，默认 results/luts/luts.parquet")
    ap.add_argument("--overwrite", action="store_true", help="覆盖已存在产物")
    args = ap.parse_args()

    root, ts_dir = _dirs()
    grid = load_grid(args.grid)
    lut_path = args.lut or str(root / "results" / "luts" / "luts.parquet")
    lut = load_lut(lut_path)

    cases = list(iter_cases(grid, subset=args.subset))
    run_log = []
    print(f"Replaying {args.sim} for {len(cases)} cases × {args.controls}")

    for k, case in enumerate(cases, 1):
        for control in args.controls:
            out_csv = ts_dir / f"{args.sim}_{control}_{case.id}.csv"
            if out_csv.exists() and not args.overwrite:
                print(f"  [skip] {out_csv.name} exists")
                continue

            t0 = time.time()
            status, err = "ok", ""
            try:
                settings = _settings_for(lut, case.id, control)
                out_dir = str(root / "results" / "_work" / args.sim /
                              f"{control}_{case.id}")
                if args.sim == "floris":
                    out = replay_floris(case, control, settings, output_dir=out_dir)
                else:
                    out = replay_fastfarm(case, control, settings, output_dir=out_dir)

                # 即使 FAST.Farm 中途 abort，replay 也会返回**部分**结果（或 None）。
                # 只要有数据就写 CSV，保证不丢已跑出的部分。
                if out is not None and out.time is not None and len(out.time) > 0:
                    out.to_csv(str(out_csv))
                    if (out.metadata or {}).get("aborted"):
                        status, err = "partial", "FAST.Farm aborted mid-run; partial CSV saved"
                        print(f"  [PARTIAL] {case.id}/{control}: 已保存部分结果 {out_csv.name}")
                else:
                    status, err = "fail", "no output produced"
                    print(f"  [FAIL] {case.id}/{control}: 无输出")

                # 仿真完成后立即清理 _work 中的 .bts 入流风文件（节省磁盘）
                if args.sim == "fastfarm":
                    import glob as _glob
                    for bts in _glob.glob(os.path.join(out_dir, "FarmInputs", "*.bts")):
                        try:
                            os.remove(bts)
                        except OSError:
                            pass
            except Exception as e:  # noqa
                status, err = "fail", repr(e)
                print(f"  [FAIL] {case.id}/{control}: {e}")
                traceback.print_exc()

            dt = time.time() - t0
            print(f"  [{k}/{len(cases)}] {args.sim}/{control}/{case.id} "
                  f"-> {status} ({dt:.1f}s)")
            run_log.append({
                "sim": args.sim, "case_id": case.id, "control": control,
                "status": status, "seconds": dt, "error": err,
            })

    log_path = root / "results" / f"run_log_{args.sim}.csv"
    pd.DataFrame(run_log).to_csv(log_path, index=False)
    print(f"\nRun log: {log_path}")


if __name__ == "__main__":
    main()
