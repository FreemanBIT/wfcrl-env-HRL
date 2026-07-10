"""
run_replay_steady.py — 稳态风快速回放入口
==========================================
对每个稳态工况 × {baseline, yaw, derating}，在指定仿真器中回放控制，
时序存到 results/timeseries_steady/{sim}_{control}_{case_id}.csv。

**复用已建好的 FLORIS LUT**：稳态工况无真实 TI，按 (风速, 风向偏移,
ti=grid.lut_reference_ti, 间距) 在 LUT 中最近邻查询最优控制下发。

支持：断点续跑、子集、失败日志（与主工程 run_replay 一致）。

用法：
    # 先冒烟单工况跑通（强烈建议）
    python -m induction_vs_yaw_study.steady.run_replay_steady --sim fastfarm --subset

    # FLORIS 全 60 工况
    python -m induction_vs_yaw_study.steady.run_replay_steady --sim floris

    # FAST.Farm 全 60 工况，只跑降额与基准
    python -m induction_vs_yaw_study.steady.run_replay_steady --sim fastfarm --controls baseline derating
"""

from __future__ import annotations

import argparse
import os
import time
import traceback
from pathlib import Path

import numpy as np
import pandas as pd

from induction_vs_yaw_study.steady.cases_steady import (
    iter_steady_cases, load_steady_grid,
)
from induction_vs_yaw_study.steady.replay_steady import (
    replay_fastfarm_steady, replay_floris_steady,
)
from induction_vs_yaw_study.lut.schema import load_lut
from induction_vs_yaw_study.lut.interpolate import lookup


def _dirs():
    root = Path(__file__).resolve().parents[2]
    ts = root / "results" / "timeseries_steady"
    ts.mkdir(parents=True, exist_ok=True)
    return root, ts


def _settings_for_steady(lut: pd.DataFrame, case, control_type: str):
    """
    取某稳态工况某控制的三机设定。

    baseline：全零 / 满额（不查 LUT）。
    yaw / derating：在已建 LUT 中按 (U, wd_offset, ti=参考值, spacing) 最近邻查询。
    """
    if control_type == "baseline":
        return {i: {"yaw_deg": 0.0, "power_target_mw": float("nan"),
                    "ratio": 1.0, "min_pitch_deg": 0.0,
                    "greedy_power_w": float("nan")} for i in (1, 2, 3)}

    return lookup(
        lut, control_type,
        wind_speed_ms=case.wind_speed_ms,
        wind_direction_offset_deg=case.wind_direction_offset_deg,
        turbulence_intensity=case.lut_reference_ti,   # 用参考 TI 切片
        spacing_D=case.spacing_D,
        method="nearest",
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True, choices=["floris", "fastfarm"])
    ap.add_argument("--subset", action="store_true")
    ap.add_argument("--controls", nargs="+",
                    default=["baseline", "yaw", "derating"],
                    choices=["baseline", "yaw", "derating"])
    ap.add_argument("--grid", default=None)
    ap.add_argument("--lut", default=None,
                    help="LUT 路径，默认 results/luts/luts.parquet（复用湍流版已建 LUT）")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    root, ts_dir = _dirs()
    grid = load_steady_grid(args.grid)
    lut_path = args.lut or str(root / "results" / "luts" / "luts.parquet")
    lut = load_lut(lut_path)

    cases = list(iter_steady_cases(grid, subset=args.subset))
    run_log = []
    ref_ti = float(grid.get("lut_reference_ti", 0.10))
    print(f"[STEADY] Replaying {args.sim} for {len(cases)} cases × {args.controls} "
          f"(LUT 参考 TI={ref_ti})")

    for k, case in enumerate(cases, 1):
        for control in args.controls:
            out_csv = ts_dir / f"{args.sim}_{control}_{case.id}.csv"
            if out_csv.exists() and not args.overwrite:
                print(f"  [skip] {out_csv.name} exists")
                continue

            t0 = time.time()
            status, err = "ok", ""
            try:
                settings = _settings_for_steady(lut, case, control)
                out_dir = str(root / "results" / "_work_steady" / args.sim /
                              f"{control}_{case.id}")
                if args.sim == "floris":
                    out = replay_floris_steady(case, control, settings, output_dir=out_dir)
                else:
                    out = replay_fastfarm_steady(case, control, settings, output_dir=out_dir)

                if out is not None and out.time is not None and len(out.time) > 0:
                    out.to_csv(str(out_csv))
                    if (out.metadata or {}).get("aborted"):
                        status, err = "partial", "aborted mid-run; partial CSV saved"
                        print(f"  [PARTIAL] {case.id}/{control}: 已保存部分结果 {out_csv.name}")
                else:
                    status, err = "fail", "no output produced"
                    print(f"  [FAIL] {case.id}/{control}: 无输出")

                # 稳态无 .bts，但仍清理 _work 中可能的中间盒（保险）
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

    log_path = root / "results" / f"run_log_steady_{args.sim}.csv"
    pd.DataFrame(run_log).to_csv(log_path, index=False)
    print(f"\n[STEADY] Run log: {log_path}")


if __name__ == "__main__":
    main()
