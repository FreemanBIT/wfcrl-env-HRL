"""
build_luts.py — 构建偏航 + 降额两套 LUT (Stage 3 入口)
======================================================
遍历工况网格，在 FLORIS 中分别求解最优偏航与最优降额，写入 results/luts/。

用法：
    python -m induction_vs_yaw_study.run.build_luts                 # 全网格
    python -m induction_vs_yaw_study.run.build_luts --subset        # 冒烟子集
    python -m induction_vs_yaw_study.run.build_luts --only yaw      # 只跑偏航
    python -m induction_vs_yaw_study.run.build_luts --only derating
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import iter_cases, load_grid
from induction_vs_yaw_study.optimize.floris_yaw_opt import optimize_yaw
from induction_vs_yaw_study.optimize.floris_derating_opt import optimize_derating
from induction_vs_yaw_study.lut.schema import empty_lut, append_row, save_lut


def _results_dir() -> Path:
    root = Path(__file__).resolve().parents[2]
    d = root / "results" / "luts"
    d.mkdir(parents=True, exist_ok=True)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--subset", action="store_true", help="只跑冒烟子集")
    ap.add_argument("--only", choices=["yaw", "derating"], default=None)
    ap.add_argument("--grid", default=None, help="自定义 grid.yaml 路径")
    args = ap.parse_args()

    grid = load_grid(args.grid)
    cases = list(iter_cases(grid, subset=args.subset))
    print(f"Building LUTs for {len(cases)} cases "
          f"({'subset' if args.subset else 'full grid'})")

    ds = grid.get("derating_search", {})
    ys = grid.get("yaw_search", {})

    lut = empty_lut()

    for k, case in enumerate(cases, 1):
        print(f"\n[{k}/{len(cases)}] case {case.id}  "
              f"U={case.wind_speed_ms} wd+{case.wind_direction_offset_deg} "
              f"TI={case.turbulence_intensity} s={case.spacing_D}D")

        # ---- 偏航 ----
        if args.only in (None, "yaw"):
            try:
                yaw_res = optimize_yaw(
                    case,
                    yaw_min_deg=ys.get("yaw_min_deg", -30.0),
                    yaw_max_deg=ys.get("yaw_max_deg", 30.0),
                    fix_last_turbine_zero=ys.get("fix_last_turbine_zero", True),
                )
                print(f"  yaw opt: {yaw_res['yaw_deg']} "
                      f"gain={yaw_res['gain_vs_baseline_pct']:.2f}%")
                for tid in range(1, 4):
                    lut = append_row(
                        lut,
                        case_id=case.id,
                        wind_speed_ms=case.wind_speed_ms,
                        wind_direction_offset_deg=case.wind_direction_offset_deg,
                        turbulence_intensity=case.turbulence_intensity,
                        spacing_D=case.spacing_D,
                        control_type="yaw",
                        turbine_id=tid,
                        yaw_deg=yaw_res["yaw_deg"][tid - 1],
                        power_target_mw=None,
                        ratio=None,
                        min_pitch_deg=0.0,
                        a_diag=None,
                        farm_power_opt_mw=yaw_res["farm_power_opt_mw"],
                        farm_power_baseline_mw=yaw_res["farm_power_baseline_mw"],
                        gain_vs_baseline_pct=yaw_res["gain_vs_baseline_pct"],
                        greedy_power_w=None,
                    )
            except Exception as e:  # noqa
                print(f"  YAW FAILED: {e}")

        # ---- 降额 ----
        if args.only in (None, "derating"):
            try:
                der_res = optimize_derating(
                    case,
                    ratio_min=ds.get("ratio_min", 0.4),
                    ratio_grid_step=ds.get("ratio_grid_step", 0.05),
                )
                print(f"  derating opt: ratios={['%.2f'%r for r in der_res['ratios']]} "
                      f"P(MW)={['%.2f'%p for p in der_res['power_target_mw']]} "
                      f"a={['%.3f'%a for a in der_res['a_diag']]} "
                      f"gain={der_res['gain_vs_baseline_pct']:.2f}%")
                for tid in range(1, 4):
                    lut = append_row(
                        lut,
                        case_id=case.id,
                        wind_speed_ms=case.wind_speed_ms,
                        wind_direction_offset_deg=case.wind_direction_offset_deg,
                        turbulence_intensity=case.turbulence_intensity,
                        spacing_D=case.spacing_D,
                        control_type="derating",
                        turbine_id=tid,
                        yaw_deg=0.0,
                        power_target_mw=der_res["power_target_mw"][tid - 1],
                        ratio=der_res["ratios"][tid - 1],
                        min_pitch_deg=0.0,
                        a_diag=der_res["a_diag"][tid - 1],
                        farm_power_opt_mw=der_res["farm_power_opt_mw"],
                        farm_power_baseline_mw=der_res["farm_power_baseline_mw"],
                        gain_vs_baseline_pct=der_res["gain_vs_baseline_pct"],
                        greedy_power_w=der_res["greedy_power_w"],
                    )
            except Exception as e:  # noqa
                print(f"  DERATING FAILED: {e}")

    out = _results_dir() / "luts.parquet"
    save_lut(lut, str(out))
    print(f"\nSaved LUT: {out} ({len(lut)} rows)")


if __name__ == "__main__":
    main()
