"""
run_compare.py — 交叉对比与报告 (Stage 7 入口)
==============================================
从 results/timeseries/ 构建 summary，注入 case 元数据（间距/TI/风速），出图 +
一页 REPORT.md。

用法：
    python -m induction_vs_yaw_study.run.run_compare
"""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from induction_vs_yaw_study.cases.three_nrel5mw import iter_cases, load_grid
from induction_vs_yaw_study.analysis.metrics import build_summary
from induction_vs_yaw_study.analysis.plots import make_all_plots


def _inject_case_meta(summary: pd.DataFrame, grid: dict) -> pd.DataFrame:
    """把 case_id → (U, wd, TI, spacing) 元数据并入 summary。"""
    meta = {}
    for case in iter_cases(grid, subset=False):
        meta[case.id] = {
            "wind_speed_ms": case.wind_speed_ms,
            "wind_direction_offset_deg": case.wind_direction_offset_deg,
            "turbulence_intensity": case.turbulence_intensity,
            "spacing_D": case.spacing_D,
        }
    # 也并入子集（避免只跑了子集时查不到）
    for case in iter_cases(grid, subset=True):
        meta.setdefault(case.id, {
            "wind_speed_ms": case.wind_speed_ms,
            "wind_direction_offset_deg": case.wind_direction_offset_deg,
            "turbulence_intensity": case.turbulence_intensity,
            "spacing_D": case.spacing_D,
        })
    for col in ("wind_speed_ms", "wind_direction_offset_deg",
                "turbulence_intensity", "spacing_D"):
        summary[col] = summary["case_id"].map(
            lambda cid: meta.get(cid, {}).get(col))
    return summary


def _write_report(summary: pd.DataFrame, fig_paths, out_md: Path):
    lines = ["# 诱导(降额) vs 偏航 控制对比报告\n"]
    if summary.empty:
        lines.append("（无数据，请先运行 build_luts 与 run_replay。）")
        out_md.write_text("\n".join(lines), encoding="utf-8")
        return

    # 关键数字
    for control in ("yaw", "derating"):
        sub = summary[summary["control"] == control]
        if sub.empty:
            continue
        lines.append(f"\n## {control}")
        for sim in sorted(sub["sim"].unique()):
            s2 = sub[sub["sim"] == sim]
            g = s2["gain_vs_baseline_pct"].mean()
            lines.append(f"- {sim}: 平均增益 {g:.2f}%（n={len(s2)}）")

    # 模型偏差
    ff = summary[(summary["sim"] == "fastfarm") & summary["model_bias_pct"].notna()]
    if not ff.empty:
        lines.append("\n## FLORIS vs FAST.Farm 模型偏差")
        for control in ("yaw", "derating"):
            s2 = ff[ff["control"] == control]
            if not s2.empty:
                lines.append(f"- {control}: FAST.Farm 相对 FLORIS "
                             f"{s2['model_bias_pct'].mean():+.2f}%（>0 表示 FLORIS 低估）")

    # 图
    if fig_paths:
        lines.append("\n## 图")
        for p in fig_paths:
            rel = Path(p).name
            lines.append(f"- ![{rel}](figures/{rel})")

    out_md.write_text("\n".join(lines), encoding="utf-8")


def main():
    root = Path(__file__).resolve().parents[2]
    ts_dir = root / "results" / "timeseries"
    fig_dir = root / "results" / "figures"
    grid = load_grid()

    summary = build_summary(str(ts_dir), frac=0.5)
    if summary.empty:
        print("No timeseries found. Run build_luts + run_replay first.")
        return

    summary = _inject_case_meta(summary, grid)
    summary_path = root / "results" / "summary.parquet"
    try:
        summary.to_parquet(summary_path, index=False)
    except Exception:
        pass
    summary.to_csv(root / "results" / "summary.csv", index=False)
    print(f"Summary: {len(summary)} rows -> {summary_path}")

    fig_paths = make_all_plots(summary, str(fig_dir))
    print(f"Figures: {fig_paths}")

    _write_report(summary, fig_paths, root / "results" / "REPORT.md")
    print(f"Report: {root / 'results' / 'REPORT.md'}")


if __name__ == "__main__":
    main()
