"""check_doe_quality.py — DOE 数据质量审计（长跑期间定期监控）
================================================================================

扫描 <run>/cases/*/timeseries.csv，逐 case 检查：
  1. 完整性（有无 timeseries.csv、行数、窗内样本数）
  2. 数值合理性（功率非负、不超理论上限、无 NaN）
  3. 稳定性（统计窗内 pitch>40° 或 rpm<2rpm → 失稳，OMP>3/控制失稳的典型特征）
  4. 一致性（同 (U,TI,WD,S,seed) 的 baseline/yaw/induction 是否齐全）

输出：<run>/analysis/quality_report.md + quality_issues.csv
退出码：0 = 无问题；1 = 存在失稳/缺数据（便于脚本化监控）

用法：
    python -m induction_vs_yaw_study.run.check_doe_quality --run results/run_full_doe
    python -m induction_vs_yaw_study.run.check_doe_quality --run ... --strict
"""
from __future__ import annotations

import argparse
import csv
import os
import re
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

_STUDY_ROOT = Path(__file__).resolve().parents[1]

_CASE_RE = re.compile(
    r"U(?P<U>\d+)_TI(?P<TI>\d+)_seed(?P<seed>\d+)_wd(?P<WD>-?\d+)_s(?P<S>[\d.]+)D$")

# NREL 5MW 单机额定电功率 ~5.3MW；farm 上限给 5% 余量
_P_RATED_KW = 5296.6
FARM_MAX_KW = 3 * _P_RATED_KW * 1.05


def _parse(name: str):
    for ctl in ("baseline", "yaw", "induction"):
        suf = "_" + ctl
        if name.endswith(suf):
            base = name[: -len(suf)]
            m = _CASE_RE.search(base)
            if not m:
                return None, ctl
            g = m.groupdict()
            return {
                "case_id": base, "U": float(g["U"]), "TI": int(g["TI"]) / 100.0,
                "seed": int(g["seed"]), "WD": float(g["WD"]),
                "spacing_D": float(g["S"]),
            }, ctl
    return None, "unknown"


def audit_case(case_dir: Path, t_settle: float, t_end: float,
               expect_rows: int = 400) -> Optional[Dict]:
    meta, control = _parse(case_dir.name)
    if meta is None:
        return None
    rec = {**meta, "control": control, "path": case_dir.name,
           "has_ts": False, "rows": 0, "window_rows": 0,
           "farm_kw": float("nan"), "unstable": "", "issues": ""}
    ts = case_dir / "timeseries.csv"
    if not ts.exists():
        rec["issues"] = "missing_timeseries"
        return rec
    try:
        with open(ts, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except Exception as e:  # noqa
        rec["issues"] = f"read_error:{type(e).__name__}"
        return rec
    rec["has_ts"] = True
    rec["rows"] = len(rows)
    if len(rows) < expect_rows * 0.9:
        rec["issues"] = (rec["issues"] + ";row_short").strip(";")
    try:
        t = np.array([float(r["time_s"]) for r in rows])
        farm = np.array([float(r["farm_power_MW"]) for r in rows])
    except (KeyError, ValueError):
        rec["issues"] = (rec["issues"] + ";bad_columns").strip(";")
        return rec
    win = (t >= t_settle) & (t <= t_end)
    rec["window_rows"] = int(win.sum())
    if not win.any():
        rec["issues"] = (rec["issues"] + ";empty_window").strip(";")
        return rec
    f = farm[win]
    if not np.all(np.isfinite(f)):
        rec["issues"] = (rec["issues"] + ";nan_power").strip(";")
    rec["farm_kw"] = float(np.nanmean(f) * 1e3)
    if rec["farm_kw"] < 0:
        rec["issues"] = (rec["issues"] + ";negative_power").strip(";")
    elif rec["farm_kw"] > FARM_MAX_KW:
        rec["issues"] = (rec["issues"] + ";power_over_limit").strip(";")
    # 稳定性
    bad = []
    for i in (1, 2, 3):
        try:
            pit = np.array([float(r[f"T{i}_pitch_deg"]) for r in rows])
            rpm = np.array([float(r[f"T{i}_rotor_speed_rpm"]) for r in rows])
        except (KeyError, ValueError):
            continue
        if np.mean(pit[win]) > 40.0 or np.mean(rpm[win]) < 2.0:
            bad.append(i)
    if bad:
        rec["unstable"] = ";".join(str(i) for i in bad)
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--t-settle", type=float, default=600.0)
    ap.add_argument("--t-end", type=float, default=1200.0)
    ap.add_argument("--expect-rows", type=int, default=400)
    ap.add_argument("--strict", action="store_true",
                    help="任何失稳/问题都返回非零（默认失稳率 >2% 才返回非零）")
    a = ap.parse_args()

    cases_dir = a.run / "cases"
    if not cases_dir.exists():
        raise SystemExit(f"no cases dir: {cases_dir}")
    recs = []
    for d in sorted(cases_dir.iterdir()):
        if d.is_dir():
            r = audit_case(d, a.t_settle, a.t_end, a.expect_rows)
            if r:
                recs.append(r)
    if not recs:
        raise SystemExit("no case dirs matched")

    n = len(recs)
    n_ts = sum(1 for r in recs if r["has_ts"])
    n_issue = sum(1 for r in recs if r["issues"])
    n_unst = sum(1 for r in recs if r["unstable"])
    issues = [r for r in recs if r["issues"]]
    unstable = [r for r in recs if r["unstable"]]

    out_dir = a.run / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "quality_issues.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(recs[0].keys()))
        w.writeheader()
        for r in recs:
            if r["issues"] or r["unstable"]:
                w.writerow(r)

    lines = ["# DOE 数据质量审计", ""]
    lines.append(f"- case 目录: {n}；有 timeseries: {n_ts}；缺失/异常: {n_issue}；"
                 f"失稳: {n_unst}（{100.0*n_unst/max(1,n_ts):.1f}%）")
    lines.append("")
    if issues:
        lines.append("## 缺失/异常（前 20）")
        lines.append("")
        lines.append("| case | control | issue |")
        lines.append("|---|---|---|")
        for r in issues[:20]:
            lines.append(f"| {r['case_id']} | {r['control']} | {r['issues']} |")
        lines.append("")
    if unstable:
        lines.append("## 失稳 case（前 20，pitch>40° 或 rpm<2）")
        lines.append("")
        lines.append("| case | control | 失稳机 | farm kW |")
        lines.append("|---|---|---|---|")
        for r in unstable[:20]:
            lines.append(f"| {r['case_id']} | {r['control']} | T{r['unstable']} | {r['farm_kw']:.0f} |")
        lines.append("")
        # 失稳按 U / control 分布
        from collections import Counter
        cu = Counter(r["U"] for r in unstable)
        cc = Counter(r["control"] for r in unstable)
        lines.append("失稳分布：U " + ", ".join(f"{int(k):.0f}:{v}" for k, v in sorted(cu.items()))
                     + " | control " + ", ".join(f"{k}:{v}" for k, v in sorted(cc.items())))
        lines.append("")
    # 功率分布合理性
    vals = np.array([r["farm_kw"] for r in recs
                     if r["has_ts"] and np.isfinite(r["farm_kw"]) and not r["unstable"]])
    if vals.size:
        lines.append("## 功率分布（排除失稳）")
        lines.append("")
        lines.append(f"- n={vals.size}  均值 {vals.mean():.0f} kW  中位 {np.median(vals):.0f} kW"
                     f"  P5 {np.percentile(vals,5):.0f}  P95 {np.percentile(vals,95):.0f}")
    (out_dir / "quality_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"cases={n} with_ts={n_ts} issues={n_issue} unstable={n_unst}"
          f" ({100.0*n_unst/max(1,n_ts):.1f}%) -> {out_dir/'quality_report.md'}")
    if a.strict:
        raise SystemExit(1 if (n_issue or n_unst) else 0)
    raise SystemExit(1 if (100.0 * n_unst / max(1, n_ts)) > 2.0 else 0)


if __name__ == "__main__":
    main()
