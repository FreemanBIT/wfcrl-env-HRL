"""analyze_doe.py — Full DOE 结果分析（读 cases/*/timeseries.csv → 汇总/增益/模型偏差）
================================================================================

输入布局（run_fastfarm_batch 产出）：
    <run>/cases/<case_id>_<control>/timeseries.csv
    case_id = U{06,08,10,12}_TI{05,10,15}_seed{01..06}_wd{0,10,20}_s{2,4,6,8}D

输出（默认 <run>/analysis/）：
    case_summary.csv   : 每 case 一行（窗内 farm/T1..T3 功率 + 稳定性标记）
    group_summary.csv  : (U,TI,WD,S,control) 聚合（跨 seed 均值/标准差）
    gains.csv          : 同组内 control vs baseline 增益（%），附 FLORIS LUT 预测对比
    report.md          : 可读报告（关键结论）

稳定性判据：统计窗内任一机 pitch>80° 或 rpm<1 视为失稳（OMP>3 曾导致 37% 失稳，
              正常 OMP=3 应为 0%）。失稳 case 默认剔除出统计，并可单独列出。

用法：
    python -m induction_vs_yaw_study.run.analyze_doe --run results/run_full_doe
    python -m induction_vs_yaw_study.run.analyze_doe --run ... --include-unstable
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


def _parse_case_dir(name: str):
    """'U08_TI10_seed01_wd0_s4D_baseline' -> (meta dict, control)。"""
    for ctl in ("baseline", "yaw", "induction"):
        suf = "_" + ctl
        if name.endswith(suf):
            base, control = name[: -len(suf)], ctl
            m = _CASE_RE.search(base)
            if not m:
                return None, control
            g = m.groupdict()
            return {
                "case_id": base,
                "U": int(g["U"]) / 1.0,
                "TI": int(g["TI"]) / 100.0,
                "seed": int(g["seed"]),
                "WD": float(g["WD"]),
                "spacing_D": float(g["S"]),
            }, control
    return None, "unknown"


def _case_metrics(ts_path: Path, t_settle: float, t_end: float) -> Optional[Dict]:
    """读 timeseries.csv，返回窗内 farm/T1..T3 功率与稳定性。"""
    try:
        with open(ts_path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except Exception:
        return None
    if not rows:
        return None
    try:
        t = np.array([float(r["time_s"]) for r in rows])
    except (KeyError, ValueError):
        return None
    win = (t >= t_settle) & (t <= t_end)
    if not win.any():
        return None
    out = {}
    farm = np.array([float(r["farm_power_MW"]) for r in rows])
    out["farm_kw"] = float(np.mean(farm[win]) * 1e3)
    unstable = []
    for i in (1, 2, 3):
        p = np.array([float(r[f"T{i}_power_MW"]) for r in rows])
        rpm = np.array([float(r[f"T{i}_rotor_speed_rpm"]) for r in rows])
        pit = np.array([float(r[f"T{i}_pitch_deg"]) for r in rows])
        out[f"T{i}_kw"] = float(np.mean(p[win]) * 1e3)
        # 失稳判据（2026-10-26 收紧）：正常运行的桨距在 U12 强风下也不超过 ~25°，
        # 转速 >= ~5 rpm。此前用 pitch>80/rpm<1 漏掉了“半顺桨”状态（如 pitch 79°、
        # rpm 1.5），会产出 +117% 的假增益。
        if np.mean(pit[win]) > 40.0 or np.mean(rpm[win]) < 2.0:
            unstable.append(i)
    out["unstable_turbines"] = ";".join(str(i) for i in unstable)
    out["stable"] = not unstable
    out["n_samples"] = int(win.sum())
    return out


def collect(run_dir: Path, t_settle: float, t_end: float) -> List[Dict]:
    cases_dir = run_dir / "cases"
    if not cases_dir.exists():
        raise SystemExit(f"no cases dir: {cases_dir}")
    rows: List[Dict] = []
    for d in sorted(cases_dir.iterdir()):
        if not d.is_dir():
            continue
        meta, control = _parse_case_dir(d.name)
        if meta is None:
            continue
        ts = d / "timeseries.csv"
        if not ts.exists():
            continue
        m = _case_metrics(ts, t_settle, t_end)
        if m is None:
            continue
        rows.append({**meta, "control": control, **m})
    return rows


def _agg(rows: List[Dict], include_unstable: bool) -> List[Dict]:
    """按 (U,TI,WD,S,control) 聚合跨 seed。"""
    from collections import defaultdict
    groups = defaultdict(list)
    for r in rows:
        if not r["stable"] and not include_unstable:
            continue
        key = (r["U"], r["TI"], r["WD"], r["spacing_D"], r["control"])
        groups[key].append(r)
    out = []
    for key, lst in sorted(groups.items()):
        farm = np.array([x["farm_kw"] for x in lst])
        out.append({
            "U": key[0], "TI": key[1], "WD": key[2], "spacing_D": key[3],
            "control": key[4], "n_seeds": len(lst),
            "farm_kw_mean": float(farm.mean()),
            "farm_kw_std": float(farm.std(ddof=1)) if len(lst) > 1 else 0.0,
            "T1_kw_mean": float(np.mean([x["T1_kw"] for x in lst])),
            "T2_kw_mean": float(np.mean([x["T2_kw"] for x in lst])),
            "T3_kw_mean": float(np.mean([x["T3_kw"] for x in lst])),
        })
    return out


def _gains(group_rows: List[Dict], case_rows: Optional[List[Dict]] = None,
           include_unstable: bool = False) -> List[Dict]:
    """同组 (U,TI,WD,S) 内 control vs baseline 增益；并合 FLORIS LUT 预测。

    若给出 case_rows，则按 **seed 配对** 计算增益（每 seed 先算 test/baseline 比值，
    再跨 seed 平均并给标准差）——比“先平均后对比”更严谨，且能给出不确定度。
    """
    lut = None
    try:
        import pandas as pd
        lut_path = _STUDY_ROOT / "results" / "luts" / "luts.parquet"
        if lut_path.exists():
            lut = pd.read_parquet(lut_path)
    except Exception:
        lut = None

    # seed 配对增益表：(U,TI,WD,S) -> {seed: (baseline_kw, {ctl: test_kw})}
    pair = {}
    if case_rows:
        for r in case_rows:
            if not r["stable"] and not include_unstable:
                continue
            key = (r["U"], r["TI"], r["WD"], r["spacing_D"])
            slot = pair.setdefault(key, {})
            sd = slot.setdefault(r["seed"], {})
            sd[r["control"]] = r["farm_kw"]

    idx = {}
    for r in group_rows:
        idx[(r["U"], r["TI"], r["WD"], r["spacing_D"], r["control"])] = r
    out = []
    for (u, ti, wd, s, ctl) in sorted(idx):
        if ctl == "baseline":
            continue
        b = idx.get((u, ti, wd, s, "baseline"))
        t = idx[(u, ti, wd, s, ctl)]
        if b is None:
            continue
        # 优先 seed 配对
        per_seed = []
        for sd, d in (pair.get((u, ti, wd, s), {}) or {}).items():
            if "baseline" in d and ctl in d and d["baseline"] > 0:
                per_seed.append(100.0 * (d[ctl] - d["baseline"]) / d["baseline"])
        if per_seed:
            gain = float(np.mean(per_seed))
            gain_std = float(np.std(per_seed, ddof=1)) if len(per_seed) > 1 else 0.0
            n_pair = len(per_seed)
        else:
            gain = 100.0 * (t["farm_kw_mean"] - b["farm_kw_mean"]) / b["farm_kw_mean"] if b["farm_kw_mean"] else float("nan")
            gain_std = 0.0
            n_pair = 0
        # FLORIS 预测（LUT 同工况）
        pred = float("nan")
        if lut is not None:
            try:
                cid = "U%02d_TI%02d_seed01_wd%g_s%gD" % (u, round(ti * 100), wd, s)
                ct = "derating" if ctl == "induction" else "yaw"
                sub = lut[(lut["case_id"] == cid) & (lut["control_type"] == ct)]
                if not sub.empty:
                    pred = float(sub["gain_vs_baseline_pct"].iloc[0])
            except Exception:
                pass
        out.append({
            "U": u, "TI": ti, "WD": wd, "spacing_D": s, "control": ctl,
            "n_seeds": n_pair or min(b["n_seeds"], t["n_seeds"]),
            "baseline_kw": b["farm_kw_mean"], "test_kw": t["farm_kw_mean"],
            "gain_pct": gain, "gain_std_pct": gain_std, "floris_pred_pct": pred,
            "bias_pct": (100.0 * (pred - gain) / abs(pred)) if pred == pred and pred != 0 else float("nan"),
        })
    return out


def _per_turbine(group_rows: List[Dict]) -> List[Dict]:
    """逐机分解：(U,TI,WD,S) 内各 control 的 T1/T2/T3 功率与相对 baseline 的增量。

    用于回答“偏航/降载让哪台机受益/受损”（上游机降功率、下游机尾流恢复）。
    """
    idx = {}
    for r in group_rows:
        idx[(r["U"], r["TI"], r["WD"], r["spacing_D"], r["control"])] = r
    out = []
    for (u, ti, wd, s, ctl) in sorted(idx):
        if ctl == "baseline":
            continue
        b = idx.get((u, ti, wd, s, "baseline"))
        t = idx[(u, ti, wd, s, ctl)]
        if b is None:
            continue
        row = {"U": u, "TI": ti, "WD": wd, "spacing_D": s, "control": ctl}
        for i in (1, 2, 3):
            k = f"T{i}_kw_mean"
            row[f"T{i}_base_kw"] = b[k]
            row[f"T{i}_test_kw"] = t[k]
            row[f"T{i}_delta_kw"] = t[k] - b[k]
        row["farm_delta_kw"] = sum(row[f"T{i}_delta_kw"] for i in (1, 2, 3))
        out.append(row)
    return out


def _bias_summary(gain_rows: List[Dict]) -> List[Dict]:
    """FLORIS 预测 vs FAST.Farm 实测的系统偏差（按 control × U 聚合）。"""
    from collections import defaultdict
    g = defaultdict(list)
    for r in gain_rows:
        if r["floris_pred_pct"] != r["floris_pred_pct"]:
            continue
        g[(r["control"], r["U"])].append((r["floris_pred_pct"], r["gain_pct"]))
    out = []
    for (ctl, u), vals in sorted(g.items()):
        pred = np.array([v[0] for v in vals])
        meas = np.array([v[1] for v in vals])
        bias = meas - pred
        out.append({
            "control": ctl, "U": u, "n": len(vals),
            "pred_mean_pct": float(pred.mean()),
            "meas_mean_pct": float(meas.mean()),
            "bias_mean_pct": float(bias.mean()),
            "bias_rmse_pct": float(np.sqrt(np.mean(bias ** 2))),
            "overpred_count": int((bias < 0).sum()),
        })
    return out


def _write_csv(rows: List[Dict], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def _report(case_rows, group_rows, gain_rows, unstable_n, path: Path,
            include_unstable: bool = False, per_t=None, bias=None) -> None:
    lines = ["# Full DOE 分析报告", ""]
    incl = "纳入" if include_unstable else "剔除"
    lines.append(f"- 统计窗: [t_settle, t_end]；case 数: {len(case_rows)}；失稳 case {unstable_n}（已{incl}）")
    lines.append("")
    if gain_rows:
        g = np.array([r["gain_pct"] for r in gain_rows])
        lines.append("## 增益总览（control vs baseline，跨 seed×工况）")
        lines.append("")
        lines.append(f"- 样本（组）数: {len(gain_rows)}")
        lines.append(f"- 增益均值: **{g.mean():+.2f}%**  中位: {np.median(g):+.2f}%  范围: {g.min():+.2f}% .. {g.max():+.2f}%")
        for ctl in ("yaw", "induction"):
            sub = [r for r in gain_rows if r["control"] == ctl]
            if sub:
                a = np.array([r["gain_pct"] for r in sub])
                lines.append(f"- **{ctl}**: n={len(sub)} 均值 {a.mean():+.2f}% 中位 {np.median(a):+.2f}%")
        lines.append("")
        lines.append("## 按风速")
        lines.append("")
        lines.append("| U | control | n | 增益均值 | FLORIS 预测均值 |")
        lines.append("|---|---|---|---|---|")
        for u in sorted({r["U"] for r in gain_rows}):
            for ctl in ("yaw", "induction"):
                sub = [r for r in gain_rows if r["U"] == u and r["control"] == ctl]
                if not sub:
                    continue
                a = np.array([r["gain_pct"] for r in sub])
                p = [r["floris_pred_pct"] for r in sub if r["floris_pred_pct"] == r["floris_pred_pct"]]
                pv = f"{np.mean(p):+.2f}%" if p else "n/a"
                lines.append(f"| {u:.0f} | {ctl} | {len(sub)} | {a.mean():+.2f}% | {pv} |")
        lines.append("")
        lines.append("## 按间距")
        lines.append("")
        lines.append("| S | control | n | 增益均值 |")
        lines.append("|---|---|---|---|")
        for s in sorted({r["spacing_D"] for r in gain_rows}):
            for ctl in ("yaw", "induction"):
                sub = [r for r in gain_rows if r["spacing_D"] == s and r["control"] == ctl]
                if not sub:
                    continue
                a = np.array([r["gain_pct"] for r in sub])
                lines.append(f"| {s:g}D | {ctl} | {len(sub)} | {a.mean():+.2f}% |")
        lines.append("")
        lines.append("## 按湍流")
        lines.append("")
        lines.append("| TI | control | n | 增益均值 |")
        lines.append("|---|---|---|---|")
        for ti in sorted({r["TI"] for r in gain_rows}):
            for ctl in ("yaw", "induction"):
                sub = [r for r in gain_rows if r["TI"] == ti and r["control"] == ctl]
                if not sub:
                    continue
                a = np.array([r["gain_pct"] for r in sub])
                lines.append(f"| {ti:.2f} | {ctl} | {len(sub)} | {a.mean():+.2f}% |")

    if per_t:
        lines.append("")
        lines.append("## 逐机分解（相对 baseline 的平均功率增量）")
        lines.append("")
        lines.append("| control | T1 ΔkW | T2 ΔkW | T3 ΔkW | farm ΔkW |")
        lines.append("|---|---|---|---|---|")
        for ctl in ("yaw", "induction"):
            sub = [r for r in per_t if r["control"] == ctl]
            if not sub:
                continue
            lines.append("| %s | %+.0f | %+.0f | %+.0f | %+.0f |" % (
                ctl,
                np.mean([r["T1_delta_kw"] for r in sub]),
                np.mean([r["T2_delta_kw"] for r in sub]),
                np.mean([r["T3_delta_kw"] for r in sub]),
                np.mean([r["farm_delta_kw"] for r in sub]),
            ))

    if bias:
        lines.append("")
        lines.append("## FLORIS 预测 vs FAST.Farm 实测（系统偏差）")
        lines.append("")
        lines.append("| control | U | n | 预测均值 | 实测均值 | 偏差(实测-预测) | RMSE | 高估组数 |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for r in bias:
            lines.append("| %s | %.0f | %d | %+.2f%% | %+.2f%% | %+.2f%% | %.2f%% | %d/%d |" % (
                r["control"], r["U"], r["n"], r["pred_mean_pct"],
                r["meas_mean_pct"], r["bias_mean_pct"], r["bias_rmse_pct"],
                r["overpred_count"], r["n"]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--t-settle", type=float, default=600.0)
    ap.add_argument("--t-end", type=float, default=1200.0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--include-unstable", action="store_true")
    a = ap.parse_args()

    out_dir = a.out or (a.run / "analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    case_rows = collect(a.run, a.t_settle, a.t_end)
    if not case_rows:
        raise SystemExit("no case timeseries found")
    n_unstable = sum(1 for r in case_rows if not r["stable"])
    group_rows = _agg(case_rows, a.include_unstable)
    gain_rows = _gains(group_rows, case_rows, a.include_unstable)

    per_t = _per_turbine(group_rows)
    bias = _bias_summary(gain_rows)

    _write_csv(case_rows, out_dir / "case_summary.csv")
    _write_csv(group_rows, out_dir / "group_summary.csv")
    _write_csv(gain_rows, out_dir / "gains.csv")
    _write_csv(per_t, out_dir / "per_turbine.csv")
    _write_csv(bias, out_dir / "bias_summary.csv")
    _report(case_rows, group_rows, gain_rows, n_unstable, out_dir / "report.md",
            a.include_unstable, per_t=per_t, bias=bias)

    print(f"cases: {len(case_rows)} (unstable {n_unstable})  groups: {len(group_rows)}"
          f"  gain rows: {len(gain_rows)}  per-turbine: {len(per_t)}  bias: {len(bias)}")
    print(f"-> {out_dir}")


if __name__ == "__main__":
    main()
