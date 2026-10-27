"""doe_plots.py — Full DOE 结果图（研究交付物）
================================================================================

读取 <run>/analysis/ 下的 gains.csv / bias_summary.csv / per_turbine.csv，
输出 PNG：
  1. gain_vs_spacing.png   增益随间距（yaw / induction 分面）
  2. gain_vs_ti.png        增益随湍流
  3. gain_vs_u.png         增益随风速 + FLORIS 预测对照
  4. floris_vs_fastfarm.png 预测 vs 实测散点（对角线=一致）
  5. per_turbine_delta.png 逐机功率增量（偏航/降载分面）
  6. bias_bars.png          FLORIS 系统偏差（按 control×U）

用法：
    python -m induction_vs_yaw_study.run.analyze_doe --run results/run_full_doe
    python -m induction_vs_yaw_study.analysis.doe_plots --analysis results/run_full_doe/analysis
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

_CTL_COLORS = {"yaw": "#1f77b4", "induction": "#d62728"}


def _load(analysis: Path, name: str) -> pd.DataFrame:
    p = analysis / name
    if not p.exists():
        raise SystemExit(f"missing {p} (run analyze_doe first)")
    return pd.read_csv(p)


def _save(fig, analysis: Path, name: str) -> str:
    out = analysis / name
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return str(out)


def plot_gain_vs(analysis: Path, dim: str, fname: str, xlabel: str) -> str:
    g = _load(analysis, "gains.csv")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, ctl in zip(axes, ("yaw", "induction")):
        sub = g[g["control"] == ctl]
        if sub.empty:
            ax.set_title(f"{ctl}: no data")
            continue
        vals = sorted(sub[dim].unique())
        means = [sub[sub[dim] == v]["gain_pct"].mean() for v in vals]
        stds = [sub[sub[dim] == v]["gain_pct"].std(ddof=1) if len(sub[sub[dim] == v]) > 1 else 0.0
                for v in vals]
        ns = [len(sub[sub[dim] == v]) for v in vals]
        ax.errorbar(vals, means, yerr=stds, marker="o", capsize=4,
                    color=_CTL_COLORS[ctl], lw=1.5, ms=6)
        # FLORIS 预测均值（若有）
        pred = [sub[sub[dim] == v]["floris_pred_pct"].mean() for v in vals]
        if any(p == p for p in pred):
            ax.plot(vals, pred, "--", color="gray", lw=1.2, label="FLORIS pred")
            ax.legend(fontsize=8)
        for x, y, n in zip(vals, means, ns):
            if y == y:
                ax.annotate(f"n={n}", (x, y), textcoords="offset points",
                            xytext=(0, -12), ha="center", fontsize=7, color="gray")
        ax.axhline(0, color="black", lw=0.8, ls=":")
        ax.set_title(f"{ctl}")
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("farm power gain (%)")
    fig.suptitle(f"Power gain vs {xlabel} (mean +/- std over seeds x cases)", fontsize=11)
    return _save(fig, analysis, fname)


def plot_floris_scatter(analysis: Path) -> str:
    g = _load(analysis, "gains.csv")
    g = g[g["floris_pred_pct"] == g["floris_pred_pct"]]
    fig, ax = plt.subplots(figsize=(6, 6))
    for ctl in ("yaw", "induction"):
        sub = g[g["control"] == ctl]
        if sub.empty:
            continue
        ax.scatter(sub["floris_pred_pct"], sub["gain_pct"], s=28, alpha=0.7,
                   color=_CTL_COLORS[ctl], label=ctl)
    lim = [min(-5, g["floris_pred_pct"].min() - 1, g["gain_pct"].min() - 1),
           max(15, g["floris_pred_pct"].max() + 1, g["gain_pct"].max() + 1)]
    ax.plot(lim, lim, "k--", lw=1, label="y = x")
    ax.axhline(0, color="gray", lw=0.6, ls=":")
    ax.axvline(0, color="gray", lw=0.6, ls=":")
    ax.set_xlim(lim); ax.set_ylim(lim)
    ax.set_xlabel("FLORIS predicted gain (%)")
    ax.set_ylabel("FAST.Farm measured gain (%)")
    ax.set_title("Predicted vs measured (below diagonal = overestimation)")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
    return _save(fig, analysis, "floris_vs_fastfarm.png")


def plot_per_turbine(analysis: Path) -> str:
    pt = _load(analysis, "per_turbine.csv")
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax, ctl in zip(axes, ("yaw", "induction")):
        sub = pt[pt["control"] == ctl]
        if sub.empty:
            ax.set_title(f"{ctl}: no data")
            continue
        d = [sub[f"T{i}_delta_kw"].mean() for i in (1, 2, 3)]
        e = [sub[f"T{i}_delta_kw"].std(ddof=1) if len(sub) > 1 else 0 for i in (1, 2, 3)]
        x = np.arange(3)
        ax.bar(x, d, yerr=e, capsize=4, color=["#888", "#888", "#888"])
        ax.set_xticks(x); ax.set_xticklabels(["T1", "T2", "T3"])
        ax.axhline(0, color="black", lw=0.8)
        ax.set_title(f"{ctl} (farm delta={np.mean(d):+.0f} kW)")
        ax.set_ylabel("power delta vs baseline (kW)" if ctl == "yaw" else "")
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Per-turbine power delta (upstream yields, downstream gains)", fontsize=11)
    return _save(fig, analysis, "per_turbine_delta.png")


def plot_bias(analysis: Path) -> str:
    b = _load(analysis, "bias_summary.csv")
    fig, ax = plt.subplots(figsize=(8, 4))
    us = sorted(b["U"].unique())
    width = 0.35
    for k, ctl in enumerate(("yaw", "induction")):
        sub = b[b["control"] == ctl].set_index("U")
        vals = [sub.loc[u, "bias_mean_pct"] if u in sub.index else np.nan for u in us]
        ax.bar(np.arange(len(us)) + (k - 0.5) * width, vals, width,
               color=_CTL_COLORS[ctl], label=ctl)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xticks(np.arange(len(us))); ax.set_xticklabels([f"U{u:.0f}" for u in us])
    ax.set_ylabel("bias = measured - predicted (%)")
    ax.set_title("FLORIS systematic bias (negative = overestimates)  ")
    ax.legend(); ax.grid(axis="y", alpha=0.3)
    return _save(fig, analysis, "bias_bars.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--analysis", type=Path, required=True)
    a = ap.parse_args()
    ad = a.analysis
    outs = [
        plot_gain_vs(ad, "spacing_D", "gain_vs_spacing.png", "spacing (D)"),
        plot_gain_vs(ad, "TI", "gain_vs_ti.png", "turbulence intensity"),
        plot_gain_vs(ad, "U", "gain_vs_u.png", "wind speed (m/s)"),
        plot_floris_scatter(ad),
        plot_per_turbine(ad),
        plot_bias(ad),
    ]
    for o in outs:
        print("->", o)


if __name__ == "__main__":
    main()
