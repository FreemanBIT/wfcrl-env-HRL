"""
plots.py — 对比图 (Stage 7)
===========================
核心图：
  A. 功率增益对比（control_type × sim，分面 by 工况）
  B. FLORIS vs FAST.Farm 增益散点（对角线=一致）—— 主结论图
  C. 增益随间距/TI/风速趋势
  D. 载荷 vs 功率增益 trade-off
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _save(fig, out_dir: str, name: str):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_gain_comparison(summary: pd.DataFrame, out_dir: str) -> Optional[str]:
    df = summary[summary["control"].isin(["yaw", "derating"])].copy()
    if df.empty:
        return None
    fig, ax = plt.subplots(figsize=(10, 6))
    controls = ["yaw", "derating"]
    sims = sorted(df["sim"].unique())
    x = np.arange(len(controls))
    width = 0.8 / max(1, len(sims))
    for j, sim in enumerate(sims):
        means = [df[(df["control"] == c) & (df["sim"] == sim)]["gain_vs_baseline_pct"].mean()
                 for c in controls]
        ax.bar(x + j * width, means, width, label=sim)
    ax.set_xticks(x + width * (len(sims) - 1) / 2)
    ax.set_xticklabels(["Yaw", "Derating (Induction)"])
    ax.set_ylabel("Mean farm power gain vs baseline (%)")
    ax.set_title("Power gain: Yaw vs Derating, FLORIS vs FAST.Farm")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    return _save(fig, out_dir, "A_gain_comparison.png")


def plot_floris_vs_fastfarm(summary: pd.DataFrame, out_dir: str) -> Optional[str]:
    """核心结论图：x=FLORIS 增益, y=FAST.Farm 增益。"""
    rows = []
    for (case_id, control), grp in summary.groupby(["case_id", "control"]):
        if control not in ("yaw", "derating"):
            continue
        flo = grp[grp["sim"] == "floris"]
        ff = grp[grp["sim"] == "fastfarm"]
        if flo.empty or ff.empty:
            continue
        rows.append({
            "control": control,
            "floris_gain": flo["gain_vs_baseline_pct"].iloc[0],
            "fastfarm_gain": ff["gain_vs_baseline_pct"].iloc[0],
        })
    if not rows:
        return None
    d = pd.DataFrame(rows)
    fig, ax = plt.subplots(figsize=(7, 7))
    colors = {"yaw": "tab:blue", "derating": "tab:orange"}
    for control in d["control"].unique():
        sub = d[d["control"] == control]
        ax.scatter(sub["floris_gain"], sub["fastfarm_gain"],
                   c=colors.get(control, "gray"), label=control, s=60, alpha=0.7)
    lim = [min(d["floris_gain"].min(), d["fastfarm_gain"].min(), 0) - 1,
           max(d["floris_gain"].max(), d["fastfarm_gain"].max()) + 1]
    ax.plot(lim, lim, "k--", alpha=0.5, label="y=x (perfect agreement)")
    ax.set_xlim(lim); ax.set_ylim(lim)
    ax.set_xlabel("FLORIS predicted gain (%)")
    ax.set_ylabel("FAST.Farm realized gain (%)")
    ax.set_title("Model agreement: FLORIS vs FAST.Farm")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return _save(fig, out_dir, "B_floris_vs_fastfarm.png")


def plot_gain_vs_spacing(summary: pd.DataFrame, out_dir: str) -> Optional[str]:
    """需 summary 含 spacing_D 列（由 run_compare 注入 case 元数据）。"""
    if "spacing_D" not in summary.columns:
        return None
    df = summary[summary["control"].isin(["yaw", "derating"])].copy()
    if df.empty:
        return None
    fig, ax = plt.subplots(figsize=(9, 6))
    for (sim, control), grp in df.groupby(["sim", "control"]):
        g = grp.groupby("spacing_D")["gain_vs_baseline_pct"].mean().sort_index()
        ax.plot(g.index, g.values, marker="o", label=f"{sim}-{control}")
    ax.set_xlabel("Spacing (D)")
    ax.set_ylabel("Mean gain vs baseline (%)")
    ax.set_title("Gain vs turbine spacing")
    ax.legend(); ax.grid(True, alpha=0.3)
    return _save(fig, out_dir, "C_gain_vs_spacing.png")


def plot_load_tradeoff(summary: pd.DataFrame, out_dir: str) -> Optional[str]:
    if "blade_load_std" not in summary.columns:
        return None
    df = summary[summary["control"].isin(["yaw", "derating"])].copy()
    if df.empty or df["blade_load_std"].isna().all():
        return None
    fig, ax = plt.subplots(figsize=(8, 6))
    colors = {"yaw": "tab:blue", "derating": "tab:orange"}
    for control in df["control"].unique():
        sub = df[df["control"] == control]
        ax.scatter(sub["gain_vs_baseline_pct"], sub["blade_load_std"],
                   c=colors.get(control, "gray"), label=control, s=50, alpha=0.7)
    ax.set_xlabel("Power gain vs baseline (%)")
    ax.set_ylabel("Blade load std (proxy for DEL)")
    ax.set_title("Load vs power-gain trade-off")
    ax.legend(); ax.grid(True, alpha=0.3)
    return _save(fig, out_dir, "D_load_tradeoff.png")


def make_all_plots(summary: pd.DataFrame, out_dir: str) -> list:
    paths = []
    for fn in (plot_gain_comparison, plot_floris_vs_fastfarm,
               plot_gain_vs_spacing, plot_load_tradeoff):
        try:
            p = fn(summary, out_dir)
            if p:
                paths.append(p)
        except Exception as e:  # noqa
            print(f"[plots] {fn.__name__} failed: {e}")
    return paths
