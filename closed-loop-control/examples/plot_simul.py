#!/usr/bin/env python
"""
Simulation result plotter for WFCRL closed-loop FAST.Farm runs.

Usage:
    python plot_simul.py                                    # default path
    python plot_simul.py --path __simul__/launcher/b_mode1_1784859269
    python plot_simul.py --path <path> --plots power,yaw,pitch,loads
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ══════════════════════════════════════════════════════════════════════════
# CONFIG
# ══════════════════════════════════════════════════════════════════════════
DEFAULT_SIMUL_DIR = r"d:\HR_Project\wfcrl-env-HRL\closed-loop-control/__simul__/launcher/b_mode3_1784874613"

MEAS_CHANNELS = {
    "genpwr": ("Power", "MW"),   "blpitch": ("Blade Pitch", "deg"),
    "nacyaw": ("Nacelle Yaw", "deg"), "wind_x": ("Wind Speed", "m/s"),
    "mip1": ("Blade MIP", "kNm"), "moop1": ("Blade MOoP", "kNm"),
    "mzb1": ("Blade Mzb", "kNm"),
}
CONTROL_CHANNELS = {
    "yaw": ("Yaw Target", "deg"), "power": ("Power Target", "MW"),
    "pitch": ("Pitch Target", "deg"),
}

N_TURBINES = 6
COLORS = plt.cm.tab10(np.linspace(0, 1, N_TURBINES))


def _load(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        print(f"  [skip] {path.name} not found")
        return None
    return pd.read_csv(path)


# ══════════════════════════════════════════════════════════════════════════
# Plot 1: Power (total + per-turbine in two subplots)
# ══════════════════════════════════════════════════════════════════════════

def plot_power(simul_dir: Path) -> None:
    df = _load(simul_dir / "measurements_dtl.csv")
    if df is None:
        return

    t = df["t"].values
    p_cols = [f"genpwr_T{i}" for i in range(1, N_TURBINES + 1)]
    p_total = np.sum([df[c].values for c in p_cols], axis=0)

    fig, (ax_total, ax_turbines) = plt.subplots(
        2, 1, figsize=(14, 9), sharex=True,
        gridspec_kw={"height_ratios": [1, 2]})
    fig.suptitle(f"Power — {simul_dir.name}", fontsize=13)

    # Total farm power
    ax_total.plot(t, p_total, color="black", linewidth=1.5)
    ax_total.set_ylabel("Farm Power (MW)", fontsize=10)
    ax_total.grid(True, alpha=0.3)
    ax_total.set_ylim(p_total.min() * 0.9, p_total.max() * 1.05)

    # Per-turbine
    for i in range(N_TURBINES):
        ax_turbines.plot(t, df[p_cols[i]].values, color=COLORS[i],
                         alpha=0.85, linewidth=2, label=f"T{i+1}")
    ax_turbines.set_ylabel("Turbine Power (MW)", fontsize=10)
    ax_turbines.set_xlabel("Time (s)", fontsize=10)
    ax_turbines.grid(True, alpha=0.3)
    ax_turbines.legend(fontsize=8, ncol=6, loc="upper right")
    plt.tight_layout()


# ══════════════════════════════════════════════════════════════════════════
# Plot 2: Control commands vs actual execution
# ══════════════════════════════════════════════════════════════════════════

def plot_control_vs_actual(simul_dir: Path, channels: list[str]) -> None:
    ctrl_df = _load(simul_dir / "controls.csv")
    meas_df = _load(simul_dir / "measurements_dtl.csv")
    if ctrl_df is None or meas_df is None:
        return

    t_ctrl = ctrl_df["t"].values
    t_meas = meas_df["t"].values

    for ch in channels:
        cfg = CONTROL_CHANNELS[ch]
        fig, axes = plt.subplots(2, 3, figsize=(16, 10), sharex=True)
        fig.suptitle(f"{cfg[0]} — Control vs Actual — {simul_dir.name}", fontsize=13)

        # Find global y-range across all turbines
        y_min, y_max = float("inf"), float("-inf")
        for i in range(N_TURBINES):
            meas_col = {
                "yaw": f"nacyaw_T{i+1}", "power": f"genpwr_T{i+1}",
                "pitch": f"blpitch_T{i+1}"}[ch]
            y_min = min(y_min, meas_df[meas_col].min(), ctrl_df[f"{ch}_{i+1}"].min())
            y_max = max(y_max, meas_df[meas_col].max(), ctrl_df[f"{ch}_{i+1}"].max())
        padding = max((y_max - y_min) * 0.05, 0.1)
        y_lim = (y_min - padding, y_max + padding)

        for i in range(N_TURBINES):
            ax = axes[i // 3][i % 3]
            meas_col = {
                "yaw": f"nacyaw_T{i+1}", "power": f"genpwr_T{i+1}",
                "pitch": f"blpitch_T{i+1}"}[ch]
            ax.plot(t_meas, meas_df[meas_col].values, color="blue",
                    alpha=0.7, linewidth=0.6, label="Actual")
            cmd = ctrl_df[f"{ch}_{i+1}"].values
            ax.step(t_ctrl, cmd, where="post", color="red",
                    linewidth=1.5, label="Command")
            ax.set_title(f"Turbine {i+1}", fontsize=10)
            ax.set_ylabel(cfg[1], fontsize=9)
            ax.set_ylim(y_lim)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)

        for ax in axes[-1]:
            ax.set_xlabel("Time (s)", fontsize=10)
        plt.tight_layout()


# ══════════════════════════════════════════════════════════════════════════
# Plot 3: Blade loads
# ══════════════════════════════════════════════════════════════════════════

def plot_loads(simul_dir: Path) -> None:
    df = _load(simul_dir / "measurements_dtl.csv")
    if df is None:
        return

    t = df["t"].values
    loads = {"mip1": "In-plane (MIP)", "moop1": "Out-of-plane (MOoP)",
             "mzb1": "Torsion (Mzb)"}

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"Blade Root Loads — {simul_dir.name}", fontsize=13)

    for idx, (ch, title) in enumerate(loads.items()):
        ax = axes[idx]
        for i in range(N_TURBINES):
            ax.plot(t, df[f"{ch}_T{i+1}"].values / 1e3, color=COLORS[i],
                    alpha=0.8, linewidth=0.6, label=f"T{i+1}")
        ax.set_ylabel(f"{title}\n(MNm)", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=6, loc="upper right")

    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()


# ══════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(description="Plot WFCRL closed-loop simulation results")
    p.add_argument("--path", type=str, default=DEFAULT_SIMUL_DIR)
    p.add_argument("--plots", type=str, default="power,yaw,loads",
                   help="Comma-separated: power, yaw, pitch, loads (or 'all')")
    args = p.parse_args()

    simul_dir = Path(args.path)
    if not simul_dir.exists():
        print(f"Error: path not found — {simul_dir}")
        sys.exit(1)
    print(f"Loading: {simul_dir}")

    plots = "power,yaw,pitch,loads" if args.plots == "all" else args.plots
    plot_names = [s.strip() for s in plots.split(",")]

    ctrl_channels = [ch for ch in ["yaw", "pitch", "power"] if ch in plot_names]
    if ctrl_channels:
        plot_control_vs_actual(simul_dir, ctrl_channels)
    if "power" in plot_names:
        plot_power(simul_dir)
    if "loads" in plot_names:
        plot_loads(simul_dir)

    if not plt.get_fignums():
        print("No data. Check that measurements_dtl.csv and controls.csv exist.")
    else:
        plt.show()


if __name__ == "__main__":
    main()
