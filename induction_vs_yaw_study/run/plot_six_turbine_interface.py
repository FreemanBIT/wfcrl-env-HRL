"""plot_six_turbine_interface.py — 2×3 六机接口测试结果绘图

输入：results/_six_turbine_{yaw,induction}/{ts}/interface_timeseries.csv
输出（每个测试一个目录）：
  1. farm_power_timeseries.png      全场功率时序
  2. per_turbine_power.png          各机功率时序
  3. yaw_execution.png              各机偏航执行（机舱方位 + 命令）时序
  4. induction_execution.png        各机诱导执行（命令 a + seq 回读）时序
  5. wind_field_animation.gif       全场风速分布时序动图（盘面风 IDW 插值）

用法：
    python -m induction_vs_yaw_study.run.plot_six_turbine_interface
"""
from __future__ import annotations

import csv
import math
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# 中文字体（Windows：Microsoft YaHei / 兜底 Noto Sans SC），避免标题 tofu
plt.rcParams["font.family"] = ["Microsoft YaHei", "Noto Sans SC", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

_STUDY_ROOT = Path(__file__).resolve().parents[1]
REPO = _STUDY_ROOT.parent

ALL_TURBINES = [1, 2, 3, 4, 5, 6]
# 2×3 布局坐标（与 LayoutRegistry "6T" 一致）
LAYOUT = {
    1: (0.0, -252.0), 2: (504.0, -252.0), 3: (1008.0, -252.0),
    4: (0.0, 252.0), 5: (504.0, 252.0), 6: (1008.0, 252.0),
}
CTRL = [3, 4]
STEP_AT = 200.0


def load_csv(path: Path) -> list:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def col(rows, key):
    return np.array([float(r[key]) if r.get(key) not in (None, "") else float("nan")
                     for r in rows])


def save_fig(fig, out_dir, name):
    p = out_dir / name
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {p}")
    return p


def plot_farm_power(rows, out_dir, ctrl):
    t = col(rows, "sim_time_s")
    farm = np.nansum(np.vstack([col(rows, f"T{i}_power_kw") for i in ALL_TURBINES]), axis=0)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(t, farm, "-", color="#1f77b4", lw=1.5, label="全场功率")
    ax.axvline(STEP_AT, color="red", ls="--", lw=1, label=f"{STEP_AT:.0f}s 阶跃")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("全场功率 (kW)")
    ax.set_title(f"全场功率时序（{ctrl} 控制，T3/T4 受控，稳态 11 m/s，270°）")
    ax.legend()
    ax.grid(alpha=0.3)
    return save_fig(fig, out_dir, "farm_power_timeseries.png")


def plot_per_turbine_power(rows, out_dir, ctrl):
    t = col(rows, "sim_time_s")
    colors = {1: "#1f77b4", 2: "#ff7f0e", 3: "#2ca02c", 4: "#d62728", 5: "#9467bd", 6: "#8c564b"}
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for i in ALL_TURBINES:
        ax.plot(t, col(rows, f"T{i}_power_kw"), "-", color=colors[i], lw=1.3,
                label=f"T{i} {'(受控)' if i in CTRL else ''}")
    ax.axvline(STEP_AT, color="red", ls="--", lw=1)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("单机功率 (kW)")
    ax.set_title(f"各机功率时序（{ctrl} 控制）")
    ax.legend(ncol=2, fontsize=9)
    ax.grid(alpha=0.3)
    return save_fig(fig, out_dir, "per_turbine_power.png")


def plot_yaw_execution(rows, out_dir, ctrl):
    t = col(rows, "sim_time_s")
    n = len(ALL_TURBINES)
    fig, axes = plt.subplots(n, 1, figsize=(10, 2.0 * n), sharex=True)
    for ax, i in zip(axes, ALL_TURBINES):
        h = col(rows, f"T{i}_heading_rad")
        cmd = col(rows, f"T{i}_cmd_yaw_deg")
        seq = col(rows, f"T{i}_yaw_seq")
        ax.plot(t, np.degrees(h), "-", color="#2ca02c", lw=1.4, label="机舱方位 (deg)")
        ax.step(t, cmd, "-", color="#1f77b4", lw=1.0, alpha=0.7, where="post",
                label="命令 yaw (deg CW+)")
        # seq 变化标记
        seq_chg = np.where(np.diff(np.concatenate(([0], seq))) > 0)[0]
        if len(seq_chg):
            ax.scatter(t[seq_chg], np.degrees(h)[seq_chg], marker="o", s=40,
                       color="red", zorder=5, label="yaw_seq 生效")
        ax.axvline(STEP_AT, color="red", ls="--", lw=0.8)
        ax.set_ylabel(f"T{i} (deg)")
        if i == ALL_TURBINES[0]:
            ax.legend(fontsize=7, loc="upper right")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("时间 (s)")
    axes[0].set_title(f"各机偏航执行时序（{ctrl} 控制；T3/T4 200s 阶跃 +10°）")
    fig.tight_layout()
    return save_fig(fig, out_dir, "yaw_execution.png")


def plot_induction_execution(rows, out_dir, ctrl):
    t = col(rows, "sim_time_s")
    n = len(ALL_TURBINES)
    fig, axes = plt.subplots(n, 1, figsize=(10, 2.0 * n), sharex=True)
    for ax, i in zip(axes, ALL_TURBINES):
        a = col(rows, f"T{i}_cmd_a")
        seq = col(rows, f"T{i}_ind_seq")
        p = col(rows, f"T{i}_power_kw")
        ax.step(t, a, "-", color="#1f77b4", lw=1.4, where="post", label="命令 a")
        ax.scatter(t, [1.0 / 3.0] * len(t), s=0)  # 占位对齐 y 轴
        seq_chg = np.where(np.diff(np.concatenate(([0], seq))) > 0)[0]
        if len(seq_chg):
            ax.scatter(t[seq_chg], a[seq_chg], marker="o", s=45, color="red", zorder=5,
                       label="ind_seq 生效")
        ax.axvline(STEP_AT, color="red", ls="--", lw=0.8)
        ax.set_ylim(0.0, 0.45)
        ax.set_ylabel(f"T{i} (a)")
        if i == ALL_TURBINES[0]:
            ax.legend(fontsize=7, loc="upper right")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("时间 (s)")
    axes[0].set_title(f"各机诱导因子执行时序（{ctrl} 控制；T3/T4 200s 阶跃 a=0.2）")
    fig.tight_layout()
    return save_fig(fig, out_dir, "induction_execution.png")


def plot_wind_field_animation(rows, out_dir, ctrl):
    """全场风速分布时序动图：以各机盘面风做 IDW 插值到 2D 网格，逐帧动画。

    展示全场风速分布随控制命令演化（尾流偏转/减弱）。y 范围覆盖 2×3 布局。
    """
    t = col(rows, "sim_time_s")
    n_frames = len(rows)
    # 网格（覆盖布局 + 余量）
    xs = np.linspace(-600, 1600, 120)
    ys = np.linspace(-900, 900, 120)
    X, Y = np.meshgrid(xs, ys)
    pts = np.array([LAYOUT[i] for i in ALL_TURBINES], dtype=float)  # (6,2)
    # 预计算 IDW 距离权重矩阵（(NY*NX, 6)，网格与风机位置固定）
    _d2 = (X.ravel()[:, None] - pts[:, 0][None, :]) ** 2 \
        + (Y.ravel()[:, None] - pts[:, 1][None, :]) ** 2
    _D2W = 1.0 / (_d2 + 900.0)   # 30m 平滑半径

    fig, ax = plt.subplots(figsize=(9, 6))
    # 背景网格固定
    scat = ax.scatter([], [], c=[], cmap="viridis", vmin=5.0, vmax=11.0, s=120,
                      edgecolors="black", zorder=5)
    im = ax.imshow(np.zeros_like(X), extent=[xs[0], xs[-1], ys[0], ys[-1]],
                   origin="lower", cmap="viridis", vmin=5.0, vmax=11.0, alpha=0.85,
                   aspect="auto")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("全场风速分布（盘面风 IDW 插值） t = 0 s")
    cb = fig.colorbar(im, ax=ax, label="盘面风速 (m/s)")
    txt = ax.text(0.02, 0.96, "", transform=ax.transAxes, fontsize=12,
                  va="top", ha="left", color="white",
                  bbox=dict(boxstyle="round", fc="black", alpha=0.6))

    def idw_frame(r):
        """由各机盘面扰动风做反距离加权插值（距离矩阵预计算）。"""
        v = np.array([float(r.get(f"T{i}_disk_disturbed", 11.0)) for i in ALL_TURBINES])
        v = np.nan_to_num(v, nan=11.0)
        grid = (_D2W * v[None, :]).sum(axis=1) / _D2W.sum(axis=1)
        return grid.reshape(X.shape), v

    def update(frame):
        r = rows[frame]
        grid, v = idw_frame(r)
        im.set_data(grid)
        scat.set_offsets(pts)
        scat.set_array(v)
        # 受控机标记
        ax.set_title(f"全场风速分布（盘面风 IDW 插值） t = {float(r['sim_time_s']):.0f} s")
        txt.set_text(f"t={float(r['sim_time_s']):.0f}s | T3={v[2]:.2f} T4={v[3]:.2f} m/s"
                     f" | {'阶跃后' if float(r['sim_time_s']) >= STEP_AT else '阶跃前'}")
        return (im, scat, txt)

    ani = FuncAnimation(fig, update, frames=n_frames, interval=200, blit=False)
    out = out_dir / "wind_field_animation.gif"
    ani.save(out, writer=PillowWriter(fps=5))
    plt.close(fig)
    print(f"  -> {out}（{n_frames} 帧）")
    return out


def main() -> int:
    results_root = _STUDY_ROOT / "results"
    # 找每个控制的最新 run
    for ctrl in ("yaw", "induction"):
        runs = sorted((results_root / f"_six_turbine_{ctrl}").glob("*/interface_timeseries.csv"))
        if not runs:
            print(f"[skip] {ctrl}: 无结果")
            continue
        csv_path = runs[-1]
        out_dir = csv_path.parent / "plots"
        out_dir.mkdir(parents=True, exist_ok=True)
        rows = load_csv(csv_path)
        print(f"== {ctrl} control: {csv_path.parent.name} ({len(rows)} steps)")
        plot_farm_power(rows, out_dir, ctrl)
        plot_per_turbine_power(rows, out_dir, ctrl)
        if ctrl == "yaw":
            plot_yaw_execution(rows, out_dir, ctrl)
        else:
            plot_induction_execution(rows, out_dir, ctrl)
        plot_wind_field_animation(rows, out_dir, ctrl)
    return 0


if __name__ == "__main__":
    sys.exit(main())
