"""plot_vtk_wind_field.py — 用 FAST.Farm VTK 扰动风场画全场风速分布（正确的尾流结构）

输入：results/_six_turbine_{yaw,induction}/{ts}/FarmInputs/vtk_ff/Case.Low.DisXY001.*.vtk
（ASCII STRUCTURED_POINTS，轮毂高度 90 m XY 平面，VECTORS Velocity 3 分量）
输出：plots/wind_field_vtk_{t}.png（关键时刻）+ wind_field_vtk_animation.gif（时序动图）

尾流物理：风机后方（下游 +X）风速减速区沿流向拉长（非圆形）；偏航/降载后尾流
偏转/减弱，均可在此场中看到。
"""
from __future__ import annotations

import glob
import os
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

plt.rcParams["font.family"] = ["Microsoft YaHei", "Noto Sans SC", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

_STUDY_ROOT = Path(__file__).resolve().parents[1]
# 2×3 布局（与 LayoutRegistry "6T" 一致）
LAYOUT = {
    1: (0.0, -252.0), 2: (504.0, -252.0), 3: (1008.0, -252.0),
    4: (0.0, 252.0), 5: (504.0, 252.0), 6: (1008.0, 252.0),
}
CTRL = [3, 4]
STEP_AT = 200.0
WIND_DIR_DEG = 270.0   # 西风 → 流向 +X


def parse_vtk_xy(path):
    """解析 FAST.Farm DisXY VTK（ASCII STRUCTURED_POINTS）→ (x, y, u)。"""
    with open(path, "r") as f:
        lines = f.read().splitlines()
    dims = None
    origin = None
    spacing = None
    vel_start = None
    for i, l in enumerate(lines):
        s = l.strip()
        if s.startswith("DIMENSIONS"):
            dims = tuple(int(v) for v in s.split()[1:])
        elif s.startswith("ORIGIN"):
            origin = tuple(float(v) for v in s.split()[1:])
        elif s.startswith("SPACING"):
            spacing = tuple(float(v) for v in s.split()[1:])
        elif s.startswith("VECTORS"):
            vel_start = i + 1
            break
    nx, ny, nz = dims
    ox, oy, oz = origin
    sx, sy, sz = spacing
    xs = ox + np.arange(nx) * sx
    ys = oy + np.arange(ny) * sy
    # 读 (nx*ny) 行，每行 3 个 float（u,v,w）
    u = np.zeros((ny, nx), dtype=np.float32)
    for j in range(ny):
        for i in range(nx):
            toks = lines[vel_start + j * nx + i].split()
            u[j, i] = float(toks[0])
    return xs, ys, u


def plot_frame(ax, xs, ys, u, t_s, title):
    X, Y = np.meshgrid(xs, ys)
    im = ax.pcolormesh(X, Y, u, shading="auto", cmap="viridis",
                       vmin=4.0, vmax=11.0)
    # 风机位置
    for tid, (px, py) in LAYOUT.items():
        mark = "s" if tid in CTRL else "o"
        ax.plot(px, py, mark, color="red" if tid in CTRL else "white",
                markersize=9, markeredgecolor="black", zorder=5)
        ax.text(px, py + 35, f"T{tid}", color="black", fontsize=8,
                ha="center", zorder=6)
    # 风向箭头
    ax.annotate("", xy=(xs[0] + 150, 0), xytext=(xs[0], 0),
                arrowprops=dict(arrowstyle="->", color="white", lw=2))
    ax.text(xs[0] + 60, 30, f"风 {WIND_DIR_DEG}°", color="white", fontsize=9)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"{title}  t={t_s:.0f}s")
    ax.set_aspect("equal")
    return im


def main() -> int:
    results_root = _STUDY_ROOT / "results"
    ctrl = sys.argv[1] if len(sys.argv) > 1 else "yaw"
    runs = sorted(glob.glob(str(results_root / f"_six_turbine_{ctrl}" / "*/FarmInputs/vtk_ff/Case.Low.DisXY001.*.vtk")))
    if not runs:
        print(f"[skip] {ctrl}: 无 VTK 文件")
        return 1
    vtk_files = sorted(runs, key=lambda p: int(re.search(r"\.(\d+)\.vtk$", p).group(1)))
    out_dir = Path(vtk_files[0]).parents[3] / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 解析关键帧 + 全部帧动图
    frames = []
    for p in vtk_files:
        m = re.search(r"\.(\d+)\.vtk$", p)
        idx = int(m.group(1))
        t_s = idx * 3.0   # WrDisDT=3s
        frames.append((p, t_s))

    # 关键时刻单帧图
    for label, tt in (("pre_step", 180.0), ("post_step", 300.0), ("end", 378.0)):
        p, t_s = min(frames, key=lambda f: abs(f[1] - tt))
        xs, ys, u = parse_vtk_xy(p)
        fig, ax = plt.subplots(figsize=(11, 5.5))
        im = plot_frame(ax, xs, ys, u, t_s, f"全场风速分布（FAST.Farm VTK，{ctrl} 控制）")
        fig.colorbar(im, ax=ax, label="X 向风速 u (m/s)")
        fig.tight_layout()
        fig.savefig(out_dir / f"wind_field_vtk_{label}.png", dpi=130, bbox_inches="tight")
        plt.close(fig)
        print(f"  -> {out_dir / ('wind_field_vtk_' + label + '.png')}  (t={t_s:.0f}s)")

    # 动图（抽样，每 3 帧取 1 帧，~42 帧）
    sel = frames[::3]
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.set_aspect("equal")

    def update(i):
        p, t_s = sel[i]
        xs, ys, u = parse_vtk_xy(p)
        ax.clear()
        im = plot_frame(ax, xs, ys, u, t_s, f"全场风速分布（FAST.Farm VTK，{ctrl} 控制）")
        return (im,)

    ani = FuncAnimation(fig, update, frames=len(sel), interval=250, blit=False)
    gif = out_dir / "wind_field_vtk_animation.gif"
    ani.save(gif, writer=PillowWriter(fps=4))
    plt.close(fig)
    print(f"  -> {gif}（{len(sel)} 帧，t=0..{sel[-1][1]:.0f}s）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
