"""
D3 — Comparison evaluation & report.

Runs greedy + a chosen scheme under identical conditions and produces a
side-by-side report: farm-power time series, net gain, per-turbine power bars,
CoV, and load proxy. Output is a self-contained HTML file (SVG charts, no
external deps) plus a printed text summary.

Usage::

    python -m closedloop.demos.compare_report --controller B --mode 1 \\
        --wind steady_8ms --duration 400 --out report.html
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np

from ..evaluate import Evaluator, Trajectory
from .run_demo import run_demo


# ---------------------------------------------------------------------------
# Minimal dependency-free SVG charts
# ---------------------------------------------------------------------------
def _svg_line(series: list[tuple[str, np.ndarray, str]], t: np.ndarray,
              width=680, height=240, title="") -> str:
    """series: list of (label, y-array, color)."""
    if t.size == 0:
        return "<p>(no data)</p>"
    ymax = max(1e-9, max(float(np.max(y)) for _, y, _ in series))
    ymin = min(0.0, min(float(np.min(y)) for _, y, _ in series))
    tmin, tmax = float(t.min()), float(t.max())
    pad = 40

    def X(tv): return pad + (tv - tmin) / max(tmax - tmin, 1e-9) * (width - 2 * pad)
    def Y(yv): return height - pad - (yv - ymin) / max(ymax - ymin, 1e-9) * (height - 2 * pad)

    parts = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
             f'style="max-width:100%;height:auto;font-family:sans-serif">']
    parts.append(f'<rect width="{width}" height="{height}" fill="#fafafa"/>')
    # axes
    parts.append(f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#999"/>')
    parts.append(f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#999"/>')
    parts.append(f'<text x="{width/2}" y="16" text-anchor="middle" font-size="13" font-weight="bold">{title}</text>')
    parts.append(f'<text x="{pad-4}" y="{Y(ymax):.0f}" text-anchor="end" font-size="10">{ymax:.0f}</text>')
    parts.append(f'<text x="{pad-4}" y="{Y(ymin):.0f}" text-anchor="end" font-size="10">{ymin:.0f}</text>')
    for label, y, color in series:
        pts = " ".join(f"{X(t[i]):.1f},{Y(y[i]):.1f}" for i in range(len(y)))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.8"/>')
    # legend
    lx = width - pad - 140
    for k, (label, _, color) in enumerate(series):
        ly = pad + 6 + k * 16
        parts.append(f'<rect x="{lx}" y="{ly-8}" width="12" height="4" fill="{color}"/>')
        parts.append(f'<text x="{lx+16}" y="{ly}" font-size="11">{label}</text>')
    parts.append("</svg>")
    return "".join(parts)


def _svg_bars(labels, values_a, values_b, label_a="ctrl", label_b="greedy",
              width=680, height=240, title="") -> str:
    n = len(labels)
    if n == 0:
        return "<p>(no data)</p>"
    ymax = max(1e-9, max(np.max(values_a), np.max(values_b)))
    pad = 40
    bw = (width - 2 * pad) / n / 2.4
    parts = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
             f'style="max-width:100%;height:auto;font-family:sans-serif">']
    parts.append(f'<rect width="{width}" height="{height}" fill="#fafafa"/>')
    parts.append(f'<text x="{width/2}" y="16" text-anchor="middle" font-size="13" font-weight="bold">{title}</text>')
    parts.append(f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#999"/>')
    for i in range(n):
        cx = pad + (i + 0.5) * (width - 2 * pad) / n
        ha = (values_a[i] / ymax) * (height - 2 * pad)
        hb = (values_b[i] / ymax) * (height - 2 * pad)
        parts.append(f'<rect x="{cx-bw-1:.1f}" y="{height-pad-hb:.1f}" width="{bw:.1f}" height="{hb:.1f}" fill="#bbb"/>')
        parts.append(f'<rect x="{cx+1:.1f}" y="{height-pad-ha:.1f}" width="{bw:.1f}" height="{ha:.1f}" fill="#2b7bba"/>')
        parts.append(f'<text x="{cx:.1f}" y="{height-pad+14:.0f}" text-anchor="middle" font-size="10">{labels[i]}</text>')
    parts.append(f'<rect x="{width-pad-140}" y="{pad-2}" width="12" height="8" fill="#2b7bba"/>')
    parts.append(f'<text x="{width-pad-124}" y="{pad+6}" font-size="11">{label_a}</text>')
    parts.append(f'<rect x="{width-pad-70}" y="{pad-2}" width="12" height="8" fill="#bbb"/>')
    parts.append(f'<text x="{width-pad-54}" y="{pad+6}" font-size="11">{label_b}</text>')
    parts.append("</svg>")
    return "".join(parts)


def build_report_html(ctrl_traj: Trajectory, greedy_traj: Trajectory,
                      gain: dict, controller: str, mode: int, wind: str) -> str:
    n = ctrl_traj.P.shape[1] if ctrl_traj.P.size else 0
    labels = [f"T{i+1}" for i in range(n)]

    ts_chart = _svg_line(
        [("greedy", greedy_traj.P_farm, "#bbb"),
         (f"{controller}", ctrl_traj.P_farm, "#2b7bba")],
        ctrl_traj.time, title="Farm power time series (kW)")
    power_bars = _svg_bars(labels, gain["per_turbine_power_ctrl"],
                           gain["per_turbine_power_base"],
                           controller, "greedy",
                           title="Per-turbine mean power (kW)")
    cov_bars = _svg_bars(labels, gain["cov_ctrl"], gain["cov_base"],
                         controller, "greedy", title="Per-turbine power CoV")
    load_bars = _svg_bars(labels, gain["load_ctrl"], gain["load_base"],
                          controller, "greedy",
                          title="Blade-root moment RMS (kNm) — load proxy")

    g = gain["gain_pct"]
    color = "#2e7d32" if g >= 0 else "#c62828"
    return f"""<!doctype html><html><head><meta charset="utf-8">
<title>{controller} vs greedy</title></head>
<body style="font-family:sans-serif;max-width:760px;margin:24px auto;color:#222">
<h2>Closed-loop control demo report</h2>
<p><b>Controller:</b> {controller} &nbsp; <b>Mode:</b> {mode} &nbsp;
   <b>Wind:</b> {wind}</p>
<div style="font-size:28px;font-weight:bold;color:{color}">
   Net farm-power gain vs greedy: {g:+.2f}%</div>
<p>Mean farm power: <b>{gain['mean_power_ctrl']:.1f} kW</b>
   (greedy {gain['mean_power_base']:.1f} kW)</p>
<div>{ts_chart}</div>
<div>{power_bars}</div>
<div>{cov_bars}</div>
<div>{load_bars}</div>
<hr>
<p style="color:#888;font-size:12px">Generated by closedloop.demos.compare_report.
Mock-plant results are illustrative; magnitudes reflect the analytical wake and
will differ on FAST.Farm. Machinery and sign are the deliverable.</p>
</body></html>"""


def compare(
    controller: str = "B",
    mode: int = 1,
    wind: str = "steady_8ms",
    duration: float = 400.0,
    dt: float = 2.0,
    seed: int = 0,
    transient_s: float = 60.0,
    out: Optional[str] = None,
    fastfarm_dir: Optional[str] = None,
) -> dict:
    """Run greedy + the chosen controller under identical conditions and report."""
    greedy_traj = run_demo("greedy", 1, wind, duration, dt, seed,
                           fastfarm_dir=fastfarm_dir, verbose=False)
    ctrl_traj = run_demo(controller, mode, wind, duration, dt, seed,
                         fastfarm_dir=fastfarm_dir, verbose=False)

    ev = Evaluator(transient_s=transient_s)
    gain = ev.gain_vs(ctrl_traj, greedy_traj)
    print(ev.summary_text(gain, f"{controller} mode{mode}"))

    if out:
        html = build_report_html(ctrl_traj, greedy_traj, gain, controller, mode, wind)
        Path(out).write_text(html)
        print(f"Wrote report -> {out}")
    return gain


def main(argv=None):
    p = argparse.ArgumentParser(description="Compare a controller against greedy")
    p.add_argument("--controller", default="B", choices=["A", "B", "C", "a", "b", "c"])
    p.add_argument("--mode", type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument("--wind", default="steady_8ms")
    p.add_argument("--duration", type=float, default=400.0)
    p.add_argument("--dt", type=float, default=2.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--transient", type=float, default=60.0)
    p.add_argument("--out", default=None, help="output HTML report path")
    p.add_argument("--fastfarm", default=None)
    args = p.parse_args(argv)
    compare(args.controller, args.mode, args.wind, args.duration, args.dt,
            args.seed, args.transient, args.out, args.fastfarm)


if __name__ == "__main__":
    main()
