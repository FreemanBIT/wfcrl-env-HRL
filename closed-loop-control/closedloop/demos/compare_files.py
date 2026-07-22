"""
Compare two SAVED trajectories (offline).

For real FAST.Farm runs you typically run greedy and a scheme as two separate
simulations. Save each trajectory (the runner/demo can pickle or CSV them), then
compare them here without re-running anything.

Usage::

    # after two runs that saved trajectories:
    python -m closedloop.demos.compare_files \\
        --ctrl results/traj_B.pkl --baseline results/traj_greedy.pkl \\
        --out report_B.html

Accepts .pkl (pickled Trajectory) or .csv (written by Trajectory.save_csv).
"""
from __future__ import annotations

import argparse
from pathlib import Path

from ..evaluate import Evaluator, Trajectory
from .compare_report import build_report_html


def _load(path: str) -> Trajectory:
    p = Path(path)
    if p.suffix == ".csv":
        return Trajectory.from_csv(p)
    return Trajectory.load(p)


def compare_files(
    ctrl_path: str,
    baseline_path: str,
    controller: str = "controller",
    mode: int = 1,
    wind: str = "",
    transient_s: float = 60.0,
    out: str | None = None,
) -> dict:
    ctrl = _load(ctrl_path)
    base = _load(baseline_path)
    ev = Evaluator(transient_s=transient_s)
    gain = ev.gain_vs(ctrl, base)
    print(ev.summary_text(gain, controller))
    if out:
        html = build_report_html(ctrl, base, gain, controller, mode, wind)
        Path(out).write_text(html)
        print(f"Wrote report -> {out}")
    return gain


def main(argv=None):
    p = argparse.ArgumentParser(description="Compare two saved trajectories")
    p.add_argument("--ctrl", required=True, help="controlled trajectory (.pkl/.csv)")
    p.add_argument("--baseline", required=True, help="baseline (greedy) trajectory")
    p.add_argument("--name", default="controller", help="label for the report")
    p.add_argument("--mode", type=int, default=1)
    p.add_argument("--wind", default="")
    p.add_argument("--transient", type=float, default=60.0)
    p.add_argument("--out", default=None, help="output HTML report path")
    args = p.parse_args(argv)
    compare_files(args.ctrl, args.baseline, args.name, args.mode, args.wind,
                  args.transient, args.out)


if __name__ == "__main__":
    main()
