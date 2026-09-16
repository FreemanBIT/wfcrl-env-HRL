"""calibrate_induction.py — 1T induction operating map（方案 V2 §21.1 / 开发计划 §7.1）

对 4 U × 6 a_cmd 做 1T FAST.Farm 回放，得 (U, a_cmd) -> (P, Ct, Cp, Pitch, RotorSpeed)，
子进程隔离（_calib_worker），产出 induction_operating_map.csv。

用法：
    python -m induction_vs_yaw_study.run.calibrate_induction
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]

US = [6.0, 8.0, 10.0, 12.0]
AS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.33]
MAP = _STUDY_ROOT / "results" / "_calib_ind" / "induction_operating_map.csv"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--us", nargs="+", type=float, default=US)
    ap.add_argument("--as", dest="a_commands", nargs="+", type=float, default=AS)
    ap.add_argument("--map", type=Path, default=MAP)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--resume", action="store_true")
    args = ap.parse_args()

    args.map.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if args.resume and args.map.exists():
        import csv as _csv
        with open(args.map, newline="", encoding="utf-8") as f:
            for row in _csv.DictReader(f):
                done.add((float(row["U"]), float(row["a_cmd"])))

    jobs = [(u, a) for u in args.us for a in args.a_commands if (u, a) not in done]
    print(f"calibrate {len(jobs)} points (4U x 6a) -> {args.map}")
    if not jobs:
        print("nothing to run")
        return

    from concurrent.futures import ThreadPoolExecutor
    def one(job):
        u, a = job
        cmd = [sys.executable, "-m", "induction_vs_yaw_study.run._calib_worker",
               "--u", str(u), "--a", str(a), "--map", str(args.map)]
        import subprocess
        r = subprocess.run(cmd, capture_output=True, text=True)
        tail = ((r.stdout or "") + (r.stderr or "")).strip().splitlines()
        print("  " + (tail[-1] if tail else f"rc={r.returncode}"), flush=True)
        return job, r.returncode

    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        for job, rc in pool.map(one, jobs):
            if rc != 0:
                print(f"  [FAIL] U={job[0]} a={job[1]} rc={rc}", flush=True)

    print(f"-> {args.map}")


if __name__ == "__main__":
    sys.exit(main())
