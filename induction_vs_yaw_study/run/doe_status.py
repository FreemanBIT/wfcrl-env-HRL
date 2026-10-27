"""doe_status.py — DOE 进度看板（一键：进度 / 速率 / ETA / 质量 / 资源）
================================================================================

用法：
    python -m induction_vs_yaw_study.run.doe_status --run results/run_full_doe
    python -m induction_vs_yaw_study.run.doe_status --run ... --json   # 机器可读
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import time
from collections import Counter
from pathlib import Path


def _fmt_hms(sec: float) -> str:
    if sec != sec or sec <= 0:
        return "-"
    h = int(sec // 3600)
    m = int((sec % 3600) // 60)
    return f"{h}h{m:02d}m"


def collect(run: Path) -> dict:
    stat = run / "manifest_status.csv"
    rows = []
    if stat.exists():
        with open(stat, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    done = [r for r in rows if r.get("status") == "done"]
    failed = [r for r in rows if r.get("status") == "failed"]
    feathered = [r for r in rows if r.get("status") == "feathered"]
    walls = []
    for r in done:
        try:
            w = float(r.get("wall_s") or 0)
            if w > 0:
                walls.append(w)
        except ValueError:
            pass
    total_target = 2592  # full DOE
    n_done = len(done)
    n_failed = len(failed)
    remaining = max(0, total_target - n_done - n_failed)
    rate = (sum(1 for _ in done) / (sum(walls) / 3600.0)) if walls else 0.0
    eta_h = (remaining / rate) if rate > 0 else float("inf")
    # 按 U 统计
    by_u = Counter((r.get("case_id", "").split("_")[0]) for r in done)
    # 磁盘/内存
    try:
        import shutil
        free = shutil.disk_usage(str(run)).free / 1e9
    except Exception:
        free = float("nan")
    try:
        import subprocess
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "(Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory"],
            capture_output=True, text=True, timeout=15)
        mem_free_gb = float(out.stdout.strip()) / 1e6 if out.stdout.strip() else float("nan")
    except Exception:
        mem_free_gb = float("nan")
    n_cases = len(glob.glob(str(run / "cases" / "*")))
    return {
        "run": str(run),
        "total_target": total_target,
        "done": n_done, "failed": n_failed, "feathered": len(feathered),
        "remaining": remaining,
        "progress_pct": round(100.0 * n_done / total_target, 2),
        "rate_cases_per_h": round(rate, 2),
        "mean_wall_s": round(sum(walls) / len(walls), 0) if walls else 0,
        "eta_hours": round(eta_h, 1) if eta_h != float("inf") else None,
        "eta_str": _fmt_hms(eta_h * 3600),
        "by_u": dict(by_u),
        "case_dirs": n_cases,
        "disk_free_gb": round(free, 1),
        "mem_free_gb": round(mem_free_gb, 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    info = collect(a.run)
    if a.json:
        print(json.dumps(info, ensure_ascii=False, indent=1))
        return
    print("=" * 62)
    print(f"DOE 进度  {info['run']}")
    print("=" * 62)
    print(f"  完成      : {info['done']} / {info['total_target']}  ({info['progress_pct']}%)")
    print(f"  失败      : {info['failed']}   失稳标记: {info['feathered']}")
    print(f"  剩余      : {info['remaining']}")
    print(f"  速率      : {info['rate_cases_per_h']} case/h   平均 {info['mean_wall_s']:.0f}s/case")
    print(f"  ETA       : {info['eta_str']}" + (f"  ({info['eta_hours']} h)" if info['eta_hours'] else ""))
    print(f"  按风速    : " + ", ".join(f"{k}:{v}" for k, v in sorted(info['by_u'].items())))
    print(f"  磁盘空闲  : {info['disk_free_gb']} GB    内存空闲: {info['mem_free_gb']} GB")
    print(f"  case 目录 : {info['case_dirs']}")
    print("=" * 62)


if __name__ == "__main__":
    main()
