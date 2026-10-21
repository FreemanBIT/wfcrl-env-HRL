"""compute_del_batch.py — DEL 端到端工具（Phase 7 Power-Load Pareto 前置）
================================================================================

输入（三选一）：
  1) 批跑 run 目录：results/run_*/cases/<case_id>_<control>/FarmInputs/Case.T{i}.outb
     （功率从同目录 timeseries.csv 的 ZMQ 状态帧取）
  2) outb_only 目录：results/<out>/outb/T{i}.csv + outb_summary.csv
  3) 单个 case 目录（含 FarmInputs/Case.T{i}.outb）

输出：<target>/del_summary.csv —— 每行 = (case_id, control, turbine, channel,
      m, n_eq, n_cycles, load_mean, load_std, DEL, P_kW_mean, U_mps_mean)

用法：
    python -m induction_vs_yaw_study.run.compute_del_batch \
        --run results/run_pilot_v1 [--n-eq 600] [--t-settle 60] [--out del_summary.csv]
    python -m induction_vs_yaw_study.run.compute_del_batch --outb-only results/_outb_x
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from pathlib import Path

import numpy as np

_STUDY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_STUDY_ROOT))

from induction_vs_yaw_study.analysis.del_analysis import (
    compute_del_from_outb, CHANNEL_M, DEFAULT_N_EQ,
)

# 计算哪些通道（按组件覆盖，排除诊断/位移）
DEL_CHANNELS = ["RootMIP1", "RootMOoP1", "RootMzb1",
                "TwrBsMyt", "TwrBsMxt", "TwrBsFxt",
                "LSSGagMxa", "LSSGagFxa", "YawBrMyp", "YawBrFxn"]


def _parse_outb_csv(csv_path: Path):
    """outb_only 保存的 T{i}.csv（openfast_toolbox toDataFrame 产物）。"""
    import pandas as pd
    return pd.read_csv(csv_path)


def _outb_candidates(case_dir: Path) -> dict:
    """收集 case 目录的 (turbine -> outb 路径)。支持两种布局：
      - FarmInputs/Case.T{i}.outb（批跑 case 或乙模式 FarmInputs）
      - outb/T{i}.csv（outb_only 落盘）
    """
    out = {}
    fi = case_dir / "FarmInputs"
    if fi.exists():
        for ob in sorted(fi.glob("Case.T*.outb")):
            m = ob.stem.rsplit(".T", 1)
            if len(m) == 2 and m[1].isdigit():
                out[int(m[1])] = ob
    obdir = case_dir / "outb"
    if obdir.exists():
        for c in sorted(obdir.glob("T*.csv")):
            m = c.stem[1:]
            if m.isdigit():
                out[int(m)] = c
    return out


def _power_from_timeseries(case_dir: Path, turbine: int) -> float:
    """从批跑 case 的 timeseries.csv（ZMQ 状态帧）取该机平均功率 (kW)。"""
    ts = case_dir / "timeseries.csv"
    if not ts.exists():
        return float("nan")
    import pandas as pd
    df = pd.read_csv(ts)
    col = [c for c in df.columns if c.endswith(f"_power_MW") and
           c.startswith(f"T{turbine}")]
    if not col:
        col = [c for c in df.columns if f"T{turbine}" in c and "power" in c.lower()]
    if not col:
        return float("nan")
    return float(np.nanmean(df[col[0]].to_numpy(float))) * 1e3


def _power_from_outb_summary(case_dir: Path, turbine: int) -> tuple:
    """outb_only 的 outb_summary.csv：返回 (P_kW_mean, U_mps_mean)。"""
    s = case_dir / "outb" / "outb_summary.csv"
    if not s.exists():
        return float("nan"), float("nan")
    with open(s, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if str(row.get("turbine", "")) == str(turbine):
                try:
                    return float(row.get("P_kW_mean", "nan")), float(row.get("U_mps_mean", "nan"))
                except ValueError:
                    return float("nan"), float("nan")
    return float("nan"), float("nan")


def _del_for_turbine(outb: Path, turbine: int, n_eq: float, t_settle: float) -> list:
    """单机全部载荷通道的 DEL 行。"""
    rows = []
    try:
        from openfast_toolbox.io.fast_output_file import FASTOutputFile
        if outb.suffix == ".outb":
            df = FASTOutputFile(str(outb)).toDataFrame()
        else:
            df = _parse_outb_csv(outb)
        cols = list(df.columns)
        for ch in DEL_CHANNELS:
            hit = [c for c in cols if ch in c]
            if not hit:
                continue
            lcol = hit[0]
            series = df[lcol].to_numpy(float)
            unit = "kN" if "kN" in lcol else "raw"
            if unit == "kN":
                series = series * 1e3
            tcol = [c for c in cols if "Time" in c]
            if tcol and t_settle > 0:
                t = df[tcol[0]].to_numpy(float)
                series = series[t >= t_settle]
            series = series[~np.isnan(series)]
            if len(series) < 10:
                continue
            m = next((v for k, v in CHANNEL_M.items() if k in lcol), 10)
            from induction_vs_yaw_study.analysis.del_analysis import (
                rainflow, del_from_cycles,
            )
            cyc = rainflow(series)
            delv = del_from_cycles(cyc, float(m), n_eq=n_eq)
            rows.append({
                "turbine": turbine, "channel": lcol, "m": m, "n_eq": n_eq,
                "n_cycles": len(cyc),
                "load_mean": float(np.mean(series)),
                "load_std": float(np.std(series, ddof=1)),
                "DEL": float(delv),
            })
    except Exception as e:  # noqa
        print(f"  [warn] T{turbine} {outb.name} DEL failed: {e}")
    return rows


def process_case(case_dir: Path, n_eq: float, t_settle: float) -> list:
    """处理一个 case 目录，返回 del_summary 行（含 case_id/control 前缀）。"""
    name = case_dir.name
    parts = name.rsplit("_", 1)
    control = parts[1] if len(parts) == 2 else "unknown"
    case_id = parts[0] if len(parts) == 2 else name
    outbs = _outb_candidates(case_dir)
    rows = []
    for tid in sorted(outbs):
        del_rows = _del_for_turbine(outbs[tid], tid, n_eq, t_settle)
        # 功率
        p_kw = _power_from_timeseries(case_dir, tid)
        u = float("nan")
        if np.isnan(p_kw):
            p_kw, u = _power_from_outb_summary(case_dir, tid)
        for dr in del_rows:
            dr.update({"case_id": case_id, "control": control,
                       "P_kW_mean": p_kw, "U_mps_mean": u})
            rows.append(dr)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, default=None,
                    help="批跑 run 目录（cases/ 子目录逐个处理）")
    ap.add_argument("--outb-only", type=Path, default=None,
                    help="outb_only 输出目录（含 outb/T{i}.csv）")
    ap.add_argument("--case", type=Path, default=None,
                    help="单个 case 目录")
    ap.add_argument("--n-eq", type=float, default=DEFAULT_N_EQ)
    ap.add_argument("--t-settle", type=float, default=60.0)
    ap.add_argument("--out", type=Path, default=None,
                    help="输出 del_summary.csv（默认 <输入目录>/del_summary.csv）")
    a = ap.parse_args()

    targets: list = []
    if a.run:
        for d in sorted((a.run / "cases").glob("*")):
            if d.is_dir():
                targets.append(d)
        print(f"[del-batch] run {a.run}: {len(targets)} case dirs")
    elif a.outb_only:
        targets = [a.outb_only]
        print(f"[del-batch] outb_only dir: {a.outb_only}")
    elif a.case:
        targets = [a.case]
    else:
        raise SystemExit("need --run | --outb-only | --case")

    all_rows = []
    for t in targets:
        rows = process_case(t, a.n_eq, a.t_settle)
        if rows:
            all_rows.extend(rows)
            print(f"  {t.name}: {len(rows)} DEL rows")
        else:
            print(f"  {t.name}: (no outb found)")

    if not all_rows:
        raise SystemExit("no DEL rows produced — check outb presence/load channels")

    out_path = a.out or (targets[0] / "del_summary.csv")
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["case_id", "control", "turbine", "channel", "m", "n_eq",
              "n_cycles", "load_mean", "load_std", "DEL",
              "P_kW_mean", "U_mps_mean"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    print(f"-> {out_path} ({len(all_rows)} rows)")
    # 汇总：每 case×turbine 平均 DEL（跨通道），供快速 Pareto
    import collections
    by_ct = collections.defaultdict(list)
    for r in all_rows:
        by_ct[(r["case_id"], r["control"], r["turbine"])].append(r["DEL"])
    print("per case×turbine mean DEL (kN·m/N 尺度随通道不同，仅示意排序):")
    for k in sorted(by_ct):
        print(f"  {k[0]}_{k[1]} T{k[2]}: {np.mean(by_ct[k]):.3e}")


if __name__ == "__main__":
    main()
