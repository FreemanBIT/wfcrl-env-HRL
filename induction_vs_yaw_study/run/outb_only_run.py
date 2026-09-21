"""outb_only_run.py — FAST.Farm 乙模式运行：无 ZMQ 客户端 -> 全机 .outb（DEL/载荷通道）

背景（根因实证，2026-09-16）：
  * ZMQ 客户端在线（引擎轮询状态）时，本集成 FAST.Farm 只写 Case.T1.outb
    （T2/T3 缺失；客户端早闭还会触发 0xC0000005 崩溃）；
  * 无客户端直接运行 exe -> 正常收尾且 Case.T1/T2/T3.outb 全部写出。
  因此载荷/DEL 采集使用本"乙模式"（直接运行 + 解析 T1..Tn .outb）。
  功率统计仍用 ZMQ 模式（全机状态帧在线）。

用法：
    python -m induction_vs_yaw_study.run.outb_only_run         --u 8 --ti 0.10 --seed 1 --wd 0 --spacing 4 --t-sim 60 --out <dir>
产出：<out>/outb/T{i}.csv（原始高频）+ <out>/outb/outb_summary.csv（均值/统计）
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config


def _parse_outb(outb_path: Path):
    from openfast_toolbox.io.fast_output_file import FASTOutputFile
    f = FASTOutputFile(str(outb_path))
    return f.toDataFrame()


def run_case(case: Case, out_dir: Path, wind_time_series: str,
             exe: str, dt_low: float = 3.0) -> dict:
    """直接运行 FAST.Farm（无 ZMQ 客户端），解析全部 turbine .outb。"""
    cfg = make_fastfarm_config(case, output_dir=str(out_dir))
    w0 = cfg.wind
    from wfcrl.config import WindConfig
    from wfcrl.config.types import WindType
    cfg.wind = WindConfig(speed=w0.speed, direction=w0.direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=wind_time_series)
    from wfcrl.engine import ContinuousFastFarmInterface
    ff = ContinuousFastFarmInterface(cfg)
    ff.setup()
    ff.reset(cfg.wind)   # 与 FastFarmZmqEngine.setup 一致：case 文件在此生成
    # Phase 7：DEL 载荷通道（幂等，乙模式 .outb 才是载荷统计口径）
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        add_load_channels(str(ff._farm_base), verbose=False)
    except Exception as _e:
        print(f"[warn] load channels: {_e}")
    farm_inputs = Path(ff._farm_base)  # 已是 .../FarmInputs
    env = dict(os.environ)
    env["FCR_CFG_DT_LOW"] = str(dt_low)
    t0 = time.time()
    proc = subprocess.run(
        [exe, "Case.fstf"], cwd=str(farm_inputs),
        env=env, capture_output=True, text=True, timeout=None,
    )
    wall = time.time() - t0
    rows = []
    for i in range(1, int(getattr(case, "n_turbines", 3)) + 1):
        ob = farm_inputs / f"Case.T{i}.outb"
        row = {"turbine": i, "outb": ob.name, "bytes": ob.stat().st_size if ob.exists() else 0,
               "ok": ob.exists() and ob.stat().st_size > 0}
        if row["ok"]:
            df = _parse_outb(ob)
            pcol = next((c for c in df.columns if "GenPwr" in c), None)
            wcol = next((c for c in df.columns if c.startswith("Wind1VelX")), None)
            row["P_kW_mean"] = float(df[pcol].mean()) if pcol else float("nan")
            row["P_kW_std"] = float(df[pcol].std(ddof=1)) if pcol else float("nan")
            row["U_mps_mean"] = float(df[wcol].mean()) if wcol else float("nan")
            row["n_rows"] = len(df)
            # 保存原始高频
            (out_dir / "outb").mkdir(parents=True, exist_ok=True)
            df.to_csv(out_dir / "outb" / f"T{i}.csv", index=False)
        rows.append(row)
    rows.append({"turbine": 0, "outb": "run", "bytes": 0, "ok": proc.returncode == 0,
                 "P_kW_mean": float("nan"), "P_kW_std": float("nan"),
                 "U_mps_mean": float("nan"), "n_rows": 0,
                 "rc": proc.returncode, "wall_s": round(wall, 1)})
    summary = out_dir / "outb" / "outb_summary.csv"
    summary.parent.mkdir(parents=True, exist_ok=True)
    with open(summary, "w", newline="", encoding="utf-8") as f:
        keys = ["turbine", "outb", "bytes", "ok", "P_kW_mean", "P_kW_std",
                "U_mps_mean", "n_rows", "rc", "wall_s"]
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r_ in rows:
            w.writerow(r_)
    print(f"[outb_only] rc={proc.returncode} wall={wall:.0f}s -> {summary}")
    for r_ in rows:
        if r_.get("turbine", 0) > 0:
            print(f"  T{r_['turbine']}: P_mean={r_.get('P_kW_mean', 'n/a')} "
                  f"rows={r_.get('n_rows', 0)}")
    return {"rc": proc.returncode, "rows": rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--u", required=True, type=float)
    ap.add_argument("--ti", type=float, default=0.10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--wd", type=float, default=0.0)
    ap.add_argument("--spacing", type=float, default=4.0)
    ap.add_argument("--t-sim", type=float, default=600.0)
    ap.add_argument("--control-dt", type=float, default=3.0)
    ap.add_argument("--wind-time-series", default=None)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--exe", default=None)
    args = ap.parse_args()

    if args.wind_time_series is None:
        from induction_vs_yaw_study.cases.three_nrel5mw import resolve_bts_file
        bts = resolve_bts_file(args.u, args.ti, args.seed)
        if bts is None:
            raise SystemExit("bts 缺失：" + resolve_bts_file(args.u, args.ti, args.seed))
    else:
        bts = args.wind_time_series

    exe = args.exe or os.environ.get(
        "FAST_FARM_EXE",
        str(_STUDY_ROOT.parent / "wfcrl" / "simulators" / "fastfarm" / "bin" /
            "FAST.Farm_x64_OMP.exe"),
    )
    case = Case(args.u, args.wd, args.ti, args.spacing, seed_id=args.seed,
                n_turbines=3, control_dt_s=args.control_dt,
                t_sim_s=args.t_sim, t_settle_s=0.0)
    res = run_case(case, args.out, bts, exe, dt_low=args.control_dt)
    return 0 if res["rc"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
