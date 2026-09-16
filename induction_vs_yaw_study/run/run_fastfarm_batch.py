"""run_fastfarm_batch.py — manifest 驱动的 FAST.Farm 批跑器（开发计划 §6）

对 build_case_manifest.py 生成的 case_manifest.csv 逐 case 经 ZMQ 链路回放：
  baseline（贪婪）/ yaw（LUT 绝对偏航）/ induction（LUT 绝对功率 -> a），
  统计口径 = ZMQ 状态帧（全机），500-1200s 显式窗口由 Case 参数（t_settle/n_control_steps）控制。

特性：resume（跳过 done）/ 失败重试 / --filter 分片 / 并发 worker（端口对隔离）。

用法：
    python -m induction_vs_yaw_study.run.run_fastfarm_batch \
        --manifest induction_vs_yaw_study/results/_manifests/case_manifest.csv \
        --out induction_vs_yaw_study/results/run_test --workers 2 --max-retries 1
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

_STUDY_ROOT = Path(__file__).resolve().parents[1]

# 必须在导入 wfcrl.engine 之前设置
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq

EXPERIMENT_YAML = _STUDY_ROOT / "config" / "experiment.yaml"
LUT_CSV = _STUDY_ROOT / "results" / "luts" / "luts.csv"
CONTROLS = ["baseline", "yaw", "induction"]


def _lut_settings(lut, case_id: str, control_type: str):
    """从 LUT 取该工况该控制的三机设定 dict（与 run_replay._settings_for 同构）。

    注意：manifest 的 control 列用 "induction"，LUT 的 control_type 用 "derating"（同一控制）。
    """
    lut_ctype = "derating" if control_type == "induction" else control_type
    if control_type == "baseline":
        return {i: {"yaw_deg": 0.0, "power_target_mw": float("nan"),
                    "ratio": 1.0, "min_pitch_deg": 0.0,
                    "greedy_power_w": float("nan")} for i in (1, 2, 3)}
    rows = lut[(lut["case_id"] == case_id) & (lut["control_type"] == lut_ctype)]
    if rows.empty:
        raise ValueError(f"No LUT rows for {case_id}/{control_type}")
    from induction_vs_yaw_study.lut.interpolate import _rows_to_dict
    return _rows_to_dict(rows)


def _case_from_row(row: dict, ff_cfg: dict) -> Case:
    return Case(
        wind_speed_ms=float(row["U"]),
        wind_direction_offset_deg=float(row["WD"]),
        turbulence_intensity=float(row["TI"]),
        spacing_D=float(row["spacing_D"]),
        seed_id=int(row["seed"]),
        fastfarm_dt=float(ff_cfg.get("dt", 3.0)),
        fastfarm_t_settle_s=float(ff_cfg.get("t_settle_s", 600.0)),
        fastfarm_n_control_steps=int(ff_cfg.get("n_control_steps", 200)),
    )


_STATUS_LOCK = None  # set in main (threading.Lock)


def _write_status_row(status_path: Path, row, res, wall_s: float) -> None:
    import threading
    with _STATUS_LOCK:
        exists = status_path.exists()
        with open(status_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(["case_id", "control", "status", "error", "wall_s"])
            w.writerow([row["case_id"], row["control"],
                        "done" if res["ok"] else "failed",
                        res.get("error", ""), round(wall_s, 1)])


def _run_one(row, lut, ff_cfg, out_dir: Path, ports) -> dict:
    case = _case_from_row(row, ff_cfg)
    if not row.get("bts_file"):
        return {"ok": False, "error": "no bts_file"}
    settings = _lut_settings(lut, row["case_id"], row["control"])
    case_dir = out_dir / "cases" / (f"{row['case_id']}_" + row["control"])
    case_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    try:
        out = replay_fastfarm_zmq(
            case, row["control"], settings, output_dir=str(case_dir),
            wind_time_series=row["bts_file"], yaw_mode="static",
            cmd_port=ports[0], state_port=ports[1],
        )
        if out is None or out.time is None or len(out.time) == 0:
            return {"ok": False, "error": "no output", "wall_s": time.time() - t0}
        csv_path = case_dir / "timeseries.csv"
        out.to_csv(str(csv_path))
        return {"ok": True, "error": "", "csv": str(csv_path), "wall_s": time.time() - t0}
    except Exception as e:  # noqa
        traceback.print_exc()
        return {"ok": False, "error": repr(e)[:300], "wall_s": time.time() - t0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=None,
                    help="run 目录（默认 study/results/run_<ts>）")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--max-retries", type=int, default=1)
    ap.add_argument("--filter", default=None, help="pandas 式过滤，如 'U==8 & TI==0.10'")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--experiment", type=Path, default=EXPERIMENT_YAML)
    ap.add_argument("--lut", type=Path, default=LUT_CSV)
    ap.add_argument("--controls", nargs="+", default=CONTROLS, choices=CONTROLS)
    args = ap.parse_args()

    ff_cfg = yaml.safe_load(open(args.experiment, encoding="utf-8")).get("fastfarm", {})
    out_dir = args.out or (_STUDY_ROOT / "results" / ("run_%s" % time.strftime("%Y%m%d_%H%M%S")))
    out_dir.mkdir(parents=True, exist_ok=True)

    # LUT
    lut = None
    lut_path = args.lut
    if lut_path.exists():
        import pandas as pd
        try:
            lut = pd.read_parquet(lut_path)
        except Exception:
            lut = pd.read_csv(lut_path)

    # manifest
    with open(args.manifest, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if args.controls:
        rows = [r_ for r_ in rows if r_["control"] in args.controls]
    if args.filter:
        import pandas as _pd
        mf = _pd.DataFrame(rows)
        try:
            keep = mf.query(args.filter).index
            rows = [rows[i] for i in keep]
        except Exception as e:
            print(f"[warn] filter parse failed: {e}; run unfiltered")
    print(f"manifest rows: {len(rows)} -> {out_dir} (workers={args.workers})")

    done = set()
    status_log = out_dir / "manifest_status.csv"
    if status_log.exists() and not args.overwrite:
        with open(status_log, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row.get("status") == "done":
                    done.add((row["case_id"], row["control"]))
    todo = [r_ for r_ in rows
            if (r_["case_id"], r_["control"]) not in done and r_["status"] != "failed"]
    print(f"to run: {len(todo)} (skipping {len(rows)-len(todo)} done/failed)")
    if not todo:
        print("nothing to run")
        return

    import threading
    global _STATUS_LOCK
    _STATUS_LOCK = threading.Lock()

    def worker(port_base, jobs):
        results = []
        for j in jobs:
            ports = (port_base, port_base + 1)
            res = _run_one(j, lut, ff_cfg, out_dir, ports)
            _write_status_row(status_log, j, res, res.get("wall_s", 0.0))
            print(f"  [{j['case_id']}/{j['control']}] {'ok' if res['ok'] else 'FAILED'} "
                  f"({res.get('wall_s', 0):.0f}s)", flush=True)
            results.append((j, res))
        return results

    results = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        chunks = [todo[i::max(1, args.workers)] for i in range(max(1, args.workers))]
        futs = [pool.submit(worker, 5556 + 2 * k, chunks[k]) for k in range(len(chunks))]
        for fut in futs:
            results.extend(fut.result())

    n_ok = sum(1 for _, res in results if res["ok"])
    print(f"done: {n_ok}/{len(results)}; incremental status -> {status_log}")


if __name__ == "__main__":
    main()
