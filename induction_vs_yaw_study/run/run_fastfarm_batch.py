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
        control_dt_s=float(ff_cfg.get("control_dt_s", ff_cfg.get("dt", 3.0))),
        t_sim_s=float(ff_cfg.get("t_sim_s", 1200.0)),
        t_settle_s=float(ff_cfg.get("t_settle_s", 600.0)),
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
                        ("feathered" if res.get("feathered")
                         else ("done" if res["ok"] else "failed")),
                        res.get("error", ""), round(wall_s, 1)])


def _run_one(row, lut, ff_cfg, out_dir: Path, ports) -> dict:
    """以**子进程**跑单个 case（_case_worker）。

    libzmq 致命 assert（Connection reset / Socket op on non-socket）会杀死进程；
    子进程隔离保证批跑父进程与其他 case 不受影响（全 DOE 可靠性）。
    端口默认自动（replay 按 PID 派生），无需显式分配。
    """
    import json as _json
    import subprocess as _sp
    row_path = out_dir / "_rows" / (f"{row['case_id']}_{row['control']}.json")
    row_path.parent.mkdir(parents=True, exist_ok=True)
    row_path.write_text(_json.dumps(row), encoding="utf-8")
    t0 = time.time()
    cmd = [
        sys.executable, "-m", "induction_vs_yaw_study.run._case_worker",
        "--row", str(row_path), "--out", str(out_dir),
        "--experiment", str(args_exp), "--lut", str(args_lut),
    ]
    try:
        done = _sp.run(cmd, capture_output=True, text=True, timeout=None)
        out_txt = (done.stdout or "") + (done.stderr or "")
        wall = time.time() - t0
        # P2 偶发顺桨：worker 打 FEATHERED 标记 → status=feathered（非 done，
        # resume 自动重试；汇总区分统计）。ok=True 让父进程继续写状态行。
        if done.returncode == 0 and "FEATHERED" in out_txt:
            tail = out_txt.strip().splitlines()
            err = "feathered: " + " | ".join(
                ln for ln in tail if ln.startswith("FEATHERED"))[:280]
            return {"ok": True, "error": err, "wall_s": wall, "feathered": True}
        if done.returncode == 0 and "OK " in out_txt:
            return {"ok": True, "error": "", "wall_s": wall}
        # 失败时打日志片段便于排查
        tail = out_txt.strip().splitlines()
        err = " | ".join(tail[-3:])[:300] if tail else f"rc={done.returncode}"
        return {"ok": False, "error": err, "wall_s": wall}
    except Exception as e:  # noqa
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
    ap.add_argument("--resume", action="store_true",
                    help="跳过 status=done 的行（对 failed 行重试）")
    ap.add_argument("--experiment", type=Path, default=EXPERIMENT_YAML)
    ap.add_argument("--lut", type=Path, default=LUT_CSV)
    ap.add_argument("--controls", nargs="+", default=CONTROLS, choices=CONTROLS)
    args = ap.parse_args()

    ff_cfg = yaml.safe_load(open(args.experiment, encoding="utf-8")).get("fastfarm", {})
    # Pilot 吞吐：从 experiment.yaml 读 OMP 线程数并注入父 env，使全部 worker 的
    # replay（engine.start 前读 WFCRL_STUDY_OMP_NUM_THREADS）继承；显式 env 优先。
    _exp_all = yaml.safe_load(open(args.experiment, encoding="utf-8"))
    _omp_cfg = ((_exp_all or {}).get("batch") or {}).get("omp_threads")
    if _omp_cfg is not None and not os.environ.get("WFCRL_STUDY_OMP_NUM_THREADS"):
        os.environ["WFCRL_STUDY_OMP_NUM_THREADS"] = str(_omp_cfg)
    out_dir = args.out or (_STUDY_ROOT / "results" / ("run_%s" % time.strftime("%Y%m%d_%H%M%S")))
    out_dir.mkdir(parents=True, exist_ok=True)

    # P2-1：run 级 provenance
    from induction_vs_yaw_study.run.provenance import write_run_provenance
    write_run_provenance(out_dir, extra={
        "manifest": str(args.manifest), "workers": args.workers,
        "max_retries": args.max_retries, "filter": args.filter,
    })

    global args_exp, args_lut
    args_exp, args_lut = args.experiment, args.lut
    lut = None  # LUT 由 _case_worker 子进程按 --lut 自行加载

    # manifest
    with open(args.manifest, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if args.controls:
        rows = [r_ for r_ in rows if r_["control"] in args.controls]
    if args.filter:
        import pandas as _pd
        mf = _pd.DataFrame(rows)
        # manifest 列是字符串；把数值列转 float 让 pandas query 数值比较生效
        for _c in ("U", "TI", "WD", "spacing_D", "seed"):
            if _c in mf.columns:
                mf[_c] = mf[_c].astype(float)
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
    if args.resume:
        todo = [r_ for r_ in rows if (r_["case_id"], r_["control"]) not in done]
    else:
        todo = [r_ for r_ in rows if r_["status"] != "done"]
    print(f"to run: {len(todo)} (skipping {len(rows)-len(todo)} done/failed)")
    if not todo:
        print("nothing to run")
        return

    import threading
    global _STATUS_LOCK
    _STATUS_LOCK = threading.Lock()

    def run_one_with_retry(j, lut_local, ff_cfg_local):
        res = {"ok": False, "error": "unrun", "wall_s": 0.0}
        for attempt in range(max(1, args.max_retries + 1)):
            res = _run_one(j, lut_local, ff_cfg_local, out_dir, None)
            if res["ok"]:
                break
            print(f"  [{j['case_id']}/{j['control']}] attempt{attempt+1} FAILED: "
                  f"{res.get('error','')[:120]}", flush=True)
        _write_status_row(status_log, j, res, res.get("wall_s", 0.0))
        print(f"  [{j['case_id']}/{j['control']}] {'ok' if res['ok'] else 'FAILED'} "
              f"({res.get('wall_s', 0):.0f}s)", flush=True)
        return j, res

    results = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futs = [pool.submit(run_one_with_retry, j, lut, ff_cfg) for j in todo]
        for fut in futs:
            results.append(fut.result())

    n_ok = sum(1 for _, res in results if res["ok"])
    n_feathered = sum(1 for _, res in results if res.get("feathered"))
    print(f"done: {n_ok}/{len(results)} "
          f"(feathered={n_feathered}, see status col); "
          f"incremental status -> {status_log}")


if __name__ == "__main__":
    main()
