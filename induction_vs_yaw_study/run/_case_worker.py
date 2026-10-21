"""_case_worker.py — 单 case 子进程 worker（批跑可靠性）

由 run_fastfarm_batch 以子进程方式调用：libzmq 的致命 assert（Connection reset /
Socket op on non-socket）只会杀死本进程，不影响批跑父进程与其他 case。

用法（也可手动调试）：
    python -m induction_vs_yaw_study.run._case_worker \
        --row <row.json> --out <run_dir> --experiment <exp.yaml> --lut <luts.csv>
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq


def _lut_settings(lut, case_id: str, control_type: str):
    from induction_vs_yaw_study.lut.interpolate import _rows_to_dict
    lut_ctype = "derating" if control_type == "induction" else control_type
    if control_type == "baseline":
        return {i: {"yaw_deg": 0.0, "power_target_mw": float("nan"),
                    "ratio": 1.0, "min_pitch_deg": 0.0,
                    "greedy_power_w": float("nan")} for i in (1, 2, 3)}
    rows = lut[(lut["case_id"] == case_id) & (lut["control_type"] == lut_ctype)]
    if rows.empty:
        raise ValueError(f"No LUT rows for {case_id}/{control_type}")
    return _rows_to_dict(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--row", required=True, type=Path, help="case 行 JSON")
    ap.add_argument("--out", required=True, type=Path, help="run 目录")
    ap.add_argument("--experiment", type=Path,
                    default=_STUDY_ROOT / "config" / "experiment.yaml")
    ap.add_argument("--lut", type=Path, default=None)
    args = ap.parse_args()

    row = json.loads(args.row.read_text(encoding="utf-8"))
    ff_cfg = {}
    if args.experiment.exists():
        import yaml
        ff_cfg = yaml.safe_load(args.experiment.read_text(encoding="utf-8")).get("fastfarm", {})

    lut = None
    if args.lut and Path(args.lut).exists():
        import pandas as pd
        try:
            lut = pd.read_parquet(args.lut)
        except Exception:
            lut = pd.read_csv(args.lut)

    ctype = "derating" if row["control"] == "induction" else row["control"]
    settings = _lut_settings(lut, row["case_id"], ctype) if lut is not None else {}

    case = Case(
        wind_speed_ms=float(row["U"]),
        wind_direction_offset_deg=float(row["WD"]),
        turbulence_intensity=float(row["TI"]),
        spacing_D=float(row["spacing_D"]),
        seed_id=int(row["seed"]),
        control_dt_s=float(ff_cfg.get("control_dt_s", ff_cfg.get("dt", 3.0))),
        t_sim_s=float(ff_cfg.get("t_sim_s", 1200.0)),
        t_settle_s=float(ff_cfg.get("t_settle_s", 600.0)),
    )
    if not row.get("bts_file"):
        print("FAIL no-bts")
        return 1

    case_dir = Path(args.out) / "cases" / (f"{row['case_id']}_" + row["control"])
    case_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    try:
        out = replay_fastfarm_zmq(
            case, ctype, settings, output_dir=str(case_dir),
            wind_time_series=row["bts_file"], yaw_mode="static",
        )
        if out is None or out.time is None or len(out.time) == 0:
            print("FAIL no-output")
            return 1
        # libzmq teardown 崩溃解耦：输出全部落盘后写 DONE 哨兵。
        # 批跑父进程据此判成功（即使进程在 Python 解释器关闭时被 libzmq
        # 'Connection reset by peer' assert 杀掉，数据仍完整、不误标 failed）。
        (case_dir / "DONE.sentinel").write_text("ok", encoding="utf-8")
        # P2 偶发顺桨防护：replay 在 metadata 标记非命令性顺桨（pitch>80° &
        # rotor<1rpm 连续 >=30s）。此类 case 的功率统计被不可逆运行点破坏污染，
        # 打 FEATHERED 标记（父进程写 status=feathered，resume 重试；统计侧剔除）。
        md = getattr(out, "metadata", None) or {}
        if md.get("feather_invalid"):
            print(f"FEATHERED {row['case_id']} {row['control']} "
                  f"events={md.get('feather_events')}")
        out.to_csv(str(case_dir / "timeseries.csv"))
        # feather 事件入 case metadata（provenance）
        if md.get("feather_events"):
            try:
                import json as _json
                _mp = case_dir / "feather_events.json"
                _mp.write_text(_json.dumps(md["feather_events"], indent=1),
                               encoding="utf-8")
            except Exception:
                pass
        # P2-1：case 级 metadata
        try:
            from induction_vs_yaw_study.run.provenance import write_case_metadata
            write_case_metadata(case_dir, row, ff_cfg)
        except Exception:
            pass
        print(f"OK {row['case_id']} {row['control']} wall={time.time()-t0:.0f}s")
        return 0
    except Exception as e:  # noqa
        import traceback
        traceback.print_exc()
        print(f"FAIL {row['case_id']} {row['control']} {repr(e)[:200]}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
