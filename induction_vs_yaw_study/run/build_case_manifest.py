"""build_case_manifest.py — 生成 case_manifest.csv（开发计划 §5.4）

枚举 (U x TI x seed x WD x S x control) -> case_manifest.csv：
    case_id,U,TI,seed,WD,spacing_D,control,bts_file,status
status: pending（盒就绪，可跑）/ blocked（盒缺失或 QA 未过）/ qa_failed

用法：
    python -m induction_vs_yaw_study.run.build_case_manifest
    python -m induction_vs_yaw_study.run.build_case_manifest --require-qa \
        --out induction_vs_yaw_study/results/_manifests/case_manifest.csv
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import yaml

from induction_vs_yaw_study.cases.three_nrel5mw import (
    load_grid, iter_seeded_cases, resolve_bts_file,
)

STUDY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = STUDY_ROOT / "config" / "wind_conditions.yaml"
DEFAULT_OUT = STUDY_ROOT / "results" / "_manifests" / "case_manifest.csv"
QA_CSV = STUDY_ROOT / "FarmInputs" / "qa" / "wind_box_metrics.csv"
CONTROLS = ["baseline", "yaw", "induction"]


def _qa_pass_set() -> set:
    """已通过 QA 的盒文件集合（basename，去掉 coarse/ 前缀）。"""
    if not QA_CSV.exists():
        return set()
    ok = set()
    with open(QA_CSV, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if str(row.get("pass", "")).strip().lower() == "true":
                ok.add(Path(row["file"]).name)
    return ok


def main():
    ap = argparse.ArgumentParser(description="Build FAST.Farm case manifest (V2 seed-key)")
    ap.add_argument("--grid", type=Path, default=None, help="cases/grid.yaml 路径")
    ap.add_argument("--controls", nargs="+", default=CONTROLS,
                    choices=CONTROLS, help="每个工况的控制类型")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--require-qa", action="store_true",
                    help="只放行已过 QA 的盒（默认仅要求盒文件存在）")
    args = ap.parse_args()

    grid = load_grid(str(args.grid) if args.grid else None)
    qa_ok = _qa_pass_set() if args.require_qa else None

    rows = []
    for case in iter_seeded_cases(grid):
        bts = resolve_bts_file(
            case.wind_speed_ms, case.turbulence_intensity, case.seed_id,
            require_exists=True,
        )
        if bts is None:
            status = "blocked"
        elif qa_ok is not None and Path(bts).name not in qa_ok:
            status = "qa_failed"
        else:
            status = "pending"
        for ctrl in args.controls:
            rows.append({
                "case_id": case.id,
                "U": case.wind_speed_ms,
                "TI": case.turbulence_intensity,
                "seed": case.seed_id,
                "WD": case.wind_direction_offset_deg,
                "spacing_D": case.spacing_D,
                "control": ctrl,
                "bts_file": bts or "",
                "status": status,
            })

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else
                           ["case_id", "U", "TI", "seed", "WD",
                            "spacing_D", "control", "bts_file", "status"])
        w.writeheader()
        for r_ in rows:
            w.writerow(r_)

    n_pend = sum(1 for r_ in rows if r_["status"] == "pending")
    print(f"case_manifest: {len(rows)} rows -> {args.out}")
    print(f"  pending={n_pend} blocked={len(rows)-n_pend}"
          + (f" (qa_failed={sum(1 for r_ in rows if r_['status']=='qa_failed')})"
             if args.require_qa else ""))


if __name__ == "__main__":
    main()
