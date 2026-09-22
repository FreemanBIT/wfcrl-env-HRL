"""_calib_worker.py — 1T induction 标定单点（子进程，崩溃隔离）

给定 (U, a_cmd)：1T FAST.Farm 300s baseline-降额回放（P_target=power_from_induction(a)），
从 ZMQ 状态取 P / Ct / Cp / Pitch / RotorSpeed 均值，append 到 operating map csv。

用法：python -m induction_vs_yaw_study.run._calib_worker --u 8.0 --a 0.2 --map <csv>
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

import numpy as np

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.conversion.power_setpoint_tools import power_from_induction
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--u", required=True, type=float)
    ap.add_argument("--a", required=True, type=float)
    ap.add_argument("--map", required=True, type=Path)
    ap.add_argument("--ti", type=float, default=0.10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--wind-time-series", default=None)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--settle", type=float, default=120.0,
                    help="统计窗口起点（s）：剔除降载过渡段，只平均 [settle, duration]"
                         "（旧口径全窗平均是 map 作废原因之一，2026-09-21）")
    args = ap.parse_args()

    if args.wind_time_series is None:
        from induction_vs_yaw_study.cases.three_nrel5mw import resolve_bts_file
        bts = resolve_bts_file(args.u, args.ti, args.seed, require_exists=False) or \
            f"coarse/inflow_U{int(round(args.u)):02d}_TI{int(round(args.ti*100)):02d}_seed{args.seed:02d}.bts"
    else:
        bts = args.wind_time_series

    p_target_mw = power_from_induction(float(args.a), args.u, electrical=True) / 1e6
    case = Case(args.u, 0.0, args.ti, 500.0, seed_id=int(args.seed), n_turbines=1,
                control_dt_s=3.0, t_sim_s=args.duration, t_settle_s=float(args.settle))
    t0 = time.time()
    out = replay_fastfarm_zmq(case, "derating",
                              {1: {"power_target_mw": p_target_mw}},
                              output_dir=str(_STUDY_ROOT / "results" / "_calib_ind" /
                                             f"U{int(round(args.u)):02d}_a{int(round(args.a*100)):03d}"),
                              wind_time_series=bts, yaw_mode="static")
    wall = time.time() - t0
    if out is None or out.power_mw is None:
        print(f"FAIL U{args.u} a{args.a} no-output"); return 1
    pmw = np.asarray(out.power_mw, dtype=float)
    # 显式窗口 [settle, duration]：只统计稳态降载段（重做口径，2026-09-21）
    t_vec = (np.arange(pmw.shape[0]) + 1) * 3.0   # 控制步 -> 时间（control_dt=3s）
    win = t_vec >= float(args.settle)
    if not win.any():
        win = slice(None)   # 防御：窗口为空则退回全窗
        p_kw = float(np.nanmean(pmw.sum(axis=1))) * 1e3
    else:
        p_kw = float(np.nanmean(pmw[win].sum(axis=1))) * 1e3
    # Ct/Cp 从保存的 ZMQ 状态取均值
    ct = cp = pitch = rps = float("nan")
    sp = _STUDY_ROOT / "results" / "_calib_ind" / \
        f"U{int(round(args.u)):02d}_a{int(round(args.a*100)):03d}" / "zmq_state_rows.json"
    if sp.exists():
        rows = json.loads(sp.read_text(encoding="utf-8"))
        cts, cps, pits, rpsl = [], [], [], []
        for i, st in enumerate(rows):
            t_row_time = (i + 1) * 3.0
            if t_row_time < float(args.settle):
                continue   # 过渡段剔除（与功率窗口同口径）
            for t in st:
                if t.get("turbine_id") == 1:
                    if t.get("rotor_ct") is not None: cts.append(t["rotor_ct"])
                    if t.get("rotor_cp") is not None: cps.append(t["rotor_cp"])
                    bp = t.get("blade_pitch_rad")
                    if bp: pits.append(float(np.mean(bp)) * 180 / np.pi)
                    if t.get("rotor_speed_rad_s") is not None: rpsl.append(t["rotor_speed_rad_s"] * 60 / (2 * np.pi))
        ct = float(np.nanmean(cts)) if cts else float("nan")
        cp = float(np.nanmean(cps)) if cps else float("nan")
        pitch = float(np.nanmean(pits)) if pits else float("nan")
        rps = float(np.nanmean(rpsl)) if rpsl else float("nan")

    write_header = not args.map.exists()
    with open(args.map, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["U", "a_cmd", "P_kW", "Ct", "Cp",
                                          "Pitch_deg", "RotorSpeed_rpm", "wall_s"])
        if write_header:
            w.writeheader()
        w.writerow({"U": args.u, "a_cmd": args.a, "P_kW": f"{p_kw:.2f}",
                    "Ct": f"{ct:.4f}", "Cp": f"{cp:.4f}",
                    "Pitch_deg": f"{pitch:.3f}", "RotorSpeed_rpm": f"{rps:.2f}",
                    "wall_s": f"{wall:.0f}"})
    print(f"OK U{args.u} a{args.a} P={p_kw:.1f} kW Ct={ct:.3f} Cp={cp:.3f} wall={wall:.0f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
