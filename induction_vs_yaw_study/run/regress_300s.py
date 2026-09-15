"""regress_300s.py — 300s 仿真回归门禁（开发计划 §11 红线）

在 study ZMQ 链路上跑 300s 贪婪基线（3T, U=8, TI=0.10, seed=1, WD=0, S=6D），
作为"每步代码修改后的 300s 仿真回归"（旧命令依赖的 inflow_08ms_TI10_s8D.bts
将被删除，改用新引导盒 inflow_U08_TI10_seed01，约束 3）。

输出：study/results/_regress/{ts}/（含 fastfarm_zmq.log / 时序 CSV / 均值摘要）
退出码：0 = 通过（有输出 + 有限均值）；非 0 = 失败。

用法：
    python -m induction_vs_yaw_study.run.regress_300s
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# 必须在导入 wfcrl.engine 之前设置（wfcrl/engine/_ff_case.py 模块级读取）
_STUDY_ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault(
    "WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"),
)

from induction_vs_yaw_study.cases.three_nrel5mw import Case, resolve_bts_file
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq


def main():
    ap = argparse.ArgumentParser(description="300s FAST.Farm regression gate (study chain)")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--out-root", type=Path,
                    default=_STUDY_ROOT / "results" / "_regress")
    ap.add_argument("--wind-time-series", default=None,
                    help="覆盖引导盒（默认 resolve U8/TI0.10/seed1）")
    args = ap.parse_args()

    case = Case(
        wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
        turbulence_intensity=0.10, spacing_D=6.0, seed_id=1,
        fastfarm_dt=args.dt, fastfarm_t_settle_s=0.0,
        fastfarm_n_control_steps=int(round(args.duration / args.dt)),
    )
    bts = args.wind_time_series or resolve_bts_file(8.0, 0.10, 1)
    if bts is None:
        print(f"[FAIL] 引导盒缺失：{_STUDY_ROOT / 'FarmInputs' / 'coarse'}"
              f"/inflow_U08_TI10_seed01.bts（先跑 generate_wind_boxes --seeds 1 --speeds 8 --tis 0.10 --run）")
        return 2

    out_dir = args.out_root / time.strftime("%Y%m%d_%H%M%S")
    print(f"== regress_300s: {case.id} / baseline / {args.duration:.0f}s / bts={bts}")
    print(f"   WFCRL_FARMINPUTS_DIR={os.environ['WFCRL_FARMINPUTS_DIR']}")
    t0 = time.time()
    out = replay_fastfarm_zmq(
        case, "baseline", {}, output_dir=str(out_dir),
        wind_time_series=bts, yaw_mode="static",
    )
    wall = time.time() - t0

    if out is None or out.time is None or len(out.time) == 0:
        print(f"[FAIL] 无输出（wall {wall:.1f}s）；日志见 {out_dir / 'fastfarm_zmq.log'}")
        return 1

    import numpy as np
    pmw = np.asarray(out.power_mw, dtype=float)
    mean_farm = float(np.nanmean(pmw.sum(axis=1) * 1e3 if pmw.ndim > 1 else pmw * 1e3))
    ok = bool(np.isfinite(mean_farm) and mean_farm > 0)
    csv_path = out_dir / "regress_300s_timeseries.csv"
    try:
        out.to_csv(str(csv_path))
    except Exception:
        pass
    print(f"[{'PASS' if ok else 'FAIL'}] wall={wall:.1f}s steps={len(out.time)} "
          f"mean_farm_power={mean_farm:.0f} kW -> {out_dir}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
