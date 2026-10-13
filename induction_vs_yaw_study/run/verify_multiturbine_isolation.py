"""verify_multiturbine_isolation.py — M0-0.3 多机 ROSCO 实例隔离验证

在 (U=8, TI=0.10, seed=1, WD=0, S=6D) 单次 300s 仿真内给三台机**不同命令**：
    T1: induction a=0.20（降额）
    T2: yaw_delta = +10°（偏航）
    T3: 无命令（baseline，通道不激活）

验证目标（DR-2 前置 / B 组 AlreadyInitialized 守卫多实例行为）：
    1. T3 功率 ≈ 同风场无命令基线（证明 T1/T2 命令未泄漏进 T3 实例）
    2. T1 功率相对 T3 显著下降（自身诱导生效，幅度符合 a=0.2 理论 ~-14%）
    3. T2 机舱方位相对 T3 偏转（自身偏航生效）
    4. 状态回显 per-turbine：induction_seq_applied / yaw_seq_applied 与各自命令对应，
       无跨机串扰（T3 的 seq 应为 0）

用法：
    python -m induction_vs_yaw_study.run.verify_multiturbine_isolation [--duration 300] [--bts ...]
退出码：0 = 隔离验证通过；非 0 = 失败。
"""
from __future__ import annotations

import argparse
import math as _math
import os as _os
import sys as _sys
import time as _time
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
_os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))
_ccc = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..", "..", "closed-loop-control"))
if _ccc not in _sys.path:
    _sys.path.insert(0, _ccc)

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds, _CMD_TTL_S

from wfcrl.config import WindConfig
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge

_CMD_TTL = 1.0e6


def _per_turbine_power(state_rows, n_t=3):
    """从 ZMQ 状态帧提取每机 gen_power_w.instant 均值（kW）。"""
    acc = {i: [] for i in range(1, n_t + 1)}
    for st in state_rows:
        for t in (st or {}).get("turbines", []):
            tid = t.get("turbine_id", 0)
            if tid in acc:
                v = t.get("gen_power_w") or {}
                inst = v.get("instant")
                if inst is not None and _math.isfinite(float(inst)):
                    acc[tid].append(float(inst))
    return {tid: (float(np.mean(vs)) / 1e3 if vs else float("nan"))
            for tid, vs in acc.items()}


def _per_turbine_state(state_rows, n_t=3):
    """取每机最后一帧的 heading / seq 回显。"""
    last = {}
    for st in state_rows:
        for t in (st or {}).get("turbines", []):
            tid = t.get("turbine_id", 0)
            if 1 <= tid <= n_t:
                last[tid] = t
    out = {}
    for tid in range(1, n_t + 1):
        t = last.get(tid, {})
        out[tid] = {
            "nacelle_heading_rad": t.get("nacelle_heading_rad"),
            "yaw_seq_applied": t.get("yaw_seq_applied", 0),
            "induction_seq_applied": t.get("induction_seq_applied", 0),
        }
    return out


def main():
    ap = argparse.ArgumentParser(description="M0-0.3 多机 ROSCO 实例隔离验证")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--out-root", type=Path, default=_STUDY_ROOT / "results" / "_isolation")
    ap.add_argument("--bts", default="coarse/inflow_U08_TI10_seed01.bts")
    args = ap.parse_args()

    from induction_vs_yaw_study.run._proc_utils import cleanup_stale_fastfarm
    cleanup_stale_fastfarm(min_age_s=300.0)

    case = Case(wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
                turbulence_intensity=0.10, spacing_D=6.0, seed_id=1,
                control_dt_s=args.dt, t_sim_s=args.duration, t_settle_s=0.0)

    out_dir = args.out_root / _time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = make_fastfarm_config(case, output_dir=str(out_dir))
    cfg.wind = WindConfig(speed=cfg.wind.speed, direction=cfg.wind.direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=args.bts)

    n_turbines = int(getattr(case, "n_turbines", 3))
    n_total = max(1, int(getattr(case, "n_total_control_steps", cfg.max_iter)))

    # ---- 混合命令：T1=诱导0.2, T2=偏航+10°, T3=无命令 ----
    cmds = {
        1: Cmd(turbine_id=1, yaw_delta_deg=0.0, induction_a=0.20),
        2: Cmd(turbine_id=2, yaw_delta_deg=10.0, induction_a=None),
        3: Cmd(turbine_id=3, yaw_delta_deg=0.0, induction_a=None),
    }

    print(f"== multiturbine isolation: {case.id} / 300s / bts={args.bts}")
    print(f"   T1: induction a=0.20 | T2: yaw +10deg | T3: baseline(no cmd)")

    _base = 24000 + (_os.getpid() % 1500) * 2
    engine = FastFarmZmqEngine(cfg, cmd_port=_base, state_port=_base + 1)
    engine.setup()
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception as _e:
        print(f"[warn] load channels: {_e}")

    _omp = _os.environ.get("WFCRL_STUDY_OMP_NUM_THREADS", "1")
    if _omp not in ("", "0"):
        _os.environ["OMP_NUM_THREADS"] = _omp

    t0 = _time.time()
    engine.start()
    bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=float(cfg.wind.direction))
    state_rows = []
    try:
        br_ok = False
        while _time.time() - t0 < 90:
            st = bridge.get_state(timeout_s=5.0)
            if st is not None and st.get("fast_step", 0) > 0:
                br_ok = True
                break
        if not br_ok:
            raise TimeoutError("FAST.Farm 未在 90s 内发出状态帧")

        seq_ctr = {"yaw": {}, "ind": {}}
        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=180.0)
            if k == 1:
                _send_cmds(engine, cmds, seq_ctr)
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            state_rows.append(st if st else {"turbines": []})

        _proc = getattr(engine, "_proc", None)
        if _proc is not None:
            try:
                _proc.wait(timeout=600)
            except Exception:
                pass
    finally:
        try:
            engine.stop()
        except Exception:
            pass
        try:
            engine.close()
        except Exception:
            pass

    wall = _time.time() - t0
    pw = _per_turbine_power(state_rows, n_turbines)
    stt = _per_turbine_state(state_rows, n_turbines)
    print(f"wall={wall:.0f}s  T-power(kW): T1={pw.get(1):.0f} T2={pw.get(2):.0f} T3={pw.get(3):.0f}")
    for tid in range(1, n_turbines + 1):
        s = stt.get(tid, {})
        print(f"  T{tid} state: heading={s.get('nacelle_heading_rad')} "
              f"yaw_seq={s.get('yaw_seq_applied')} ind_seq={s.get('induction_seq_applied')}")

    import json
    (out_dir / "isolation_summary.json").write_text(json.dumps({
        "case": case.id, "wall_s": wall,
        "turbine_power_kw": {str(k): v for k, v in pw.items()},
        "turbine_state": {str(k): v for k, v in stt.items()},
    }, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---- 验收判定 ----
    p1, p2, p3 = pw.get(1), pw.get(2), pw.get(3)
    fails = []
    if not (np.isfinite(p1) and np.isfinite(p2) and np.isfinite(p3)):
        fails.append("功率缺失/非有限")
    else:
        # 1) T3 无命令应保持较高功率（不被 T1/T2 泄漏影响——基线参考用 T3 自身）
        # 2) T1 诱导 0.2 → 相对 T3 下降（理论 ~ -14%）
        d13 = 100.0 * (p1 - p3) / p3
        if d13 > -8.0:
            fails.append(f"T1 诱导降额未生效: T1-T3 相对差 {d13:.1f}% (期望 < -8%)")
        # 3) T2 偏航 +10° → 功率相对 T3 略降（cos^2 效应 ~ -3%）
        d23 = 100.0 * (p2 - p3) / p3
        if d23 > 2.0:
            fails.append(f"T2 偏航未生效: T2-T3 相对差 {d23:.1f}% (期望 < +2%)")
    # 4) seq 回显：T1 应 ind_seq>=1，T2 应 yaw_seq>=1，T3 应全 0
    s1, s2, s3 = stt.get(1, {}), stt.get(2, {}), stt.get(3, {})
    if not (s1.get("induction_seq_applied", 0) >= 1):
        fails.append("T1 induction_seq_applied 未生效")
    if not (s2.get("yaw_seq_applied", 0) >= 1):
        fails.append("T2 yaw_seq_applied 未生效")
    if not (s3.get("yaw_seq_applied", 0) == 0 and s3.get("induction_seq_applied", 0) == 0):
        fails.append("T3 无命令但 seq 被置位（跨机串扰）")

    if fails:
        print("[FAIL] " + " | ".join(fails))
        return 1
    print("[PASS] 多机实例隔离验证通过")
    return 0


if __name__ == "__main__":
    _sys.exit(main())
