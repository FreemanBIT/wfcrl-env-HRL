"""diag_yaw_effect.py — P1 偏航有效性诊断（短时 3T 用例，全状态帧导出）

目的：判定 yaw 命令链断点。状态帧 FcrFarmTurbineState 已含
  yaw_seq_applied / yaw_target_heading_rad / controller_status_flags，
本脚本逐控制步全量落盘并打印判读表：
  - yaw_seq_applied==0          -> 命令未到达/未锁存（ZMQ 传输或锁存条件问题）
  - yaw_seq_applied>=1 且机舱不动 -> 执行侧问题（ROSCO/ServoDyn/DISCON.IN）
另设 FCR_DIAG_FILE 打开 DLL 内 zmq transport diag（socket 连接结果）。

用法：
    python -m induction_vs_yaw_study.run.diag_yaw_effect [--duration 180] [--yaw 30]
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

_ccc = os.path.abspath(os.path.join(_STUDY_ROOT, "..", "closed-loop-control"))
if _ccc not in sys.path:
    sys.path.insert(0, _ccc)

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import _send_cmds
from wfcrl.config import WindConfig
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
from closedloop.bridge import ZmqFarmBridge
from closedloop.types import Cmd

R2D = 57.29577951308232


def main() -> int:
    ap = argparse.ArgumentParser(description="P1 yaw effectiveness diagnostic")
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--dt", type=float, default=3.0)
    ap.add_argument("--yaw", type=float, default=30.0, help="T1 yaw delta (deg, CW+)")
    ap.add_argument("--spacing", type=float, default=6.0)
    ap.add_argument("--roscolog", action="store_true",
                    help="T1 DISCON.IN LoggingLevel=2（每 10ms 写 Case.T1.RO.dbg2，126 通道）")
    args = ap.parse_args()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = _STUDY_ROOT / "results" / "_diag_yaw" / ts
    out_dir.mkdir(parents=True, exist_ok=True)
    # DLL 内 zmq transport diag（engine.start 把父进程 env 传给 FAST.Farm 子进程）
    os.environ["FCR_DIAG_FILE"] = str(out_dir / "fcr_diag.txt")

    bts = _STUDY_ROOT / "FarmInputs" / "coarse" / "uniform8_700s.bts"
    if not bts.exists():
        print(f"[FAIL] 引导盒缺失: {bts}")
        return 2

    case = Case(
        wind_speed_ms=8.0, wind_direction_offset_deg=0.0,
        turbulence_intensity=0.10, spacing_D=args.spacing, seed_id=1,
        control_dt_s=args.dt, t_sim_s=args.duration, t_settle_s=0.0,
    )
    cfg = make_fastfarm_config(case, output_dir=str(out_dir))
    w0 = cfg.wind
    cfg.wind = WindConfig(speed=w0.speed, direction=w0.direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=str(bts))

    n_turbines = int(getattr(case, "n_turbines", 3))
    n_total = int(getattr(case, "n_total_control_steps",
                          int(args.duration / args.dt)))
    base = 24000 + (os.getpid() % 1500) * 2

    print(f"== diag_yaw_effect: {case.id} / yaw T1 +{args.yaw}deg CW / "
          f"{args.duration:.0f}s / ports {base}+1")
    engine = FastFarmZmqEngine(cfg, cmd_port=base, state_port=base + 1)
    engine.setup()
    # P2：与 replay 同口径固定 OMP（0=不干预）
    _omp = os.environ.get("WFCRL_STUDY_OMP_NUM_THREADS", "1")
    if _omp not in ("", "0"):
        os.environ["OMP_NUM_THREADS"] = _omp
    if args.roscolog:
        import re as _re
        _discon = Path(out_dir) / "FarmInputs" / "DISCON_T1.IN"
        _txt = _discon.read_text(encoding="utf-8", errors="ignore")
        _txt2 = _re.sub(r"^0\s+LoggingLevel", "2        LoggingLevel",
                        _txt, count=1, flags=_re.M)
        if _txt2 == _txt:
            print(f"[warn] LoggingLevel 行未匹配: {_discon}")
        else:
            _discon.write_text(_txt2, encoding="utf-8")
            print(f"[roscolog] {_discon} -> LoggingLevel=2")
    engine.start()
    bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=cfg.wind.direction)

    rows: list = []
    t0 = time.time()
    try:
        br_ok = False
        while time.time() - t0 < 90:
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
                cmds = {1: Cmd(turbine_id=1, yaw_delta_deg=float(args.yaw),
                               induction_a=None)}
                _send_cmds(engine, cmds, seq_ctr)
                print(f"   [cmd] k=1: T1 yaw_delta=+{args.yaw} deg (CW+) sent, ttl=1e6")
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            rows.append(st if st is not None else {"turbines": []})
        proc = getattr(engine, "_proc", None)
        if proc is not None:
            try:
                proc.wait(timeout=120)
            except Exception:
                pass
    except Exception as e:
        import traceback
        print(f"[warn] aborted: {e}")
        traceback.print_exc()
    finally:
        try:
            engine.stop()
        except Exception:
            pass

    # ---- 落盘全量状态 ----
    with open(out_dir / "diag_state_rows.json", "w", encoding="utf-8") as f:
        json.dump(rows, f)
    try:
        stats = engine._client.stats if engine._client else {}
    except Exception:
        stats = {}
    print(f"client stats: {stats}")

    # ---- 判读表 ----
    print(f"{'k':>3} {'t_sim':>6} | T1: seq tgt_deg nac_deg p_kw flags | "
          f"T2 nac_deg | T3 nac_deg")
    step_every = 5 if n_total > 100 else 1
    for i, st in enumerate(rows, start=1):
        if i % step_every and i != n_total:
            continue
        tsim = st.get("sim_time_s", i * args.dt)
        cells = []
        for tid in (1, 2, 3):
            turbs = st.get("turbines", [])
            t = turbs[tid - 1] if tid <= len(turbs) else {}
            seq = int(t.get("yaw_seq_applied", 0) or 0)
            tgt = float(t.get("yaw_target_heading_rad", 0.0) or 0.0)
            nac = float(t.get("nacelle_heading_rad", 0.0) or 0.0)
            flg = int(t.get("controller_status_flags", 0) or 0)
            p = float(t.get("gen_power_w", 0.0) or 0.0) / 1e3
            if tid == 1:
                cells.append(f"seq={seq} tgt={tgt * R2D:7.2f} nac={nac * R2D:7.2f} "
                             f"P={p:6.0f} f=0x{flg:04x}")
            else:
                cells.append(f"{nac * R2D:7.2f}")
        print(f"{i:>3} {tsim:6.1f} | {cells[0]} | {cells[1]} | {cells[2]}")

    # 稳态窗口汇总（最后 40% 步）：P1 验收口径
    n_tail = max(1, int(len(rows) * 0.4))
    tail = [st for st in rows[-n_tail:] if st.get("turbines")]
    if tail:
        t1s = [st["turbines"][0] for st in tail if len(st.get("turbines", [])) >= 1]
        if t1s:
            import numpy as _np
            p1 = _np.mean([float(t.get("gen_power_w", 0.0) or 0.0) for t in t1s]) / 1e3
            nac_last = float(t1s[-1].get("nacelle_heading_rad", 0.0) or 0.0)
            seq_last = int(t1s[-1].get("yaw_seq_applied", 0) or 0)
            tgt_last = float(t1s[-1].get("yaw_target_heading_rad", 0.0) or 0.0)
            p2l = [float((st["turbines"][1] if len(st.get("turbines", [])) > 1 else {})
                         .get("gen_power_w", 0.0) or 0.0) for st in tail]
            p3l = [float((st["turbines"][2] if len(st.get("turbines", [])) > 2 else {})
                         .get("gen_power_w", 0.0) or 0.0) for st in tail]
            print(f"==== 稳态尾窗（最后 40%，{len(tail)} 步）====")
            print(f"  T1 P_mean={p1:.0f} kW | T1 nac_last={nac_last:.4f} "
                  f"(deg~{nac_last if abs(nac_last) < 400 else nac_last / 57.2958:.2f}) "
                  f"| seq_last={seq_last} | tgt_last={tgt_last * R2D:.2f} deg")
            print(f"  T2 P_mean={_np.mean(p2l) / 1e3:.0f} kW | "
                  f"T3 P_mean={_np.mean(p3l) / 1e3:.0f} kW")

    # 结论提示
    seqs = [int(((st.get("turbines") or [{}])[0]).get("yaw_seq_applied", 0) or 0)
            for st in rows if st.get("turbines")]
    print("==== 判读 ====")
    if not seqs:
        print("  无 T1 状态行：命令/状态链均未建立")
    elif max(seqs) == 0:
        print("  yaw_seq_applied 恒 0：命令未到达 runtime 或锁存未发生（查 fcr_diag.txt）")
    elif seqs[-1] == 0:
        print("  yaw_seq_applied 末步=0：通道 TTL 过期回退（应使用长 TTL 发送）")
    else:
        print(f"  yaw_seq_applied 保持={seqs[-1]}：通道未过期；看 nac 是否停在 tgt 附近")
    print(f"输出: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
